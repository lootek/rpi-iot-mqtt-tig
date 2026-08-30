"""Shared Grafana API plumbing for the dashboard export/restore scripts.

Kept separate from central-stack/alerting/ because the two directions are
deliberately opposite: for alert rules git is the source of truth and apply.py
pushes it, whereas for dashboards **Grafana is the source of truth** and
export.py pulls it. Editing happens in the UI; git is the backup and the history.
"""

import json
import os
import re
import sys
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))

# Known live deployments. The sync job must never run git pull/commit/push here:
# in this tree a rebase would drag unreviewed upstream changes into the running
# IoT stack and collide with its untracked working files.
LIVE_DEPLOYMENT_PATHS = ("/home/pi/rpi-iot-mqtt-tig",)

# Directories that only exist in a full deployment checkout. The sync clone is
# sparse (central-stack/dashboards only), so their presence means "this is not
# the dedicated sync clone" -- a warning rather than a hard error, because the
# initial export is legitimately run from a full clone on a workstation.
DEPLOYMENT_DIRS = ("host-monitoring", "volumes", "iot")


def load_env():
    """Resolve Grafana URL + token.

    Order: $GRAFANA_ENV_FILE (how cron supplies it, from outside any repo tree),
    then this directory's .grafana.env, then the alerting directory's — so an
    interactive run from the Mac reuses the token already configured there.
    """
    candidates = [
        os.environ.get("GRAFANA_ENV_FILE"),
        os.path.join(HERE, ".grafana.env"),
        os.path.join(HERE, "..", "alerting", ".grafana.env"),
    ]
    env = {}
    for path in candidates:
        if path and os.path.exists(path):
            for line in open(path):
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    env.setdefault(k, v)
            break
    url = os.environ.get("GRAFANA_URL", env.get("GRAFANA_URL", "http://192.168.10.18:3000"))
    tok = os.environ.get("GRAFANA_TOKEN", env.get("GRAFANA_TOKEN"))
    if not tok:
        sys.exit("need GRAFANA_TOKEN (set GRAFANA_ENV_FILE, or create .grafana.env)")
    return url.rstrip("/"), tok


class Api:
    def __init__(self, url, token):
        self.url, self.token = url, token

    def __call__(self, method, path, body=None):
        req = urllib.request.Request(
            self.url + path, method=method,
            data=json.dumps(body).encode() if body is not None else None,
            headers={"Authorization": f"Bearer {self.token}",
                     "Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                raw = r.read().decode()
                return r.status, (json.loads(raw) if raw else None)
        except urllib.error.HTTPError as e:
            raw = e.read().decode()
            try:
                return e.code, json.loads(raw)
            except ValueError:
                return e.code, raw
        except urllib.error.URLError as e:
            return 0, str(e)

    def get(self, path):
        code, body = self("GET", path)
        if not 200 <= code < 300:
            sys.exit(f"GET {path} failed: HTTP {code} {str(body)[:200]}")
        return body


def slug(title):
    s = re.sub(r"[^a-z0-9]+", "-", title.lower()).strip("-")
    return s or "untitled"


def dashboard_filename(uid, title):
    return f"{uid}-{slug(title)}.json"


def normalise(dash):
    """Strip fields that churn the diff without carrying meaning.

    `id` is a per-instance autoincrement primary key -- it differs between any
    two Grafana installs and would make a restored dashboard produce a spurious
    diff on the next export. `uid` and `version` are deliberately KEPT: uid is
    what makes a restore land on top of the existing dashboard (preserving alert
    annotations and saved links), and version increments on every UI save, which
    is precisely the signal that says "someone changed this".
    """
    d = dict(dash)
    d.pop("id", None)
    return d


def dump_json(obj):
    """Byte-stable serialisation, so an unedited dashboard yields no diff.

    Without sort_keys the API's map ordering can vary between calls and cron
    would commit meaningless churn every night.
    """
    return json.dumps(obj, indent=2, sort_keys=True, ensure_ascii=False) + "\n"


# Patterns checked before any unattended commit. The repo is public and the cron
# pushes without a human in the loop; dashboards carry only datasource uids
# today, but nothing stops a future panel embedding a credential in a URL.
SECRET_PATTERNS = [
    (r"glsa_[A-Za-z0-9]{16,}", "Grafana service-account token"),
    (r"ghp_[A-Za-z0-9]{20,}", "GitHub personal access token"),
    (r"gho_[A-Za-z0-9]{20,}", "GitHub OAuth token"),
    (r"AKIA[0-9A-Z]{16}", "AWS access key id"),
    (r"-----BEGIN [A-Z ]*PRIVATE KEY-----", "private key"),
    # The optional quote after the key name matters: dashboards are JSON, so a
    # leaked credential appears as "password": "value", not password=value.
    (r"(?i)\b(password|passwd|secret|api[_-]?key|auth[_-]?token|bearer)[\"']?\s*[:=]\s*[\"']?[^\s\"'<>{}$,]{8,}",
     "credential assignment"),
    (r"(?i)://[^/\s:@]+:[^/\s@]+@", "credentials embedded in a URL"),
]


def scan_secrets(text):
    """Return [(pattern_name, matched_excerpt)] for anything that looks secret."""
    hits = []
    for pattern, name in SECRET_PATTERNS:
        for m in re.finditer(pattern, text):
            excerpt = m.group(0)
            if len(excerpt) > 60:
                excerpt = excerpt[:57] + "..."
            hits.append((name, excerpt))
    return hits


def assert_not_live_tree(repo):
    """Hard guard: never let the sync job run git operations on the live stack.

    Exact-path denial rather than heuristics, so it cannot misfire on a
    workstation clone while still absolutely covering the stated hazard.
    """
    real = os.path.realpath(repo)
    for live in LIVE_DEPLOYMENT_PATHS:
        if real == os.path.realpath(live):
            sys.exit(
                f"refusing to run git operations in {real}\n\n"
                "That is the live rpi-iot-mqtt-tig deployment -- the compose files,\n"
                "telegraf config and volumes actually running the IoT stack. This job\n"
                "does git pull --rebase / commit / push and must only ever touch the\n"
                "dedicated sync clone, e.g. /home/pi/dashboard-sync/repo."
            )
    present = [d for d in DEPLOYMENT_DIRS if os.path.isdir(os.path.join(real, d))]
    if present:
        print(f"  note: {real} is a full checkout (has {', '.join(present)}/), "
              "not the sparse sync clone", file=sys.stderr)
