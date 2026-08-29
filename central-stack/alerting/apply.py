#!/usr/bin/env python3
"""Push the alert catalog to Grafana via the provisioning API.

Deliberately NOT file provisioning: rules dropped into
volumes/grafana/conf/provisioning/alerting/ are locked read-only in the UI
("You cannot edit provisioned resources from files in Grafana"). Pushing through
the API with X-Disable-Provenance: true leaves them editable, at the cost that
rules do NOT reload on container restart. Re-run this to re-assert git as truth.

Two-phase apply, because the rule-group endpoint can only *update* rules that
already exist -- handing it a new rule with a fixed uid fails with
"failed to calculate diff ... could not find alert rule". So:
  phase 1  upsert each rule individually (PUT, falling back to POST on 404)
  phase 2  PUT each rule group, which is the only way to set its eval interval

Usage:
  ./apply.py                 push rules, mute timings, notification policy
  ./apply.py --prune-legacy  also delete the four superseded hand-made rules
  ./apply.py --dry-run       generate and validate, push nothing
"""

import argparse
import json
import os
import sys
import urllib.error
import urllib.request

import catalog

# The four hand-made UI rules this catalog replaces. Each had static annotations,
# so a firing alert could not say which host or room it was about.
LEGACY = {
    "df4amgvdypou8a": "RootFS used         -> host-rootfs-high / -critical",
    "bf4eevckxp3pca": "CO2 (paused)        -> iot-co2-high / iot-co2-implausible-low",
    "df9genywo42yob": "PM10 air pollution  -> iot-pm10-high",
    "ef9gerfyt3qwwc": "PM2.5 air pollution -> iot-pm25-high",
}


def load_env():
    env = {}
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".grafana.env")
    if os.path.exists(path):
        for line in open(path):
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                env[k] = v
    url = os.environ.get("GRAFANA_URL", env.get("GRAFANA_URL", "http://lothlorien:3000"))
    tok = os.environ.get("GRAFANA_TOKEN", env.get("GRAFANA_TOKEN"))
    if not tok:
        sys.exit("need GRAFANA_TOKEN (see .grafana.env.example)")
    return url.rstrip("/"), tok


class Api:
    def __init__(self, url, token):
        self.url, self.token = url, token

    def __call__(self, method, path, body=None):
        req = urllib.request.Request(
            self.url + path, method=method,
            data=json.dumps(body).encode() if body is not None else None,
            headers={"Authorization": f"Bearer {self.token}",
                     "Content-Type": "application/json",
                     "X-Disable-Provenance": "true"})
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                return r.status, r.read().decode()
        except urllib.error.HTTPError as e:
            return e.code, e.read().decode()
        except urllib.error.URLError as e:
            return 0, str(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--prune-legacy", action="store_true")
    args = ap.parse_args()

    groups, policy, mutes = catalog.GROUPS, catalog.POLICY, catalog.MUTE_TIMINGS
    rules = [r for g in groups for r in g["rules"]]

    uids = [r["uid"] for r in rules]
    dupes = {u for u in uids if uids.count(u) > 1}
    if dupes:
        sys.exit(f"duplicate rule uids: {sorted(dupes)}")
    print(f"catalog: {len(rules)} rules across {len(groups)} groups")

    if args.dry_run:
        for g in groups:
            print(f"  {g['title']:<18} {len(g['rules']):>2} rules  interval={g['interval']}s")
        print("dry run - nothing pushed")
        return 0

    url, token = load_env()
    api = Api(url, token)
    fail = 0

    # phase 0 -- drop superseded rules FIRST. Grafana enforces unique rule titles
    # within a folder, so "PM2.5 air pollution" cannot be recreated while the old
    # hand-made rule of the same name still exists.
    if args.prune_legacy:
        for uid, what in LEGACY.items():
            code, body = api("DELETE", f"/api/v1/provisioning/alert-rules/{uid}")
            if 200 <= code < 300:
                print(f"  pruned {uid}  {what}")
            elif code == 404:
                print(f"  already gone {uid}")
            else:
                print(f"  prune {uid} FAILED HTTP {code} {body[:200]}", file=sys.stderr)
                fail = 1
    else:
        print("  (legacy rules kept; re-run with --prune-legacy to remove)")

    # phase 1 -- upsert rules
    created = updated = 0
    for r in rules:
        code, body = api("PUT", f"/api/v1/provisioning/alert-rules/{r['uid']}", r)
        if code == 404:
            code, body = api("POST", "/api/v1/provisioning/alert-rules", r)
            if 200 <= code < 300:
                created += 1
        elif 200 <= code < 300:
            updated += 1
        if not 200 <= code < 300:
            print(f"  RULE FAILED {r['uid']:<28} HTTP {code} {body[:200]}", file=sys.stderr)
            fail = 1
    print(f"  rules: {created} created, {updated} updated")

    # phase 2 -- set group eval intervals (only possible via the group endpoint)
    for g in groups:
        code, body = api("PUT",
                         f"/api/v1/provisioning/folder/{g['folderUid']}/rule-groups/{g['title']}",
                         g)
        if 200 <= code < 300:
            print(f"  group {g['title']:<18} interval={g['interval']:>4}s  HTTP {code}")
        else:
            print(f"  group {g['title']:<18} FAILED HTTP {code} {body[:200]}", file=sys.stderr)
            fail = 1

    # mute timings
    for m in mutes:
        code, body = api("PUT", f"/api/v1/provisioning/mute-timings/{m['name']}", m)
        if code == 404:
            code, body = api("POST", "/api/v1/provisioning/mute-timings", m)
        if 200 <= code < 300:
            print(f"  mute-timing {m['name']:<12} HTTP {code}")
        else:
            print(f"  mute-timing {m['name']:<12} FAILED HTTP {code} {body[:200]}", file=sys.stderr)
            fail = 1

    # notification policy -- group_by [alertname, severity] is the load-bearing
    # part: it turns an MQTT bridge outage into one notification listing every
    # affected room instead of fourteen separate emails.
    code, body = api("PUT", "/api/v1/provisioning/policies", policy)
    if 200 <= code < 300:
        print(f"  notification-policy        HTTP {code}")
    else:
        print(f"  notification-policy        FAILED HTTP {code} {body[:200]}", file=sys.stderr)
        fail = 1

    return fail


if __name__ == "__main__":
    sys.exit(main())
