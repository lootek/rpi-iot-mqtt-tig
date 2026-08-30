#!/usr/bin/env python3
"""Dump every Grafana dashboard to disk, optionally committing the result.

Grafana is the source of truth here -- dashboards are edited in the UI, and this
pulls them into git so they are versioned and recoverable. The reverse direction
is restore.py, which is a deliberate manual act.

Interactive:
    ./export.py                    # write files, report what changed
    ./export.py --repo /path/to/checkout

Unattended (cron on lothlorien, in the dedicated sync clone -- never the live tree):
    ./export.py --commit --pull --repo /home/pi/dashboard-sync/repo
"""

import argparse
import json
import os
import subprocess
import sys

import lib

SUBDIR = os.path.join("central-stack", "dashboards")


def git(repo, *args, check=True, raw=False):
    """Run git. raw=True preserves output verbatim.

    porcelain status records begin with a status column that is a SPACE for
    unstaged changes (" M path"), so stripping would shift every subsequent
    field by one character.
    """
    r = subprocess.run(["git", "-C", repo, *args], capture_output=True, text=True)
    if check and r.returncode != 0:
        sys.exit(f"git {' '.join(args)} failed:\n{r.stderr.strip()}")
    return r.stdout if raw else r.stdout.strip()


def export_all(api, dest):
    """Write every dashboard under dest/<folder>/<uid>-<slug>.json."""
    search = api.get("/api/search?type=dash-db&limit=500")
    folders, written = {}, set()

    for entry in search:
        full = api.get(f"/api/dashboards/uid/{entry['uid']}")
        dash, meta = full["dashboard"], full["meta"]
        folder = meta.get("folderTitle") or "General"
        if meta.get("folderUid"):
            folders[folder] = meta["folderUid"]

        folder_dir = os.path.join(dest, folder)
        os.makedirs(folder_dir, exist_ok=True)
        path = os.path.join(folder_dir, lib.dashboard_filename(dash["uid"], dash["title"]))
        with open(path, "w") as f:
            f.write(lib.dump_json(lib.normalise(dash)))
        written.add(os.path.relpath(path, dest))

    with open(os.path.join(dest, "folders.json"), "w") as f:
        f.write(lib.dump_json(folders))
    written.add("folders.json")

    # Prune files for dashboards deleted in Grafana, so git reflects reality
    # rather than accumulating tombstones. Only .json under folder dirs is
    # considered; the scripts themselves are obviously left alone.
    pruned = []
    for root, _dirs, files in os.walk(dest):
        for name in files:
            if not name.endswith(".json"):
                continue
            rel = os.path.relpath(os.path.join(root, name), dest)
            if rel.startswith("."):
                continue
            if rel not in written:
                os.remove(os.path.join(root, name))
                pruned.append(rel)

    return len(search), sorted(written), pruned


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=os.path.realpath(os.path.join(lib.HERE, "..", "..")),
                    help="git checkout to write into")
    ap.add_argument("--commit", action="store_true", help="commit and push if anything changed")
    ap.add_argument("--pull", action="store_true", help="git pull --rebase first (cron mode)")
    args = ap.parse_args()

    repo = os.path.realpath(args.repo)
    dest = os.path.join(repo, SUBDIR)
    # Ask git rather than looking for a .git directory: in a submodule or worktree
    # .git is a file, not a directory.
    if subprocess.run(["git", "-C", repo, "rev-parse", "--git-dir"],
                      capture_output=True).returncode != 0:
        sys.exit(f"{repo} is not a git checkout")

    # Only guard when we are about to touch git. A read-only export is harmless
    # anywhere, but pull/commit/push in the live deployment is the hazard.
    if args.commit or args.pull:
        lib.assert_not_live_tree(repo)

    if args.pull:
        git(repo, "pull", "--rebase", "--quiet")

    os.makedirs(dest, exist_ok=True)
    api = lib.Api(*lib.load_env())
    total, written, pruned = export_all(api, dest)
    print(f"exported {total} dashboards -> {os.path.relpath(dest, repo)}")
    for p in pruned:
        print(f"  pruned (deleted in Grafana): {p}")

    # -z (NUL-separated, no C-quoting) because folder names contain spaces --
    # "Host monitoring" -- and plain --porcelain wraps those paths in quotes.
    status = git(repo, "status", "--porcelain", "-z", "--", SUBDIR, raw=True)
    if not status:
        print("no changes")
        return 0

    changed = [rec[3:] for rec in status.split("\0") if len(rec) > 3]
    print(f"{len(changed)} file(s) changed:")
    for c in changed[:20]:
        print(f"  {c}")

    if not args.commit:
        print("\n(not committing; pass --commit)")
        return 0

    git(repo, "add", "--", SUBDIR)

    # Pre-flight the staged diff. This push is unattended and the repo is
    # public, so a credential that slipped into a panel must stop the commit
    # rather than be discovered later in history.
    staged = git(repo, "diff", "--cached", "--", SUBDIR)
    hits = lib.scan_secrets(staged)
    if hits:
        git(repo, "reset", "--quiet", "HEAD", "--", SUBDIR, check=False)
        print("\nABORTED: possible secrets in the dashboard diff:", file=sys.stderr)
        for name, excerpt in hits[:10]:
            print(f"  {name}: {excerpt}", file=sys.stderr)
        print("Staged changes were unstaged. Inspect before committing.", file=sys.stderr)
        return 1

    git(repo, "commit", "--quiet", "-m", f"Dashboard snapshot: {len(changed)} changed")
    git(repo, "push", "--quiet", "origin", "HEAD")
    print(f"committed and pushed ({git(repo, 'rev-parse', '--short', 'HEAD')})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
