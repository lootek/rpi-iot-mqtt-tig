#!/usr/bin/env python3
"""Push dashboards from git back into Grafana -- the recovery path.

Deliberately manual. export.py runs nightly and unattended; this does not, because
overwriting a live dashboard should always be a decision someone made on purpose.

    ./restore.py --list                  # compare git against live
    ./restore.py --all --dry-run         # what would change, touching nothing
    ./restore.py --all                   # restore everything
    ./restore.py --uid aem53k0pia29sd    # restore one
    ./restore.py --uid X --as-uid scratch --folder General   # safe test copy
"""

import argparse
import json
import os
import sys

import lib

SUBDIR = os.path.join("central-stack", "dashboards")


def load_files(dest):
    """Return {uid: (path, folder_title, dashboard_dict)}."""
    out = {}
    for root, _dirs, files in os.walk(dest):
        for name in sorted(files):
            if not name.endswith(".json") or name == "folders.json":
                continue
            path = os.path.join(root, name)
            rel_folder = os.path.relpath(root, dest)
            folder = "General" if rel_folder == "." else rel_folder
            try:
                dash = json.load(open(path))
            except ValueError as e:
                print(f"  skipping unparseable {path}: {e}", file=sys.stderr)
                continue
            if "uid" in dash:
                out[dash["uid"]] = (path, folder, dash)
    return out


def folder_uid(api, folders, title, create=True):
    """Resolve a folder title to its uid, creating the folder if needed."""
    if title in ("General", "", None):
        return None
    if title in folders:
        return folders[title]
    live = api.get("/api/folders")
    for f in live:
        if f["title"] == title:
            folders[title] = f["uid"]
            return f["uid"]
    if not create:
        return None
    code, body = api("POST", "/api/folders", {"title": title})
    if not 200 <= code < 300:
        sys.exit(f"could not create folder {title!r}: HTTP {code} {str(body)[:200]}")
    folders[title] = body["uid"]
    print(f"  created folder {title!r} -> {body['uid']}")
    return body["uid"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=os.path.realpath(os.path.join(lib.HERE, "..", "..")))
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--uid", action="append", default=[])
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--as-uid", help="restore under a different uid (for safe testing)")
    ap.add_argument("--folder", help="override target folder title")
    args = ap.parse_args()

    dest = os.path.join(os.path.realpath(args.repo), SUBDIR)
    files = load_files(dest)
    if not files:
        sys.exit(f"no dashboard files found under {dest}")

    api = lib.Api(*lib.load_env())
    folders = {}
    fpath = os.path.join(dest, "folders.json")
    if os.path.exists(fpath):
        folders = json.load(open(fpath))

    live = {d["uid"]: d for d in api.get("/api/search?type=dash-db&limit=500")}

    if args.list or not (args.all or args.uid):
        print(f"{'uid':<40} {'folder':<18} {'title':<24} status")
        for uid, (path, folder, dash) in sorted(files.items(), key=lambda kv: kv[1][1]):
            if uid not in live:
                status = "MISSING in Grafana"
            else:
                cur = api.get(f"/api/dashboards/uid/{uid}")["dashboard"]
                same = lib.dump_json(lib.normalise(cur)) == open(path).read()
                status = "in sync" if same else "DIFFERS from git"
            print(f"{uid:<40} {folder:<18} {dash.get('title','')[:24]:<24} {status}")
        only_live = set(live) - set(files)
        for uid in sorted(only_live):
            print(f"{uid:<40} {'-':<18} {live[uid]['title'][:24]:<24} not in git (run export.py)")
        if not (args.all or args.uid):
            print("\n(nothing restored; pass --all or --uid)")
        return 0

    targets = list(files) if args.all else args.uid
    if args.as_uid and len(targets) != 1:
        sys.exit("--as-uid only makes sense with exactly one --uid")

    fail = 0
    for uid in targets:
        if uid not in files:
            print(f"  {uid}: not found in git", file=sys.stderr)
            fail = 1
            continue
        path, folder, dash = files[uid]
        folder = args.folder or folder
        dash = dict(dash)
        # id must be null so Grafana matches on uid instead of a stale primary
        # key; keeping uid is what makes the restore land on top of the existing
        # dashboard, preserving alert annotations and saved links.
        dash["id"] = None
        if args.as_uid:
            dash["uid"] = args.as_uid
            dash["title"] = dash.get("title", "") + " (restore test)"
            dash.pop("version", None)

        if args.dry_run:
            target_uid = dash["uid"]
            if target_uid in live:
                cur = api.get(f"/api/dashboards/uid/{target_uid}")["dashboard"]
                same = lib.dump_json(lib.normalise(cur)) == open(path).read()
                print(f"  {target_uid:<40} {'no change' if same else 'WOULD OVERWRITE'}  ({folder})")
            else:
                print(f"  {target_uid:<40} WOULD CREATE  ({folder})")
            continue

        payload = {"dashboard": dash, "overwrite": True,
                   "message": "restored from git"}
        fuid = folder_uid(api, folders, folder)
        if fuid:
            payload["folderUid"] = fuid
        code, body = api("POST", "/api/dashboards/db", payload)
        if 200 <= code < 300:
            print(f"  restored {body.get('uid')}  v{body.get('version')}  {folder}/{dash.get('title')}")
        else:
            print(f"  FAILED {uid}: HTTP {code} {str(body)[:300]}", file=sys.stderr)
            fail = 1

    return fail


if __name__ == "__main__":
    sys.exit(main())
