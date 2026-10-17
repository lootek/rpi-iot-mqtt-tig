#!/usr/bin/env python3
"""Provision the ZTP Infinity datasource, the Misc folder and the
MPK departures dashboard into Grafana on lothlorien.

stdlib only (urllib) so it runs from any laptop without a repo venv:

  python3 provision_grafana.py
  GRAFANA_URL=http://192.168.10.18:3000 python3 provision_grafana.py

Bearer token: first line of ~/.secrets/grafana (service account, role Admin).
Idempotent: the datasource and folder are looked up before creation and the
dashboard is pushed with overwrite:true.
"""

import json
import os
import sys
import urllib.error
import urllib.request

BASE = os.environ.get("GRAFANA_URL", "http://192.168.10.18:3000").rstrip("/")
TOKEN_PATH = os.path.expanduser(os.environ.get("GRAFANA_TOKEN_FILE", "~/.secrets/grafana"))
HERE = os.path.dirname(os.path.abspath(__file__))

DATASOURCE = {
    "name": "ZTP Infinity",
    "type": "yesoreyeram-infinity-datasource",
    "uid": "ztp-infinity",
    "access": "proxy",
    "jsonData": {"auth": False, "defaultURL": "http://127.0.0.1:8090"},
}

FOLDER_TITLE = "Misc"


def token():
    try:
        with open(TOKEN_PATH) as f:
            first = f.read().strip().splitlines()
    except OSError as exc:
        sys.exit(f"cannot read {TOKEN_PATH}: {exc}")
    if not first:
        sys.exit(f"no token found in {TOKEN_PATH}")
    return first[0]


def call(method, path, body=None):
    req = urllib.request.Request(
        BASE + path, method=method,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Authorization": f"Bearer {token()}",
                 "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return r.status, r.read().decode()
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()
    except urllib.error.URLError as e:
        return 0, str(e)


def expect(what, code, body, ok=(200, 201, 204)):
    if code not in ok:
        print(f"{what} FAILED HTTP {code} {body[:400]}", file=sys.stderr)
        sys.exit(1)


def ensure_datasource():
    code, body = call("GET", "/api/datasources/uid/ztp-infinity")
    if code == 404:
        code, body = call("POST", "/api/datasources", DATASOURCE)
        expect("create datasource", code, body)
        print(f"datasource ztp-infinity created (HTTP {code})")
    else:
        expect("lookup datasource", code, body)
        print("datasource ztp-infinity already present")


def ensure_folder():
    code, body = call("GET", "/api/folders")
    expect("list folders", code, body)
    for folder in json.loads(body):
        if folder.get("title") == FOLDER_TITLE:
            print(f"folder {FOLDER_TITLE!r} already present (uid {folder['uid']})")
            return folder["uid"]
    code, body = call("POST", "/api/folders", {"title": FOLDER_TITLE})
    expect("create folder", code, body, ok=(200, 201))
    uid = json.loads(body)["uid"]
    print(f"folder {FOLDER_TITLE!r} created (uid {uid})")
    return uid


def upsert_dashboard(folder_uid):
    with open(os.path.join(HERE, "dashboard.json")) as f:
        dashboard = json.load(f)
    code, body = call("POST", "/api/dashboards/db",
                      {"dashboard": dashboard, "folderUid": folder_uid,
                       "overwrite": True, "message": "ttss-collector provision"})
    expect("upsert dashboard", code, body)
    print(f"dashboard {dashboard['uid']!r} upserted (HTTP {code})")


def main():
    print(f"Grafana at {BASE}")
    ensure_datasource()
    folder_uid = ensure_folder()
    upsert_dashboard(folder_uid)
    print("done")


if __name__ == "__main__":
    main()
