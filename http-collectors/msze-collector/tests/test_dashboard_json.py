import json
import os

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _dashboard():
    with open(os.path.join(HERE, "dashboard.json")) as f:
        return json.load(f)


def _panel():
    panels = _dashboard()["panels"]
    assert len(panels) == 1
    return panels[0]


def test_dashboard_json_shape():
    d = _dashboard()
    assert d["uid"] == "msze"
    assert d["title"] == "Msze św."
    assert d["refresh"] == "1m"
    p = _panel()
    assert p["type"] == "table"
    assert p["gridPos"]["w"] == 24
    assert p["datasource"]["uid"] == "msze-infinity"
    assert p["fieldConfig"]["defaults"]["noValue"] == "Brak Mszy w najbliższych 3 h"
    assert p["options"].get("sortBy") in (None, [])  # keep API order
    assert p["options"].get("enablePagination") is False
    (t,) = p["targets"]
    assert t["datasource"]["uid"] == "msze-infinity"
    assert t["url"] == "http://127.0.0.1:8091/upcoming"
    assert t["root_selector"] == "rows"
    assert t["format"] == "table"
    assert [c["selector"] for c in t["columns"]] == [
        "start_hm", "church", "mode", "eta", "leave_in_min", "badges", "status"]


def test_provision_script_asserts_allowed_hosts():
    with open(os.path.join(HERE, "provision_grafana.py")) as f:
        text = f.read()
    assert '"uid": "msze-infinity"' in text
    assert '"allowedHosts": ["http://127.0.0.1:8091"]' in text
    assert "def verify_allowed_hosts" in text
    assert text.count("verify_allowed_hosts()") >= 2  # defined + called after write
    assert 'allowed != DATASOURCE["jsonData"]["allowedHosts"]' in text
    assert "sys.exit(1)" in text
    assert "overwrite" in text


def test_all_target_columns_declared():
    p = _panel()
    declared = {c["text"] for c in p["targets"][0]["columns"]}
    assert declared == {c["selector"] for c in p["targets"][0]["columns"]}
    referenced = set()
    for o in p["fieldConfig"]["overrides"]:
        assert o["matcher"]["id"] == "byName"
        referenced.add(o["matcher"]["options"])
    for t in p["targets"]:
        referenced |= {f["field"] for f in t.get("filters", [])}
    assert referenced <= declared, referenced - declared
