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
    assert p["options"].get("sortBy") in (None, [])  # keep API order
    assert p["options"].get("enablePagination") is False
    (t,) = p["targets"]
    assert t["datasource"]["uid"] == "msze-infinity"
    assert t["url"] == "http://127.0.0.1:8091/upcoming?hours=${hours}"
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
    assert '"name": "Msze św. Infinity"' in text
    assert 'existing.get("name") == DATASOURCE["name"]' in text  # rename triggers PUT


def test_panel_title_has_sw():
    assert _panel()["title"] == "Najbliższe Msze św. (${hours} h)"


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


def test_dashboard_has_hours_variable_default_6():
    d = _dashboard()
    (v,) = [v for v in d["templating"]["list"] if v["name"] == "hours"]
    assert v["type"] == "custom"
    assert v["label"] == "Okno (h)"
    assert v["query"] == "3,6,9,12,24"
    assert [o["value"] for o in v["options"]] == ["3", "6", "9", "12", "24"]
    assert v["current"]["value"] == "6"
    assert [o["value"] for o in v["options"] if o["selected"]] == ["6"]
    assert not v.get("multi") and not v.get("includeAll")
    p = _panel()
    (t,) = p["targets"]
    assert t["url"].startswith("http://127.0.0.1:8091/")  # allow-list prefix
    assert "hours=${hours}" in t["url"]
    declared = {c["selector"] for c in t["columns"]}
    referenced = {o["matcher"]["options"] for o in p["fieldConfig"]["overrides"]}
    assert referenced <= declared


def test_no_noValue_on_table_defaults():
    # noValue is applied to every empty cell (e.g. blank badges), not only to empty tables
    p = _panel()
    assert "noValue" not in p["fieldConfig"]["defaults"]
    for o in p["fieldConfig"]["overrides"]:
        assert all(x["id"] != "noValue" for x in o["properties"])


def test_status_column_excluded_by_transformation():
    p = _panel()
    organize = [t for t in p["transformations"] if t["id"] == "organize"]
    assert len(organize) == 1
    assert organize[0]["options"]["excludeByName"]["status"] is True
    # custom.hideFrom.viz does not hide table columns in Grafana 11.5
    assert "custom.hideFrom" not in json.dumps(p["fieldConfig"])
    # nothing may depend on the (hidden) status column being displayed
    assert "status" not in {o["matcher"]["options"] for o in p["fieldConfig"]["overrides"]}


def test_late_rendering_driven_by_leave_in_min_mapping():
    raw = open(os.path.join(HERE, "dashboard.json"), encoding="utf-8").read()
    assert "nie zdążysz" in raw and "spóźniona" not in raw
    (o,) = [o for o in _panel()["fieldConfig"]["overrides"]
            if o["matcher"]["options"] == "leave_in_min"]
    (m,) = [x["value"] for x in o["properties"] if x["id"] == "mappings"]
    texts = {x["type"]: x["options"] for x in m}
    assert texts["range"]["result"]["text"] == "nie zdążysz"
    assert texts["range"]["result"]["color"] == "gray"
    assert texts["special"]["match"] == "null"  # empty-window row has leave_in_min null


def test_mode_mapping_leaves_empty_mode_blank():
    (o,) = [o for o in _panel()["fieldConfig"]["overrides"]
            if o["matcher"]["options"] == "mode"]
    (m,) = [x["value"] for x in o["properties"] if x["id"] == "mappings"]
    assert set(m[0]["options"]) == {"car", "walk"}  # "" matches nothing -> renders blank


def test_timepicker_hidden():
    d = _dashboard()
    assert d["timepicker"]["hidden"] is True
    assert d["refresh"] == "1m"
