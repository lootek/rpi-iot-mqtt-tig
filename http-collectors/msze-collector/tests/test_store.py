import json
import os
from datetime import date

from store import Store, merge_exceptions

EMPTY = {"sources": {}, "heartbeat": None}
TODAY = date(2026, 11, 12)


def exc(d, time="18:00", action="cancel", location=None, note=""):
    return {"date": d, "time": time, "action": action, "new_time": None,
            "scope": "one", "location": location, "note": note, "source_url": ""}


def test_load_missing_returns_empty_shape(tmp_path):
    assert Store(str(tmp_path / "cache.json")).load() == EMPTY


def test_load_corrupt_returns_empty_shape(tmp_path):
    p = tmp_path / "cache.json"
    p.write_text("{not json")
    assert Store(str(p)).load() == EMPTY
    p.write_text("[1, 2]")
    assert Store(str(p)).load() == EMPTY


def test_save_is_atomic(tmp_path, monkeypatch):
    p = tmp_path / "cache.json"
    s = Store(str(p))
    s.save({"sources": {"a": {}}, "heartbeat": "t1"})
    seen = []
    real = os.replace

    def spy(src, dst):
        seen.append((os.path.dirname(os.path.abspath(src)), os.path.exists(dst)))
        # target still holds the complete previous version while tmp is written
        assert json.loads(p.read_text())["heartbeat"] == "t1"
        real(src, dst)

    monkeypatch.setattr(os, "replace", spy)
    s.save({"sources": {"b": {}}, "heartbeat": "t2"})
    assert seen == [(str(tmp_path), True)]  # tmp lives in the same dir
    assert json.loads(p.read_text())["heartbeat"] == "t2"
    assert os.listdir(tmp_path) == ["cache.json"]  # no leftover tmp


def test_merge_keeps_old_future_exceptions_not_in_new():
    old = [exc("2026-11-20")]
    new = [exc("2026-11-14")]
    out = merge_exceptions(old, new, TODAY)
    assert {x["date"] for x in out} == {"2026-11-14", "2026-11-20"}


def test_merge_new_overrides_same_key():
    old = [exc("2026-11-14", note="old")]
    new = [exc("2026-11-14", note="new")]
    out = merge_exceptions(old, new, TODAY)
    assert [x["note"] for x in out] == ["new"]


def test_merge_drops_past_dates():
    out = merge_exceptions([exc("2026-11-11")], [exc("2026-11-12", time="07:00")], TODAY)
    assert [x["date"] for x in out] == ["2026-11-12"]
