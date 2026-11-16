import json
import threading
import urllib.error
import urllib.request
from datetime import datetime
from zoneinfo import ZoneInfo

import pytest

from app import make_server
from config import Church
from schedule import build_rows
from store import Store

WARSAW = ZoneInfo("Europe/Warsaw")
NOW = datetime(2026, 11, 12, 17, 0, tzinfo=WARSAW)  # a Thursday
FETCHED = "2026-11-12T03:00:00+01:00"


def entry(time, day="thu"):
    return {"location": None, "day": day, "time": time, "season": "all",
            "season_assumed": False, "quote": time}


def part(entries):
    return {"entries": entries, "fetched_at": FETCHED, "content_hash": "h", "error": None}


ALPHA = Church(id="alpha", name="Alpha", mode="car", eta_min=10, eta_max=15, eta_confirmed=True)
BETA = Church(id="beta", name="Beta", mode="walk", eta_min=5, eta_max=10, eta_confirmed=False)
GAMMA = Church(id="gamma", name="Gamma", mode="car", eta_min=20, eta_max=30)  # no cache entry

CACHE = {
    "heartbeat": "2026-11-12T03:00:05+01:00",
    "sources": {
        "alpha": {"regular": part([entry("18:00")]), "exceptions": part([])},
        "beta": {"regular": part([entry("17:30"), entry("19:30"), entry("20:30"), entry("22:00")]),
                 "exceptions": {**part([]), "error": "boom"}},
    },
}


class Srv:
    def __init__(self, tmp_path, cache, now=NOW, churches=(ALPHA, BETA, GAMMA)):
        self.store = Store(str(tmp_path / "cache.json"))
        if cache is not None:
            self.store.save(cache)
        self.churches = list(churches)
        self.httpd = make_server(0, self.store, self.churches, now_fn=lambda: now, host="127.0.0.1")
        self.port = self.httpd.server_address[1]
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def get(self, path):
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{self.port}{path}", timeout=5) as r:
                return r.status, r.headers, r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.headers, e.read()

    def json(self, path):
        status, headers, body = self.get(path)
        assert status == 200
        return json.loads(body), headers

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(5)


@pytest.fixture
def serve(tmp_path):
    servers = []

    def start(cache=CACHE, **kw):
        s = Srv(tmp_path, cache, **kw)
        servers.append(s)
        return s

    yield start
    for s in servers:
        s.close()


def times(body):
    return [r["start_hm"] for r in body["rows"] if r["status"] != "no_data"]


def test_upcoming_returns_rows_sorted(serve):
    s = serve()
    body, headers = s.json("/upcoming")
    assert headers["Content-Type"] == "application/json; charset=utf-8"
    assert body["window_hours"] == 3
    assert body["generated_at"] == NOW.isoformat()
    assert times(body) == ["17:30", "18:00", "19:30"]
    assert [r["church"] for r in body["rows"][:3]] == ["Beta", "Alpha", "Beta"]
    # exactly what build_rows computes (no_data row for the church without cache)
    assert body["rows"] == build_rows(NOW, s.churches, CACHE, 3)
    assert body["rows"][-1]["status"] == "no_data" and body["rows"][-1]["church"] == "Gamma"
    first = body["rows"][0]
    assert first["leave_in_min"] == 30 - 10 and first["status"] == "ok"


def test_upcoming_hours_param_clamped_1_to_12(serve):
    s = serve()
    cases = {"1": (1, ["17:30", "18:00"]),
             "0": (1, ["17:30", "18:00"]),
             "-4": (1, ["17:30", "18:00"]),
             "12": (12, ["17:30", "18:00", "19:30", "20:30", "22:00"]),
             "99": (12, ["17:30", "18:00", "19:30", "20:30", "22:00"]),
             "4": (4, ["17:30", "18:00", "19:30", "20:30"])}
    for raw, (hours, expected) in cases.items():
        body, _ = s.json(f"/upcoming?hours={raw}")
        assert body["window_hours"] == hours, raw
        assert times(body) == expected, raw
    for garbage in ("abc", "", "2.5", "1e3"):
        body, _ = s.json(f"/upcoming?hours={garbage}")
        assert body["window_hours"] == 3, garbage
        assert times(body) == ["17:30", "18:00", "19:30"], garbage


def test_upcoming_empty_window_returns_empty_rows_with_200(serve):
    late = datetime(2026, 11, 12, 23, 0, tzinfo=WARSAW)
    s = serve(now=late, churches=(ALPHA, BETA))
    status, headers, raw = s.get("/upcoming")
    assert status == 200
    assert json.loads(raw) == {"generated_at": late.isoformat(), "window_hours": 3, "rows": []}


def test_upcoming_without_cache_is_200_with_no_data_rows(serve):
    s = serve(cache=None)
    body, _ = s.json("/upcoming")
    assert [r["status"] for r in body["rows"]] == ["no_data"] * 3


def test_upcoming_reloads_cache_from_disk_per_request(serve):
    s = serve()
    assert times(s.json("/upcoming")[0]) == ["17:30", "18:00", "19:30"]
    changed = json.loads(json.dumps(CACHE))
    changed["sources"]["alpha"]["regular"]["entries"] = [entry("19:00")]
    s.store.save(changed)
    assert times(s.json("/upcoming")[0]) == ["17:30", "19:00", "19:30"]


def test_non_ascii_not_escaped(serve):
    s = serve(now=datetime(2026, 11, 12, 17, 0, tzinfo=WARSAW))
    cache = json.loads(json.dumps(CACHE))
    cache["sources"]["alpha"]["regular"]["fetched_at"] = "2026-10-23T03:00:00+02:00"  # stale
    s.store.save(cache)
    _, headers, raw = s.get("/upcoming")
    assert "rozkład sprzed".encode("utf-8") in raw
    assert b"\\u" not in raw


def test_health_reports_unconfirmed_eta(serve):
    s = serve()
    body, headers = s.json("/health")
    assert headers["Content-Type"] == "application/json; charset=utf-8"
    assert body["unconfirmed_eta"] == ["beta", "gamma"]
    assert body["heartbeat"] == CACHE["heartbeat"]
    assert body["heartbeat_age_h"] == 14.0  # 03:00:05 -> 17:00:00, rounded
    assert body["sources"]["alpha"] == {"regular_age_days": 0.58, "exceptions_age_days": 0.58,
                                        "error": None}
    assert body["sources"]["beta"]["error"] == "boom"
    assert body["sources"]["gamma"] == {"regular_age_days": None, "exceptions_age_days": None,
                                        "error": None}


def test_health_without_cache_not_500(serve):
    s = serve(cache=None)
    body, _ = s.json("/health")
    assert body["heartbeat"] is None and body["heartbeat_age_h"] is None
    assert set(body["sources"]) == {"alpha", "beta", "gamma"}
    assert body["unconfirmed_eta"] == ["beta", "gamma"]


def test_health_filia_listed_under_its_source(serve):
    filia = Church(id="filia", name="Filia", mode="walk", eta_min=1, eta_max=2,
                   eta_confirmed=True, schedule_from="alpha")
    s = serve(churches=(ALPHA, filia))
    body, _ = s.json("/health")
    assert list(body["sources"]) == ["alpha"]
    assert body["unconfirmed_eta"] == []


def test_unknown_path_404(serve):
    s = serve()
    status, headers, raw = s.get("/nope")
    assert status == 404
    assert json.loads(raw) == {"error": "not found"}
    assert s.get("/upcoming/extra")[0] == 404
