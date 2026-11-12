import json
import re
from datetime import datetime, timedelta, timezone
from zoneinfo import ZoneInfo

import pytest

import job
from config import Church
from extract import EXCEPTIONS_SCHEMA, REGULAR_SCHEMA
from fetch import Doc, FetchError
from job import is_schedule_day, next_run_after, run_nightly, serve_scheduler
from store import Store

WARSAW = ZoneInfo("Europe/Warsaw")
THU = datetime(2026, 11, 12, 3, 0, tzinfo=WARSAW)
FRI = datetime(2026, 11, 13, 3, 0, tzinfo=WARSAW)

MORAWICA_URL = "http://morawica.example/"
DOM_SCHEDULE = "https://dom.example/msze/"
DOM_LISTING = "https://dom.example/kategoria/ogloszenia/"
DOM_HEADERS = {"User-Agent": "Mozilla/5.0 test"}

REGULAR_RAW = {
    "location": None, "day": "sun_holiday", "time": "09:00", "season": "all",
    "season_from": None, "season_to": None, "season_assumed": False, "quote": "9.00",
}


def exc_raw(date, time="09:00", action="cancel", note="n"):
    return {"date": date, "time": time, "action": action, "new_time": None,
            "scope": "one", "location": None, "note": note}


def church(cid, **kw):
    base = dict(id=cid, name=cid.title(), mode="car", eta_min=10, eta_max=15)
    base.update(kw)
    return Church(**base)


MORAWICA = church("morawica", location="Morawica", schedule_urls=(MORAWICA_URL,),
                  ogloszenia_urls=(MORAWICA_URL,))
BRZOSKWINIA = church("brzoskwinia", mode="walk", location="Brzoskwinia",
                     schedule_from="morawica")
DOMINIKANIE = church("dominikanie", schedule_urls=(DOM_SCHEDULE,),
                     ogloszenia_urls=(DOM_LISTING,), headers=DOM_HEADERS)
ALL = [MORAWICA, BRZOSKWINIA, DOMINIKANIE]


def html_doc(url, body="tekst", links=(), sha=None):
    html = "<html><body>" + body + "".join(f'<a href="{h}">link</a>' for h in links) + "</body></html>"
    return Doc(url, "html", body, None, sha or f"sha-{url}-{body}", html=html)


def pdf_doc(url, text="Ogłoszenia PDF"):
    return Doc(url, "pdf", text, None, f"sha-{url}-{text}")


class FakeFetcher:
    def __init__(self, docs):
        self.docs = dict(docs)
        self.calls = []

    def __call__(self, url, headers=None, **kw):
        self.calls.append((url, headers))
        d = self.docs.get(url)
        if d is None:
            raise FetchError(f"{url}: HTTP 404")
        if isinstance(d, Exception):
            raise d
        return d

    @property
    def urls(self):
        return [u for u, _ in self.calls]


class FakeLLM:
    """kind -> entries; exceptions can be scripted per document url."""

    def __init__(self, regular=None, exceptions=None, default_exc=()):
        self.regular = [REGULAR_RAW] if regular is None else regular
        self.exceptions = exceptions or {}
        self.default_exc = list(default_exc)
        self.calls = []

    def complete_json(self, system, parts, schema):
        text = " ".join(p.get("text", "") for p in parts if p.get("type") == "text")
        urls = re.findall(r"url=(\S+?)[ >]", text)
        if schema is REGULAR_SCHEMA:
            self.calls.append(("regular", urls))
            return {"entries": self.regular}
        assert schema is EXCEPTIONS_SCHEMA
        self.calls.append(("exceptions", urls))
        return {"entries": self.exceptions.get(urls[0], self.default_exc)}

    def kinds(self, kind):
        return [c for c in self.calls if c[0] == kind]


def good_fetcher(**extra):
    docs = {
        MORAWICA_URL: html_doc(MORAWICA_URL, "Msze: 9.00"),
        DOM_SCHEDULE: html_doc(DOM_SCHEDULE, "Dominikanie msze"),
        DOM_LISTING: html_doc(DOM_LISTING, "lista"),
    }
    docs.update(extra)
    return FakeFetcher(docs)


def part(entries=(), fetched_at="2026-11-10T03:00:00+01:00", content_hash="old", error=None):
    return {"entries": list(entries), "fetched_at": fetched_at,
            "content_hash": content_hash, "error": error}


def seeded_store(tmp_path, sources=("morawica", "dominikanie"), regular_fetched=True):
    store = Store(str(tmp_path / "cache.json"))
    cache = {"sources": {}, "heartbeat": None}
    for s in sources:
        cache["sources"][s] = {
            "regular": part([{**REGULAR_RAW, "season_from": None}] if regular_fetched else [],
                            fetched_at="2026-11-06T03:00:00+01:00" if regular_fetched else None),
            "exceptions": part(),
        }
        for en in cache["sources"][s]["regular"]["entries"]:
            en.pop("season_from"); en.pop("season_to")
    store.save(cache)
    return store


# ------------------------------------------------------------------ is_schedule_day

def test_is_schedule_day_friday_only():
    assert is_schedule_day(FRI)
    assert not is_schedule_day(THU)
    # 2026-11-13 23:30 UTC is already Saturday in Warsaw
    assert not is_schedule_day(datetime(2026, 11, 13, 23, 30, tzinfo=timezone.utc))
    # 2026-11-12 23:30 UTC is Friday 00:30 in Warsaw
    assert is_schedule_day(datetime(2026, 11, 12, 23, 30, tzinfo=timezone.utc))


# ------------------------------------------------------------------ run_nightly

def test_nightly_extracts_exceptions_every_day_but_schedule_only_friday(tmp_path):
    store = seeded_store(tmp_path)
    llm = FakeLLM(default_exc=[exc_raw("2026-11-15")])
    summary = run_nightly(THU, store, ALL, llm, fetcher=good_fetcher())
    assert summary["morawica"] == {"regular": "not_due", "exceptions": "ok"}
    assert summary["dominikanie"] == {"regular": "not_due", "exceptions": "ok"}
    assert llm.kinds("regular") == []
    cache = store.load()
    assert cache["sources"]["morawica"]["exceptions"]["entries"][0]["date"] == "2026-11-15"
    assert cache["sources"]["morawica"]["exceptions"]["fetched_at"] == THU.isoformat()
    assert cache["sources"]["morawica"]["exceptions"]["error"] is None

    llm2 = FakeLLM(default_exc=[])
    summary = run_nightly(FRI, store, ALL, llm2, fetcher=good_fetcher())
    assert summary["morawica"]["regular"] == "ok"
    assert len(llm2.kinds("regular")) == 2
    reg = store.load()["sources"]["morawica"]["regular"]
    assert reg["entries"][0]["time"] == "09:00"
    assert reg["fetched_at"] == FRI.isoformat() and reg["content_hash"]


def test_first_run_bootstraps_regular_schedule_on_non_friday(tmp_path):
    store = Store(str(tmp_path / "cache.json"))
    llm = FakeLLM()
    summary = run_nightly(THU, store, ALL, llm, fetcher=good_fetcher())
    assert summary["morawica"]["regular"] == "ok"
    assert summary["dominikanie"]["regular"] == "ok"
    # next Thursday-like run: schedule exists now, so not due
    summary = run_nightly(THU + timedelta(days=7), store, ALL, llm, fetcher=good_fetcher())
    assert summary["morawica"]["regular"] == "not_due"


def test_force_schedule_runs_on_non_friday(tmp_path):
    store = seeded_store(tmp_path)
    llm = FakeLLM()
    summary = run_nightly(THU, store, ALL, llm, fetcher=good_fetcher(), force_schedule=True)
    assert summary["morawica"]["regular"] == "ok"
    assert summary["dominikanie"]["regular"] == "ok"


def test_unchanged_hash_skips_llm(tmp_path):
    store = seeded_store(tmp_path)
    llm = FakeLLM(default_exc=[exc_raw("2026-11-15")])
    fetcher = good_fetcher()
    run_nightly(FRI, store, ALL, llm, fetcher=fetcher)
    n = len(llm.calls)
    assert n > 0
    later = FRI + timedelta(days=7)
    summary = run_nightly(later, store, ALL, llm, fetcher=fetcher)
    assert len(llm.calls) == n
    assert summary["morawica"] == {"regular": "skipped", "exceptions": "skipped"}
    exc = store.load()["sources"]["morawica"]["exceptions"]
    # unchanged content still counts as freshly read, entries kept
    assert exc["fetched_at"] == later.isoformat()
    assert exc["error"] is None


def test_changed_content_reextracts(tmp_path):
    store = seeded_store(tmp_path)
    llm = FakeLLM()
    run_nightly(FRI, store, ALL, llm, fetcher=good_fetcher())
    n = len(llm.calls)
    changed = good_fetcher(**{MORAWICA_URL: html_doc(MORAWICA_URL, "Msze: 10.00")})
    run_nightly(FRI + timedelta(days=7), store, ALL, llm, fetcher=changed)
    assert len(llm.calls) > n


def test_one_source_failing_does_not_block_others_and_keeps_old_cache(tmp_path):
    store = seeded_store(tmp_path)
    old = store.load()["sources"]["dominikanie"]
    fetcher = good_fetcher(**{DOM_LISTING: FetchError(f"{DOM_LISTING}: HTTP 503")})
    llm = FakeLLM(default_exc=[exc_raw("2026-11-15")])
    summary = run_nightly(FRI, store, ALL, llm, fetcher=fetcher)
    assert summary["dominikanie"]["exceptions"].startswith("error:")
    assert summary["morawica"]["exceptions"] == "ok"
    cache = store.load()
    dom = cache["sources"]["dominikanie"]
    assert dom["exceptions"]["entries"] == old["exceptions"]["entries"]
    assert dom["exceptions"]["fetched_at"] == old["exceptions"]["fetched_at"]
    assert "503" in dom["exceptions"]["error"]
    assert cache["sources"]["morawica"]["exceptions"]["error"] is None


def test_llm_failure_is_recorded_and_keeps_entries(tmp_path):
    store = seeded_store(tmp_path)
    old = store.load()["sources"]["morawica"]["regular"]["entries"]
    llm = FakeLLM(regular=[{**REGULAR_RAW, "time": "03:00"}])  # invalid -> ExtractionError
    summary = run_nightly(FRI, store, ALL, llm, fetcher=good_fetcher())
    assert summary["morawica"]["regular"].startswith("error:")
    reg = store.load()["sources"]["morawica"]["regular"]
    assert reg["entries"] == old
    assert reg["error"]


def test_heartbeat_written_even_when_all_fail(tmp_path):
    store = seeded_store(tmp_path)
    fetcher = FakeFetcher({})  # everything 404
    summary = run_nightly(FRI, store, ALL, FakeLLM(), fetcher=fetcher)
    assert all(v["exceptions"].startswith("error:") for v in summary.values())
    assert store.load()["heartbeat"] == FRI.isoformat()


def test_dry_run_does_not_write_cache(tmp_path):
    store = Store(str(tmp_path / "cache.json"))
    llm = FakeLLM(default_exc=[exc_raw("2026-11-15")])
    summary = run_nightly(FRI, store, ALL, llm, fetcher=good_fetcher(), dry_run=True)
    assert not (tmp_path / "cache.json").exists()
    assert summary["morawica"]["regular"] == "ok"
    assert summary["morawica"]["regular_entries"][0]["time"] == "09:00"
    assert summary["morawica"]["exception_entries"][0]["date"] == "2026-11-15"
    json.dumps(summary)  # CLI pretty-prints it


def test_dry_run_leaves_existing_cache_untouched(tmp_path):
    store = seeded_store(tmp_path)
    before = (tmp_path / "cache.json").read_text()
    run_nightly(FRI, store, ALL, FakeLLM(), fetcher=good_fetcher(), dry_run=True)
    assert (tmp_path / "cache.json").read_text() == before


def test_bulletin_pdf_links_are_fetched_and_extracted(tmp_path):
    pdf_url = "https://dom.example/files/ogloszenia-15-11.pdf"
    fetcher = good_fetcher(**{
        DOM_LISTING: html_doc(DOM_LISTING, "lista", links=[pdf_url]),
        pdf_url: pdf_doc(pdf_url),
    })
    store = seeded_store(tmp_path)
    llm = FakeLLM(exceptions={pdf_url: [exc_raw("2026-11-16", "18:00", "cancel")]})
    run_nightly(THU, store, ALL, llm, fetcher=fetcher)
    assert pdf_url in fetcher.urls
    assert ("exceptions", [pdf_url]) in llm.calls
    entries = store.load()["sources"]["dominikanie"]["exceptions"]["entries"]
    assert [(e["date"], e["source_url"]) for e in entries] == [("2026-11-16", pdf_url)]


def test_listing_page_itself_is_extracted_too(tmp_path):
    store = seeded_store(tmp_path)
    llm = FakeLLM()
    run_nightly(THU, store, ALL, llm, fetcher=good_fetcher())
    assert ("exceptions", [DOM_LISTING]) in llm.calls


def test_bulletin_links_picked_newest_first_by_url_date(tmp_path):
    base = "https://dom.example/"
    old = base + "2026/10/20/ogloszenia-xxx/"
    mid = base + "2026/10/27/ogloszenia-yyy/"
    new = base + "2026/10/31/ogloszenia-zzz/"  # newest listed LAST
    fetcher = good_fetcher(**{
        DOM_LISTING: html_doc(DOM_LISTING, "lista", links=[old, mid, new]),
        old: html_doc(old), mid: html_doc(mid), new: html_doc(new),
    })
    store = seeded_store(tmp_path)
    llm = FakeLLM()
    run_nightly(THU, store, [DOMINIKANIE], llm, fetcher=fetcher)
    fetched = fetcher.urls
    assert new in fetched and mid in fetched
    assert old not in fetched
    assert fetched.index(new) < fetched.index(mid)


def test_undated_links_follow_dated_ones_in_document_order():
    links = ["https://x/ogloszenia/a", "https://x/2026/10/01/ogloszenia-b/",
             "https://x/ogloszenia/c", "https://x/2026/10/09/ogloszenia-d/"]
    assert job.order_bulletin_links(links) == [
        "https://x/2026/10/09/ogloszenia-d/", "https://x/2026/10/01/ogloszenia-b/",
        "https://x/ogloszenia/a", "https://x/ogloszenia/c"]


def test_single_bulletin_link_failure_does_not_abort_source(tmp_path):
    base = "https://dom.example/"
    broken = base + "2026/10/31/ogloszenia-broken/"
    ok = base + "2026/10/27/ogloszenia-ok/"
    fetcher = good_fetcher(**{
        DOM_LISTING: html_doc(DOM_LISTING, "lista", links=[ok, broken]),
        ok: html_doc(ok),
        broken: FetchError(f"{broken}: HTTP 500"),
    })
    store = seeded_store(tmp_path)
    llm = FakeLLM(exceptions={ok: [exc_raw("2026-11-15")]})
    summary = run_nightly(THU, store, [DOMINIKANIE], llm, fetcher=fetcher)
    assert summary["dominikanie"]["exceptions"] == "ok"
    entries = store.load()["sources"]["dominikanie"]["exceptions"]["entries"]
    assert [e["source_url"] for e in entries] == [ok]


def test_bulletin_link_regex_from_config_is_used(tmp_path):
    base = "https://dom.example/"
    hit = base + "ogloszenia/pierwsze,12"
    miss = base + "inne/ogloszenia-xyz"
    church_ = church("dominikanie", schedule_urls=(DOM_SCHEDULE,), ogloszenia_urls=(DOM_LISTING,),
                     bulletin_link_regex=r"/ogloszenia/[^/]+,\d+$")
    fetcher = good_fetcher(**{
        DOM_LISTING: html_doc(DOM_LISTING, "lista", links=[miss, hit]), hit: html_doc(hit),
    })
    run_nightly(THU, seeded_store(tmp_path), [church_], FakeLLM(), fetcher=fetcher)
    assert hit in fetcher.urls and miss not in fetcher.urls


def test_exceptions_merged_not_replaced(tmp_path):
    store = seeded_store(tmp_path)
    cache = store.load()
    cache["sources"]["morawica"]["exceptions"]["entries"] = [
        {**exc_raw("2026-11-14", note="old"), "source_url": "http://old/"},
        {**exc_raw("2026-11-01", note="past"), "source_url": "http://old/"},
    ]
    store.save(cache)
    llm = FakeLLM(default_exc=[exc_raw("2026-11-15", note="new")])
    run_nightly(THU, store, [MORAWICA], llm, fetcher=good_fetcher())
    entries = store.load()["sources"]["morawica"]["exceptions"]["entries"]
    assert [e["note"] for e in entries] == ["old", "new"]  # past one dropped


def test_empty_exception_result_is_ok(tmp_path):
    store = seeded_store(tmp_path)
    summary = run_nightly(THU, store, [MORAWICA], FakeLLM(default_exc=[]), fetcher=good_fetcher())
    assert summary["morawica"]["exceptions"] == "ok"


def test_filia_shares_source_fetch(tmp_path):
    store = Store(str(tmp_path / "cache.json"))
    fetcher = good_fetcher()
    llm = FakeLLM()
    summary = run_nightly(FRI, store, [MORAWICA, BRZOSKWINIA], llm, fetcher=fetcher)
    assert set(summary) == {"morawica"}
    # schedule and ogloszenia share one URL: fetched once, extracted once per kind
    assert fetcher.urls.count(MORAWICA_URL) == 1
    assert len(llm.kinds("regular")) == 1
    assert len(llm.kinds("exceptions")) == 1
    assert "brzoskwinia" not in store.load()["sources"]


def test_headers_of_source_church_are_passed(tmp_path):
    fetcher = good_fetcher()
    run_nightly(FRI, seeded_store(tmp_path), [DOMINIKANIE], FakeLLM(), fetcher=fetcher)
    assert all(h == DOM_HEADERS for u, h in fetcher.calls)


def test_error_message_is_short_and_has_no_secret(tmp_path):
    store = seeded_store(tmp_path)

    class Boom:
        def complete_json(self, *a, **k):
            raise RuntimeError("sk-or-SECRET " * 100)

    run_nightly(THU, store, [MORAWICA], Boom(), fetcher=good_fetcher())
    err = store.load()["sources"]["morawica"]["exceptions"]["error"]
    assert "SECRET" not in err and len(err) <= 200


# ------------------------------------------------------------------ scheduler

def test_next_run_after_today_and_tomorrow():
    assert next_run_after(datetime(2026, 11, 12, 1, 0, tzinfo=WARSAW), 3) == \
        datetime(2026, 11, 12, 3, 0, tzinfo=WARSAW)
    assert next_run_after(datetime(2026, 11, 12, 3, 0, tzinfo=WARSAW), 3) == \
        datetime(2026, 11, 13, 3, 0, tzinfo=WARSAW)
    assert next_run_after(datetime(2026, 11, 12, 15, 0, tzinfo=WARSAW), 3) == \
        datetime(2026, 11, 13, 3, 0, tzinfo=WARSAW)


def test_next_run_after_converts_other_timezones():
    # 23:30 UTC on Nov 12 is 00:30 Warsaw on Nov 13 -> next run Nov 13 03:00 Warsaw
    nxt = next_run_after(datetime(2026, 11, 12, 23, 30, tzinfo=timezone.utc), 3)
    assert nxt == datetime(2026, 11, 13, 3, 0, tzinfo=WARSAW)
    assert nxt.utcoffset() == timedelta(hours=1)


def test_next_run_after_is_dst_safe():
    # clocks go back 2026-10-25: 03:00 that day is CET (+1); wait is 16 real hours from noon
    now = datetime(2026, 10, 24, 12, 0, tzinfo=WARSAW)
    nxt = next_run_after(now, 3)
    assert (nxt.year, nxt.month, nxt.day, nxt.hour) == (2026, 10, 25, 3)
    assert nxt.utcoffset() == timedelta(hours=1)
    assert nxt - now.astimezone(timezone.utc) == timedelta(hours=16)
    # clocks go forward 2027-03-28: 03:00 that day is CEST (+2); wait is 14 real hours from noon
    now = datetime(2027, 3, 27, 12, 0, tzinfo=WARSAW)
    nxt = next_run_after(now, 3)
    assert (nxt.month, nxt.day, nxt.hour) == (3, 28, 3)
    assert nxt.utcoffset() == timedelta(hours=2)
    assert nxt.astimezone(timezone.utc) - now.astimezone(timezone.utc) == timedelta(hours=14)


def test_next_run_after_requires_aware_datetime():
    with pytest.raises(ValueError):
        next_run_after(datetime(2026, 11, 12, 1, 0), 3)


class FakeClock:
    def __init__(self, start):
        self.now = start
        self.waits = []

    def __call__(self):
        return self.now


class FakeStop:
    """threading.Event lookalike whose wait() advances a fake clock."""

    def __init__(self, clock):
        self.clock = clock
        self._set = False

    def is_set(self):
        return self._set

    def set(self):
        self._set = True

    def wait(self, timeout=None):
        self.clock.waits.append(timeout)
        self.clock.now += timedelta(seconds=timeout)
        return self._set


def test_serve_scheduler_sleeps_in_short_chunks_then_runs():
    clock = FakeClock(datetime(2026, 11, 12, 1, 0, tzinfo=WARSAW))
    stop = FakeStop(clock)
    runs = []

    def run():
        runs.append(clock.now)
        stop.set()

    serve_scheduler(run, hour=3, stop=stop, now_fn=clock)
    assert len(runs) == 1
    assert runs[0] == datetime(2026, 11, 12, 3, 0, tzinfo=WARSAW)
    assert max(clock.waits) <= 60
    assert len(clock.waits) == 120


def test_serve_scheduler_logs_exceptions_and_continues(caplog):
    clock = FakeClock(datetime(2026, 11, 12, 2, 59, tzinfo=WARSAW))
    stop = FakeStop(clock)
    calls = []

    def run():
        calls.append(clock.now)
        if len(calls) == 1:
            raise RuntimeError("boom")
        stop.set()

    with caplog.at_level("ERROR"):
        serve_scheduler(run, hour=3, stop=stop, now_fn=clock)
    assert len(calls) == 2
    assert calls[1].date() > calls[0].date()  # next night
    assert any("boom" in r.getMessage() or r.exc_info for r in caplog.records)


def test_serve_scheduler_stops_while_waiting():
    clock = FakeClock(datetime(2026, 11, 12, 1, 0, tzinfo=WARSAW))
    stop = FakeStop(clock)
    stop.set()
    runs = []
    serve_scheduler(lambda: runs.append(1), hour=3, stop=stop, now_fn=clock)
    assert runs == []


# ------------------------------------------------------------------ CLI

def test_cli_requires_api_key(monkeypatch, capsys):
    monkeypatch.delenv("OPENROUTER_API_KEY", raising=False)
    with pytest.raises(SystemExit) as e:
        job.main(["--once"])
    assert e.value.code != 0
    assert "OPENROUTER_API_KEY" in capsys.readouterr().err


def test_cli_once_dry_run_prints_entries_and_does_not_write(monkeypatch, tmp_path, capsys):
    monkeypatch.setenv("OPENROUTER_API_KEY", "sk-or-dummy-key")
    monkeypatch.setenv("DATA_DIR", str(tmp_path))
    churches = tmp_path / "churches.yaml"
    churches.write_text(
        "churches:\n  - id: a\n    name: A\n    mode: car\n    eta_min: 1\n    eta_max: 2\n"
        "    schedule_urls: ['http://a/']\n    ogloszenia_urls: ['http://a/']\n")
    monkeypatch.setenv("CHURCHES", str(churches))
    monkeypatch.setattr(job, "fetch_doc", FakeFetcher({"http://a/": html_doc("http://a/")}))
    monkeypatch.setattr(job, "OpenRouter", lambda key: FakeLLM())
    assert job.main(["--once", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "09:00" in out and "sk-or-dummy-key" not in out
    assert not (tmp_path / "cache.json").exists()
