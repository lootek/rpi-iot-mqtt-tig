import logging
from datetime import date, datetime, timedelta
from zoneinfo import ZoneInfo

import pytest

from config import Church
from schedule import build_rows, masses_on

WAW = ZoneInfo("Europe/Warsaw")


def church(cid="morawica", mode="car", eta=(10, 15), location=None, schedule_from=None):
    return Church(id=cid, name=cid.capitalize(), mode=mode, eta_min=eta[0], eta_max=eta[1],
                  location=location, schedule_from=schedule_from)


def e(day, time, season="all", location=None, assumed=False):
    return {"location": location, "day": day, "time": time, "season": season,
            "season_assumed": assumed, "quote": ""}


def x(d, action, time=None, new_time=None, scope="one", location=None, note=""):
    return {"date": d, "time": time, "action": action, "new_time": new_time,
            "scope": scope, "location": location, "note": note, "source_url": ""}


def part(entries, fetched_at):
    return {"entries": entries, "fetched_at": fetched_at.isoformat() if fetched_at else None,
            "content_hash": None, "error": None}


def cache(source="morawica", regular=(), exceptions=(), now=None, reg_age=0, exc_age=0):
    now = now or datetime(2026, 11, 12, 8, 0, tzinfo=WAW)
    utc = now.astimezone(ZoneInfo("UTC"))  # exact ages, independent of DST
    return {"sources": {source: {
        "regular": part(list(regular), utc - timedelta(days=reg_age)),
        "exceptions": part(list(exceptions), utc - timedelta(days=exc_age)),
    }}, "heartbeat": None}


def hms(masses):
    return [m["start"] for m in masses]


# 2026-11-12 is a Thursday, 11-13 Friday, 11-14 Saturday, 11-15 Sunday.
THU, FRI, SUN = date(2026, 11, 12), date(2026, 11, 13), date(2026, 11, 15)


def test_weekday_masses_from_regular():
    entries = [e("thu", "07:00"), e("thu", "18:00"), e("fri", "07:00")]
    assert hms(masses_on(THU, entries, [], None)) == ["07:00", "18:00"]


def test_sunday_uses_sun_holiday_list():
    entries = [e("sun_holiday", "09:00"), e("sun_holiday", "11:00"), e("thu", "07:00")]
    assert hms(masses_on(SUN, entries, [], None)) == ["09:00", "11:00"]
    # holiday on a weekday (11 Nov, Wednesday) also uses the Sunday list
    assert hms(masses_on(date(2026, 11, 11), entries, [], None)) == ["09:00", "11:00"]


def test_winter_season_entry_only_in_winter():
    entries = [e("thu", "07:00", season="winter"), e("thu", "19:00", season="summer")]
    assert hms(masses_on(date(2026, 11, 12), entries, [], None)) == ["07:00"]
    assert hms(masses_on(date(2026, 7, 9), entries, [], None)) == ["19:00"]
    # boundaries: winter starts last Sunday of October (2026-10-25)
    assert hms(masses_on(date(2026, 10, 22), entries, [], None)) == ["19:00"]
    sun = [e("sun_holiday", "10:00", season="winter")]
    assert hms(masses_on(date(2026, 10, 25), sun, [], None)) == ["10:00"]


def test_season_date_range_incl_year_wrap():
    entries = [e("thu", "06:30", season={"from": "12-01", "to": "02-28"})]
    assert hms(masses_on(date(2026, 12, 3), entries, [], None)) == ["06:30"]
    assert hms(masses_on(date(2027, 1, 7), entries, [], None)) == ["06:30"]
    assert masses_on(date(2026, 11, 12), entries, [], None) == []


def test_winter_without_dates_sets_season_assumed_badge():
    entries = [e("thu", "07:00", season="winter", assumed=True), e("thu", "08:00")]
    ms = masses_on(THU, entries, [], None)
    assert [m["season_assumed"] for m in ms] == [True, False]
    now = datetime(2026, 11, 12, 6, 0, tzinfo=WAW)
    rows = build_rows(now, [church()], cache(regular=entries, now=now))
    by_hm = {r["start_hm"]: r for r in rows}
    assert "sezon: zakładam" in by_hm["07:00"]["badges"]
    assert "sezon: zakładam" not in by_hm["08:00"]["badges"]


def test_location_filter_excludes_other_places():
    entries = [e("thu", "07:00", location="Morawica"), e("thu", "08:00", location="Brzoskwinia"),
               e("thu", "09:00", location="Kleszczów"), e("thu", "10:00", location=None)]
    assert hms(masses_on(THU, entries, [], "Morawica")) == ["07:00", "10:00"]
    assert hms(masses_on(THU, entries, [], "Brzoskwinia")) == ["08:00", "10:00"]
    assert hms(masses_on(THU, entries, [], None)) == ["07:00", "08:00", "09:00", "10:00"]


def test_cancel_exception_removes_mass_and_flags_nothing_else():
    entries = [e("thu", "07:00"), e("thu", "18:00")]
    ms = masses_on(THU, entries, [x("2026-11-12", "cancel", "07:00")], None)
    assert hms(ms) == ["18:00"] and ms[0]["changed"] is False


def test_move_exception_replaces_time_and_flags_changed():
    entries = [e("thu", "07:00"), e("thu", "18:00")]
    exc = x("2026-11-12", "move", "18:00", new_time="19:30", note="przesunięta")
    ms = masses_on(THU, entries, [exc], None)
    assert hms(ms) == ["07:00", "19:30"]
    assert [m["changed"] for m in ms] == [False, True]
    assert ms[1]["note"] == "przesunięta"


def test_add_exception_adds_changed_mass():
    ms = masses_on(THU, [e("thu", "07:00")], [x("2026-11-12", "add", "20:00", note="Zaduszki")], None)
    assert hms(ms) == ["07:00", "20:00"]
    assert ms[1]["changed"] is True and ms[1]["note"] == "Zaduszki"


def test_cancel_all_that_day():
    entries = [e("thu", "07:00"), e("thu", "18:00")]
    exc = x("2026-11-12", "cancel", None, scope="all_that_day")
    assert masses_on(THU, entries, [exc], None) == []
    # exception for another day does not apply
    assert len(masses_on(THU, entries, [x("2026-11-13", "cancel", None, scope="all_that_day")], None)) == 2


def test_exception_location_scoping():
    entries = [e("thu", "18:00", location="Morawica"), e("thu", "18:00", location="Brzoskwinia")]
    exc = x("2026-11-12", "cancel", "18:00", location="Brzoskwinia")
    assert masses_on(THU, entries, [exc], "Morawica") != []
    assert masses_on(THU, entries, [exc], "Brzoskwinia") == []


def test_cancel_of_nonexistent_mass_is_ignored(caplog):
    entries = [e("thu", "07:00")]
    excs = [x("2026-11-12", "cancel", "13:37"),
            x("2026-11-12", "move", "13:37", new_time="14:00"),
            x("2026-11-12", "cancel", None, scope="one")]
    with caplog.at_level(logging.INFO):
        ms = masses_on(THU, entries, excs, None)  # must not raise
    assert hms(ms) == ["07:00"] and ms[0]["changed"] is False
    assert "13:37" in caplog.text  # logged, not silent
    now = datetime(2026, 11, 12, 6, 0, tzinfo=WAW)
    rows = build_rows(now, [church()], cache(regular=entries, exceptions=excs, now=now))
    assert [r["start_hm"] for r in rows] == ["07:00"]


def test_window_is_three_hours_inclusive_of_start_now():
    now = datetime(2026, 11, 12, 8, 0, tzinfo=WAW)
    entries = [e("thu", t) for t in ("07:59", "08:00", "11:00", "11:01")]
    rows = build_rows(now, [church()], cache(regular=entries, now=now))
    assert [r["start_hm"] for r in rows] == ["08:00", "11:00"]


def test_leave_in_uses_eta_max():
    now = datetime(2026, 11, 12, 8, 10, tzinfo=WAW)
    c = church(eta=(12, 18))
    rows = build_rows(now, [c], cache(regular=[e("thu", "09:00")], now=now))
    r = rows[0]
    assert r["leave_in_min"] == 32 and r["status"] == "ok"
    assert (r["eta_min"], r["eta_max"], r["eta"]) == (12, 18, "12–18 min")
    assert r["start"] == "2026-11-12T09:00:00+01:00" and r["church"] == "Morawica"
    assert r["mode"] == "car" and r["badges"] == ""


def test_leave_in_floors_partial_minutes():
    now = datetime(2026, 11, 12, 8, 10, 40, tzinfo=WAW)
    rows = build_rows(now, [church(eta=(5, 5))], cache(regular=[e("thu", "09:00")], now=now))
    assert rows[0]["leave_in_min"] == 49 - 5  # floor(49.33) - 5


def test_late_status_when_leave_in_negative():
    now = datetime(2026, 11, 12, 8, 50, tzinfo=WAW)
    rows = build_rows(now, [church(eta=(10, 15))], cache(regular=[e("thu", "09:00")], now=now))
    assert rows[0]["leave_in_min"] == -5 and rows[0]["status"] == "late"


def test_window_crossing_midnight_includes_pasterka():
    now = datetime(2026, 12, 24, 22, 30, tzinfo=WAW)
    excs = [x("2026-12-25", "add", "00:00", note="Pasterka")]
    rows = build_rows(now, [church()], cache(regular=[e("thu", "18:00")], exceptions=excs, now=now))
    assert len(rows) == 1
    assert rows[0]["start"] == "2026-12-25T00:00:00+01:00"
    assert rows[0]["leave_in_min"] == 90 - 15
    assert "Pasterka" in rows[0]["badges"]


def test_window_crossing_midnight_regular_tomorrow():
    now = datetime(2026, 11, 12, 22, 0, tzinfo=WAW)  # Thursday night; Friday 00:30 not a Mass, 01:00 is
    rows = build_rows(now, [church()], cache(regular=[e("thu", "21:00"), e("fri", "01:00")], now=now))
    assert [r["start"][:16] for r in rows] == ["2026-11-13T01:00"]


def test_dst_change_day_leave_in_correct():
    # autumn: 2026-10-25 03:00 CEST -> 02:00 CET; 01:30 CEST is 23:30 UTC
    now = datetime(2026, 10, 25, 1, 30, tzinfo=WAW)
    excs = [x("2026-10-25", "add", "03:00"), x("2026-10-25", "add", "04:00")]
    rows = build_rows(now, [church(eta=(10, 15))], cache(exceptions=excs, regular=[e("sun_holiday", "12:00")], now=now))
    # 03:00 CET = 02:00 UTC -> 150 real minutes away; 04:00 CET is 210 real minutes away: outside 3 h
    assert [r["start_hm"] for r in rows] == ["03:00"]
    assert rows[0]["leave_in_min"] == 150 - 15
    # spring: 2027-03-28 02:00 CET -> 03:00 CEST; 01:30 CET is 00:30 UTC
    now = datetime(2027, 3, 28, 1, 30, tzinfo=WAW)
    excs = [x("2027-03-28", "add", "03:30"), x("2027-03-28", "add", "04:30")]
    rows = build_rows(now, [church(eta=(10, 15))], cache(exceptions=excs, regular=[e("sun_holiday", "12:00")], now=now))
    # 03:30 CEST = 01:30 UTC -> 60 real minutes; 04:30 CEST = 02:30 UTC -> 120
    assert [r["start_hm"] for r in rows] == ["03:30", "04:30"]
    assert [r["leave_in_min"] for r in rows] == [60 - 15, 120 - 15]


def test_naive_now_is_rejected():
    with pytest.raises(ValueError):
        build_rows(datetime(2026, 11, 12, 8, 0), [church()], cache())


def test_sort_ties_by_eta_max():
    now = datetime(2026, 11, 12, 8, 0, tzinfo=WAW)
    far, near = church("dominikanie", eta=(15, 25)), church("karmel", eta=(5, 8))
    c = {"sources": {k: {"regular": part([e("thu", "09:00")], now), "exceptions": part([], now)}
                     for k in ("dominikanie", "karmel")}, "heartbeat": None}
    rows = build_rows(now, [far, near], c)
    assert [r["church"] for r in rows] == ["Karmel", "Dominikanie"]


def test_stale_exceptions_badge_after_3_days():
    now = datetime(2026, 11, 12, 6, 0, tzinfo=WAW)
    ent = [e("thu", "07:00")]
    fresh = build_rows(now, [church()], cache(regular=ent, now=now, exc_age=3))
    assert fresh[0]["badges"] == ""
    stale = build_rows(now, [church()], cache(regular=ent, now=now, exc_age=5))
    assert "bez ogłoszeń od 5 dni" in stale[0]["badges"]
    assert stale[0]["status"] == "ok"


def test_stale_regular_badge_after_14_days():
    now = datetime(2026, 11, 12, 6, 0, tzinfo=WAW)
    ent = [e("thu", "07:00")]
    assert build_rows(now, [church()], cache(regular=ent, now=now, reg_age=14))[0]["badges"] == ""
    r = build_rows(now, [church()], cache(regular=ent, now=now, reg_age=15))[0]
    assert "rozkład sprzed 15 dni" in r["badges"] and r["data_age_days"] == 15


def test_regular_older_than_30_days_gives_no_data_row():
    now = datetime(2026, 11, 12, 6, 0, tzinfo=WAW)
    rows = build_rows(now, [church()], cache(regular=[e("thu", "07:00")], now=now, reg_age=31))
    assert len(rows) == 1
    r = rows[0]
    assert r["status"] == "no_data" and r["leave_in_min"] is None and r["church"] == "Morawica"
    assert "brak danych" in r["badges"]
    # exactly at the threshold still shows Masses
    ok = build_rows(now, [church()], cache(regular=[e("thu", "07:00")], now=now, reg_age=30))
    assert ok[0]["status"] == "ok"


def test_church_without_cache_gives_no_data_row_sorted_last():
    now = datetime(2026, 11, 12, 6, 0, tzinfo=WAW)
    churches = [church("karmel", eta=(1, 2)), church("morawica")]
    rows = build_rows(now, churches, cache(regular=[e("thu", "07:00")], now=now))
    assert [r["status"] for r in rows] == ["ok", "no_data"]
    assert rows[1]["church"] == "Karmel"
    # empty/never-fetched cache parts are also no_data
    assert build_rows(now, [church()], {"sources": {}, "heartbeat": None})[0]["status"] == "no_data"
    assert build_rows(now, [church()], cache(regular=[], now=now))[0]["status"] == "no_data"


def test_filia_reads_source_entries():
    now = datetime(2026, 11, 12, 6, 0, tzinfo=WAW)
    entries = [e("thu", "07:00", location="Morawica"), e("thu", "08:00", location="Brzoskwinia"),
               e("thu", "09:00", location="Kleszczów")]
    brz = church("brzoskwinia", mode="walk", eta=(10, 15), location="Brzoskwinia", schedule_from="morawica")
    mor = church("morawica", location="Morawica")
    rows = build_rows(now, [mor, brz], cache("morawica", entries, now=now))
    got = {(r["church"], r["start_hm"]) for r in rows}
    assert got == {("Morawica", "07:00"), ("Brzoskwinia", "08:00")}


def test_filia_without_own_location_entries_is_no_data():
    now = datetime(2026, 11, 12, 6, 0, tzinfo=WAW)
    brz = church("brzoskwinia", mode="walk", location="Brzoskwinia", schedule_from="morawica")
    rows = build_rows(now, [brz], cache("morawica", [e("thu", "07:00", location="Morawica")], now=now))
    assert [r["status"] for r in rows] == ["no_data"]
