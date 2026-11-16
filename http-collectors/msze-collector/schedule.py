"""Schedule engine: regular entries + one-off exceptions -> Masses -> /upcoming rows."""
from __future__ import annotations

import logging
import math
from datetime import date, datetime, time, timedelta, timezone
from zoneinfo import ZoneInfo

from calendar_pl import day_key, in_winter
from config import Church

log = logging.getLogger("msze.schedule")

WARSAW = ZoneInfo("Europe/Warsaw")
UTC = timezone.utc

EXC_STALE_DAYS = 3
REG_STALE_DAYS = 14
REG_DROP_DAYS = 30

BADGE_ASSUMED = "sezon: zakładam"
BADGE_NO_DATA = "brak danych"
BADGE_NO_BULLETINS = "bez ogłoszeń"


def _hm(value) -> str | None:
    """Normalise 'H:MM'/'HH:MM' to 'HH:MM'; None when invalid."""
    try:
        h, m = str(value).split(":")
        return time(int(h), int(m)).strftime("%H:%M")
    except (ValueError, TypeError):
        return None


def _loc_ok(item_loc: str | None, church_loc: str | None) -> bool:
    if church_loc is None or item_loc is None:
        return True
    return item_loc.strip().casefold() == church_loc.strip().casefold()


def _in_range(d: date, frm: str, to: str) -> bool:
    key = (d.month, d.day)
    f = tuple(int(p) for p in frm.split("-"))
    t = tuple(int(p) for p in to.split("-"))
    return f <= key <= t if f <= t else (key >= f or key <= t)


def _season_ok(season, d: date) -> bool:
    if season in (None, "all"):
        return True
    if season == "winter":
        return in_winter(d)
    if season == "summer":
        return not in_winter(d)
    if isinstance(season, dict):
        try:
            return _in_range(d, season["from"], season["to"])
        except (KeyError, ValueError):
            log.warning("bad season range %r ignored", season)
            return False
    log.warning("unknown season %r ignored", season)
    return False


def masses_on(day: date, entries: list[dict], excs: list[dict],
              location: str | None) -> list[dict]:
    """Masses of one day, sorted by time. Exceptions that do not match a real Mass are ignored."""
    key = day_key(day)
    masses: dict[str, dict] = {}
    for ent in entries:
        if ent.get("day") != key or not _loc_ok(ent.get("location"), location):
            continue
        hm = _hm(ent.get("time"))
        if hm is None or not _season_ok(ent.get("season"), day):
            continue
        masses.setdefault(hm, {"start": hm, "changed": False, "note": "",
                               "season_assumed": bool(ent.get("season_assumed"))})

    iso = day.isoformat()
    todays = [x for x in excs
              if x.get("date") == iso and _loc_ok(x.get("location"), location)]

    for x in (x for x in todays if x.get("action") in ("cancel", "move")):
        action, hm = x["action"], _hm(x.get("time"))
        if action == "cancel" and x.get("scope") == "all_that_day":
            masses.clear()
        elif hm is None or hm not in masses:
            log.info("ignoring %s of nonexistent Mass %s on %s (%s)",
                     action, x.get("time"), iso, location)
        elif action == "cancel":
            del masses[hm]
        else:
            new = _hm(x.get("new_time"))
            if new is None:
                log.info("ignoring move of %s on %s: no valid new_time", hm, iso)
                continue
            m = masses.pop(hm)
            m.update(start=new, changed=True, note=x.get("note") or "")
            masses[new] = m

    for x in (x for x in todays if x.get("action") == "add"):
        hm = _hm(x.get("time"))
        if hm is None:
            log.info("ignoring add without valid time on %s", iso)
        elif hm in masses:
            log.info("ignoring add of existing Mass %s on %s", hm, iso)
        else:
            masses[hm] = {"start": hm, "changed": True, "note": x.get("note") or "",
                          "season_assumed": False}

    return [masses[k] for k in sorted(masses)]


def _age(fetched_at: str | None, now: datetime) -> timedelta | None:
    if not fetched_at:
        return None
    try:
        dt = datetime.fromisoformat(fetched_at)
    except ValueError:
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=WARSAW)
    return now.astimezone(UTC) - dt.astimezone(UTC)


def _eta(c: Church) -> str:
    return f"{c.eta_min} min" if c.eta_min == c.eta_max else f"{c.eta_min}–{c.eta_max} min"


def _no_data(c: Church, age_days: int | None) -> dict:
    return {"start": None, "start_hm": "", "church": c.name, "mode": c.mode, "eta": _eta(c),
            "eta_min": c.eta_min, "eta_max": c.eta_max, "leave_in_min": None,
            "status": "no_data", "badges": BADGE_NO_DATA, "data_age_days": age_days,
            "changed": False, "note": ""}


def build_rows(now: datetime, churches: list[Church], cache: dict, hours: int = 3) -> list[dict]:
    if now.tzinfo is None:
        raise ValueError("now must be timezone-aware")
    now = now.astimezone(WARSAW)
    end = now + timedelta(hours=hours)
    days, d = [], now.date()
    while d <= end.date():
        days.append(d)
        d += timedelta(days=1)

    rows: list[dict] = []
    no_data: list[dict] = []
    for c in churches:
        src = (cache.get("sources") or {}).get(c.source_id) or {}
        reg, exc = src.get("regular") or {}, src.get("exceptions") or {}
        entries = [en for en in (reg.get("entries") or []) if _loc_ok(en.get("location"), c.location)]
        reg_age = _age(reg.get("fetched_at"), now)
        if not entries or reg_age is None or reg_age > timedelta(days=REG_DROP_DAYS):
            no_data.append(_no_data(c, reg_age.days if reg_age is not None else None))
            continue

        stale = []
        if reg_age > timedelta(days=REG_STALE_DAYS):
            stale.append(f"rozkład sprzed {reg_age.days} dni")
        exc_age = _age(exc.get("fetched_at"), now)
        if exc_age is None:
            # never read: absence of one-off changes must not look like "none exist"
            stale.append(BADGE_NO_BULLETINS)
        elif exc_age > timedelta(days=EXC_STALE_DAYS):
            stale.append(f"bez ogłoszeń od {exc_age.days} dni")
        excs = exc.get("entries") or []

        for day in days:
            for m in masses_on(day, entries, excs, c.location):
                h, mi = map(int, m["start"].split(":"))
                start = datetime.combine(day, time(h, mi), tzinfo=WARSAW)
                # aware subtraction within one tzinfo ignores DST offsets; go through UTC
                delta = start.astimezone(UTC) - now.astimezone(UTC)
                if delta < timedelta(0) or delta > timedelta(hours=hours):
                    continue
                leave_in = math.floor(delta.total_seconds() / 60) - c.eta_max
                badges = list(stale)
                if m["changed"]:
                    badges.insert(0, m["note"] or "zmiana")
                if m["season_assumed"]:
                    badges.insert(0, BADGE_ASSUMED)
                rows.append({
                    "start": start.isoformat(), "start_hm": m["start"], "church": c.name,
                    "mode": c.mode, "eta": _eta(c), "eta_min": c.eta_min, "eta_max": c.eta_max,
                    "leave_in_min": leave_in, "status": "late" if leave_in < 0 else "ok",
                    "badges": " · ".join(badges), "data_age_days": reg_age.days,
                    "changed": m["changed"], "note": m["note"],
                    "_t": delta,
                })

    rows.sort(key=lambda r: (r["_t"], r["eta_max"]))
    for r in rows:
        del r["_t"]
    return rows + no_data
