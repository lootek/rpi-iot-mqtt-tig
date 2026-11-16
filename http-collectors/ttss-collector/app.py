#!/usr/bin/env python3
"""ZTP Krakow GTFS-RT departures collector.

Polls the GTFS-RT protobuf feeds (trip updates, vehicle positions, service
alerts) plus the daily GTFS static zip, and serves a small JSON API from
memory for the Grafana Infinity datasource. The HTTP path never touches ZTP:
every request is answered from the last good snapshot.

One fact drives most of the code: ZTP leaves the GTFS-RT ``delay`` field at 0,
so real delay is computed here as predicted departure minus the static
schedule (service-date midnight in Europe/Warsaw + HH:MM:SS from
stop_times.txt, which may legally exceed 24:00:00).
"""

import csv
import io
import json
import os
import threading
import time
import zipfile
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from zoneinfo import ZoneInfo

import requests
from google.transit import gtfs_realtime_pb2

STOP_ID = os.environ.get("STOP_ID", "7689")
STOP_NAME = os.environ.get("STOP_NAME", "Brzoskwinia Kamyk 02")
ROUTES = {r.strip() for r in os.environ.get("ROUTES", "258,298").split(",") if r.strip()}
DIRECTION = os.environ.get("DIRECTION", "Kraków Bronowice")
FEED_BASE = os.environ.get("FEED_BASE", "https://gtfs.ztp.krakow.pl").rstrip("/")
POLL_INTERVAL = int(os.environ.get("POLL_INTERVAL", "20"))
NIGHT_POLL_INTERVAL = int(os.environ.get("NIGHT_POLL_INTERVAL", "300"))
DAY_START = datetime.strptime(os.environ.get("DAY_START", "04:30"), "%H:%M").time()
DAY_END = datetime.strptime(os.environ.get("DAY_END", "00:30"), "%H:%M").time()
TIMETABLE_ROWS = int(os.environ.get("TIMETABLE_ROWS", "12"))
PORT = int(os.environ.get("PORT", "8090"))
STATIC_REFRESH_S = 6 * 3600
STATIC_FAST_REFRESH_S = int(os.environ.get("STATIC_FAST_REFRESH_S", "600"))
RT_MAX_AGE_S = 600
TZ = ZoneInfo("Europe/Warsaw")

SCHEDULE_RELATIONSHIP = {0: "SCHEDULED", 1: "SKIPPED", 2: "NO_DATA", 3: "UNSCHEDULED"}


def log(*args):
    print(f"[{datetime.now(TZ).isoformat(timespec='seconds')}]", *args, flush=True)


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.departures = []
        self.alerts = []
        self.generated_at = 0.0   # epoch of the last good RT poll
        self.rt_ok = 0.0
        self.static_ok = 0.0
        self.rt_failed = False
        self.trips = {}           # trip_id -> static schedule row
        self.zip_headers = {}     # conditional-GET validators for the GTFS zip


STATE = State()


# ---------------------------------------------------------------- static GTFS

def parse_hms(text):
    """'HH:MM:SS' -> seconds; hours above 24 are legal and kept as-is."""
    h, m, s = text.split(":")
    return int(h) * 3600 + int(m) * 60 + int(s)


def service_midnight(date_str):
    """'YYYYMMDD' -> epoch of local midnight (Europe/Warsaw)."""
    return datetime(int(date_str[:4]), int(date_str[4:6]), int(date_str[6:8]),
                    tzinfo=TZ).timestamp()


def trip_sched_epoch(trip_id, sched_dep):
    """Absolute scheduled departure epoch for a static trip, or None.

    ZTP trip_ids embed the service date as the first 8 chars (e.g.
    "20261017_2_129426431_13"), so the timetable view can place a trip in
    time without the RT feed. Trips without the date prefix stay in the
    cache (still useful for live delay math) but are skipped on the board.
    """
    prefix = trip_id[:8]
    if not (len(prefix) == 8 and prefix.isdigit()):
        return None
    try:
        return service_midnight(prefix) + sched_dep
    except ValueError:  # digits, but not a real date
        return None


def parse_static_zip(data):
    """Keep only the stop_times rows calling at STOP_ID, joined with trips.txt."""
    calls = {}
    with zipfile.ZipFile(io.BytesIO(data)) as zf:
        with zf.open("stop_times.txt") as raw:
            for row in csv.DictReader(io.TextIOWrapper(raw, encoding="utf-8-sig")):
                if row.get("stop_id") == STOP_ID:
                    calls[row["trip_id"]] = {
                        "sched_dep": parse_hms(row["departure_time"]),
                        "stop_sequence": int(row["stop_sequence"] or 0),
                        "stop_headsign": row.get("stop_headsign") or None,
                    }
        trips = {}
        with zf.open("trips.txt") as raw:
            for row in csv.DictReader(io.TextIOWrapper(raw, encoding="utf-8-sig")):
                if row["trip_id"] in calls:
                    call = calls[row["trip_id"]]
                    trips[row["trip_id"]] = {
                        "route": row.get("route_id"),
                        "headsign": row.get("trip_headsign") or None,
                        "direction_id": row.get("direction_id"),
                        "sched_dep": call["sched_dep"],
                        "sched_epoch": trip_sched_epoch(row["trip_id"], call["sched_dep"]),
                        "stop_sequence": call["stop_sequence"],
                        "stop_headsign": call["stop_headsign"],
                    }
    return trips


def refresh_static():
    headers = dict(STATE.zip_headers)
    r = requests.get(f"{FEED_BASE}/GTFS_KRK_A.zip", headers=headers, timeout=180)
    if r.status_code == 304:
        with STATE.lock:
            STATE.static_ok = time.time()
        log("static: GTFS zip not modified, keeping cache")
        return
    r.raise_for_status()
    validators = {}
    if r.headers.get("ETag"):
        validators["If-None-Match"] = r.headers["ETag"]
    if r.headers.get("Last-Modified"):
        validators["If-Modified-Since"] = r.headers["Last-Modified"]
    trips = parse_static_zip(r.content)
    with STATE.lock:
        STATE.trips = trips
        STATE.zip_headers = validators
        STATE.static_ok = time.time()
    log(f"static: {len(trips)} scheduled trips call at stop {STOP_ID}")


def static_sleep_s():
    """6 h between static refreshes, except when the cached timetable is
    about to run dry.

    The daily zip only covers its own service date, so near the end of
    service the next regular tick could land hours after ZTP publishes the
    next day's feed — poll every STATIC_FAST_REFRESH_S instead so the board
    refills quickly (conditional GETs, so mostly 304s).
    """
    now = time.time()
    with STATE.lock:
        horizon = max((t["sched_epoch"] for t in STATE.trips.values()
                       if t["route"] in ROUTES and t["sched_epoch"]), default=0.0)
    return STATIC_REFRESH_S if horizon >= now + STATIC_REFRESH_S else STATIC_FAST_REFRESH_S


def static_refresher():
    while True:
        started = time.time()
        try:
            refresh_static()
        except Exception as exc:
            log(f"static refresh failed, keeping old cache: {exc}")
        time.sleep(static_sleep_s())


# -------------------------------------------------------------------- real time

def fetch_feed(name, attempts=3):
    """Fetch and decode one protobuf feed.

    TripUpdates_A.pb comes back truncated roughly one request in five (ZTP
    serves the file mid-rewrite), which protobuf rejects as corrupt wire
    format — so retry before giving up and letting the fail-open path take over.
    """
    last_exc = None
    for i in range(attempts):
        try:
            r = requests.get(f"{FEED_BASE}/{name}", timeout=10)
            r.raise_for_status()
            msg = gtfs_realtime_pb2.FeedMessage()
            msg.ParseFromString(r.content)
            return msg
        except Exception as exc:
            last_exc = exc
            if i < attempts - 1:
                time.sleep(1)
    raise last_exc


def build_departures(tu_msg, vp_msg, now):
    vehicles = {}
    for e in vp_msg.entity:
        v = e.vehicle
        if v.trip.trip_id:
            vehicles[v.trip.trip_id] = (v.vehicle.id or None,
                                        v.current_stop_sequence or None)

    # Trips crossing midnight carry start_date; the rest are assumed to belong
    # to the local date of the feed itself.
    fallback_date = datetime.fromtimestamp(tu_msg.header.timestamp or now, TZ).strftime("%Y%m%d")

    rows = []
    for e in tu_msg.entity:
        tu = e.trip_update
        if tu.trip.route_id not in ROUTES:
            continue
        stu = next((s for s in tu.stop_time_update if s.stop_id == STOP_ID), None)
        if stu is None:
            continue
        predicted = stu.departure.time or stu.arrival.time
        if not predicted:
            continue

        trip_id = tu.trip.trip_id  # entity.id has a "trip_" prefix; trip_id does not
        static = STATE.trips.get(trip_id)
        date = tu.trip.start_date or (static and fallback_date)
        if static and date:
            sched_epoch = service_midnight(date) + static["sched_dep"]
            scheduled = datetime.fromtimestamp(sched_epoch, TZ).strftime("%H:%M")
            delay_min = round((predicted - sched_epoch) / 60, 1)
        else:
            scheduled, delay_min = None, None

        vehicle_id, current_seq = vehicles.get(trip_id, (None, None))
        stops_away = None
        if vehicle_id and current_seq and static:
            stops_away = static["stop_sequence"] - current_seq

        rows.append((predicted, {
            "trip_id": trip_id,
            "line": tu.trip.route_id,
            "headsign": (static or {}).get("headsign") or (static or {}).get("stop_headsign"),
            "scheduled": scheduled,
            "expected": datetime.fromtimestamp(predicted, TZ).strftime("%H:%M"),
            "delay_min": delay_min,
            "countdown_min": round((predicted - now) / 60, 1),
            "stops_away": stops_away,
            "vehicle": vehicle_id,
            "status": SCHEDULE_RELATIONSHIP.get(stu.schedule_relationship, "SCHEDULED"),
        }))

    rows = [pair for pair in rows if pair[0] - now > -120]
    rows.sort(key=lambda pair: pair[0])
    return [row for _, row in rows]


def build_board(now, live_usable):
    """Next N scheduled departures from the static cache, overlaid with live
    RT rows.

    The RT feed only carries trips with active vehicles, which leaves the
    board empty roughly 21:00-05:00; the timetable view fills those hours.
    Every row gets a "source": "live" when the trip is currently tracked,
    "timetable" when it is schedule-only (expected/delay/vehicle unknown).
    """
    live_rows = [dict(r) for r in (STATE.departures if live_usable else [])]
    for row in live_rows:
        row["source"] = "live"
    live_by_trip = {r["trip_id"]: r for r in live_rows}

    window = sorted(
        (t["sched_epoch"], trip_id) for trip_id, t in STATE.trips.items()
        if t["route"] in ROUTES and t["sched_epoch"] is not None
        and t["sched_epoch"] >= now - 120
    )[:TIMETABLE_ROWS]

    board = []
    for epoch, trip_id in window:
        live = live_by_trip.get(trip_id)
        if live is not None:
            board.append(live)
            continue
        static = STATE.trips[trip_id]
        board.append({
            "trip_id": trip_id,
            "line": static["route"],
            "headsign": static["headsign"] or static["stop_headsign"],
            "scheduled": datetime.fromtimestamp(epoch, TZ).strftime("%H:%M"),
            "expected": None,
            "delay_min": None,
            "countdown_min": round((epoch - now) / 60, 1),
            "stops_away": None,
            "vehicle": None,
            "status": "TIMETABLE",
            "source": "timetable",
        })

    # Live trips outside the static window (unscheduled, or beyond the N-row
    # horizon) keep their place on the board.
    in_window = {trip_id for _, trip_id in window}
    board.extend(r for r in live_rows if r["trip_id"] not in in_window)

    board.sort(key=lambda r: r["countdown_min"])
    return board


def build_alerts(sa_msg, now):
    out = []
    for e in sa_msg.entity:
        a = e.alert
        routes = {ie.route_id for ie in a.informed_entity if ie.route_id}
        if not routes & ROUTES:
            continue
        active = not a.active_period  # no active_period means always active
        for period in a.active_period:
            if (period.start or 0) <= now <= (period.end or now + 1):
                active = True
                break
        if not active:
            continue
        text = None
        for tr in a.header_text.translation:
            if not tr.text:
                continue
            if tr.language == "pl":
                text = tr.text
                break
            if text is None:
                text = tr.text
        if not text:
            continue
        try:
            severity = gtfs_realtime_pb2.Alert.SeverityLevel.Name(a.severity_level)
        except ValueError:
            severity = "UNKNOWN"
        out.append({
            "route": ",".join(sorted(routes & ROUTES)),
            "header_text": text,
            "severity": severity,
            "timestamp": sa_msg.header.timestamp or int(now),
        })
    return out


def rt_poll_interval():
    """Day cadence; back off at night when no vehicles are out there anyway.
    Day window spans midnight (04:30 .. 00:30 next day)."""
    t = datetime.now(TZ).time()
    if t >= DAY_START or t <= DAY_END:
        return POLL_INTERVAL
    return NIGHT_POLL_INTERVAL


def rt_poller():
    while True:
        started = time.time()
        try:
            tu = fetch_feed("TripUpdates_A.pb")
            vp = fetch_feed("VehiclePositions_A.pb")
            sa = fetch_feed("ServiceAlerts_A.pb")
            now = time.time()
            with STATE.lock:
                STATE.departures = build_departures(tu, vp, now)
                STATE.alerts = build_alerts(sa, now)
                STATE.generated_at = now
                STATE.rt_ok = now
                STATE.rt_failed = False
            log(f"rt ok: {len(STATE.departures)} departures, {len(STATE.alerts)} alerts")
        except Exception as exc:
            with STATE.lock:
                STATE.rt_failed = True
            log(f"rt poll failed, serving last snapshot: {exc}")
        time.sleep(max(1, rt_poll_interval() - (time.time() - started)))


# ----------------------------------------------------------------- HTTP server

class Handler(BaseHTTPRequestHandler):
    server_version = "ttss-collector/1.0"

    def do_GET(self):
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        with STATE.lock:
            now = time.time()
            age = now - STATE.generated_at if STATE.generated_at else None
            stale = STATE.rt_failed or age is None
            usable = age is not None and age <= RT_MAX_AGE_S
            if path == "/departures":
                payload = {
                    "stop": {"id": int(STOP_ID) if STOP_ID.isdigit() else STOP_ID,
                             "name": STOP_NAME},
                    "direction": DIRECTION,
                    "generated_at": STATE.generated_at,
                    "feed_age_s": round(age, 1) if age is not None else None,
                    "stale": stale,
                    "departures": build_board(now, usable),
                }
            elif path == "/alerts":
                payload = {"alerts": STATE.alerts if usable else []}
            elif path == "/health":
                payload = {
                    "last_rt_ok": STATE.rt_ok,
                    "last_static_ok": STATE.static_ok,
                    "static_age_h": round((now - STATE.static_ok) / 3600, 1) if STATE.static_ok else None,
                    "feed_age_s": round(age, 1) if age is not None else None,
                    "stale": stale,
                    "departures_count": len(STATE.departures) if usable else 0,
                    "timetable_rows": TIMETABLE_ROWS,
                }
            else:
                self.send_json(404, {"error": "not found"})
                return
        self.send_json(200, payload)

    def send_json(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        log("http", self.address_string(), fmt % args)


def main():
    threading.Thread(target=static_refresher, daemon=True, name="static").start()
    threading.Thread(target=rt_poller, daemon=True, name="rt").start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    log(f"serving stop {STOP_ID} ({STOP_NAME}) lines {','.join(sorted(ROUTES))} "
        f"on port {PORT}, polling every {POLL_INTERVAL}s day / {NIGHT_POLL_INTERVAL}s night")
    server.serve_forever()


if __name__ == "__main__":
    main()
