# ttss-collector

ZTP Kraków GTFS-RT departures API for the Grafana Infinity datasource.
Polls the protobuf feeds every 15 s, keeps the daily GTFS static schedule in
memory, and serves plain JSON from RAM — the HTTP path never touches ZTP.

Single-purpose instance: stop **7689** (Brzoskwinia Kamyk 02), lines **258/298**
towards Kraków Bronowice.

## Endpoints

```sh
curl 127.0.0.1:8090/departures   # departures with countdown, delay, vehicle
curl 127.0.0.1:8090/alerts       # service alerts affecting 258/298 (often [])
curl 127.0.0.1:8090/health       # feed ages, staleness, departure count
```

`/departures` rows: `line, headsign, scheduled, expected, delay_min,
countdown_min, stops_away, vehicle, status, source` (status `SKIPPED` =
cancelled), sorted by expected departure, future-ish only.

## Timetable-backed board

GTFS-RT only carries trips with active vehicles, so a pure live view goes
blank roughly 21:00-05:00. `/departures` therefore always returns the next
`TIMETABLE_ROWS` (default 12) departures from the static schedule — ZTP
trip_ids embed the service date as their first 8 chars, which pins each trip
to an absolute epoch — and overlays live predictions when a trip is currently
tracked. Rows carry `source: "timetable"` (status `TIMETABLE`, `expected`/
`delay_min`/`vehicle` null, countdown from the schedule) or `source: "live"`.

The daily zip covers only its own service date, so once the cached horizon
runs out (end of service) the static refresher switches from the 6 h cadence
to `STATIC_FAST_REFRESH_S` until the next day's feed is picked up.

## The delay is computed, not read

ZTP always leaves the GTFS-RT `delay` field at **0**. Real delay is computed
here as `predicted departure − static schedule`, where the static schedule is
service-date midnight (Europe/Warsaw) plus `HH:MM:SS` from `stop_times.txt`
(hours may exceed 24:00:00 for post-midnight trips). If a trip_id is missing
from the static cache, `delay_min` is `null` and `scheduled` is omitted.

Fail-open: on a fetch error the last good RT snapshot is served for up to
600 s with `stale: true`; a static-refresh error keeps the old cache.

## Environment

| var | default | |
|---|---|---|
| `STOP_ID` | `7689` | stop_id in GTFS |
| `STOP_NAME` | `Brzoskwinia Kamyk 02` | cosmetic |
| `ROUTES` | `258,298` | comma list, filters everything |
| `DIRECTION` | `Kraków Bronowice` | cosmetic |
| `FEED_BASE` | `https://gtfs.ztp.krakow.pl` | |
| `POLL_INTERVAL` | `20` | day-cadence seconds (04:30-00:30) |
| `NIGHT_POLL_INTERVAL` | `300` | night-cadence seconds (00:30-04:30) |
| `TIMETABLE_ROWS` | `12` | next-N scheduled departures on the board |
| `STATIC_FAST_REFRESH_S` | `600` | static poll when the day's trips run out |
| `PORT` | `8090` | |

## Deploy (lothlorien)

```sh
git pull
docker compose -f central-stack/docker-compose.ttss.yml up -d --build
```

## Grafana

```sh
python3 provision_grafana.py     # from the Mac; token in ~/.secrets/grafana
```

Ensures the `ztp-infinity` Infinity datasource (default URL
`http://127.0.0.1:8090` — Grafana runs host-network on the same Pi), the
`Misc` folder and the `mpk` dashboard. The nightly dashboards export will
also snapshot it into `central-stack/dashboards/Misc/`.

## Future

The opposite direction is a second instance: stop **7687** (Nielepice), its
own compose file with different env values and port (e.g. 8091), plus an
analogous dashboard.
