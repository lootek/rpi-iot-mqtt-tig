#!/usr/bin/env python3
"""Grafana alert catalog for the RPi/IoT fleet -- emits provisioning-API JSON.

Why Python and not provisioning YAML: file-provisioned rules are read-only in the
Grafana UI ("You cannot edit provisioned resources from files in Grafana"), and we
want to keep tweaking thresholds by hand. apply.sh therefore pushes these through
the provisioning HTTP API with X-Disable-Provenance: true, which leaves them
editable. This file stays the source of truth; re-running apply.sh re-asserts it.

Usage:  python3 catalog.py > /tmp/alerts.json
"""

import json
import sys

from rosters import (DHT22_ALL, DHT22_FREEZE_RISK, DHT22_INDOOR, EXPECTED_CONTAINERS,
                     HOSTS, SCD30_LOCATIONS, SPS30_LOCATIONS, WIFI_LOCATIONS, rx)

# --- environment ------------------------------------------------------------
DS_MQTT = "fecz574dkbda8b"                            # InfluxDB MQTT     (db: mqtt)
DS_TELEGRAF = "be414ba9-5127-4be8-a621-e910639f96b1"  # InfluxDB Telegraf (db: telegraf)

F_MQTT = "ed1f5847-d688-4bcd-a726-8b0255a22282"       # folder "MQTT"
F_HOST = "a129b860-4707-4296-8eaa-0dee5978b4db"       # folder "Host monitoring"

# IP, not hostname: `lothlorien` does not resolve on the router, so a hostname
# link in an alert email is dead on a phone. Matches root_url in grafana.ini.
GRAFANA_BASE = "http://192.168.10.18:3000"
DASH_ROOM = "aem53k0pia29sd"   # Room conditions
DASH_NODES = "000000127"       # Nodes

MIN, HOUR = 60, 3600


# --- payload builders -------------------------------------------------------
def query(ref, ds, sql, frm, to=0):
    return {
        "refId": ref,
        "relativeTimeRange": {"from": frm, "to": to},
        "datasourceUid": ds,
        "model": {
            "refId": ref,
            "datasource": {"type": "influxdb", "uid": ds},
            "query": sql,
            "rawQuery": True,
            "resultFormat": "time_series",
            "intervalMs": 60000,
            "maxDataPoints": 43200,
        },
    }


def _expr(ref, model):
    model.update({"refId": ref, "datasource": {"type": "__expr__", "uid": "__expr__"}})
    return {"refId": ref, "relativeTimeRange": {"from": 0, "to": 0},
            "datasourceUid": "__expr__", "model": model}


def reduce(ref, src, reducer="last"):
    # dropNN: a series whose window contains only nulls is dropped rather than
    # evaluated as NaN, which would otherwise show up as a bogus alert instance.
    return _expr(ref, {"type": "reduce", "reducer": reducer, "expression": src,
                       "settings": {"mode": "dropNN"}})


def math(ref, expression):
    return _expr(ref, {"type": "math", "expression": expression})


def threshold(ref, src, op, *params):
    return _expr(ref, {
        "type": "threshold", "expression": src,
        "conditions": [{
            "type": "query",
            "evaluator": {"type": op, "params": list(params)},
            "operator": {"type": "and"},
            "query": {"params": [src]},
            "reducer": {"type": "last", "params": []},
        }],
    })


def rule(uid, title, folder, group, data, condition, for_, severity, summary,
         description, domain="iot", no_data="OK", dashboard=None, paused=False):
    annotations = {"summary": summary, "description": description}
    if dashboard:
        # Plain link rather than Grafana's __dashboardUid__ annotation: that one
        # is only accepted alongside __panelId__, and pinning panel IDs makes the
        # rules break quietly whenever a dashboard is rearranged.
        annotations["description"] += f"\n\nDashboard: {GRAFANA_BASE}/d/{dashboard}"
    return {
        "uid": uid,
        "title": title,
        "condition": condition,
        "data": data,
        "noDataState": no_data,
        "execErrState": "Error",
        "for": for_,
        "annotations": annotations,
        "labels": {"severity": severity, "domain": domain},
        "isPaused": paused,
        "orgID": 1,
        "folderUID": folder,
        "ruleGroup": group,
    }


def silence_rule(uid, title, folder, group, ds, sql, window, for_, severity,
                 what, unit_hint, no_data="OK", dashboard=None):
    """A 'has this series stopped reporting?' rule.

    The mechanism that makes this work: a dead sensor's series simply vanishes
    from an InfluxQL result, so a naive query has nothing left to evaluate and
    never fires. Bucketing with fill(0) over a long window keeps the series
    present as zeros, and we then test the most recent bucket.

    The trailing bucket is always partially elapsed, so a slow publisher reads 0
    for a moment on every cycle -- the `for` duration absorbs that, because a
    live sensor never stays at 0 for the full pending period.
    """
    return rule(
        uid, title, folder, group,
        [query("A", ds, sql, window), reduce("B", "A", "last"), threshold("C", "B", "lt", 1)],
        "C", for_, severity,
        summary=f"{what} has stopped reporting",
        description=(
            f"{unit_hint}\n"
            f"No datapoints in the most recent bucket (0 received; expected >0).\n"
            f"Silent for at least {for_}.\n\n"
            "Check: is the device powered, on Wi-Fi, and is mosquitto reachable?\n"
            "If several go quiet at once, look at 'MQTT pipeline down' first -- "
            "that means the broker or the influx bridge died, not the sensors."
        ),
        no_data=no_data, dashboard=dashboard,
    )


# --- rule groups ------------------------------------------------------------
GROUPS = []


def group(folder, name, interval, rules):
    GROUPS.append({"title": name, "folderUid": folder, "interval": interval, "rules": rules})


# ===========================================================================
# MQTT / sensor-liveness
# ===========================================================================
group(F_MQTT, "sensor-liveness", 5 * MIN, [
    silence_rule(
        "iot-dht22-silent", "DHT22 sensor silent", F_MQTT, "sensor-liveness", DS_MQTT,
        f'SELECT count("temperature") FROM "dht22" WHERE $timeFilter '
        f'AND "location" =~ /{rx(DHT22_ALL)}/ GROUP BY time(5m), "location" fill(0)',
        24 * HOUR, "15m", "critical",
        what="DHT22 in {{ $labels.location }}",
        unit_hint="Room: {{ $labels.location }}",
        dashboard=DASH_ROOM,
    ),
    silence_rule(
        "iot-scd30-silent", "SCD30 CO2 sensor silent", F_MQTT, "sensor-liveness", DS_MQTT,
        f'SELECT count("co2") FROM "scd30" WHERE $timeFilter '
        f'AND "location" =~ /{rx(SCD30_LOCATIONS)}/ GROUP BY time(5m), "location" fill(0)',
        24 * HOUR, "15m", "warning",
        what="SCD30 CO2 sensor in {{ $labels.location }}",
        unit_hint="Room: {{ $labels.location }}",
        dashboard=DASH_ROOM,
    ),
    silence_rule(
        "iot-sps30-silent", "SPS30 particulate sensor silent", F_MQTT, "sensor-liveness", DS_MQTT,
        f'SELECT count("mc_2p5") FROM "sps30" WHERE $timeFilter '
        f'AND "location" =~ /{rx(SPS30_LOCATIONS)}/ GROUP BY time(5m), "location" fill(0)',
        24 * HOUR, "15m", "warning",
        what="SPS30 particulate sensor at {{ $labels.location }}",
        unit_hint="Location: {{ $labels.location }}",
    ),
    # The backstop. All 14 DHT22s share one path: ESP nodes -> mosquitto ->
    # mqtt-to-influxdb -> influxdb, every hop on lothlorien. A single failure
    # there silences everything, and this rule names that cause directly
    # instead of leaving you to infer it from 14 sensor alerts.
    rule(
        "iot-pipeline-down", "MQTT pipeline down", F_MQTT, "sensor-liveness",
        [query("A", DS_MQTT,
               'SELECT count("temperature") FROM "dht22" WHERE $timeFilter '
               'GROUP BY time(5m) fill(0)', 6 * HOUR),
         reduce("B", "A", "last"), threshold("C", "B", "lt", 1)],
        "C", "10m", "critical",
        summary="MQTT pipeline down - no sensor data reaching InfluxDB at all",
        description=(
            "Zero DHT22 datapoints arrived in the last bucket, across every room.\n"
            "All 14 sensors failing simultaneously is not plausible, so the fault is "
            "in the shared path rather than the devices.\n\n"
            "Check on lothlorien, in order:\n"
            "  docker ps | grep -E 'mosquitto|mqtt-to-influxdb|influxdb'\n"
            "  docker logs --tail 50 mqtt-to-influxdb\n"
            "  docker logs --tail 50 mosquitto"
        ),
        no_data="Alerting",
    ),
])

# ===========================================================================
# MQTT / air-quality
# ===========================================================================
group(F_MQTT, "air-quality", 1 * MIN, [
    rule(
        "iot-co2-high", "CO2 high", F_MQTT, "air-quality",
        [query("A", DS_MQTT,
               f'SELECT median("co2") FROM "scd30" WHERE $timeFilter '
               f'AND "location" =~ /{rx(SCD30_LOCATIONS)}/ GROUP BY time(5m), "location"',
               15 * MIN),
         reduce("B", "A", "last"), threshold("C", "B", "gt", 2000)],
        "C", "10m", "warning",
        summary='{{ $labels.location }}: CO2 {{ printf "%.0f" $values.B.Value }} ppm (limit 2000)',
        description=(
            "Room:      {{ $labels.location }}\n"
            'Measured:  {{ printf "%.0f" $values.B.Value }} ppm (5-minute median)\n'
            "Threshold: 2000 ppm sustained for 10m\n\n"
            "Above ~2000 ppm concentration and sleep quality measurably suffer.\n"
            "Action: open a window or door for a few minutes."
        ),
        dashboard=DASH_ROOM,
    ),
    # Sensor-trust rule, not an air-quality rule. Outdoor air is ~420 ppm, so a
    # six-hour indoor mean below 300 ppm is not physically reachable -- it means
    # the SCD30's automatic self-calibration has walked off its baseline. Without
    # this, a drifted sensor reads plausibly-low forever and the CO2-high rule
    # silently stops protecting the room.
    rule(
        "iot-co2-implausible-low", "CO2 implausibly low (sensor drift)", F_MQTT, "air-quality",
        [query("A", DS_MQTT,
               f'SELECT mean("co2") FROM "scd30" WHERE $timeFilter '
               f'AND "location" =~ /{rx(SCD30_LOCATIONS)}/ GROUP BY time(1h), "location"',
               6 * HOUR),
         reduce("B", "A", "mean"), threshold("C", "B", "lt", 300)],
        "C", "2h", "warning",
        summary='{{ $labels.location }}: CO2 reads {{ printf "%.0f" $values.B.Value }} ppm - below outdoor baseline, sensor suspect',
        description=(
            "Room:      {{ $labels.location }}\n"
            'Measured:  {{ printf "%.0f" $values.B.Value }} ppm (6-hour mean)\n'
            "Threshold: below 300 ppm sustained for 2h\n\n"
            "Outdoor air sits near 420 ppm, so an indoor room cannot genuinely\n"
            "average below 300. The SCD30's automatic self-calibration has almost\n"
            "certainly drifted, which means its high readings are untrustworthy too.\n\n"
            "Action: expose the sensor to outdoor air for ~10 min with ASC enabled,\n"
            "or force a recalibration. Do not treat this as good air quality."
        ),
        dashboard=DASH_ROOM,
    ),
    rule(
        "iot-pm25-high", "PM2.5 air pollution", F_MQTT, "air-quality",
        [query("A", DS_MQTT,
               'SELECT median("mc_2p5") FROM "sps30" WHERE $timeFilter '
               'GROUP BY time(5m), "location"', 30 * MIN),
         reduce("B", "A", "last"), threshold("C", "B", "gt", 50)],
        "C", "15m", "warning",
        summary='{{ $labels.location }}: PM2.5 {{ printf "%.1f" $values.B.Value }} ug/m3 (limit 50)',
        description=(
            "Location:  {{ $labels.location }}\n"
            'Measured:  {{ printf "%.1f" $values.B.Value }} ug/m3 (5-minute median)\n'
            "Threshold: 50 ug/m3 sustained for 15m\n\n"
            "For reference the WHO 24h guideline is 15 ug/m3.\n"
            "Action: keep windows shut; check for nearby burning."
        ),
    ),
    rule(
        "iot-pm10-high", "PM10 air pollution", F_MQTT, "air-quality",
        [query("A", DS_MQTT,
               'SELECT median("mc_10p0") FROM "sps30" WHERE $timeFilter '
               'GROUP BY time(5m), "location"', 30 * MIN),
         reduce("B", "A", "last"), threshold("C", "B", "gt", 100)],
        "C", "15m", "warning",
        summary='{{ $labels.location }}: PM10 {{ printf "%.1f" $values.B.Value }} ug/m3 (limit 100)',
        description=(
            "Location:  {{ $labels.location }}\n"
            'Measured:  {{ printf "%.1f" $values.B.Value }} ug/m3 (5-minute median)\n'
            "Threshold: 100 ug/m3 sustained for 15m\n\n"
            "Action: keep windows shut; check for nearby burning."
        ),
    ),
])

# ===========================================================================
# MQTT / environment
#
# Every threshold below reduces a median or mean, never last(). The live data
# contains real single-sample garbage -- living_room logged -11.8 C and garage
# exactly 0.0 C this August -- and last() would page you for each one.
# ===========================================================================
group(F_MQTT, "environment", 5 * MIN, [
    rule(
        "iot-freeze-risk", "Freeze risk", F_MQTT, "environment",
        [query("A", DS_MQTT,
               f'SELECT median("temperature") FROM "dht22" WHERE $timeFilter '
               f'AND "location" =~ /{rx(DHT22_FREEZE_RISK)}/ GROUP BY time(15m), "location"',
               1 * HOUR),
         reduce("B", "A", "last"), threshold("C", "B", "lt", 3)],
        "C", "30m", "warning",
        summary='{{ $labels.location }}: {{ printf "%.1f" $values.B.Value }} C - freeze risk',
        description=(
            "Location:  {{ $labels.location }}\n"
            'Measured:  {{ printf "%.1f" $values.B.Value }} C (15-minute median)\n'
            "Threshold: below 3 C sustained for 30m\n\n"
            "Action: check pipes and anything stored that can freeze or burst."
        ),
        dashboard=DASH_ROOM,
    ),
    rule(
        "iot-humidity-mould", "High humidity - mould risk", F_MQTT, "environment",
        [query("A", DS_MQTT,
               f'SELECT mean("humidity") FROM "dht22" WHERE $timeFilter '
               f'AND "location" =~ /{rx(DHT22_INDOOR)}/ GROUP BY time(1h), "location"',
               12 * HOUR),
         reduce("B", "A", "mean"), threshold("C", "B", "gt", 75)],
        "C", "6h", "info",
        summary='{{ $labels.location }}: humidity {{ printf "%.0f" $values.B.Value }}% for 12h - mould risk',
        description=(
            "Room:      {{ $labels.location }}\n"
            'Measured:  {{ printf "%.0f" $values.B.Value }}% RH (12-hour mean)\n'
            "Threshold: above 75% sustained for 6h\n\n"
            "The window is deliberately slow so ordinary shower spikes in the two\n"
            "bathrooms do not trigger it -- only genuinely persistent damp does.\n"
            "Action: ventilate or dehumidify.\n\n"
            "Tuning note: dining_room and office sat at 73-74% when this rule was\n"
            "written, so they are close to the line. Raise the threshold if it nags."
        ),
        dashboard=DASH_ROOM,
    ),
    rule(
        "iot-room-cold", "Room too cold (heating fault)", F_MQTT, "environment",
        [query("A", DS_MQTT,
               f'SELECT median("temperature") FROM "dht22" WHERE $timeFilter '
               f'AND "location" =~ /{rx(DHT22_INDOOR)}/ GROUP BY time(15m), "location"',
               2 * HOUR),
         reduce("B", "A", "last"), threshold("C", "B", "lt", 16)],
        "C", "2h", "warning",
        summary='{{ $labels.location }}: {{ printf "%.1f" $values.B.Value }} C - heating may have failed',
        description=(
            "Room:      {{ $labels.location }}\n"
            'Measured:  {{ printf "%.1f" $values.B.Value }} C (15-minute median)\n'
            "Threshold: below 16 C sustained for 2h\n\n"
            "A heated room holding under 16 C for two hours usually means the\n"
            "heating has stopped or a window was left open.\n"
            "Action: check the boiler and any open windows."
        ),
        dashboard=DASH_ROOM,
    ),
])

# ===========================================================================
# MQTT / sensor-health -- is the sensor telling the truth?
# ===========================================================================
group(F_MQTT, "sensor-health", 10 * MIN, [
    # The nastiest failure mode: the device still publishes on schedule, so every
    # liveness check stays green, but the value never changes again. Only variance
    # catches it.
    rule(
        "iot-dht22-flatlined", "DHT22 reading flatlined", F_MQTT, "sensor-health",
        # spread() (max-min), not stddev. A thermally stable room legitimately has
        # a tiny stddev -- basement measured 0.035 over 6h and tripped a stddev<0.05
        # test while actually moving 21.9->22.1. Spread asks the sharper question:
        # has the reading moved *at all*? DHT22 resolution is 0.1 C, so a working
        # sensor always spreads >=0.1 (the calmest room, basement, spreads 0.2),
        # while a frozen one spreads exactly 0.0.
        [query("A", DS_MQTT,
               f'SELECT spread("temperature") FROM "dht22" WHERE $timeFilter '
               f'AND "location" =~ /{rx(DHT22_ALL)}/ GROUP BY "location"', 6 * HOUR),
         reduce("B", "A", "last"), threshold("C", "B", "lt", 0.05)],
        "C", "1h", "warning",
        summary="{{ $labels.location }}: DHT22 stuck on one value for 6h",
        description=(
            "Room:      {{ $labels.location }}\n"
            'Measured:  reading moved {{ printf "%.1f" $values.B.Value }} C over 6h\n'
            "Threshold: below 0.05 C of total movement\n\n"
            "The sensor is still publishing, so liveness checks see nothing wrong,\n"
            "but the value has not changed by even one 0.1 C step in six hours.\n"
            "Real rooms always drift; the calmest one here moves 0.2 C.\n"
            "Usually a wedged DHT22 or a stuck one-wire read.\n"
            "Action: power-cycle the node."
        ),
        dashboard=DASH_ROOM,
    ),
    # Counts bad samples rather than alerting on each one -- glitches are normal
    # in ones and twos, a rising rate means the sensor is on its way out.
    # This is what would have flagged living_room's -11.8 C.
    rule(
        "iot-dht22-implausible", "DHT22 implausible readings", F_MQTT, "sensor-health",
        [query("A", DS_MQTT,
               f'SELECT count("temperature") FROM "dht22" WHERE $timeFilter '
               f'AND "location" =~ /{rx(DHT22_INDOOR)}/ '
               f'AND ("temperature" < 5 OR "temperature" > 40) GROUP BY "location"', 6 * HOUR),
         reduce("B", "A", "last"), threshold("C", "B", "gt", 3)],
        "C", "1h", "info",
        summary='{{ $labels.location }}: {{ printf "%.0f" $values.B.Value }} impossible temperature readings in 6h',
        description=(
            "Room:      {{ $labels.location }}\n"
            'Measured:  {{ printf "%.0f" $values.B.Value }} samples outside 5-40 C in 6h\n'
            "Threshold: more than 3\n\n"
            "A heated room cannot really be below 5 C or above 40 C, so these are\n"
            "bad reads. An occasional one is normal DHT22 behaviour; a cluster means\n"
            "the sensor or its wiring is degrading. Every other rule reduces with a\n"
            "median specifically so these cannot cause false alarms.\n"
            "Action: reseat the wiring, then replace the sensor if it continues."
        ),
        dashboard=DASH_ROOM,
    ),
    rule(
        "iot-dht22-humidity-pegged", "DHT22 humidity pegged at rail", F_MQTT, "sensor-health",
        [query("A", DS_MQTT,
               f'SELECT count("humidity") FROM "dht22" WHERE $timeFilter '
               f'AND "location" =~ /{rx(DHT22_INDOOR)}/ '
               f'AND ("humidity" > 99.8 OR "humidity" < 1) GROUP BY "location"', 6 * HOUR),
         reduce("B", "A", "last"), threshold("C", "B", "gt", 10)],
        "C", "1h", "info",
        summary='{{ $labels.location }}: humidity pinned at sensor rail {{ printf "%.0f" $values.B.Value }} times in 6h',
        description=(
            "Room:      {{ $labels.location }}\n"
            'Measured:  {{ printf "%.0f" $values.B.Value }} samples at >99.8% or <1% RH in 6h\n'
            "Threshold: more than 10\n\n"
            "99.9% is the DHT22 saturation ceiling -- readings stuck there are the\n"
            "sensor giving up rather than a genuinely saturated room.\n"
            "Scoped to heated rooms on purpose: backyard_shed legitimately sits at\n"
            "94.9-99.9% and would otherwise alert permanently.\n"
            "Action: dry or replace the sensor."
        ),
        dashboard=DASH_ROOM,
    ),
    # Shipped paused. Three of the four reporting nodes read -90 to -94 dBm while
    # streaming perfectly reliably, which is self-contradictory: at a genuine -94
    # they would be dropping packets constantly. The RSSI figures are more likely
    # miscalibrated than the link genuinely marginal. Validate, then unpause.
    rule(
        "iot-wifi-marginal", "Wi-Fi link marginal", F_MQTT, "sensor-health",
        [query("A", DS_MQTT,
               f'SELECT median("rssi") FROM "wifi" WHERE $timeFilter '
               f'AND "location" =~ /{rx(WIFI_LOCATIONS)}/ GROUP BY time(15m), "location"',
               1 * HOUR),
         reduce("B", "A", "last"), threshold("C", "B", "lt", -85)],
        "C", "30m", "info",
        summary='{{ $labels.location }}: Wi-Fi {{ printf "%.0f" $values.B.Value }} dBm',
        description=(
            "Location:  {{ $labels.location }}\n"
            'Measured:  {{ printf "%.0f" $values.B.Value }} dBm (15-minute median)\n'
            "Threshold: below -85 dBm sustained for 30m\n\n"
            "PAUSED ON PURPOSE. At the time of writing bathroom_tetris, dining_room\n"
            "and office all reported -90 to -94 dBm while delivering data without\n"
            "loss, which does not add up -- a real -94 dBm link would be dropping\n"
            "packets constantly. Confirm whether these RSSI values are trustworthy\n"
            "before unpausing, otherwise this fires permanently on all three."
        ),
        paused=True,
    ),
])

# ===========================================================================
# Host monitoring / host-liveness
# ===========================================================================
group(F_HOST, "host-liveness", 1 * MIN, [
    # rohan died on 2026-06-23 and nothing noticed for over two months.
    # This is the rule that would have caught it.
    silence_rule(
        "host-down", "Host not reporting", F_HOST, "host-liveness", DS_TELEGRAF,
        f'SELECT count("uptime") FROM "system" WHERE $timeFilter '
        f'AND "host" =~ /{rx(HOSTS)}/ GROUP BY time(2m), "host" fill(0)',
        6 * HOUR, "6m", "critical",
        what="Host {{ $labels.host }}",
        unit_hint="Host: {{ $labels.host }}",
        no_data="Alerting", dashboard=DASH_NODES,
    ),
    rule(
        "host-container-down", "Container not running", F_HOST, "host-liveness",
        [query("A", DS_TELEGRAF,
               'SELECT count("started_at") FROM "docker_container_status" WHERE $timeFilter AND ('
               + " OR ".join(
                   f"""(\"host\" = '{h}' AND \"container_name\" =~ /{rx(cs)}/)"""
                   for h, cs in sorted(EXPECTED_CONTAINERS.items()))
               + ') GROUP BY time(5m), "host", "container_name" fill(0)', 24 * HOUR),
         reduce("B", "A", "last"), threshold("C", "B", "lt", 1)],
        "C", "10m", "critical",
        summary="{{ $labels.host }}: container {{ $labels.container_name }} is not running",
        description=(
            "Host:      {{ $labels.host }}\n"
            "Container: {{ $labels.container_name }}\n"
            "Telegraf stopped reporting this container, which means it is no longer up.\n\n"
            "The expected-container list is pinned per host in rosters.py, because\n"
            "telegraf has also reported 18 stale one-off container names that would\n"
            "otherwise each become a phantom alert.\n\n"
            "Action: ssh {{ $labels.host }} then\n"
            "  docker ps -a --filter name={{ $labels.container_name }}\n"
            "  docker logs --tail 50 {{ $labels.container_name }}"
        ),
        domain="host", dashboard=DASH_NODES,
    ),
])

# ===========================================================================
# Host monitoring / host-resources
# ===========================================================================
def host_resource(uid, title, measurement, field, op, limit, for_, severity,
                  summary, description, window=1 * HOUR):
    return rule(
        uid, title, F_HOST, "host-resources",
        [query("A", DS_TELEGRAF,
               f'SELECT mean("{field}") FROM "{measurement}" WHERE $timeFilter '
               f'AND "host" =~ /{rx(HOSTS)}/ GROUP BY "host"', window),
         reduce("B", "A", "last"), threshold("C", "B", op, limit)],
        "C", for_, severity, summary, description,
        domain="host", dashboard=DASH_NODES,
    )


# cpu_temperature is stored in milli-degrees (57939 == 57.9 C), so the Math node
# is load-bearing: without it the rule compares 57939 against 75 and fires forever.
def cpu_temp(uid, title, limit, severity, for_="10m"):
    return rule(
        uid, title, F_HOST, "host-resources",
        [query("A", DS_TELEGRAF,
               f'SELECT mean("value") FROM "cpu_temperature" WHERE $timeFilter '
               f'AND "host" =~ /{rx(HOSTS)}/ GROUP BY "host"', 10 * MIN),
         reduce("B", "A", "last"), math("C", "$B / 1000"),
         threshold("D", "C", "gt", limit)],
        "D", for_, severity,
        summary='{{ $labels.host }}: CPU {{ printf "%.1f" $values.C.Value }} C (limit '
                + str(limit) + " C)",
        description=(
            "Host:      {{ $labels.host }}\n"
            'Measured:  {{ printf "%.1f" $values.C.Value }} C (10-minute mean)\n'
            f"Threshold: above {limit} C sustained for {for_}\n\n"
            "Raspberry Pis begin thermal throttling around 80-85 C.\n"
            "Action: check ventilation, dust in the heatsink, and enclosure airflow.\n\n"
            "Note: telegraf reports this field in milli-degrees, so the rule divides\n"
            "by 1000 before comparing. Keep that Math step if you edit this."
        ),
        domain="host", dashboard=DASH_NODES,
    )


group(F_HOST, "host-resources", 5 * MIN, [
    # lothlorien was at 99.96% when this was written -- expect this to fire
    # immediately on first evaluation. That is a true positive, not mistuning.
    host_resource(
        "host-swap-full", "Swap nearly full", "swap", "used_percent", "gt", 80, "30m", "warning",
        '{{ $labels.host }}: swap {{ printf "%.1f" $values.B.Value }}% used (limit 80%)',
        "Host:      {{ $labels.host }}\n"
        'Measured:  {{ printf "%.1f" $values.B.Value }}% of swap in use (1-hour mean)\n'
        "Threshold: above 80% sustained for 30m\n\n"
        "On an SD-card Pi, sustained swapping is both a symptom of memory pressure\n"
        "and a cause of card wear.\n"
        "Action: find the consumer with `docker stats` / `ps aux --sort=-%mem`,\n"
        "and clear out stale exited containers with `docker container prune`.",
    ),
    host_resource(
        "host-mem-high", "Memory nearly exhausted", "mem", "used_percent", "gt", 90, "30m", "warning",
        '{{ $labels.host }}: memory {{ printf "%.1f" $values.B.Value }}% used (limit 90%)',
        "Host:      {{ $labels.host }}\n"
        'Measured:  {{ printf "%.1f" $values.B.Value }}% RAM in use (1-hour mean)\n'
        "Threshold: above 90% sustained for 30m\n\n"
        "ithilien only has 1.8 GB, so it reaches this first.\n"
        "Action: `docker stats` to find the consumer.",
    ),
    cpu_temp("host-cpu-temp-high", "CPU temperature high", 75, "warning"),
    cpu_temp("host-cpu-temp-critical", "CPU temperature critical", 82, "critical", for_="5m"),
    # Replaces the old rule that fired at 60%, which was below any real risk
    # (hosts sat at 29-48%) and so trained you to ignore it.
    rule(
        "host-rootfs-high", "RootFS filling", F_HOST, "host-resources",
        [query("A", DS_TELEGRAF,
               f'SELECT mean("used_percent") FROM "disk" WHERE $timeFilter '
               f"AND \"path\" = '/' AND \"host\" =~ /{rx(HOSTS)}/ GROUP BY \"host\"", 1 * HOUR),
         reduce("B", "A", "last"), threshold("C", "B", "gt", 75)],
        "C", "1h", "warning",
        summary='{{ $labels.host }}: rootfs {{ printf "%.1f" $values.B.Value }}% full (limit 75%)',
        description=(
            "Host:      {{ $labels.host }}\n"
            'Measured:  {{ printf "%.1f" $values.B.Value }}% of / in use\n'
            "Threshold: above 75% sustained for 1h\n\n"
            "Action: `docker system prune`, then check /var/log and old journals."
        ),
        domain="host", dashboard=DASH_NODES,
    ),
    rule(
        "host-rootfs-critical", "RootFS critically full", F_HOST, "host-resources",
        [query("A", DS_TELEGRAF,
               f'SELECT mean("used_percent") FROM "disk" WHERE $timeFilter '
               f"AND \"path\" = '/' AND \"host\" =~ /{rx(HOSTS)}/ GROUP BY \"host\"", 1 * HOUR),
         reduce("B", "A", "last"), threshold("C", "B", "gt", 90)],
        "C", "15m", "critical",
        summary='{{ $labels.host }}: rootfs {{ printf "%.1f" $values.B.Value }}% full - imminent',
        description=(
            "Host:      {{ $labels.host }}\n"
            'Measured:  {{ printf "%.1f" $values.B.Value }}% of / in use\n'
            "Threshold: above 90% sustained for 15m\n\n"
            "At 100% the container runtime and InfluxDB writes start failing.\n"
            "Action now: `docker system prune -a`, clear journals with\n"
            "`journalctl --vacuum-size=100M`."
        ),
        domain="host", dashboard=DASH_NODES,
    ),
])


# ===========================================================================
# Notification routing
# ===========================================================================
RECEIVER = "Grafana alert"

# group_by [alertname, severity] is the load-bearing part: when the MQTT bridge
# dies, all 14 "DHT22 sensor silent" instances share an alertname and arrive as
# ONE notification listing every affected room, instead of 14 separate emails.
POLICY = {
    "receiver": RECEIVER,
    "group_by": ["alertname", "severity"],
    "group_wait": "1m",
    "group_interval": "5m",
    "repeat_interval": "12h",
    "routes": [
        {"receiver": RECEIVER, "object_matchers": [["severity", "=", "critical"]],
         "group_by": ["alertname", "severity"],
         "group_wait": "10s", "group_interval": "5m", "repeat_interval": "4h"},
        {"receiver": RECEIVER, "object_matchers": [["severity", "=", "info"]],
         "group_by": ["alertname", "severity"],
         "group_wait": "10m", "group_interval": "30m", "repeat_interval": "24h",
         "mute_time_intervals": ["night"]},
    ],
}

# Mould risk and glitch counters are not 03:00 problems. Only severity=info is
# routed through this; critical and warning always get through.
MUTE_TIMINGS = [{
    "name": "night",
    "time_intervals": [{"times": [{"start_time": "23:00", "end_time": "24:00"},
                                  {"start_time": "00:00", "end_time": "07:00"}]}],
}]


def main():
    json.dump({"groups": GROUPS, "policy": POLICY, "mute_timings": MUTE_TIMINGS},
              sys.stdout, indent=2)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
