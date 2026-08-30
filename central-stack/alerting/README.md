# Alerting for the RPi/IoT fleet

23 Grafana alert rules covering the MQTT sensor estate, the pipeline it depends
on, and host health. Source of truth is `catalog.py`; `apply.py` pushes it to
Grafana on lothlorien.

```sh
cp .grafana.env.example .grafana.env   # paste a service-account token (role: Admin)
./apply.py --dry-run                   # validate, push nothing
./apply.py                             # push
```

## Why not provisioning YAML

Rules dropped into `volumes/grafana/conf/provisioning/alerting/` are locked
read-only in the UI — *"You cannot edit provisioned resources from files in
Grafana"*. Since thresholds here need hand-tuning against a live house, `apply.py`
pushes through the provisioning **HTTP API** with `X-Disable-Provenance: true`,
which leaves every rule editable.

The trade-off: rules do **not** reload when the Grafana container restarts.
They live in `grafana.db`. Re-run `./apply.py` to re-assert git as truth and
overwrite any UI drift.

Two API quirks the applier works around:

- The rule-group endpoint can only *update* rules that already exist. Creating a
  rule with a fixed uid through it fails with `failed to calculate diff ... could
  not find alert rule`, so rules are upserted individually first, then the group
  is PUT purely to set its evaluation interval.
- Rule **titles** must be unique within a folder, so superseded rules have to be
  deleted before their replacements can be created — hence `--prune-legacy` runs
  first.

## The two mechanisms worth understanding

**Absence detection.** A dead sensor's series simply vanishes from an InfluxQL
result, so a naive query has nothing left to evaluate and never fires. The
liveness rules bucket with `fill(0)` over a long window, which keeps the series
present as zeros, then test the most recent bucket:

```sql
SELECT count("temperature") FROM "dht22"
WHERE $timeFilter AND "location" =~ /^(attic|...|patio)$/
GROUP BY time(5m), "location" fill(0)
```

The trailing bucket is always partly elapsed, so a 60-second publisher briefly
reads 0 on every cycle. The `for:` duration absorbs that — a live sensor never
stays at 0 for the full pending period.

Known limit: after more than 24h of silence the series leaves the query window
entirely and the alert resolves itself. *MQTT pipeline down* is the backstop.

**Glitch resistance.** Every threshold reduces a `median` or `mean`, never
`last`. The live data contains real single-sample garbage — `living_room` logged
−11.8 °C and `garage` exactly 0.0 °C in August 2026 — and `last` would page you
for each one. *DHT22 implausible readings* counts those bad samples instead, so
a degrading sensor still gets noticed without every glitch becoming an alert.

## Rules

### MQTT / sensor-liveness (5m)
| rule | fires when | for | severity |
|---|---|---|---|
| DHT22 sensor silent | any of the 14 stops publishing | 15m | critical |
| SCD30 CO2 sensor silent | dining_room or office stops | 15m | warning |
| SPS30 particulate sensor silent | patio stops | 15m | warning |
| MQTT pipeline down | *all* DHT22 data stops at once | 10m | critical |

All 14 sensors share one path — ESP nodes → mosquitto → mqtt-to-influxdb →
influxdb, every hop on lothlorien. *MQTT pipeline down* names that cause directly
instead of leaving you to infer it from fourteen sensor alerts.

### MQTT / air-quality (1m)
| rule | fires when | for | severity |
|---|---|---|---|
| CO2 high | 5-min median > 2000 ppm | 10m | warning |
| CO2 implausibly low (sensor drift) | 6-hour mean < 300 ppm | 2h | warning |
| PM2.5 air pollution | 5-min median > 50 µg/m³ | 15m | warning |
| PM10 air pollution | 5-min median > 100 µg/m³ | 15m | warning |

The low-CO2 rule is a sensor-trust check, not an air-quality one. Outdoor air is
~420 ppm, so an indoor room cannot genuinely average below 300 — it means the
SCD30's automatic self-calibration has drifted, which also makes its *high*
readings untrustworthy. Without it, a drifted sensor reads plausibly-low forever
and the CO2-high rule silently stops protecting the room.

### MQTT / environment (5m)
| rule | fires when | for | severity |
|---|---|---|---|
| Freeze risk | 15-min median < 3 °C in attic/shed/basement/garage/patio | 30m | warning |
| High humidity — mould risk | 12-hour mean > 75% RH indoors | 6h | info |
| Room too cold (heating fault) | 15-min median < 16 °C indoors | 2h | warning |

### MQTT / sensor-health (10m)
| rule | fires when | for | severity |
|---|---|---|---|
| DHT22 reading flatlined | reading has not moved 0.1 °C in 6h | 1h | warning |
| DHT22 implausible readings | >3 samples outside 5–40 °C in 6h | 1h | info |
| DHT22 humidity pegged at rail | >10 samples at >99.8% or <1% RH in 6h | 1h | info |
| Wi-Fi link marginal — **paused** | 15-min median RSSI < −85 dBm | 30m | info |

*Flatlined* catches the nastiest failure mode: the device still publishes on
schedule, so every liveness check stays green, but the value never changes.

*Wi-Fi link marginal* ships paused. bathroom_tetris, dining_room and office all
reported −90 to −94 dBm while delivering data without loss, which does not add
up — a real −94 dBm link would drop packets constantly. Confirm those RSSI values
are trustworthy before unpausing, or it fires permanently on all three.

### Host monitoring / host-liveness (1m)
| rule | fires when | for | severity |
|---|---|---|---|
| Host not reporting | telegraf silent from a pinned host | 6m | critical |
| Container not running | an expected container stops | 10m | critical |

rohan died on 2026-06-23 and nothing noticed for over two months. *Host not
reporting* is the rule that would have caught it.

### Host monitoring / host-resources (5m)
| rule | fires when | for | severity |
|---|---|---|---|
| Swap exhausted with low memory | swap > 80% **and** available RAM < 20% | 30m | warning |
| Memory nearly exhausted | 1-hour mean > 90% | 30m | warning |
| CPU temperature high | > 75 °C | 10m | warning |
| CPU temperature critical | > 82 °C | 5m | critical |
| RootFS filling | > 75% | 1h | warning |
| RootFS critically full | > 90% | 15m | critical |

`cpu_temperature` is stored in **milli-degrees** (`57939` = 57.9 °C), so those two
rules carry a Math step dividing by 1000. Delete it and the rule compares 57939
against 75 and fires forever.

## Rosters are pinned on purpose

InfluxDB keeps tag values forever, so an unpinned query happily alerts on
hardware retired years ago. `rosters.py` pins every list and documents what is
deliberately excluded:

- `arbor` (silent 423d), `backyard-shed` (hyphen typo, 413d), `soil_1`/`soil_2` (426d)
- `rohan` (dead since 2026-06-23), `rivendell`, `rivendell.local` (retired Mac)
- `"bathroom_tetris"` **with literal quote characters** — a second physical device
  is still publishing under a malformed tag, with humidity 91.6–96.5% against the
  real sensor's 70.1–92.3%. The `^(...)$` anchoring keeps it out; the real fix
  belongs in that device's firmware.
- 18 stale one-off container names (`cool_yalow`, `elated_clarke`, …) that would
  each otherwise become a phantom "container down" alert.

## Notification routing

`group_by: [alertname, severity]` is the load-bearing setting. When the MQTT
bridge dies, all 14 *DHT22 sensor silent* instances share an alertname and arrive
as **one** notification listing every affected room, rather than 14 emails.

- default: `group_wait 1m`, `repeat_interval 12h`
- `severity=critical`: `group_wait 10s`, `repeat_interval 4h`
- `severity=info`: `group_wait 10m`, `repeat_interval 24h`, muted 23:00–07:00

Mould risk and glitch counters are not 3 a.m. problems; critical and warning
always get through.

## Tuning notes

- **High humidity — mould risk**: dining_room and office sat at 73–74% when this
  was written, just under the 75% line. Raise it if it nags.
- **Swap exhausted with low memory** requires *both* conditions on purpose.
  A first version tested swap% alone and fired on lothlorien at 99.96% — which
  turned out to be meaningless: Raspberry Pi OS ships a 99 MB dphys swapfile that
  drifts to 100% over months of uptime while RAM stays free (2.3 GB available at
  the time). Swap exhaustion only matters when memory is *also* short, so the rule
  ANDs `swap > 80%` with `available_percent < 20%` via a Math node. That is why it
  has 6 nodes rather than the usual 3.
- **DHT22 reading flatlined** uses `spread()` (max−min), not `stddev`. A stddev
  test at 0.05 flagged `basement` on the first evaluation, but basement was
  genuinely moving 21.9→22.1 °C — it is simply a thermally stable room. Spread
  asks the sharper question, *has the value moved at all*: DHT22 resolution is
  0.1 °C, the calmest room spreads 0.2 °C over 6h, and a frozen sensor spreads
  exactly 0.0. That leaves 4× headroom instead of a coin-flip.
- **RootFS**: replaces a hand-made rule that fired at 60% while hosts sat at
  29–48% — below any real risk, which trained everyone to ignore it.
