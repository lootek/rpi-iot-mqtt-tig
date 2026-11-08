# ups — NUT → MQTT publisher

Read-only monitoring of USB UPSes (Green Cell Micropower 600VA, `0001:0000 MEC0003`) via NUT.
Nothing here sends shutdown/test/beeper commands to the UPS.

## Install (per host with a USB-attached UPS)
```sh
# as root, from this directory; argument = battery capacity in Ah (sets runtimecal)
./install.sh 26
```
Installs NUT (read-only: no upsmon, no users, no instant commands), writes `/etc/nut/*`, deploys
`ups-mqtt.py` + `ups-mqtt.service`. Add the host to the `locations` map in `ups-mqtt.py`
(lothlorien → `living_room`, shire → `attic`, moria → `boiler_room`).

## runtimecal
`E_wh = 12 * Ah * 0.8 * 0.85`; `t(load%) = E_wh / (load% * 3.6 W) * 3600` (600VA ≈ 360W);
value `t(100),100,t(25),25` (high-load pair first).

| Host | Ah | runtimecal |
|---|---|---|
| lothlorien | 20 | `1632,100,6528,25` |
| shire (UPS#2) | 26 | `2122,100,8486,25` |
| moria (UPS#3) | ~9 (confirm) | `734,100,2938,25` |

Charge % is a voltage estimate (low/high from ups.conf); runtime is an extrapolation — indicative only.

## MQTT fields — `/sensors/<location>/ups/<field>`
Every 15 s and immediately on status change; not retained.

`input_voltage`, `input_voltage_fault`, `output_voltage`, `input_frequency`, `load_pct`,
`temperature` (if the UPS reports it), `battery_voltage`, `battery_charge`, `battery_runtime` (s),
`status` (string, e.g. `OL`, `OB LB`), `on_battery` (0/1), `battery_low` (0/1).
Use the numeric flags for alerts and graphs.
