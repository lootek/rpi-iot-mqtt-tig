# ups — NUT → MQTT publisher

Read-only monitoring of USB UPSes (Green Cell Micropower 600VA, `0001:0000 MEC0003`) via NUT.
Nothing here sends shutdown/test/beeper commands to the UPS.

## Install (per host with a USB-attached UPS, run as root)
```sh
apt install -y nut nut-client python3-paho-mqtt
echo MODE=standalone > /etc/nut/nut.conf
echo 'LISTEN 127.0.0.1 3493' > /etc/nut/upsd.conf
truncate -s0 /etc/nut/upsd.users          # read-only: no users, no instcmds
cat > /etc/nut/ups.conf <<CONF
[ups]
  driver = nutdrv_qx
  port = auto
  protocol = q1
  pollfreq = 15
  default.battery.voltage.nominal = 12
  default.battery.voltage.low = 10.5
  default.battery.voltage.high = 13.0
  runtimecal = <t100>,100,<t25>,25
CONF
systemctl enable --now nut-driver nut-server
install -m755 ups-mqtt.py /usr/local/sbin/ups-mqtt.py
install -m644 ups-mqtt.service /etc/systemd/system/
systemctl daemon-reload && systemctl enable --now ups-mqtt
```
Add the host to the `locations` map in `ups-mqtt.py` (lothlorien → `living_room`, shire → `attic`).

## runtimecal
`E_wh = 12 * Ah * 0.8 * 0.85`; `t(load%) = E_wh / (load% * 3.6 W) * 3600` (600VA ≈ 360W);
value `t(100),100,t(25),25` (high-load pair first).

| Host | Ah | runtimecal |
|---|---|---|
| lothlorien | 20 | `1632,100,6528,25` |
| shire (UPS#2) | 26 | `2122,100,8486,25` |

Charge % is a voltage estimate (low/high from ups.conf); runtime is an extrapolation — indicative only.

## MQTT fields — `/sensors/<location>/ups/<field>`
Every 15 s and immediately on status change; not retained.

`input_voltage`, `input_voltage_fault`, `output_voltage`, `input_frequency`, `load_pct`,
`temperature` (if the UPS reports it), `battery_voltage`, `battery_charge`, `battery_runtime` (s),
`status` (string, e.g. `OL`, `OB LB`), `on_battery` (0/1), `battery_low` (0/1).
Use the numeric flags for alerts and graphs.
