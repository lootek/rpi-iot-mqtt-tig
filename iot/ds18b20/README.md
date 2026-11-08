# ds18b20 — 1-Wire temperatures to MQTT

Publishes `/sensors/<location>/floor_heating/<zone>` (°C, every 30 s, not retained).
Currently on `moria` (`boiler_room`): zones `main_in` and `main_out`.

## Wiring (Pi 3.3 V, never 5 V on GPIO-connected sensors)
VDD → 3V3 (pin 1), GND → pin 6, DATA → GPIO4 (pin 7), 4.7 kΩ between DATA and 3V3.
Both sensors share the one data wire. Schematic: see the ups-monitoring session notes (moria-wiring.svg).

## Install
```sh
./install.sh          # as root; adds dtoverlay=w1-gpio, reboot once
ls /sys/bus/w1/devices        # after wiring: 28-xxxxxxxxxxxx per sensor
```
Map sensor ids to zones in `/etc/ds18b20.json` (warm one sensor with your fingers to tell them apart):
```json
{"28-0123456789ab": "main_in", "28-ba9876543210": "main_out"}
```
Unmapped sensors are logged (`journalctl -u ds18b20`) but not published.
Bad reads (CRC error, the 85.0 °C power-on value, -127) are skipped.
