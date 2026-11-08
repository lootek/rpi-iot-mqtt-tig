#!/bin/bash
# Install read-only NUT (Megatec q1, USB) + the MQTT publisher on a Debian/Raspberry Pi OS host.
# Usage (as root, from this directory): ./install.sh <battery_Ah>
# Idempotent. Never configures upsmon, users or instant commands (monitoring only).
set -euo pipefail

AH=${1:?usage: $0 <battery_Ah>}
cd "$(dirname "$0")"

# runtimecal: E_wh = 12 * Ah * 0.8 * 0.85 ; t(load%) = E_wh / (load% * 3.6 W) * 3600 (600VA ~ 360W)
read -r T100 T25 < <(awk -v ah="$AH" 'BEGIN { e = 12*ah*0.8*0.85; printf "%d %d\n", e/360*3600+0.5, e/90*3600+0.5 }')

apt-get install -y nut nut-client python3-paho-mqtt

# the package enables a default upsmon (shutdown -h) - not wanted here
systemctl disable --now nut-monitor 2>/dev/null || true

echo "MODE=standalone" > /etc/nut/nut.conf
echo "LISTEN 127.0.0.1 3493" > /etc/nut/upsd.conf
: > /etc/nut/upsd.users
cat > /etc/nut/ups.conf <<EOF
[ups]
  driver = nutdrv_qx
  port = auto
  protocol = q1
  pollfreq = 15
  default.battery.voltage.nominal = 12
  default.battery.voltage.low = 10.5
  default.battery.voltage.high = 13.0
  runtimecal = ${T100},100,${T25},25
EOF

udevadm control --reload
udevadm trigger --subsystem-match=usb

if systemctl list-unit-files 'nut-driver@.service' | grep -q nut-driver@; then
  # NUT >= 2.8: per-UPS driver units
  systemctl enable --now nut-driver-enumerator 2>/dev/null || true
  systemctl enable --now nut-driver@ups
else
  systemctl enable --now nut-driver
fi
systemctl enable nut-server
systemctl restart nut-server

install -m755 ups-mqtt.py /usr/local/sbin/ups-mqtt.py
install -m644 ups-mqtt.service /etc/systemd/system/ups-mqtt.service
systemctl daemon-reload
systemctl enable ups-mqtt
systemctl restart ups-mqtt

sleep 10
upsc ups ups.status
systemctl is-active nut-server ups-mqtt
