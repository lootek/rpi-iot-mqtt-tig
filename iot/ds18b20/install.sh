#!/bin/bash
# Install the DS18B20 -> MQTT publisher (run as root from this directory).
# Enables the 1-Wire overlay on GPIO4 (a reboot is needed the first time).
set -euo pipefail
cd "$(dirname "$0")"

apt-get install -y python3-paho-mqtt

CFG=/boot/firmware/config.txt
[ -f "$CFG" ] || CFG=/boot/config.txt
if ! grep -q '^dtoverlay=w1-gpio' "$CFG"; then
  echo "dtoverlay=w1-gpio,gpiopin=4" >> "$CFG"
  echo "1-Wire overlay added to $CFG - reboot required"
fi

[ -f /etc/ds18b20.json ] || echo '{}' > /etc/ds18b20.json   # fill: {"28-xxxxxxxxxxxx": "main_in", ...}

install -m755 ds18b20.py /usr/local/sbin/ds18b20.py
install -m644 ds18b20.service /etc/systemd/system/ds18b20.service
systemctl daemon-reload
systemctl enable ds18b20
systemctl restart ds18b20
systemctl is-active ds18b20 || true
