#!/bin/bash
# Install the DHT22 -> MQTT publisher on a bare Pi (run as root from this directory).
# Same dht22.py as the container in ../dht22; the sensor is read through the pigpio daemon
# (Adafruit_DHT does not build on current Raspberry Pi OS, and kernel/userspace GPIO polling
# is too slow on a Pi Zero). The Debian pigpiod unit listens on localhost only.
# Data line on GPIO17 (DHT22_PIN in dht22.service).
set -euo pipefail
cd "$(dirname "$0")"

apt-get install -y python3-venv pigpio
systemctl enable --now pigpiod

python3 -m venv /opt/dht22
/opt/dht22/bin/pip install -r requirements.txt

install -m755 ../dht22/dht22.py /opt/dht22/dht22.py
install -m644 dht22.service /etc/systemd/system/dht22.service
systemctl daemon-reload
systemctl enable dht22
systemctl restart dht22
systemctl is-active dht22 || true
