#!/usr/bin/python3
"""Publish DS18B20 (1-Wire) temperatures to MQTT as /sensors/<location>/floor_heating/<zone>.

Sensor id -> zone mapping lives in /etc/ds18b20.json, e.g. {"28-0123456789ab": "main_in"}.
Unmapped sensors are logged (so you can find their ids) but not published.
Bad reads (CRC error, the 85.0 power-on default, -127) are skipped, never retained.
"""
import glob
import json
import os
import socket
import sys
import time

from paho.mqtt import client as mqtt_client

PORT = 1883
PUBLISH_EVERY = 30
CONFIG = os.environ.get("DS18B20_CONFIG", "/etc/ds18b20.json")
W1_GLOB = "/sys/bus/w1/devices/28-*"

hostname = socket.gethostname()
locations = {
    "moria": "boiler_room",
}
location = locations.get(hostname, hostname)
broker = os.environ.get("MQTT_BROKER", "192.168.10.18")


def load_zones():
    try:
        with open(CONFIG) as f:
            return json.load(f)
    except (OSError, ValueError) as e:
        print("cannot read {}: {}".format(CONFIG, e), file=sys.stderr, flush=True)
        return {}


def read_temp(dev):
    """Degrees C, or None on a bad read."""
    try:
        with open(os.path.join(dev, "w1_slave")) as f:
            lines = f.read().splitlines()
        if len(lines) < 2 or not lines[0].endswith("YES"):
            return None
        milli = int(lines[1].rsplit("t=", 1)[1])
    except (OSError, ValueError, IndexError):
        return None
    if milli in (85000, -127000):
        return None
    return milli / 1000.0


def connect():
    cid = "{}-python-ds18b20".format(hostname)
    if hasattr(mqtt_client, "CallbackAPIVersion"):  # paho 2.x
        c = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2, cid)
    else:
        c = mqtt_client.Client(cid)
    c.on_connect = lambda *a: print("MQTT connected", flush=True)
    c.on_disconnect = lambda *a: print("MQTT disconnected", flush=True)
    c.connect_async(broker, PORT, keepalive=60)
    c.loop_start()
    return c


def main():
    print("host={} location={} broker={}".format(hostname, location, broker), flush=True)
    client = connect()
    seen = set()
    while True:
        zones = load_zones()
        for dev in sorted(glob.glob(W1_GLOB)):
            sid = os.path.basename(dev)
            if sid not in seen:
                print("found sensor {}".format(sid), flush=True)
                seen.add(sid)
            zone = zones.get(sid)
            temp = read_temp(dev)
            if zone is None or temp is None:
                continue
            client.publish("/sensors/{}/floor_heating/{}".format(location, zone), temp)
        time.sleep(PUBLISH_EVERY)


if __name__ == "__main__":
    main()
