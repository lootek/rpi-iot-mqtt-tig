#!/usr/bin/python3
"""Publish NUT `upsc` readings to MQTT as /sensors/<location>/ups/<field>.

Read-only: only queries upsd, never sends instant commands to the UPS.
Publishes every PUBLISH_EVERY seconds and immediately when ups.status changes.
Never retained, never publishes empty/NaN values (missing fields are skipped).
"""
import socket
import subprocess
import sys
import time

from paho.mqtt import client as mqtt_client

UPS = "ups"
PORT = 1883
POLL_EVERY = 3
PUBLISH_EVERY = 15

hostname = socket.gethostname()
locations = {
    "lothlorien": "living_room",
    "shire": "attic",
}
brokers = {
    "lothlorien": "127.0.0.1",
}
location = locations.get(hostname, hostname)
broker = brokers.get(hostname, "192.168.10.18")

# upsc variable -> topic field
NUMERIC = {
    "input.voltage": "input_voltage",
    "input.voltage.fault": "input_voltage_fault",
    "output.voltage": "output_voltage",
    "input.frequency": "input_frequency",
    "ups.load": "load_pct",
    "ups.temperature": "temperature",
    "battery.voltage": "battery_voltage",
    "battery.charge": "battery_charge",
    "battery.runtime": "battery_runtime",
}


def read_ups():
    out = subprocess.run(
        ["upsc", UPS], capture_output=True, text=True, timeout=10, check=True
    ).stdout
    data = {}
    for line in out.splitlines():
        k, sep, v = line.partition(": ")
        if sep:
            data[k.strip()] = v.strip()
    return data


def fields(data):
    res = {}
    for var, name in NUMERIC.items():
        try:
            res[name] = float(data[var])
        except (KeyError, ValueError):
            continue
    status = data.get("ups.status", "").strip()
    if status:
        flags = status.split()
        res["status"] = status
        res["on_battery"] = int("OB" in flags)
        res["battery_low"] = int("LB" in flags)
    return res


def connect():
    c = mqtt_client.Client("{}-python-ups".format(hostname))
    c.on_connect = lambda cl, u, f, rc: print("MQTT connect rc={}".format(rc), flush=True)
    c.on_disconnect = lambda cl, u, rc: print("MQTT disconnect rc={}".format(rc), flush=True)
    c.connect_async(broker, PORT, keepalive=60)
    c.loop_start()
    return c


def main():
    print("host={} location={} broker={}".format(hostname, location, broker), flush=True)
    client = connect()
    last_status = None
    last_pub = 0.0
    while True:
        try:
            f = fields(read_ups())
        except (subprocess.SubprocessError, OSError) as e:
            print("upsc failed: {}".format(e), file=sys.stderr, flush=True)
            time.sleep(POLL_EVERY)
            continue
        now = time.monotonic()
        status = f.get("status")
        if f and (now - last_pub >= PUBLISH_EVERY or status != last_status):
            for name, val in f.items():
                client.publish("/sensors/{}/ups/{}".format(location, name), val)
            last_pub, last_status = now, status
        time.sleep(POLL_EVERY)


if __name__ == "__main__":
    main()
