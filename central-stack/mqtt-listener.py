import datetime
import logging
import os
import re

import paho.mqtt.client as mqtt
import requests
from influxdb import InfluxDBClient

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

influx_client_mqtt = InfluxDBClient("192.168.10.18", 8086, database="mqtt")

AQICN_MEASUREMENT_MAP = {
    "sps30.mc_2p5": "pm25",
    "sps30.mc_10p0": "pm10",
}
AQICN_TOKEN = os.getenv("AQICN_TOKEN")
AQICN_STATION = os.getenv("AQICN_STATION")
AQICN_API_URL = os.getenv("AQICN_API_URL", "https://aqicn.org/data-platform/upload")
AQICN_TIMEOUT = float(os.getenv("AQICN_TIMEOUT", "5"))


def extract_sensor_data(path):
    if not path:
        return None

    normalized = path if path.startswith("/") else f"/{path}"

    match = re.match(r"/sensors/([^/]+)/(lastwill)", normalized)
    if match:
        return match.group(1), match.group(2), None

    match = re.match(r"/sensors/([^/]+)/(status)", normalized)
    if match:
        return match.group(1), match.group(2), None

    match = re.match(r"/sensors/([^/]+)/(wifi)/(ip)", normalized)
    if match:
        return match.group(1), match.group(2), match.group(3)

    match = re.match(r"/sensors/([^/]+)/([^/]+)/([^/]+)", normalized)
    if match:
        return match.group(1), match.group(2), match.group(3)

    return None


def push_to_aqicn(measurement_key, value, timestamp_iso):
    if not AQICN_TOKEN or not AQICN_STATION:
        logging.warning("AQICN_TOKEN or AQICN_STATION not set; skipping push to AQICN")
        return

    api_measurement = AQICN_MEASUREMENT_MAP.get(measurement_key)
    if not api_measurement:
        return

    payload = {
        "token": AQICN_TOKEN,
        "station": AQICN_STATION,
        api_measurement: value,
        "time": timestamp_iso,
    }

    try:
        response = requests.post(AQICN_API_URL, json=payload, timeout=AQICN_TIMEOUT)
        response.raise_for_status()
    except requests.RequestException as error:
        logging.error(
            "Failed to push to AQICN: %s (measurement=%s value=%s station=%s)",
            error,
            measurement_key,
            value,
            AQICN_STATION,
        )


def save_msg(msg):
    payload_raw = msg.payload
    payload_text = (
        payload_raw.decode("utf-8", errors="ignore")
        if isinstance(payload_raw, (bytes, bytearray))
        else str(payload_raw)
    )
    if payload_text.lower() == "nan":
        logging.info("Skipping invalid measurement")
        return

    current_time = datetime.datetime.now(datetime.timezone.utc).isoformat()

    value = payload_text
    try:
        value = float(payload_text)
    except ValueError:
        logging.info("Couldn't convert %s to float", payload_text)

    sensor_data = extract_sensor_data(msg.topic)
    if not sensor_data:
        logging.warning("Failed to parse topic %s; dropping message", msg.topic)
        return

    location, sensor, measurement = sensor_data
    if not measurement:
        measurement = sensor
        sensor = "system"

    if location == "arbor":
        location = "backyard_shed"

    if location == "patio":
        measurement_key = f"{sensor}.{measurement}"
        if measurement_key in AQICN_MEASUREMENT_MAP and isinstance(value, (int, float)):
            logging.info("Pushing %s=%s to AQICN", measurement_key, value)
            push_to_aqicn(measurement_key, value, current_time)

    json_body = [
        {
            "measurement": sensor,
            "time": current_time,
            "tags": {
                "location": location,
            },
            "fields": {
                measurement: value,
            },
        }
    ]

    try:
        logging.info("Writing points to InfluxDB: %s", json_body)
        influx_client_mqtt.write_points(json_body)
    except Exception as error:
        logging.error(
            "Failed to write to influxdb: %s (topic=%s payload=%s)",
            error,
            msg.topic,
            payload_text,
        )


def save_msg_wrapper(msg):
    try:
        save_msg(msg)
    except Exception as error:
        logging.error(
            "Unexpected error processing message from topic %s: %s",
            getattr(msg, "topic", "<missing>"),
            error,
        )


client = mqtt.Client()
client.reconnect_delay_set(min_delay=1, max_delay=30)


def on_connect(client, userdata, flags, rc):
    logging.info("Connected to MQTT broker with rc=%s", rc)
    client.subscribe("#")


def on_disconnect(client, userdata, rc):
    logging.warning("Disconnected from MQTT broker with rc=%s", rc)


def on_message(client, userdata, msg):
    save_msg_wrapper(msg)


client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
client.connect("mosquitto", 1883, keepalive=60)
client.loop_forever(retry_first_connection=True)
