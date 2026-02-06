import datetime
import logging
import os
import re
import time

import paho.mqtt.client as mqtt
import requests
from influxdb import InfluxDBClient

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

influx_client_mqtt = InfluxDBClient("192.168.10.18", 8086, database="mqtt")

AQICN_MEASUREMENTS_KEYS_MAP = {
    "sps30.mc_2p5": "pm25",
    "sps30.mc_10p0": "pm10",
    "dht22.temperature": "temp",
    "dht22.humidity": "humidity",
}
AQICN_LAST_MEASUREMENT = {}
AQICN_TOKEN = os.getenv("AQICN_TOKEN")
AQICN_STATION_ID = os.getenv("AQICN_STATION_ID")
AQICN_STATION_NAME = os.getenv("AQICN_STATION_NAME")
AQICN_STATION_LATITUDE = os.getenv("AQICN_STATION_LATITUDE")
AQICN_STATION_LONGITUDE = os.getenv("AQICN_STATION_LONGITUDE")
AQICN_API_URL = os.getenv("AQICN_API_URL", "https://aqicn.org/sensor/upload")
AQICN_TIMEOUT = float(os.getenv("AQICN_TIMEOUT", "5"))
AQICN_MIN_UPLOAD_INTERVAL = float(os.getenv("AQICN_MIN_UPLOAD_INTERVAL", "150"))


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


def push_to_aqicn(measurement_key, value):
    if not AQICN_TOKEN or not AQICN_STATION_ID:
        logging.warning(
            "AQICN_TOKEN or AQICN_STATION_ID not set; skipping push to AQICN"
        )
        return

    global AQICN_LAST_MEASUREMENT
    now = time.time()

    if not measurement_key in AQICN_MEASUREMENTS_KEYS_MAP:
        return

    AQICN_LAST_MEASUREMENT[AQICN_MEASUREMENTS_KEYS_MAP[measurement_key]] = value

    readings = []
    for _, aqicn_key in AQICN_MEASUREMENTS_KEYS_MAP.items():
        if not aqicn_key in AQICN_LAST_MEASUREMENT:
            logging.info("Not enough data to push to AQICN, skipping for now...")
            return

        readings.append(
            {"specie": aqicn_key, "value": AQICN_LAST_MEASUREMENT[aqicn_key]}
        )

    payload = {
        "token": AQICN_TOKEN,
        "station": {
            "id": AQICN_STATION_ID,
            "name": AQICN_STATION_NAME,
            "latitude": AQICN_STATION_LATITUDE,
            "longitude": AQICN_STATION_LONGITUDE,
        },
        "readings": readings,
    }

    try:
        last_upload = AQICN_LAST_MEASUREMENT.get("last_upload", 0)
        if now - last_upload < AQICN_MIN_UPLOAD_INTERVAL:
            logging.info("Not enough time has passed since last upload, skipping")
            return

        logging.info("Pushing to AQICN: %s", readings)
        response = requests.post(AQICN_API_URL, json=payload, timeout=AQICN_TIMEOUT)
        response.raise_for_status()
        print(f"AQICN: Data uploaded successfully for {AQICN_STATION_ID}")

        AQICN_LAST_MEASUREMENT = {
            "last_upload": now,
        }
    except requests.RequestException as error:
        logging.error("Failed to push to AQICN: %s", error)


def save_msg(msg):
    payload_raw = msg.payload
    payload_text = (
        payload_raw.decode("utf-8", errors="ignore")
        if isinstance(payload_raw, (bytes, bytearray))
        else str(payload_raw)
    )
    if payload_text.lower() == "nan":
        logging.info("Skipping invalid measurement %s at %s", payload_text, msg.topic)
        return

    current_time = datetime.datetime.now(datetime.timezone.utc).isoformat()

    value = payload_text
    try:
        value = float(payload_text)
    except ValueError:
        logging.info("Couldn't convert %s to float at %s", payload_text, msg.topic)

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
        if measurement_key in AQICN_MEASUREMENTS_KEYS_MAP and isinstance(
            value, (int, float)
        ):
            push_to_aqicn(measurement_key, value)

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
