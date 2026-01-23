import datetime
import logging
import re
import paho.mqtt.client as mqtt
from influxdb import InfluxDBClient

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

influx_client_mqtt = InfluxDBClient("192.168.10.18", 8086, database="mqtt")


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
    logging.info(json_body)

    try:
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
