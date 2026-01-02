#!/usr/bin/python3

from time import sleep
import socket

import Adafruit_DHT
from paho.mqtt import client as mqtt_client

port = 1883

broker = "192.168.10.18"
hostname = socket.gethostname()
client_id = "{}-python-dht22".format(hostname)

locations = {
    "ithilien": "attic",
    "lothlorien": "living_room",
    "rohan": "electrical_box",
    "shire": "attic",
}


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code {}\n".format(rc))

    def on_disconnect(client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection.")

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.connect(broker, port, keepalive=60)
    return client


def publish(client, topic, msg):
    result = client.publish(topic, msg)
    status = result[0]
    if status == 0:
        print("Sent `{}` to topic `{}`".format(msg, topic))
    else:
        raise RuntimeError("Failed to send message to topic {}".format(topic))


def measure(client):
    while True:
        humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, 4)
        print("Temp: {0:0.1f} C  Humidity: {1:0.1f} %".format(temperature, humidity))
        location = locations.get(hostname, "living_room")
        publish(client, "/sensors/{}/dht22/temperature".format(location), temperature)
        publish(client, "/sensors/{}/dht22/humidity".format(location), humidity)

        sleep(60)


if __name__ == "__main__":
    while True:
        client = connect_mqtt()
        client.loop_start()

        while True:
            try:
                measure(client)
            except Exception as error:
                print("Exception: ", error)
                break

        client.loop_stop()
        client.disconnect()

        print("Retrying in 5 min")
        sleep(300)
