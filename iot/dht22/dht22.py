#!/usr/bin/python3

from time import sleep

import Adafruit_DHT
from paho.mqtt import client as mqtt_client

port = 1883

broker = "192.168.10.18"
client_id = "lothlorien-python-dht22"

#broker = "mosquitto"
#client_id = "ithilien-python-dht22"


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code {}\n".format(rc))

    def on_disconnect(client, userdata, rc):
        client.connect(broker, port, keepalive=60)

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


def measure():
    while True:
        humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, 4)
        print("Temp: {0:0.1f} C  Humidity: {1:0.1f} %".format(temperature, humidity))
        publish(client, "/sensors/living_room/dht22/temperature", temperature)
        publish(client, "/sensors/living_room/dht22/humidity", humidity)

        sleep(60)


if __name__ == "__main__":
    while True:
        client = connect_mqtt()
        client.loop_start()

        while True:
            try:
                measure()
            except Exception as error:
                print("Exception: ", error)
                break

        print("Retrying in 5 min")
        sleep(300)
