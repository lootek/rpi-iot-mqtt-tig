#!/usr/bin/python3

from time import sleep

import Adafruit_DHT
from paho.mqtt import client as mqtt_client

broker = "mosquitto"
port = 1883
client_id = "ithilien-python-dht11"


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code {}\n".format(rc))

    def on_disconnect(client, userdata, rc):
        client.connect(broker, port, keepalive=60)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set("admin", "password")
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
        print("Failed to send message to topic {}".format(topic))


def measure():
    while True:
        humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, 4)
        print("Temp: {0:0.1f} C  Humidity: {1:0.1f} %".format(temperature, humidity))
        publish(client, "/sensors/living_room/dht11/temperature", temperature)
        publish(client, "/sensors/living_room/dht11/humidity", humidity)

        sleep(60)


if __name__ == "__main__":
    client = connect_mqtt()
    client.loop_start()

    while True:
        try:
            measure()
        except Exception as error:
            print("Exception: ", error)
        finally:
            print("Retrying in 5 min")
            sleep(300)
            # continue


# import RPi.GPIO as GPIO

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(4, GPIO.IN)

# while True:
#  print(GPIO.input(4))
#  sleep(1)
