#!/usr/bin/python3

from time import sleep
import os
import socket

from paho.mqtt import client as mqtt_client

# BCM pin the sensor data line is on
pin = int(os.environ.get("DHT22_PIN", 4))

try:
    import Adafruit_DHT
except ImportError:
    # bare-metal Pi on a current OS (see ../dht22-bare): read through the pigpio daemon instead
    Adafruit_DHT = None
    import pigpio

port = 1883
broker = "192.168.10.18"

hostname = socket.gethostname()
client_id = "{}-python-dht22".format(hostname)

locations = {
    "ithilien": "attic",
    "lothlorien": "living_room",
    "moria": "boiler_room",
    "rohan": "electrical_box",
    "shire": "attic",
}
location = locations.get(hostname, "living_room")


print("Detected hostname: {}, setting location to: {}".format(hostname, location))


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


def read_pigpio(pi):
    """One DHT22 transaction timed by pigpio (5 us sampling); raises RuntimeError on a bad read."""
    edges = []
    cb = pi.callback(pin, pigpio.EITHER_EDGE, lambda gpio, level, tick: edges.append((level, tick)))
    try:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0)
        sleep(0.02)  # start signal: host pulls low for at least 18 ms
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
        pi.set_mode(pin, pigpio.INPUT)
        sleep(0.1)
    finally:
        cb.cancel()
    highs, rise = [], None
    for level, tick in edges:
        if level == 1:
            rise = tick
        elif level == 0 and rise is not None:
            highs.append((tick - rise) & 0xFFFFFFFF)
            rise = None
    if len(highs) < 40:
        raise RuntimeError("only {} pulses".format(len(highs)))
    bits = "".join("1" if width > 50 else "0" for width in highs[-40:])
    data = [int(bits[i:i + 8], 2) for i in range(0, 40, 8)]
    if sum(data[:4]) & 0xFF != data[4]:
        raise RuntimeError("bad checksum")
    humidity = ((data[0] << 8) | data[1]) / 10.0
    temperature = (((data[2] & 0x7F) << 8) | data[3]) / 10.0
    return humidity, -temperature if data[2] & 0x80 else temperature


def read_dht22():
    """(humidity, temperature), (None, None) when the sensor does not answer."""
    if Adafruit_DHT:
        return Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, pin)
    pi = pigpio.pi()
    try:
        for _ in range(15):
            try:
                return read_pigpio(pi)
            except RuntimeError:  # timing/checksum errors are routine with DHTs
                sleep(2)
    finally:
        pi.stop()
    return None, None


def measure(client):
    while True:
        humidity, temperature = read_dht22()
        print("Temp: {0:0.1f} C  Humidity: {1:0.1f} %".format(temperature, humidity))
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
