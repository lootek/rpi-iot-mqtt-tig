#!/usr/bin/python
# -*- coding:utf-8 -*-

from os import popen
from time import sleep, time
from datetime import datetime
from math import sqrt
from influxdb import InfluxDBClient
import ADS1263
import RPi.GPIO as GPIO

from paho.mqtt import client as mqtt_client

debug = True

# ADS1263
samples = 200
ref_voltage = 3.3
inputs_count = 10
time_elapsed = 0

# MQTT
port = 1883
broker = "192.168.10.18"
client_id = "rohan-python-sct013"


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


def get_measurement(adc):
    channelList = [i for i in range(inputs_count)]
    raw_data = adc.ADS1263_GetAll(channelList)
    for i in channelList:
        # if raw_data[i] >> 31 == 1:
        #     raw_data[i] = round(
        #         ref_voltage * 2 - raw_data[i] * ref_voltage / 0x80000000, 4
        #     )
        # else:
        raw_data[i] = round(raw_data[i] * ref_voltage / 0x7FFFFFFF, 4)

    #     print("ADC1 IN%d = %lf" % (i, raw_data[i]))

    # for i in channelList:
    #     print("\33[2A")

    return raw_data


def measure(adc):
    while True:
        print("Starting measurement")

        # start timer for kWh calculations
        start_time = time()

        # while 1:
        #     raw_data = get_measurement(adc)

        count = int(0)
        peaks = [0.0 for i in range(inputs_count)]
        IrmsA = [0.0 for i in range(inputs_count)]
        ampsA = [0.0 for i in range(inputs_count)]
        kW = float(0)

        while count < samples:
            count += 1

            raw_data = get_measurement(adc)
            for i in range(0, inputs_count):
                if raw_data[i] > peaks[i]:
                    peaks[i] = raw_data[i]

            # Calibrated for SCT-013 30A/1V
            for i in range(0, inputs_count):
                IrmsA[i] = round(float(peaks[i] / float(2047) * 30), 4)
                ampsA[i] = round(IrmsA[i] / sqrt(2), 4)

        print("raw_data: ", raw_data)
        print("peaks:    ", peaks)
        print("IrmsA:    ", IrmsA)
        print("ampsA:    ", ampsA)

        # # Calculate total AMPS from all sensors and convert to kW
        # kW = 0.0

        # # convert kW to kW / hour (kWh)
        # kWh = round((kW * time_elapsed) / 3600, 8)

        # iso = datetime.now(datetime.timezone.utc).isoformat() + "Z"

        # json_data = [
        #     {
        #         "measurement": "current",
        #         "tags": {},
        #         "time": iso,
        #         "fields": {
        #             "voltage": LINEV,
        #             "kWh": kWh,
        #             "kW_0": kW[0],
        #             "A_0": amps[0],
        #         },
        #     }
        # ]

        # client = InfluxDBClient(
        #     "192.168.10.18", 8086, "", "", "ampread", timeout=60, retries=0
        # )
        # try:
        #     client.write_points(json_data)
        # except ConnectionError:
        #     print("influxdb server not responding")
        #     continue

        # humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, 4)
        # print("Temp: {0:0.1f} C  Humidity: {1:0.1f} %".format(temperature, humidity))
        # publish(client, "/sensors/living_room/dht22/temperature", temperature)
        # publish(client, "/sensors/living_room/dht22/humidity", humidity)

        delay = 300
        if debug:
            delay = 10
            print("Measurement done, sleeping for {}s".format(delay))
        sleep(delay)

        time_elapsed = time() - start_time


if __name__ == "__main__":
    try:
        print("Initializing ADS1263")
        adc = ADS1263.ADS1263()
#        adc.ADS1263_reset()
#        adc.ADS1263_init()

#        chip_id = adc.ADS1263_ReadChipID()
#        print("Chip ID: ", chip_id)

        if adc.ADS1263_init_ADC1("ADS1263_14400SPS") == -1:
            exit()
        adc.ADS1263_SetMode(0)

        print("Running measurements loop")
        while True:
            client = connect_mqtt()
            client.loop_start()

            while True:
                try:
                    measure(adc)
                except IOError as e:
                    print("Exception: ", error)
                except Exception as error:
                    print("Exception: ", error)
                    break

            delay = 300
            if debug:
                delay = 60
            print("Retrying in {}s".format(delay))
            sleep(delay)

    except Exception as error:
        print("Exception: ", error)
        adc.ADS1263_Exit()
        exit()
