#!/usr/bin/python
# -*- coding:utf-8 -*-

import sys
import time
from time import sleep, time
from datetime import datetime
from influxdb import InfluxDBClient
import ADS1263
import config
import RPi.GPIO as GPIO
from paho.mqtt import client as mqtt_client

debug = False

# ADS1263
samples_num = 500
ref_voltage = 2.5
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
            raise RuntimeError("Failed to connect to MQTT, return code {}\n".format(rc))

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


def get_measurement(ads):
    channelList = [i for i in range(inputs_count)]
    raw_data = ads.ADS1263_GetAll(channelList)
    for i in channelList:
        raw_data[i] = round(raw_data[i] * ref_voltage / 0x7FFFFFFF, 6)
        # raw_data[i] = round(raw_data[i] * ref_voltage / 0x7FFFFF, 6)

    #     print("ADC1 IN%d = %lf" % (i, raw_data[i]))

    # for i in channelList:
    #     print("\33[2A")

    return raw_data


def measure(ads):
    while True:
        if debug:
            print("Starting measurement")

        # start timer for kWh calculations
        start_time = time()

        squares_sum = [0.0 for i in range(inputs_count)]
        IrmsA = [0.0 for i in range(inputs_count)]
        kW = float(0)

        count = 0
        while count < samples_num:
            count += 1

            raw_data = get_measurement(ads)
            for i in range(inputs_count):
                squares_sum[i] += raw_data[i] ** 2

        # Calibrated for SCT-013 30A/1V
        for i in range(inputs_count):
            IrmsA[i] = round((squares_sum[i] / samples_num) ** 0.5, 6)

        time_elapsed = time() - start_time

        if debug:
            print(time_elapsed)

        print("IrmsA:\t", IrmsA)

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

        delay = 1

        if debug:
            print("Measurement done, sleeping for {}s".format(delay))

        sleep(delay)


def setup_ads1263():
    print("Initializing ADS1263")
    ads = ADS1263.ADS1263()

    spi_status = config.module_init()
    print("Module init: ", spi_status)

    ads.ADS1263_reset()
    chip_id = ads.ADS1263_ReadChipID()
    print("Chip ID: ", chip_id)

    ads.ADS1263_WriteCmd(ADS1263.ADS1263_CMD["CMD_STOP1"])
    ads.ADS1263_SetMode(0)

    MODE = 0x00  # 0x80:PGA bypassed, 0x00:PGA enabled
    gain = ADS1263.ADS1263_GAIN["ADS1263_GAIN_1"]
    drate = ADS1263.ADS1263_DRATE["ADS1263_38400SPS"]
    MODE |= (gain << 4) | drate
    ads.ADS1263_WriteReg(ADS1263.ADS1263_REG["REG_MODE2"], MODE)

    ads.ADS1263_WriteReg(ADS1263.ADS1263_REG["REG_REFMUX"], 0x00)
    ads.ADS1263_WriteCmd(ADS1263.ADS1263_CMD["CMD_START1"])

    return ads


if __name__ == "__main__":
    try:
        ads = setup_ads1263()

        print("Running measurements loop")
        while True:
            try:
                # client = connect_mqtt()
                # client.loop_start()

                while True:
                    try:
                        measure(ads)
                    except IOError as e:
                        print("Exception: ", error)
                    except Exception as error:
                        print("Exception: ", error)
                        break

            except Exception as error:
                print("Exception: ", error)

            delay = 300
            if debug:
                delay = 60

            print("Retrying in {}s".format(delay))
            sleep(delay)

    except Exception as error:
        print("Exception: ", error)
        ads.ADS1263_Exit()
        exit()
