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
samples_num = 1000
adc_rate = "ADS1263_7200SPS"
ref_voltage = 2.5
inputs_count = 10
time_elapsed = 0
ampers_volt_ratio = 30.0
voltage = 230.0
power_factor = 0.9

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


def measure(client, ads):
    while True:
        if debug:
            print("Starting measurement")

        for i in range(inputs_count):
            start_time = time()
            ads.ADS1263_SetChannal(i)
            samples = []

            for _ in range(samples_num):
                ads.ADS1263_WaitDRDY()
                raw = ads.ADS1263_Read_ADC_Data()

                # Correct 32-bit signed integer conversion
                if raw >> 31 == 1:
                    val = raw - 0x100000000
                else:
                    val = raw

                # Convert to Amps
                amps = (val / 0x7FFFFFFF) * ref_voltage * ampers_volt_ratio
                samples.append(amps)

            time_elapsed = time() - start_time

            if len(samples) > 0:
                dc_offset = sum(samples) / len(samples)
                irms = (
                    sum((x - dc_offset) ** 2 for x in samples) / len(samples)
                ) ** 0.5
            else:
                irms = 0.0

            power = (irms * voltage * power_factor) / 1000.0
            energy = (power * time_elapsed) / 3600.0

            print("Ch{}: Irms: {:.2f} A, Power: {:.3f} kW".format(i, irms, power))
            publish(client, "/sensors/sct013/{}/current".format(i), irms)
            publish(client, "/sensors/sct013/{}/power".format(i), power)
            publish(client, "/sensors/sct013/{}/energy".format(i), energy)

        delay = 1

        if debug:
            print("Measurement done, sleeping for {}s".format(delay))

        sleep(delay)


def setup_ads1263():
    print("Initializing ADS1263")
    ads = ADS1263.ADS1263()

    ads.ADS1263_init_ADC1(adc_rate)

    # spi_status = config.module_init()
    # print("Module init: ", spi_status)

    # ads.ADS1263_reset()
    # chip_id = ads.ADS1263_ReadChipID()
    # print("Chip ID: ", chip_id)

    # ads.ADS1263_WriteCmd(ADS1263.ADS1263_CMD["CMD_STOP1"])
    # ads.ADS1263_SetMode(0)

    # MODE = 0x00  # 0x80:PGA bypassed, 0x00:PGA enabled
    # gain = ADS1263.ADS1263_GAIN["ADS1263_GAIN_1"]
    # drate = ADS1263.ADS1263_DRATE[adc_rate]
    # MODE |= (gain << 4) | drate
    # ads.ADS1263_WriteReg(ADS1263.ADS1263_REG["REG_MODE2"], MODE)

    # ads.ADS1263_WriteReg(ADS1263.ADS1263_REG["REG_REFMUX"], 0x00)
    # ads.ADS1263_WriteCmd(ADS1263.ADS1263_CMD["CMD_START1"])

    return ads


if __name__ == "__main__":
    try:
        ads = setup_ads1263()

        print("Running measurements loop")
        while True:
            try:
                client = connect_mqtt()
                client.loop_start()

                while True:
                    try:
                        measure(client, ads)
                    except IOError as error:
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
