#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time


class SerialInterface():
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=1.0):
        self.port =  port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout

        self.client = None
        self.is_open = False


    def __del__(self):
        self.close()


    def open(self):
        if not self.is_open:
            try:
                self.client = serial.Serial(self.port, self.baudrate, self.bytesize, self.parity, self.stopbits, self.timeout)
                self.is_open = True
                time.sleep(0.5)
                print('Connection is established\n')
            except Exception as e:
                print('Connection error\n')


    def is_connected(self):
        return self.is_open


    def close(self):
        if(self.client):
            self.client.close()
