#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time


class SerialInterface():
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1.0):
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


    def write(self, cmd):
        # cmd: byte
        self.client.write(cmd)


    def write_str(self, cmd_str):
        # cmd: string
        cmd = cmd_str.encode()  # str to byte
        self.client.write(cmd)


    def readline(self):
        return self.client.readline()  # byte


    def readline_with_decode(self, code='utf-8', error_option='strict'):
        '''
        arg:
        - code: utf-8, ascii or shift_jis
        - error_option: strict, replace, ignore or backslashreplace'
        '''
        rcv_data = self.client.readline()
        return rcv_data.decode(code, error_option)


    def readline_str(self, error_option='strict'):
        return self.readline_with_decode('utf-8', error_option)  # byte to utf-8(str)


    def readline_ascii(self, error_option='strict'):
        return self.readline_with_decode('ascii', error_option)  # byte to ascii


    def readline_shiftjis(self, error_option='strict'):
        return self.readline_with_decode('shift_jis', error_option)  # byte to shift_jis
