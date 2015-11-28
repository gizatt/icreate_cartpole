#!/usr/bin/env python
# -*- coding: utf-8 -*-

# RASPI IROBOT BALANCER
# Heavy reference to Create2_TetheredDrive.py, for serial interface
# Extended 20151127ish, gizatt and jizatt

###########################################################################
# Copyright (c) 2015 iRobot Corporation
# http://www.irobot.com/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
#   Neither the name of iRobot Corporation nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###########################################################################

import struct
import sys, glob # for listing serial ports
import time, math, re

import serial

TEXTWIDTH = 40 # window width, in characters
TEXTHEIGHT = 16 # window height, in lines

VELOCITYCHANGE = 500
ROTATIONCHANGE = 300

SINUSOID_PERIOD = 2
SINUSOID_AMPLITUDE = 500

MIDANGLE = 0.5
RANGE = 1.0

K_P = 300

class Balancer():
    lastDriveCommand = ''

    def __init__(self):
        self.performSinusoidTest = False
        self.sinusoid_period = SINUSOID_PERIOD
        self.sinusoid_start = time.time()
        self.sinusoid_amplitude =  SINUSOID_AMPLITUDE
        self.testToPerform = 0
        self.connection = None

    def __del__(self):
        if self.connection is not None:
            self.sendCommandASCII('128')

    def reset(self):
        self.sendCommandASCII('128')
        time.sleep(0.1)
        self.sendCommandASCII('131')
        time.sleep(0.1)
        self.sendCommandASCII('140 3 1 64 16 141 3')
        time.sleep(0.1)
        self.sendCommandASCII('140 3 1 64 16 141 3')
        time.sleep(0.1)
        self.sendCommandASCII('140 3 1 64 16 141 3')


    def update(self, analogVal):
        theta = (analogVal - MIDANGLE)*2.*RANGE
        print "Theta: ", theta
        if self.testToPerform == 1:
            elapsed = (time.time() - self.sinusoid_start)
            velocity = self.sinusoid_amplitude*math.sin(elapsed/self.sinusoid_period*2*3.1415)
            rotation = 0
            self.sendDriveCommand(velocity, rotation)
        elif self.testToPerform == 2:
            err = -theta
            velocity = err * K_P
            rotation = 0
            print "velocity",  velocity
            self.sendDriveCommand(velocity, rotation)

    # sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
    def sendCommandASCII(self, command):
        cmd = ""
        for v in command.split():
            cmd += chr(int(v))

        self.sendCommandRaw(cmd)

    # sendCommandRaw takes a string interpreted as a byte array
    def sendCommandRaw(self, command):
        try:
            if self.connection is not None:
                self.connection.write(command)
            else:
                print "Not connected to the robot!"
        except serial.SerialException:
            print "Lost connection to the robot!"
            self.connection = None

    # getDecodedBytes returns a n-byte value decoded using a format string.
    # Whether it blocks is based on how the connection was set up.
    def getDecodedBytes(self, n, fmt):
        try:
            return struct.unpack(fmt, self.connection.read(n))[0]
        except serial.SerialException:
            print "Lost connection to the robot!"
            self.connection = None
            return None
        except struct.error:
            print "Got unexpected data from serial port."
            return None

    # get8Unsigned returns an 8-bit unsigned value.
    def get8Unsigned(self):
        return getDecodedBytes(1, "B")

    # get8Signed returns an 8-bit signed value.
    def get8Signed(self):
        return getDecodedBytes(1, "b")

    # get16Unsigned returns a 16-bit unsigned value.
    def get16Unsigned(self):
        return getDecodedBytes(2, ">H")

    # get16Signed returns a 16-bit signed value.
    def get16Signed(self):
        return getDecodedBytes(2, ">h")

    def sendDriveCommand(self, velocity, rotation):
        # compute left and right wheel velocities
        vr = velocity + (rotation/2)
        vl = velocity - (rotation/2)

        # create drive command
        cmd = struct.pack(">Bhh", 145, vr, vl)
        if cmd != self.lastDriveCommand:
            self.sendCommandRaw(cmd)
            self.lastDriveCommand = cmd

    def connect(self, port):
        print "Trying " + str(port) + "... "
        try:
            self.connection = serial.Serial(port, baudrate=115200, timeout=1)
            print "Connected!"
        except:
            print "Failed to connect!"


class AnalogIn():
    def __init__(self):
        self.connection = None
        self.analogVal = -1
        self.input_buf = ''

    def update(self):
        if self.connection is not None:
            try:
                self.input_buf += self.connection.read(5)
                if len(self.input_buf) > 0:
                    m = re.findall('[0-9][0-9][0-9][0-9]', self.input_buf)
                    if len(m)>0:
                        self.analogVal = float(m[-1]) / 1023.
                        self.input_buf = ''

            except Exception as e:
                print "Failed in read"
                print e

    def getAnalogOut(self):
        return self.analogVal

    def connect(self, port):
        print "Trying " + str(port) + "... "
        try:
            self.connection = serial.Serial(port, baudrate=115200, timeout=0.01)
            print "Connected!"
        except:
            print "Failed to connect!"

if __name__ == "__main__":
    balancer = Balancer()
    analogIn = AnalogIn()
    analogIn.connect('/dev/ttyACM0')
    balancer.connect('/dev/ttyUSB0')
    balancer.reset()
    if len(sys.argv) > 1:
        balancer.testToPerform = int(sys.argv[1])

    while(1):
        analogIn.update()
        balancer.update(analogIn.getAnalogOut())