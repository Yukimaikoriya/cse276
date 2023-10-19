#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Copyright 2023, UC San Diego, Contextual Robotics Institute

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

from megapi import MegaPi
import time


MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left


class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.verbose = verbose
        if verbose:
            self.printConfiguration()
        self.bot = MegaPi()
        self.bot.start(port=port)
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left   

    
    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports: MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) + 
              " MBR: " + repr(MBR) + 
              " MFL: " + repr(MFL))


    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) + 
                  " vfr: " + repr(int(round(vfr,0))) +
                  " vbl: " + repr(int(round(vbl,0))) +
                  " vbr: " + repr(int(round(vbr,0))))
        self.bot.motorRun(self.mfl,vfl)
        self.bot.motorRun(self.mfr,vfr)
        self.bot.motorRun(self.mbl,vbl)
        self.bot.motorRun(self.mbr,vbr)


    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(-speed, speed, -speed, speed)


    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
        self.setFourMotors(speed, speed, speed, speed)


    def carSlide(self, speed):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(speed, speed, -speed, -speed)

    
    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )
    
    def close(self):
        self.bot.close()
        self.bot.exit()

    def wait(self, t=1):
        time.sleep(t)
    def move_fb(self, dist):
        d2t = 0.0001
        spd = 30
        if dist < 0:
            dist = -dist
            spd = -spd
        self.carStraight(spd)
        self.wait(d2t * dist)
        self.carStop()
    def rotate(self, angle):
        a2t = 0.02
        spd = 30
        if angle < 0:
            spd = -spd
            angle = -angle
        self.carRotate(spd)
        self.wait(a2t * angle)
        self.carStop()
    '''
    4
    v
    |  ^
    3>-2&5
       |  ↘
       1>-06>
    '''
    def execute(self):
        #0
        self.move_fb(-10000)
        self.wait() #1
        self.rotate(-90)
        self.move_fb(10000)
        self.wait() #2
        self.rotate(90)
        self.move_fb(-10000)
        self.wait()#3
        self.rotate(90)
        self.move_fb(-10000)
        self.wait() #4
        self.rotate(-45)
        self.move_fb(14142)
        self.wait()#5
        self.move_fb(14142)
        self.rotate(45) #6

    def test(self):
        self.move_fb(10000)
        self.wait()
        self.move_fb(-10000)
        self.wait()
        self.rotate(90)
        self.wait()
        self.rotate(-90)


if __name__ == "__main__":
    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)
    mpi_ctrl.test()
    # print("If your program cannot be closed properly, check updated instructions in google doc.")