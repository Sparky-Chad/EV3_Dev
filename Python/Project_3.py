#!/usr/bin/env micropython
# Project 3 Code for the ev3 robot in python

# Created by Chad Lape

# Will need certain ports and such

# Touch sensor will end the class
from ev3dev2.sensor.lego import TouchSensor
# Import motor object
import ev3dev2.motor
# Import sound
from ev3dev2.sound import Sound


# Create instances of the Sensor and Motor
motor = ev3dev2.motor.LargeMotor('outA')    # Motor on Port A
touch = TouchSensor('in1')                  # Sensor on Port 1
sound_play = Sound()

speed_value = ev3dev2.motor.SpeedPercent(-1.8)# Speed set at 10%


# Start Motor at Speed
motor.on(speed_value)
# Wait for touch sensor
touch.wait_for_pressed()
# Stop Motor and Play sound
motor.off()
sound_play.beep()

# Reset Motor Position
motor.on_for_degrees(10, 200)

