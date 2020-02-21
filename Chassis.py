#!usr/bin/env micropython

# Just do a little loopy loops
# Chad Lape

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, MoveTank, SpeedPercent, MoveDifferential
from ev3dev2.wheel import Wheel, EV3EducationSetTire
from time import sleep
from math import pi



# Constants
Right_Motor_Port = OUTPUT_C
Left_Motor_Port = OUTPUT_B

Sprocket_Wheel_Diameter = 4.2 #In CM
Sprocket_Wheel_Thickness = 1.5 #In CM

# Mini class
class sprocket(Wheel):
    def __init__(self):
        Wheel.__init__(self, Sprocket_Wheel_Diameter*10, Sprocket_Wheel_Thickness*10)


# This is the distance between the two moments on the most inner point
Wheel_Well_diameter = 10 + Sprocket_Wheel_Thickness*0.7826# in cm

CONVERSION_TO_CM = Sprocket_Wheel_Diameter * pi
CONVERSION_TO_DEGREE = 10.2

class chassis:



    # Sets up motors
    def __init__(self):
        self.right = LargeMotor(Right_Motor_Port)
        self.left = LargeMotor(Left_Motor_Port)
        self.chassis = MoveTank(Left_Motor_Port, Right_Motor_Port)

        self.rot_conversion = lambda distance: distance / CONVERSION_TO_CM # This will convert from a distance to a
        self.sp_conversion = lambda distance: 360 * self.rot_conversion(distance)
        # theoretical number of rotations
        self.degree_conversion = lambda deg: self.rot_conversion((deg * pi * Wheel_Well_diameter) / 360) # Will convert from degrees into a distance


    # Send move comand but backwards, just for ease if I use it
    def move_backwards(self, units=10, speed=15, backwards=True):
        self.move(units, speed, backwards)

    # Move forward, units being in cm
    def move(self, units=10, speed=15, forwards=False):
        
        # Speed forward if forwards is ture or go back
        speed = speed if forwards else (-speed)
        self.chassis.on_for_rotations(SpeedPercent(speed), SpeedPercent(speed), self.rot_conversion(units),brake=True)
        
        self.chassis.off(brake=False)

    def move_rel(self, units=10, speed=15, forwards=False):

        speed = speed if forwards else(-speed)
        self.chassis.run_to_rel_pos()


    # Move to a relative position forward
    def turn_counter_clockwise(self, degrees=180, speed=10, counter_clockwise=True):
        self.turn_clockwise(degrees, speed, not counter_clockwise)

    def turn_clockwise(self, degrees=180, speed=10, clockwise=True):

        speeds = SpeedPercent(-speed)
        speedb = SpeedPercent(speed)
        self.chassis.on_for_rotations(speedb, speeds, self.degree_conversion(degrees))

        self.chassis.off(brake=False)

    def move_dif(self, speed=15, units=10, backwards=False):

        speed = -speed if backwards else speed

        self.dif = MoveDifferential(Left_Motor_Port, Right_Motor_Port, EV3EducationSetTire, Wheel_Well_diameter*10)

        self.dif.on_for_distance(SpeedPercent(speed), units*10)

        sleep(0.3)

        self.dif.off(brake=False)

    def turn_angle(self, angle=180):
        self.turn = MoveDifferential(Left_Motor_Port, Right_Motor_Port, EV3EducationSetTire, Wheel_Well_diameter*10)
        self.turn.turn_left(15, angle)        

        
    def position(self):

        return [self.right.position, self.left.position]

    def odo_start(self):
        self.dif.odometry_start()
    
    def odo_stop(self):
        self.dif.odometry_stop()

    