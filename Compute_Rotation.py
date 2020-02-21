# Compute the distance traveled in one rotation of the robot

from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D, SpeedPercent


move = MoveTank(OUTPUT_A, OUTPUT_D)

speed = SpeedPercent(20)
move.on_for_rotations(speed, speed, 1)