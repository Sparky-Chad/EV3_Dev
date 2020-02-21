#! /usr/bin/env micropython

from Chassis import chassis
from ev3dev2.console import Console
from ev3dev2.button import Button
from time import sleep
from random import randint

chassis = chassis()
"""
lcd = Console()
button = Button()
chassis.chassis.off()
global n
global en
n = 1
en = True
def enter(state):
    global en
    en = False
def right(state):
    if state:
        global n
        if n < 5:
            n = n + 1
    
def left(state):
    if state:
        global n
        if n > 1:
            n = n-1

button.on_enter = enter
button.on_right = right
button.on_left = left
button._state = set([])
while en:
    output = "u = " + str(n)
    lcd.text_at(output, column=0, row=5, reset_console=True)
    lcd.text_at("Num U to Loop:", column=0, row=3)
    button.process()
    sleep(0.02)
"""
n=3
sleep(0.5)
position = []

rand = 80

for i in range(n):

    position.append(chassis.position())
    chassis.move_dif(units=rand)
    sleep(0.05)
    position.append(chassis.position())
    chassis.move_dif(units=rand, backwards=True)
    sleep(0.05)      




f = open("subtask1.log", "a")
out = "----------------------\nNEW\n--------------------\n"
f.write(out)
past = position[0]
for i in position[1:]:
    out = "[{}, {}] --> [{}, {}]\n".format(past[0], past[1], i[0], i[1])
    f.write(out)
    dif = [(i[0] - past[0]), (i[1] - past[1])]
    out = "DIF: [{}, {}]\n".format(dif[0], dif[1])
    f.write(out)
    past = i
out = "--------------------\nEND\n--------------------\n"
f.write(out)
f.close()
    