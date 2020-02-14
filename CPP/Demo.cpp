
// Demo code for ev3

#include "ev3dev.h"

#include <chrono>
#include <thread>
#include <string>

using namespace std;


int main() {


    ev3dev::device d = ev3dev::device();
    // Create large motor class 
    ev3dev::large_motor motor = ev3dev::large_motor(ev3dev::OUTPUT_A);
    // Create Touch Sensor Object
    ev3dev::touch_sensor touch = ev3dev::touch_sensor(ev3dev::INPUT_1);
    // Create Sound object
    ev3dev::sound sound = ev3dev::sound();

    motor.set_speed_sp(-10);
    motor.run_forever();

    while (!touch.is_pressed()){

        this_thread::sleep_for(chrono::milliseconds(100));

    }

    motor.stop_action();

    sound.beep();



    return 0;
}