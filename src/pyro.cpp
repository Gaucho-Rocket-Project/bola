#include <iostream>
#include "wiringPi.h"

void setupPyroChannel(int pinNumber){
    pinMode(pinNumber, OUTPUT);
    
}

void changePyroChannelState(int pinNumber, bool state){
    if(state)
        digitalWrite(pinNumber, ON);
    else
        digitalWrite(pinNumber, OFF);
}