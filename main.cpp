#include <iostream>
#include <wiringPi.h>

#define RED 1
#define YELLOW 2
#define GREEN 3

#define LED_B 15
#define LED_G 13
#define LED_R 11

#define PYRO_1 16
#define PYRO_2 18
#define PYRO_3 22



void parseLED(ushort state = 0) {
    switch(state) {
        case RED:
            digitalWrite (LED_B, HIGH);
            digitalWrite (LED_G, HIGH);
            digitalWrite (LED_R, LOW);
            break;
        case YELLOW:
            digitalWrite (LED_B, LOW);
            digitalWrite (LED_G, LOW);
            digitalWrite (LED_R, HIGH);
            break;
        case GREEN:
            digitalWrite (LED_B, HIGH);
            digitalWrite (LED_G, LOW);
            digitalWrite (LED_R, HIGH);
            break;
        default:
            digitalWrite (LED_B, HIGH);
            digitalWrite (LED_G, HIGH);
            digitalWrite (LED_R, HIGH);
    }
}

int main() {

    wiringPiSetupGpio();

    // LED Pins
    pinMode(LED_B, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);

    // PYRO channels
    pinMode(PYRO_1, OUTPUT);
    pinMode(PYRO_2, OUTPUT);
    pinMode(PYRO_3, OUTPUT);

    return 0;
}