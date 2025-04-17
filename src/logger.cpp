#include "logger.h"

int digitalRead(int pin) {
    std::cout << "[MOCK] digitalRead(" << pin << ")" << std::endl;
    return LOW;
}

void pinMode(int pin, int mode) {
    std::cout << "[MOCK] pinMode(" << pin << ", " << mode << ")" << std::endl;
}

void digitalWrite(int pin, int value) {
    std::cout << "[MOCK] digitalWrite(" << pin << ", " << value << ")" << std::endl;
}

void pwmWrite(int pin, int value) {
    std::cout << "[MOCK] pwmWrite(" << pin << ", " << value << ")" << std::endl;
}

void usleep(int ms) {
    std::cout << "[MOCK] usleep(" << ms << ")" << std::endl;   
}

void wiringPiSetupGpio() {
    std::cout << "[MOCK] wiringPiSetupGpio()" << std::endl;
}
