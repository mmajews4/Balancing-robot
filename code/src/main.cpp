#include <Arduino.h>

// ---   T E S T S   ---
//#include "tests/bluetooth_test.h" - passed
//#include "tests/mpu6050_test.h" - passed
#include "tests/motors_test.h"

/*
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;
const int ENA = 10;
const int ENB = 5;

const int LED_PIN = 13; // For status check 

//---------------- P I D ---------

// dobrać według symulacji
float vP = 80;     
float vI = 0.005;
float vD = 300;
float targetValue = 0;
float xP, xI, xD, currentValue, integralSum, currError, lastError;
float throttle, throttleA, throttleB; 
float duration;

int turn = 0;
int watchdog = 0;

void setup() {

}

void loop() {
// -----------   MPU5060 MEASUREMENT AND KALMAN FILTER   ------


// ----------------------------------   P I D   ---------------
    currentValue = angle_roll_output;
  
    xP = (targetValue - currentValue)* vP;
    integralSum += (targetValue - currentValue);
    xI = integralSum * vI;
    currError = (targetValue - currentValue);
    xD = (currError - lastError)* vD;
    lastError = currError;

    throttle = xP + xI + xD;

    if(throttle < -255) throttle = -255;
    if(throttle > 255) throttle = 255;

    throttleA = (throttle + turn)*0.9;
    if(throttleA < -255) throttleA = -255;
    if(throttleA > 255) throttleA = 255;

// -----------------------   M O T O R   C O N T R O L L   -----



// MPU5060 sampling frequency
    while(...){
        watchdog++;
    }
    watchdog = 0;
}
*/