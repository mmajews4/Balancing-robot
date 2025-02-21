#include "PID.h"

PID::PID(){
    targetValue = 0;
    currentValue = 0;
    integralSum = 0;
    currError = 0;
    lastError = 0;
}

float PID::calculate(){

    xP = (targetValue - currentValue)* vP;
    integralSum += (targetValue - currentValue);
    xI = integralSum * vI;
    currError = (targetValue - currentValue);
    xD = (currError - lastError)* vD;
    lastError = currError;

    return xP + xI + xD;
}

void PID::reset(){
    integralSum = 0;
    currError = 0;
    lastError = 0;
}