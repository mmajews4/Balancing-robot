#ifndef PID_H
#define PID_H

class PID
{
public:
    float vP;     
    float vI;
    float vD;
    float targetValue;
    float xP, xI, xD, currentValue, integralSum, currError, lastError;
    float duration;

    PID();
    float calculate();
    void reset();
};

#endif