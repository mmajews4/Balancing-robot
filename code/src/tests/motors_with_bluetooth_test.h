/*#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"

const int STEP_R = 5;
const int DIR_R = 17;
const int STEP_L = 19;
const int DIR_L = 18;
const int MAX_SPEED_DELAY = 420; // 420+780=1200 780(minimal from my throttle equation) in steps/sec corrected by max throttle value
const int SAMPLING_PERIOD = 10000; // micros

int throttle = 0, throttleL = 0, throttleR = 0, turn = 0;
int time_to_next_step = 1000;
int64_t currSampleTime, prevSampleTime, currMotorTime, prevMotorTime;

BluetoothSerial ESP_BT; // Create a BluetoothSerial object
String commandBuffer = ""; // Buffer for incoming command
int run = 0;

void setup() {
    Serial.begin(115200);
    ESP_BT.begin("ESP32_BT"); // Bluetooth device name
    prevSampleTime = micros();
    prevMotorTime = micros();
    pinMode(DIR_R, OUTPUT);
    pinMode(DIR_L, OUTPUT);
    pinMode(STEP_R, OUTPUT);
    pinMode(STEP_L, OUTPUT);
    digitalWrite(DIR_L, LOW);
    digitalWrite(DIR_R, LOW);
    digitalWrite(STEP_L, LOW);
    digitalWrite(STEP_R, LOW);
}

void processBluetoothCommand() {
    while (ESP_BT.available()) {
        char incomingChar = (char)ESP_BT.read();
        if (incomingChar == '\n') { // End of command
            if (commandBuffer == "r") {
                run = 1;
            } else if (commandBuffer == "t") {
                run = 0;
            } else {
                throttle = commandBuffer.toInt();
            }
            commandBuffer = ""; // Clear buffer
        } else {
            commandBuffer += incomingChar; // Append to buffer
        }
    }
}

void motorControlLogic() {
    if (run) {
        if (throttle < -255) throttle = -255;
        if (throttle > 255) throttle = 255;

        throttleL = throttle + turn;
        if (throttleL < -255) throttleL = -255;
        if (throttleL > 255) throttleL = 255;

        throttleR = throttle - turn;
        if (throttleR < -255) throttleR = -255;
        if (throttleR > 255) throttleR = 255;

        digitalWrite(DIR_L, throttleL <= 0 ? HIGH : LOW);
        digitalWrite(DIR_R, throttleR <= 0 ? LOW : HIGH);

        if (throttle != 0) time_to_next_step = MAX_SPEED_DELAY + (200000 / abs(throttle));

        currMotorTime = micros();
        if (currMotorTime - prevMotorTime >= time_to_next_step) {
            digitalWrite(STEP_R, HIGH);
            digitalWrite(STEP_L, HIGH);
            delayMicroseconds(10); // Trigger step
            digitalWrite(STEP_R, LOW);
            digitalWrite(STEP_L, LOW);
            prevMotorTime = currMotorTime;
        }
    }
}

void loop() {
    if (ESP_BT.connected()) {
        processBluetoothCommand();
        motorControlLogic();
    } else {
        Serial.println("Bluetooth not connected. Waiting...");
        delay(1000);
    }
}*/


#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"

const int STEP_R = 5;
const int DIR_R = 17;
const int STEP_L = 19;
const int DIR_L = 18;
const int MAX_SPEED_DELAY = 420; // 420+780=1200 780(mnimal from my throttle equation)in stpes/sec corrected by max throttle value
const int SAMPLING_PERIOD = 10000; // millis

int throttle = 0, throttleL = 0, throttleR = 0, turn = 0;
int time_to_next_step = 1000;
int wait_for_sample = 1;
int64_t currSampleTime, prevSampleTime, currMotorTime, prevMotorTime;

BluetoothSerial ESP_BT; // Create a BluetoothSerial object
String command;
int run = 0;

void setup() {
    Serial.begin(115200);
    ESP_BT.begin("ESP32_BT");       // Bluetooth device name
    prevSampleTime = micros();
    prevMotorTime = micros();
    pinMode(DIR_R, OUTPUT);
    pinMode(DIR_L, OUTPUT);
    pinMode(STEP_R, OUTPUT);
    pinMode(STEP_L, OUTPUT);
    digitalWrite(DIR_L, LOW);
    digitalWrite(DIR_R, LOW);
    digitalWrite(STEP_L, LOW);
    digitalWrite(STEP_R, LOW);
}

void loop(){
    if (ESP_BT.connected()) {
        // Receive data from Bluetooth
        if (ESP_BT.available()){
            command = ESP_BT.readStringUntil('\n');
        }
        if(command == "r"){ //run
            run = 1;
        } else if(command == "t"){ //terminate
            run = 0;
        } else {
            throttle = command.toInt();
        }

        if(run){
            if(throttle < -255) throttle = -255;
            if(throttle > 255) throttle = 255;

            throttleL = throttle + turn;
            if(throttleL < -255) throttleL = -255;
            if(throttleL > 255) throttleL = 255;

            throttleR = throttle - turn;
            if(throttleR < -255) throttleR = -255;
            if(throttleR > 255) throttleR = 255;

            if(throttleL <= 0){
                digitalWrite(DIR_L, HIGH);
            } else {
                digitalWrite(DIR_L, LOW);
            }

            if(throttleR <= 0){
                digitalWrite(DIR_R, LOW);
            } else {
                digitalWrite(DIR_R, HIGH);
            }

            // zrobić wersję jeszcze z użyciem rtosa jak by to nie działo


            // jeżli sybkość samplowania jest szybsza niż szybkość motoru to przechodzić przez tą pętę nawet kila razy i doppiero za którymś puścić silnik
            // żeby nie staneły koła i czały czas updateowała się szykość

            // tak żeby thorttle było liniowe
            if(throttle != 0) time_to_next_step = MAX_SPEED_DELAY + ((200000/abs(throttle))); // dobrać najmniejszą prędkość

            // w trakcie czekania na kolejnego sampla ma kręcić silnikami
            do{
                // czeka aż minie czas do zrobienia kolejnego stepu
                // jeżeli ten czas jest większy niż czas do sampla to wyjdzie z czekania na motor i zrobi sampla
                do{
                    currMotorTime = micros();
                    currSampleTime = currMotorTime;
                    wait_for_sample = (currSampleTime - prevSampleTime < SAMPLING_PERIOD);
                }while(currMotorTime - prevMotorTime < time_to_next_step && wait_for_sample);
                
                // Jeżeli Sample time jest mniejszy niż Motor Delay to ominie motor i zrobi kolejny pomiar
                if(wait_for_sample){
                    digitalWrite(STEP_R, HIGH);
                    digitalWrite(STEP_L, HIGH);
                    delayMicroseconds(10); // według dokumentacji wystraczy 1us impuls żeby zainicjować step, falling edge nic nie robi
                    digitalWrite(STEP_R, LOW);
                    digitalWrite(STEP_L, LOW);
                    prevMotorTime = currMotorTime;
                }
            }while(wait_for_sample);
            prevSampleTime = currSampleTime;
        }
    } else {
        Serial.println("Bluetooth not connected. Waiting...");
        delay(1000);
    }
}
