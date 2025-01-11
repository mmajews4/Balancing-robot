#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"

// ---   T E S T S   ---
//#include "tests/bluetooth_test.h" - passed
//#include "tests/mpu6050_test.h" - passed
//#include "tests/motors_test.h" - passed
//#include "tests/motors_with_bluetooth_test.h"// - passed

// -------   M O T O R   C O N T R O L L  ---
const int STEP_R = 5;
const int DIR_R = 17;
const int STEP_L = 19;
const int DIR_L = 18;
const int MAX_ACCELERATION = 20;
const int MAX_SPEED_DELAY = 320; // 420+780=1200 780(mnimal from my throttle equation)in stpes/sec corrected by max throttle value
const int SAMPLING_PERIOD = 20000; // millis
const int SENDING_PERIOD = 10; // one in how many periods i send parameters via bluetooth

int throttle = 0, last_throttle = 0, motor_throttle = 0, throttleL = 0, throttleR = 0, turn = 0;
int time_to_next_step = 1000;
int wait_for_sample = 1;
int64_t currSampleTime, prevSampleTime, currMotorTime, prevMotorTime;
int64_t exec_time = 0;

float steer = 0;
float adjustment = 0;

//------------   B L U E T O O T H   ------ 
BluetoothSerial ESP_BT; // Create a BluetoothSerial object
char incomingChar;
String command = "";
int run = 0;
int sendDelay = SENDING_PERIOD;
char message[100];
void tune();

//-----------------   P I D   --------------
// dobrać według symulacji
float vP = 50;     
float vI = 0.01; //0.005;
float vD = 10;
float targetValue = 0;
float xP, xI, xD, currentValue, integralSum, currError, lastError;
float duration;

// -----------   ANGLE MEASUREMENT   ------
MPU6050 mpu;

float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

float accelAngleX, accelAngleY;
float gyroAngleX, gyroAngleY;
float angleX, angleY;
float elapsedTime, currentTime, previousTime;
int16_t ax, ay, az, gx, gy, gz;
float axCal, ayCal, azCal, gxCal, gyCal, gzCal;

void take_measurement();
void calibrateMPU6050();

void setup() {
    Serial.begin(115200);
    ESP_BT.begin("ESP32_BT");       // Bluetooth device name
    Wire.begin();
    pinMode(DIR_R, OUTPUT);
    pinMode(DIR_L, OUTPUT);
    pinMode(STEP_R, OUTPUT);
    pinMode(STEP_L, OUTPUT);
    digitalWrite(DIR_L, LOW);
    digitalWrite(DIR_R, LOW);
    digitalWrite(STEP_L, LOW);
    digitalWrite(STEP_R, LOW);
        
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    calibrateMPU6050();
    prevSampleTime = micros();
    prevMotorTime = micros();
}

void loop(){
    // -------------------------   B L U E T O O T H   R E C I V E   C O M M A N D S   --- 
    if (ESP_BT.connected()) {
        // Receive data from Bluetooth
        if (ESP_BT.available()){
            incomingChar = ESP_BT.read();  // Read one character

            switch(incomingChar){
                case 'r':   // run
                    run = 1;
                    previousTime = millis(); // to not mess up gyroAngle mesurement
                    gyroAngleX = 0;
                    break;
                case 't':   // terminate
                    run = 0;
                    integralSum = 0;
                    currError = 0;
                    lastError = 0;
                    break;
                case 'q':   // tune
                    run = 0;
                    tune(); // it exists because while driving sending more than one character creates unacceptable delay
                    break;
                default:    // steer value 0/9 -> -0.8/1.0
                    if(incomingChar >= '0' && incomingChar <= '9') // so that non number things doesn't change steer
                        steer =  0.2 * (incomingChar - '0' - 4); //konwersja char do int
                    break;
            }
        }

        if(run){
            // --------------------------------------------   P I D   --------------------
            take_measurement();

            currentValue = angleX + steer + adjustment;
        
            xP = (targetValue - currentValue)* vP;
            integralSum += (targetValue - currentValue);
            xI = integralSum * vI;
            currError = (targetValue - currentValue);
            xD = (currError - lastError)* vD;
            lastError = currError;

            throttle = xP + xI + xD;
            // -------------------------   B L U E T O O T H   S T A T U S   S E N D   ---

            // Sending data to the controller every SENDING_PERIOD
            if(sendDelay == SENDING_PERIOD){
                sprintf(message, "P:%.0f   I:%.1f   D:%.0f\nAngle: %0.3f\nA+Str: %0.3f\nThrottle: %d   Steer: %.1f\nExec_t: %d", \
                    xP, xI, xD, angleX, currentValue, throttle, steer, exec_time);
                ESP_BT.print(message);
                sendDelay = 0;
            }
            sendDelay++;

            // ---------------------------------   M O T O R   C O N T R O L L   ---------

            if(throttle < -255) throttle = -255;
            else if(throttle > 255) throttle = 255;

            
            if(last_throttle - throttle < -MAX_ACCELERATION){
                motor_throttle = last_throttle + MAX_ACCELERATION;
            } else if(last_throttle - throttle > MAX_ACCELERATION){
                motor_throttle = last_throttle - MAX_ACCELERATION;
            } else {
                motor_throttle = throttle;
            }
            last_throttle = motor_throttle;
            
//            motor_throttle = throttle;

            throttleL = motor_throttle + turn;
            if(throttleL < -255) throttleL = -255;
            else if(throttleL > 255) throttleL = 255;

            throttleR = motor_throttle - turn;
            if(throttleR < -255) throttleR = -255;
            else if(throttleR > 255) throttleR = 255;

            if(throttleL < 0){
                digitalWrite(DIR_L, LOW);
            } else {
                digitalWrite(DIR_L, HIGH);
            }

            if(throttleR < 0){
                digitalWrite(DIR_R, HIGH);
            } else {
                digitalWrite(DIR_R, LOW);
            }

            // zrobić wersję jeszcze z użyciem rtosa jak by to nie działo


            // jeżli sybkość samplowania jest szybsza niż szybkość motoru to przechodzić przez tą pętę nawet kila razy i doppiero za którymś puścić silnik
            // żeby nie staneły koła i czały czas updateowała się szykość

            // tak żeby thorttle było liniowe                               200000
            if(motor_throttle != 0) time_to_next_step = MAX_SPEED_DELAY + ((100000/abs(motor_throttle))); // dobrać najmniejszą prędkość

            exec_time = prevSampleTime - micros();

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

// ----------------   T U N E   R O B O T   P A R A M E T E R S   ------------
void tune(){
    ESP_BT.print("Entered tune mode");
    while(!run){
        // Receive data from Bluetooth
        if (ESP_BT.available()) {
            command = ESP_BT.readString(); // Read incoming data
        
            switch(command[0]){
                case 'r':   // run
                    run = 1;
                    previousTime = millis(); // to not mess up gyroAngle mesurement
                    gyroAngleX = 0;
                    break;
                case 't':   // terminate
                    run = 0;
                    integralSum = 0;
                    currError = 0;
                    lastError = 0;
                    break;
                case 'p':
                    command.remove(0, 1); // Remove first character starting at index 0
                    vP = command.toFloat();
                    break;
                case 'i':
                    command.remove(0, 1);
                    vI = command.toFloat();
                    break;
                case 'd':
                    command.remove(0, 1);
                    vD = command.toFloat();
                    break;
                case 'm':
                    command.remove(0, 1);
                    adjustment = command.toFloat();
                    break;
                case 'c':
                    calibrateMPU6050();
                    ESP_BT.print("Calibrated\n");
                    break;
                default:    // steer value 0/9 -> -0.8/1.0
                    steer =  0.2 * (incomingChar - '0' - 4); //konwersja char do int
                    break;
            }
        }
        take_measurement();
        sprintf(message, "P:%.0f   I:%.3f   D:%.0f\nAngle: %0.3f\nA+Str: %0.3f\nAdjst: %0.3f\nGyro: %.3f   Accel: %.3f",\
            vP, vI, vD, angleX, angleX + steer + adjustment, adjustment, gyroAngleX, accelAngleX);
        ESP_BT.print(message);

        delay(200);
    }
}

// -----------------------   A N G L E   M E A S U R E M E N T   -------------
void take_measurement(){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Apply calibration offsets
    axCal = ax - accelOffsetX;
    ayCal = ay - accelOffsetY;
    azCal = az - accelOffsetZ;
    gxCal = gx - gyroOffsetX;
    gyCal = gy - gyroOffsetY;
    gzCal = gz - gyroOffsetZ;

    // Calculate accelerometer angles
    accelAngleX = atan(ayCal / sqrt(pow(axCal, 2) + pow(azCal, 2))) * 180 / PI;
//    accelAngleY = atan(-axCal / sqrt(pow(ayCal, 2) + pow(azCal, 2))) * 180 / PI;

    // Calculate elapsed time
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // Integrate gyroscope data
    gyroAngleX += gxCal / 131.0 * elapsedTime;
//    gyroAngleY += gyCal / 131.0 * elapsedTime;

    // Complementary filter
    angleX = 0.92 * gyroAngleX + 0.08 * accelAngleX;
//    angleY = 0.92 * gyroAngleY + 0.08 * accelAngleY;
}

void calibrateMPU6050() {
  int numReadings = 2000; // Number of samples for calibration

  long accelSumX = 0, accelSumY = 0, accelSumZ = 0;
  long gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;

  for (int i = 0; i < numReadings; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    accelSumX += ax;
    accelSumY += ay;
    accelSumZ += az;
    gyroSumX += gx;
    gyroSumY += gy;
    gyroSumZ += gz;

    delay(2); // Small delay for consistent sampling
  }

  // Calculate offsets
  accelOffsetX = accelSumX / numReadings;
  accelOffsetY = accelSumY / numReadings;
  accelOffsetZ = accelSumZ / numReadings - 16384; // Adjust for 1g on Z-axis
  gyroOffsetX = gyroSumX / numReadings;
  gyroOffsetY = gyroSumY / numReadings;
  gyroOffsetZ = gyroSumZ / numReadings;

  gyroAngleX = 0; // resets its last value
}
