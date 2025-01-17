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
const int MAX_SPEED_DELAY = -3120; // 3920-3120=800 (mnimal from my throttle equation)in stpes/sec corrected by max throttle value
const int SAMPLING_PERIOD = 20000; // millis
const int SENDING_PERIOD = 10; // one in how many periods i send parameters via bluetooth

int throttle = 0, last_throttle = 0, motor_throttle = 0, throttleL = 0, throttleR = 0;
int time_to_next_step_L = 1000, time_to_next_step_R = 1000;
int wait_for_sample = 1, wait_for_motor_L = 1, wait_for_motor_R = 1;
int do_step_R, do_step_L;
int64_t currSampleTime, prevSampleTime, currMotorTimeL, prevMotorTimeL, currMotorTimeR, prevMotorTimeR;
int64_t exec_time = 0;
float acceleration = 20, steer_factor = 0.8;
float turn = 0, turn_factor = 10, turn_time_counter = 0, turn_dir = 0;

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
float vP = 130;     
float vI = 2;
float vD = 80;
float targetValue = 0;
float xP, xI, xD, currentValue, integralSum, currError, lastError;
float duration;

// -----------   ANGLE MEASUREMENT   ------
MPU6050 mpu;

float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float prevAxCal = 0, axFiltered = 0;
float prevAyCal = 0, ayFiltered = 0;
float prevAzCal = 0, azFiltered = 0;

float accelAngleX, accelAngleY;
float gyroAngleX, gyroAngleY;
float angleX, angleY;
double elapsedTime, currentTime, previousTime;
int16_t ax, ay, az, gx, gy, gz;
float axCal, ayCal, azCal, gxCal, gyCal, gzCal;
float alpha = 0.1; // low pass filter factor

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
    prevMotorTimeL = prevSampleTime;
    prevMotorTimeR = prevSampleTime;
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
                    previousTime = micros(); // to not mess up gyroAngle mesurement
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
                case 'w':
                    steer = steer + steer_factor;
                    break;
                case 's':
                    steer = steer - steer_factor;
                    break;
                case 'a':
                    turn_dir = 1;
                    turn_time_counter = 30;
                    break;
                case 'd':
                    turn_dir = -1;
                    turn_time_counter = 30;
                    break;
                default:    // steer value 0/9 -> -0.8/1.0
                    if(incomingChar >= '0' && incomingChar <= '9') // so that non number things doesn't change steer
                        steer =  steer_factor * (incomingChar - '0' - 4); //konwersja char do int
                    break;
            }
        }

        if(run){
            // --------------------------------------------   P I D   --------------------
            take_measurement();

            currentValue = angleX + adjustment;
            targetValue = steer;
        
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
                sprintf(message, "-----\nP:%.0f   I:%.1f   D:%.0f\nAngle: %0.3f\nSteer: %0.2f    Turn: %.2f\nThrottle: %d\n", \
                    xP, xI, xD, angleX, steer, turn, throttle);
                ESP_BT.print(message);
                sendDelay = 0;
            }
            sendDelay++;

            // ---------------------------------   M O T O R   C O N T R O L L   ---------

            if(throttle < -255) throttle = -255;
            else if(throttle > 255) throttle = 255;

            
            if(last_throttle - throttle < -acceleration){
                motor_throttle = last_throttle + acceleration;
            } else if(last_throttle - throttle > acceleration){
                motor_throttle = last_throttle - acceleration;
            } else {
                motor_throttle = throttle;
            }
            last_throttle = motor_throttle;
            
            // (filter turn)slowly dampening turn
            turn = 0.5 * turn + 0.5 * turn_factor * turn_dir;
            if(turn_time_counter > 0){  // turn for 30 cycles, then stop turning
                turn_time_counter--;
            } else {
                turn_dir = 0;
            }

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

            // tak żeby thorttle było liniowe                            200000
            if(throttleR != 0) time_to_next_step_R = MAX_SPEED_DELAY + ((1000000/abs(throttleR)));
            else time_to_next_step_R = 3000000;

            if(throttleL != 0) time_to_next_step_L = MAX_SPEED_DELAY + ((1000000/abs(throttleL)));
            else time_to_next_step_L = 3000000;

            exec_time = prevSampleTime - micros();

            // w trakcie czekania na kolejnego sampla ma kręcić silnikami
            do{
                // czeka aż minie czas do zrobienia kolejnego stepu
                // jeżeli ten czas jest większy niż czas do sampla to wyjdzie z czekania na motor i zrobi sampla
                do{
                    currMotorTimeL = micros();
                    currMotorTimeR = currMotorTimeL;
                    currSampleTime = currMotorTimeL;
                    wait_for_sample = (currSampleTime - prevSampleTime < SAMPLING_PERIOD);
                    wait_for_motor_L = (currMotorTimeL - prevMotorTimeL < time_to_next_step_L);
                    wait_for_motor_R = (currMotorTimeR - prevMotorTimeR < time_to_next_step_R);
                }while(wait_for_motor_R && wait_for_motor_R && wait_for_sample);
                
                // Jeżeli Sample time jest mniejszy niż Motor Delay to ominie motor i zrobi kolejny pomiar
                if(wait_for_sample){
                    do_step_R = currMotorTimeR - prevMotorTimeR > time_to_next_step_R - 600; // 390 us is a resulution of one throttle
                    do_step_L = currMotorTimeL - prevMotorTimeL > time_to_next_step_L - 600; // it us used so that both motors fire unless there is a turn
                    if(do_step_R) digitalWrite(STEP_R, HIGH);
                    if(do_step_L) digitalWrite(STEP_L, HIGH);
                    delayMicroseconds(10); // według dokumentacji wystraczy 1us impuls żeby zainicjować step, falling edge nic nie robi
                    if(do_step_R){
                        digitalWrite(STEP_R, LOW);
                        prevMotorTimeR = currMotorTimeR;
                    }
                    if(do_step_L){
                        digitalWrite(STEP_L, LOW);
                        prevMotorTimeL = currMotorTimeL;
                    }
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
                    previousTime = micros(); // to not mess up gyroAngle mesurement
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
                case 'a':
                    command.remove(0, 1);
                    acceleration = command.toFloat();
                    break;
                case 's':
                    command.remove(0, 1);
                    steer_factor = command.toFloat();
                    break;
                case 'm':
                    command.remove(0, 1);
                    adjustment = command.toFloat();
                    break;
                case 'c':
                    calibrateMPU6050();
                    ESP_BT.print("Calibrated\n");
                    break;
                default:    // steer value 1/9 -> -2.0/2.0
                    if((command[0] >= '0' && command[0] <= '9')||(command[0] >= 0 && command[0] <= 9)) // zabezpieczenie przed niewałaściwym stringiem
                        steer =  steer_factor * (command[0] - '1' - 5); //konwersja char do int
                    break;
            }
        }
        take_measurement();
        sprintf(message, "-----\nP:%.0f   I:%.2f   D:%.0f\nAngle: %0.3f\nSteer: %0.1f\nGyro: %.3f  Accel: %.3f\n",\
            vP, vI, vD, angleX, steer, gyroAngleX, accelAngleX);
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

    // Apply low-pass filter to accelerometer data
    axFiltered = alpha * axCal + (1 - alpha) * prevAxCal;
    ayFiltered = alpha * ayCal + (1 - alpha) * prevAyCal;
    azFiltered = alpha * azCal + (1 - alpha) * prevAzCal;
    prevAxCal = axFiltered;
    prevAyCal = ayFiltered;
    prevAzCal = azFiltered;

    // Calculate accelerometer angles
    accelAngleX = atan(ayFiltered / sqrt(pow(axFiltered, 2) + pow(azFiltered, 2))) * 180 / PI;
//    accelAngleY = atan(-axCal / sqrt(pow(ayCal, 2) + pow(azCal, 2))) * 180 / PI;

    // Calculate elapsed time
    currentTime = micros();
    elapsedTime = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    // Integrate gyroscope data
    gyroAngleX = 0.99 * (gyroAngleX + gxCal / 131.0 * elapsedTime) + 0.01 * accelAngleX;

    // Complementary filter
//    angleX = 0.98 * gyroAngleX + 0.02 * angleX;
    angleX = gyroAngleX;
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
