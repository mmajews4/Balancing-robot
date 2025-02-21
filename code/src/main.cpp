#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"
#include "PID.h"

// ---   T E S T S   ---
//#include "tests/bluetooth_test.h" - passed
//#include "tests/mpu6050_test.h" - passed
//#include "tests/motors_test.h" - passed
//#include "tests/motors_with_bluetooth_test.h"// - passed

// Half-step PID parameters

// controll parameters 
enum State {TUNE, RUN};
enum Mode {ANGLE_CONTROLL, POSITION_CONTROLL, SPEED_CONTROLL};
State state = TUNE;
Mode mode = ANGLE_CONTROLL;

// -------   M O T O R   C O N T R O L L  ---
const int STEP_R = 5;
const int DIR_R = 17;
const int STEP_L = 19;
const int DIR_L = 18;
const int MAX_SPEED_DELAY = -1650; // 2150-1650=500
//const int MAX_SPEED_DELAY = -3420; // 3920-3420=500 (mnimal from my throttle equation)in stpes/sec corrected by max throttle value
const int SAMPLING_PERIOD = 30000; // millis
const int SENDING_PERIOD = 20; // one in how many periods i send parameters via bluetooth

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
uint8_t show_stats = 1; // you can hide stats while riding so that robot cycle doesn't stop momentarly to send bluetooh message
int sendDelay = SENDING_PERIOD;
int BT_timeout = 10;
char message[180];
void tune();

//-----------------   P I D   --------------
// dobrać według symulacji
PID angPID;
PID posPID;
PID spdPID;

void initialize_PIDs(){
    angPID.vP = 130;     
    angPID.vI = 18;
    angPID.vD = 1500;

    posPID.vP = 0;     
    posPID.vI = 0;
    posPID.vD = 0;

    spdPID.vP = 0;     
    spdPID.vI = 0;
    spdPID.vD = 0;
}

int position = 0; // in steps
int posL = 0;
int posR = 0;
float speed = 0;

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
        
    initialize_PIDs();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    calibrateMPU6050();
    while(!ESP_BT.connected()) {}
    BT_timeout = 10;
    prevSampleTime = micros();
    prevMotorTimeL = prevSampleTime;
    prevMotorTimeR = prevSampleTime;
}

void loop(){
    // -------------------------   B L U E T O O T H   R E C I V E   C O M M A N D S   --- 
    if (BT_timeout>0) {
        if (ESP_BT.connected()) BT_timeout = 10;
        BT_timeout--;
        // Receive data from Bluetooth
        if (ESP_BT.available()){
            incomingChar = ESP_BT.read();  // Read one character

            switch(incomingChar){
                case 'r':   // run
                    state = RUN;
                    previousTime = micros(); // to not mess up gyroAngle mesurement
                    gyroAngleX = 0;
                    break;
                case 't':   // tune
                    state = TUNE;
                    angPID.reset();
                    tune();
                    break;
                case 'h':   // hides statistics while driving to prevent micro breaks in stearing
                    if(show_stats) show_stats = 0;
                    else show_stats = 1;
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

        if(state == RUN){
            // --------------------------------------------   P I D   --------------------
            take_measurement();
            position = (posL + posR)>>1;    // right shift by one is equivalent to division by 2 but more optimal in case of time 
                                            // in mm, with 200step per rot motor and r=3.2cm, mine 3.5, one step is one mm

            if(mode == ANGLE_CONTROLL){

                angPID.targetValue = steer;
            } 
            else if(mode == POSITION_CONTROLL) {

                posPID.targetValue = steer*100; // one step is about one mm, with default steer factor being 0.8, one arrow click is about 8cm or 0.8dm
                posPID.currentValue = position;

                angPID.targetValue = posPID.calculate();
            }
            else if(mode == SPEED_CONTROLL) {
                speed = position*1000000/SAMPLING_PERIOD; // in mm/s, division takes long time, I only want it in this mode

                spdPID.targetValue = steer*10; // with default steer factor being 0.8, one arrow click is about 8mm/s which is okay
                spdPID.currentValue = speed;

                angPID.targetValue = spdPID.calculate();
            }
            angPID.currentValue = angleX + adjustment;
            throttle = angPID.calculate();

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

            // -------------------------   B L U E T O O T H   S T A T U S   S E N D   ---

            // Sending data to the controller every SENDING_PERIOD
            if(sendDelay == SENDING_PERIOD && show_stats){
                sprintf(message, "-----\n\
Ang P:%.0f  I:%.2f  D:%.0f\n\
Pos P:%.0f  I:%.2f  D:%.0f\n\
Spd P:%.0f  I:%.2f  D:%.0f\n\
Angle:%0.3f\n\
Steer:%0.1f  Turn:%.2f\n\
Throt:%d  MotSpeed:%d\n\
Pos:%dmm  Speed:%.0fmm/s\n",\
angPID.xP, angPID.xI, angPID.xD, posPID.xP, posPID.xI, posPID.xD, spdPID.xP, spdPID.xI, spdPID.xD, angleX, steer, turn, throttle, motor_throttle, position, speed);
                ESP_BT.print(message);
                sendDelay = 0;
            }
            sendDelay++;

            // zrobić wersję jeszcze z użyciem rtosa jak by to nie działo


            // jeżli sybkość samplowania jest szybsza niż szybkość motoru to przechodzić przez tą pętę nawet kila razy i doppiero za którymś puścić silnik
            // żeby nie staneły koła i czały czas updateowała się szykość

            // tak żeby thorttle było liniowe                            200000
            if(throttleR != 0) time_to_next_step_R = MAX_SPEED_DELAY + ((550000/abs(throttleR)));
            else time_to_next_step_R = 3000000;

            if(throttleL != 0) time_to_next_step_L = MAX_SPEED_DELAY + ((550000/abs(throttleL)));
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
                        if(throttleR < 0) posR--;
                        else posR++;
                        prevMotorTimeR = currMotorTimeR;
                    }
                    if(do_step_L){
                        digitalWrite(STEP_L, LOW);
                        if(throttleL < 0) posL--;
                        else posL++;
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
    while(state == TUNE){
        // Receive data from Bluetooth
        if (ESP_BT.available()) {
            command = ESP_BT.readString(); // Read incoming data
        
            switch(command[0]){
                case 'r':   // run
                    state = RUN;
                    angPID.reset();
                    posPID.reset();
                    spdPID.reset();
                    posL = 0;
                    posR = 0;
                    gyroAngleX = 0;
                    previousTime = micros(); // to not mess up gyroAngle mesurement
                    break;
                case 'a':   // tune angle PID and acceleration
                    switch(command[1]){
                        case 'm':
                            mode = ANGLE_CONTROLL;
                            break;
                        case 'p':
                            command.remove(0, 2); // Remove first two characters starting at index 0
                            angPID.vP = command.toFloat();
                            break;
                        case 'i':
                            command.remove(0, 2);
                            angPID.vI = command.toFloat();
                            break;
                        case 'd':
                            command.remove(0, 2);
                            angPID.vD = command.toFloat();
                            break;
                        case 'c':   // acceleration
                            command.remove(0, 2);
                            acceleration = command.toFloat();
                            break;
                    }
                    break;
                case 'p':   // tune position PID
                    switch(command[1]){
                        case 'm':
                            mode = POSITION_CONTROLL;
                            break;
                        case 'p':
                            command.remove(0, 2); 
                            posPID.vP = command.toFloat();
                            break;
                        case 'i':
                            command.remove(0, 2);
                            posPID.vI = command.toFloat();
                            break;
                        case 'd':
                            command.remove(0, 2);
                            posPID.vD = command.toFloat();
                            break;
                    }
                    break;
                case 's':   // tune speed PID
                    switch(command[1]){
                        case 'm':
                            mode = SPEED_CONTROLL;
                            break;
                        case 'p':
                            command.remove(0, 2); 
                            spdPID.vP = command.toFloat();
                            break;
                        case 'i':
                            command.remove(0, 2);
                            spdPID.vI = command.toFloat();
                            break;
                        case 'd':
                            command.remove(0, 2);
                            spdPID.vD = command.toFloat();
                            break;
                    }
                    break;
                case 'f':   // speed factor
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
        sprintf(message, "-----\n\
Ang P:%.0f   I:%.2f   D:%.0f\n\
Pos P:%.0f   I:%.2f   D:%.0f\n\
Spd P:%.0f   I:%.2f   D:%.0f\n\
Angle: %0.3f  Steer: %0.1f\n\
Gyro:  %.3f  Accel: %.3f\n\
Pos: %dmm  Speed: %.0fmm/s\n",\
            angPID.vP, angPID.vI, angPID.vD, posPID.vP, posPID.vI, posPID.vD, spdPID.vP, spdPID.vI, spdPID.vD, angleX, steer, gyroAngleX, accelAngleX, position, speed);
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
