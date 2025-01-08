// Aby użyć tego pliku należy zakomentować cały plik main i zaincludować ten

// DIAŁA!!! :)
#include <Wire.h>
#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;

float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

float accelAngleX, accelAngleY;
float gyroAngleX, gyroAngleY;
float angleX, angleY;
float elapsedTime, currentTime, previousTime;
int16_t ax, ay, az, gx, gy, gz;
float axCal, ayCal, azCal, gxCal, gyCal, gzCal;

void calibrateMPU6050();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("MPU6050 initialized successfully.");
  Serial.println("Calibrating...");

  calibrateMPU6050();
  Serial.println("Calibration complete.");
}

void loop() {
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
  accelAngleY = atan(-axCal / sqrt(pow(ayCal, 2) + pow(azCal, 2))) * 180 / PI;

  // Calculate elapsed time
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // Integrate gyroscope data
  gyroAngleX += gxCal / 131.0 * elapsedTime;
  gyroAngleY += gyCal / 131.0 * elapsedTime;

  // Complementary filter
  angleX = 0.92 * gyroAngleX + 0.08 * accelAngleX;
  angleY = 0.92 * gyroAngleY + 0.08 * accelAngleY;

  // Print angles
  Serial.print("Angle X: ");
  Serial.print(angleX);
  Serial.print("\tAngle Y: ");
  Serial.println(angleY);

  delay(10);
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
}
