#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // Initialize the MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print(F("MPU6050 initialization failed with status: "));
    Serial.println(status);
    while (1);
  }

  // Calibrate the MPU6050
  Serial.println(F("Calibrating MPU6050, please keep the device still..."));
  delay(1000);
  mpu.calcOffsets(); // This function calculates the offsets and sets them internally
  Serial.println(F("Calibration complete!\n"));

  // Retrieve and print the calculated offsets
  Serial.print(F("Accelerometer Offsets:\n"));
  Serial.print(F("X: ")); Serial.println(mpu.getAccXoffset());
  Serial.print(F("Y: ")); Serial.println(mpu.getAccYoffset());
  Serial.print(F("Z: ")); Serial.println(mpu.getAccZoffset());

  Serial.print(F("\nGyroscope Offsets:\n"));
  Serial.print(F("X: ")); Serial.println(mpu.getGyroXoffset());
  Serial.print(F("Y: ")); Serial.println(mpu.getGyroYoffset());
  Serial.print(F("Z: ")); Serial.println(mpu.getGyroZoffset());
}

void loop() {
  // The main loop can remain empty for this example
}
