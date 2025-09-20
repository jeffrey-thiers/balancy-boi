#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050

int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;
int16_t raw_temp;

// Choose your desired sensitivity range
// Default values are +/-2g for accel and +/-250 deg/s for gyro
// Modify these if you change the configuration registers
const float ACCEL_SCALE_FACTOR = 16384.0;  // For +/-2g
const float GYRO_SCALE_FACTOR = 131.0;    // For +/-250 dps


void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}


void loop() {
  float* data = getData();
  printReadings(data);
}


float* getData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers

  raw_ax = Wire.read() << 8 | Wire.read();
  raw_ay = Wire.read() << 8 | Wire.read();
  raw_az = Wire.read() << 8 | Wire.read();
  raw_temp = Wire.read() << 8 | Wire.read();
  raw_gx = Wire.read() << 8 | Wire.read();
  raw_gy = Wire.read() << 8 | Wire.read();
  raw_gz = Wire.read() << 8 | Wire.read();

  float accel_y = raw_ay / ACCEL_SCALE_FACTOR;
  float gyro_x = raw_gx / GYRO_SCALE_FACTOR;

  static float readings[2];
  readings[0] = accel_y;
  readings[1] = gyro_x;

  return readings;
}


void printReadings(float arr[]) {
  Serial.print("accel, Y="); Serial.print(arr[0]);
  Serial.print(" | w (dps): X="); Serial.println(arr[1]);
  delay(200);
}
