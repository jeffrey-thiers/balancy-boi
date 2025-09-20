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


// print time interval
const int interval = 200;
int lastMillis = 0;
int counter = 0;

// define the state machine
enum SystemState{
  HOLD,
  TILT_RIGHT,
  TILT_LEFT
};

SystemState currentState = HOLD;


void setup() {
  // comms for MCU6050 IMU
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // LED pins
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
}


void loop() {

  // read acceleration and speed data
  float* data = getData();
  
  // enter the state machine
  switch (currentState){
    case HOLD:
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      if (data[1] > .1){
        currentState = TILT_RIGHT;
        break;
      }
      if (data[1] < -.1){
        currentState = TILT_LEFT;
        break;
      }
      break;

    case TILT_RIGHT:
      digitalWrite(2, HIGH);
      if (data[1] < .1){
        currentState = HOLD;
      }
      break;

    case TILT_LEFT:
      digitalWrite(3, HIGH);
      if (data[1] > -.1){
        currentState = HOLD;
      }
      break;
  }

  // write data to the console
  int currentMillis = millis();
  if (currentMillis - lastMillis >= interval){
    lastMillis = currentMillis;
    printReadings(data);
  }
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
  float accel_z = raw_az / ACCEL_SCALE_FACTOR;
  float gyro_x = raw_gx / GYRO_SCALE_FACTOR;

  static float readings[3];
  readings[0] = accel_y;
  readings[1] = accel_z;
  readings[2] = gyro_x;

  return readings;
}


void printReadings(float arr[]) {
  Serial.print("accel, Y = "); Serial.print(arr[0]);
  Serial.print(" | accel, Z = "); Serial.print(arr[1]);
  Serial.print(" | w (dps): X = "); Serial.println(arr[2]);
  // delay(200);
}
