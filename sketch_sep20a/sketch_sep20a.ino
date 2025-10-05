#include <Wire.h>
#include <Servo.h>

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
const int dataPrintInterval = 200;
int lastMillis = 0;
int counter = 0;

Servo esc;  // Create a Servo object to control the ESC
int escPin = 9;
int buttonPin = 12;
int currentSpeed = 1500; //neutral throttle
int speedStep = 1; // step size of pwm changes
int accelInterval = 100;

// define the state machine
enum SystemState{
  HOLD,
  TILTED_RIGHT,
  TILTED_LEFT
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

  // wait for button push to enter loop
  pinMode(buttonPin, INPUT_PULLUP);
  //Serial.println("push the button, fool");
  //while(digitalRead(buttonPin) == HIGH){
  //  // wait
  //}
  Serial.println("setting up the motor");
  setupMotor();
  delay(1000);
  Serial.println("Entering the loop");
}


void loop() {

  // read acceleration and speed data
  float* data = getData();
  
  // enter the state machine
  switch (currentState){
    case HOLD:
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      currentSpeed = 1500;
      Serial.println(currentSpeed);
      esc.writeMicroseconds(currentSpeed);
      if (data[1] > .1){
        currentState = TILTED_RIGHT;
        break;
      }
      if (data[1] < -.1){
        currentState = TILTED_LEFT;
        break;
      }
      break;

    case TILTED_RIGHT:
      digitalWrite(2, HIGH);
      Serial.println(currentSpeed);
      accelerateRight(currentSpeed);
      if (data[1] < .1){
        currentState = HOLD;
      }
      break;

    case TILTED_LEFT:
      digitalWrite(3, HIGH);
      accelerateLeft(currentSpeed);
      if (data[1] > -.1){
        currentState = HOLD;
      }
      break;
  }

  //// write data to the console in set intervals
  //if (millis() - lastMillis >= dataPrintInterval){
  //  lastMillis = millis();
  //  printReadings(data);
  //}
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


void setupMotor(){
  esc.attach(escPin); // ESC signal connected to pin D9
  Serial.begin(9600);

  // Arm the ESC
  Serial.println("Arming ESC...");
  esc.writeMicroseconds(1000); // Send low throttle to arm (usually 1000µs)
  Serial.println("plug in battery now, then push button");
  while(digitalRead(buttonPin) == HIGH){
    // wait
  }

  Serial.println("ESC Armed. Starting motor...");
}


void changeThrottle(int newSpeed){
  esc.writeMicroseconds(newSpeed); // Range is typically 1000 (stop) to 2000 (full throttle)
}


void accelerateRight(int speed){
  //accelertates the wheel
  if(speed >= 1600){
    // too fast to change the speed
    Serial.println("pwm speed limit reached"); //typically 2000us for bidirectional speed controllers.
    currentSpeed = 1590;
    esc.writeMicroseconds(currentSpeed);
    return;
  } else if (speed <= 1300){
    // too fast to change the speed
    Serial.println("pwm speed floor reached"); // typically 1000us for bidirectional speed controllers.
    currentSpeed = 1310;
    esc.writeMicroseconds(currentSpeed);
    return;
  } else if (millis() - lastMillis >= accelInterval){
    // change the speed of the motor
    lastMillis = millis();
    currentSpeed = speed + speedStep;
    esc.writeMicroseconds(currentSpeed);
    Serial.println(currentSpeed);
  }
}


void accelerateLeft(int speed){
  //accelertates the wheel
  if(speed >= 1600){
    // too fast to change the speed
    Serial.println("pwm speed limit reached"); //typically 2000us for bidirectional speed controllers.
    currentSpeed = 1590;
    esc.writeMicroseconds(currentSpeed);
    return;
  } else if (speed <= 1300){
    // too fast to change the speed
    Serial.println("pwm speed floor reached"); // typically 1000us for bidirectional speed controllers.
    currentSpeed = 1310;
    esc.writeMicroseconds(currentSpeed);
    return;
  } else if (millis() - lastMillis >= accelInterval){
    // change the speed of the motor
    lastMillis = millis();
    currentSpeed = speed - speedStep;
    esc.writeMicroseconds(currentSpeed);
    Serial.println(currentSpeed);
  }
}

