#include <Servo.h>

Servo esc;  // Create a Servo object to control the ESC
int escPin = 9;
int buttonPin = 12;

int CURRENT_SPEED = 1500; //neutral throttle
int SPEED_STEP = 10; // step size of pwm changes

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.println("setting up the motor");
  setupMotor();
  delay(3000);
  Serial.println("Entering the loop");
}

void loop(){
  // accelerate wheel
  delay(3000);
  accelerateWheel(CURRENT_SPEED);
  delay(3000);
  Serial.println("end of loop");
}

void accelerateWheel(int speed){
  //accelertates the wheel
  if(speed >= 2000){
    Serial.println("can't accelerate past 2000 us");
    return;
  } else if (speed <= 1000){
    Serial.println("can't go slower than 0 us");
    return;
  } else {
    CURRENT_SPEED = speed + SPEED_STEP;
    Serial.println(CURRENT_SPEED);
    esc.writeMicroseconds(CURRENT_SPEED);
  }
  
}


void setupMotor(){
  esc.attach(escPin); // ESC signal connected to pin D9
  Serial.begin(9600);

  // Arm the ESC
  Serial.println("Arming ESC...");
  esc.writeMicroseconds(1500); // Send low throttle to arm (usually 1500µs for bidirectional)
  Serial.println("plug in battery now, then push button");
  while(digitalRead(buttonPin) == HIGH){
    // wait
  }

  Serial.println("ESC Armed. Starting motor.");
}
