#include <Servo.h>

Servo esc;  // Create a Servo object to control the ESC
int buttonPin = 12;
int escPin = 9;
int CURRENT_SPEED = 1500; //neutral throttle
int SPEED_STEP = 2; // step size of pwm changes
int lastMillis = 0;
int interval = 50;


void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.println("setting up the motor");
  setupMotor();
  delay(1000);
  Serial.println("Entering the loop");
}


void loop(){
  accelerateRight(CURRENT_SPEED);
  // Serial.println("end of loop");
}


void accelerateRight(int speed){
  //accelertates the wheel
  if(speed >= 1600){
    Serial.println("pwm speed limit reached"); //typically 2000us for bidirectional speed controllers.
    return;
  } else if (speed <= 1000){
    Serial.println("pwm speed floor reached"); // typically 1000us for bidirectional speed controllers.
    return;
  } else if (millis() - lastMillis >= interval){
    lastMillis = millis();
    CURRENT_SPEED = speed + SPEED_STEP;
    //Serial.println(CURRENT_SPEED);
    esc.writeMicroseconds(CURRENT_SPEED);
    Serial.println(CURRENT_SPEED);
  }
}


void accelerateLeft(int speed){
  //accelertates the wheel
  if(speed >= 1600){
    Serial.println("pwm speed limit reached"); //typically 2000us for bidirectional speed controllers.
    return;
  } else if (speed <= 1000){
    Serial.println("pwm speed floor reached"); // typically 1000us for bidirectional speed controllers.
    return;
  } else if (millis() - lastMillis >= interval){
    lastMillis = millis();
    CURRENT_SPEED = speed - SPEED_STEP;
    //Serial.println(CURRENT_SPEED);
    esc.writeMicroseconds(CURRENT_SPEED);
    Serial.println(CURRENT_SPEED);
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
