#include <Servo.h>

Servo esc;  // Create a Servo object to control the ESC
int escPin = 9;
int buttonPin = 12;

void setup() {
  setupMotor();
  
  // wait for button push
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.print("push the button, fool");
  while(digitalRead(buttonPin) == HIGH){
    // wait
  }
  Serial.println("...   entering the loop");
}

void loop() {
  // Example: spin motor at 50% throttle
  changeThrottle(1050);
  delay(4000); // Spin for 4 seconds
  changeThrottle(1000);
  Serial.println("Motor stopped.");
  delay(5000); // Wait 5 seconds before restarting
}


void setupMotor(){
  esc.attach(escPin); // ESC signal connected to pin D9
  Serial.begin(9600);

  // Arm the ESC
  Serial.println("Arming ESC...");
  esc.writeMicroseconds(1000); // Send low throttle to arm (usually 1000µs)
  delay(3000); // Wait for arming (usually ~2-5 sec)

  Serial.println("ESC Armed. Starting motor...");
}

void changeThrottle(int newSpeed){
  esc.writeMicroseconds(newSpeed); // Range is typically 1000 (stop) to 2000 (full throttle)
}
