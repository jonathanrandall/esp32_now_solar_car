#include <Arduino.h>

const int top_servo = 16; // Top servo 
const int bottom_servo = 4; //Bottomservo
const int servoFrequency = 50; // PWM frequency in Hz
const int bottom_ch = 4; //timer channels
const int top_ch = 5;


//analogue inputs for photoresistors
const int photo_rb = 39; //right bottom
const int photo_rt = 36; //right top
const int photo_lb = 35; //left bottom
const int photo_lt = 34; //left top


void setup() {
  // Configure LED Controller Channel 4 for Servo 1
  ledcSetup(bottom_ch, servoFrequency, 16);
  ledcAttachPin(bottom_servo, bottom_ch);

  // Configure LED Controller Channel 5 for Servo 2
  ledcSetup(top_ch, servoFrequency, 16);
  ledcAttachPin(top_servo, top_ch);

  Serial.begin(115200); // Initialize serial communication
  Serial.println("Enter desired microseconds (750 to 2250) for Servo 2:");
}

void loop() {
  if (Serial.available() > 0) {
    int microsInput = Serial.parseInt(); // Read the input from serial monitor

    // Ensure the input is within the valid range
    if (microsInput >= 350 && microsInput <= 2750) {
      writeServoMicros(top_ch, microsInput); // change to bottom_ch to calibrate bottom servo
      Serial.print("Setting Servo 2 to ");
      Serial.print(microsInput);
      Serial.println(" microseconds");
    } else {
      Serial.println("Invalid input. Enter a value between 750 and 2250 microseconds.");
    }
  }
}

void writeServoMicros(int ch, int micros) {
  // Map the desired pulse width (micros) to the PWM range (0-65535)
  int dutyCycle = map(micros, 0, 20000, 0, 65535);

  // Set the PWM duty cycle to move the servo
  ledcWrite(ch, dutyCycle);
}
