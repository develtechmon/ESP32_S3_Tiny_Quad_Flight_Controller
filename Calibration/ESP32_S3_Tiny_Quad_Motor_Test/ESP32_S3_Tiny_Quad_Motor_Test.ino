// Define PWM output pins for four motors (adjust these pins to your wiring)
const int motor1Pin = 10;
const int motor2Pin = 11;
const int motor3Pin = 12;
const int motor4Pin = 13;

// PWM configuration parameters
const int pwmFrequency = 20000; // 20 kHz PWM frequency to reduce audible noise
const int pwmResolution = 8;    // 8-bit resolution: duty cycle from 0 to 255

// Assign each motor a unique LEDC channel
const int motor1Channel = 0;
const int motor2Channel = 1;
const int motor3Channel = 2;
const int motor4Channel = 3;

void setup() {
  // Set up PWM for each channel
  ledcSetup(motor1Channel, pwmFrequency, pwmResolution);
  ledcSetup(motor2Channel, pwmFrequency, pwmResolution);
  ledcSetup(motor3Channel, pwmFrequency, pwmResolution);
  ledcSetup(motor4Channel, pwmFrequency, pwmResolution);

  // Attach PWM channels to the corresponding GPIO pins
  ledcAttachPin(motor1Pin, motor1Channel);
  ledcAttachPin(motor2Pin, motor2Channel);
  ledcAttachPin(motor3Pin, motor3Channel);
  ledcAttachPin(motor4Pin, motor4Channel);

  // Ensure all motors start off
  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);
}

void loop() {
  // Example: Ramp up speed for all four motors simultaneously
  for (int duty = 0; duty <= 255; duty++) {
    ledcWrite(motor1Channel, duty);
    ledcWrite(motor2Channel, duty);
    ledcWrite(motor3Channel, duty);
    ledcWrite(motor4Channel, duty);
    delay(10);
  }

  // Run at full speed for 2 seconds
  delay(2000);

  // Ramp down speed for all motors
  for (int duty = 255; duty >= 0; duty--) {
    ledcWrite(motor1Channel, duty);
    ledcWrite(motor2Channel, duty);
    ledcWrite(motor3Channel, duty);
    ledcWrite(motor4Channel, duty);
    delay(10);
  }

  // Wait for 2 seconds before repeating the cycle
  delay(2000);
}
