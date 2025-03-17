#include <Arduino.h>

// ***** PPM Receiver Setup *****
#define PPM_PIN 13               // PPM input pin (must be interrupt-capable)
#define NUM_CHANNELS 8           // Number of channels in the PPM signal
#define PPM_SYNC_THRESHOLD 3000  // Sync pulse threshold (in microseconds)
#define CHANNEL_MIN 1000         // Minimum valid pulse width (in microseconds)
#define CHANNEL_MAX 2000         // Maximum valid pulse width (in microseconds)

volatile int ReceiverValue[NUM_CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;

void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    // Sync pulse detected; reset channel index.
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    // Save the pulse width as a channel value (constrained to valid range)
    ReceiverValue[channelIndex] = constrain(pulseWidth, CHANNEL_MIN, CHANNEL_MAX);
    channelIndex++;
  }
}

void readReceiver(int* channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// ***** Motor Driver PWM Setup *****
// Define motor PWM output pins (choose pins not used by PPM input)
const int motor1Pin = 25;
const int motor2Pin = 26;
const int motor3Pin = 27;
const int motor4Pin = 32;

// PWM parameters
const int pwmFrequency = 20000; // 20 kHz PWM frequency
const int pwmResolution = 8;    // 8-bit resolution: 0-255 duty cycle
const int motor1Channel = 0;
const int motor2Channel = 1;
const int motor3Channel = 2;
const int motor4Channel = 3;

void setup() {
  Serial.begin(115200);
  Serial.println("PPM Reader & Quad Motor Controller Initialized");

  // Setup PPM input pin and attach interrupt
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);

  // Setup PWM for each motor using the ESP32 LEDC peripheral
  ledcSetup(motor1Channel, pwmFrequency, pwmResolution);
  ledcSetup(motor2Channel, pwmFrequency, pwmResolution);
  ledcSetup(motor3Channel, pwmFrequency, pwmResolution);
  ledcSetup(motor4Channel, pwmFrequency, pwmResolution);

  // Attach PWM channels to the corresponding motor output pins
  ledcAttachPin(motor1Pin, motor1Channel);
  ledcAttachPin(motor2Pin, motor2Channel);
  ledcAttachPin(motor3Pin, motor3Channel);
  ledcAttachPin(motor4Pin, motor4Channel);

  // Initialize motor outputs to zero (motors off)
  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);
}

void loop() {
  int channels[NUM_CHANNELS];
  readReceiver(channels);

  // For debugging: print out the PPM channel values
  Serial.print("Roll [µs]: "); Serial.print(channels[0]);
  Serial.print(" Pitch [µs]: "); Serial.print(channels[1]);
  Serial.print(" Throttle [µs]: "); Serial.print(channels[2]);
  Serial.print(" Yaw [µs]: "); Serial.print(channels[3]);
  Serial.println();

  // Use the throttle channel (channel index 2) to control motor speed
  // Map the pulse width (1000-2000 µs) to an 8-bit PWM value (0-255)
  int throttlePWM = map(channels[2], CHANNEL_MIN, CHANNEL_MAX, 0, 255);
  throttlePWM = constrain(throttlePWM, 0, 255);

  // Drive all four motors with the throttle PWM value
  ledcWrite(motor1Channel, throttlePWM);
  ledcWrite(motor2Channel, throttlePWM);
  ledcWrite(motor3Channel, throttlePWM);
  ledcWrite(motor4Channel, throttlePWM);

  delay(50);
}
