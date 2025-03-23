#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <VL53L0X.h>

// ==== Define the pin for the built-in LED. Change this if needed ====
#define LED_PIN 48  
#define NUM_LEDS 1

// ===== PPM Definitions =====
#define PPM_PIN 13             // PPM signal pin
#define NUM_CHANNELS 8         // Total channels in the PPM stream
#define PPM_SYNC_THRESHOLD 3000 // Sync pulse threshold in µs
#define CHANNEL_MIN 1000       // Minimum valid pulse width in µs
#define CHANNEL_MAX 2000       // Maximum valid pulse width in µs

// ===== Custom I2C Pins =====
#define CUSTOM_SDA_PIN 3   // Example: GPIO3
#define CUSTOM_SCL_PIN 4   // Example: GPIO4

// ===== Flight Controller / PID Declarations =====
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
float AccXCalibration, AccYCalibration, AccZCalibration;

// Global variables for PPM
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;
int channelValues[NUM_CHANNELS];

float PAngleRoll = 2;       float PAnglePitch = PAngleRoll;
float IAngleRoll = 0.5;     float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.007;   float DAnglePitch = DAngleRoll;

float PRateRoll = 0.625;    float PRatePitch = PRateRoll;
float IRateRoll = 2.1;      float IRatePitch = IRateRoll;
float DRateRoll = 0.0088;   float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

// Throttle limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;
int led_time = 500;

volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;
volatile float KalmanGainPitch, KalmanGainRoll;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

// Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4; // (2*2)
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// Battery Parameters
float Voltage;

// PWM configuration: using LEDC peripheral
const int pwmFrequency = 20000; // 20 kHz PWM frequency
const int pwmResolution = 8;    // 8-bit resolution (0 to 255)

// Assign each motor to a unique LEDC channel
const int motor1Channel = 11;
const int motor2Channel = 10;
const int motor3Channel = 9;
const int motor4Channel = 8;

// Time step (seconds)
const float t = 0.004; 

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== Altitude Hold Variables using VL53L0X =====
VL53L0X altSensor;  // VL53L0X instance for altitude
unsigned long lastAltUpdate = 0;   // Timing variable for altitude updates
float currentAltitude = 0.0;         // Measured altitude in meters
float altitudeIntegral = 0.0;
float lastAltitudeError = 0.0;
float altitudePIDOutput = 0.0;
// PID gains for altitude hold (tune these for your system)
float Kp_alt = 3.5;
float Ki_alt = 0.0015;
float Kd_alt = 0.01;

// Map the throttle channel (range: 1000 to 1800) to a desired altitude (0.5 m to 3.0 m)
float getDesiredAltitude() {
  int throttle = ReceiverValue[2];
  throttle = constrain(throttle, 1000, 1800);
  float desiredAlt = ((float)(throttle - 1000) / 800.0) * (3.0 - 0.5) + 0.5;
  return desiredAlt;
}

// ===== PPM Interrupt Handler =====
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;
  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    if (pulseWidth < CHANNEL_MIN) pulseWidth = CHANNEL_MIN;
    else if (pulseWidth > CHANNEL_MAX) pulseWidth = CHANNEL_MAX;
    ReceiverValue[channelIndex] = pulseWidth;
    channelIndex++;
  }
}

void battery_voltage(void) {
  Voltage = (float)analogRead(1) / 237; // Example conversion
}

void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// Simple 1D Kalman Filter for angle estimation
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 16); // (4 deg/s)^2
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9); // (3 deg)^2
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  strip.begin();
  strip.show();
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(led_time);
    digitalWrite(LED_BUILTIN, LOW);
    delay(led_time);
  }
  
  // Setup PPM Receiver
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);
  
  // Setup MPU6050 (I2C)
  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Setup PWM for Motor Drivers using LEDC
  ledcAttach(motor1Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor2Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor3Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor4Channel, pwmFrequency, pwmResolution);
  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);
  
  // Calibration Values (adjust as needed)
  RateCalibrationRoll  = 2.80;
  RateCalibrationPitch = -1.86;
  RateCalibrationYaw   = -1.47;
  AccXCalibration = 0.04;
  AccYCalibration = 0.01;
  AccZCalibration = 0.09;
  
  // Signal readiness with a green LED flash
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
  
  LoopTimer = micros();
  
  // Initialize the VL53L0X Sensor for Altitude Measurement
  altSensor.setTimeout(500);
  if (!altSensor.init()) {
    Serial.println("Failed to detect and initialize VL53L0X!");
    while (1);
  }
  altSensor.startContinuous();
  lastAltUpdate = micros();
}

void loop() {
  read_receiver(channelValues);
  
  // ----- Read MPU6050 Data -----
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  
  // Apply calibration offsets
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;
  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;
  
  // Calculate angles from accelerometer data
  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
  
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
  
  // Clamp the estimated angles to ±20 degrees
  if (KalmanAngleRoll > 20) KalmanAngleRoll = 20;
  else if (KalmanAngleRoll < -20) KalmanAngleRoll = -20;
  if (KalmanAnglePitch > 20) KalmanAnglePitch = 20;
  else if (KalmanAnglePitch < -20) KalmanAnglePitch = -20;
  
  DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  // Initially, use the receiver throttle
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);
  
  // ----- Attitude PID (Angle Mode) -----
  float ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  if (ItermRoll > 400) ItermRoll = 400;
  else if (ItermRoll < -400) ItermRoll = -400;
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  if (PIDOutputRoll > 400) PIDOutputRoll = 400;
  else if (PIDOutputRoll < -400) PIDOutputRoll = -400;
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;
  
  float ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  if (ItermPitch > 400) ItermPitch = 400;
  else if (ItermPitch < -400) ItermPitch = -400;
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  if (PIDOutputPitch > 400) PIDOutputPitch = 400;
  else if (PIDOutputPitch < -400) PIDOutputPitch = -400;
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;
  
  // ----- Rate PID Calculations -----
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;
  
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  if (ItermRoll > 400) ItermRoll = 400;
  else if (ItermRoll < -400) ItermRoll = -400;
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  if (PIDOutputRoll > 400) PIDOutputRoll = 400;
  else if (PIDOutputRoll < -400) PIDOutputRoll = -400;
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;
  
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  if (ItermPitch > 400) ItermPitch = 400;
  else if (ItermPitch < -400) ItermPitch = -400;
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  if (PIDOutputPitch > 400) PIDOutputPitch = 400;
  else if (PIDOutputPitch < -400) PIDOutputPitch = -400;
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;
  
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  if (ItermYaw > 400) ItermYaw = 400;
  else if (ItermYaw < -400) ItermYaw = -400;
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  if (PIDOutputYaw > 400) PIDOutputYaw = 400;
  else if (PIDOutputYaw < -400) PIDOutputYaw = -400;
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;
  
  if (InputThrottle > 1800) { InputThrottle = 1800; }
  
  // ----------- Altitude Hold using VL53L0X -----------
  // Update altitude hold every ~50ms
  unsigned long currentAltTime = micros();
  if (currentAltTime - lastAltUpdate >= 50000) {
    float dtAlt = (currentAltTime - lastAltUpdate) / 1000000.0; // dt in seconds
    lastAltUpdate = currentAltTime;
    
    uint16_t distance = altSensor.readRangeContinuousMillimeters();
    // If no timeout occurred, update current altitude (convert mm to m)
    if (!altSensor.timeoutOccurred()) {
      currentAltitude = distance / 1000.0;
    }
    
    float desiredAltitude = getDesiredAltitude();
    float altError = desiredAltitude - currentAltitude;
    altitudeIntegral += altError * dtAlt;
    float altitudeDerivative = (altError - lastAltitudeError) / dtAlt;
    altitudePIDOutput = Kp_alt * altError + Ki_alt * altitudeIntegral + Kd_alt * altitudeDerivative;
    lastAltitudeError = altError;
    
    // Use a fixed baseline throttle (1550 µs for hover) plus altitude correction
    InputThrottle = 1550 + altitudePIDOutput;
    if (InputThrottle > 1800) { InputThrottle = 1800; }
    if (InputThrottle < ThrottleIdle) { InputThrottle = ThrottleIdle; }
  }
  
  // ----- Motor Mixing -----
  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw); // front right
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw); // front left
  
  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 1999);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 1999);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 1999);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 1999);
  
  // Disarm motors if throttle channel is too low
  if (ReceiverValue[2] < 1030) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
  }
  
  // ----- Map Motor Input (µs) to PWM (0-255) -----
  int pwm1 = map(MotorInput1, 1000, 2000, 0, 255);
  int pwm2 = map(MotorInput2, 1000, 2000, 0, 255);
  int pwm3 = map(MotorInput3, 1000, 2000, 0, 255);
  int pwm4 = map(MotorInput4, 1000, 2000, 0, 255);
  
  pwm1 = constrain(pwm1, 0, 255);
  pwm2 = constrain(pwm2, 0, 255);
  pwm3 = constrain(pwm3, 0, 255);
  pwm4 = constrain(pwm4, 0, 255);
  
  ledcWrite(motor1Channel, pwm1);
  ledcWrite(motor2Channel, pwm2);
  ledcWrite(motor3Channel, pwm3);
  ledcWrite(motor4Channel, pwm4);
  
  while (micros() - LoopTimer < (t * 1000000)) {
    // Wait until next loop cycle
  }
  LoopTimer = micros();
}
