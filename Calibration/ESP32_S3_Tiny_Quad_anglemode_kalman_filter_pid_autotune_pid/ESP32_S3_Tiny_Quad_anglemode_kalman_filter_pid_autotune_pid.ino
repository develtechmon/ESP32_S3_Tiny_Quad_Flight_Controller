#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

// ==== Define the pin for the built-in LED. Change if needed ====
#define LED_PIN 48  
#define NUM_LEDS 1

// ===== PPM Definitions =====
#define PPM_PIN 13             // Pin where the PPM signal is connected
#define NUM_CHANNELS 8         // Total number of channels in the PPM stream
#define PPM_SYNC_THRESHOLD 3000 // Pulse width (µs) above which a sync pulse is assumed
#define CHANNEL_MIN 1000       // Minimum valid pulse width (µs)
#define CHANNEL_MAX 2000       // Maximum valid pulse width (µs)

// Define your custom I2C pins (change these as needed)
#define CUSTOM_SDA_PIN 3       // Example: GPIO3
#define CUSTOM_SCL_PIN 4       // Example: GPIO4

// ===== Flight Controller / PID Declarations =====
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
float AccXCalibration, AccYCalibration, AccZCalibration;

// Global variables for PPM
volatile int ReceiverValue[NUM_CHANNELS] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;

int channelValues[NUM_CHANNELS];

// PID gains for Angle control (used to compute desired rates)
float PAngleRoll = 2;       float PAnglePitch = PAngleRoll;
float IAngleRoll = 0.5;     float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.007;   float DAnglePitch = DAngleRoll;

// PID gains for Rate control
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

// PID computation variables (Angle PID)
volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;

// PID computation variables (Rate PID)
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;
volatile float KalmanGainPitch, KalmanGainRoll;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

// Kalman filter variables for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// Battery Parameters
float Voltage;

// PWM configuration: using LEDC peripheral
const int pwmFrequency = 20000; // 20 kHz PWM frequency for low noise operation
const int pwmResolution = 8;    // 8-bit resolution: values from 0 to 255

// Assign each motor to a unique LEDC channel
const int motor1Channel = 11;
const int motor2Channel = 10;
const int motor3Channel = 9;
const int motor4Channel = 8;

// Time step (seconds)
const float t = 0.004; 

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== PPM Interrupt Handler =====
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    // A long pulse indicates a sync pulse – reset channel index.
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    // Constrain pulse width and store it.
    if (pulseWidth < CHANNEL_MIN) pulseWidth = CHANNEL_MIN;
    else if (pulseWidth > CHANNEL_MAX) pulseWidth = CHANNEL_MAX;
    ReceiverValue[channelIndex] = pulseWidth;
    channelIndex++;
  }
}

// ==== Read Battery Voltage ====
void battery_voltage(void) {
  Voltage = (float)analogRead(1) / 237; // Example conversion (adjust as needed)
}

// ===== Helper Function to Safely Copy Receiver Values =====
void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// ===== Simple 1D Kalman Filter =====
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4); // '4' is variance (4 deg/s)
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3); // 3 deg std error
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// ===== Autotune Variables and Function =====
// Autotune for Roll axis using Ziegler–Nichols method
bool autoTuneMode = false;
float autotune_P_gain = 1.0; // initial autotune P gain
float ultimateGain = 0;
float ultimatePeriod = 0;
unsigned long lastPeakTimeAuto = 0;
int oscillationCount = 0;
const int requiredOscillations = 5;  // number of oscillations to sample T_u
unsigned long lastGainUpdate = 0;
const unsigned long gainUpdateInterval = 100000; // update every 100 ms
float prevAutoTuneError = 0;

void autoTunePID() {
  // Use roll angle error as the signal to monitor:
  float currentError = DesiredAngleRoll - KalmanAngleRoll;
  unsigned long currentTime = micros();

  // Detect a zero crossing (error changing sign)
  if ((prevAutoTuneError < 0 && currentError >= 0) ||
      (prevAutoTuneError > 0 && currentError <= 0)) {
    if (lastPeakTimeAuto != 0) {
      unsigned long period = currentTime - lastPeakTimeAuto;
      // Average the periods over oscillations:
      if (oscillationCount == 0) {
        ultimatePeriod = period;
      } else {
        ultimatePeriod = (ultimatePeriod * oscillationCount + period) / (oscillationCount + 1);
      }
      oscillationCount++;
      Serial.print("Oscillation count: ");
      Serial.print(oscillationCount);
      Serial.print(" Period (us): ");
      Serial.println(period);
    }
    lastPeakTimeAuto = currentTime;
  }
  
  // Increase autotune P gain gradually every gainUpdateInterval
  if (currentTime - lastGainUpdate > gainUpdateInterval) {
    autotune_P_gain += 0.1;
    lastGainUpdate = currentTime;
    Serial.print("Increasing autotune P gain to: ");
    Serial.println(autotune_P_gain);
  }
  
  // Set the roll angle controller to use the autotune P gain (I and D set to zero)
  PAngleRoll = autotune_P_gain;
  IAngleRoll = 0;
  DAngleRoll = 0;
  
  prevAutoTuneError = currentError;
  
  // If sufficient oscillations have been detected, compute new PID gains
  if (oscillationCount >= requiredOscillations) {
    ultimateGain = autotune_P_gain;
    Serial.println("Autotuning complete!");
    Serial.print("Ultimate Gain (K_u): ");
    Serial.println(ultimateGain);
    Serial.print("Ultimate Period (us): ");
    Serial.println(ultimatePeriod);
    
    // Convert period to seconds
    float T_u = ultimatePeriod / 1000000.0;
    // Ziegler–Nichols formulas for a PID controller:
    float newP = 0.6 * ultimateGain;
    float T_i = 0.5 * T_u;
    float T_d = 0.125 * T_u;
    float newI = (T_i > 0) ? (newP / T_i) : 0;
    float newD = newP * T_d;
    
    // Update the angle PID gains for roll
    PAngleRoll = newP;
    IAngleRoll = newI;
    DAngleRoll = newD;
    
    Serial.println("New PID parameters for Roll angle:");
    Serial.print("P: ");
    Serial.println(PAngleRoll);
    Serial.print("I: ");
    Serial.println(IAngleRoll);
    Serial.print("D: ");
    Serial.println(DAngleRoll);
    
    // Reset autotune variables and exit autotune mode
    autoTuneMode = false;
    autotune_P_gain = 1.0;
    oscillationCount = 0;
    lastPeakTimeAuto = 0;
    ultimatePeriod = 0;
    lastGainUpdate = 0;
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  strip.begin();       // Initialize the NeoPixel strip
  strip.show();        // Turn all pixels off initially

  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(led_time);
    digitalWrite(LED_BUILTIN, LOW);
    delay(led_time);
  }
  
  // ----- Setup PPM Receiver -----
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  // ----- Setup MPU6050 (I2C) -----
  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ----- Setup PWM for Motor Drivers using LEDC -----
  ledcAttach(motor1Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor2Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor3Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor4Channel, pwmFrequency, pwmResolution);

  // Initialize motor outputs to minimum throttle (1000 µs mapped to 0 PWM)
  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);

  // ----- Calibration Values -----
  RateCalibrationRoll = 2.57;
  RateCalibrationPitch = -1.71;
  RateCalibrationYaw = -1.57;
  AccXCalibration = 0.03;
  AccYCalibration = 0.01;
  AccZCalibration = 0.11;

  // ----- Indicate system ready with LED strip -----
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  LoopTimer = micros();
}

// ===== Main Loop =====
void loop() {
  read_receiver(channelValues);

  // ----- Check for Autotune Mode Trigger (Channel 7: index 6) -----
  // For safety, ensure that autotuning only runs when the drone is on the ground (propellers removed)
  if (ReceiverValue[6] > 1800) {  
    if (!autoTuneMode) {
      autoTuneMode = true;
      Serial.println("Entering Autotune Mode (Roll axis)");
    }
  }

  // ----- Read MPU6050 Data -----
  // Request accelerometer data
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
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  // Apply calibration offsets
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;
  AccX      -= AccXCalibration;
  AccY      -= AccYCalibration;
  AccZ      -= AccZCalibration;

  // ----- Calculate Angles from Accelerometer -----
  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
  
  // Apply Kalman filter for smoother angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Clamp filtered angles to ±20 degrees
  KalmanAngleRoll  = (KalmanAngleRoll  > 20) ? 20 : ((KalmanAngleRoll  < -20) ? -20 : KalmanAngleRoll);
  KalmanAnglePitch = (KalmanAnglePitch > 20) ? 20 : ((KalmanAnglePitch < -20) ? -20 : KalmanAnglePitch);
  
  // Set desired angles from receiver inputs (Channel 0 and 1)
  DesiredAngleRoll  = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  // Throttle from Channel 2 and desired yaw rate from Channel 3
  InputThrottle  = ReceiverValue[2];
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

  // ----- Autotune Mode for Roll PID -----
  if (autoTuneMode) {
    // Run the autotune routine (this updates the roll PID gains)
    autoTunePID();
    // Optionally, skip the normal roll PID calculations while autotuning.
    // For safety, you might set the desired rate to zero during autotune.
    DesiredRateRoll = 0;
  } else {
    // ----- Angle PID for Roll (normal mode) -----
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    PtermRoll = PAngleRoll * ErrorAngleRoll;
    ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
    ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
    DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
    PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
    PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
    DesiredRateRoll = PIDOutputRoll;
    PrevErrorAngleRoll = ErrorAngleRoll;
    PrevItermAngleRoll = ItermRoll;
    
    // ----- Angle PID for Pitch (normal mode) -----
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
    PtermPitch = PAnglePitch * ErrorAnglePitch;
    ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
    ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
    DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
    PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
    PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
    DesiredRatePitch = PIDOutputPitch;
    PrevErrorAnglePitch = ErrorAnglePitch;
    PrevItermAnglePitch = ItermPitch;
  }

  // ----- Compute errors for Rate PID (for Roll, Pitch, and Yaw) -----
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // ----- Rate PID for Roll (normal mode) -----
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // ----- Rate PID for Pitch (normal mode) -----
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // ----- Rate PID for Yaw (normal mode) -----
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  if (InputThrottle > 1800) { InputThrottle = 1800; }

  // ----- Compute Motor Inputs -----
  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw); // front left - clockwise

  // Clamp motor outputs to safe range (between ThrottleIdle and 1999)
  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 1999);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 1999);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 1999);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 1999);
  
  // ----- Disarm Motors if Throttle is Too Low -----
  if (ReceiverValue[2] < 1030) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
  }

  // --- Convert Motor Inputs (in µs) to PWM value (0-255) ---
  int pwm1 = map(MotorInput1, 1000, 2000, 0, 255);
  int pwm2 = map(MotorInput2, 1000, 2000, 0, 255);
  int pwm3 = map(MotorInput3, 1000, 2000, 0, 255);
  int pwm4 = map(MotorInput4, 1000, 2000, 0, 255);
  
  pwm1 = constrain(pwm1, 0, 255);
  pwm2 = constrain(pwm2, 0, 255);
  pwm3 = constrain(pwm3, 0, 255);
  pwm4 = constrain(pwm4, 0, 255);

  // Write PWM values to motor channels using LEDC
  ledcWrite(motor1Channel, pwm1);
  ledcWrite(motor2Channel, pwm2);
  ledcWrite(motor3Channel, pwm3);
  ledcWrite(motor4Channel, pwm4);

  while (micros() - LoopTimer < (t * 1000000));
  {
    LoopTimer = micros();
  }
}
