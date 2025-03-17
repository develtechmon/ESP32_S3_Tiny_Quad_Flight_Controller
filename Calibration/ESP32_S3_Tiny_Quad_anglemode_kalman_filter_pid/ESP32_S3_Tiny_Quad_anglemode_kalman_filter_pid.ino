#include <Wire.h>
#include <math.h>

// ===== PPM Definitions =====
#define PPM_PIN 13             // PPM input pin
#define NUM_CHANNELS 8         // Total number of channels in the PPM stream
#define PPM_SYNC_THRESHOLD 3000 // Sync pulse threshold (µs)
#define CHANNEL_MIN 1000       // Minimum valid pulse width (µs)
#define CHANNEL_MAX 2000       // Maximum valid pulse width (µs)

// ===== Flight Controller / PID Declarations =====
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float AccXCalibration, AccYCalibration, AccZCalibration;

// Global variables for PPM
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;

int ESCfreq = 500; // originally used with Servo library, not used now
int channelValues[NUM_CHANNELS];
float PAngleRoll = 2, PAnglePitch = 2;
float IAngleRoll = 0.5, IAnglePitch = 0.5;
float DAngleRoll = 0.007, DAnglePitch = 0.007;

float PRateRoll = 0.625, PRatePitch = 0.625;
float IRateRoll = 2.1, IRatePitch = 2.1;
float DRateRoll = 0.0088, DRatePitch = 0.0088;
float PRateYaw = 4, IRateYaw = 3, DRateYaw = 0;

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

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
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// Battery voltage variable
float Voltage;

// ===== PPM Interrupt Handler =====
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;
  
  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    // Constrain and store each pulse width
    ReceiverValue[channelIndex] = constrain(pulseWidth, CHANNEL_MIN, CHANNEL_MAX);
    channelIndex++;
  }
}

void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// ===== Battery Voltage Reading =====
void battery_voltage() {
  Voltage = (float)analogRead(1) / 237.0; // Example: analogRead from ADC channel (adjust as needed)
}

// ===== Simple 1D Kalman Filter =====
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 16); // variance: 4 deg/s squared
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9); // error std: 3 deg
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// ===== Motor Driver PWM Setup =====
// Define motor output pins (using ESP32-S3 PWM with LEDC)
const int motor1Pin = 25;
const int motor2Pin = 26;
const int motor3Pin = 27;
const int motor4Pin = 32;

// PWM configuration: using LEDC peripheral
const int pwmFrequency = 20000; // 20 kHz PWM frequency for low noise operation
const int pwmResolution = 8;    // 8-bit resolution: values from 0 to 255

// Assign each motor to a unique LEDC channel
const int motor1Channel = 0;
const int motor2Channel = 1;
const int motor3Channel = 2;
const int motor4Channel = 3;

void setup() {
  Serial.begin(115200);
  Serial.println("Flight Controller with Motor PWM Initialized");

  // Setup indicator pins (example)
  pinMode(2, OUTPUT);
  pinMode(48, OUTPUT);
  // Flash LED on pin 48 a few times for startup indication
  for (int i = 0; i < 6; i++) {
    digitalWrite(48, HIGH);
    delay(100);
    digitalWrite(48, LOW);
    delay(100);
  }

  // ----- Setup PPM Receiver -----
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  // ----- Setup I2C for MPU6050 -----
  Wire.setClock(400000);
  Wire.begin(4, 5); // using CUSTOM_SDA_PIN and CUSTOM_SCL_PIN
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ----- Setup PWM for Motor Drivers using LEDC -----
  ledcSetup(motor1Channel, pwmFrequency, pwmResolution);
  ledcSetup(motor2Channel, pwmFrequency, pwmResolution);
  ledcSetup(motor3Channel, pwmFrequency, pwmResolution);
  ledcSetup(motor4Channel, pwmFrequency, pwmResolution);
  ledcAttachPin(motor1Pin, motor1Channel);
  ledcAttachPin(motor2Pin, motor2Channel);
  ledcAttachPin(motor3Pin, motor3Channel);
  ledcAttachPin(motor4Pin, motor4Channel);

  // Initialize motor outputs to minimum throttle (1000 µs mapped to 0 PWM)
  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);

  // ----- Calibration Values (example values) -----
  RateCalibrationRoll = -4.74;
  RateCalibrationPitch = 0.79;
  RateCalibrationYaw = -0.88;
  AccXCalibration = 0.04;
  AccYCalibration = 0.01;
  AccZCalibration = 0.07;

  // Read battery voltage
  battery_voltage();
  if (Voltage < 11.6) {
    digitalWrite(48, HIGH);
    // Disarm motors if battery voltage is low
    ledcWrite(motor1Channel, 0);
    ledcWrite(motor2Channel, 0);
    ledcWrite(motor3Channel, 0);
    ledcWrite(motor4Channel, 0);
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    return;
  }

  delay(500);
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);

  LoopTimer = micros();
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
  AccX      -= AccXCalibration;
  AccY      -= AccYCalibration;
  AccZ      -= AccZCalibration;

  // Calculate angles from accelerometer data
  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Clamp Kalman angles to ±20 degrees
  KalmanAngleRoll  = constrain(KalmanAngleRoll, -20, 20);
  KalmanAnglePitch = constrain(KalmanAnglePitch, -20, 20);

  // Use receiver channels:
  // Channels: 0 = Roll, 1 = Pitch, 2 = Throttle, 3 = Yaw, etc.
  DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

  // --- Angle PID for Roll ---
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = constrain(PIDOutputRoll, -400, 400);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // --- Angle PID for Pitch ---
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = constrain(PIDOutputPitch, -400, 400);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // --- Rate PID Calculations ---
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Rate PID for Roll
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = constrain(PIDOutputRoll, -400, 400);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Rate PID for Pitch
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = constrain(PIDOutputPitch, -400, 400);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Rate PID for Yaw
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = constrain(ItermYaw, -400, 400);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = constrain(PIDOutputYaw, -400, 400);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  // Limit throttle if too high
  if (InputThrottle > 1800) { InputThrottle = 1800; }

  // Motor Mixing (example configuration)
  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw; // front right
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw; // rear right
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw; // rear left
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw; // front left

  // Clamp motor outputs to safe range (between ThrottleIdle and 2000)
  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 1999);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 1999);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 1999);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 1999);

  // If throttle channel is too low, disarm motors
  if (ReceiverValue[2] < 1030) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
  }

  // --- Convert Motor Input (in µs) to PWM value (0-255) ---
  // Assuming 1000 µs corresponds to 0 and 2000 µs to 255.
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

  // Maintain loop timing
  while (micros() - LoopTimer < (t * 1000000)) {
    // wait until the time step is reached
  }
  LoopTimer = micros();
}
