#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <VL53L0X.h>  // ToF sensor library

// ==== Define the pin for the built-in LED. Change this if your board uses a different pin ====
#define LED_PIN 48  
#define NUM_LEDS 1

// ===== PPM Definitions =====
#define PPM_PIN 13             // Pin where the PPM signal is connected
#define NUM_CHANNELS 8         // Total number of channels in the PPM stream
#define PPM_SYNC_THRESHOLD 3000 // Pulse width (µs) above which a sync pulse is assumed
#define CHANNEL_MIN 1000       // Minimum valid pulse width (µs)
#define CHANNEL_MAX 2000       // Maximum valid pulse width (µs)

// Define your custom I2C pins (change these as needed)
#define CUSTOM_SDA_PIN 3   // Example: GPIO3
#define CUSTOM_SCL_PIN 4   // Example: GPIO4

// ===== Altitude Hold Definitions =====
#define ALT_HOLD_CHANNEL 5  // Channel for enabling altitude hold (typically a switch)

// ===== Flight Controller / PID Declarations =====
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;

// Global variables for PPM
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;
int channelValues[NUM_CHANNELS];

// Angle PID parameters
float PAngleRoll = 2, PAnglePitch = 2;
float IAngleRoll = 0.5, IAnglePitch = 0.5;
float DAngleRoll = 0.007, DAnglePitch = 0.007;

// Rate PID parameters
float PRateRoll = 0.625, PRatePitch = 0.625;
float IRateRoll = 2.1, IRatePitch = 2.1;
float DRateRoll = 0.0088, DRatePitch = DRateRoll;
float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

// ===== VL53L0X Reference Filtering Altitude Hold Variables =====
int16_t vlxHistTab[21];         // Circular buffer for 21 distance readings
int8_t vlxHistIdx = 0;          // Index for circular buffer
int32_t vlxHigh = 0;            // Running sum of distance readings
float vlxGroundDistance = 0;    // Ground reference distance
bool altitudeHoldActive = false; // Altitude hold state

// Altitude PID variables (using reference code gains)
float pid_p_gain_alt = 0.6;     // Proportional gain
float pid_i_gain_alt = 0.8;     // Integral gain  
float pid_d_gain_alt = 5.0;     // Derivative gain
int pid_max_alt = 200;          // Maximum PID output

float pid_alt_setpoint = 0;     // Target altitude (distance)
float pid_alt_input = 0;        // Current altitude (filtered distance)
float pid_output_alt = 0;       // PID output for throttle correction
float pid_i_mem_alt = 0;        // Integral memory
float pid_last_alt_d_error = 0; // Last error for derivative
int baseline_throttle = 1500;   // Throttle when altitude hold was activated

// Distance reading variables
float currentDistance = 0;      // Current distance reading
unsigned long lastVlxTime = 0;  // Timing for distance readings

// Throttle limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;
int led_time = 500;

// PID calculation variables for Roll, Pitch, Yaw
volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

// Kalman filters for angle estimation
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// VL53L0X sensor object
VL53L0X distanceSensor;

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

// ===== Arming/Disarming Variables =====
bool armed = false;             // Drone armed state
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000; // milliseconds required to hold stick

// ===== VL53L0X Reference Filtering Functions =====

// Initialize VL53L0X distance sensor and calibrate ground reference
void initializeVL53L0XAltitude() {
  Serial.println("📏 Calibrating VL53L0X altitude hold...");
  
  // Initialize the circular buffer
  for (int i = 0; i < 21; i++) {
    vlxHistTab[i] = 0;
  }
  vlxHistIdx = 0;
  vlxHigh = 0;
  vlxGroundDistance = 0;
  
  // Take 150 readings to establish ground reference (like original code)
  for (int cal_int = 0; cal_int < 150; cal_int++) {
    // Read distance from VL53L0X
    currentDistance = distanceSensor.readRangeContinuousMillimeters() / 10.0; // Convert to cm
    
    // Handle sensor errors
    if (distanceSensor.timeoutOccurred() || currentDistance > 200) {
      currentDistance = 200; // Max range fallback
    }
    if (currentDistance < 3) {
      currentDistance = 3; // Min range fallback
    }
    
    // Store in circular buffer for averaging (scale like original)
    vlxHistTab[vlxHistIdx] = currentDistance / 10;  // Scale down for storage
    vlxHigh += vlxHistTab[vlxHistIdx];
    vlxHigh -= vlxHistTab[(vlxHistIdx + 1) % 21];
    vlxHistIdx++;
    if (vlxHistIdx == 21) vlxHistIdx = 0;
    
    // Heavy filtering for ground reference (like original)
    vlxGroundDistance = vlxGroundDistance * 0.7f + vlxHigh * 0.15f;
    
    delay(10); // Small delay between readings
    
    if (cal_int % 25 == 0) {
      Serial.print(".");
    }
  }
  
  Serial.println(" ✅ VL53L0X calibration complete!");
  Serial.print("Ground distance reference: ");
  Serial.println(vlxGroundDistance * 10); // Convert back to cm for display
}

// Update VL53L0X altitude readings (call this every loop)
void updateVL53L0XAltitude() {
  // Read distance at reasonable intervals (not every loop for efficiency)
  if (millis() - lastVlxTime >= 50) { // 20Hz update rate (VL53L0X optimal)
    lastVlxTime = millis();
    
    // Read current distance
    currentDistance = distanceSensor.readRangeContinuousMillimeters() / 10.0; // Convert to cm
    
    // Handle sensor errors
    if (distanceSensor.timeoutOccurred() || currentDistance > 200) {
      currentDistance = 200; // Keep previous reading effectively
    }
    if (currentDistance < 3) {
      currentDistance = 3;
    }
    
    // Update circular buffer with new reading
    vlxHistTab[vlxHistIdx] = currentDistance / 10;  // Scale for storage
    vlxHigh += vlxHistTab[vlxHistIdx];              // Add new reading
    vlxHigh -= vlxHistTab[(vlxHistIdx + 1) % 21];  // Remove oldest reading
    vlxHistIdx++;
    if (vlxHistIdx == 21) vlxHistIdx = 0;
    
    // Apply filtering based on altitude hold state (exactly like original)
    if (!altitudeHoldActive) {
      // When not in altitude hold, track current altitude smoothly
      pid_alt_setpoint = pid_alt_setpoint * 0.7f + vlxHigh * 0.15f;
      pid_alt_input = pid_alt_setpoint;
    } else {
      // When in altitude hold, filter current reading
      pid_alt_input = pid_alt_input * 0.7f + vlxHigh * 0.15f;
    }
  }
}

// Calculate VL53L0X altitude PID (exactly like reference code)
void calculateVL53L0XAltitudePID() {
  if (!altitudeHoldActive) {
    pid_output_alt = 0;
    return;
  }
  
  // Calculate error (target - current)
  float pid_error_temp = pid_alt_input - pid_alt_setpoint;
  
  // Integral term with windup protection
  pid_i_mem_alt += pid_i_gain_alt * pid_error_temp;
  if (pid_i_mem_alt > pid_max_alt) pid_i_mem_alt = pid_max_alt;
  else if (pid_i_mem_alt < pid_max_alt * -1) pid_i_mem_alt = pid_max_alt * -1;
  
  // Complete PID calculation
  pid_output_alt = pid_p_gain_alt * pid_error_temp + 
                   pid_i_mem_alt + 
                   pid_d_gain_alt * (pid_error_temp - pid_last_alt_d_error);
  
  // Output limiting
  if (pid_output_alt > pid_max_alt) pid_output_alt = pid_max_alt;
  else if (pid_output_alt < pid_max_alt * -1) pid_output_alt = pid_max_alt * -1;
  
  // Store error for next derivative calculation
  pid_last_alt_d_error = pid_error_temp;
}

// Handle altitude hold activation/deactivation (like original logic)
void handleVL53L0XAltitudeHoldSwitching() {
  // Check channel for altitude hold switch
  bool altHoldSwitch = (channelValues[ALT_HOLD_CHANNEL] > 1700);
  
  // Deactivate altitude hold
  if (!altHoldSwitch && altitudeHoldActive) {
    altitudeHoldActive = false;
    pid_output_alt = 0;
    Serial.println("🔓 VL53L0X Altitude Hold DEACTIVATED");
  }
  
  // Activate altitude hold  
  if (altHoldSwitch && !altitudeHoldActive) {
    altitudeHoldActive = true;
    
    // Reset PID controller for smooth engagement
    pid_output_alt = 0;
    pid_i_mem_alt = 0;
    pid_last_alt_d_error = 0;
    
    // Remember current throttle as baseline (like original)
    baseline_throttle = channelValues[2];
    
    // Set current position as target
    pid_alt_setpoint = pid_alt_input;
    
    Serial.println("🔒 VL53L0X Altitude Hold ACTIVATED");
    Serial.print("Target distance: ");
    Serial.print(pid_alt_setpoint * 10); // Convert to cm for display
    Serial.print("cm | Baseline throttle: ");
    Serial.println(baseline_throttle);
  }
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

// ==== Read Battery Voltage ====
void battery_voltage(void) {
  Voltage = (float)analogRead(1) / 237; // GPIO1
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
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4); // IMU variance (4 deg/s)
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3); // error variance (3 deg)
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  strip.begin();       // Initialize the NeoPixel strip
  strip.show();        // Turn all pixels off as an initial state

  // Flash built-in LED a few times for startup indication
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

  // ----- Initialize VL53L0X sensor -----
  if (!distanceSensor.init()) {
    // Sensor initialization failed - indicate with red LED
    strip.setPixelColor(0, strip.Color(255, 0, 0));
    strip.show();
    Serial.println("Failed to detect and initialize VL53L0X sensor!");
    delay(2000);
  } else {
    // Configure VL53L0X for faster measurements
    distanceSensor.setTimeout(100); // 100ms timeout
    distanceSensor.setMeasurementTimingBudget(20000); // 20ms per measurement = 50Hz
    
    // Start continuous mode
    distanceSensor.startContinuous();
    
    // Initialize reference filtering altitude system
    initializeVL53L0XAltitude();
    
    // Indicate successful sensor initialization with blue flash
    strip.setPixelColor(0, strip.Color(0, 0, 255));
    strip.show();
    delay(500);
  }

  // ----- Setup PWM for Motor Drivers using LEDC -----
  ledcAttach(motor1Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor2Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor3Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor4Channel, pwmFrequency, pwmResolution);

  // Initialize motor outputs to minimum throttle
  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);

  // ----- Calibration Values -----
  // ----- 8520 Motor -----
  RateCalibrationRoll=2.80;
  RateCalibrationPitch=-1.86;
  RateCalibrationYaw=-1.47;
  AccXCalibration=0.04;
  AccYCalibration=0.01;
  AccZCalibration=-0.09;

  // Green LED to indicate normal startup
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  LoopTimer = micros();
}

void loop() {
  // ----- Read receiver values -----
  read_receiver(channelValues);

  // ----- Update VL53L0X altitude tracking -----
  updateVL53L0XAltitude();
  
  // ----- Handle VL53L0X altitude hold switching logic -----
  handleVL53L0XAltitudeHoldSwitching();

  // ----- Arming/Disarming Logic -----
  if (channelValues[2] < 1050) {
    if (!armed && channelValues[3] > 1900) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = true;
        // Yellow LED (Red + Green) to indicate arming
        strip.setPixelColor(0, strip.Color(255, 255, 0));
        strip.show();
        delay(1000);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        armDisarmTimer = 0;
      }
    } else if (armed && channelValues[3] < 1100) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = false;
        // White LED (all colors) to indicate disarming
        strip.setPixelColor(0, strip.Color(255, 255, 255));
        strip.show();
        delay(1000);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        armDisarmTimer = 0;
      }
    } else {
      armDisarmTimer = 0;
    }
  } else {
    armDisarmTimer = 0;
  }
  
  // If not armed, immediately cut off motor output and reset PID integrals
  if (!armed) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    pid_i_mem_alt = 0; // Reset VL53L0X altitude integral
    
    // Map and update motor outputs
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
    
    while (micros() - LoopTimer < (t * 1000000));
    LoopTimer = micros();
    return;
  }

  // Show altitude hold status with LED
  if (altitudeHoldActive) {
    // Blue LED when altitude hold is active
    strip.setPixelColor(0, strip.Color(0, 0, 50));
  } else {
    // Green LED when armed but no altitude hold
    strip.setPixelColor(0, strip.Color(0, 50, 0));
  }
  strip.show();

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
  Wire.write(0x08);
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
  AccX      = (float)AccXLSB / 4096;
  AccY      = (float)AccYLSB / 4096;
  AccZ      = (float)AccZLSB / 4096;

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
  
  // Run Kalman filters for both angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Clamp the filtered angles to ±20 degrees
  KalmanAngleRoll = constrain(KalmanAngleRoll, -20, 20);
  KalmanAnglePitch = constrain(KalmanAnglePitch, -20, 20);

  // ----- Set Desired Angles and Throttle from Receiver Inputs -----
  DesiredAngleRoll  = 0.1 * (channelValues[0] - 1500);
  DesiredAnglePitch = 0.1 * (channelValues[1] - 1500);
  
  // Calculate VL53L0X altitude PID
  calculateVL53L0XAltitudePID();
  
  // Throttle handling with VL53L0X altitude hold
  if (altitudeHoldActive) {
    // Use VL53L0X altitude hold (professional filtering approach)
    InputThrottle = baseline_throttle + pid_output_alt;
    
    // Allow pilot to adjust target altitude with small stick movements
    float throttleStickInput = channelValues[2] - baseline_throttle;
    if (abs(throttleStickInput) > 50) { // Significant stick movement
      baseline_throttle = channelValues[2]; // Update baseline
      // Fine adjustment of target (optional)
      // pid_alt_setpoint += throttleStickInput * 0.001; 
    }
  } else {
    // Normal throttle control
    InputThrottle = channelValues[2];
  }
  
  // Set desired yaw rate
  DesiredRateYaw = 0.15 * (channelValues[3] - 1500);

  // --- Angle PID for Roll ---
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // --- Angle PID for Pitch ---
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // ----- Rate PID Calculations -----
  ErrorRateRoll  = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw   = DesiredRateYaw - RateYaw;

  // Roll Rate PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch Rate PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw Rate PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = constrain(ItermYaw, -400, 400);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = constrain(PtermYaw + ItermYaw + DtermYaw, -400, 400);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  // Limit throttle to safe range
  InputThrottle = constrain(InputThrottle, ThrottleCutOff, 2000);

  // ----- Motor Mixing -----
  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw;
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw;
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw;
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw;

  // Clamp motor outputs to safe range
  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 2000);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 2000);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 2000);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 2000);

  // If throttle is too low and altitude hold is not active, reset PID integrals and cut motors
  if (channelValues[2] < 1030 && !altitudeHoldActive) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    pid_i_mem_alt = 0;
  }

  // --- Convert Motor Input (µs) to PWM value (0-255) and update outputs ---
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

  // Debug altitude information (uncomment if needed)
  // Serial.print("Alt: "); Serial.print(pid_alt_input * 10);
  // Serial.print(" Tgt: "); Serial.print(pid_alt_setpoint * 10);
  // Serial.print(" Out: "); Serial.println(pid_output_alt);

  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}
