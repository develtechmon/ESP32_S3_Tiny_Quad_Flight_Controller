#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <VL53L0X.h>  // ToF sensor library

// ==== Define the pin for the built-in LED ====
#define LED_PIN 48  
#define NUM_LEDS 1

// ===== PPM Definitions =====
#define PPM_PIN 13             
#define NUM_CHANNELS 8         
#define PPM_SYNC_THRESHOLD 3000 
#define CHANNEL_MIN 1000       
#define CHANNEL_MAX 2000       

// Define your custom I2C pins
#define CUSTOM_SDA_PIN 3   
#define CUSTOM_SCL_PIN 4   

// ===== Altitude Hold Channel =====
#define ALT_HOLD_CHANNEL 5  // Channel for enabling altitude hold

// ===== ADVANCED ALTITUDE HOLD SYSTEM (VL53L0X Implementation) =====

// VL53L0X Filtering System (adapted from TF Mini approach)
#define LIDAR_BUFFER_SIZE 21   
int lidarHistTab[LIDAR_BUFFER_SIZE];
int lidarHistIdx = 0;
int lidarSum = 0;

// Complementary Filter
float complementaryFiltered = 0;
const float COMP_FILTER_ALPHA = 0.75;  

// Altitude Hold Variables
bool altitudeHoldActive = false;
float groundReference = 0;           
float targetAltitude = 0;            
float currentAltitude = 0;           
int baseThrottle = 0;                

// LANDING MODE VARIABLES (NEW)
bool landingMode = false;
float landingStartAltitude = 0;
unsigned long landingStartTime = 0;
const float LANDING_RATE_CMS = 30.0;    // 30 cm/s descent rate
const int THROTTLE_DEADBAND = 40;       // Deadband around center stick
const int LANDING_THROTTLE_THRESHOLD = 1300; // Below this = landing mode

// Altitude PID Controller
float altPID_error = 0;
float altPID_errorPrev = 0;
float altPID_integral = 0;
float altPID_output = 0;

// Improved PID Gains (less aggressive to prevent spikes)
float altPID_Kp = 0.8;          // Reduced from 1.2
float altPID_Ki = 0.15;         // Reduced from 0.3  
float altPID_Kd = 0.4;          // Reduced from 0.8
float altPID_integralMax = 150.0; // Reduced from 200

// ===== Flight Controller Variables =====
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
float AccXCalibration, AccYCalibration, AccZCalibration;

// PPM variables
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;
int channelValues[NUM_CHANNELS];

// PID parameters
float PAngleRoll = 2, PAnglePitch = 2;
float IAngleRoll = 0.5, IAnglePitch = 0.5;
float DAngleRoll = 0.007, DAnglePitch = 0.007;
float PRateRoll = 0.625, PRatePitch = 0.625;
float IRateRoll = 2.1, IRatePitch = 2.1;
float DRateRoll = 0.0088, DRatePitch = DRateRoll;
float PRateYaw = 4, IRateYaw = 3, DRateYaw = 0;

// Throttle limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;
int led_time = 500;

// PID variables
volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;
volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

// Kalman filter variables
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// Arming variables
bool armed = false;
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000;

// Hardware
const int pwmFrequency = 20000;
const int pwmResolution = 8;
const int motor1Channel = 11, motor2Channel = 10;
const int motor3Channel = 9, motor4Channel = 8;
const float t = 0.004;

// VL53L0X sensor
VL53L0X distanceSensor;

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== VL53L0X ALTITUDE FUNCTIONS (adapted from TF Mini approach) =====

int readVL53L0XDistance() {
  uint16_t distance = distanceSensor.readRangeContinuousMillimeters();
  
  // Handle sensor errors
  if (distanceSensor.timeoutOccurred()) {
    return 0; // Invalid reading
  }
  
  // Convert to cm and validate range
  int distanceCm = distance / 10;
  
  // VL53L0X range validation (adjust based on your sensor's capabilities)
  if (distanceCm >= 1 && distanceCm <= 200) {
    return distanceCm;
  }
  
  return 0; // Invalid reading
}

// YMFC-style circular buffer filter (adapted for VL53L0X)
int applyCircularBufferFilter(int newValue) {
  lidarHistTab[lidarHistIdx] = newValue;
  lidarSum += lidarHistTab[lidarHistIdx];
  lidarSum -= lidarHistTab[(lidarHistIdx + 1) % LIDAR_BUFFER_SIZE];
  
  lidarHistIdx++;
  if (lidarHistIdx == LIDAR_BUFFER_SIZE) lidarHistIdx = 0;
  
  return lidarSum / (LIDAR_BUFFER_SIZE - 1);
}

// Complementary filter with alpha = 0.75
int applyComplementaryFilter(int newValue) {
  static bool firstReading = true;
  if (firstReading) {
    complementaryFiltered = newValue;
    firstReading = false;
    return (int)complementaryFiltered;
  }
  
  complementaryFiltered = COMP_FILTER_ALPHA * newValue + 
                         (1.0 - COMP_FILTER_ALPHA) * complementaryFiltered;
  
  return (int)complementaryFiltered;
}

void initializeAltitudeSystem() {
  Serial.println("Initializing VL53L0X altitude system...");
  
  // Initialize VL53L0X
  if (!distanceSensor.init()) {
    Serial.println("VL53L0X sensor failed!");
    strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red = sensor failed
    strip.show();
    delay(2000);
    return;
  }
  
  // Configure sensor
  distanceSensor.setTimeout(100);
  distanceSensor.setMeasurementTimingBudget(20000);
  distanceSensor.startContinuous();
  
  // Initialize circular buffer
  for (int i = 0; i < LIDAR_BUFFER_SIZE; i++) {
    lidarHistTab[i] = 0;
  }
  lidarSum = 0;
  lidarHistIdx = 0;
  
  // Calibrate ground reference
  Serial.println("Calibrating ground reference...");
  int validReadings = 0;
  float sum = 0;
  
  for (int i = 0; i < 100; i++) {
    int reading = readVL53L0XDistance();
    if (reading > 0) {
      sum += reading;
      validReadings++;
    }
    delay(20);
    
    if (i % 10 == 0) {
      strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue during calibration
      strip.show();
      delay(50);
      strip.setPixelColor(0, strip.Color(0, 0, 0));
      strip.show();
    }
  }
  
  if (validReadings > 10) {
    groundReference = sum / validReadings;
    Serial.print("Ground reference: ");
    Serial.print(groundReference);
    Serial.println(" cm");
  } else {
    groundReference = 0; // Default
    Serial.print("Using default ground reference: ");
    Serial.print(groundReference);
    Serial.println(" cm");
  }
  
  Serial.println("VL53L0X altitude system ready");
}

void updateAltitude() {
  int rawReading = readVL53L0XDistance();
  
  if (rawReading > 0) {
    // Apply both filters
    int circularFiltered = applyCircularBufferFilter(rawReading);
    int complementaryFiltered = applyComplementaryFilter(circularFiltered);
    
    // Use complementary filtered result for control (more stable)
    currentAltitude = complementaryFiltered - groundReference;

    // Ensure positive altitude
    if (currentAltitude < 0) currentAltitude = 0;
  }
}

void calculateAltitudePID() {
  if (!altitudeHoldActive) {
    altPID_output = 0;
    altPID_integral = 0;  // Reset integral when not active
    return;
  }
  
  // Calculate error
  altPID_error = targetAltitude - currentAltitude;
  
  // Add deadband to reduce small oscillations
  if (abs(altPID_error) < 3.0) {
    altPID_error = 0;  // 3cm deadband
  }
  
  // Proportional term
  float P_term = altPID_Kp * altPID_error;
  
  // Integral term with windup protection
  altPID_integral += altPID_error * t;
  if (altPID_integral > altPID_integralMax) altPID_integral = altPID_integralMax;
  if (altPID_integral < -altPID_integralMax) altPID_integral = -altPID_integralMax;
  float I_term = altPID_Ki * altPID_integral;
  
  // Derivative term with filtering to reduce noise
  float error_derivative = (altPID_error - altPID_errorPrev) / t;
  static float prev_derivative = 0;
  error_derivative = 0.7 * prev_derivative + 0.3 * error_derivative; // Simple filter
  prev_derivative = error_derivative;
  
  float D_term = altPID_Kd * error_derivative;
  altPID_errorPrev = altPID_error;
  
  // Calculate total PID output
  altPID_output = P_term + I_term + D_term;
  
  // Limit output (reduced for smoother control)
  if (altPID_output > 200) altPID_output = 200;
  if (altPID_output < -200) altPID_output = -200;
}

// Handle pilot input for altitude adjustments and landing
void handlePilotAltitudeInput() {
  if (!altitudeHoldActive) return;
  
  int throttleInput = ReceiverValue[2];
  int throttleCenter = 1500;
  int throttleError = throttleInput - throttleCenter;
  
  // Check for landing mode
  if (throttleInput < LANDING_THROTTLE_THRESHOLD) {
    if (!landingMode) {
      // Enter landing mode
      landingMode = true;
      landingStartAltitude = currentAltitude;
      landingStartTime = millis();
      Serial.println("LANDING MODE ACTIVATED - Gentle descent");
    }
    
    // Calculate gentle descent
    float elapsedTime = (millis() - landingStartTime) / 1000.0; // seconds
    float descentDistance = LANDING_RATE_CMS * elapsedTime;
    targetAltitude = landingStartAltitude - descentDistance;
    
    // Stop descent when close to ground
    if (targetAltitude < 10) {
      targetAltitude = 5; // Hover just above ground
    }
    
    return; // Skip normal altitude adjustment
  } else {
    // Exit landing mode if throttle raised
    if (landingMode) {
      landingMode = false;
      Serial.println("Landing mode CANCELLED");
      // Reset target to current altitude to prevent jumps
      targetAltitude = currentAltitude;
    }
  }
  
  // Normal altitude adjustment (outside deadband)
  if (abs(throttleError) > THROTTLE_DEADBAND) {
    // Convert throttle input to altitude change rate
    float altitudeChangeRate = (throttleError - (throttleError > 0 ? THROTTLE_DEADBAND : -THROTTLE_DEADBAND)) * 0.1; // cm/s
    
    // Limit altitude change rate
    if (altitudeChangeRate > 50) altitudeChangeRate = 50;   // Max 50 cm/s up
    if (altitudeChangeRate < -30) altitudeChangeRate = -30; // Max 30 cm/s down
    
    // Update target altitude
    targetAltitude += altitudeChangeRate * t * 250; // Scale for 4ms loop
    
    // Limit target altitude
    if (targetAltitude < 10) targetAltitude = 10;     // Minimum 10cm
    if (targetAltitude > 300) targetAltitude = 300;   // Maximum 3m
  }
}

void handleAltitudeHold() {
  bool altHoldSwitch = (ReceiverValue[ALT_HOLD_CHANNEL] > 1700);
  
  // Activate altitude hold
  if (altHoldSwitch && !altitudeHoldActive && armed && currentAltitude > 15) {
    altitudeHoldActive = true;
    targetAltitude = currentAltitude;
    baseThrottle = ReceiverValue[2];
    landingMode = false; // Ensure landing mode is off
    
    // Reset PID
    altPID_error = 0;
    altPID_errorPrev = 0;
    altPID_integral = 0;
    altPID_output = 0;
    
    Serial.print("Altitude Hold ACTIVATED at ");
    Serial.print(targetAltitude);
    Serial.println(" cm");
  }
  
  // Deactivate altitude hold
  if (!altHoldSwitch && altitudeHoldActive) {
    altitudeHoldActive = false;
    landingMode = false;
    altPID_output = 0;
    Serial.println("Altitude Hold DEACTIVATED");
  }
}

int calculateThrottleOutput() {
  if (altitudeHoldActive) {
    int throttle;
    
    if (landingMode) {
      // In landing mode, use reduced base throttle for gentle descent
      throttle = baseThrottle - 100 + (int)altPID_output; // Reduced base for landing
    } else {
      // Normal altitude hold
      throttle = baseThrottle + (int)altPID_output;
    }
    
    // Safety limits
    if (throttle > 1750) throttle = 1750; // Reduced max for safety
    if (throttle < 1100) throttle = 1100;
    
    return throttle;
  } else {
    // Manual throttle control
    return ReceiverValue[2];
  }
}

// ===== PPM Interrupt =====
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

// ===== Read Receiver =====
void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// ===== Kalman Filter =====
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4);
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
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

  // Flash LED for startup
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(led_time);
    digitalWrite(LED_BUILTIN, LOW);
    delay(led_time);
  }
  
  // Setup PPM
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  // Setup I2C and MPU6050
  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Initialize VL53L0X altitude system
  initializeAltitudeSystem();

  // Setup motors
  ledcAttach(motor1Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor2Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor3Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor4Channel, pwmFrequency, pwmResolution);

  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);

  // Calibration Values
  RateCalibrationRoll=2.80;
  RateCalibrationPitch=-1.86;
  RateCalibrationYaw=-1.47;
  AccXCalibration=0.04;
  AccYCalibration=0.01;
  AccZCalibration=-0.09;

  // Ready!
  strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green = ready
  strip.show();
  delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  Serial.println("=== ADVANCED VL53L0X Flight Controller Ready ===");
  Serial.println("Features:");
  Serial.println("- Channel 5: Altitude Hold On/Off");
  Serial.println("- Throttle stick: Adjust altitude in altitude hold");
  Serial.println("- Throttle < 1300: Gentle landing mode");

  LoopTimer = micros();
}

void loop() {
  read_receiver(channelValues);

  // ===== ADVANCED ALTITUDE SYSTEM =====
  updateAltitude();
  handleAltitudeHold();
  handlePilotAltitudeInput();  // Handle pilot input for altitude changes
  calculateAltitudePID();

  // ===== ARMING LOGIC =====
  if (channelValues[2] < 1050) {
    if (!armed && channelValues[3] > 1900) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = true;
        strip.setPixelColor(0, strip.Color(255, 255, 0)); // Yellow = armed
        strip.show();
        delay(1000);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        Serial.println("Drone Armed");
        armDisarmTimer = 0;
      }
    } else if (armed && channelValues[3] < 1100) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = false;
        altitudeHoldActive = false;
        landingMode = false;
        strip.setPixelColor(0, strip.Color(255, 255, 255)); // White = disarmed
        strip.show();
        delay(1000);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        Serial.println("Drone Disarmed");
        armDisarmTimer = 0;
      }
    } else {
      armDisarmTimer = 0;
    }
  } else {
    armDisarmTimer = 0;
  }
  
  // If not armed, stop everything
  if (!armed) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    
    int pwm1 = map(MotorInput1, 1000, 2000, 0, 255);
    int pwm2 = map(MotorInput2, 1000, 2000, 0, 255);
    int pwm3 = map(MotorInput3, 1000, 2000, 0, 255);
    int pwm4 = map(MotorInput4, 1000, 2000, 0, 255);
    ledcWrite(motor1Channel, constrain(pwm1, 0, 255));
    ledcWrite(motor2Channel, constrain(pwm2, 0, 255));
    ledcWrite(motor3Channel, constrain(pwm3, 0, 255));
    ledcWrite(motor4Channel, constrain(pwm4, 0, 255));
    
    while (micros() - LoopTimer < (t * 1000000));
    LoopTimer = micros();
    return;
  }

  // Show status LED
  if (altitudeHoldActive) {
    if (landingMode) {
      strip.setPixelColor(0, strip.Color(255, 165, 0)); // Orange = landing mode
    } else {
      strip.setPixelColor(0, strip.Color(0, 0, 50)); // Blue = altitude hold
    }
  } else {
    strip.setPixelColor(0, strip.Color(0, 50, 0)); // Green = normal
  }
  strip.show();

  // ===== READ IMU DATA =====
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  RateRoll  = (float)GyroX / 65.5 - RateCalibrationRoll;
  RatePitch = (float)GyroY / 65.5 - RateCalibrationPitch;
  RateYaw   = (float)GyroZ / 65.5 - RateCalibrationYaw;
  AccX = (float)AccXLSB / 4096 - AccXCalibration;
  AccY = (float)AccYLSB / 4096 - AccYCalibration;
  AccZ = (float)AccZLSB / 4096 - AccZCalibration;

  // Calculate angles
  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
  
  // Kalman filtering
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = constrain(Kalman1DOutput[0], -20, 20);
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = constrain(Kalman1DOutput[0], -20, 20);
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Set desired angles
  DesiredAngleRoll  = 0.1 * (channelValues[0] - 1500);
  DesiredAnglePitch = 0.1 * (channelValues[1] - 1500);
  
  // Calculate throttle output (with altitude hold)
  InputThrottle = calculateThrottleOutput();
  
  DesiredRateYaw = 0.15 * (channelValues[3] - 1500);

  // ===== ANGLE PID FOR ROLL =====
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // ===== ANGLE PID FOR PITCH =====
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // ===== RATE PID CALCULATIONS =====
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

  // Limit throttle
  InputThrottle = constrain(InputThrottle, ThrottleCutOff, 2000);

  // ===== MOTOR MIXING =====
  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw; // front right - CCW
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw; // rear right - CW
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw; // rear left  - CCW
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw; // front left - CW

  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 2000);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 2000);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 2000);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 2000);

  // Emergency stop and reset conditions
  if (channelValues[2] < 1030) {  // Emergency stop
      MotorInput1 = ThrottleCutOff;
      MotorInput2 = ThrottleCutOff;
      MotorInput3 = ThrottleCutOff;
      MotorInput4 = ThrottleCutOff;
      PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
      PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
      PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
      PrevItermAngleRoll = PrevItermAnglePitch = 0;
      groundReference = 0;
      // Reset altitude hold and landing mode
      if (altitudeHoldActive || landingMode) {
        altitudeHoldActive = false;
        landingMode = false;
        Serial.println("Emergency stop - all modes disabled");
      }
  }

  // Convert to PWM and output
  int pwm1 = constrain(map(MotorInput1, 1000, 2000, 0, 255), 0, 255);
  int pwm2 = constrain(map(MotorInput2, 1000, 2000, 0, 255), 0, 255);
  int pwm3 = constrain(map(MotorInput3, 1000, 2000, 0, 255), 0, 255);
  int pwm4 = constrain(map(MotorInput4, 1000, 2000, 0, 255), 0, 255);

  ledcWrite(motor1Channel, pwm1);
  ledcWrite(motor2Channel, pwm2);
  ledcWrite(motor3Channel, pwm3);
  ledcWrite(motor4Channel, pwm4);

  // Uncomment for debugging:
  // Serial.print("Alt: "); Serial.print(currentAltitude);
  // Serial.print(" Tgt: "); Serial.print(targetAltitude);
  // Serial.print(" Out: "); Serial.print(altPID_output);
  // Serial.print(" Thr: "); Serial.println(InputThrottle);

  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}
