#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <VL53L0X.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

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

// ===== Flight Controller Variables (ORIGINAL - UNCHANGED) =====
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
float AccXCalibration, AccYCalibration, AccZCalibration;

// PPM variables (ORIGINAL - UNCHANGED)
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;
int channelValues[NUM_CHANNELS];

// PID parameters (ORIGINAL - UNCHANGED)
float PAngleRoll = 2, PAnglePitch = 2;
float IAngleRoll = 0.5, IAnglePitch = 0.5;
float DAngleRoll = 0.007, DAnglePitch = 0.007;
float PRateRoll = 0.625, PRatePitch = 0.625;
float IRateRoll = 2.1, IRatePitch = 2.1;
float DRateRoll = 0.0088, DRatePitch = DRateRoll;
float PRateYaw = 4, IRateYaw = 3, DRateYaw = 0;

// Throttle limits (ORIGINAL - UNCHANGED)
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;
int led_time = 500;

// PID variables (ORIGINAL - UNCHANGED)
volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;
volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

// Kalman filter variables (ORIGINAL - UNCHANGED)
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// ===== ALTITUDE HOLD VARIABLES (ORIGINAL - UNCHANGED) =====
VL53L0X distanceSensor;
bool altitudeHoldActive = false;
int baselineThrottle = 1500;

// Target management
float altitudeTarget = 0;
bool targetSet = false;

// Reference filtering variables (21-point circular buffer)
int16_t altitudeBuffer[21];  // Stores last 21 readings
int bufferIndex = 0;         // Current position in buffer
int32_t bufferSum = 0;       // Running sum of buffer
float groundReference = 0;   // What "ground level" looks like
float filteredAltitude = 0;  // Final smooth altitude value

// Altitude PID variables
float altP = 0.3;        // Proportional gain
float altI = 0.05;       // Integral gain  
float altD = 0.15;       // Derivative gain
float altIntegral = 0;   // Integral accumulator
float altPrevError = 0;  // Previous error for derivative
float altOutput = 0;     // PID output

// Current sensor reading (shared with FreeRTOS task)
volatile float currentDistance = 0;
volatile bool newDistanceAvailable = false;

// Arming variables (ORIGINAL - UNCHANGED)
bool armed = false;
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000;

// Hardware (ORIGINAL - UNCHANGED)
const int pwmFrequency = 20000;
const int pwmResolution = 8;
const int motor1Channel = 11, motor2Channel = 10;
const int motor3Channel = 9, motor4Channel = 8;
const float t = 0.004;

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== SIMPLE FREERTOS FOR ALTITUDE SENSOR ONLY =====
SemaphoreHandle_t altitudeMutex;  // Only one mutex for altitude data

// ===== FREERTOS TASK: VL53L0X ALTITUDE SENSOR READING ONLY =====
void altitudeSensorTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz (50ms period)
  
  Serial.println("📏 Altitude Sensor Task Started (20Hz)");
  
  for (;;) {
    // Wait for next cycle (20Hz)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Read VL53L0X distance sensor
    float newDistance = distanceSensor.readRangeContinuousMillimeters() / 10.0; // Convert to cm
    
    // Handle sensor errors
    if (distanceSensor.timeoutOccurred() || newDistance > 200) {
      newDistance = 200;
    }
    if (newDistance < 3) {
      newDistance = 3;
    }
    
    // Safely update the shared variable
    if (xSemaphoreTake(altitudeMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      currentDistance = newDistance;
      newDistanceAvailable = true;
      xSemaphoreGive(altitudeMutex);
    }
  }
}

// ===== PPM INTERRUPT (ORIGINAL - UNCHANGED) =====
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

// ===== READ RECEIVER (ORIGINAL - UNCHANGED) =====
void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// ===== KALMAN FILTER (ORIGINAL - UNCHANGED) =====
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4);
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}

// ===== SAFE DISTANCE READING FROM FREERTOS TASK =====
bool getNewDistance(float* distance) {
  bool hasNewData = false;
  
  if (xSemaphoreTake(altitudeMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    if (newDistanceAvailable) {
      *distance = currentDistance;
      newDistanceAvailable = false;
      hasNewData = true;
    }
    xSemaphoreGive(altitudeMutex);
  }
  
  return hasNewData;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  strip.begin();
  strip.show();

  // Flash LED for startup (ORIGINAL - UNCHANGED)
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(led_time);
    digitalWrite(LED_BUILTIN, LOW);
    delay(led_time);
  }
  
  // ===== CREATE SIMPLE FREERTOS MUTEX =====
  altitudeMutex = xSemaphoreCreateMutex();
  if (altitudeMutex == NULL) {
    Serial.println("❌ Failed to create altitude mutex!");
    while(1);
  }
  Serial.println("✅ Altitude mutex created");
  
  // Setup PPM (ORIGINAL - UNCHANGED)
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  // Setup I2C and MPU6050 (ORIGINAL - UNCHANGED)
  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ===== INITIALIZE VL53L0X (ORIGINAL CODE) =====
  if (!distanceSensor.init()) {
    strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red = sensor failed
    strip.show();
    Serial.println("❌ VL53L0X sensor failed!");
    delay(2000);
  } else {
    // Configure sensor (ORIGINAL SETTINGS)
    distanceSensor.setTimeout(100);
    distanceSensor.setMeasurementTimingBudget(20000);
    distanceSensor.startContinuous();
    
    // Initialize altitude buffer (ORIGINAL CODE)
    for (int i = 0; i < 21; i++) {
      altitudeBuffer[i] = 0;
    }
    
    // CALIBRATE GROUND REFERENCE (ORIGINAL ALGORITHM) 
    Serial.println("📏 Calibrating altitude...");
    for (int cal = 0; cal < 150; cal++) {
      // Read sensor directly during calibration
      float calDistance = distanceSensor.readRangeContinuousMillimeters() / 10.0;
      
      // Handle errors
      if (distanceSensor.timeoutOccurred() || calDistance > 200) {
        calDistance = 200;
      }
      if (calDistance < 3) {
        calDistance = 3;
      }
      
      // Store in circular buffer (scaled down)
      altitudeBuffer[bufferIndex] = calDistance / 10;
      bufferSum += altitudeBuffer[bufferIndex];
      bufferSum -= altitudeBuffer[(bufferIndex + 1) % 21];
      bufferIndex++;
      if (bufferIndex == 21) bufferIndex = 0;
      
      // Update ground reference (heavy filtering)
      groundReference = groundReference * 0.7f + bufferSum * 0.15f;
      
      delay(10);
      if (cal % 25 == 0) Serial.print(".");
    }
    
    Serial.println(" ✅ Calibration complete!");
    Serial.print("Ground reference: ");
    Serial.println(groundReference * 10);
    
    strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue = success
    strip.show();
    delay(500);
  }

  // Setup motors (ORIGINAL - UNCHANGED)
  ledcAttach(motor1Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor2Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor3Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor4Channel, pwmFrequency, pwmResolution);

  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);

  // Calibration Values (ORIGINAL - UNCHANGED)
  RateCalibrationRoll=2.80;
  RateCalibrationPitch=-1.86;
  RateCalibrationYaw=-1.47;
  AccXCalibration=0.04;
  AccYCalibration=0.01;
  AccZCalibration=-0.09;

  // ===== CREATE SIMPLE FREERTOS TASK FOR ALTITUDE SENSOR ONLY =====
  xTaskCreatePinnedToCore(
    altitudeSensorTask,       // Task function
    "AltitudeSensor",         // Name  
    2048,                     // Stack size
    NULL,                     // Parameters
    1,                        // Low priority (higher number = higher priority)
    NULL,                     // Task handle
    0                         // Core 0 (separate from main loop on core 1)
  );
  
  Serial.println("✅ Simple altitude sensor task created on Core 0");

  // Ready! (ORIGINAL - UNCHANGED)
  strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green = ready
  strip.show();
  delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  LoopTimer = micros();
}

void loop() {
  // ===== MAIN LOOP - ORIGINAL CODE WITH MINIMAL CHANGES =====
  
  read_receiver(channelValues);

  // ===== UPDATE ALTITUDE READING (Modified to use FreeRTOS data) =====
  float newSensorDistance;
  if (getNewDistance(&newSensorDistance)) {
    // Process new distance data using original algorithm
    
    // Update circular buffer (ORIGINAL ALGORITHM)
    altitudeBuffer[bufferIndex] = newSensorDistance / 10;
    bufferSum += altitudeBuffer[bufferIndex];                    // Add new reading
    bufferSum -= altitudeBuffer[(bufferIndex + 1) % 21];        // Remove oldest reading
    bufferIndex++;
    if (bufferIndex == 21) bufferIndex = 0;
    
    // Apply final filtering based on altitude hold state (ORIGINAL CODE)
    if (!altitudeHoldActive) {
      // Not in altitude hold - track current altitude
      filteredAltitude = filteredAltitude * 0.7f + bufferSum * 0.15f;
    } else {
      // In altitude hold - filter sensor reading
      filteredAltitude = filteredAltitude * 0.7f + bufferSum * 0.15f;
    }
  }

  // ===== ALTITUDE HOLD SWITCHING (ORIGINAL - UNCHANGED) =====
  bool altHoldSwitch = (channelValues[ALT_HOLD_CHANNEL] > 1700);
  
  // Turn OFF altitude hold
  if (!altHoldSwitch && altitudeHoldActive) {
    altitudeHoldActive = false;
    altOutput = 0;
    targetSet = false;  // Reset target for next activation
    Serial.println("🔓 Altitude Hold OFF");
  }
  
  // Turn ON altitude hold
  if (altHoldSwitch && !altitudeHoldActive) {
    altitudeHoldActive = true;
    
    // Reset PID
    altOutput = 0;
    altIntegral = 0;
    altPrevError = 0;
    
    // Remember current throttle and altitude
    baselineThrottle = channelValues[2];
    altitudeTarget = filteredAltitude;
    targetSet = true;
    
    Serial.print("🔒 Altitude Hold ON - Target: ");
    Serial.print(altitudeTarget * 10);
    Serial.print("cm, Throttle: ");
    Serial.println(baselineThrottle);
  }

  // ===== ARMING LOGIC (ORIGINAL - UNCHANGED) =====
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
        armDisarmTimer = 0;
      }
    } else if (armed && channelValues[3] < 1100) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = false;
        strip.setPixelColor(0, strip.Color(255, 255, 255)); // White = disarmed
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
  
  // If not armed, stop everything (ORIGINAL - UNCHANGED)
  if (!armed) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    altIntegral = 0;
    
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

  // Show status LED (ORIGINAL - UNCHANGED)
  if (altitudeHoldActive) {
    strip.setPixelColor(0, strip.Color(0, 0, 50)); // Blue = altitude hold
  } else {
    strip.setPixelColor(0, strip.Color(0, 50, 0)); // Green = normal
  }
  strip.show();

  // ===== READ IMU DATA (ORIGINAL - UNCHANGED) =====
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

  // Calculate angles (ORIGINAL - UNCHANGED)
  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
  
  // Kalman filtering (ORIGINAL - UNCHANGED)
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = constrain(Kalman1DOutput[0], -20, 20);
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = constrain(Kalman1DOutput[0], -20, 20);
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Set desired angles (ORIGINAL - UNCHANGED)
  DesiredAngleRoll  = 0.1 * (channelValues[0] - 1500);
  DesiredAnglePitch = 0.1 * (channelValues[1] - 1500);
  
  // ===== ALTITUDE HOLD PID CALCULATION (ORIGINAL - UNCHANGED) =====
  if (altitudeHoldActive && targetSet) {
      // Calculate error
      float altError = altitudeTarget - filteredAltitude;
      
      // ADD DEADBAND - ignore small errors (prevents jitter)
      if (abs(altError) < 0.02) {  // 2cm deadband
        altError = 0;
      }
      
      // P term
      float pTerm = altP * altError;
      
      // I term with windup protection
      altIntegral += altI * altError * t;
      altIntegral = constrain(altIntegral, -200, 200);
      
      // D term
      float dTerm = altD * (altError - altPrevError) / t;
      
      // Combine PID terms
      altOutput = pTerm + altIntegral + dTerm;
      altOutput = constrain(altOutput, -200, 200);
      
      // Update for next loop
      altPrevError = altError;
      
      // Apply altitude correction to throttle
      InputThrottle = baselineThrottle + altOutput;
      
      // Allow pilot to adjust target altitude with stick
      float stickInput = channelValues[2] - baselineThrottle;
      if (abs(stickInput) > 50) {
        baselineThrottle = channelValues[2];
      }
      
    } else {
      // Normal throttle control
      InputThrottle = channelValues[2];
      altOutput = 0;
    }

  DesiredRateYaw = 0.15 * (channelValues[3] - 1500);

  // ===== ANGLE PID FOR ROLL (ORIGINAL - UNCHANGED) =====
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // ===== ANGLE PID FOR PITCH (ORIGINAL - UNCHANGED) =====
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // ===== RATE PID CALCULATIONS (ORIGINAL - UNCHANGED) =====
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

  // Limit throttle (ORIGINAL - UNCHANGED)
  InputThrottle = constrain(InputThrottle, ThrottleCutOff, 2000);

  // ===== MOTOR MIXING (ORIGINAL - UNCHANGED) =====
  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw;
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw;
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw;
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw;

  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 2000);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 2000);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 2000);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 2000);

  // Reset if throttle too low and not in altitude hold (ORIGINAL - UNCHANGED)
  if (channelValues[2] < 1030 && !altitudeHoldActive) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    altIntegral = 0;
  }

  // Convert to PWM and output (ORIGINAL - UNCHANGED)
  int pwm1 = constrain(map(MotorInput1, 1000, 2000, 0, 255), 0, 255);
  int pwm2 = constrain(map(MotorInput2, 1000, 2000, 0, 255), 0, 255);
  int pwm3 = constrain(map(MotorInput3, 1000, 2000, 0, 255), 0, 255);
  int pwm4 = constrain(map(MotorInput4, 1000, 2000, 0, 255), 0, 255);

  ledcWrite(motor1Channel, pwm1);
  ledcWrite(motor2Channel, pwm2);
  ledcWrite(motor3Channel, pwm3);
  ledcWrite(motor4Channel, pwm4);

  // Uncomment for debugging:
  // Serial.print("Alt: "); Serial.print(filteredAltitude * 10);
  // Serial.print(" Out: "); Serial.println(altOutput);

  // Maintain exact 250Hz loop rate (ORIGINAL - UNCHANGED)
  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}

/*
 * ===== SIMPLE FREERTOS IMPLEMENTATION SUMMARY =====
 * 
 * 🎯 WHAT CHANGED (Minimal Changes):
 * 
 * ✅ ADDED:
 * - ONE FreeRTOS task for VL53L0X sensor reading only (20Hz on Core 0)
 * - ONE mutex to protect shared distance variable
 * - Simple getNewDistance() function to safely read sensor data
 * 
 * 🔄 KEPT EXACTLY THE SAME:
 * - Your entire main loop structure and timing (250Hz)
 * - All flight control, PID calculations, and motor mixing
 * - All arming/disarming logic and controls
 * - All IMU sensor reading in main loop
 * - All altitude hold PID calculations in main loop
 * - Original altitude filtering algorithm
 * - All variable names and structures
 * 
 * 🚁 HOW IT WORKS:
 * 
 * Core 0: Simple FreeRTOS task reads VL53L0X every 50ms
 *         ↓ (through mutex-protected variable)
 * Core 1: Your original main loop checks for new data every 4ms
 *         ↓ (processes with original algorithm)
 *         Your original flight control continues unchanged
 * 
 * 📊 BENEFITS:
 * - VL53L0X sensor reading won't block your main flight loop
 * - Sensor reads at optimal 20Hz rate independently  
 * - Zero impact on flight control timing and responsiveness
 * - If altitude sensor has issues, flight control continues normally
 * - Easy to disable - just comment out the task creation
 * 
 * 🎮 CONTROLS (100% Same as Original):
 * - Arm: Throttle low + Yaw right (hold 1 second)
 * - Disarm: Throttle low + Yaw left (hold 1 second)
 * - Altitude Hold: Channel 5 switch high
 * - All stick responses identical to your original code
 * 
 * 🔧 MINIMAL COMPLEXITY:
 * - Only 1 FreeRTOS task (not 5 like before)
 * - Only 1 mutex (not 4 like before)  
 * - Only 50 lines of FreeRTOS code added
 * - 99% of your code remains unchanged
 * - No queues, no complex synchronization
 * 
 * 🛡️ SAFETY:
 * - If FreeRTOS task fails, altitude hold simply stops working
 * - Flight control continues normally in all cases
 * - Mutex has timeout to prevent blocking
 * - Original arming/disarming works exactly the same
 * 
 * RESULT: Your exact original flight controller + smooth background 
 * altitude sensor reading without any flight control changes! 🚁✨
 */
