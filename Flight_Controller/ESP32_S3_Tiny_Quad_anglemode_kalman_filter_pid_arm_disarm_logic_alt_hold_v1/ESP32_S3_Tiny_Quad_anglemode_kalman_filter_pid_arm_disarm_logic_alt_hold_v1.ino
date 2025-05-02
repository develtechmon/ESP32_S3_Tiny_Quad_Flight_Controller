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

// Altitude PID parameters
float PAltitude = 1.2;   // Increased proportional gain (was 0.8)
float IAltitude = 0.08;  // Increased integral gain (was 0.05)
float DAltitude = 0.2;   // Derivative gain

// New altitude control parameters
float altitudeDropBias = 1.5;        // More aggressive response to dropping
float hoverThrottleLearnRate = 0.01; // Auto-learn the hover throttle
float minThrottlePercent = 0.95;     // Minimum throttle allowed as a percentage of hover throttle
int dropDetectionCounter = 0;        // Counter for consecutive drops in altitude
int dropDetectionThreshold = 3;      // Threshold for drop detection

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

// Altitude control variables
VL53L0X distanceSensor;
volatile bool altitudeHoldEnabled = false;
volatile float currentAltitude = 0;
volatile float desiredAltitude = 0;
volatile float altitudeError = 0;
volatile float previousAltitudeError = 0;
volatile float altitudeIterm = 0;
volatile float altitudeOutput = 0;
volatile float throttleHover = 1600;  // Base throttle needed to hover - will be learned

// For altitude filtering
const int ALTITUDE_FILTER_SIZE = 5;
float altitudeReadings[ALTITUDE_FILTER_SIZE];
int altitudeFilterIndex = 0;

// Variables for non-blocking sensor reading
uint8_t sensorReadState = 0;          // 0=idle, 1=reading
unsigned long sensorReadStartTime = 0;
unsigned int sensorReadCounter = 0;
const unsigned int SENSOR_READ_INTERVAL = 6; // Read sensor more frequently (was 8)

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

// ===== PPM Interrupt Handler =====
// Use IRAM_ATTR for faster interrupt handling on the ESP32.
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    // A long pulse is assumed to be the sync pulse – reset channel index.
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    // Store a valid channel pulse (constrained to valid range)
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

// ===== Process altitude reading and apply filter =====
void processAltitudeReading(uint16_t rawDistance) {
  // Check for timeout or out-of-range reading
  if (rawDistance > 2000) {
    // Out of range, keep previous value
    return;
  }
  
  // Convert to meters
  float distance = rawDistance / 1000.0;
  
  // Apply weighted moving average filter - more weight to previous values for stability
  // Check if the reading is too different from our current altitude
  float maxChange = 0.1; // Maximum change allowed in one reading (10cm)
  
  if (abs(distance - currentAltitude) > maxChange) {
    // Outlier detection - limit the change rate
    if (distance < currentAltitude) {
      // When altitude appears to suddenly decrease, be conservative
      distance = currentAltitude - maxChange;
    } else {
      // When altitude appears to suddenly increase, can be more accepting
      distance = currentAltitude + maxChange;
    }
  }
  
  // Apply moving average filter with more weight to existing value for stability
  altitudeReadings[altitudeFilterIndex] = distance;
  altitudeFilterIndex = (altitudeFilterIndex + 1) % ALTITUDE_FILTER_SIZE;
  
  // Calculate weighted average - more weight to previous values
  float sum = 0;
  float weights = 0;
  float decayFactor = 0.8; // Weight decay factor - higher means more smoothing
  
  for (int i = 0; i < ALTITUDE_FILTER_SIZE; i++) {
    // Calculate index going backward from current
    int idx = (altitudeFilterIndex - 1 - i + ALTITUDE_FILTER_SIZE) % ALTITUDE_FILTER_SIZE;
    float weight = pow(decayFactor, i);
    
    sum += altitudeReadings[idx] * weight;
    weights += weight;
  }
  
  // Gradual update of current altitude for smoother transitions
  currentAltitude = sum / weights;
}

// ===== Altitude Hold PID Controller =====
float altitudePID() {
  // Calculate error
  altitudeError = desiredAltitude - currentAltitude;
  
  // Detect consecutive dropping
  if (altitudeError > 0 && (altitudeError > previousAltitudeError)) {
    dropDetectionCounter++;
    if (dropDetectionCounter > dropDetectionThreshold) {
      // Apply bias to make PID more aggressive when consistently dropping
      altitudeError *= altitudeDropBias;
    }
  } else {
    dropDetectionCounter = 0;
  }
  
  // P term
  float pTerm = PAltitude * altitudeError;
  
  // I term with anti-windup and asymmetric response
  // Accumulate integral faster when dropping (error > 0)
  float integralRate = IAltitude;
  if (altitudeError > 0) {
    integralRate *= 1.5; // Faster integral accumulation when dropping
  }
  
  altitudeIterm += integralRate * altitudeError * t;
  altitudeIterm = constrain(altitudeIterm, -400, 400);
  
  // D term - more damping when rising, less when falling
  float dGain = DAltitude;
  if (altitudeError > 0) {
    dGain *= 0.7; // Less derivative action when dropping (to allow faster recovery)
  }
  
  float dTerm = dGain * ((altitudeError - previousAltitudeError) / t);
  
  // PID output
  float output = pTerm + altitudeIterm + dTerm;
  output = constrain(output, -300, 400); // Asymmetric constraints - allow more positive correction
  
  // Update previous error for next iteration
  previousAltitudeError = altitudeError;
  
  return output;
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
    // Don't halt the program, but indicate the error
    delay(2000);
  } else {
    // Configure VL53L0X for faster measurements with slightly reduced accuracy
    distanceSensor.setTimeout(100); // 100ms timeout
    distanceSensor.setMeasurementTimingBudget(20000); // 20ms per measurement = 50Hz
    
    // Initialize altitude filter
    for (int i = 0; i < ALTITUDE_FILTER_SIZE; i++) {
      altitudeReadings[i] = 0;
    }
    
    // Start the first measurement - start non-blocking continuous mode
    distanceSensor.startContinuous();
    
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

  // Initialize motor outputs to minimum throttle (1000 µs mapped to 0 PWM)
  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);

  // ----- Calibration Values -----
  // ----- 8520 Motor -----
  RateCalibrationRoll  = 2.80;
  RateCalibrationPitch = -1.86;
  RateCalibrationYaw   = -1.47;
  AccXCalibration = 0.04;
  AccYCalibration = 0.01;
  AccZCalibration = 0.09;

  // ---- 712 Motor ----
  // RateCalibrationRoll=-1.50;
  // RateCalibrationPitch=-1.14;
  // RateCalibrationYaw=0.48;
  // AccXCalibration=0.06;
  // AccYCalibration=-0.04;
  // AccZCalibration=-0.00;

  // Green LED to indicate normal startup
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  LoopTimer = micros();
}

void loop() {
  // ----- Handle altitude sensor readings in a non-blocking way -----
  sensorReadCounter++;
  
  // Check for new distance reading periodically to avoid interfering with flight control timing
  if (sensorReadCounter >= SENSOR_READ_INTERVAL) {
    sensorReadCounter = 0;
    
    // Read distance from sensor (non-blocking since we're using continuous mode)
    uint16_t rawDistance = distanceSensor.readRangeContinuousMillimeters();
    
    // Only process valid readings
    if (!distanceSensor.timeoutOccurred()) {
      processAltitudeReading(rawDistance);
    }
  }
  
  // ----- Read receiver values -----
  read_receiver(channelValues);

  // ----- Arming/Disarming Logic -----
  // Check if throttle (channel 2) is low enough to allow arming/disarming
  if (channelValues[2] < 1050) {
    // To arm: yaw (channel 3) high
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
    }
    // To disarm: yaw (channel 3) low
    else if (armed && channelValues[3] < 1100) {
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
    altitudeIterm = 0;
    
    // Map the throttle cutoff to PWM and update motor outputs
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
    return; // Skip the rest of the control loop if disarmed
  }

  // ----- Check if altitude hold is enabled via transmitter switch -----
  altitudeHoldEnabled = (channelValues[ALT_HOLD_CHANNEL] > 1500);
  
  // Show altitude hold status with LED
  if (altitudeHoldEnabled) {
    // Blue LED when altitude hold is active
    strip.setPixelColor(0, strip.Color(0, 0, 50));
  } else {
    // Green LED when armed but no altitude hold
    strip.setPixelColor(0, strip.Color(0, 50, 0));
  }
  strip.show();
  
  // Check if altitude hold was just enabled
  static bool wasAltitudeHoldEnabled = false;
  if (altitudeHoldEnabled && !wasAltitudeHoldEnabled) {
    // Reset integrator when first entering altitude hold
    altitudeIterm = 0;
    dropDetectionCounter = 0;
    
    // Store current throttle as base hover throttle
    // Ensure a minimum throttle level to prevent dropping
    throttleHover = max(channelValues[2], 1350);
  }
  wasAltitudeHoldEnabled = altitudeHoldEnabled;
  
  // When altitude hold is enabled, map throttle position to desired altitude
  if (altitudeHoldEnabled) {
    // Map throttle position (1000-2000) to altitude (0.1-1.9m)
    // Use non-linear mapping for better control at lower altitudes
    if (channelValues[2] < 1500) {
      // More resolution at lower altitudes
      desiredAltitude = map(channelValues[2], 1000, 1500, 10, 90) / 100.0;
    } else {
      // Less resolution at higher altitudes
      desiredAltitude = map(channelValues[2], 1500, 2000, 90, 190) / 100.0;
    }
    
    // Auto-learn the hover throttle when altitude is stable
    if (abs(altitudeError) < 0.05 && abs(previousAltitudeError) < 0.05) {
      // Slowly learn what throttle level maintains altitude
      throttleHover = throttleHover * (1.0 - hoverThrottleLearnRate) + 
                      InputThrottle * hoverThrottleLearnRate;
      
      // Debug hover throttle learning (uncomment if needed)
      // Serial.print("Learning hover: "); Serial.println(throttleHover);
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
  
  // Request gyroscope data
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
  
  // Run simple Kalman filters for both angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Clamp the filtered angles to ±20 degrees
  KalmanAngleRoll = (KalmanAngleRoll > 20) ? 20 : ((KalmanAngleRoll < -20) ? -20 : KalmanAngleRoll);
  KalmanAnglePitch = (KalmanAnglePitch > 20) ? 20 : ((KalmanAnglePitch < -20) ? -20 : KalmanAnglePitch);

  // ----- Set Desired Angles and Throttle from Receiver Inputs -----
  DesiredAngleRoll  = 0.1 * (channelValues[0] - 1500);
  DesiredAnglePitch = 0.1 * (channelValues[1] - 1500);
  
  // If altitude hold is enabled, use PID to control throttle
  if (altitudeHoldEnabled) {
    // Calculate altitude PID output
    altitudeOutput = altitudePID();
    
    // Apply altitude correction to throttle
    float rawThrottle = throttleHover + altitudeOutput;
    
    // Ensure minimum effective throttle (prevent dropping below a certain percentage of hover)
    float minThrottle = throttleHover * minThrottlePercent;
    if (rawThrottle < minThrottle && rawThrottle > ThrottleCutOff) {
      rawThrottle = minThrottle;
    }
    
    // Apply the calculated throttle
    InputThrottle = rawThrottle;
    
    // Debug output (uncomment if needed)
    // Serial.print("T:"); Serial.print(channelValues[2]);
    // Serial.print(" D:"); Serial.print(desiredAltitude, 2);
    // Serial.print(" C:"); Serial.print(currentAltitude, 2);
    // Serial.print(" E:"); Serial.print(altitudeError, 2);
    // Serial.print(" O:"); Serial.print(altitudeOutput);
    // Serial.print(" H:"); Serial.println(throttleHover);
  } else {
    // Use manual throttle control
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
  if (InputThrottle > 2000) { 
    InputThrottle = 2000;
  } else if (InputThrottle < ThrottleIdle && InputThrottle > ThrottleCutOff) {
    // Don't let altitude hold push throttle below idle but above cutoff
    InputThrottle = ThrottleIdle;
  }

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

  // If throttle is too low, reset PID integrals and cut motors
  if (channelValues[2] < 1030 && !altitudeHoldEnabled) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    altitudeIterm = 0;
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
  // Serial.print("Alt: "); Serial.print(currentAltitude);
  // Serial.print(" Des: "); Serial.print(desiredAltitude);
  // Serial.print(" Out: "); Serial.println(altitudeOutput);

  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}
