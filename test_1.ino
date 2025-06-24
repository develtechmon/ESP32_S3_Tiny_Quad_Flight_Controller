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

// ===== Advanced VL53L0X Altitude Hold Definitions (Method 1 Style) =====
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

// ===== Advanced VL53L0X Altitude Hold Variables (Method 1 Implementation) =====
// Multiple filtering layers like STM32 version
int16_t distance_z_average_short[26];        // Short-term average buffer (like acc_z_average_short)
int16_t distance_z_average_long[51];         // Long-term average buffer (like acc_z_average_long)
uint8_t distance_z_average_short_rotating_mem_location = 0;
uint8_t distance_z_average_long_rotating_mem_location = 0;
int32_t distance_z_average_short_total = 0;
int32_t distance_z_average_long_total = 0;

// Pressure-style filtering variables adapted for distance
float actual_distance, actual_distance_slow, actual_distance_fast, actual_distance_diff;
float ground_distance, altitude_hold_distance, distance_integrated;
int32_t distance_rotating_mem[50], distance_total_average;
uint8_t distance_rotating_mem_location = 0;
float distance_rotating_mem_actual;

// Enhanced PID controller (STM32 style gains and limits)
float pid_p_gain_altitude = 1.4;           // Higher precision like STM32 version
float pid_i_gain_altitude = 0.2;           // More conservative integral
float pid_d_gain_altitude = 0.75;          // Stronger derivative for stability
int pid_max_altitude = 400;                // Higher output range like STM32

float pid_error_gain_altitude = 1.0;       // Error scaling factor
float pid_throttle_gain_altitude = 1.0;    // Throttle compensation
bool takeoff_detected = false;             // Takeoff detection like STM32
bool altitude_hold_active = false;         // Altitude hold state
bool manual_altitude_change = false;       // Manual override detection

// PID variables with STM32 naming convention
float pid_i_mem_altitude = 0;
float pid_altitude_setpoint = 0;
float pid_altitude_input = 0;
float pid_output_altitude = 0;
float pid_last_altitude_d_error = 0;
int16_t manual_takeoff_throttle = 0;       // Auto takeoff detection
int takeoff_throttle = 0;                  // Detected takeoff throttle

// Battery compensation (like STM32 version)
float battery_compensation = 40.0;         // Voltage drop compensation

// VL53L0X sensor object
VL53L0X distanceSensor;

// Enhanced timing and measurement variables
unsigned long distance_timer = 0;
unsigned long altitude_timer = 0;
bool distance_measurement_ready = false;
uint8_t measurement_cycle = 0;             // Cycle counter for timing

// Throttle limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;
int led_time = 500;

// PID calculation variables for Roll, Pitch, Yaw
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

// Battery Parameters
float Voltage, battery_voltage;

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
uint8_t start = 0;             // STM32 style start variable
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000; // milliseconds required to hold stick

// ===== Advanced VL53L0X Functions (Method 1 Implementation) =====

// Initialize VL53L0X with advanced filtering system
void setup_advanced_vl53l0x_altitude() {
  Serial.println("🚁 Initializing Advanced VL53L0X Altitude System...");
  
  // Initialize all filter buffers to zero
  for (int i = 0; i < 26; i++) {
    distance_z_average_short[i] = 0;
  }
  for (int i = 0; i < 51; i++) {
    distance_z_average_long[i] = 0;
  }
  for (int i = 0; i < 50; i++) {
    distance_rotating_mem[i] = 0;
  }
  
  // Reset all variables
  distance_z_average_short_total = 0;
  distance_z_average_long_total = 0;
  distance_total_average = 0;
  ground_distance = 0;
  actual_distance = 0;
  actual_distance_slow = 0;
  actual_distance_fast = 0;
  
  // Configure VL53L0X for high-speed continuous measurement
  distanceSensor.setTimeout(100);
  distanceSensor.setMeasurementTimingBudget(20000); // 20ms = 50Hz
  distanceSensor.startContinuous();
  
  Serial.println("📏 Calibrating ground reference (like STM32 barometer calibration)...");
  
  // Calibration phase - 100 readings like STM32 version
  for (int start_cal = 0; start_cal < 100; start_cal++) {
    read_vl53l0x_distance();
    delay(4); // Same timing as STM32 main loop
    
    if (start_cal % 20 == 0) {
      Serial.print(".");
    }
  }
  
  // Set initial ground reference
  ground_distance = actual_distance_slow;
  altitude_hold_distance = ground_distance;
  
  // Pre-load all averaging buffers with current distance
  for (int i = 0; i < 26; i++) {
    distance_z_average_short[i] = actual_distance * 10; // Scale like STM32
  }
  for (int i = 0; i < 51; i++) {
    distance_z_average_long[i] = actual_distance * 10;
  }
  for (int i = 0; i < 50; i++) {
    distance_rotating_mem[i] = actual_distance * 10;
  }
  
  distance_z_average_short_total = actual_distance * 10 * 26;
  distance_z_average_long_total = actual_distance * 10 * 51;
  distance_total_average = actual_distance * 10 * 50;
  
  Serial.println(" ✅ Advanced VL53L0X altitude system ready!");
  Serial.print("Ground reference: ");
  Serial.print(ground_distance);
  Serial.println("cm");
}

// Enhanced distance reading with multiple filtering layers (STM32 style)
void read_vl53l0x_distance() {
  // Read raw distance (equivalent to reading barometer)
  float raw_distance = distanceSensor.readRangeContinuousMillimeters() / 10.0; // Convert to cm
  
  // Handle sensor errors gracefully
  if (distanceSensor.timeoutOccurred() || raw_distance > 200 || raw_distance < 3) {
    // Keep previous reading on error (like STM32 error handling)
    raw_distance = actual_distance;
  }
  
  // Apply multiple complementary filters (like STM32 pressure filtering)
  actual_distance = raw_distance;
  actual_distance_slow = actual_distance_slow * 0.985 + actual_distance * 0.015;  // Very slow filter
  actual_distance_fast = actual_distance_fast * 0.8 + actual_distance * 0.2;     // Fast filter
  actual_distance_diff = actual_distance_fast - actual_distance_slow;            // Difference for rate
  
  // Short-term averaging (26 samples like STM32)
  distance_z_average_short_total -= distance_z_average_short[distance_z_average_short_rotating_mem_location];
  distance_z_average_short[distance_z_average_short_rotating_mem_location] = actual_distance * 10; // Scale for precision
  distance_z_average_short_total += distance_z_average_short[distance_z_average_short_rotating_mem_location];
  distance_z_average_short_rotating_mem_location++;
  if (distance_z_average_short_rotating_mem_location == 26) {
    distance_z_average_short_rotating_mem_location = 0;
  }
  
  // Long-term averaging (51 samples like STM32)
  distance_z_average_long_total -= distance_z_average_long[distance_z_average_long_rotating_mem_location];
  distance_z_average_long[distance_z_average_long_rotating_mem_location] = actual_distance * 10;
  distance_z_average_long_total += distance_z_average_long[distance_z_average_long_rotating_mem_location];
  distance_z_average_long_rotating_mem_location++;
  if (distance_z_average_long_rotating_mem_location == 51) {
    distance_z_average_long_rotating_mem_location = 0;
  }
  
  // Rotating memory for additional smoothing (50 samples like STM32)
  distance_total_average -= distance_rotating_mem[distance_rotating_mem_location];
  distance_rotating_mem[distance_rotating_mem_location] = actual_distance * 10;
  distance_total_average += distance_rotating_mem[distance_rotating_mem_location];
  distance_rotating_mem_location++;
  if (distance_rotating_mem_location == 50) {
    distance_rotating_mem_location = 0;
  }
  
  // Calculate averaged distance for PID input
  distance_rotating_mem_actual = distance_total_average / 50.0;
}

// Vertical acceleration calculations (adapted from STM32 for distance rate)
void vertical_distance_calculations() {
  // Calculate distance rate of change (similar to vertical acceleration in STM32)
  static float previous_distance = 0;
  static float distance_velocity = 0;
  
  // Simple velocity calculation
  distance_velocity = (actual_distance_fast - previous_distance) / t;
  previous_distance = actual_distance_fast;
  
  // Integration for smooth distance tracking (like STM32 acc_alt_integrated)
  distance_integrated += distance_velocity * t;
  
  // Limit integration drift
  if (abs(distance_integrated) > 50) {
    distance_integrated *= 0.95; // Slowly reduce accumulated error
  }
}

// Advanced altitude hold PID controller (STM32 implementation)
void calculate_advanced_altitude_pid() {
  // Only calculate if altitude hold is active and armed
  if (!altitude_hold_active || start != 2) {
    pid_output_altitude = 0;
    return;
  }
  
  // Use the rotating memory average as PID input (like STM32)
  pid_altitude_input = distance_rotating_mem_actual / 10.0;
  
  // Calculate error (target - current)
  float pid_error_temp = pid_altitude_setpoint - pid_altitude_input;
  
  // Apply error gain (like STM32)
  pid_error_temp *= pid_error_gain_altitude;
  
  // Integral term with windup protection (STM32 style)
  pid_i_mem_altitude += pid_i_gain_altitude * pid_error_temp;
  if (pid_i_mem_altitude > pid_max_altitude) {
    pid_i_mem_altitude = pid_max_altitude;
  } else if (pid_i_mem_altitude < -pid_max_altitude) {
    pid_i_mem_altitude = -pid_max_altitude;
  }
  
  // Complete PID calculation (STM32 format)
  pid_output_altitude = (pid_p_gain_altitude * pid_error_temp) + 
                       pid_i_mem_altitude + 
                       (pid_d_gain_altitude * (pid_error_temp - pid_last_altitude_d_error));
  
  // Apply throttle gain (like STM32)
  pid_output_altitude *= pid_throttle_gain_altitude;
  
  // Output limiting (STM32 style)
  if (pid_output_altitude > pid_max_altitude) {
    pid_output_altitude = pid_max_altitude;
  } else if (pid_output_altitude < -pid_max_altitude) {
    pid_output_altitude = -pid_max_altitude;
  }
  
  // Store error for next derivative calculation
  pid_last_altitude_d_error = pid_error_temp;
}

// Takeoff detection (STM32 style)
void detect_takeoff() {
  static bool takeoff_detection_complete = false;
  
  // Manual takeoff throttle override
  if (manual_takeoff_throttle > 0) {
    takeoff_throttle = manual_takeoff_throttle;
    takeoff_detected = true;
    takeoff_detection_complete = true;
    return;
  }
  
  // Auto takeoff detection based on throttle increase
  if (!takeoff_detection_complete && start == 2) {
    static int throttle_increase_counter = 0;
    static int baseline_throttle_reading = 0;
    
    if (baseline_throttle_reading == 0) {
      baseline_throttle_reading = channelValues[2];
    }
    
    // Detect significant throttle increase
    if (channelValues[2] > baseline_throttle_reading + 150) {
      throttle_increase_counter++;
      if (throttle_increase_counter > 20) { // Stable high throttle for 20 loops
        takeoff_throttle = channelValues[2] - 1500; // Store offset
        takeoff_detected = true;
        takeoff_detection_complete = true;
        Serial.println("🚁 Takeoff detected! Throttle offset: " + String(takeoff_throttle));
      }
    } else {
      throttle_increase_counter = 0;
    }
  }
}

// Advanced start/stop/takeoff logic (STM32 style)
void start_stop_takeoff() {
  // Starting sequence: throttle low and yaw left
  if (channelValues[2] < 1050 && channelValues[3] < 1050) {
    start = 1;
  }
  
  // Complete start: yaw stick back to center
  if (start == 1 && channelValues[2] < 1050 && channelValues[3] > 1450) {
    start = 2;
    
    // Reset PID controllers for clean start (STM32 style)
    pid_i_mem_altitude = 0;
    pid_last_altitude_d_error = 0;
    pid_output_altitude = 0;
    
    // Reset angle PIDs too
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    
    Serial.println("🟢 Motors STARTED");
  }
  
  // Stopping: throttle low and yaw right
  if (start == 2 && channelValues[2] < 1050 && channelValues[3] > 1950) {
    start = 0;
    takeoff_detected = false;
    altitude_hold_active = false;
    Serial.println("🔴 Motors STOPPED");
  }
  
  // Detect takeoff if motors are running
  if (start == 2) {
    detect_takeoff();
  }
}

// Flight mode management (simplified STM32 approach)
void manage_flight_modes() {
  static bool previous_alt_hold_switch = false;
  bool alt_hold_switch = (channelValues[ALT_HOLD_CHANNEL] > 1700);
  
  // Altitude hold activation
  if (alt_hold_switch && !previous_alt_hold_switch && start == 2 && takeoff_detected) {
    altitude_hold_active = true;
    
    // Set current altitude as target (STM32 style)
    pid_altitude_setpoint = pid_altitude_input;
    altitude_hold_distance = actual_distance_slow;
    
    // Reset PID controller for smooth engagement
    pid_i_mem_altitude = 0;
    pid_last_altitude_d_error = 0;
    pid_output_altitude = 0;
    
    Serial.println("🔒 Advanced Altitude Hold ACTIVATED");
    Serial.print("Target altitude: ");
    Serial.print(pid_altitude_setpoint);
    Serial.println("cm");
  }
  
  // Altitude hold deactivation
  if (!alt_hold_switch && previous_alt_hold_switch) {
    altitude_hold_active = false;
    pid_output_altitude = 0;
    Serial.println("🔓 Advanced Altitude Hold DEACTIVATED");
  }
  
  previous_alt_hold_switch = alt_hold_switch;
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
void read_battery_voltage(void) {
  // Enhanced battery reading with complementary filter (STM32 style)
  static float raw_voltage = 0;
  raw_voltage = (float)analogRead(1) / 237; // GPIO1
  
  // Apply complementary filter like STM32
  battery_voltage = battery_voltage * 0.92 + raw_voltage * 0.08;
  Voltage = battery_voltage; // Keep compatibility
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
    Serial.println("❌ Failed to detect and initialize VL53L0X sensor!");
    delay(2000);
  } else {
    // Configure VL53L0X for optimal performance
    distanceSensor.setTimeout(100);
    distanceSensor.setMeasurementTimingBudget(20000); // 20ms = 50Hz
    distanceSensor.startContinuous();
    
    // Initialize advanced altitude system
    setup_advanced_vl53l0x_altitude();
    
    // Indicate successful sensor initialization
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
  RateCalibrationRoll=2.80;
  RateCalibrationPitch=-1.86;
  RateCalibrationYaw=-1.47;
  AccXCalibration=0.04;
  AccYCalibration=0.01;
  AccZCalibration=-0.09;

  // Initialize battery voltage
  battery_voltage = 12.0; // Initial value
  
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

  // ----- Enhanced VL53L0X distance reading with advanced filtering -----
  read_vl53l0x_distance();
  
  // ----- Vertical distance calculations (like STM32 vertical acceleration) -----
  vertical_distance_calculations();
  
  // ----- STM32-style start/stop/takeoff management -----
  start_stop_takeoff();
  
  // ----- Flight mode management -----
  manage_flight_modes();
  
  // ----- Calculate advanced altitude PID -----
  calculate_advanced_altitude_pid();

  // ----- Enhanced battery monitoring -----
  read_battery_voltage();

  // If not started, cut motors and reset everything
  if (start != 2) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    pid_i_mem_altitude = 0;
    
    // Update motor outputs
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
    
    // Show status with LED
    if (start == 1) {
      // Yellow during start sequence
      strip.setPixelColor(0, strip.Color(255, 255, 0));
    } else {
      // Red when stopped
      strip.setPixelColor(0, strip.Color(255, 0, 0));
    }
    strip.show();
    
    while (micros() - LoopTimer < (t * 1000000));
    LoopTimer = micros();
    return;
  }

  // Show flight status with LED
  if (altitude_hold_active) {
    // Blue LED when altitude hold is active
    strip.setPixelColor(0, strip.Color(0, 0, 100));
  } else if (takeoff_detected) {
    // Green LED when flying but no altitude hold
    strip.setPixelColor(0, strip.Color(0, 100, 0));
  } else {
    // Orange LED when motors running but not taken off
    strip.setPixelColor(0, strip.Color(100, 50, 0));
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
  
  // ----- Advanced Throttle Control (STM32 Style) -----
  if (takeoff_detected && start == 2) {
    // Base throttle calculation (like STM32)
    int throttle = channelValues[2] + takeoff_throttle;
    
    if (altitude_hold_active) {
      // Advanced altitude hold mode (STM32 implementation)
      throttle = 1500 + takeoff_throttle + pid_output_altitude;
      
      // Allow manual altitude adjustment (STM32 style)
      static int manual_throttle = 0;
      int stick_input = channelValues[2] - 1500;
      if (abs(stick_input) > 50) {
        manual_throttle = stick_input / 4; // Gentle adjustment
        manual_altitude_change = true;
      } else {
        manual_altitude_change = false;
      }
      
      throttle += manual_throttle;
    }
    
    InputThrottle = throttle;
  } else {
    // Normal throttle before takeoff
    InputThrottle = channelValues[2];
  }
  
  // Throttle limiting (STM32 style)
  if (InputThrottle > 1800) InputThrottle = 1800; // Reserve headroom for control
  
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

  // ----- Motor Mixing (STM32 Style) -----
  MotorInput1 = InputThrottle - InputPitch + InputRoll - InputYaw;  // Front-right - CCW
  MotorInput2 = InputThrottle + InputPitch + InputRoll + InputYaw;  // Rear-right - CW
  MotorInput3 = InputThrottle + InputPitch - InputRoll - InputYaw;  // Rear-left - CCW
  MotorInput4 = InputThrottle - InputPitch - InputRoll + InputYaw;  // Front-left - CW

  // ----- Battery Compensation (STM32 Implementation) -----
  if (battery_voltage < 12.40 && battery_voltage > 6.0) {
    float compensation_factor = (12.40 - battery_voltage) * battery_compensation;
    MotorInput1 += compensation_factor;
    MotorInput2 += compensation_factor;
    MotorInput3 += compensation_factor;
    MotorInput4 += compensation_factor;
  }

  // ----- Motor Output Limiting (STM32 Style) -----
  // Keep motors running at minimum idle speed
  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  // Maximum output limiting
  if (MotorInput1 > 2000) MotorInput1 = 2000;
  if (MotorInput2 > 2000) MotorInput2 = 2000;
  if (MotorInput3 > 2000) MotorInput3 = 2000;
  if (MotorInput4 > 2000) MotorInput4 = 2000;

  // If throttle is too low and altitude hold is not active, reset and cut motors
  if (channelValues[2] < 1030 && !altitude_hold_active) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    pid_i_mem_altitude = 0;
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

  // ----- Debug Output (Uncomment for tuning) -----
  /*
  Serial.print("Alt: "); Serial.print(pid_altitude_input);
  Serial.print(" Tgt: "); Serial.print(pid_altitude_setpoint);
  Serial.print(" Out: "); Serial.print(pid_output_altitude);
  Serial.print(" Dist: "); Serial.print(actual_distance);
  Serial.print(" Slow: "); Serial.print(actual_distance_slow);
  Serial.print(" Fast: "); Serial.println(actual_distance_fast);
  */

  // ----- Maintain 250Hz Loop Rate (4ms) -----
  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}
