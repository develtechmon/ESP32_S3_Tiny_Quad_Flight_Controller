#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_VL53L0X.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Define pins and constants
#define LED_PIN 48  
#define NUM_LEDS 1
#define PPM_PIN 13
#define NUM_CHANNELS 8
#define PPM_SYNC_THRESHOLD 3000
#define CHANNEL_MIN 1000
#define CHANNEL_MAX 2000
#define CUSTOM_SDA_PIN 3
#define CUSTOM_SCL_PIN 4

// Flight controller variables
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch = -1.14, RateCalibrationRoll = -1.50, RateCalibrationYaw = 0.48;
float AccXCalibration = 0.06, AccYCalibration = -0.04, AccZCalibration = 0.00;

// PPM variables
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;
int channelValues[NUM_CHANNELS];

// PID gains
float PAngleRoll = 2, PAnglePitch = 2, IAngleRoll = 0.5, IAnglePitch = 0.5, DAngleRoll = 0.007, DAnglePitch = 0.007;
float PRateRoll = 0.625, PRatePitch = 0.625, IRateRoll = 2.1, IRatePitch = 2.1, DRateRoll = 0.0088, DRatePitch = 0.0088;
float PRateYaw = 4, IRateYaw = 3, DRateYaw = 0;
float P_alt = 100, I_alt = 0.1, D_alt = 10;

// Altitude hold variables
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool altitude_hold_active = false;
float desired_altitude = 0;
volatile float current_altitude = 0;
volatile bool altitude_valid = false;
float prev_error_alt = 0, I_term_alt = 0, hover_throttle = 1500;

// Throttle limits
int ThrottleIdle = 1170, ThrottleCutOff = 1000, led_time = 500;

// PID variables
volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;
volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch, PrevItermAngleRoll, PrevItermAnglePitch;

// Kalman filter variables
volatile float AccX, AccY, AccZ, AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch, ErrorAngleRoll, ErrorAnglePitch;

// PWM configuration
const int pwmFrequency = 20000, pwmResolution = 8;
const int motor1Channel = 11, motor2Channel = 10, motor3Channel = 9, motor4Channel = 8;
const float t = 0.004; // Time step (seconds)

// Other variables
float Voltage;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
bool armed = false;
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000;

// Mutex for altitude data
SemaphoreHandle_t altitude_mutex;

// PPM interrupt handler
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

// Battery voltage reading
void battery_voltage() {
  Voltage = (float)analogRead(1) / 237;
}

// Read receiver values safely
void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// Kalman filter
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4); // IMU variance (4 deg/s)
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3); // error variance (3 deg)
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}


// Sensor reading task with low-pass filter
void sensorTask(void *pvParameters) {
  static float filtered_altitude = 0;
  static bool first_reading = true;
  const float alpha = 0.1;  // Smoothing factor

  while (true) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 2000) {
      float new_altitude = measure.RangeMilliMeter / 1000.0;  // Convert to meters
      if (first_reading) {
        filtered_altitude = new_altitude;
        first_reading = false;
      } else {
        filtered_altitude = alpha * new_altitude + (1 - alpha) * filtered_altitude;
      }
      xSemaphoreTake(altitude_mutex, portMAX_DELAY);
      current_altitude = filtered_altitude;
      altitude_valid = true;
      xSemaphoreGive(altitude_mutex);
    } else {
      xSemaphoreTake(altitude_mutex, portMAX_DELAY);
      altitude_valid = false;
      xSemaphoreGive(altitude_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // Read every 50ms
  }
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

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }

  altitude_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 2048, NULL, 1, NULL, 0);  // Core 0

  ledcAttach(motor1Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor2Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor3Channel, pwmFrequency, pwmResolution);
  ledcAttach(motor4Channel, pwmFrequency, pwmResolution);

  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);

  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  LoopTimer = micros();
}

void loop() {
  read_receiver(channelValues);

  // Altitude hold logic
  if (channelValues[5] > 1500) {
    if (!altitude_hold_active) {
      altitude_hold_active = true;
      xSemaphoreTake(altitude_mutex, portMAX_DELAY);
      desired_altitude = altitude_valid ? current_altitude : 1.0;
      xSemaphoreGive(altitude_mutex);
      I_term_alt = 0;
      prev_error_alt = 0;
    } else {
      if (channelValues[2] > 1550) desired_altitude += 0.01;
      else if (channelValues[2] < 1450) desired_altitude -= 0.01;
      desired_altitude = constrain(desired_altitude, 0.1, 2.0);
    }
  } else {
    altitude_hold_active = false;
  }

  // Arming/Disarming
  if (channelValues[2] < 1050) {
    if (!armed && channelValues[3] > 1900) {
      if (armDisarmTimer == 0) armDisarmTimer = millis();
      else if (millis() - armDisarmTimer > armHoldTime) {
        armed = true;
        strip.setPixelColor(0, strip.Color(255, 255, 0));
        strip.show();
        delay(1000);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        armDisarmTimer = 0;
      }
    } else if (armed && channelValues[3] < 1100) {
      if (armDisarmTimer == 0) armDisarmTimer = millis();
      else if (millis() - armDisarmTimer > armHoldTime) {
        armed = false;
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

  // Get altitude data
  float local_current_altitude;
  bool local_altitude_valid;
  xSemaphoreTake(altitude_mutex, portMAX_DELAY);
  local_current_altitude = current_altitude;
  local_altitude_valid = altitude_valid;
  xSemaphoreGive(altitude_mutex);

  // Compute throttle with altitude hold
  if (altitude_hold_active && local_altitude_valid) {
    float error_alt = desired_altitude - local_current_altitude;
    float P_term_alt = P_alt * error_alt;
    I_term_alt += I_alt * error_alt * t;
    I_term_alt = constrain(I_term_alt, -500, 500);
    float D_term_alt = D_alt * (error_alt - prev_error_alt) / t;
    InputThrottle = constrain(hover_throttle + P_term_alt + I_term_alt + D_term_alt, 1000, 2000);
    prev_error_alt = error_alt;
  } else {
    InputThrottle = channelValues[2];
  }

  // MPU6050 readings
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5 - RateCalibrationRoll;
  RatePitch = (float)GyroY / 65.5 - RateCalibrationPitch;
  RateYaw = (float)GyroZ / 65.5 - RateCalibrationYaw;
  AccX = (float)AccXLSB / 4096 - AccXCalibration;
  AccY = (float)AccYLSB / 4096 - AccYCalibration;
  AccZ = (float)AccZLSB / 4096 - AccZCalibration;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);

  KalmanAngleRoll = constrain(KalmanAngleRoll, -20, 20);
  KalmanAnglePitch = constrain(KalmanAnglePitch, -20, 20);

  DesiredAngleRoll = 0.1 * (channelValues[0] - 1500);
  DesiredAnglePitch = 0.1 * (channelValues[1] - 1500);
  DesiredRateYaw = 0.15 * (channelValues[3] - 1500);

  // Angle PID
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  DesiredRateRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  DesiredRatePitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // Rate PID
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  InputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  ErrorRatePitch = DesiredRatePitch - RatePitch;
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  InputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  ErrorRateYaw = DesiredRateYaw - RateYaw;
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = constrain(ItermYaw, -400, 400);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  InputYaw = constrain(PtermYaw + ItermYaw + DtermYaw, -400, 400);
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  if (InputThrottle > 2000) InputThrottle = 2000;

  // Motor mixing
  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw;
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw;
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw;
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw;

  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 2000);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 2000);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 2000);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 2000);

  if (channelValues[2] < 1030) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
  }

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
}
