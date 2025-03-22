#include <Wire.h>
#include "Adafruit_VL53L1X.h"

#define CUSTOM_SDA_PIN 3
#define CUSTOM_SCL_PIN 4

// Use the default constructor (we are not specifying XSHUT/IRQ pins here)
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X sensor demo with custom I2C pins"));

  // Initialize I2C with custom SDA and SCL pins
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);

  // Scan for an I2C device and get its address
  byte sensorAddress = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      sensorAddress = address;
      break;
    }
  }

  if (sensorAddress == 0) {
    Serial.println(F("No I2C device found!"));
    while (1) delay(10);
  }

  Serial.print(F("Found I2C device at address: 0x"));
  Serial.println(sensorAddress, HEX);

  // Initialize the sensor using the found address
  if (!vl53.begin(sensorAddress, &Wire)) {
    Serial.print(F("Error on init of VL53L1X sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Ranging started"));

  // Set a valid timing budget (ms): 15, 20, 33, 50, 100, 200, or 500
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());
}

void loop() {
  if (vl53.dataReady()) {
    int16_t distance = vl53.distance();
    if (distance == -1) {
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    // Clear the interrupt to allow a new measurement
    vl53.clearInterrupt();
  }
}
