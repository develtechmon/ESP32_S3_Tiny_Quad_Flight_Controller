#include <Wire.h>
#include <VL53L0X.h>

#define CUSTOM_SDA_PIN 3
#define CUSTOM_SCL_PIN 4

VL53L0X sensor;

void setup()
{
  Serial.begin(9600);

  // Initialize Wire with your custom SDA and SCL pins
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);

  delay(100);

  sensor.setTimeout(500);

  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.startContinuous();
}

void loop()
{
  uint16_t distance = sensor.readRangeContinuousMillimeters();
  Serial.print(distance);

  if (sensor.timeoutOccurred())
  {
    Serial.print(" TIMEOUT");
  }

  Serial.println();
  delay(100); // Optional delay for readability
}
