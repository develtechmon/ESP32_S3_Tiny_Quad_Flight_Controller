/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.0  29 December 2022 -  initial release
*/
//taken from Carbon Aeronautics
#include <Wire.h>

// Define your custom I2C pins (change these as needed)
#define CUSTOM_SDA_PIN 3   // Example: GPIO3
#define CUSTOM_SCL_PIN 4   // Example: GPIO4

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;

void gyro_signals(void) {
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

  Wire.requestFrom(0x68,6);
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

  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  // RateRoll=(float)GyroX/65.5;
  // RatePitch=(float)GyroY/65.5;
  // RateYaw=(float)GyroZ/65.5;

  // AccX=(float)AccXLSB/4096;
  // AccY=(float)AccYLSB/4096;
  // AccZ=(float)AccZLSB/4096;
  
  RateRoll=(float)GyroX/65.5 + 4.93;
  RatePitch=(float)GyroY/65.5 - 0.81;
  RateYaw=(float)GyroZ/65.5 + 0.86;
  AccX=(float)AccXLSB/4096 - 0.02;
  AccY=(float)AccYLSB/4096 - 0.01;
  AccZ=(float)AccZLSB/4096 - 0.07;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
  
}
void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  
  int led_time = 500;

  for (int i = 0; i < 4; i++) {
    digitalWrite(2, HIGH); delay(led_time);
    digitalWrite(2, LOW); delay(led_time);
  }

  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}
void loop() {
  gyro_signals();

  Serial.print("Rate Roll = ");
  Serial.print(RateRoll);
  Serial.print("  ");

  Serial.print(" Rate Pitch = ");
  Serial.print(RatePitch);
  Serial.print("  ");

  Serial.print(" Rate Yaw Z = ");
  Serial.print(RateYaw);
  Serial.print("  ");

  Serial.print(" Acceleration X [g]= ");
  Serial.print(AccX);
  Serial.print("  ");

  Serial.print(" Acceleration Y [g]= ");
  Serial.print(AccY);
  Serial.print("  ");

  Serial.print(" Acceleration Z [g]= ");
  Serial.print(AccZ);
  Serial.print("  ");

  Serial.print(" Roll Angle = ");
  Serial.print(AngleRoll);
  Serial.print("  ");
  
  Serial.print(" Roll Pitch = ");
  Serial.println(AnglePitch);
  delay(100);
}
