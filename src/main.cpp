#define M5STACK_MPU6886
#define LCD
#include <Arduino.h>
#include <M5Stack.h>
#include "BMM150class.h"
#include <utility/quaternionFilters.h>

//#define DISPLAY_RAW
#define DISPLAY_AHRS
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float magnetX = 0.0F;
float magnetY = 0.0F;
float magnetZ = 0.0F;

float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;

float temp = 0.0F;

BMM150class bmm150;

uint32_t Now = 0;
uint32_t lastUpdate = 0, firstUpdate = 0;
float deltat = 0.0f, sum = 0.0f;

void setup()
{
  // put your setup code here, to run once:
  M5.begin();
  M5.Power.begin();

  M5.IMU.Init();
  bmm150.Init();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setTextSize(2);
}

void loop()
{
  // put your main code here, to run repeatedly:
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  bmm150.getMagnetData(&magnetX, &magnetY, &magnetZ);
  M5.IMU.getTempData(&temp);

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f);
  lastUpdate = Now;

  MadgwickQuaternionUpdate(accX, accY, accZ, gyroX * DEG_TO_RAD,
                           gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD,
                           magnetY, magnetX, magnetZ, deltat);
#ifdef DISPLAY_RAW
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
  M5.Lcd.setCursor(220, 42);
  M5.Lcd.print(" o/s");
  M5.Lcd.setCursor(0, 65);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
  M5.Lcd.setCursor(220, 87);
  M5.Lcd.print(" G");
  M5.Lcd.setCursor(0, 110);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", magnetX, magnetY, magnetZ);
  M5.Lcd.setCursor(220, 132);
  M5.Lcd.print(" mT");
  M5.Lcd.setCursor(0, 155);
  M5.Lcd.printf("Temperature : %.2f C", temp);
#endif

  yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                                                          *(getQ() + 3)),
              *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
  pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                                                            *(getQ() + 2)));
  roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) *
                                                     *(getQ() + 3)),
               *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
  pitch *= RAD_TO_DEG;
  yaw *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;
  yaw -= 8.5;
#ifdef DISPLAY_AHRS
  M5.Lcd.setCursor(0, 100);
  M5.Lcd.printf("  yaw: % 5.2f    pitch: % 5.2f    roll: % 5.2f   \r\n", (yaw), (pitch), (roll));
#endif
}