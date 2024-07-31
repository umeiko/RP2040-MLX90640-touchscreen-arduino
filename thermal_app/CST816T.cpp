/**************************************************************************/
/*!
  @file     CST816T.cpp
  Author: Atsushi Sasaki (https://github.com/aselectroworks)
  License: MIT (see LICENSE)
*/
/**************************************************************************/

#include "CST816T.h"
#include "Arduino.h"
#include <Wire.h>

CST816T::CST816T()
{
}

CST816T::CST816T(uint8_t rst_n_pin, uint8_t int_n_pin)
{
    rst_n = rst_n_pin;
    int_n = int_n_pin;
}

CST816T::CST816T(uint8_t sda_pin, uint8_t scl_pin, uint8_t rst_n_pin, uint8_t int_n_pin)
{
    sda = sda_pin;
    scl = scl_pin;
    rst_n = rst_n_pin;
    int_n = int_n_pin;
}

CST816T::~CST816T() {
}

void CST816T::begin(void) {
    // Initialize I2C
    if(sda != -1 && scl != -1) {
        Serial1.printf("I2C initializing (SDA %d, SCL %d)\n", sda, scl);
        Wire1.setSDA(sda);
        Wire1.setSCL(scl);
        Wire1.begin(); 
        // Wire1.begin(sda, scl); 
        Serial1.printf("I2C initialized (SDA %d, SCL %d)\n", sda, scl);
    }
    else {
        Wire1.begin(); 
        Serial.println("I2C initialized (SDA, SCL)1");
    }

    // Int Pin Configuration
    if(int_n != -1) {
        pinMode(int_n, INPUT); 
    }
    // Reset Pin Configuration
    if(rst_n != -1) {
        pinMode(rst_n, OUTPUT); 
        digitalWrite(rst_n, LOW); 
        delay(10); 
        digitalWrite(rst_n, HIGH); 
        delay(500); 
    }
    // Initialize Touch
    Serial1.printf("Initialize Touch\n");
    readByte(0x15);
    delay(1);
    readByte(0xa7);
    delay(1);
    writeByte(0xEC, 0b00000101);
    delay(1);
    writeByte(0xFA, 0b01110000);
    delay(1);
}

/**
 * @设置旋转方向,默认为Rotation_0
 * @Rotation：方向 0~3
*/
void CST816T::setRotation(TouchRotation Rotation)
{
    switch (Rotation) {
    case Rotation_0:
      touch_rotation = Rotation_0;
      break;
    case Rotation_1:
      touch_rotation = Rotation_1;
      break;
    case Rotation_2:
      touch_rotation = Rotation_2;
      break;
    case Rotation_3:
      touch_rotation = Rotation_3;
      break;
    }
}

//coordinate diagram（FPC downwards）
TouchInfos CST816T::GetTouchInfo(void){
    // Serial.println("GetTouchInfo");
    byte error;
    TouchInfos info;
    uint8_t touchData[7];
    uint8_t rdDataCount;
    // 唤醒
    // readByte(0x15);
    // readByte(0xa7);
    uint8_t i = 0;
    long startTime = millis();
    do {
        Wire1.beginTransmission(I2C_ADDR_CST816T); 
        Wire1.write(0); 
        error = Wire1.endTransmission(false); // Restart
        if (error != 0) {
            info.isValid = false;
            return info;
        }
        rdDataCount = Wire1.requestFrom(I2C_ADDR_CST816T, sizeof(touchData)); 
        // Serial.printf("读取中%d  %d\n", i, rdDataCount);
        if(millis() - startTime > 1) {
            info.isValid = false;
            return info;
        }
    } while(rdDataCount == 0); 
    i = 0;
    while(Wire1.available()) {
        touchData[i] = Wire1.read();
        // Serial.printf("%d %02X \n", i, touchData[i]);
        i++;
        if(i >= sizeof(touchData)) {
            break;
        }
    }
    // Serial.println("GetTouchInfo end");

    // This can only be 0 or 1
    uint8_t nbTouchPoints = touchData[touchPointNumIndex] & 0x0f;
    uint8_t xHigh = touchData[touchXHighIndex] & 0x0f;
    uint8_t xLow = touchData[touchXLowIndex];
    uint16_t x = (xHigh << 8) | xLow;
    uint8_t yHigh = touchData[touchYHighIndex] & 0x0f;
    uint8_t yLow = touchData[touchYLowIndex];
    uint16_t y = (yHigh << 8) | yLow;
    Gestures gesture = static_cast<Gestures>(touchData[gestureIndex]);

    // Validity check
    if(x >= TouchWidth || y >= TouchHeight ||
        (gesture != Gestures::None &&
        gesture != Gestures::SlideDown &&
        gesture != Gestures::SlideUp &&
        gesture != Gestures::SlideLeft &&
        gesture != Gestures::SlideRight &&
        gesture != Gestures::SingleTap &&
        gesture != Gestures::DoubleTap &&
        gesture != Gestures::LongPress)) {
        info.isValid = false;
        return info;
    }

    info.x = x;
    info.y = y;
    info.touching = (nbTouchPoints > 0);
    info.gesture = gesture;
    info.isValid = true;
    return info;
}

//coordinate diagram（FPC downwards）
int CST816T::update(void){
    TouchInfos data = GetTouchInfo();

    tp.x = data.x;
    tp.y = data.y;
    tp.touching = data.touching;
    tp.gesture = data.gesture;
    return 0;
}

// Private Function
uint8_t CST816T::readByte(uint8_t addr) {
    uint8_t rdData; 
    uint8_t rdDataCount;
    uint8_t i;
    do {
        Wire1.beginTransmission(I2C_ADDR_CST816T); 
        Wire1.write(addr); 
        Wire1.endTransmission(false); // Restart
        rdDataCount = Wire1.requestFrom(I2C_ADDR_CST816T, 1);
        i ++; 
    } while(rdDataCount == 0 && i<250); 
    
    while(Wire1.available()) {
        rdData = Wire1.read(); 
    }
    return rdData; 
}

void CST816T::writeByte(uint8_t addr, uint8_t data) {
  Wire1.beginTransmission(I2C_ADDR_CST816T); 
    Wire1.write(addr); 
    Wire1.write(data); 
    Wire1.endTransmission(); 
}