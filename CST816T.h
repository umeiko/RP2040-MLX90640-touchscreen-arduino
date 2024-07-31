/**************************************************************************/
/*!
  @file     CST816T.h
  Author: Atsushi Sasaki(https://github.com/aselectroworks)
  License: MIT (see LICENSE)
*/
/**************************************************************************/

#ifndef _CST816T_H
#define _CST816T_H

#include <stdint.h>
#include <stdbool.h>

#define TouchWidth 250
#define TouchHeight 300

#define I2C_ADDR_CST816T 0x15

// 触摸旋转方向
typedef enum {
    Rotation_0 = 0, 
    Rotation_1, 
    Rotation_2,
    Rotation_3, 
} TouchRotation; 

typedef enum :uint8_t {
    None = 0x00,
    SlideDown = 0x01,
    SlideUp = 0x02,
    SlideLeft = 0x03,
    SlideRight = 0x04,
    SingleTap = 0x05,
    DoubleTap = 0x0B,
    LongPress = 0x0C
}Gestures;

typedef struct{
    uint16_t x = 0;
    uint16_t y = 0;
    Gestures gesture = Gestures::None;
    bool touching = false;
    bool isValid = false;
}TouchInfos;

struct data_struct {
    uint16_t x = 0;
    uint16_t y = 0;
    Gestures gesture = Gestures::None;
    bool touching = false;
    bool isValid = false;
};


/**************************************************************************/
/*!
    @brief  CST816T I2C CTP controller driver
*/
/**************************************************************************/
class CST816T 
{
public:
    CST816T();
    CST816T(uint8_t rst_n, uint8_t int_n);
    CST816T(uint8_t sda, uint8_t scl, uint8_t rst_n, uint8_t int_n); 
    data_struct tp;
    virtual ~CST816T(); 

    void begin(void);
    void setRotation(TouchRotation Rotation);
    int update();

    // Scan Function
    TouchInfos GetTouchInfo(void);

private: 
    int sda = -1; 
    int scl = -1; 
    uint8_t int_n = -1; 
    uint8_t rst_n = -1; 
    uint8_t touch_rotation = Rotation_0;

    // Unused/Unavailable commented out
    static constexpr uint8_t gestureIndex = 1;
    static constexpr uint8_t touchPointNumIndex = 2;
    //static constexpr uint8_t touchEventIndex = 3;
    static constexpr uint8_t touchXHighIndex = 3;
    static constexpr uint8_t touchXLowIndex = 4;
    //static constexpr uint8_t touchIdIndex = 5;
    static constexpr uint8_t touchYHighIndex = 5;
    static constexpr uint8_t touchYLowIndex = 6;
    //static constexpr uint8_t touchStep = 6;
    //static constexpr uint8_t touchXYIndex = 7;
    //static constexpr uint8_t touchMiscIndex = 8;

    uint8_t readByte(uint8_t addr); 
    void writeByte(uint8_t addr, uint8_t data); 
}; 
#endif