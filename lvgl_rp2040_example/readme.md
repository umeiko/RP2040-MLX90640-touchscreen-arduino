## LVGL示例

![3a3f02000d5065bd46c5adb85640058](https://github.com/user-attachments/assets/9a445a53-78a6-4317-9958-f931e3456c1c)


### 配置环境
- 配置 arduino-pico环境，具体可参考[arduino-pico](https://github.com/earlephilhower/arduino-pico/tree/master)仓库。
- 配置好 `TFT_eSPI`屏幕驱动库，版本为`2.5.43`，具体可参考[TFT_eSPI](https://github.com/Bodmer/TFT_eSPI/tree/master)仓库。
- 配置好 `lvgl`库，版本为`8.3.11`，具体可参考[lvgl](https://github.com/lvgl/lvgl)仓库。
- 将`C:\Users\*你自己的名字*\Documents\Arduino\libraries\TFT_eSPI\User_Setup.h`中替换成以下内容：

    ```C
    // ST7789 240 x 280 display with no chip select line
    #define USER_SETUP_ID 18
    #define ST7789_DRIVER 

    #define TFT_WIDTH  240
    #define TFT_HEIGHT 280

    #define CGRAM_OFFSET      // Library will add offsets required
    #define TFT_RGB_ORDER TFT_RGB  // Colour order Red-Green-Blue

    #define TFT_CS 9   // CS
    #define TFT_DC 8   // RX
    #define TFT_RST -1  // No Reset pin
    #define TFT_MOSI 7  // TX
    #define TFT_SCLK 6  // SCK

    #define LOAD_GLCD   
    #define LOAD_FONT2  
    #define LOAD_FONT4 
    #define LOAD_FONT6  
    #define LOAD_FONT7  
    #define LOAD_FONT8  
    #define LOAD_GFXFF  

    #define SMOOTH_FONT
    #define RP2040_PIO_SPI
    #define RP2040_DMA
    #define SPI_FREQUENCY  40000000
    #define SUPPORT_TRANSACTIONS
    ```
- 将`lv_conf.h`复制到`C:\Users\*你自己的名字*\Documents\Arduino\libraries\`中：
- 直接编译上传即可。

## 有问题发ISSUE
