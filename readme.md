## RP2040热成像计划

电路图可以在[这里找到](https://oshwhub.com/umekoko/re-cheng-xiang_copy)

- RP2040双核MCU + ST7789屏幕（240 * 280）物理分辨率 + CST816T 触摸IC + MLX90640热成像传感器。
- 引出了UART1，以及 SWD 调试接口。
- 可连接上位机图传。
- 集成卡尔曼滤波算法。

![94acdedc4a31000fca7f8e9933caeeaf1839720](https://github.com/user-attachments/assets/5003e117-2848-4e2d-9113-3dc7317ecc94)
![image](https://github.com/user-attachments/assets/fe6f153e-51b8-4891-8e94-cc8b2bdb17b4)
![3a3f02000d5065bd46c5adb85640058](https://github.com/user-attachments/assets/5373e2d0-d3f9-4648-afc6-5e263d305e0c)

- 提供了热成像代码，位于`thermal_app`中。
- 提供lvgl示例代码，位于`lvgl_rp2040_example`中。
### 配置环境
- 配置 arduino-pico环境，具体可参考[arduino-pico](https://github.com/earlephilhower/arduino-pico/tree/master)仓库。
- 配置好 `TFT_eSPI`屏幕驱动库，版本为`2.5.43`，具体可参考[TFT_eSPI](https://github.com/Bodmer/TFT_eSPI/tree/master)仓库。
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
- 直接编译上传即可。

## 有问题发ISSUE
