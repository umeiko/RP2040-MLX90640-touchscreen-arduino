/*
 Huge thank you to Uspizig for offering this library that I have modified for TFT_eSPI: https://github.com/Uspizig/MLX90640
 Has to be Library, TFT_eSPI Rev 2.5.43
 The latest does not work

*/
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <TFT_eSPI.h> 
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/xosc.h"
// #include "hardware/rosc.h"
#include "hardware/regs/io_bank0.h"
// For __wfi
#include "hardware/sync.h"
// For scb_hw so we can enable deep sleep
#include "hardware/structs/scb.h"


#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "CST816T.h"
#include "kalman_filter.h"
#include "BilinearInterpolation.h"

#define TA_SHIFT 8 //Default shift for MLX90640 in open air
#define MLX_VDD  11
#define MLX_SDA  12
#define MLX_SCL  13

#define TOUCH_SDA 2
#define TOUCH_SCL 3
#define TOUCH_RST -1

#define BAT_ADC  26
#define SCREEN_BL_PIN 4
#define SCREEN_VDD 5

#define SCREEN_ROTATION 1

// #define DRAW_BLOCKS // 使用方块来绘制热力图
// #define DRAW_PIXELS  // 使用像素来绘制
#define DRAW_PIXELS_DMA  // 使用DMA来绘制

#define KALMAN  // 使用 卡尔曼滤波器
// #define SERIAL1_DEBUG  


#if defined(KALMAN)
#include "kalman_filter.h"
const static float init_P = 0.1;
const static float init_G = 0.0;
const static float init_O = 26;

static KFPTypeS kfpVar3Array[768];  // 卡尔曼滤波器变量数组
// 初始化卡尔曼滤波器数组的函数
void KalmanArrayInit() {
    // 循环遍历数组中的每个元素
    for (int i = 0; i < 768; ++i) {
        // 初始化每个元素
        kfpVar3Array[i] = (KFPTypeS){
         init_P,     //估算协方差. 初始化值为 0.02
         init_G,     //卡尔曼增益. 初始化值为 0
         init_O    //卡尔曼滤波器输出. 初始化值为 0
        };
    }
}

#endif

#define _SCALE 9
#define BTN_LONG_PUSH_T 1000
#define TOUCH_LONG_PUSH_T 200
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */


const int buttonPin1 = 24;
const int buttonPin2 = -1; 


bool buttonState1 = 1;  
bool buttonState2 = 1;  

const byte MLX90640_address = 0x33;
static float mlx90640To[768];              // 从MLX90640读取的温度数据
static int mlx90640To_buffer[768];       // 缓存区域，复制MLX90640读取的温度数据并用于绘制热力图
static float mlx90640To_send_buffer[768];  // 缓存区域，复制MLX90640读取的温度数据，用于发送到上位机
static uint8_t* mlx90640To_Serial_buffer = (uint8_t*)mlx90640To_send_buffer;  



paramsMLX90640 mlx90640;

#if defined(DRAW_PIXELS)
static uint16_t heat_bitmap[32*_SCALE * 24*_SCALE] = {}; // rgb56556形式的内存，用于存储要渲染的图像
#endif

uint16_t test_points[5][2];
int brightness = 100;

int R_colour, G_colour, B_colour;            
// int i, j;                                    
float T_max, T_min, T_avg;                            
float T_center;  
int max_x, max_y, min_x, min_y;

float bat_v;

bool lock = false;  // 简单的锁，防止拷贝温度数据的时候对内存的访问冲突
bool serial_cp_lock = false;  // 简单的锁，防止拷贝温度数据的时候对内存的访问冲突
bool touch_updated = false;
bool mlx_is_connected = false;

bool power_on = true;  // 是否开机
bool freeze = false;  // 暂停画面
bool show_local_temp_flag = true;  // 是否显示点测温
bool clear_local_temp = false;     // 点测温清除

bool use_upsample = true;  // 是否上采样

TFT_eSPI tft = TFT_eSPI();  
CST816T touch(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, -1);	// sda, scl, rst, irq

int diffx, diffy;

// ===============================
// ===== determine the colour ====
// ===============================
void getColour(int j)
   {
    if (j >= 0 && j < 30)
       {
        R_colour = 0;
        G_colour = 0;
        B_colour = 20 + 4 * j;
       }
    
    if (j >= 30 && j < 60)
       {
        R_colour = 4 * (j - 30);
        G_colour = 0;
        B_colour = 140 - 2 * (j - 30);
       }

    if (j >= 60 && j < 90)
       {
        R_colour = 120 + 4 * (j - 60);
        G_colour = 0;
        B_colour = 80 - 2 * (j - 60);
       }

    if (j >= 90 && j < 120)
       {
        R_colour = 255;
        G_colour = 0 + 2 * (j - 90);
        B_colour = 10 - (j - 90) / 3;
       }

    if (j >= 120 && j < 150)
       {
        R_colour = 255;
        G_colour = 60 + 175 * (j - 120) / 30;
        B_colour = 0;
       }

    if (j >= 150 && j <= 180)
       {
        R_colour = 255;
        G_colour = 235 + (j - 150) * 20 / 30;
        B_colour = 0 + 85 * (j - 150) / 10;
       }
}

void print_bitmap(int x, int y, uint16_t * data){
   
   for(int i=0; i<y; i++){
      for(int j=0; j<x; j++){
         Serial.print(data[i*j]);
         Serial.print(" ");
      }
      Serial.print("\n");
   }
   Serial.print("\n\n");
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
   {
    Wire.beginTransmission((uint8_t)MLX90640_address);
    if (Wire.endTransmission() != 0){return (false);}
    return (true);
   }   

// 绘制十字
void draw_cross(int x, int y, int len){
   tft.drawLine(x - len/2, y, x + len/2, y, tft.color565(255, 255, 255));
   tft.drawLine(x, y-len/2, x, y+len/2,  tft.color565(255, 255, 255));

   tft.drawLine(x - len/4, y, x + len/4, y, tft.color565(0, 0, 0));
   tft.drawLine(x, y-len/4, x, y+len/4,  tft.color565(0, 0, 0));
}

// 点测温功能
void show_local_temp(int x, int y){
   draw_cross(x, y, 10);
   float temp_xy = mlx90640To[(24 - y / _SCALE) * 32 + (x / _SCALE)];
   int shift_x, shift_y;
   if (x<140){shift_x=10;} else {shift_x=-40;}
   if (y<120){shift_y=10;} else {shift_y=-10;}
   tft.setCursor(x+shift_x, y+shift_y);
   tft.printf("%.2f", temp_xy);
}  

#if defined(DRAW_BLOCKS)
void draw_heat_image(bool re_mapcolor=true){
   tft.setRotation(3);
   for (int i = 0 ; i < 24 ; i++){
   for (int j = 0; j < 32; j++){
      if (re_mapcolor) {mlx90640To_buffer[i*32 + j] = 180.0 * (mlx90640To_buffer[i*32 + j] - T_min) / (T_max - T_min);}
      getColour(mlx90640To_buffer[i*32 + j]);
      tft.fillRect(280 - j * _SCALE, (240 - _SCALE * 24) + i * _SCALE, _SCALE, _SCALE, tft.color565(R_colour, G_colour, B_colour));  
      // tft.drawBitmap();
      }
   }
   tft.setRotation(SCREEN_ROTATION);
}
#endif


#if defined(DRAW_PIXELS)
// 在一块图像内存上绘制一个同色的方块
void draw_block_bitmap(int x, int y, int w, int max_x, int max_y, uint16_t color, uint16_t* data){
   for(int i =y; i < y + w; i++){
        for(int j = x; j < x + w; j++){
         data[i * max_x + j] = color;
      }
   }
}

// 更新图像内存中的图像
void update_bitmap(bool re_mapcolor=true){
   for(int y=0; y<24; y++){ 
      for(int x=0; x<32; x++){
         int id = (23-y) * 32 + x;
         if (re_mapcolor) {mlx90640To_buffer[id] = 180.0 * (mlx90640To_buffer[id] - T_min) / (T_max - T_min);}
         getColour(mlx90640To_buffer[id]);
         // draw_block_bitmap(x*_SCALE, y*_SCALE, _SCALE, 32*_SCALE, 24*_SCALE, tft.color565(255, 0, 0), heat_bitmap);
         draw_block_bitmap(x*_SCALE, y*_SCALE, _SCALE, 32*_SCALE, 24*_SCALE, tft.color565(R_colour, G_colour, B_colour), heat_bitmap);
      } 
   }
}

void update_bitmap_bio_linear(){
   int value;
   for(int y=0; y<24 * _SCALE; y++){ 
      for(int x=0; x<32 * _SCALE; x++){
         value = bio_linear_interpolation(x, y, mlx90640To_buffer);
         getColour(value);
         heat_bitmap[y * 32 * _SCALE + x] = tft.color565(R_colour, G_colour, B_colour);
      }
   }
}

// 在屏幕上绘制热力图
void draw_heat_image(bool re_mapcolor=true){
   // tft.setRotation(3);
   tft.setRotation(SCREEN_ROTATION);
   if(re_mapcolor){
      update_bitmap_bio_linear();
   }
   tft.pushImage(0, 0, 32*_SCALE, 24*_SCALE, heat_bitmap);
}
#endif


#if defined(DRAW_PIXELS_DMA)
const int lines = 25;
uint16_t  lineBuffer[32 * _SCALE * lines]; // Toggle buffer for lines
uint16_t  dmaBuffer1[32 * _SCALE * lines]; // Toggle buffer for lines
uint16_t  dmaBuffer2[32 * _SCALE * lines]; // Toggle buffer for lines
uint16_t* dmaBufferPtr = dmaBuffer1;
bool dmaBufferSel = 0;
// 在屏幕上绘制热力图
void draw_heat_image(bool re_mapcolor=true){  
   static int value;
   static int now_y = 0;
   tft.setRotation(SCREEN_ROTATION);
   if(use_upsample){
      tft.startWrite();
      for(int y=0; y<24 * _SCALE; y++){ 
         for(int x=0; x<32 * _SCALE; x++){
            value = bio_linear_interpolation(x, y, mlx90640To_buffer);
            getColour(value);
            lineBuffer[x + now_y*32 * _SCALE] = tft.color565(R_colour, G_colour, B_colour);
         }
         now_y ++;
         if(now_y==lines){
            if (dmaBufferSel) dmaBufferPtr = dmaBuffer2;
            else dmaBufferPtr = dmaBuffer1;
            dmaBufferSel = !dmaBufferSel; // Toggle buffer selection
            // tft.startWrite();
            tft.pushImageDMA(0, y-now_y, 32*_SCALE, lines, lineBuffer, dmaBufferPtr);
            // tft.endWrite();
            now_y = 0;
         }
      }if(now_y!=0){
         if (dmaBufferSel) dmaBufferPtr = dmaBuffer2;
         else dmaBufferPtr = dmaBuffer1;
         dmaBufferSel = !dmaBufferSel; // Toggle buffer selection
         // tft.startWrite();
         tft.pushImageDMA(0, 24*_SCALE-1-now_y, 32*_SCALE, now_y, lineBuffer, dmaBufferPtr);
         // tft.endWrite();
         now_y = 0;
      }
      tft.endWrite();
   }else{
      tft.setRotation(3);
      for (int i = 0 ; i < 24 ; i++){
      for (int j = 0; j < 32; j++){
         // if (re_mapcolor) {mlx90640To_buffer[i*32 + j] = 180.0 * (mlx90640To_buffer[i*32 + j] - T_min) / (T_max - T_min);}
         getColour(mlx90640To_buffer[i*32 + j]);
         tft.fillRect(280 - j * _SCALE, (240 - _SCALE * 24) + i * _SCALE, _SCALE, _SCALE, tft.color565(R_colour, G_colour, B_colour));  
      }
      }
   }
}

#endif

int status;
uint16_t eeMLX90640[832];
int mlx_setup(){
   pinMode(MLX_VDD, OUTPUT);
   digitalWrite(MLX_VDD, LOW);
   Wire.setSDA(MLX_SDA);
   Wire.setSCL(MLX_SCL);
   Wire.begin(); 
   vTaskDelay(500);
   Wire.setClock(800000); //Increase I2C clock speed to 800kHz
   Serial1.println("MLX90640 IR Array Example");
   mlx_is_connected = isConnected();
   if (mlx_is_connected == false){
      // while(!isConnected()){
         Serial1.println("MLX90640 not detected at default I2C address. Please check wiring.");
      // }
      return 1;
   }
   Serial1.println("MLX90640 online!");
   status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
   if (status != 0)
      Serial1.println("Failed to load system parameters");

   status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
   if (status != 0)
   {
      Serial1.println("Parameter extraction failed");
      Serial1.print(" status = ");
      Serial1.println(status);
   }
   MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 4Hz effective - Works
   // MLX90640_I2CWrite(0x33, 0x800D, 6401);    // writes the value 1901 (HEX) = 6401 (DEC) in the register at position 0x800D to enable reading out the temperatures!!!
   MLX90640_SetRefreshRate(MLX90640_address, 0x05); //Set rate to 8Hz effective - Works at 800kHz
   return 0;
}

uint16_t count_retry = 0;
void mlx_loop(){
   if(!mlx_is_connected && count_retry < 10){
      mlx_setup();
      count_retry++;
   }
   if (!freeze && mlx_is_connected==true){ // 如果画面被暂停会跳过这个热成像图的刷新
      lock = true;
      for (byte x = 0 ; x < 2 ; x++){
         uint16_t mlx90640Frame[834];
         int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
      
         if (status < 0){
            Serial1.print("GetFrame Error: ");
            Serial1.println(status);
            }
         float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
         float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
         float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
         float emissivity = 0.95;
         MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
      }

      // mlx90640To[229] = 0.5 * (mlx90640To[228] + mlx90640To[230]);    // eliminate the error-pixels
      
      T_min = mlx90640To[0];
      T_max = mlx90640To[0];
      T_avg = mlx90640To[0];
      for (int i = 1; i < 768; i++){
         if((mlx90640To[i] > -41) && (mlx90640To[i] < 301))
            {
               if(mlx90640To[i] < T_min)
                  {
                  T_min = mlx90640To[i];
                  }

               if(mlx90640To[i] > T_max)
                  {
                  T_max = mlx90640To[i];
                  max_x = i / 32;
                  max_y = i % 32;
                  }
            #if defined(KALMAN)
            mlx90640To[i] = KalmanFilter(&kfpVar3Array[i], mlx90640To[i]);
            #endif
            }
         else if(i > 0){
               mlx90640To[i] = mlx90640To[i-1];
            }
         else{
                mlx90640To[i] = mlx90640To[i+1];
            }
            T_avg = T_avg + mlx90640To[i];
         }
      T_avg = T_avg / 768;
      #if defined(KALMAN)
      T_avg = KalmanFilter(&kfpVar1, T_avg);
      T_max = KalmanFilter(&kfpVar2, T_max);
      T_min = KalmanFilter(&kfpVar3, T_min);
      #endif
      lock = false;
   }
}

// 热成像读取多任务
void task_mlx(void * ptr){
   mlx_setup();
   // MLX主循环
   for(;power_on==true;){
      mlx_loop();
      vTaskDelay(10);
   }
   vTaskDelete(NULL); 
}

// 关机
void power_off(){

//    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
   for(int i=brightness; i>0; i--){
      analogWrite(SCREEN_BL_PIN, i);
      vTaskDelay(2);
   }
   
   power_on = false;
   digitalWrite(MLX_VDD, HIGH);
   analogWrite(SCREEN_BL_PIN, 0);
   digitalWrite(SCREEN_BL_PIN, LOW);
   vTaskDelay(2000);
   
   xosc_dormant();
//    sleep_run_from_xosc();
   // esp_light_sleep_start();
//    esp_deep_sleep_start();
   vTaskDelete(NULL); 
}

// 背光调节,会限制输入亮度在正确范围内
void set_brightness(int _brightness){
   if (_brightness < 255 && _brightness > 5){
      analogWriteFreq(10000);
      analogWrite(SCREEN_BL_PIN, _brightness);
      brightness = _brightness;
   }else if(_brightness >= 255){analogWrite(SCREEN_BL_PIN, 255); brightness=255;
   }else if(_brightness <= 5)   {analogWrite(SCREEN_BL_PIN, 5); brightness=5;
   }
}

void smooth_on(){
   pinMode(SCREEN_BL_PIN, OUTPUT);
   analogWrite(SCREEN_BL_PIN, 0);
   analogWriteFreq(10000);
   for(int i=0; i<brightness; i++){
      analogWrite(SCREEN_BL_PIN, i);
      vTaskDelay(2);
   }
}

// 平滑的开机
void task_smooth_on(void * ptr){
   smooth_on();
   vTaskDelete(NULL); 
}


uint32_t dt = millis();
void screen_setup(){
   tft.setRotation(SCREEN_ROTATION);
   tft.fillScreen(TFT_BLACK);
   // tft.fillScreen(TFT_GREEN);
   test_points[0][0] = 120;
   test_points[0][1] = 110;
   tft.setCursor(25, 220);
   tft.printf("max: %.2f  ", T_max);
   tft.setCursor(25, 230);
   tft.printf("min: %.2f  ", T_min);

   tft.setCursor(105, 220);
   tft.printf("avg: %.2f  ", T_avg);
   tft.setCursor(105, 230);
   tft.printf("bat: %.2f v ", bat_v);

   tft.setCursor(180, 220);
   tft.printf("bright: %d  ", brightness);
   tft.setCursor(180, 230);
   tft.printf("time: %d ", dt);
   tft.printf("ms     ");
}
void screen_loop(){
   if (!freeze){ // 如果画面被暂停会跳过这个热成像图的刷新
   // 只有画面更新才会绘制一张热成像图
   dt = millis();
   while(lock && power_on){
      // 阻塞画面
      vTaskDelay(1);
   }
   for (int i = 0; i < 768; i++) {
      // mlx90640To_buffer[i] = mlx90640To[i];
      mlx90640To_buffer[i] = (int)(180.0 * (mlx90640To[i] - T_min) / (T_max - T_min));
   }  // 拷贝温度信息
   draw_heat_image();
   dt = millis() - dt;
   }else{dt = 0;}

   tft.setRotation(SCREEN_ROTATION);
   if (test_points[0][0]==0 && test_points[0][1]==0 ){}else{show_local_temp(test_points[0][0], test_points[0][1]);}
   if (clear_local_temp==true) {draw_heat_image(false); clear_local_temp=false;}

   tft.setRotation(SCREEN_ROTATION);
   tft.setTextColor(TFT_WHITE, TFT_BLACK); 
   if (!mlx_is_connected){
      tft.setCursor(25, 110);
      tft.printf("MLX90640 not detected at default I2C address. Please check wiring.");
   }
   tft.setCursor(25, 220);
   tft.printf("max: %.2f  ", T_max);
   tft.setCursor(25, 230);
   tft.printf("min: %.2f  ", T_min);

   tft.setCursor(105, 220);
   tft.printf("avg: %.2f  ", T_avg);
   tft.setCursor(105, 230);
   tft.printf("bat: %.2f v ", bat_v);

   tft.setCursor(180, 220);
   tft.printf("bright: %d  ", brightness);
   tft.setCursor(180, 230);
   tft.printf("time: %d ", dt);
   tft.printf("ms     ");
   // vTaskDelay(10);
}

void task_screen_draw(void * ptr){
   screen_setup();
   for(;power_on==true;){
      screen_loop();
   }
   vTaskDelete(NULL);
}

// 通过串口传输单个浮点数据
void send_float_as_uint8(float f, uint8_t *buf) {
   memcpy(buf, &f, sizeof(float));
   Serial.write(buf, sizeof(float));
}

// 通过串口把整个温度数据矩阵传输
void send_to_serial() {
   // memcpy(mlx90640To_Serial_buffer, mlx90640To_send_buffer, 768 * sizeof(float));
   Serial.write(mlx90640To_Serial_buffer, 768 * sizeof(float));
}

void task_serial_communicate(void * ptr){
   vTaskDelay(3000);
   uint8_t send_buf[4];

   for(;power_on==true;){
      // 拷贝温度信息
      while (lock == true) {vTaskDelay(1);}
      memcpy(mlx90640To_send_buffer, mlx90640To, 768 * sizeof(float));
      // for (int i = 0; i < 768; i++) {mlx90640To_send_buffer[i] = mlx90640To[i];} 
      Serial.print("BEGIN");
      send_float_as_uint8(T_max, send_buf);
      send_float_as_uint8(T_min, send_buf);
      send_float_as_uint8(T_avg, send_buf);
      // for (int i = 0; i < 768; i++){
      //    send_float_as_uint8(mlx90640To_send_buffer[i], send_buf);
      //    if(i % 5==0){vTaskDelay(1);}
      // }
      send_to_serial();
      Serial.print("END");
      vTaskDelay(30);
   }
   vTaskDelete(NULL);
}



void setup1(void)
 {
   Serial.begin(115200);
   EEPROM.begin(128);
   uint8_t value = EEPROM.read(0);
   if (value != 0 && value != 1) {
      value = 0;  
   }
   use_upsample = value;
   brightness = EEPROM.read(1);
   #if defined(SERIAL1_DEBUG)
   Serial1.begin(115200);
   Serial1.println("RP2040 is starting...");
   #endif
   #if defined(KALMAN)
   KalmanArrayInit();
   #endif
   touch.begin();
//  mlx_setup();
   // 按钮启用
   pinMode(SCREEN_BL_PIN, OUTPUT);
   digitalWrite(SCREEN_BL_PIN, LOW);
   pinMode(SCREEN_VDD, OUTPUT);
   digitalWrite(SCREEN_VDD, LOW);
   // xTaskCreate(task_mlx, "MLX_FLASHING", 1024 * 4, NULL, 1, NULL);
//  xTaskCreate(task_bat, "BAT_MANAGER", 1024 * 2, NULL, 3, NULL);
   tft.init();
   tft.setSwapBytes(true);
   tft.initDMA();
   screen_setup();
   vTaskDelay(300);
   smooth_on();
}

void loop1() 
{
   screen_loop();
}

void setup(void)
{
   pinMode(buttonPin1, INPUT_PULLUP);
   pinMode(BAT_ADC, INPUT);
   // mlx_setup();
   xTaskCreate(task_mlx, "MLX_FLASHING", 1024 * 4, NULL, 1, NULL);
   delay(1000);
   uint8_t send_buf[4];
   unsigned long btn1_pushed_start_time =  0;
   unsigned long btn2_pushed_start_time =  0;
   bool btn2_pushed = false;
   bool btn2_long_pushed = false;
   bool btn1_pushed = false;
   
   float r1 = 300.;
   float r2 = 680.;
   float coef = (r1+r2) / r2;
   int adc_value = analogRead(BAT_ADC);
   TickType_t xStartTime = xTaskGetTickCount();
   const TickType_t xWait = 5000;

   uint16_t x, y;
   uint16_t start_x, start_y;
   bool long_pushed = false;
   unsigned long touch_pushed_start_time =  0;
   bool touched = false;
   int start_br = brightness;

   
   for(;power_on==true;){
      if (BOOTSEL){  // 长按btn1的关机功能
         if (millis() - btn1_pushed_start_time >= BTN_LONG_PUSH_T){
         power_off();
         Serial1.println("power off");
         } 
         vTaskDelay(5);
         if (BOOTSEL){btn1_pushed=true;}
      }else{
         btn1_pushed_start_time = millis();
         if (btn1_pushed) {  // 短按btn1
         test_points[0][0] = 0;
         test_points[0][1] = 0;
         if (freeze==true){ clear_local_temp=true; }
         }
         btn1_pushed=false;
      }

      if (digitalRead(buttonPin1) == LOW){
         if (millis() - btn2_pushed_start_time >= BTN_LONG_PUSH_T){
            // power_off();
            if (!btn2_long_pushed){
               use_upsample = !use_upsample;
               btn2_long_pushed = true;
               EEPROM.write(0, use_upsample);
               EEPROM.commit();
               }
         }
         vTaskDelay(5);
         if (digitalRead(buttonPin1) == LOW){btn2_pushed=true;}
      }else{
         btn2_pushed_start_time = millis();
         if (btn2_pushed) {
            if (!btn2_long_pushed){freeze = !freeze; }
         }
         btn2_pushed=false;
         btn2_long_pushed = false;
      }

      buttonState1 = BOOTSEL;
      buttonState2 = digitalRead(buttonPin1);
   
      while (lock == true) {vTaskDelay(1);}
      memcpy(mlx90640To_send_buffer, mlx90640To, 768 * sizeof(float));
      // for (int i = 0; i < 768; i++) {mlx90640To_send_buffer[i] = mlx90640To[i];} 
      Serial.print("BEGIN");
      send_float_as_uint8(T_max, send_buf);
      send_float_as_uint8(T_min, send_buf);
      send_float_as_uint8(T_avg, send_buf);
      send_to_serial();
      Serial.print("END");
   //   vTaskDelay(30);

      if(xStartTime + xWait > xTaskGetTickCount()){
      adc_value = analogRead(BAT_ADC);
      bat_v = (float)adc_value / 1024. * 3.3 * coef;
      xStartTime = xTaskGetTickCount();
      }

      if (!touch_updated){touch.update(); touch_updated=true;}
      if (touch_updated) {
      if( touch.tp.touching )
      {
         x= touch.tp.y;
         y = 240 - touch.tp.x;
         if (touched==false){start_x = x;  start_y = y; diffy=0; diffx=0;}  // 下降沿
         if (millis() - touch_pushed_start_time >= TOUCH_LONG_PUSH_T){
            long_pushed = true;
            diffx= start_x-x;
            diffy= start_y-y;
            set_brightness(start_br+diffy*5);
            }else{ // 短按的中间
               
            }
      }else{
         touch_pushed_start_time = millis();
         if (touched==true){  // 上升沿
            if (start_br == brightness){
               if (y < 216){test_points[0][0] = x; test_points[0][1] = y;}
            }
            if (long_pushed==false){  // 短按时
               if (y < 216){test_points[0][0] = x; test_points[0][1] = y;}
            }
            start_br = brightness;
            EEPROM.write(1, brightness);
            EEPROM.commit();
            long_pushed = false;  // 上升沿将长按检测标识符进行复位
         }  
      }
      touched = touch.tp.touching;
      touch_updated = false;
   } 
   // mlx_loop();
   vTaskDelay(10);
   }
   vTaskDelete(NULL);
}

void loop() 
{
   vTaskDelay(3000);
}
