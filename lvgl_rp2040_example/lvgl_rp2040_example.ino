/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <FreeRTOS.h>
#include <task.h>
#include "CST816T.h"

/*To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 *You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 Note that the `lv_examples` library is for LVGL v7 and you shouldn't install it for this version (since LVGL v8)
 as the examples and demos are now part of the main LVGL library. */

#include <examples/lv_examples.h>
#include <demos/lv_demos.h>


#define TOUCH_SDA 2
#define TOUCH_SCL 3
#define TOUCH_RST -1
#define SCREEN_BL_PIN 4
#define SCREEN_VDD 5

#define ROTATE 1

#if (ROTATE == 0)
/*Change to your screen resolution*/
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 280;
#endif
#if (ROTATE == 1)
/*Change to your screen resolution*/
static const uint16_t screenWidth  = 280;
static const uint16_t screenHeight = 240;
#endif
#if (ROTATE == 2)
/*Change to your screen resolution*/
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 280;
#endif

#if (ROTATE == 3)
static const uint16_t screenWidth  = 280;
static const uint16_t screenHeight = 240;
#endif

int brightness = 128;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */
CST816T touch(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, -1);


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    #if (ROTATE == 0 || ROTATE == 2)
    tft.setAddrWindow( area->x1, area->y1, w, h );
    #endif
    #if (ROTATE == 1 || ROTATE == 3)
    tft.setAddrWindow( area->x1 + 20, area->y1, w, h );
    #endif
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp_drv );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data )
{

    touch.update();
    // Serial.print( "touch called " );
    // Serial.println( touch.tp.touching );
    bool touched = touch.tp.touching;
    if( !touched )
    // if( 0!=touch.data.points )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;
        #if (ROTATE == 0)
        /*Change to your screen resolution*/
        data->point.x = touch.tp.x;
        data->point.y = touch.tp.y;
        #endif
        #if (ROTATE == 1)
        /*Change to your screen resolution*/
        data->point.x = touch.tp.y;
        data->point.y = 240-touch.tp.x;
        #endif
        #if (ROTATE == 2)
        /*Change to your screen resolution*/
        data->point.x = 240-touch.tp.x;
        data->point.y = 280-touch.tp.y;
        #endif

        #if (ROTATE == 3)
        data->point.x = 280-touch.tp.y;
        data->point.y = touch.tp.x;
        #endif
        // data->point.x = touch.tp.x;
        // data->point.y = touch.tp.y;
        // Serial.print( "Data x " );
        // Serial.println( touch.tp.x );

        // Serial.print( "Data y " );
        // Serial.println( touch.tp.y );  
    }
}

// 这个函数可以缓慢的开启屏幕
void task_smooth_on(void * ptr){
//    ledcSetup(0, 3000, 10);
//    ledcAttachPin(SCREEN_BL_PIN, 0);
   pinMode(SCREEN_BL_PIN, OUTPUT);
   analogWrite(SCREEN_BL_PIN, 0);
   vTaskDelay(500);
   for(int i=0; i<brightness; i++){
      analogWrite(SCREEN_BL_PIN, i);
      vTaskDelay(2);
   }
   vTaskDelete(NULL); 
}

void smooth_on(){
    xTaskCreate(task_smooth_on, "SMOOTH_ON", 1024, NULL, 2, NULL);
}

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    touch.begin();
    pinMode(SCREEN_VDD, OUTPUT);
    digitalWrite(SCREEN_VDD, LOW);
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    tft.begin();          /* TFT init */
    tft.setRotation( ROTATE ); /* Landscape orientation, flipped */
    touch.begin();
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );
  
    // /* Create simple label */
    // lv_obj_t *label = lv_label_create( lv_scr_act() );
    // lv_label_set_text( label, "Hello Ardino and LVGL!");
    // lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );


    /* Try an example. See all the examples 
     * online: https://docs.lvgl.io/master/examples.html
     * source codes: https://github.com/lvgl/lvgl/tree/e7f88efa5853128bf871dde335c0ca8da9eb7731/examples */
     //lv_example_btn_1();
   
     /*Or try out a demo. Don't forget to enable the demos in lv_conf.h. E.g. LV_USE_DEMOS_WIDGETS*/
    lv_demo_widgets();               
    // lv_demo_benchmark();          
    // lv_demo_keypad_encoder();     
    // lv_demo_music();              
    // lv_demo_printer();
    // lv_demo_stress();
    smooth_on();
    Serial.println( "Setup done" );
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    delay( 5 );
}

// void begin1(){
//     delay(3000);
    
//     for(;;){
//         touch.update();
//         vTaskDelay(10);
//     }
// }