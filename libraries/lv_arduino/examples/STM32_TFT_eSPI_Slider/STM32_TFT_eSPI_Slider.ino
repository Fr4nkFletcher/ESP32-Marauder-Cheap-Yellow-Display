//Created by Hamid Saffari @ Apr 2020. https://github.com/HamidSaffari/ Released into the public domain.
#include <SPI.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <TFT_eTouch.h>  // https://github.com/achillhasler/TFT_eTouch

/* using TFT_eTouch.h             : 106860 bytes (81%) FALSH,  13432 bytes (65%) RAM 
   using TFT_eSPI.h touch include : 105096 bytes (80%) FALSH,  13392 bytes (65%) RAM 
*/

#define LED_PIN PB6

TFT_eSPI tft = TFT_eSPI(); /* TFT instance */
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];


//TFT_eTouch<TFT_eSPI> touch(tft, TFT_ETOUCH_CS, 0xff, TFT_eSPI::getSPIinstance());
SPIClass spi_touch(PB15, PB14, PB13);// SPIClass SPITwo(PB15, PB14, PB13); (MOSI, MISO, CLK)
TFT_eTouch<TFT_eSPI> touch(tft, TFT_ETOUCH_CS, TFT_ETOUCH_PIRQ, spi_touch); 


lv_obj_t * slider_label;
int screenWidth = 320;
int screenHeight = 240;
  
  
/* By Bodmer in https://github.com/Bodmer/TFT_eSPI/issues/581
   faster and more efficient than previous one: */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) 
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
 #ifdef USE_DMA
   tft.pushImageDMA(area->x1, area->y1, w, h, &color_p->full);
   // Note: tft.endWrite(); must be called in my_lv_tick_handler when DMA is complete
 #else
   tft.setAddrWindow(area->x1, area->y1, w, h);
   tft.pushColors(&color_p->full, w * h, true);
   tft.endWrite();
 #endif

  lv_disp_flush_ready(disp);
}

bool my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{
    
	int16_t touchX, touchY;
	bool touched = touch.getXY(touchX, touchY);  //  bool TFT_eTouch<T>::getXY(int16_t& x, int16_t& y)
    
    if(!touched)
    {
      return false;
    }

    if(touchX>screenWidth || touchY > screenHeight)
    {
      Serial.println("Y or y outside of expected parameters..");
      Serial.print("y:");
      Serial.print(touchX);
      Serial.print(" x:");
      Serial.print(touchY);
    }
    else
    {
		
      data->state = touched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL; 
  
      /*Save the state and save the pressed coordinate*/
      //if(data->state == LV_INDEV_STATE_PR) touchpad_get_xy(&last_x, &last_y);
     
      /*Set the coordinates (if released use the last pressed coordinates)*/
      data->point.x = touchX;
      data->point.y = touchY;
  
      Serial.print("Data x");
      Serial.println(touchX);
      
      Serial.print("Data y");
      Serial.println(touchY);

    }

    return false; /*Return `false` because we are not buffering and no more data to read*/
}


void slider_event_cb(lv_obj_t * slider, lv_event_t event)
{

  printEvent("Slider", event);

  if(event == LV_EVENT_VALUE_CHANGED) {
      static char buf[4];                                 /* max 3 bytes  for number plus 1 null terminating byte */
      snprintf(buf, 4, "%u", lv_slider_get_value(slider));
      lv_label_set_text(slider_label, buf);               /*Refresh the text*/
	  int16_t value = lv_slider_get_value(slider);
	  value = map(value, 0, 100, 0, 255);
	  //analogWriteFrequency(2000); // Set PMW period to 2000 Hz instead of 1000
	  analogWrite(LED_PIN, value);//8-bit by default
  }
}

void printEvent(String Event, lv_event_t event)
{
  
  Serial.print(Event);
  Serial.printf(" ");

  switch(event) {
      case LV_EVENT_PRESSED:
          Serial.printf("Pressed\n");
          break;

      case LV_EVENT_SHORT_CLICKED:
          Serial.printf("Short clicked\n");
          break;

      case LV_EVENT_CLICKED:
          Serial.printf("Clicked\n");
          break;

      case LV_EVENT_LONG_PRESSED:
          Serial.printf("Long press\n");
          break;

      case LV_EVENT_LONG_PRESSED_REPEAT:
          Serial.printf("Long press repeat\n");
          break;

      case LV_EVENT_RELEASED:
          Serial.printf("Released\n");
          break;
  }
}


void setup() {

  Serial.begin(115200); /* prepare for possible serial debug */
  pinMode(LED_PIN, OUTPUT);
  
  lv_init();

  tft.begin(); /* TFT init */
  tft.setRotation(1);

  spi_touch.begin();
  touch.init();
  
/*  @brief  set measure
  * @param drop_first ignore first n measures
  * @param z_once read z1 and z2 only once. When this flag is true z_first must be also true.
  * @param z_first when true start measure with Z1 otherwise with X
  * @param z_local_min when true, get local minimum of RZ
  * @param count how many values used by averaging or witch measure is taken. 0 means read until measure is equal last measure
  *
  *  inline void setMeasure(uint8_t drop_first = 0, bool z_once = false, bool z_first = false, bool z_local_min = false, uint8_t count = 0);
*/
  touch.setMeasure(0, false, true, false, 2); // z first, take 2'th z,x,y   // untouched: 35 us touched: 95 us
 
  //touch.setMeasure(0, true, true, false, 1); // Differential mode fastest (each axis read only once, may work)
  //touch.setAcurateDistance(25); // in this mode acurate distance must be higher for getUserCalibration (default 10)
  
  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);             /*Descriptor of a input device driver*/
  indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
  indev_drv.read_cb = my_touchpad_read;      /*Set your driver function*/
  lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/

  //Set the theme..
  //lv_theme_t * th = lv_theme_night_init(210, NULL);     //Set a HUE value and a Font for the Night Theme
  //lv_theme_set_current(th);

  lv_obj_t * scr = lv_cont_create(NULL, NULL);
  lv_disp_load_scr(scr);

  //lv_obj_t * tv = lv_tabview_create(scr, NULL);
  //lv_obj_set_size(tv, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));

  /* Create simple label */
  lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(label, "PB6 LED DIMMER");
  lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, -50);

  /* Create a slider in the center of the display */
  lv_obj_t * slider = lv_slider_create(lv_scr_act(), NULL);
  lv_obj_set_width(slider, screenWidth-50);                        /*Set the width*/
  lv_obj_set_height(slider, 50);
  lv_obj_align(slider, NULL, LV_ALIGN_CENTER, 0, 0);    /*Align to the center of the parent (screen)*/
  lv_obj_set_event_cb(slider, slider_event_cb);         /*Assign an event function*/

  /* Create a label below the slider */
  slider_label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(slider_label, "0");
  lv_obj_set_auto_realign(slider, true);
  lv_obj_align(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

}

void loop() {

  lv_task_handler(); /* let the GUI do its work */
  delay(5);
}
