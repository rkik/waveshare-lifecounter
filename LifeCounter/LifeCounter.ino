
// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include <Adafruit_XCA9554.h>
#include "pin_config.h"
#include "lv_conf.h"
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorPCF85063.hpp"
#include "HWCDC.h"
#include <ui.h>

// ----------------------------------------------------------------------------
// Definitions
// ----------------------------------------------------------------------------
HWCDC USBSerial;
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
Adafruit_XCA9554 expander;

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Local Variables
// ----------------------------------------------------------------------------
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];

lv_obj_t *label;  // Global label object
SensorPCF85063 rtc;
uint32_t lastMillis;

// --- App state variables ---
bool timerRunning = false;
uint32_t countdownSeconds = 55 * 60; // Initial countdown time: 55 minutes
uint32_t life = 0; // Life counter
static uint8_t lifeCount = 0;

lv_timer_t *countdownTimer = nullptr; // LVGL timer for countdown

uint32_t dbg_counter = 0;

// --- Timestamps for long press debounce ---
unsigned long lastTimerLongPressTime = 0;
unsigned long lastLifeUpLongPressTime = 0;
const unsigned long DEBOUNCE_TIME_MS = 500;

// --- Battery and Sleep Optimization Variables ---
const unsigned long BATTERY_UPDATE_INTERVAL_MS = 30000; // Update battery every 30 seconds
lv_timer_t *batteryUpdateTimer = nullptr;

// ----------------------------------------------------------------------------
// Static Function Prototypes (Declarations)
// ----------------------------------------------------------------------------
static void adcOn();
static void adcOff();
static void Arduino_IIC_Touch_Interrupt(void);
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
static void example_increase_lvgl_tick(void *arg);
static void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
static void update_minutes_display();
static void update_life_display();
static void update_battery_display();
static void countdown_task(lv_timer_t *timer);
static void timer_button_cb(lv_event_t *e);
static void life_up_cb(lv_event_t *e);
static void life_down_cb(lv_event_t *e);
static void battery_update_task(lv_timer_t *timer);

// ----------------------------------------------------------------------------
// Class Objects
// ----------------------------------------------------------------------------
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
  LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

Arduino_SH8601 *gfx = new Arduino_SH8601(
    bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, LCD_WIDTH /* width */, LCD_HEIGHT /* height */);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt));

// ----------------------------------------------------------------------------
// Functions
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void Arduino_IIC_Touch_Interrupt(void) {
  FT3168->IIC_Interrupt_Flag = true;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
#if LV_USE_LOG != 0
/* Serial debugging */
static void my_print(const char *buf) {
  USBSerial.printf(buf);
  USBSerial.flush();
}
#endif

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
/* Display flushing */
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
void example_increase_lvgl_tick(void *arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static uint8_t count = 0;
void example_increase_reboot(void *arg) {
  count++;
  if (count == 30) {
    esp_restart();
  }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  int32_t touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  int32_t touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

  if (FT3168->IIC_Interrupt_Flag == true) {
    FT3168->IIC_Interrupt_Flag = false;
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;

    USBSerial.print("Data x ");
    USBSerial.print(touchX);

    USBSerial.print("Data y ");
    USBSerial.println(touchY);
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        // static uint8_t cnt = 0;
        lifeCount++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", lifeCount);
    }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
void setup() {
  USBSerial.begin(115200); /* prepare for possible serial debug */
  Wire.begin(IIC_SDA, IIC_SCL);

  if (!expander.begin(0x20)) {  // Replace with actual I2C address if different
    Serial.println("Failed to find XCA9554 chip");
    while (1)
      ;
  }

  expander.pinMode(4, OUTPUT);
  expander.pinMode(5, OUTPUT);
  expander.digitalWrite(4, 1);
  expander.digitalWrite(5, 1);
  if (!rtc.begin(Wire, IIC_SDA, IIC_SCL)) {
    USBSerial.println("Failed to find PCF8563 - check your wiring!");
    while (1) {
      delay(1000);
    }
  }

  uint16_t year = 2024;
  uint8_t month = 9;
  uint8_t day = 24;
  uint8_t hour = 11;
  uint8_t minute = 9;
  uint8_t second = 41;

  rtc.setDateTime(year, month, day, hour, minute, second);

  // pinMode(LCD_EN, OUTPUT);
  // digitalWrite(LCD_EN, HIGH);
  while (FT3168->begin() == false) {
    delay(2000);
  }
  USBSerial.println("FT3168 initialization successfully");

  gfx->begin();
  gfx->setBrightness(200);

  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  USBSerial.println(LVGL_Arduino);
  USBSerial.println("I am LVGL_Arduino");

  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                 FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);


  lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };

  const esp_timer_create_args_t reboot_timer_args = {
    .callback = &example_increase_reboot,
    .name = "reboot"
  };

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  // Initialize the UI created with SquareLine Studio
  ui_init();

  // Add a single event callback to the timer and lifeUp buttons to handle all events.
  // The logic inside the callback now correctly distinguishes between click and long press.
  lv_obj_add_event_cb(ui_timer, timer_button_cb, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_lifeUp, life_up_cb, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_lifeDown, life_down_cb, LV_EVENT_CLICKED, NULL);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  delay(5);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void update_minutes_display() {
  char buffer[6];
  snprintf(buffer, sizeof(buffer), "%02lu:%02lu", countdownSeconds / 60, countdownSeconds % 60);
  lv_label_set_text(ui_minutes, buffer);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void update_life_display() {
  char buffer[8];
  snprintf(buffer, sizeof(buffer), "%lu", life);
  lv_label_set_text(ui_Life, buffer);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void update_battery_display() {
  // Battery measurement should only happen when the ADC is enabled
  // String percentStr = String(power.getBatteryPercent()) + "%";
  // lv_label_set_text(ui_percent, percentStr.c_str());
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void countdown_task(lv_timer_t *timer) {
  if (timerRunning && countdownSeconds > 0) {
    countdownSeconds--;
    update_minutes_display();
  }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void timer_button_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED) {
    // Ignore clicks that happen shortly after a long press
    if ((millis() - lastTimerLongPressTime) < DEBOUNCE_TIME_MS) {
      return;
    }
    timerRunning = !timerRunning;
  } else if (code == LV_EVENT_LONG_PRESSED) {
    lastTimerLongPressTime = millis(); // Record the time of the long press
    countdownSeconds = 55 * 60;
    update_minutes_display();
    timerRunning = false;
  }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void life_up_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED) {
    // Ignore clicks that happen shortly after a long press
    if ((millis() - lastLifeUpLongPressTime) < DEBOUNCE_TIME_MS) {
      return;
    }
    life++;
    update_life_display();
  } else if (code == LV_EVENT_LONG_PRESSED) {
    lastLifeUpLongPressTime = millis(); // Record the time of the long press
    life = 0;
    update_life_display();
  }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void life_down_cb(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    if (life > 0) {
      life--;
      update_life_display();
    }
  }
}
