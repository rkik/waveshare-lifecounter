
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
#include "XPowersLib.h" // For power management IC (PMU)

// ----------------------------------------------------------------------------
// Definitions
// ----------------------------------------------------------------------------
#define DISPLAY_BUFFER_DIVIDER (20)
#define DISPLAY_BUFFER_SIZE ((LCD_WIDTH * LCD_HEIGHT) / DISPLAY_BUFFER_DIVIDER)
  
#define DBG_ENTER(x) {USBSerial.print("Enter "); USBSerial.println(x);}
#define DBG_EXIT(x) {USBSerial.print("Exit "); USBSerial.println(x);}

#define LIFE_LOG_ENTRY_MAX (50)

typedef struct {
  uint32_t timestamp;
  int lifeChange;
  int lifeTotal;
} LIFE_LOG_S;

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
// Global variables from your working code (brightness, PMU flags)
int bri[4] = {100, 150, 200, 250}; // Brightness levels
int b = 0; // Current brightness index - set to lowest for better battery life
Adafruit_XCA9554 expander;
HWCDC USBSerial;

// ----------------------------------------------------------------------------
// Local Variables
// ----------------------------------------------------------------------------
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[DISPLAY_BUFFER_SIZE];

lv_obj_t *label;  // Global label object
SensorPCF85063 rtc;
uint32_t lastMillis;

// --- App state variables ---
bool timerRunning = false;
uint32_t countDownMaxSeconds = 55 * 60; // Initial countdown time: 55 minutes
uint32_t countdownSeconds = countDownMaxSeconds; 
uint32_t logTimerSeconds = 0; 
int32_t life = 0; // Life counter
int32_t tempLife = 0; // Life counter
static uint8_t lifeCount = 0;
lv_timer_t *lifeUpdateTimer = nullptr;

lv_timer_t *countdownTimer = nullptr; // LVGL timer for countdown

uint32_t dbg_counter = 0;

uint8_t LifeLogIndex = 0;
static LIFE_LOG_S LifeLog[LIFE_LOG_ENTRY_MAX];

// --- Timestamps for long press debounce ---
static uint32_t pressTime;
unsigned long lastTimerLongPressTime = 0;
unsigned long lastLifeUpLongPressTime = 0;
const unsigned long DEBOUNCE_TIME_MS = 700;

// --- Battery and Sleep Optimization Variables ---
const unsigned long BATTERY_UPDATE_INTERVAL_MS = 30000; // Update battery every 30 seconds
lv_timer_t *batteryUpdateTimer = nullptr;

// PMU object
XPowersPMU power;

char dbg_buffer[30];
char log_buffer[30];

// ----------------------------------------------------------------------------
// Static Function Prototypes (Declarations)
// ----------------------------------------------------------------------------
static void rtc_init(void);
static void pmu_init(void);
static void adcOn();
static void adcOff();
static void Arduino_IIC_Touch_Interrupt(void);
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
static void example_increase_lvgl_tick(void *arg);
static void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
static void temp_life_update(int value);
static void timer_reset();
static void add_log_entry(int value);
static void reset_log(void);
static void display_log(void);

// Button callbacks
static void timer_button_cb(lv_event_t *e);
static void life_up_click_cb(lv_event_t *e);
static void life_up_long_cb(lv_event_t *e);
static void life_down_cb(lv_event_t *e);
static void settings_enter_cb(lv_event_t *e);
static void settings_back_cb(lv_event_t *e);
static void top_cut_timer_cb(lv_event_t *e);
static void log_enter_cb(lv_event_t *e);
static void log_back_cb(lv_event_t *e);

// Timer tasks
static void battery_update_task(lv_timer_t *timer);
static void life_update_task(lv_timer_t *timer);
static void countdown_task(lv_timer_t *timer);

// Display updates
static void update_temp_life_display();
static void reset_temp_life_display();
static void update_minutes_display();
static void update_life_display();
static void update_battery_display();

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
// Global Functions
// ----------------------------------------------------------------------------

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

  pmu_init();

  // pinMode(LCD_EN, OUTPUT);
  // digitalWrite(LCD_EN, HIGH);
  while (FT3168->begin() == false) {
    delay(2000);
  }
  USBSerial.println("FT3168 initialization successfully");

  gfx->begin();
  gfx->setBrightness(100);

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


  lv_disp_draw_buf_init(&draw_buf, buf, NULL, DISPLAY_BUFFER_SIZE);

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
  indev_drv.long_press_time = 1000;
  lv_indev_drv_register(&indev_drv);

  // Initialize the UI created with SquareLine Studio
  ui_init();

  // Add event callbacks to the timer and lifeUp buttons to handle the main events.
  // The logic inside the callback now correctly distinguishes between click and long press.
  lv_obj_add_event_cb(ui_timer, timer_button_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(ui_timer, timer_button_cb, LV_EVENT_LONG_PRESSED, NULL);
  lv_obj_add_event_cb(ui_lifeUp, life_up_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(ui_lifeUp, life_up_cb, LV_EVENT_LONG_PRESSED , NULL);
  lv_obj_add_event_cb(ui_lifeDown, life_down_cb, LV_EVENT_CLICKED, NULL);

  // Settings
  lv_obj_add_event_cb(ui_topCutTimerEnable, top_cut_timer_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // Add screen change buttong
  lv_obj_add_event_cb(ui_settingsButton, settings_enter_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(ui_settingsBack, settings_back_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(ui_LogButton, log_enter_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_add_event_cb(ui_logBack, log_back_cb, LV_EVENT_CLICKED, NULL);

  // Initial display updates
  update_minutes_display();
  update_life_display();
  update_battery_display();

  // Create LVGL timers
  countdownTimer = lv_timer_create(countdown_task, 1000, NULL);
  batteryUpdateTimer = lv_timer_create(battery_update_task, BATTERY_UPDATE_INTERVAL_MS, NULL);
  lifeUpdateTimer = lv_timer_create(life_update_task, 1000, NULL);
  lv_timer_pause(lifeUpdateTimer);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  delay(3);
}

// ----------------------------------------------------------------------------
// Local Functions
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void rtc_init(void) {
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
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void pmu_init(void) {
  // Initialize PMU
  bool result = power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);
  if (result == false) {
    USBSerial.println("PMU is not online...");
    while (1) delay(50);
  }

  // PMU configuration for wake-up from power key press
  power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  power.setChargeTargetVoltage(3);
  power.clearIrqStatus();

  // The enableIRQ function expects one argument at a time
  power.enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ);
  power.enableIRQ(XPOWERS_AXP2101_VBUS_INSERT_IRQ);
  
  // Set up deep sleep to wake on power button press. - TODO get correct GPIO
  // const gpio_num_t PMU_IRQ_GPIO = GPIO_NUM_16;
  // esp_sleep_enable_ext1_wakeup(PMU_IRQ_GPIO, 0); // 0 = low level wakeup

  // Initial ADC state is OFF for power saving
  adcOff();
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void Arduino_IIC_Touch_Interrupt(void) {
  FT3168->IIC_Interrupt_Flag = true;
  pressTime = millis();
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// #if LV_USE_LOG != 0
/* Serial debugging */
static void my_print(const char *buf) {
  USBSerial.printf(buf);
  USBSerial.flush();
}
// #endif

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
static void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  int32_t touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  int32_t touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
  static lv_indev_state_t lastState = LV_INDEV_STATE_REL;
  uint32_t readTime = millis();

  // DBG_ENTER("my_touchpad_read")
  if (FT3168->IIC_Interrupt_Flag == true) {
    FT3168->IIC_Interrupt_Flag = false;
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;

    // USBSerial.print("Data x ");
    // USBSerial.print(touchX);

    // USBSerial.print("Data y ");
    // USBSerial.println(touchY);
  } else if ((readTime - 100) > pressTime) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = lastState;
  }

  lastState = data->state;
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
  snprintf(buffer, sizeof(buffer), "%i", life);
  lv_label_set_text(ui_Life, buffer);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void update_temp_life_display() {
  char buffer[8];
  if (tempLife > 0) {
    // Add a plus sign to the text
    snprintf(buffer, sizeof(buffer), "+%i", tempLife);
  } else {
    snprintf(buffer, sizeof(buffer), "%i", tempLife);
  }
  lv_label_set_text(ui_tempLife, buffer);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void reset_temp_life_display() {
  lv_label_set_text(ui_tempLife, "");
  lv_timer_pause(lifeUpdateTimer);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void adcOn() {
  power.enableTemperatureMeasure();
  power.enableBattDetection();
  power.enableVbusVoltageMeasure();
  power.enableBattVoltageMeasure();
  power.enableSystemVoltageMeasure();
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void adcOff() {
  power.disableTemperatureMeasure();
  power.disableBattDetection();
  power.disableVbusVoltageMeasure();
  power.disableBattVoltageMeasure();
  power.disableSystemVoltageMeasure();
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void update_battery_display() {
  // Battery measurement should only happen when the ADC is enabled
  String percentStr = String(power.getBatteryPercent()) + "%";
  lv_label_set_text(ui_percent, percentStr.c_str());
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void countdown_task(lv_timer_t *timer) {
  if (timerRunning && countdownSeconds > 0) {
    countdownSeconds--;
    logTimerSeconds = countdownSeconds;
    update_minutes_display();
  } else {
    logTimerSeconds++;
  }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
void battery_update_task(lv_timer_t *timer) {
  // Temporarily enable ADC to get the measurement
  adcOn();
  delay(100); // Small delay to allow ADC to stabilize
  update_battery_display();
  adcOff(); // Disable ADC immediately after measurement
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
void life_update_task(lv_timer_t *timer) {
  life += tempLife;
  if (life < 0) {
    life = 0;
  }

  // Log the change in life
  add_log_entry(tempLife, life);
 
  // snprintf(dbg_buffer, sizeof(dbg_buffer), "LUT: tempLife: %d - life: %d\n", tempLife, life);
  // my_print(dbg_buffer);

  tempLife = 0;
  update_life_display();
  lv_timer_pause(lifeUpdateTimer);
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
    timer_reset();
  }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void timer_reset() {  
    countdownSeconds = countDownMaxSeconds;
    update_minutes_display();
    timerRunning = false;
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
    
    temp_life_update(1);
  } else if (code == LV_EVENT_LONG_PRESSED) {
    lastLifeUpLongPressTime = millis(); // Record the time of the long press
    life = 0;
    update_life_display();
    reset_temp_life_display();
    reset_log();
  }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void life_down_cb(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    temp_life_update(-1);
  }
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void settings_enter_cb(lv_event_t *e) {
    // snprintf(dbg_buffer, sizeof(dbg_buffer), "Screen: %s", lv_scr_act());
    // my_print(dbg_buffer);
    lv_scr_load(ui_Settings);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void settings_back_cb(lv_event_t *e) {
    // snprintf(dbg_buffer, sizeof(dbg_buffer), "Screen: %s", lv_scr_act());
    // my_print(dbg_buffer);
    lv_scr_load(ui_MainScreen);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void log_enter_cb(lv_event_t *e) {
    // snprintf(dbg_buffer, sizeof(dbg_buffer), "Screen: %s", lv_scr_act());
    // my_print(dbg_buffer);
    lv_scr_load(ui_LogScreen);
    display_log();
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void log_back_cb(lv_event_t *e) {
    // snprintf(dbg_buffer, sizeof(dbg_buffer), "Screen: %s", lv_scr_act());
    // my_print(dbg_buffer);
    lv_scr_load(ui_MainScreen);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void top_cut_timer_cb(lv_event_t *e) {
    // snprintf(dbg_buffer, sizeof(dbg_buffer), "State: %s\n", lv_obj_has_state(ui_topCutTimerEnable, LV_STATE_CHECKED) ? "On" : "Off");
    // my_print(dbg_buffer);

    if (lv_obj_has_state(ui_topCutTimerEnable, LV_STATE_CHECKED)) {
      countDownMaxSeconds = 75 * 60;
    } else {
      countDownMaxSeconds = 55 * 60;
    }
    
    // Reset the timer value and pause it
    timer_reset();
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void temp_life_update(int value) {
    tempLife += value;
    // snprintf(dbg_buffer, sizeof(dbg_buffer), "TLU: val: %d - tempLife: %d\n", value, tempLife);
    // my_print(dbg_buffer);

    update_temp_life_display();
    lv_timer_reset(lifeUpdateTimer);
    lv_timer_resume(lifeUpdateTimer);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void add_log_entry(int lifeChange, int lifeTotal) {
  // Timestamps
  // - if timer running use the value of the timer
  // - if not running, keep track of time since health was last reset
  LifeLog[LifeLogIndex].lifeChange = lifeChange;
  LifeLog[LifeLogIndex].lifeTotal = lifeTotal;
  LifeLog[LifeLogIndex].timestamp = logTimerSeconds;
  LifeLogIndex++;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void reset_log(void) {
  logTimerSeconds = 0;
  LifeLogIndex = 0;
  memset(LifeLog, 0, sizeof(LifeLog));
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
static void display_log(void) {
  memset(log_buffer, '\0', sizeof(log_buffer));

  lv_textarea_set_text (ui_Log, log_buffer);

  for (uint8_t i; i < LifeLogIndex; i++) {
    int log_minutes = LifeLog[i].timestamp / 60;
    int log_seconds = LifeLog[i].timestamp % 60;
    snprintf(log_buffer, sizeof(log_buffer), "%d: %02d:%02d | %+3d -> %2d\n", 
            i, 
            log_minutes,
            log_seconds,
            LifeLog[i].lifeChange,
            LifeLog[i].lifeTotal);
    // my_print(log_buffer);
    lv_textarea_add_text(ui_Log, log_buffer);
  }
}


