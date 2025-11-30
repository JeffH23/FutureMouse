#include "pico/stdlib.h"
#include <stdlib.h>
#include <stdio.h>
#include "pico/time.h"
#include <string.h>

#include "tusb_config.h"
#include "tusb.h"
#include "bsp/board_api.h"
#include "usb_descriptors.h"

#include "hardware/spi.h"
#include "PMW3360_Firmware.h"

#define PICO_ENTER_USB_BOOT_ON_EXIT 0
#define DT 6 //encoder pin
#define CLK 7 //encoder pin

/////PMW3360 setup
#define PIN_SCK 2
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_CS 5
#define SPI_PORT spi0

#define DELTA_X_L 0x03
#define DELTA_X_H 0x04
#define DELTA_Y_L 0x05
#define DELTA_Y_H 0x06

#define REST_EN_BIT_0 0x0//0b00000000 //sets the Rest_En bit of the config_2 register to 0
                      //..^.....

#define CONFIG_2 0x10         // address of Config_2 register
#define SROM_ENABLE 0x13      // address of SROM_Enable register
#define SROM_LOAD_BURST 0x62  // address of SROM_Load_Burst register
#define SROM_ID 0x2A          // address of SROM_ID register

volatile int knobDelta = 0;
static int8_t wheel_multiplier = 1;

typedef struct TU_ATTR_PACKED {
  uint8_t buttons;
  int16_t x;
  int16_t y;
  int8_t wheel;
} mouse_report_t;

typedef struct TU_ATTR_PACKED {
  int8_t wheel;
} wheel_report_t;

typedef struct {
  uint16_t x;
  uint16_t y;
}Delta_xy;

void knob_init(){
  gpio_init(DT);
  gpio_set_dir(DT, GPIO_IN);
  gpio_pull_up(DT);

  gpio_init(CLK);
  gpio_set_dir(CLK, GPIO_IN);
  gpio_pull_up(CLK);
}
void led_init(){
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

void handle_encoder(void){//A=DT B=CLK
  static bool a_expect = false;
  static bool b_expect = false;
  static int last_time = 0;
  int now = to_ms_since_boot(get_absolute_time());
  if(now - last_time < 2) return;
  last_time = now;
  if(gpio_get_irq_event_mask(DT) & GPIO_IRQ_EDGE_RISE){
    gpio_acknowledge_irq(DT, GPIO_IRQ_EDGE_RISE);
    if(!gpio_get(CLK)){
      b_expect = true;
      a_expect = false;
      return;
    }
    else if(a_expect){
      a_expect = false;
      knobDelta++;
      printf("%d\n",knobDelta);
    }
  }
if(gpio_get_irq_event_mask(CLK) & GPIO_IRQ_EDGE_RISE){
    gpio_acknowledge_irq(CLK, GPIO_IRQ_EDGE_RISE);
    if(!gpio_get(DT)){
      b_expect = false;
      a_expect = true;
      return;
    }
    else if(b_expect){
      b_expect = false;
      knobDelta--;
      printf("%d\n",knobDelta);
    }
  }
  gpio_acknowledge_irq(DT, gpio_get_irq_event_mask(DT));
  gpio_acknowledge_irq(CLK, gpio_get_irq_event_mask(CLK));
}
static inline void read_reg(uint8_t reg, uint8_t *data);
static inline void write_reg(uint8_t reg, uint8_t *data);
static inline void pmw3360_delay(void);

uint8_t read_motion(Delta_xy *delta);

void PMW3360_init();

int main(){
  board_init();

  //init spi
  sleep_ms(10);
  spi_init(spi0, 19200);
  spi_set_format(spi0,8,SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

  // initialise chip select as driven-high
  gpio_init(PIN_CS);
  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_put(PIN_CS, 1);
  sleep_ms(10);

  PMW3360_init();

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }
  stdio_init_all();
  led_init();
  knob_init();

  gpio_add_raw_irq_handler(DT,handle_encoder);
  gpio_set_irq_enabled(DT, GPIO_IRQ_EDGE_RISE, true); //enables interrupts for gpio DT
  gpio_add_raw_irq_handler(CLK,handle_encoder);
  gpio_set_irq_enabled(CLK, GPIO_IRQ_EDGE_RISE, true); //enables interrupts for gpio CLK
  irq_set_enabled(IO_IRQ_BANK0, true);

  while(1){
    tud_task();
    Delta_xy delta;
    uint8_t motion = read_motion(&delta);
    if(motion){
      printf("x:%d, y:%d\n", (int16_t)delta.x, (int16_t)delta.y);
    }
    mouse_report_t mouse_report = {0};
    if(tud_hid_ready() && (knobDelta || motion)){
      mouse_report.wheel = (int8_t)knobDelta;
      mouse_report.x = (int16_t)delta.x;
      mouse_report.y = (int16_t)delta.y;
      tud_hid_report(/*REPORT_ID_MOUSE*/2,&mouse_report,sizeof(mouse_report_t));
      knobDelta = 0;
      motion = 0x0;
    }
    
//    sleep_ms(100);
  }
}

void PMW3360_init(){
  //sensor boot
  uint8_t data;
  uint8_t reg;

  gpio_put(PIN_CS, 0);
  sleep_ms(10);
  gpio_put(PIN_CS, 1);
  data = 0x5A;
  write_reg(0x3A, &data); //write 0x5A to POWER_UP_RESET register
  sleep_ms(55);
  read_reg(0x02, &data);
  sleep_us(20);
  read_reg(0x03, &data);
  sleep_us(20);
  read_reg(0x04, &data);
  sleep_us(20);
  read_reg(0x05, &data);
  sleep_us(20);
  read_reg(0x06, &data);
  sleep_us(20);

  read_reg(0x00, &data); // read chip ID
  sleep_us(20);
  printf("chip ID is:0x%02x\n", data);

  //load SROM Firmware
  uint8_t REST_Enable_Value = REST_EN_BIT_0;
  write_reg(CONFIG_2, &REST_Enable_Value);
  uint8_t SROM_Enable_Value = 0x1d;
  write_reg(SROM_ENABLE,&SROM_Enable_Value);
  sleep_ms(10);
  SROM_Enable_Value = 0x18;
  write_reg(SROM_ENABLE, &SROM_Enable_Value);

  gpio_put(PIN_CS, 0);
  pmw3360_delay();
  uint8_t SROM_Load_Burst_Register = SROM_LOAD_BURST | 0x80;
  spi_write_blocking(spi0, &SROM_Load_Burst_Register, 1);
  sleep_us(20);
  for(int i=0; i < firmware_length; i++){
    // raw spi write
    spi_write_blocking(spi0, &firmware_data[i], 1);
    sleep_us(20);
  }
  gpio_put(PIN_CS, 1);
  sleep_us(1);
  sleep_us(250);

  write_reg(CONFIG_2,&REST_Enable_Value);
  sleep_ms(10);
  uint8_t SROM_ID_Value = 0x7;
  read_reg(SROM_ID, &SROM_ID_Value);
  printf("SROM_ID:0x%02x\n", SROM_ID_Value);
}

static inline void read_reg(uint8_t reg, uint8_t *data){
  gpio_put(PIN_CS, 0);
  pmw3360_delay();
  spi_write_blocking(spi0, &reg, 1);
  busy_wait_us(160);
  spi_read_blocking(spi0, 0, data, 1);
  pmw3360_delay();
  gpio_put(PIN_CS, 1);
}
static inline void write_reg(uint8_t reg, uint8_t *data){
  gpio_put(PIN_CS, 0);
  uint8_t Buffer[2];
  Buffer[0] = reg | 0b10000000;
  Buffer[1] = *data;
  pmw3360_delay();
  spi_write_blocking(spi0, Buffer, 2);
  pmw3360_delay();
  gpio_put(PIN_CS, 1);
}
static inline void pmw3360_delay(void){
   asm volatile(
    "nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t"
    "nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t"
    "nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t"
    "nop\n\t"
    ::: "memory");
}
uint8_t read_motion(Delta_xy *delta){
  uint8_t reg = 0x02;
  uint8_t motion_bit = 0x00;
  uint8_t data = 0x00;
  uint16_t delta_x = 0x0;
  uint16_t delta_y = 0x0;
  write_reg(reg, &motion_bit);
  read_reg(reg, &motion_bit);
  if(motion_bit & 0b10000000){
    uint8_t delta_L;
    read_reg(DELTA_X_L, (uint8_t *)&delta_x);
    read_reg(DELTA_X_H, &data);
    delta_x = (delta_x & 0xFF) | ((uint16_t)data << 8);
    read_reg(DELTA_Y_L, (uint8_t *)&delta_y);
    read_reg(DELTA_Y_H, &data);
    delta_y = (delta_y & 0xFF) | ((uint16_t)data << 8);

    delta->x = delta_x;
    delta->y= delta_y;

  }else{
    motion_bit = 0x00;
  }
  return motion_bit;
}

void USB0_IRQHandler(void) {
  tusb_int_handler(0, true);
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// Host sets multiplier (Windows does this automatically on good mice)
// === CORRECT FOR TINYUSB 0.15+ (2024-2025) ===
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, 
                           hid_report_type_t report_type, 
                           uint8_t const* buffer, uint16_t bufsize)
{
    (void)instance;

    if (report_id == 1 && report_type == HID_REPORT_TYPE_FEATURE && bufsize >= 1) {
        uint8_t new_mult = buffer[0];
        if (new_mult >= 1 && new_mult <= 14) {
            wheel_multiplier = new_mult;
            printf("HIGH-RES MULTIPLIER SET TO %d×\n", wheel_multiplier);
        }
    }
}