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

#define PICO_ENTER_USB_BOOT_ON_EXIT 0
#define DT 6 //encoder pin
#define CLK 7 //encoder pin

#define PIN_SCK 2
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_CS 5
#define SPI_PORT spi0

volatile int knobDelta = 0;
static int8_t wheel_multiplier = 1;

typedef struct TU_ATTR_PACKED {
  uint8_t buttons;
  int8_t x;
  int8_t y;
  int8_t wheel;
} mouse_report_t;

typedef struct TU_ATTR_PACKED {
  int8_t wheel;
} wheel_report_t;

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
int main(){
  board_init();

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

  int i = 10;
  while(1){
    tud_task();
    if(tud_hid_ready() && knobDelta != 0){
      wheel_report_t wheel_report = {0};
      wheel_report.wheel = (int8_t)knobDelta;
      tud_hid_report(REPORT_ID_MOUSE,&wheel_report,sizeof(wheel_report));
      knobDelta = 0;
    }
//    sleep_ms(100);
  }
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