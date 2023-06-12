#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <math.h>
#include "pico/binary_info.h"

#include "bsp/board.h"
#include "tusb.h"
#include "hardware/i2c.h"

#define GPIO_INPUT false
#define GPIO_OUTPUT true

enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

#define LED 25


static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

void led_blinking_task(void);
void midi_task(void);

uint columns[16] = { 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 };  // 16 input lines
uint rows[4] = { 3, 4, 5, 6 };  // 1 of 16 output scan

absolute_time_t c1[256];
absolute_time_t c2[256];
char cdown[256];


absolute_time_t cleared;

void pico_keybed_init() {

    cleared = get_absolute_time();

    for (int i = 0; i < 256; i++) {
        c1[i] = nil_time;
        c2[i] = nil_time;
	cdown[i]=0;
    }

    for (int i = 7; i < 23; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_INPUT);
        gpio_pull_down(i);
    }
    for (int i = 3; i < 7; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_OUTPUT);
        gpio_put(i, 0);
    }

}

void pico_keybed_scan(void) {
    int row,col,kcount,key,clearkey;
    uint32_t cols,cx;
    bool pressed = false;


    kcount = 0;
    key = -1;
    
    for (row = 0; row < 11; row++) {  // can go to 16 rows

        if (row == 0) {  gpio_put(6, 0); gpio_put(5, 0); gpio_put(4, 0); gpio_put(3, 0);}
        if (row == 1) {  gpio_put(6, 0); gpio_put(5, 0); gpio_put(4, 0); gpio_put(3, 1);}
        if (row == 2) {  gpio_put(6, 0); gpio_put(5, 0); gpio_put(4, 1); gpio_put(3, 0);}
        if (row == 3) {  gpio_put(6, 0); gpio_put(5, 0); gpio_put(4, 1); gpio_put(3, 1);}
        if (row == 4) {  gpio_put(6, 0); gpio_put(5, 1); gpio_put(4, 0); gpio_put(3, 0);}
        if (row == 5) {  gpio_put(6, 0); gpio_put(5, 1); gpio_put(4, 0); gpio_put(3, 1);}
        if (row == 6) {  gpio_put(6, 0); gpio_put(5, 1); gpio_put(4, 1); gpio_put(3, 0);}
        if (row == 7) {  gpio_put(6, 0); gpio_put(5, 1); gpio_put(4, 1); gpio_put(3, 1);}
        if (row == 8) {  gpio_put(6, 1); gpio_put(5, 0); gpio_put(4, 0); gpio_put(3, 0);}
        if (row == 9) {  gpio_put(6, 1); gpio_put(5, 0); gpio_put(4, 0); gpio_put(3, 1);}
        if (row ==10) {  gpio_put(6, 1); gpio_put(5, 0); gpio_put(4, 1); gpio_put(3, 0);}
        if (row ==11) {  gpio_put(6, 1); gpio_put(5, 0); gpio_put(4, 1); gpio_put(3, 1);}
        if (row ==12) {  gpio_put(6, 1); gpio_put(5, 1); gpio_put(4, 0); gpio_put(3, 0);}
        if (row ==13) {  gpio_put(6, 1); gpio_put(5, 1); gpio_put(4, 0); gpio_put(3, 1);}
        if (row ==14) {  gpio_put(6, 1); gpio_put(5, 1); gpio_put(4, 1); gpio_put(3, 0);}
        if (row ==15) {  gpio_put(6, 1); gpio_put(5, 1); gpio_put(4, 1); gpio_put(3, 1);}


        busy_wait_us(1);
        cols = gpio_get_all();
        for (col = 15; col < 23; col++) { // can go to 23 columns
	    cx = cols & (1 << col);

	    key = (row  * 4 + ((col-15)/2))+29;
	    if ((col & 1) != 0){
              if (cx != 0x0) {
		pressed = true;
                kcount++;
		if ( c1[key] == nil_time) {
                  c1[key] = get_absolute_time();
		}
	      }else{
	    	c1[key] = nil_time;
	      }
	    }else{
              if (cx != 0x0) {
		pressed = true;
                kcount++;
		if ( c2[key] == nil_time) {
                  c2[key] = get_absolute_time();
		}
	      }else{
	    	c2[key] = nil_time;
	      }
	    }
	    
	}
    }

  if(pressed){
    board_led_write(true);
  }else{
    board_led_write(false);
  }

}

void pico_keybed_irq_enable(bool enable, gpio_irq_callback_t callback) {
    for (int i = 0; i < 8; i++) {
        gpio_set_irq_enabled_with_callback(columns[i], 0x8, enable, callback);
    }
}

#define FLAG_VALUE 123
 
void core1_entry() {
 
    multicore_fifo_push_blocking(FLAG_VALUE);
 
    uint32_t g = multicore_fifo_pop_blocking();
 
    if (g != FLAG_VALUE){
//        printf("Hmm, that's not right on core 1!\n");
    }else{
//        printf("Its all gone well on core 1!");
    }
    while (1){
	    pico_keybed_scan();
    }
}


int main() {
  board_init();

  tusb_init();

    // Initialise UART 0
  uart_init(uart0, 31250);
  // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
  gpio_set_function(0, GPIO_FUNC_UART);
  gpio_set_function(1, GPIO_FUNC_UART);

  pico_keybed_init();
  multicore_reset_core1();
  multicore_launch_core1(core1_entry);
 
  // Wait for it to start up
 
  uint32_t g = multicore_fifo_pop_blocking();
 
  if (g != FLAG_VALUE){
  } else {
      multicore_fifo_push_blocking(FLAG_VALUE);
  }



  while (1)
  {
    tud_task(); // tinyusb device task
//    led_blinking_task();
    midi_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}



//--------------------------------------------------------------------+
// UART Helper
//--------------------------------------------------------------------+

static size_t
__time_critical_func (uart_read) (uart_inst_t *uart, uint8_t *dst, size_t maxlen)
{
  size_t result = 0;

  while (maxlen && uart_is_readable(uart))
  {
    *dst++ = uart_getc(uart);

    result++;
    maxlen--;
  }

  return result;
}


void midi_task(void)
{
  static uint32_t start_ms = 0;
  int diff = 0;
  int speed = 0;
  int down = 0;
  int i;
  char akey;  
  char mkey;
  char disp = 0;
  
  for (i = 0; i<128; i++){
   

    akey = i;
    mkey = i;

    uint8_t packet[4];

    if((c2[akey]==nil_time)&&(cdown[mkey]==1)){
      tudi_midi_write24(0, 0x80, mkey , 0);
      packet[0]=80;packet[1]=mkey,packet[2]=0;uart_write_blocking(uart0, packet, 3);
      cdown[mkey]=0;
    }
    if((c2[akey]!=nil_time)&&(cdown[mkey]==0)){
      diff = (absolute_time_diff_us(c1[akey],c2[akey])  ) >> 11 ;
      speed = (127 - (diff*2))+5;
      if (speed > 127) speed = 127;
      if (speed < 32) speed = 32;
     
      tudi_midi_write24(0, 0x90, mkey, speed); // - (speed>>5));
      packet[0]=90;packet[1]=mkey,packet[2]=speed;uart_write_blocking(uart0, packet, 3);
      cdown[mkey]=1;
    }
    


    if(cdown[mkey]==1) down++;
   
  }

//  if(down>0){
//    board_led_write(true);
//  }else{
//    board_led_write(false);
//  }
  

}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
