/* Copyright (c) 2013, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/  
#include "common.h"
#include "uart.h"
#include "fat.h"
#include "config.h"
#include "timer.h"
#include "buff.h"
#include "a2d.h"
#include "bt.h"
#include "emg.h"
#include "accel.h"
#include "sp02.h"
#include "accel.h"
#include "mmc.h"
#include "i2c.h"
#include "data.h"
#include "ad8557.h"
#include "ringbuffer.h"

#define __MAIN_C_


extern volatile uint8_t buff_flushReady;
extern volatile uint8_t uart_status;
extern volatile uint8_t x;
extern volatile uint8_t ad5933_mode;
extern volatile uint32_t bt_startTime;
extern volatile uint32_t bt_killTime;
volatile uint32_t rtc;
volatile uint32_t spin=0;
volatile uint16_t  main_status;
volatile enum serialState_t serialState = NONE;
uint8_t disk_reset = 0;
volatile istate_t istate;

void fs_open(void) {
  if (buff_init() == OK)
    main_status |= _DISK_WRITE_OK;
}

void fs_connect(void) {
  if (fat_init() == OK) {
    main_status |= _FILESYSTEM_OK;
    fs_open();
  }
}


void main(void) {
  volatile uint16_t i;
  main_status =    _DISK_WRITE |  _SAMPLE_VOLTAGE | _SAMPLE_PULSEOX |
                   _SAMPLE_ECG | _SAMPLE_EMG | _SAMPLE_ACCEL | _SAMPLE_BIOI;


  // configure XT2 16Mhz XTAL
  // ACLK is the 32khz clock xtal
  WDTCTL = WDTPW + WDTHOLD;  // Stop WDT
  BCSCTL1 = 0x00; // XT2ON, XFXT1 LF mode, ACLK div/1me
  BCSCTL3 = XT2S_2 | XCAP_3; //16Mhz, 32768Hz, 12.5pF, ACLK=LFXT1
  while (IFG1 & OFIFG) {  // wait for oscillator to settle
    IFG1 &= ~OFIFG;
    for (i = 0x2FFF; i > 0; i--);
  }
  BCSCTL2 |= SELM_2 + SELS;  // MCLK = SMCLK = XT2 (safe)
  
  ENABLE_INTERRUPTS();

   // configure misc ports
  P1DIR |= _BV(0); // set LED port to output
  LED_ON();

  // dissable the various chip selects on the PCB
  P4DIR |= _BV(4);
  P4OUT |= _BV(4);
  P5DIR |= _BV(4);
  P5OUT |= _BV(4);
  P3DIR |= _BV(6); //ACCEL
  P3OUT |= _BV(6); //

  // Initialize Peripherals
  rtc = 0;    // setup clock
  bt_off();
  uart_init();
 // bt_on();
 // bt_init(); //enable bluetooth, wait for it to finis init
  spi_init();
  accel_init(RANGE_2G,BANDWIDTH_25);
  a2d_init();  
  sp02_init();
  ad8557_init(74,2); // Gain = 4.016 * 25 
  //ad8557_init(0,0); // Gain = 4.016 * 25 
  ad5933_init();     //bioI & I2C CONFIG
  timer_init();      // this starts aquiring data
  LED_OFF(); 

  /*
  ** If _DISK_WRITE then the filesystem writes are enabled 
  ** fs_connect() -> _FILESYSTEM_OK
  ** fs_open() -> _DISK_WRITE_OK
  */

  while (1) {
main_status |= _BT_TRANSMIT;//XXX
    //__low_power_mode_3();
    
    //Attempt to connect to disk if present
    if ((main_status & _DISK_WRITE) && !(main_status & _DISK_WRITE_OK)) {
       if (!(main_status & _FILESYSTEM_OK)) fs_connect();
       if (main_status & _FILESYSTEM_OK && !(main_status & _DISK_WRITE_OK)) fs_open();
    }
    
    // if data to be flushed AND filesystem writes are enabled..
    if (main_status & (_DISK_WRITE | _FILESYSTEM_OK | _DISK_WRITE_OK | _DISK_TX_OK)) {
        buff_flush(); //write out data to disk if we have enough
        main_status &= ~_DISK_TX_OK; //clear write flag
    }
    
    // handle messages from the BT module
    if (uart_status & UART_RX_READY) uart_handle(); 
    
    // start BT transmit after sufficient delay 
    if (bt_startTime && rtc > bt_startTime) {
        main_status |= _BT_TRANSMIT;
        bt_startTime = 0;
    }
    // restart the BT module after sufficient delay
    if (bt_killTime && rtc > bt_killTime) {
        bt_off();
        uart_init();
        bt_init();
        bt_killTime = 0;
    }

    spin++; //counter to determine how many times we've been in this loop
  }

}



