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
#include "data.h"
#include "timer.h"
#include "uart.h"
#include "buff.h"
#include "ringbuffer.h"
#include <string.h>

extern rb buff;
extern volatile uint32_t timer_seconds;
volatile struct data_t data;


void data_init(void) {
  uint8_t i;
  uint8_t *ptr = (uint8_t*)(&data);
  for (i=0; i<sizeof(data); i++) {
    ptr[i] = 0;
  }
}

void static inline _add(uint8_t x, uint16_t y) {
  if (main_status & _BT_TRANSMIT) {
    uart_putc(x);
    uart_putc(y>>8);
    uart_putc(y&0xFF);
  }

  if (main_status & _DISK_WRITE) {
    rb_push(&buff, x);
    rb_push(&buff, (y>>8)  );
    rb_push(&buff, (y&0xFF));
  }
}

void static inline _add8(uint8_t x) {
   if (main_status & _BT_TRANSMIT) uart_putc(x);
   if (main_status & _DISK_WRITE)  rb_push(&buff, x);
}


#define ASCII_EOL 0x0A
void data_text(enum buff_data_t dataType, uint8_t *str) {
  uint8_t len,i,z=0;

  len = strlen((char const *)str);
  for (i=0; i<len; i+=2) {
      _add8(dataType);
      _add8(str[i]);
      if (i+1<len) {
          _add8(str[i++]);
       } else {
          _add8(ASCII_EOL);
          z=1;
      }
  }
  if (!z) {
      _add8(dataType);
      _add8(ASCII_EOL);
      _add8(ASCII_EOL);
  }

}


#define MID_VALUE (0xFF/2)
uint8_t _un2sComplement(uint8_t x) {
    if (x&_BV(7)) {
        x = ~x+1;
        return MID_VALUE - x;
    } else {
        return MID_VALUE + x;
    }
}

void data_accelAdd(uint8_t x, uint8_t y, uint8_t z) {
    _add( B_ACCEL_X, _un2sComplement(x));
    _add( B_ACCEL_Y, _un2sComplement(y));
    _add( B_ACCEL_Z, _un2sComplement(z));
}

void data_add(enum buff_data_t dataType, uint16_t value) {
//XXX 
  LED_TOGGLE();
  //XXX
    _add(dataType, value);
}

void data_sync(void) {
  if (main_status & _BT_TRANSMIT) {
    uart_putc(0x55);
    uart_putc(0x55);
    uart_putc(0x55);
  }
}

void data_timestamp(void) {
  _add(B_TIMESTAMP_HI,  (rtc>>16) );
  _add(B_TIMESTAMP_LO,  (rtc&0xFFFF) );
}
