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
#include "timer.h"
#include "buff.h"
#include "data.h"
#include "a2d.h"
#include "config.h"
#include "uart.h"
#include "sp02.h"
#include "d2a.h"
#include "accel.h"
#include "ad5933.h"
#include "bt.h"

/* Notes :
**
** TACCTL0.CCIE needs to be set to trigger TIMERA0_VECTOR
** TACTL.TAIE needs to be set to trigger TIMERA1_VECTOR's OVERFLOW (TAIV=0xA)
**/

uint32_t clock = 1;
uint32_t rtc_old=0;

void timer_init(void) {
  // Set timer in UP mode with frequency of 32.768Khz/4
  TACTL = TACLR; //reset timer A
  TAR = 0;
  TACCR0 = 1; //period = TACCR0+1
  TACTL = TASSEL_1 | ID_0 | MC_1 /*|kk TAIE*/;  // ACLK 1/div, up, interrupt enabled.
  TACCTL0 = CCIE; //enable TIMERA0_VECTOR interrupt

  // I'm not sure if this needs to be configured
  //TACCTL0 = CM_0 | CCIS_2 | OUTMOD_0 | CCIE; //No Capture, GND, OUT bit value, interrupt enabled
}


void timer_eventHandler(void) {
  static uint16_t accum=0;
  uint16_t t;

  if (!(++clock & 0x3FFF)) rtc++; //tick seconds counter

 // if (rtc == 60*30) bt_off(); //XXX turn off bluetooth after 30 minutes

  switch(clock & 0x03F) {
    case 1:
      if (SAMPLE_PULSEOX) {
        SPO2_RED();
        d2a1_set( sp02_red_dc );
      }
      break;
    case 2:
      if (SAMPLE_EMG) a2d_sample( EMG );
      break;
    case 3:
      if (SAMPLE_EMG) data_add( B_EMG, a2d_get( EMG ) );
      break;
    case 4:
      main_status |= _DISK_TX_OK;  // Ok to start write, if needed
      break;
    case 9:
      if (SAMPLE_PULSEOX) {
        accum = 0;
        a2d_sample(PULSEOX_AC);
      }
      break;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
      if (SAMPLE_PULSEOX) {
        t = a2d_get(PULSEOX_AC);
        accum += t;
        a2d_sample(PULSEOX_AC);
      }
      break;
    case 17:
      if (SAMPLE_PULSEOX) {
        SPO2_OFF();
        d2a1_set( sp02_off_dc );
        accum += a2d_get(PULSEOX_AC);
        data_add( B_PULSEOX_RED_AC, accum );
        if ((accum/8) < 512 && sp02_red_dc < 3003) {
            sp02_red_dc++;
            data_add( B_PULSEOX_RED_OUTPUTOFFSET, sp02_red_dc );
        }
        if ((accum/8)  > 3583 && sp02_red_dc > 0) {
            sp02_red_dc--;
            data_add( B_PULSEOX_RED_OUTPUTOFFSET, sp02_red_dc );
        }
      }
      break;
    case 18:
      if (SAMPLE_EMG) a2d_sample( EMG );
      break;
    case 19:
      if (SAMPLE_EMG) data_add( B_EMG, a2d_get( EMG ) );
      break;
    case 20:
      if (SAMPLE_VOLTAGE && (rtc != rtc_old)) a2d_sample(VOLTAGE);
      break;
    case 21:
      if (SAMPLE_VOLTAGE && (rtc != rtc_old)) data_add( B_VOLTAGE, a2d_get(VOLTAGE) );
      break;
    case 25:
      if (SAMPLE_PULSEOX) {
        accum = 0;
        a2d_sample(PULSEOX_AC);
      }
      break;
    case 26:
    case 27:
    case 28:
    case 29:
    case 30:
    case 31:
    case 32:
      if (SAMPLE_PULSEOX) {
        accum += a2d_get(PULSEOX_AC);
        a2d_sample( PULSEOX_AC );
      }
      break;
    case 33:
      if (SAMPLE_PULSEOX) {
        accum += a2d_get(PULSEOX_AC);
        SPO2_IR();
        d2a1_set( sp02_ir_dc );
        data_add( B_PULSEOX_OFF_AC, accum );
        if ((accum >> 3)  < 0x200 && sp02_off_dc < 3003) {
            sp02_off_dc++;
            data_add( B_PULSEOX_OFF_OUTPUTOFFSET, sp02_off_dc );
        }
        if ((accum >> 3)> 0xe00 && sp02_off_dc > 0) {
            sp02_off_dc--;
            data_add( B_PULSEOX_OFF_OUTPUTOFFSET, sp02_off_dc );
        }
      }
      break;
    case 34:
      if (SAMPLE_EMG) a2d_sample( EMG );
      break;
    case 35:
      if (SAMPLE_EMG) data_add( B_EMG, a2d_get( EMG ) );
      break;
    case 41:
      if (SAMPLE_PULSEOX) {
        accum = 0;
        a2d_sample(PULSEOX_AC);
      }
      break;
    case 42:
    case 43:
    case 44:
    case 45:
    case 46:
    case 47:
    case 48:
      if (SAMPLE_PULSEOX) {
        accum += a2d_get(PULSEOX_AC);
        a2d_sample( PULSEOX_AC );
      }
      break;
    case 49:
      if (SAMPLE_PULSEOX) {
        SPO2_OFF();
        d2a1_set( 0 );
        accum += a2d_get(PULSEOX_AC);
        data_add( B_PULSEOX_IR_AC, accum );
        if ((accum >> 3) < 0x200 && sp02_ir_dc < 3003) {
            sp02_ir_dc++;
            data_add( B_PULSEOX_IR_OUTPUTOFFSET, sp02_ir_dc );
        }
        if ((accum >> 3)> 0xe00 && sp02_ir_dc > 0) {
            sp02_ir_dc--;
            data_add( B_PULSEOX_IR_OUTPUTOFFSET, sp02_ir_dc );
        }
      }
      break;
    case 50:
      if (SAMPLE_EMG) a2d_sample( EMG );
      break;
    case 51:
      if (SAMPLE_EMG) data_add( B_EMG, a2d_get( EMG ) );
      if (SAMPLE_ECG) a2d_sample( ECG );
      break;
    case 52:
      if (SAMPLE_ECG) data_add( B_ECG, a2d_get( ECG ) );
      break;
    case 54:
      if (SAMPLE_EMG) data_add( B_EMG, a2d_get( EMG ) );
      break;
    case 55:
    // TIMESTAMP and SYNC BLOCK
      if (rtc != rtc_old) {
        rtc_old = rtc;
        data_sync();  //always send sync block of 0x55 0x55 0x55
        if (TX_CLOCK) data_timestamp();
      }
      break;
    case 56:
      if(SAMPLE_ACCEL && (serialState != SPI_BUSY)) accel_get();
      break;
    case 57:
      if (main_status & _BIOI_RUNNING) break;
      // Ensure disk was not writing
      if(SAMPLE_BIOI) {
        ENABLE_INTERRUPTS();
        main_status |= _BIOI_RUNNING;
        ad5933_get();
        DISABLE_INTERRUPTS();
        main_status &= ~_BIOI_RUNNING;
      }
      break;
    default:
      break;
  }
}



// TACCR0 interrupt vector for TACCR0 CCIFG
// this is the highest priority timerA interrupt and occurs when
// TAR=TACCR0
// for all other timerA interrupts, there is only one vector
#pragma vector=TIMERA0_VECTOR
__interrupt void timer_a0_isr (void) {
  __low_power_mode_off_on_exit();//exit low power state
  timer_eventHandler();
}

