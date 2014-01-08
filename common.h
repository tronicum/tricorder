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
          


#ifndef __COMMON_H_
#define __COMMON_H_


#include <stdio.h>
#include <inttypes.h>
#include <msp430x26x.h>

/********** COMPILE TIME OPTIONS ***********/
#define BLUETOOTH_FLOWCONTROL
#define UART_BLOCKING //wait for TX buffer to be free, then write to it directly
//#define DEBUG
//#define IAR_TERMINAL_DEBUG

// subsystem debugging support
#ifdef DEBUG
#define DEBUG_DOSFS_SUPPORT
#define DEBUG_DOSFS
#define DEBUG_MMC
#define DEBUG_FAT
#endif

/********************************************/

/* global status variables */
#ifndef __MAIN_C_
extern volatile uint16_t main_status;
extern volatile enum serialState_t serialState;
#endif

extern volatile uint32_t rtc;
extern volatile istate_t istate;

#define _SAMPLE_BIOI    (1<<0)
#define _SAMPLE_ECG     (1<<1)
#define _SAMPLE_TEMP    (1<<2)
#define _SAMPLE_EMG     (1<<3)
#define _SAMPLE_ACCEL   (1<<4)
#define _SAMPLE_PULSEOX (1<<5)
#define _SAMPLE_VOLTAGE (1<<6)
#define _TX_CLOCK       (1<<7)

#define _FILESYSTEM_OK (1<<8) //MMC was able to initalize
#define _BT_TRANSMIT   (1<<9) //Steam data over BT (same as goes to flash)
#define _DISK_WRITE    (1<<10) //Write data to flash
#define _DISK_WRITE_OK (1<<11) //Able to open file for writing on flash
#define _DISK_TX_OK    (1<<12) //it's ok to initiate a disk write

#define _BIOI_RUNNING  (1<<13) //bioImp code is running
#define _DISKTX_RUNNING (1<<14)

#define SAMPLE_BIOI    (main_status & _SAMPLE_BIOI)
#define SAMPLE_EMG     (main_status & _SAMPLE_EMG)
#define SAMPLE_ECG     (main_status & _SAMPLE_ECG)
#define SAMPLE_TEMP    (main_status & _SAMPLE_TEMP)
#define SAMPLE_ACCEL   (main_status & _SAMPLE_ACCEL)
#define SAMPLE_PULSEOX (main_status & _SAMPLE_PULSEOX)
#define TX_CLOCK       (main_status & _TX_CLOCK)
#define SAMPLE_VOLTAGE (main_status & _SAMPLE_VOLTAGE)


enum osc_mode_t {
  INTERNAL,
  HIGHSPEED,
  LOWSPEED,
  SLEEP
};

enum serialState_t {
  SPI_READY,
  SPI_BUSY,
  I2C_READY,
  I2C_BUSY,
  NONE
};

typedef void(*fxn_t)(void);

#define sbi(port,pin) port |= pin
#define cbi(port,pin) port &= ~pin
#define _BV(x) (1<<x)

#define LED_OFF()  P1OUT |= _BV(0)
#define LED_ON() P1OUT &= ~(_BV(0))
#define LED_TOGGLE() P1OUT ^= _BV(0);

#define TEMP_POWER_ON()  P2OUT |= _BV(3)
#define TEMP_POWER_OFF() P2OUT &= ~_BV(3)
#define DEBUG1_ON()      P1OUT |= _BV(7)
#define DEBUG1_OFF()     P1OUT &= ~_BV(7)
#define ENABLE_INTERRUPTS()   __bis_SR_register(GIE)
#define DISABLE_INTERRUPTS()  __bic_SR_register(GIE)

// XXX figure this out
#define SAVE_AND_DISABLE_INTERRUPTS()  istate=__get_interrupt_state(); __disable_interrupt()
#define RESTORE_INTERRUPTS()           __set_interrupt_state(istate)

#endif
