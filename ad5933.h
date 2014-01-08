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
          
#ifndef __AD5933_H_
#define __AD5933_H_

#include "common.h"
#include "i2c.h"
#include "ad8557.h"

#define AD5933_CONTROL1   0x80
#define AD5933_CONTROL2   0x81
#define AD5933_STARTFREQ1 0x82
#define AD5933_STARTFREQ2 0x83
#define AD5933_STARTFREQ3 0x84
#define AD5933_FREQINC1   0x85
#define AD5933_FREQINC2   0x86
#define AD5933_FREQINC3   0x87
#define AD5933_NUMINC1    0x88
#define AD5933_NUMINC2    0x89
#define AD5933_NUMSET1    0x8A
#define AD5933_NUMSET2    0x8B
#define AD5933_REAL1      0x94
#define AD5933_STATUS     0x8F
//control register values
#define AD5933_POWERDOWN  0xA0
#define AD5933_STANDBY    0xB0
#define AD5933_INIT       0x10
#define AD5933_START      0X20
#define AD5933_REPEAT     0X40
#define AD5933_NEXT       0x30
#define AD5933_RANGE1     0x00
#define AD5933_RANGE2     0X06
#define AD5933_GAIN1      0x01
#define AD5933_GAIN5      0x00
//status register
#define AD5933_VALIDDATA  0X02

/* this init data was sniffed from the eval board */
/*
  ad5933_setWait(0x80, 0xA0);
  ad5933_setWait(0x80, 0xB0);
  ad5933_setWait(0x84,0x73);
  ad5933_setWait(0x83,0x6a);
  ad5933_setWait(0x82,0x18);
  ad5933_setWait(0x89,0x64);
  ad5933_setWait(0x88,0);
  ad5933_setWait(0x87,0x80);
  ad5933_setWait(0x86,0x02);
  ad5933_setWait(0x85,0x00);
  ad5933_setWait(0x8B,0x0F);
  ad5933_setWait(0x8A,0x00);
  ad5933_setWait(0x81,0x00);
  ad5933_setWait(0x80,0x16);
  ad5933_setWait(0x80,0x26);*/

extern volatile uint8_t i2c_txBuff[I2C_TXBUFFSIZE];
extern volatile uint8_t i2c_txBuffIndex;
extern volatile uint8_t i2c_txBuffSize;


void ad5933_setAddr(uint8_t addr);
uint8_t ad5933_read(uint8_t addr);
uint8_t ad5933_setWait(uint8_t addr, uint8_t val);
void ad5933_init();
void ad5933_waitForValidData(void);
void ad5933_get(void);


#endif
