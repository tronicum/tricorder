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
#include "i2c.h"
#include "ad8557.h"
#include "ad5933.h"
#include "data.h"

extern volatile uint8_t i2c_txBuff[];
extern volatile uint8_t i2c_txBuffIndex;
extern volatile uint8_t i2c_txBuffSize;
extern volatile uint8_t i2c_x;
extern volatile uint8_t i2c_stopIssued;
volatile uint8_t ad5933_mode = I2C_READ;
struct ad5933_data_t ad5933_data;

#define I2C_WAIT()        while(!(IFG2 & UCB0TXIFG))
#define I2C_WAITRX()      while(!(IFG2 & UCB0RXIFG))
#define I2C_CLEAR()       IFG2 &= ~(UCB0TXIFG|UCB0RXIFG)
#define I2C_WAITCLEAR()   I2C_WAIT(); I2C_CLEAR()

#define I2C_STARTWRITE()  UCB0CTL1 |= UCTXSTT | UCTR
#define I2C_STOP()        UCB0CTL1 |= UCTXSTP
#define I2C_WAITSTOP()    while (UCB0CTL1 & UCTXSTP)
#define I2C_START()       UCB0CTL1 |= UCTXSTT
#define I2C_WAITSTART()   while (UCB0CTL1 & UCTXSTT)

/* RANGE1 - 2v p2p
** RANGE2 - 1v p2p
** GAIN1  - 1x
** GAIN5  - 5x
*/

#define AD5933_RANGE AD5933_RANGE1
#define AD5933_GAIN  AD5933_GAIN1

uint8_t ad5933_read(uint8_t addr) {
  uint8_t val;
  //go to address
  ad5933_setAddr(addr);
  // clear out old stuff
  I2C_CLEAR();
  UCB0CTL1 &= ~UCTR; //set read mode
  // send addr, wait for ack
  I2C_START();
  I2C_WAITSTART();
  I2C_STOP();
  // read byte
  I2C_WAITRX(); // wait for data to be read
  val = UCB0RXBUF;
  return val;
}

void ad5933_unjam(void) {
  if (UCB0STAT & UCNACKIFG) {
    I2C_STOP();
    I2C_WAITSTOP(); //XXX not sure if needed
    I2C_CLEAR(); //Clear Interrupts
  }
}

#define ad5933_setWait(a,b) ad5933_setValue(a,b)
void ad5933_setValue(uint8_t addr, uint8_t val) {
  i2c_init();

  ad5933_unjam(); //clear NAK or other funny state

  I2C_STARTWRITE();
  I2C_WAIT();
  UCB0TXBUF = addr;
  I2C_WAIT();
  UCB0TXBUF = val;
  I2C_WAIT();
  I2C_STOP();
  I2C_WAITSTOP();
 
}



void ad5933_init() {
  i2c_init();

  // 50khz, 1Hz increment, 10 sample sweep, 10 cycle warmup
  ad5933_setValue( AD5933_STARTFREQ1, 0x18);
  ad5933_setValue( AD5933_STARTFREQ2, 0x6A);
  ad5933_setValue( AD5933_STARTFREQ3, 0x74);
  ad5933_setValue( AD5933_FREQINC1, 0);
  ad5933_setValue( AD5933_FREQINC2, 0x01);
  ad5933_setValue( AD5933_FREQINC3, 0x40);
  ad5933_setValue( AD5933_NUMINC1, 0);
  ad5933_setValue( AD5933_NUMINC2, 0x0A);
  ad5933_setValue( AD5933_NUMSET1, 0x00);
  ad5933_setValue( AD5933_NUMSET2, 0x0F);
  // INIT ROUTINE
  ad5933_setValue( AD5933_CONTROL2, 0X00);
  ad5933_setValue( AD5933_CONTROL1, AD5933_STANDBY | AD5933_RANGE | AD5933_GAIN );
  ad5933_setValue( AD5933_CONTROL1, AD5933_INIT | AD5933_RANGE | AD5933_GAIN );
  // XXX add delay ?
  ad5933_setValue( AD5933_CONTROL1, AD5933_START | AD5933_RANGE | AD5933_GAIN );
}



void ad5933_setAddr(uint8_t addr) {
  ad5933_setValue(0xB0, addr);
}


void ad5933_waitForValidData(void) {
  uint8_t i;
    do {
      i = ad5933_read(0x8F);
    } while (!(i & (1<<1)));
}


void ad5933_get(void) {
  uint8_t val;

  //Set Address
  ad5933_setAddr(0x94);

  //TRANSMIT READ BLOCK COMMAND
  I2C_START();  //start
  I2C_WAITCLEAR();
  UCB0TXBUF = 0xA1;
  I2C_WAITCLEAR();
  UCB0TXBUF = 0x04;
  I2C_WAITCLEAR();

  // READ DATA
  UCB0CTL1 &= ~UCTR;  // read mode
  I2C_START();        // restart with read

  I2C_WAITSTART();    // ACK from start recieved
  I2C_WAITRX();
  val = UCB0RXBUF;
  ad5933_data.real = (((uint16_t)(val))<<8);

  I2C_WAITRX();         // Second byte of data recieved
  val = UCB0RXBUF;
  ad5933_data.real |= val;

  I2C_WAITRX();
  val = UCB0RXBUF;
  ad5933_data.imag = (((uint16_t)(val))<<8);

  I2C_STOP(); // set stop bit to signify that this is the last byte

  I2C_WAITRX();
  val = UCB0RXBUF;
  ad5933_data.imag |= val;
  I2C_WAITSTOP();

  //send restart command
  ad5933_setValue( AD5933_CONTROL1, AD5933_REPEAT | AD5933_RANGE | AD5933_GAIN );

  //add data to buffer
  data_add(B_BIOIMP_REAL, ad5933_data.real);
  data_add(B_BIOIMP_IMAG, ad5933_data.imag);
}
