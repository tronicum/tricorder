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
#include "accel.h"
#include "uart.h"
#include "data.h"



#define SPI_RX_COMPLETE (IFG2 & URXIFG1)
#define SPI_TX_READY    (IFG2 & UTXIFG1)
#define SPI_TX_DONE      while(UCB1STAT & UCBUSY)
#define SPI_WAIT()       while((UC1IFG & UCB1TXIFG) == 0)
#define ACCEL_CS_LOW()   P3OUT &= ~(1<<6);             
#define ACCEL_CS_HIGH()  SPI_TX_DONE; P3OUT |= (1<<6) 

uint8_t spi(uint8_t data) {
  SPI_WAIT();
  UCB1TXBUF = data;
  SPI_WAIT();
  data = UCB1RXBUF;
  return data;
}

void accel_write(uint8_t address, uint8_t data) {
  ACCEL_CS_LOW();
  spi(address);
  spi(data);
  ACCEL_CS_HIGH();
}

uint8_t accel_read(uint8_t address) {
  volatile uint8_t i;
  ACCEL_CS_LOW();
  spi(address | _BV(7));
  SPI_WAIT();
  UCB1TXBUF = 0x00;
  SPI_WAIT();
  ACCEL_CS_HIGH();
  return UCB1RXBUF;
}



void spi_init(void) {
  P5SEL |= _BV(1) | _BV(2) | _BV(3);
  
  UCB1CTL1 = UCSWRST;		// 8-bit SPI Master **SWRST**
  UCB1CTL0 = UCMST | UCSYNC | UCCKPL | UCMODE_0 | UCMSB;	
  UCB1BR0 = 0x02;
  UCB1BR1 = 0x00;
  UCB1CTL1 |= UCSSEL_2; //SMCLK
  UCB1CTL1 &= ~UCSWRST;	// clear SWRST
  while (! (UC1IFG & UCB1TXIFG));
}

void accel_init(uint8_t range, uint8_t bandwidth) {
  uint8_t status;

  //configure ports
  P3DIR &= ~(1<<7); //Interrupt Pin
  P3DIR |= 1<<6;    //CS Pin

  //make sure the accelerometer is not asleep
  accel_wakeup();

  //configure Accelerometer
  status = accel_read( 0x14 );
  accel_write( 0x14, status | (0x1F & (range | bandwidth)));
}

void accel_sleep(void) {
    uint8_t data;
    data = accel_read( 0x0A );
    accel_write( 0x0A, (data | _BV(0)) );
}

void accel_wakeup(void) {
    accel_write( 0x0A, 0x00);
}

void accel_get(void) {
    static uint16_t x,y,z;

    //if the disk is using the SPI interface then just
    //use the previous values so we don't have unsynchronized
    //data
    if (main_status & _DISKTX_RUNNING) {
        data_accelAdd(x,y,z);
        return;
    }

    x  = accel_read( 0x02 );
    x |= (((uint16_t)accel_read( 0x03 )) << 8);
    y  = accel_read( 0x04 );
    y |= (((uint16_t)accel_read( 0x05 )) << 8);
    z  = accel_read( 0x06 );
    z |= (((uint16_t)accel_read( 0x07 )) << 8);
    x = (x >> 6);
    y = (y >> 6);
    z = (z >> 6);

    // test if negatie 10bit, conver to int16_t
 /*   if (x&(1<<9)) x |= 0xFC00;
    if (y&(1<<9)) y |= 0xFC00;
    if (z&(1<<9)) z |= 0xFC00; 
    */

    data_accelAdd(x,y,z);
}
