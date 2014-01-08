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
          


// header: 1000 0000 0001 = 0x801
// footer: 0111 1111 1110 = 0x7FE

#include "common.h"

#define FX_CHANGESENSE (0<<14)
#define FX_SIMULATE    (1<<14)
#define FX_PROGRAM     (2<<14)
#define FX_READ        (3<<14)
#define PARAM_SECOND   (0<<12)
#define PARAM_FIRST    (1<<12)
#define PARAM_OFFSET   (2<<12)
#define PARAM_OTHER    (3<<12)
#define DUMMY          (2<<10)

#define AD8557_HIGH()  P5OUT |= _BV(7)
#define AD8557_LOW()   P5OUT &= ~_BV(7)
#define AD8557_ONE()   TBR=0;AD8557_HIGH();while(TBR<150);AD8557_LOW()
#define AD8557_ZERO()  TBR=0;AD8557_HIGH();while(TBR<2);AD8557_LOW()
#define AD8557_SPACE() TBR=0;AD8557_LOW();while(TBR<40)

void ad8557_msdelay() {
  TBR=0;
  while (TBR<2000);
}

void ad8557_tx(uint16_t data, uint8_t bits) {
  uint8_t index = 0;
  while (index < bits) {
    if (data & _BV(15-index)) {
      TBR=0;
      P5OUT |= _BV(7);
      while(TBR<150);
      P5OUT &= ~_BV(7);
    } else {
     TBR=0;
     P5OUT |= _BV(7);
     while(TBR<2);
     P5OUT &= ~_BV(7);
    }
    TBR=0;
    P5OUT &= ~_BV(7);
    while(TBR<40);
    index++;
  }
}


void ad8557_set(uint16_t x) {
  ad8557_tx(0x8010,12);
  ad8557_tx(x,14);
  ad8557_tx(0x7FE0,12);
  ad8557_msdelay();
}



void ad8557_init(uint8_t first, uint8_t second) {
  //setup port pins
  P5OUT &= ~_BV(7);
  P5DIR |= _BV(7); //DIN

  //setup timerB
  TBCTL = TBSSEL_2 | ID_3 | MC_2;
  TBR=0; while (TBR<0xFFF); // wait short bit
  ad8557_msdelay();

  //transmit configuration
  ad8557_set(FX_SIMULATE | PARAM_FIRST  | DUMMY | (first<<2));
  ad8557_set(FX_SIMULATE | PARAM_SECOND | DUMMY | (second<<2));
  ad8557_set(FX_SIMULATE | PARAM_OFFSET | DUMMY | (127<<2)); //midrail output offset

  //stop timer
  TBCTL = MC_0;
}


