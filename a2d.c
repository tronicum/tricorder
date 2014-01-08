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
#include "a2d.h"


/* Tsample = (Rs + 2kOhm) * 9.011 * 40pF + 800nS
** minimal Tsample = (Rs=0) -> 1.52088 microseconds
** additional 0.36044 nanoseconds per Ohm resistance
**
** For 6111 specific processor..
** Tsample = 9.011*(1000+Rs)*30pF + 800nS
** For Rs=10k, Tsample = 3.77uS
** From datasheet, minimum clock speed is 3.7Mhz (Nominal 5, Max 6.3)
**
*/
// set REFON =0 to disable reference when not in use,
// else everything goes into low power mode and shuts off when
// not being used

void a2d_init(void) {
  ADC12CTL0 =  REFON | ADC12ON | SHT1_4 | SHT0_4;
      // use 1.5 ref, turn on, and adc12 on, 16 ADC12CLK cycles for sample time (see above)
  ADC12CTL1 = SHP | ADC12DIV_0 | ADC12SSEL_0 | CONSEQ_1;
      // ADC12SC bit S&H source, pulse mode, /1 clock ADC12OSC div, sequence sample

  /* setup the PINS */
  P6DIR = 0x00; //INPUT
  P6SEL = 0xFF; //Peripheral Module Use (A2D)

  /* setup the various channels to be sampled
  ** and sepecify where in memory the results are to be
  ** stored */
  ADC12MCTL0 = INCH_0 | SREF_0 | EOS; //Batt Voltage
  ADC12MCTL1 = INCH_1 | SREF_0 | EOS; //EMG
  ADC12MCTL2 = INCH_2 | SREF_0 | EOS; //ECG
 // ADC12MCTL3 = INCH_3 | SREF_0 | EOS; //MISC
 // ADC12MCTL4 = INCH_4 | SREF_0 | EOS; //PULSEOX DC
  ADC12MCTL5 = INCH_5 | SREF_0 | EOS; //PULSEOX AC
 // ADC12MCTL6 = INCH_7 | SREF_0 | EOS; //Temp_IN (another misc connector)
  ADC12MCTL7 = INCH_10 | SREF_0 | EOS; // Internal IC TEMP

  //setup interrupt for pulseox_ac
 // ADC12IE |= ADC12IE5; //pulseox AC
}


#define A2D_Trigger() ADC12CTL0 |= ADC12SC

void a2d_sample(enum channel_t channel) {
  while(ADC12CTL1 & ADC12BUSY); //wait for last conversion to complete
  A2D_DISABLE();
  ADC12CTL1 &= 0x0FFF;
  ADC12CTL1 |= ((uint16_t)channel << 12);
  A2D_ENABLE();
  A2D_Trigger(); // start conversion (toggle ADC12SC)
}

uint16_t a2d_get(enum channel_t channel) {
  while (! ADC12IFG & _BV(channel) ); // wait for conversion
  switch(channel) {
    case VOLTAGE: return ADC12MEM0;
    case EMG: return ADC12MEM1;
    case ECG: return ADC12MEM2;
   // case MISC: return ADC12MEM3;
  //  case PULSEOX_DC: return ADC12MEM4;
    case PULSEOX_AC: return ADC12MEM5;
   // case TEMP: return ADC12MEM6;
    case IC_TEMP: return ADC12MEM7;
  }
  return 0;
};

void a2d_wait(void) {
    while(ADC12CTL1 & ADC12BUSY); //wait for last conversion to complete
}

uint8_t a2d_ready(enum channel_t channel) {
  switch(channel) {
    case VOLTAGE: return (ADC12IFG & _BV(0));
    case EMG: return (ADC12IFG & _BV(1));
    case ECG: return (ADC12IFG & _BV(2));
 //   case MISC: return (ADC12IFG & _BV(3));
  //  case PULSEOX_DC: return (ADC12IFG & _BV(4));
    case PULSEOX_AC: return (ADC12IFG & _BV(5));
  //  case TEMP: return (ADC12IFG & _BV(6));
    case IC_TEMP: return (ADC12IFG & _BV(7));
    default: return 0;
  }
}


