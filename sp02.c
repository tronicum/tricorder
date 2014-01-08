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
/* Some code was derived from the TI MSP430 Pulse Oximeter
** reference design */

#define __SP02_C_
#include "common.h"
#include "sp02.h"
#include "d2a.h"


static const int16_t coeffs[12] =
{
    688,
    1283,
    2316,
    3709,
    5439,
    7431,
    9561,
    11666,
    13563,
    15074,
    16047,
    16384
};


//Filter hard above a few Hertz,
//using a symmetric FIR.
//This has benign phase
//characteristics */
/* Filter 50/120Hz */
int16_t ir_filter(int16_t sample) {
    static int16_t buf[32];
    static int offset = 0;
    int32_t z;
    int i;

    buf[offset] = sample;
    z =  (int32_t)coeffs[11]* buf[(offset - 11) & 0x1F];
    for (i = 0;  i < 11;  i++) {
        z += (int32_t)coeffs[i]*( buf[(offset - i) & 0x1F] + buf[(offset - 22 + i) & 0x1F]);
    }
    offset = (offset + 1) & 0x1F;
    return  z >> 15;
}

int16_t red_filter(int16_t sample) {
    static int16_t buf[32];
    volatile int16_t a,b;
    static int offset = 0;
    int32_t z;
    int i;

    buf[offset] = sample;
    z =  (int32_t)coeffs[11]* buf[(offset - 11) & 0x1F];
    for (i = 0;  i < 11;  i++) {
        z += (int32_t)coeffs[i]*( buf[(offset - i) & 0x1F] + buf[(offset - 22 + i) & 0x1F]);
    }
    offset = (offset + 1) & 0x1F;
    return  z >> 15;
}

uint16_t dc_estimator(register volatile int32_t *p, register uint16_t x) {
	uint16_t z;
	/* Noise shaped DC estimator. */
	*p += ((((int32_t) x << 16) - *p) >> 11);
	z = (*p >> 16);
	if (x < z) {
		*p = (*p&0xFFFF) | ((int32_t)x<<16);
		return x;
	}
	return z;
}



/* The rest of the code is (c) Reza Naima 2008 */

volatile enum sp02_state_t sp02_state;
volatile uint16_t sp02_red_dc, sp02_ir_dc, sp02_off_dc;
volatile int32_t  sp02_red_dc_register;
volatile int32_t  sp02_ir_dc_register;



void sp02_init(void) {
  P2DIR |= _BV(0); //OXY_CONTROL2
  P3DIR |= _BV(3); //OXY_CONTROL1
  
  SPO2_OFF();
  sp02_state = RED;
  sp02_red_dc = 2000;
  sp02_ir_dc  = 2000;
  sp02_off_dc = 2000;
  sp02_red_dc_register = 0;
  sp02_ir_dc_register = 0;

  d2a1_init();
}



