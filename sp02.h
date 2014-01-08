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
#ifndef __SP02_H_
#define __SP02_H_

enum sp02_state_t {
	RED,
	IR
};

#define SPO2_IR() P2OUT &= ~_BV(0);P3OUT |= _BV(3)
#define SPO2_RED()  P2OUT |= _BV(0);P3OUT &= ~_BV(3)
#define SPO2_OFF() P2OUT &= ~_BV(0); P3OUT &=~_BV(3)


#ifndef __SP02_C_
extern volatile enum sp02_state_t sp02_state;
extern volatile uint16_t sp02_red_dc, sp02_ir_dc, sp02_off_dc;
extern volatile int32_t  sp02_red_dc_register;
extern volatile int32_t  sp02_ir_dc_register;
#endif

int32_t mul16(register int16_t x, register int16_t y);
int16_t ir_filter(int16_t sample);
int16_t red_filter(int16_t sample);
uint16_t dc_estimator(register volatile int32_t *p, register uint16_t x);
void sp02_init(void);

#endif
