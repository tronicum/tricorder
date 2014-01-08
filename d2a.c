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
#include "d2a.h"

/* DAC12_0 is P6.6 => 
 * DAC12_1 is P6.7 => puleox offset
 * VeRef is from the VREG_PLUS from the external reference
 * and it is equal to 5/8*3.3v = 2.06V
**
** the d2a should range from 0V to 3.3V (positive rail)
** by using 3*VeREF to get it to hit that voltage.
*/

void d2a0_init(void) {
	// VeREF+ * 3 = full range (12 bit)
        // Note - VeREF+ = 2.06V; 0->2187 = 0V->3.3
	// Updates to DAC12_xDAT are immediate
	// No interrupts, straight binary, not grouped.
	DAC12_0CTL = DAC12SREF_2 | DAC12LSEL_0 | DAC12AMP_5;

	// perform calibration, wait for it to finish
	DAC12_0CTL |= DAC12CALON;
	while (DAC12_0CTL & DAC12CALON);
}

void d2a0_set(uint16_t value) {
  DAC12_0DAT = value;
}


void d2a1_init(void) {
	// VeREF+ * 3 = full range (12 bit)
        // Note - VeREF = 1.5V; 0->3003 = 0V->3.3
	// Updates to DAC12_xDAT are immediate
	// No interrupts, straight binary, not grouped.
        DAC12_1CTL = 0;
	DAC12_1CTL = DAC12SREF_1 | DAC12LSEL_0 | DAC12AMP_5;

	// perform calibration, wait for it to finish
	DAC12_1CTL |= DAC12CALON;
	while (DAC12_1CTL & DAC12CALON);
}

void d2a1_set(uint16_t value) {
	DAC12_1DAT = value;
}


void d2a0_test(void) {
	volatile uint16_t x,y;
	while(1) {
		for (x=0; x<0xFFF; x++) {
			d2a0_set(x);
                        for (y=0; y<0xFFF; y++);
		}
		
	}
}

void d2a1_test(void) {
	volatile uint16_t x,y;
	while(1) {
		for (x=0; x<3003; x++) {
			d2a1_set(x);
                        for (y=0; y<0xF; y++);
		}
		
	}
}
