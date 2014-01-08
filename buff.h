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
          


#ifndef __BUFF_H_
#define __BUFF_H_

enum buff_data_t {
  B_EMG=0,
  B_ECG=1,
  B_ECG_FILTER=2, //FIR FILTER'D DATA
  B_ACCEL_X=3,
  B_ACCEL_Y=4,
  B_ACCEL_Z=5,
  B_TEMP=6,
  B_IC_TEMP=7,
  B_PULSEOX_COMPOSITE=8,
  B_PULSEOX_RED_AC=9, //PULSEOX DATA MINUS DC COMPONENT
  B_PULSEOX_RED_OUTPUTOFFSET=10, //SUBTRACTED OFFSET OUTPUT
  B_PULSEOX_IR_AC=11, //PULSEOX DATA MINUS DC COMPONENT
  B_PULSEOX_IR_OUTPUTOFFSET=12, //SUBTRACTED OFFSET OUTPUT
  B_PULSEOX_OFF_AC=13,
  B_PULSEOX_OFF_OUTPUTOFFSET=14,
  B_DEBUG=15,
  B_ERROR=16,
  B_MESSAGE=17,
  B_SYNC=18,
  B_BIOIMP_REAL=19,
  B_BIOIMP_IMAG=20,
  B_VOLTAGE=21,
  B_TIMESTAMP_HI=22,
  B_TIMESTAMP_LO=23,
  B_STATUS=24
};

enum fat_t buff_init(void);
void buff_add(enum buff_data_t dataType, uint16_t value);
enum fat_t buff_flush(void);
void buff_test(void);
void buff_clearPage(void);
//void buff_timeStamp(void);
#endif
