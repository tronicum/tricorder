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
#include "bt.h"
#include "uart.h"

volatile uint32_t bt_startTime=0;
volatile uint32_t bt_killTime =0;

void bt_init(void) {
  P4DIR |= _BV(0) | _BV(1); //reset and on/off
  P4OUT |= _BV(1); //ON_OFF SHOULD ALWAYS BE HIGH
  bt_on();
  // wait for "ROK" string
  uart_hasStringBlocking("ROK");
  //Set Friendly Name
  printf("AT+JSLN=09,Tricorder\r\n");
  uart_hasStringBlocking("OK");
  //Make Discoverable
  printf("AT+JDIS=3\r\n");
  uart_hasStringBlocking("OK");
  //Register Local Service
  printf("AT+JRLS=1101,07,BioData,01,000000\r\n"); 
  uart_hasStringBlocking("OK");
  //Auto accept connection requests
  printf("AT+JAAC=1\r\n");
  uart_hasStringBlocking("OK");
  
}

void bt_stopStream(void) {
  volatile uint32_t i;

  for (i=0; i<300000; i++);
  uart_putchar('^');
  for (i=0; i<110000; i++);
  uart_putchar('^');
  for (i=0; i<110000; i++);
  uart_putchar('^');
  for (i=0; i<110000; i++);  
}

void bt_reset(void) {
  volatile uint16_t i;
  P4DIR |= _BV(0); //BLUETOOTH_RESEET output
  bt_off(); //leave in reset
  for (i=0; i<0xffff; i++); //wait a bit
  bt_on(); // TURN ON
  for (i=0; i<0xffff; i++); //wait a bit
}

void bt_off(void) {
  P4DIR |= _BV(0) | _BV(1); //reset and on/off
  P4OUT |= _BV(1); //ON_OFF SHOULD ALWAYS BE HIGH
  P4OUT &= ~(_BV(0)); 
}

void bt_on(void) {
  P4DIR |= _BV(0) | _BV(1); //reset and on/off
  P4OUT |= _BV(1); //ON_OFF SHOULD ALWAYS BE HIGH
  P4OUT |= _BV(0); 
}

