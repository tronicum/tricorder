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

#ifndef __UART_H_
#define __UART_H_

#include "common.h"
#include "ringbuffer.h"

#define error(x) printf("ERROR: %s\r\n",x);
#define UART_RX_READY _BV(0)

#define UART_IN_BUFFSIZE 25
#define UART_OUT_BUFFSIZE 128

#define BT_RTS() (P1IN & _BV(7))
#define BT_CTS_HIGH() P1OUT |=  _BV(6)
#define BT_CTS_LOW()  P1OUT &= ~_BV(6)

#define uart_rx_off() BT_CTS_HIGH()
#define uart_rx_on()  BT_CTS_LOW()

#define UART_TX(x)    while (!(UCA0TXIFG&IFG2));UCA0TXBUF=x

extern rb uart_outBuff;
extern volatile uint8_t uart_status;
void uart_handle(void);
void uart_init(void);
void uart_hasStringBlocking(uint8_t *str);
void uart_clearRX(void);
void uart_putchar(uint8_t);

static inline void uart_putc(uint8_t c) {
#ifdef UART_BLOCKING
  UART_TX(c);
  return;
#else
  rb_push( &uart_outBuff, c );
  IE2 |= UCA0TXIE;
#endif
}

#endif //__UART_H_
