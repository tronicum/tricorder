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

#define __UART_C_
#include <yfuns.h>
#include "common.h"
#include "d2a.h"
#include "uart.h"
#include "bt.h"
#include "ringbuffer.h"
#include <string.h>

/* PIN SIGNAL (MCU)          SIGNAL (BlueTooth)
** 1.6 RTS ----------------> CTS
** 1.7 CTS <---------------- RTS
*/

#define SBI(byte,bit) byte|=(1<<bit)
#define CBI(byte,bit) byte&=~(1<<bit)

#define DISSABLE_UART_TX() CBI(IE2,1)
#define ENABLE_UART_TX() SBI(IE2,1)
#define UART_GETC()  UCA0RXBUF;

volatile uint8_t uart_status;

uint8_t uart_inBuff[UART_IN_BUFFSIZE];
//uint8_t uart_txBuff[UART_OUT_BUFFSIZE];
uint8_t uart_inPtr;
uint8_t uart_outPtr;
rb uart_outBuff;


// number of seconds before action is taken
#define BT_KILL_DELAY 1
#define BT_START_DELAY 2
// when these variables are set, action is taken in the 
// main loop.
extern volatile uint32_t bt_startTime;
extern volatile uint32_t bt_killTime;


#ifdef RX_DEBUG
rb debug;
uint8_t foo[200];
#endif

void uart_init(void) {

#ifdef RX_DEBUG
  rb_init(&debug, 200, foo);
#endif

  P3SEL |= 0x30;                            // P3.4,5 = USART0 TXD/RXD
  UCA0CTL1 = UCSWRST;
  UCA0CTL0 = /*UCMSB | */ 0;
  UCA0CTL1 |= UCSSEL_2;
  UCA0BR0  = 138;
  UCA0BR1  = 0;
  UCA0MCTL = ( 0 /*UCBRF*/ << 4 |
               7 /*UCBRS*/ << 1 |
               0 /*UCOS16*/ );
  UCA0CTL1 &= ~UCSWRST;
  IE2 |= UCA0RXIE;

  //setup hardware handshaking lines
  // PxDIR - 0=input 1=output
  P1DIR &= ~_BV(7); //set to input (handshaking line) - RTS
  P1DIR |=  _BV(6); //set to output (handshaking line) - CTS
  uart_rx_on();     //configure handshaking line to accept data

  uart_inPtr = 0;
  uart_outPtr = 0;
  uart_status = 0;

  /* SETUP RINGBUFFER AND ASSOCIATED INTERRUTPS */
  rb_init( &uart_outBuff, 256, NULL);
  P1IES |= _BV(7); //high->low transtion triggers interrupt on P1.7 (uart handshake line)
}


/* 
** x<word> -> set main_status
** r -> enable bluetooth transmit
** o -> power down bluetooth and stop transmission
** s -> stop bluetooth transmission
** z -> reset bluetooth
*/


uint8_t uart_hasString(uint8_t *str) {
  return (strstr((char const*)uart_inBuff, (char const*)str)) ? 1 : 0;
}
          
void uart_handle(void) {
  volatile uint16_t i;
  if (!(uart_status & UART_RX_READY)) return;
  //wait for +RPCI command
  /* PASSWORD AUTH */
  if (uart_hasString("+RPCI")) { 
    bt_killTime = 0;
    printf("AT+JPCR=04,1234\r\n");
    uart_clearRX();
    return;
  } 
  /* CONNECTION REQUSET */
  if (uart_hasString("+RCOI")) {
    bt_killTime=0;
#ifdef RX_DEBUG
rb_pushStr(&debug, "A:");
rb_pushStr(&debug, uart_inBuff);
#endif
    printf("AT+JACR=1\r\n"); //accept connection
    uart_clearRX();
  //  uart_hasStringBlocking("OK");
    return;
  }
  
  
  /* MORE CONNECTION STUFF */
  if (uart_hasString("+RCCRCNF")) {
    bt_killTime=0;
#ifdef RX_DEBUG
rb_pushStr(&debug, "B:");
rb_pushStr(&debug, uart_inBuff);
#endif
    printf("AT+JSCR\r\n"); //start streaming data
    uart_clearRX();
    uart_hasStringBlocking("OK");
    for (i=0; i<0xFFFF; i++) ; //delay a sec
    for (i=0; i<0xFFFF; i++) ; //delay a sec
    bt_startTime=rtc+BT_START_DELAY; //set time to start transmitting
 //   main_status |= _BT_TRANSMIT;
    LED_ON();
    uart_clearRX();
    return;
  }
  
  /* DISCONNECT */
  if (uart_hasString("+RDII")) { //connection lost
#ifdef RX_DEBUG
rb_pushStr(&debug, "C:");
rb_pushStr(&debug, uart_inBuff);
#endif
    bt_startTime = 0;
    main_status &= ~_BT_TRANSMIT;
    LED_OFF();
    uart_clearRX();
    // reboot the BT module to get it in a sane state
    // if we are still in disconnect mode for 2 seconds
    bt_killTime = rtc+BT_KILL_DELAY; 
    return;
  }
  
  /* Unrecognized Msg */
#ifdef RX_DEBUG
rb_pushStr(&debug, "D:");
rb_pushStr(&debug, uart_inBuff);
#endif
  uart_clearRX();
}


void uart_hasStringBlocking(uint8_t *str) { 
  
  while(1) {
    //wait for \n
    while (!(uart_status & UART_RX_READY));
    // scan for desired text
    if (strstr((char const*)uart_inBuff, (char const*)str)) {
      //we found it, return
      uart_clearRX();
      return;
    } 
    //not found, clear buffer and keep trying
    uart_clearRX();
  }
}

void uart_clearRX(void) {
  uint8_t i;
  
  uart_status &= ~(UART_RX_READY);
  uart_inPtr = 0;
  for (i=0;i<UART_IN_BUFFSIZE; i++) uart_inBuff[i]=0;
  uart_rx_on();
}

void uart_putchar(uint8_t c) {
  while (!(IFG2 & UCA0TXIFG));
  UCA0TXBUF = c;
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void usart0_tx (void) {
  uint16_t ret;

  if (IFG2 & UCA0TXIFG) {
#ifdef BLUETOOTH_FLOWCONTROL
    if (BT_RTS()) {
      DISSABLE_UART_TX();
      CBI(IE2, UCA0TXIE);  //dissable uart tx interrupt
      P1IFG &= ~_BV(7);    //turn off handshaking interrupt flag
      P1IE  |=  _BV(7);    //enable handshaking interrupt;
      return;
    }
#endif

    ret = rb_pop(&uart_outBuff);
    if (ret & NO_DATA) {
      DISSABLE_UART_TX();
      return;
    }

    UCA0TXBUF = (uint8_t)ret;
  }
}

#pragma vector=PORT1_VECTOR
__interrupt void usart0_tx_handshake(void) {
  P1IE  &= ~_BV(7); //disable handshaking interrupt;
  P1IFG &= ~_BV(7); //clear handshaking interrupt flag
  ENABLE_UART_TX(); 
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void usart0_rx (void) {
  uint8_t c;
  if (uart_inPtr >= UART_IN_BUFFSIZE) {
    uart_inPtr -=1 ;
  }
  c = UART_GETC();
  uart_inBuff[uart_inPtr++] = c;
  if (c == '\n') {
    uart_status |= UART_RX_READY;
    uart_rx_off();
  }
  CBI(IFG2,0); //XXX not sure if this is cleared automatically
}




#ifndef IAR_TERMINAL_DEBUG
_STD_BEGIN
size_t __write(int handle, const unsigned char * buffer, size_t size){
  uint16_t i;

  for (i=0; i<size; i++) {  
    while (!uart_outBuff.free); //wait for free space on the buffer
    rb_push( &uart_outBuff, buffer[i] );
    IE2 |= UCA0TXIE;
  }
  // if error return _LLIO_ERROR

  return size;
}
_STD_END
#endif
