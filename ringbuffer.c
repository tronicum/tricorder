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
#include <stdlib.h>
#include <stdio.h>
#include "ringbuffer.h"

void rb_clear(rb *x) {
  x->head = 0;
  x->next = 0;
  x->free = x->size;
}


//will use preallocated memory if supplied
uint8_t rb_init(rb  *x, uint16_t size, uint8_t *ptr) {
  x->ptr = (ptr) ? ptr : (uint8_t*) malloc(size);
  if (!x->ptr) {
#ifdef DEBUG
    printf("Unable to malloc(%d)\r\n", size);
#endif
    return 0;
  }
  x->head = 0;
  x->next = 0;
  x->overflow = 0;
  x->size = size;
  x->free = size;

  return 1;
}

void rb_pushStr(rb *x, uint8_t *str) {
  while (*str) {
    rb_push(x, *str);
    str++;
  }
}

__monitor void rb_push(rb *x, uint8_t byte) {
  if (x->free) {
    x->free--;
    // add byte
    x->ptr[x->next] = byte;
    // increment next ptr
    if (++x->next == x->size)
      x->next = 0;
  } else {
    x->overflow++;
  }
}



__monitor uint16_t rb_pop(rb *x) {
  uint8_t ret;
  if (rb_len(x) == 0) return NO_DATA;

  ret = x->ptr[x->head++];
  // increment head
  if (x->head == x->size)  x->head = 0;
  x->free++;
  return ret;
}



/* This function will mark a section of the ringbuffer as 'read'
** this function is optimized such that you should /NEVER/ mark
** it as being read beyond the buffer size.
*/
__monitor void rb_markRead(rb *x, uint16_t len) {
  if ((x->size - x->free) < len) {
    return;
  }

  x->head += len;
  if (x->head == x->size) x->head = 0;
  x->free += len;

  return;
}


