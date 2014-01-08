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
#ifndef __RINGBUFFER_H_
#define __RINGBUFFER_H_

#include "common.h"

#define NO_DATA (1<<9)

typedef void(*fnx_t)(void*);
struct rb_t {
        uint16_t size; //how big
        uint16_t free; //free space
        uint16_t head; //where data is pulled off of
        uint16_t next; //where data is added to
        uint16_t overflow; //will keep track of how many times you try to write to the buffer that's full
        uint8_t *ptr; //ptr to the data
};
typedef volatile struct rb_t rb;

void rb_clear(rb *x) ;
uint8_t rb_init(rb  *x, uint16_t size, uint8_t *ptr) ;
__monitor void rb_push(rb *x, uint8_t byte) ;
void rb_dump(rb *x) ;
__monitor uint16_t rb_pop(rb *x) ;
__monitor void rb_markRead(rb *x, uint16_t len) ;
void rb_pushStr(rb *x, uint8_t *str);


static inline uint8_t  *rb_getPtr(rb *x) {
    return (uint8_t*) &(x->ptr[x->head]);
}

static inline uint16_t  rb_len(rb *x) {
	return (x->size - x->free);
}

static inline uint16_t rb_free(rb *x) {
	return (x->free);
}

#endif
