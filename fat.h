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
#ifndef __FAT_H_
#define __FAT_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "dosfs.h"


enum fat_t {
  ERR_PARTITION, // unable to find 1st primary partition
  ERR_VOLUME,    // unable to determine information on FAT fs
  ERR_OPEN,      // unable to open a file
  ERR_WRITE,
  ERR_READ,
  ERR_MMC,
  ERR_MMCPING,
  ERR_MMCINIT,
  ERR_MALLOC, //unable to malloc space for buffer
  ERR_NOTENOUGH, //not enough data to do a write
  ERR_DIR, //direcotry related error
  ERR_NOFREEFAT, //disk is full (no free clusters)
  ERR_CONNECT, //write attempted with a non-OK disk statu
  ERR_MMC_RESPONSE_ERROR, // random error
  ERR_MMC_BUSY, // MMC card is busy so we're postponing operation
  ERR_FULL, // DISK IS FULL
  ERR_OTHER, //unkown error
  OK
};

enum fat_t fat_init(void);
enum fat_t fat_openWrite(FILEINFO *fi, uint8_t *filename);
enum fat_t fat_openRead(FILEINFO *fi, uint8_t *filename);
enum fat_t fat_openAppend(FILEINFO *fi, uint8_t *filename);
enum fat_t fat_append(FILEINFO *fi, uint8_t *data, uint32_t len);
enum fat_t fat_write(FILEINFO *fi, uint8_t *data, uint32_t len);
enum fat_t fat_read(FILEINFO *fi, uint8_t *buffer, uint32_t len);
enum fat_t fat_getFreeClusters(uint32_t start_cluster, uint8_t count, uint32_t *list);
uint32_t fat_getNextFreeCluster(void);
enum fat_t fat_dumpFile(uint8_t *filename);
enum fat_t fat_dumpDir(uint8_t *dir);


#endif
