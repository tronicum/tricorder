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
          

/*note: all code written by VGbio has been removed */

#include "common.h"
#include "timer.h"
#include "fat.h"
#include "uart.h"
#include "buff.h"
#include "ringbuffer.h"
#include "accel.h"
#include "mmc.h"

/* space out write operations by WRITE_SPACING * timing loop duration (~3.9ms)
** if problem occurs, then wait RESET_SPACING itterations
** before retrying
*/

#define WRITE_SPACING 9
#define RESET_SPACING 2 //also applies to how long to wait for a busy MMC to get unbusy
#define BUFFER_SIZE 510
#define NO_SECTOR 0xffffffff
#define _100M 204800UL
#define _10M  20480UL
#define _1M   2048UL
#define _100k 204
#define _10k  20


enum buff_flushState_t {
    DATA_FLUSH=0,
    WAIT_DONE=WRITE_SPACING,
    RESET=100,
    RESET_DONE=(100+RESET_SPACING)
} buff_flushState = DATA_FLUSH;


// to access RTC
extern volatile uint32_t timer_seconds;
extern uint8_t SCRATCH[];
rb buff = { 0, 0, 0, 0, 0, 0 };
uint8_t bdata[4 * BUFFER_SIZE];
// current restart counter
uint16_t restart_count;
uint32_t targetSector = NO_SECTOR;


uint16_t buff_getLastRestart(uint32_t start) {
	uint16_t header;
	uint8_t ret;

	ret = mmcReadSector(start-1, bdata);
        if (ret != MMC_SUCCESS) return 0xFFFF;
	header = ((uint16_t)bdata[0]<<8) | (uint16_t)bdata[1];
#ifdef DEBUG
	if (!header) {
		printf("buff_getLastRestart(): bad start sector\r\n");
	}
#endif
	return header;
}


// do a search for the last block written
uint32_t buff_findStart(uint32_t startSector, uint32_t filelen) {
	uint32_t endSector;
	uint32_t i;
	uint8_t  ret;

	// we might miss last sector due to rounding but if that's the case
	// then we're out of space anyhow so who cares?
	endSector   = startSector + (filelen/512);

	//Phase 1: seek in 100M chunks
	for (i=startSector; i<endSector; i += _100M) {
		ret = mmcReadSector(i, bdata);
		if (!bdata[0] && !bdata[1]) {
			if (i == startSector) return startSector;
			startSector = i - _100M;
			break;
		}
	}
	//Phase 2: seek in 10M chunks
	for (i=startSector; i<endSector; i += _10M) {
		ret = mmcReadSector(i, bdata);
		if (!bdata[0] && !bdata[1]) {
			if (i == startSector) return startSector;
			startSector = i - _10M;
			break;
		}
	}
	//Phase 3: seek in 1M chunks
	for (i=startSector; i<endSector; i += _1M) {
		ret = mmcReadSector(i, bdata);
		if (!bdata[0] && !bdata[1]) {
			if (i == startSector) return startSector;
			startSector = i - _1M;
			break;
		}
	}
	//Phase 4: seek in 100k chunks
	for (i=startSector; i<endSector; i += _100k) {
		ret = mmcReadSector(i, bdata);
		if (!bdata[0] && !bdata[1]) {
			if (i == startSector) return startSector;
			startSector = i - _100k;
			break;
		}
	}
	//Phase 5: seek in 10k chunks
	for (i=startSector; i<endSector; i += _10k) {
		ret = mmcReadSector(i, bdata);
		if (!bdata[0] && !bdata[1]) {
			if (i == startSector) return startSector;
			startSector = i - _10k;
			break;
		}
	}
	//Phase 6: seek in 512 byte chunks
	for (i=startSector; i<endSector; i++) {
		ret = mmcReadSector(i, bdata);
;		if (!bdata[0] && !bdata[1]) {
			return i;
		}
	}
	return NO_SECTOR; //couldn't find anything
}



enum fat_t buff_init(void) {
    enum fat_t ret;
    FILEINFO F;

    rb_init(&buff, 4*BUFFER_SIZE, bdata);

    // open file for reading, populate FILEINFO
    ret = fat_openRead(&F, "data"); //XXX check ret?
    if (ret != OK) return ret;
    
    // find where we should start writing to
    targetSector = buff_findStart(F.startsector, F.filelen);
    if (targetSector == NO_SECTOR) return ERR_FULL;

    // determine which restart count instance this was
    restart_count = (F.startsector == targetSector) ? 0 : buff_getLastRestart(targetSector);
    if (restart_count == 0xffff) return ERR_OTHER;
    restart_count++;

    //XXX not sure if this is required as we're not modifying anything in the FAT or directory structure
    /*
    ret = fat_openWrite(&Fout, "data");
#ifdef DEBUG
    if (ret != OK) {
	printf("buff_init: unable to open file for writing\r\n");
    }
#endif
    */
    return ret;
}



enum fat_t buff_flush(void) {
    enum fat_t ret;
    uint8_t *pBuffer;
    uint8_t mmc_status;

    // make sure that the filesystem is OK and the file is open for writing
    if (!(main_status & (_DISK_WRITE_OK | _FILESYSTEM_OK)))
	return ERR_CONNECT;

    //Ensure that the file start sector has been set
    if (targetSector == NO_SECTOR) {
      main_status &= ~(_DISK_WRITE_OK | _FILESYSTEM_OK);
#ifdef DEBUG
       printf("buff_flush: unable to write data\r\n");
#endif
       return ERR_WRITE;
    }

    // do write operation
    switch(buff_flushState) {
      case DATA_FLUSH:
        if (rb_len(&buff) < BUFFER_SIZE) return ERR_NOTENOUGH;
        // Get the head of the buffer
        pBuffer = (buff.ptr + buff.head);
        // Write out the sector
  //LED_ON();
        main_status |= _DISKTX_RUNNING;
        mmc_status = my_mmcWriteSector(restart_count, targetSector, pBuffer);
        main_status &= ~_DISKTX_RUNNING;
   //LED_OFF();
        if (mmc_status == MMC_SUCCESS) {
            // If write status is OK... Increment the block
	    targetSector++;
            ret = OK;
            buff_flushState++;
        } else if (mmc_status == MMC_BUSY) {
            // nothing done, let's try again
            buff_flushState = RESET;
            return ERR_MMC_BUSY;
        } else {
            buff_flushState = RESET;
            main_status &= ~(_DISK_WRITE_OK | _FILESYSTEM_OK);
            return ERR_WRITE;
        }
        break;
      case WAIT_DONE:
      case RESET_DONE:
        buff_flushState = DATA_FLUSH;
        return OK;
      // increment wait states to allow enough time for mmc to finish write
      // each wait block is ~3.9ms
      default:
        buff_flushState++;
        break;
    }


    return ret;
}

void buff_clearPage(void) {
	__bic_SR_register(GIE);
	rb_markRead(&buff, BUFFER_SIZE);
	__bis_SR_register(GIE);
}



/*************************************************
* TEST FUNCTIONS
*************************************************/

#ifdef DEBUG
void buff_test3(void)
{
    FILEINFO _F;
    uint8_t _b[512];
    int i, ret;

    for (i = 0; i < 512; i++)
	_b[i] = i & 0xFF;

    // Initialize
    printf("Fat Init");
    while (fat_init() != OK)
	printf(".");		// wait for flash to initalize
    printf("OK\r\n");

    // create file
    ret = fat_openAppend(&_F, "test");
    if (ret != OK) {
	printf("Unable to open file for writing (%d) \r\n", ret);
	while (1);
    }

    ret = fat_write(&_F, _b, 512);
    if (ret != OK) {
	printf("1.Unable to write to file (%d)\r\n", ret);
	while (1);
    }

    ret = fat_write(&_F, _b, 512);
    if (ret != OK) {
	printf("2.Unable to write to file (%d)\r\n", ret);
	while (1);
    }

    while (1);

}


void buff_test2(void)
{
    FILEINFO _F;
    uint8_t _b[100];
    int i, ret;

    for (i = 0; i < 100; i++)
	_b[i] = i;

    // Initialize
    printf("Fat Init");
    while (fat_init() != OK)
	printf(".");		// wait for flash to initalize
    printf("OK\r\n");


    // print new directory listing
    fat_dumpDir("");

    // create file
    ret = fat_openAppend(&_F, "test");
    if (ret != OK) {
	printf("Unable to open file for writing (%d) \r\n", ret);
	while (1);
    }
    // write to file
    ret = fat_write(&_F, _b, 100);
    if (ret != OK) {
	printf("Unable to write to file (%d)\r\n", ret);
	while (1);
    }
    // check disk structure
    fat_dumpDir("");

    //end
    while (1);

}
#endif

