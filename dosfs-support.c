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
#include "dosfs.h"
#include "mmc.h"

/*
The functions you must supply in your embedded app are:

DFS_ReadSector(unit,buffer,sector,count)
DFS_WriteSector(unit,buffer,sector,count)

These two functions read and write, respectively, "count" sectors of
size SECTOR_SIZE (512 bytes; see below) from/to physical sector
#"sector" of device "unit", to/from the scratch buffer "buffer". They
should return 0 for success or nonzero for failure. In the current
implementation of DOSFS, count will always be 1.

The "unit" argument is designed to permit implementation of multiple
storage devices, for example multiple media slots on a single device,
or to differentiate between master and slave devices on an ATAPI bus.
		
This code is designed for 512-byte sectors. Although the sector size
is a #define, you should not tinker with it because the vast majority
of FAT filesystems use 512-byte sectors, and the DOSFS code doesn't
support runtime determination of sector size. This will not affect the
vast majority of users.
*/

//note: convienently, the MMC code is also based on 512 byte sectors :) :)

uint32_t DFS_ReadSector(uint8_t unit, uint8_t *buffer, uint32_t sector, uint32_t count) {
#ifdef DEBUG_DOSFS_SUPPORT
  printf("DFS_ReadSector( %ld )\r\n", sector);
#endif
  if (mmcReadSector(sector, buffer) == MMC_SUCCESS) return 0; //succeed
  main_status &= ~(_FILESYSTEM_OK | _DISK_WRITE_OK);
  return 1; //fail
}


uint32_t DFS_WriteSector(uint8_t unit, uint8_t *buffer, uint32_t sector, uint32_t count) {
#ifdef DEBUG_DOSFS_SUPPORT
  printf("DFS_WriteSector( %ld )\r\n", sector);
#endif
  if (mmcWriteSector(sector, buffer) == MMC_SUCCESS) return 0; //succeed
  main_status &= ~(_FILESYSTEM_OK | _DISK_WRITE_OK);
  return 1; //fail
}
