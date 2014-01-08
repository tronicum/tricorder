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

/* this file is a simpler UI for accessig the dosfs functions */

#include "common.h"
#include "fat.h"
#include "mmc.h"
#include "dosfs.h"

#define FREE_CLUSTER_LIST_SIZE 128
uint32_t FREE_CLUSTER_LIST[FREE_CLUSTER_LIST_SIZE]; //store a list of the next FAT_LIST_SIZE free FATs here for optimization

VOLINFO vi; //global to store FAT info

uint8_t SCRATCH[SECTOR_SIZE];     //global scratch pad
uint8_t FAT_SCRATCH[SECTOR_SIZE];
uint8_t DIR_SCRATCH[SECTOR_SIZE];

uint32_t    _scratch_sector = 0;
uint32_t dir_scratch_sector = 0;
uint32_t fat_scratch_sector = 0;

enum fat_t fat_mmcInit(void) {
  uint8_t retry = 50;

  // INIT
  while(retry--) {
    if (initMMC()) break;
  }
  if (!retry) return ERR_MMCINIT;

  // PING
  while(retry--) {
    if (mmc_ping() == MMC_SUCCESS) break;
  }
  if (!retry) return ERR_MMCPING;

  return OK;
}

enum fat_t fat_init(void) {
	uint32_t pstart, psize;
   	uint8_t pactive, ptype, i;

        if (fat_mmcInit() != OK) return ERR_MMC;
        pstart = DFS_GetPtnStart(0, SCRATCH, 0, &pactive, &ptype, &psize);
        if (pstart == 0xffffffff){
#ifdef DEBUG
          printf("Unable to find partition information\r\n");
#endif
          return ERR_PARTITION;
        }

	if (DFS_GetVolInfo(0, SCRATCH, pstart, &vi)) {
#ifdef DEBUG
           printf("Error getting volume information\r\n");
#endif
          return ERR_VOLUME;
        }

        //clear fat list - store 128 cluster worth

        for (i=0; i<FREE_CLUSTER_LIST_SIZE; i++) {
          FREE_CLUSTER_LIST[i]=0;
        }
        FREE_CLUSTER_LIST[0] = 0xffffffff; //denote uninitalized list
	return OK;
}

enum fat_t fat_openWrite(FILEINFO *fi, uint8_t *filename) {
  if (DFS_OpenFile(&vi,filename, DFS_WRITE, SCRATCH, fi)) {
#ifdef DEBUG_FAT
    printf("error opening file\n");
#endif
    return ERR_OPEN;
  }
  return OK;
}

enum fat_t fat_openAppend(FILEINFO *fi, uint8_t *filename) {

  if (DFS_OpenFile(&vi,filename, DFS_WRITE, SCRATCH, fi)) {
#ifdef DEBUG_FAT
    printf("error opening file\n");
#endif
    return ERR_OPEN;
  }

  DFS_Seek(fi, fi->filelen, SCRATCH); //seek to end of file

  return OK;
}


enum fat_t fat_openRead(FILEINFO *fi, uint8_t *filename) {
  uint32_t openstatus = 0;
  if ((openstatus = DFS_OpenFile(&vi,filename, DFS_READ, SCRATCH, fi))) {
#ifdef DEBUG_FAT
    printf("error opening file\n");
#endif
    return ERR_OPEN;
  }
  return OK;
}

#ifdef DEBUG
enum fat_t fat_dumpFile(uint8_t *filename) {
  FILEINFO fi;
  uint8_t buff;
  if (fat_openRead(&fi, filename) != OK) {
    printf("error opening file %s\r\n", filename);
    return ERR_OPEN;
  }
  printf("[");
  while (fat_read(&fi, &buff, 1) == OK) {
    printf("%c",buff);
  }
  printf("]\r\n");
  return OK;
}
#endif


// XXX : -1 should go away.
enum fat_t fat_append(FILEINFO *fi, uint8_t *data, uint32_t len) {
  uint32_t written;

#ifdef DEBUG_FAT
  printf("seeking to %lu\r\n", fi->filelen);
#endif
  DFS_Seek(fi, fi->filelen, SCRATCH); //seek to end of file
  DFS_WriteFile(fi, SCRATCH, data, &written, len);
  if (written != len) {
#ifdef DEBUG_FAT
     printf("Only wrote %lu of %lu bytes\n", written, len);
#endif
     return ERR_WRITE;
  }
  return OK;
}

// not sure if this will truncate the file XXX
enum fat_t fat_write(FILEINFO *fi, uint8_t *data, uint32_t len) {
  uint32_t written;

  DFS_WriteFile(fi, SCRATCH, data, &written, len);
  if (written != len) {
#ifdef DEBUG_FAT
     printf("Only wrote %lu of %lu bytes\n", written, len);
#endif
     return ERR_WRITE;
  }
  return OK;
}

enum fat_t fat_read(FILEINFO *fi, uint8_t *buffer, uint32_t len) {
  uint32_t read;
  DFS_ReadFile(fi, SCRATCH, buffer, &read, len);
  if (read != len) {
#ifdef DEBUG_FAT
    printf("only read %lu of %lu bytes\r\n",read,len);
#endif
    return ERR_READ;
  }
  return OK;
}


#ifdef DEBUG
enum fat_t fat_dumpDir(uint8_t *dir) {
  DIRINFO di;
  DIRENT  de;
  uint8_t ret;

  di.scratch = SCRATCH;
  ret = DFS_OpenDir(&vi, dir, &di);

  if (ret) {
    printf("Unable to open %s (%d)\r\n", dir,ret);
    return ERR_DIR;
  }
  printf("Listing for %s :\r\n", dir);
  while (!DFS_GetNext(&vi, &di, &de)) {
    if (de.name[0]) printf("\t'%-11.11s'\r\n", de.name);
  }
  return OK;
}
#endif

/* this function will populate *list with the the next count free clusters
** the search will start at start_cluster, so you can feed the last free cluster
** to this function so it will not start at the beginning again
** This function assumes volinfo (vi) has already been populated */
#if 0
enum fat_t fat_getFreeClusters(uint32_t start_cluster, uint8_t count, uint32_t *list) {
  uint32_t i,result;
  uint8_t  found;

#ifdef DEBUG_FAT
  printf("fat_getFreeClusters(%ld, %d)\r\n", start_cluster, count);
#endif

  for (found=0; found < count; found++) {
    for (i=start_cluster; i < vi.numclusters; i++) {
      result = DFS_GetFAT(&vi, i);
      if (!result) {
        list[found] = i;
        start_cluster = i+1;
        break;
      }
    }
  }
  if (found == count)
    return OK;
  return ERR_NOFREEFAT;		// count free sectors not found)
}
#endif

/* return next free cluster.  configurable length list of free clusters that
** auto fill when empty */
uint32_t fat_getNextFreeCluster(void) {
  static uint8_t current_cluster_index = 0;
  enum fat_t ret = OK;

  //uninitalized list
  if (FREE_CLUSTER_LIST[0] == 0xffffffff) {
    ret = fat_getFreeClusters(2, FREE_CLUSTER_LIST_SIZE, FREE_CLUSTER_LIST);
    current_cluster_index = 0;
  }
  //at the end of list, need to refill
  if (current_cluster_index == FREE_CLUSTER_LIST_SIZE) {
    ret = fat_getFreeClusters(FREE_CLUSTER_LIST[FREE_CLUSTER_LIST_SIZE-1], FREE_CLUSTER_LIST_SIZE, FREE_CLUSTER_LIST);
    current_cluster_index = 0;
  }

  if (ret != OK) return 0x0ffffff7; // Can't find a FREE_CLUSTER_LIST_SIZE clusters

  return FREE_CLUSTER_LIST[current_cluster_index++];
}
