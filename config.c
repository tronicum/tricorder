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
          


#define __CONFIG_C_

#include "common.h"
#include "fat.h"
#include "timer.h"
#include "config.h"

struct config_t config;

/* this function will compute a simple checksum based on the sum
** of all bytes in the config_t struct except the last one */
uint8_t config_checksum(struct config_t *c) {
  uint8_t i,checksum=0;
  uint8_t *data;

  data = (uint8_t*) c;
  for (i=0; i<(sizeof(struct config_t)-1); i++) {
    checksum += data[i];
  }
  return checksum;
}

void config_defaults(void) {

  config.checksum = config_checksum(&config);
}


/* write config to flash */
void config_write(void) {
  FILEINFO f;

  // update checksum
  config.checksum = config_checksum(&config);
  // open file for writing
  if (fat_openWrite(&f, "config") != OK) {
#ifdef DEBUG
    printf("unable to update config (openWrite)\r\n");
#endif
    return;
  }
  // write data
  if (fat_write(&f,(uint8_t*)(&config), sizeof(struct config_t)) != OK) {
#ifdef DEBUG
    printf("unable to write new config\r\n");
#endif
  }
}

/* read config from flash */
void config_read(void) {
  FILEINFO f;
  struct config_t newConfig;
  enum fat_t ret;
  uint8_t *n,*o,i;

  // open config else restore defaults and write
  ret = fat_openRead(&f, "config");
  if (ret != OK) {
#ifdef DEBUG
   printf("file config does not exist - using defaults\r\n");
#endif
   config_defaults();
   config_write();
   return;
  }

  // read config, if checksum missmatch or other error, restore defaults
  ret = fat_read(&f, (uint8_t*)&newConfig, sizeof(struct config_t));
  if (ret != OK || config_checksum(&newConfig) != newConfig.checksum) {
#ifdef DEBUG
   printf("config file error - using defaults\r\n");
#endif
   config_defaults();
   config_write();
   return;
  }

  // update config with new values
  n = (uint8_t*)(&newConfig);
  o = (uint8_t*)(&config);
  for (i=0; i<(sizeof(struct config_t)); i++) {
    o[i] = n[i];
  }

}
