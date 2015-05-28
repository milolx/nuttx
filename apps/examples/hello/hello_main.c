/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_FS_YAFFS2
#include <nuttx/fs/yaffsfs.h>
#endif
#include <stdio.h>
#include <fcntl.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
#ifdef CONFIG_FS_YAFFS2
  {
    char mountpt[] = "/nand";
    int h;
    int wrote;
    static char buffer[1000];
    char fn[100];

#if 0
    sprintf(fn, "%s/text", mountpt);
    yaffs_mount(mountpt);
#endif

#if 0
    h = yaffs_open(fn, O_CREAT | O_TRUNC | O_RDWR, 0666);
    printf("h=%d\n", h);
    wrote = yaffs_write(h, buffer, sizeof(buffer));
    printf("wrote=%d\n", wrote);
    //yaffs_unlink(fn);
    yaffs_close(h);
#endif
#if 1
    {
      int i;
      int fsize;
      for(i = 1; i < 2000; i++) {
        sprintf(fn,"%s/f%d",mountpt, i);
        fsize = (i%10) * 10000 + 20000;
        h = yaffs_open(fn, O_CREAT | O_TRUNC | O_RDWR, 0666);
        while (fsize > 0) {
          wrote = yaffs_write(h, buffer, sizeof(buffer));
          if (wrote != sizeof(buffer)) {
            printf("Writing file %s, only wrote %d bytes\n", fn, wrote);
            break;
          }
          fsize -= wrote;
        }
        yaffs_unlink(fn);
        yaffs_close(h);
      }
#endif
    }

#if 1
    yaffs_unmount(mountpt);
#endif
  }
#endif /* CONFIG_FS_YAFFS2 */

#ifdef CONFIG_ARCH_BOARD_AMBE2K
  {
    extern void hexdump(const void *buf_, size_t size, uint16_t ofs);
    uint8_t frm[24*2];

    extern void ambe2k_send_frame(uint8_t *frm);
    extern void ambe2k_recv_frame(uint8_t *frm);
    while (1) {
      ambe2k_recv_frame(frm);
      hexdump(frm, 2*24, 0);
      ambe2k_send_frame(frm);
    }
  }
#endif
#if 1
  {
    char chw=0;
    char chr;
    int fd;

    fd = open("/dev/ttyS1", O_RDWR);
    while (1) {
      write(fd, &chw, 1);
      read(fd, &chr, 1);
      if (chw != chr)
        printf("exp: %02x, get: %02x\n", chw, chr);
      else
        printf("ok\n");
      ++chw;
    }
  }
#endif

  printf("Hello, World!!\n");

  return 0;
}
