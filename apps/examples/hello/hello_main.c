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
#include <nuttx/fs/yaffsfs.h>
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

  printf("Hello, World!!\n");

#if 1
  yaffs_unmount(mountpt);
#endif

  return 0;
}
