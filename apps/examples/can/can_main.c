#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <pthread.h>

#include <nuttx/can.h>

#define CONFIG_EXAMPLES_CAN_READWRITE 1
#define CAN_OFLAGS O_RDWR

#define MAX_ID (1 << 11)
#define CANID	88

uint32_t tot = 0;

static void *thread_func_s(void *parameter)
{
	int fd = *((int *)parameter);
	struct can_msg_s txmsg;

	size_t msgsize;
	ssize_t nbytes;

	uint8_t seq=0;
	int i;

	while (1) {
		/* Construct the next TX message */

		txmsg.cm_hdr.ch_id    = CANID;
		txmsg.cm_hdr.ch_rtr   = false;
		txmsg.cm_hdr.ch_dlc   = 8;

		for (i = 0; i < 8; i++)
			txmsg.cm_data[i] = seq++;

		/* Send the TX message */

		msgsize = CAN_MSGLEN(8);
		nbytes = write(fd, &txmsg, msgsize);
		if (nbytes != msgsize)
		{
			printf("ERROR: write(%ld) returned %ld\n", (long)msgsize, (long)nbytes);
		}
		//tot += 8;
	}
}

static void *thread_func_r(void *parameter)
{
	int fd = *((int *)parameter);
	struct can_msg_s rxmsg;
	size_t msgsize;
	uint8_t exp = 0;
	ssize_t nbytes;
	int i;

	/* Read the RX message */

	msgsize = sizeof(struct can_msg_s);
	while (1) {
		nbytes = read(fd, &rxmsg, msgsize);
		if (nbytes < CAN_MSGLEN(0) || nbytes > msgsize)
		{
			printf("ERROR: read(%ld) returned %ld\n", (long)msgsize, (long)nbytes);
		}

		/* Verify that the received messages are the same */

		if (rxmsg.cm_hdr.ch_id != CANID)
			printf("ERROR: id mismatch\n");

		for (i=0; i<8; ++i) {
			if (rxmsg.cm_data[i] != exp) {
				printf("data err: %02x exp %02x\n", rxmsg.cm_data[i], exp);
				exp = rxmsg.cm_data[i];
			}

			++exp;
		}
		tot += 8;
	}
}

int can_main(int argc, char *argv[])
{
	int fd;
	int fd2;

	pthread_t thread;
	pthread_t thread2;

	fd = open("/dev/can0", O_RDWR);
	if (fd < 0)
	{
		printf("can_main: open %s failed: %d\n", "/dev/can0", errno);
		goto errout;
	}

	fd2 = open("/dev/can1", O_RDWR);
	if (fd2 < 0)
	{
		printf("can_main: open %s failed: %d\n", "/dev/can1", errno);
		goto errout_with_dev0;
	}

	pthread_create(&thread, NULL, thread_func_s, (pthread_addr_t)&fd);
	pthread_create(&thread2, NULL, thread_func_r, (pthread_addr_t)&fd2);

	while (1) {
		sleep(1);
		printf("-> %d kB\n", tot/1000);
	}

	close(fd2);
errout_with_dev0:
	close(fd);

errout:
	printf("Terminating!\n");
	fflush(stdout);
	return 0;
}

