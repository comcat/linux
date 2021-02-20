#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

#include "sx127x.h"

void print_hex(uint8_t *buff, size_t len)
{
	int i;
	uint8_t b;

	for (i = 0; i < len; i++) {
		printf("%02X ", buff[i]);
	}
}

int setup_radio(int fd, enum sx127x_opmode opmode)
{

	if (ioctl(fd, SX127X_IOCTL_CMD_SET_PAOUTPUT, SX127X_PA_PABOOST) != 0) {
		printf("failed to set pa output\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SET_MODULATION, SX127X_MODULATION_LORA)
	    != 0) {
		printf("failed to set modulation\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SET_FREQ, 472500000) != 0) {
		printf("failed to set carrier frequency\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SET_SF, SF_10) != 0) {
		printf("failed to set spreading factor\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SET_BW, BW_500) != 0) {
		printf("failed to set bw\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SET_CR, CR_6) != 0) {
		printf("failed to set cr\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SET_CRC, CRC_ON) != 0) {
		printf("failed to set crc\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SET_OPMODE, opmode) != 0) {
		printf("failed to set opmode\n");
		return 1;
	}
}


int main(int argc, char **argv)
{
	long count = -1;

	if (argc != 1 && argc != 2) {

		printf("usage: %s\n", argv[0]);
		return 1;

	} else if (argc == 2) {

		count = strtol(argv[1], NULL, 10);
	}

	int fd = open("/dev/sx127x0", O_RDWR);

	if (fd < 0)
		printf("failed to open device\n");

	if (setup_radio(fd, SX127X_OPMODE_RXCONTINUOS))
		return 1;

	void *buff = malloc(256);

	while (count != 0) {

		read(fd, buff, sizeof(size_t));
		read(fd, buff + sizeof(size_t), *((size_t *)buff));

		struct sx127x_pkt *pkt = buff;
		void *payload = buff + pkt->hdrlen;

		print_hex(payload, pkt->payloadlen);
		printf("/%d/%d\n", pkt->payloadlen, (int)pkt->rssi);

		write(fd, payload, pkt->payloadlen);

		if (count > 0)
			count--;
	}

	return 0;
}
