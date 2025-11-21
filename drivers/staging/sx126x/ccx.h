#ifndef __CCX_H__
#define __CCX_H__

#include <linux/types.h>

char *decode_devid(char dev_id[], uint8_t *pkt)
{
	/* The len of dev_id must be 24 */
	int a = 0, b = 0;

	uint64_t devid = 0UL;

	for (a = 3; a < 11; a++, b++) {

		*(((uint8_t *)&devid) + 7 - b) = pkt[a];
	}

	sprintf(dev_id, "%llu", devid);

	return dev_id;
}

bool check_crc(uint8_t *p, int plen)
{
	int i, len = 0;
	uint16_t hh = 0, sum = 0;

	len = plen - 6;
	sum = p[len] << 8 | p[len+1];

	for (i = 0; i < len; i++) {
		hh += p[i];
	}

	if (hh == sum)
		return true;
	else
		return false;
}

uint16_t get_crc(uint8_t *pp, int len)
{
	int i;
	uint16_t hh = 0;

	for (i = 0; i < len; i++) {
		hh += pp[i];
	}
	return hh;
}

uint16_t update_crc(uint8_t *p, int len)
{
	uint8_t *x;
	uint16_t hh = 0;
	int i, pos = len - 6;

	for (i = 0; i < pos; i++) {
		hh += p[i];
	}

	x = (uint8_t *) &hh;

	p[pos] = x[1]; p[pos+1] = x[0];

	return hh;
}

#endif
