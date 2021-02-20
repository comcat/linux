all: rx

CC=arm-none-linux-gnueabi-gcc
CFLAGS=-D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64  -Os -g2

rx: rx.c
	$(CC) $(LDFLAGS) $(CFLAGS) -I./ $^ -o $@

clean:
	rm -f rx
