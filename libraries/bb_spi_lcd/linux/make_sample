CFLAGS=-c -Wall -O2 -D__LINUX__
LIBS = -lbb_spi_lcd -lgpiod -lm -lpthread

all: sample

sample: sample.o
	$(CC) sample.o $(LIBS) -o sample

sample.o: sample.c
	$(CC) $(CFLAGS) sample.c

clean:
	rm *.o sample
