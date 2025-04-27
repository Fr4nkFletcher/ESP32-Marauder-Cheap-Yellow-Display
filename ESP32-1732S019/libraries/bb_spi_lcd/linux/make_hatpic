CFLAGS=-c -Wall -O2 -D__LINUX__
LIBS = -lbb_spi_lcd -larmbianio -lm -lpthread

all: hatpic

hatpic: hatpic.o libbb_spi_lcd.a
	$(CC) hatpic.o $(LIBS) -o hatpic

hatpic.o: hatpic.c
	$(CC) $(CFLAGS) hatpic.c

clean:
	rm *.o hatpic
