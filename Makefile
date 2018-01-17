SUBMIT = procsim.h procsimsim_driver.c Makefile
CFLAGS := -g -Wall -std=c99 -lm
CC=gcc

all: procsim

procsim: procsim.o procsim_driver.o
	$(CC) -o procsim procsim.o procsim_driver.o 

procsim.o: procsim.c
	$(CC) -c -o procsim.o $(CFLAGS) procsim.c 

procsim_driver.o: procsim_driver.c
	$(CC) -c -o procsim_driver.o $(CFLAGS) procsim_driver.c 

clean:
	rm -f procsim *.o

submit: clean
	tar zcvf bonus-submit.tar.gz $(SUBMIT)
