CC = gcc
CFLAGS = -std=c11 -D_DEFAULT_SOURCE -Wall -Wextra -Wpedantic -pthread
LDLIBS = -pthread -lrt

PROGRAMS = call internal safety car controller

all: $(PROGRAMS)

call: call.o common.o
internal: internal.o common.o
car: car.o common.o
controller: controller.o common.o
safety: safety.o   # safety-critical: self-contained, does not use common.c

safety.o: safety.c
	$(CC) $(CFLAGS) -c $<

%.o: %.c common.h
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(PROGRAMS) *.o

.PHONY: all clean
