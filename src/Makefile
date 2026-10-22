CC = gcc
CFLAGS = -std=c11 -D_DEFAULT_SOURCE -Wall -Wextra -Wpedantic -pthread
LDLIBS = -pthread -lrt

PROGRAMS = call internal safety car controller

all: $(PROGRAMS)

call: call.o shared.o
internal: internal.o shared.o
car: car.o shared.o
controller: controller.o shared.o
safety: safety.o   # safety-critical: self-contained, does not use shared.c

safety.o: safety.c
	$(CC) $(CFLAGS) -c $<

%.o: %.c shared.h
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(PROGRAMS) *.o

.PHONY: all clean
