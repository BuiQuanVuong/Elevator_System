CC = gcc
CFLAGS = -std=c11 -D_DEFAULT_SOURCE -Wall -Wextra -Wpedantic -pthread
LDLIBS = -pthread -lrt

# car, controller and safety get added here as they are written
PROGRAMS = call internal

all: $(PROGRAMS)

call: call.o common.o
internal: internal.o common.o

%.o: %.c common.h
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(PROGRAMS) *.o

.PHONY: all clean
