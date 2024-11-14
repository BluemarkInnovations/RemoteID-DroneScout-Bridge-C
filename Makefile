# (c) Bluemark Innovations BV 
# MIT license

CC=gcc
LD=ld
BINDIR = ./bin
OPTIONS = -O2 -lm -lpthread -I./opendroneid-core-c/mavlink_c_library_v2/ -I./opendroneid-core-c/libopendroneid/ -I./opendroneid-core-c/libmav2odid/

all: $(BINDIR)/main

$(BINDIR)/main: main.c	
	$(CC) main.c ./opendroneid-core-c/libopendroneid/opendroneid.c -o $(BINDIR)/main $(OPTIONS) 

clean:
	rm -f $(BINDIR)/main


