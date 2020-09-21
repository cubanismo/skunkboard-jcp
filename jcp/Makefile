CC=gcc
CFLAGS=-O2 -Wall

SRCC=jcp.c
SRCH=dumpver.h flashstub.h romdump.h turbow.h univbin.h
OBJS=$(SRCC:.c=.o) 

all: .depend jcp 

jcp: $(OBJS) $(SRCH)
	gcc -o jcp $(OBJS) -lusb

%.o: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *~ $(OBJS) jcp

.depend: $(SRCC)
	$(CC) -MM $(SRCC) > .depend

-include .depend
