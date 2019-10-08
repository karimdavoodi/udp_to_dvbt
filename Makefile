CC = gcc-9
EXTRA_CFLAGS := -Wno-pointer-to-int-cast
CFLAGS = -Wno-unused-but-set-variable $(EXTRA_CFLAGS) -MD -Wall -O2 -I../../include -I../lib -I./ 
LFLAGS = -Wall -O2 -lc -lpthread ../lock/libRockey4Smart.a
RM = rm -f
KD_NA = torf
TARGETS = $(KD_NA)
OBJS = api.o $(KD_NA).o 

ifeq "$(MACHINE)" "x86_64"
EXTRA_CFLAGS += -Wno-pointer-to-int-cast
endif

all: clean $(OBJS) $(TARGETS) 
	cp -f  $(KD_NA) ../../bin/torft
 
.c.o:
	$(CC) $(CFLAGS) -o $@ -c $<

.o:
	$(CC) -o $@ api.o $(KD_NA).o $(LFLAGS) 

clean:
	$(RM) $(TARGETS) core* *.o *.d .depend

-include $(wildcard *.d) dummy

