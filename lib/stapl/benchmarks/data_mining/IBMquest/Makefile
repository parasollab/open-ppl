# http://sourceforge.net/projects/ibmquestdatagen/files/latest/download

HFILES1 = glob.h dist.h
HFILES2 = gen.h

CFILES1 =  ran1.cc expdev.cc gammln.cc gasdev.cc poidev.cc dist.cc
CFILES2 =  gen.cc main.cc command.cc

OBJECTS1 = ran1.o expdev.o gammln.o gasdev.o poidev.o dist.o
OBJECTS2 = gen.o main.o command.o

LIBES = -lm
CC = g++
CFLAGS = -O

gen.exe:	$(OBJECTS1) $(OBJECTS2)
	$(CC) $(CFLAGS) $(OBJECTS1) $(OBJECTS2) $(LIBES) -o gen.exe

test.exe:	$(OBJECTS1) test.o
	$(CC) $(CFLAGS) $(OBJECTS1) test.o $(LIBES) -o test.exe

clean:
	/bin/rm $(OBJECTS1) $(OBJECTS2)

$(OBJECTS1): $(HFILES1)
$(OBJECTS2): $(HFILES1) $(HFILES2)

