include ../makefile_includes/Makefile.defaults

INCL = -I. -I/usr/local/include

OBJECTS = KM_ANN.o KMeans.o KMterm.o KMrand.o KCutil.o KCtree.o KMdata.o KMcenters.o \
	KMfilterCenters.o KMlocal.o

CLEAN = $(OBJECTS) *.a *.so *~ *.o

#default target
default_target: library

library: libkmeans.a

install: library

libkmeans.a: ${OBJECTS}
	/bin/rm -f libkmeans.a
	${AR} libkmeans.a ${OBJECTS}

.SUFFIXES: .o .cpp


