# External Makefiles ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

include ../makefile_includes/Makefile.defaults

# Compiler Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

INCL = -I. -I/usr/local/include

# Object File Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

OBJECTS = KM_ANN.o KMeans.o KMterm.o KMrand.o KCutil.o KCtree.o KMdata.o \
	  KMcenters.o KMfilterCenters.o KMlocal.o

# Library Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Default target is the library.
.DEFAULT_GOAL := $(LIBRARY)

LIBRARY := libkmeans.a
$(LIBRARY): $(OBJECTS)
	@echo Linking k-means library...
	@rm -f $@
	@$(AR) $@ $(OBJECTS)

# Cleanup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

.PHONY:
clean:
	@echo Cleaning k-means library...
	@rm -f $(OBJECTS) $(LIBRARY)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
