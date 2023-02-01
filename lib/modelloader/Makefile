# External Makefiles ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

include ../makefile_includes/Makefile.defaults

# Compiler Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

DEFS := $(CGAL_DEF)
INCL := -I. -I../mathtool/ $(CGAL_INCL)
ifeq (${platform}, CRAY_gcc)
  # TODO Remove Matthew's personal directory from this include and replace with
  #      a system copy. If one is not present, we should get it there or add a
  #      version to our utilities.
  INCL += -I${BOOST_ROOT}/include -I/mnt/lustre/lus0/fourcow2/CGAL/include/
endif

# Object File Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

OBJECTS := IModel.o ModelFactory.o MovieBYULoader.o ObjLoader.o BVHDataLoader.o

# Library Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Default is to build the library.
.DEFAULT_GOAL = $(LIBRARY)

LIBRARY := libmodloader.a
$(LIBRARY): $(OBJECTS)
	@echo Linking modelloader library...
	@$(AR) $@ $^

# Cleanup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

.PHONY: clean
clean:
	@echo Cleaning modelloader library...
	@rm -f $(LIBRARY) $(OBJECTS)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
