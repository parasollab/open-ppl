#------------------------------------------------------
# Make your own and use it or use the one in BASELINE
#------------------------------------------------------

   ADAPT_DIR = .
   ADAPT_LIB = -L${ADAPT_DIR} -ladaptive

# -- for dynamic shared object library libobprm.so
   OBPRM_DIR   = ../OBPRM
   OBPRM_LIB   = -L${OBPRM_DIR} -lobprm
   OBPRM_INCL  = -I${OBPRM_DIR}/

#------------------------------------------------------
# Basic defns; do not change
#------------------------------------------------------

#-----------------------------------------------------------------------------
# collision detection packages available
#--------------------------------------------------------------------------
 CD_DIR = /research/sunburn/dsmft/CD-libs/linux

 VCLIP_DIR     = ${CD_DIR}/VCLIP
 VCLIPLIB_DIR  = ${VCLIP_DIR}/lib
 VCLIPLIB      = -L${VCLIPLIB_DIR} -lvclip
 VCLIP_INCL    = -I${VCLIP_DIR}/include

 RAPID_DIR     = ${CD_DIR}/RAPID
 RAPIDLIB_DIR  = ${RAPID_DIR}
 RAPIDLIB      = -L${RAPIDLIB_DIR} -lRAPID
 RAPID_INCL    = -I${RAPID_DIR}

 PQP_DIR       = ${CD_DIR}/PQP
 PQPLIB_DIR    = ${PQP_DIR}/lib
 PQPLIB        = -L${PQPLIB_DIR} -lPQP
 PQP_INCL      = -I${PQP_DIR}/include

#-----------------------------------------------------------------------------
# Defines for various types of executables
#	VERBOSE  for printing out all kinds of babble for debugging
#       INTERMEDIATE_FILES for creating intermediate "*.path" files
#--------------------------------------------------------------------------
   ADEFS = -DVERBOSE=0 -DINTERMEDIATE_FILES=1 -DCHECKIFSAMECC=0  -DUSE_RAPID -DUSE_VCLIP -DUSE_PQP

### DEFS = ${ADEFS} -DQUIETGRAPH -DQUIET
    DEFS = ${ADEFS} 

showdefs :: 
	echo "${DEFS}";


 AFLAGS    = -I. ${OBPRM_INCL} ${STL_INCL} ${CSTK_INCL} ${VCLIP_INCL} ${RAPID_INCL} ${PQP_INCL} ${DEFS}

#-----------------------------------------------------------------------------
# Choose a compiler & its options
#--------------------------------------------------------------------------
 CC = g++
#CC = /pub/gnu_sgi/bin/g++

CXX = ${CC}

# -- for development on TERMINATOR (can't seem to use it better due to cstk)
   OPTS = -g -O2 -fexceptions
#  OPTS = -o32 -O1 -show -ptv 
# -- optimized (our O2's) (slow_compile/good_code) --
#  OPTS =  -o32 -O2 -OPT:Olimit=0 -show

#  DEPS = 
#  DEPS = -MDupdate Dependencies 
   DEPS = -MMD #Dependencies

# -- warnings (default, full, none) --
#  CWARNS = 
#  CWARNS = -fullwarn
   CWARNS = -w

CFLAGS     = $(OPTS) $(AFLAGS) $(DEPS) $(CWARNS)
CFLAGS2    = $(OPTS) $(AFLAGS) $(CWARNS)
CXXFLAGS   = $(CFLAGS) 
DSOFLAGS   = $(CFLAGS) -shared #-no_unresolved

#-----------------------------------------------------------------------------
# Build 
#--------------------------------------------------------------------------

#--------------------------------------------------------------------
deps:: 
	touch Dependencies

libs:: objs libadaptive.so

libadaptive.so:: $(OBJS)
	$(CXX) -o libadaptive.so $(DSOFLAGS) $(OBJS)

#--------------------------------------------------------------------
aprm:: main_aprm.c libs
	$(CXX) $(CFLAGS2) -o aprm \
                main_aprm.c \
		$(OBJS) $(ADAPT_LIB) $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}
	rm -f main_aprm.o aprm.d

#--------------------------------------------------------------------
query:: main_query.c libs 
	$(CXX) $(CFLAGS2) -o query \
                main_query.c \
                $(OBJS) $(ADAPT_LIB) $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}
	rm -f main_query.o query.d

#--------------------------------------------------------------------
.SUFFIXES: .c
.c.o:
	$(CXX) ${CXXFLAGS} -c $<
	cat $*.d >> Dependencies
	rm -f $*.d

% :: %.o
	$(CXX) $(CXXFLAGS) -o $@ $< $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}

#--------------------------------------------------------------------

OBJS = \
        AdaptiveQuery.o \
	QueryRequirements.o \
	NoopLocalPlanners.o

objs:: $(OBJS)

#--------------------------------------------------------------------
runclean::
	rm -f mapnodes.*
	rm -f *.map
	rm -f *.path

clean::
	rm -f query
	rm -f aprm

reallyclean::
	rm -f query
	rm -f aprm
	rm -f *.a *.so *.o
	rm -R -f ii_files
	rm -f so_locations
	rm -f Dependencies
	touch Dependencies

#-----------------------------------------------------------------------------
# Build and update dependencies
#-----------------------------------------------------------------------------

Dependencies::
	echo >Dependencies "#-*- Makefile -*-"

include Dependencies

