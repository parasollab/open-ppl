# $Id$
#------------------------------------------------------
# Make your own and use it or use the one in BASELINE
#------------------------------------------------------

# -- for dynamic shared object library libobprm.so
   OBPRM_DIR   = ../OBPRM
   OBPRM_LIB   = -L${OBPRM_DIR} -lobprm

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


 AFLAGS    = -I. -I${OBPRM_DIR} ${STL_INCL} ${CSTK_INCL} ${VCLIP_INCL} ${RAPID_INCL} ${PQP_INCL} ${DEFS}


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
CXXFLAGS   = $(CFLAGS) 
DSOFLAGS   = $(CFLAGS) -shared #-no_unresolved

#-----------------------------------------------------------------------------
# Build 
#--------------------------------------------------------------------------

#--------------------------------------------------------------------
deps:: 
	touch Dependencies

libs:: objs

#--------------------------------------------------------------------
prm:: main_prm.c libs
	$(CXX) $(CXXFLAGS) -o prm \
                main_prm.c \
		$(OBJS) $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}
	rm -f main_prm.o prm.d

#--------------------------------------------------------------------
aprm:: main_aprm.c libs
	$(CXX) $(CXXFLAGS) -o aprm \
                main_aprm.c \
		$(OBJS) $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}
	rm -f main_aprm.o aprm.d

#--------------------------------------------------------------------
obprm:: main_obprm.c libs
	$(CXX) $(CXXFLAGS) -o obprm \
                main_obprm.c \
		$(OBJS) $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}
	rm -f main_obprm.o obprm.d

#--------------------------------------------------------------------
query:: main_query.c libs
	$(CXX) $(CXXFLAGS) -o query \
                main_query.c \
                $(OBJS) $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}
	rm -f main_query.o query.d

#--------------------------------------------------------------------
.SUFFIXES: .c
.c.o:
	$(CXX) ${CXXFLAGS} -c $<
	cat $*.d >> Dependencies
	rm -f $*.d

#--------------------------------------------------------------------
% :: %.o
	$(CXX) $(CXXFLAGS) -o $@ $< $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}

#--------------------------------------------------------------------

OBJS = \
        MyInput.o \
        DynamicsLocalPlanners.o \
        CfgProtein.o \
        BioPotentials.o \
        MyQuery.o \
	CfgLigand.o \
	PotentialForceGrids.o \
	GradientDecent.o

objs:: $(OBJS)

#--------------------------------------------------------------------
runclean::
	rm -f mapnodes.*
	rm -f *.map
	rm -f *.path

clean::
	rm -f query
	rm -f obprm
	rm -f aprm
	rm -f prm

reallyclean::
	rm -f query
	rm -f obprm
	rm -f aprm
	rm -f prm
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

