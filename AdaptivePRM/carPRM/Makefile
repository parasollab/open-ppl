# $Id$
#------------------------------------------------------
# Make your own and use it or use the one in BASELINE
#------------------------------------------------------
   CARLIKE_DIR = .
   CARLIKE_LIB = -L${CARLIKE_DIR} -lcarlike

# -- for dynamic shared object library libobprm.so
   obprm_dir   = ../obprm

   OBPRM_DIR   = ${obprm_dir}/OBPRM
   OBPRM_LIB   = -L${OBPRM_DIR} -lobprm
   OBPRM_INCL  = -I${OBPRM_DIR}/

   GRAPH_DIR   = ${obprm_dir}/stapl/include/graph
   GRAPH_INCL  = -I${GRAPH_DIR}/

# -- for dynamic shared object library libadaptive.so
   ADAPT_DIR   = ../APRM
   ADAPT_LIB   = -L${ADAPT_DIR} -ladaptive
   ADAPT_INCL  = -I${ADAPT_DIR}

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
   ADEFS = -DVERBOSE=0 -DINTERMEDIATE_FILES=0 -DCHECKIFSAMECC=0  -DUSE_RAPID -DUSE_VCLIP -DUSE_PQP

### DEFS = ${ADEFS} -DQUIETGRAPH -DQUIET
    DEFS = ${ADEFS} 

showdefs :: 
	echo "${DEFS}";


 AFLAGS    = -I. ${ADAPT_INCL} ${OBPRM_INCL} ${GRAPH_INCL} ${STL_INCL} ${CSTK_INCL} ${VCLIP_INCL} ${RAPID_INCL} ${PQP_INCL} ${DEFS}


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
   DEPS = -MMD #Dependencies 

# -- warnings (default, full, none) --
#  CWARNS = 
#  CWARNS = -fullwarn
   CWARNS = -w

CFLAGS     = $(OPTS) $(AFLAGS) $(DEPS) $(CWARNS)
CXXFLAGS   = $(CFLAGS) 
DSOFLAGS   = $(CFLAGS) -shared # -no_unresolved

#-----------------------------------------------------------------------------
# Build 
#--------------------------------------------------------------------------

#--------------------------------------------------------------------
deps:: 
	touch Dependencies

libs:: objs libcarlike.so

libcarlike.so:: $(OBJS)
	$(CXX) -o libcarlike.so $(DSOFLAGS) $(OBJS)

#--------------------------------------------------------------------
aprm:: car_aprm.c libs
	$(CXX) $(CXXFLAGS) -o aprm \
                car_aprm.c \
		$(OBJS) $(CARLIKE_LIB) $(ADAPT_LIB) $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}
	rm -f car_aprm.o aprm.d

#--------------------------------------------------------------------
query:: car_query.c libs
	$(CXX) $(CXXFLAGS) -o query \
                car_query.c \
                $(OBJS) $(CARLIKE_LIB) $(ADAPT_LIB) $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}
	rm -f car_query.o query.d

#--------------------------------------------------------------------
.SUFFIXES: .c
.c.o:
	$(CXX) ${CXXFLAGS} -c $<
	cat $*.d >> Dependencies
	rm -f $*.d

% :: %.o
	$(CXX) $(CXXFLAGS) -o $@ $< $(ADAPT_LIB) $(OBPRM_LIB) $(MATHLIB) $(CSTKLIB) ${VCLIPLIB} ${RAPIDLIB} ${PQPLIB}

#--------------------------------------------------------------------

OBJS = \
	CarAdaptiveQuery.o \
	CarPriorityQuery.o \
	CarQueryReq.o \
	CfgCarLike.o \
	CurveWeight.o \
	Project.o

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
	rm -f *.d
	rm -f Dependencies
	touch Dependencies

#-----------------------------------------------------------------------------
# Build and update dependencies
#-----------------------------------------------------------------------------

Dependencies::
	echo >Dependencies "#-*- Makefile -*-"

include Dependencies

