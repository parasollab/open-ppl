# Makefile for RMPULP with C-space
# Author Marco A. Morales

#Compiler g++
CC = g++

# Compiler flags
# -g enables debugging
# -Wall displays all the warning messages
CFLAGS = -g -Wall


#OBPRM directory
OBPRM = ../obprm/OBPRM
OBPRM_INC = -I${OBPRM}
OBPRM_LIB = -L${OBPRM} -lobprm

#Graph directory
GRAPH = ../obprm/stapl
GRAPH_INC = -I${GRAPH}/include/graph

#Collision detection packages available
#CD = /research/sunburn/dsmft/CD-libs/linux
CD = /pub/dsmft1/CD-libs/solaris


VCLIP = ${CD}/VCLIP
VCLIP_INC = -I${VCLIP}/include
VCLIP_LIB = -L${VCLIP}/lib -lvclip
VCLIP_DEF = -DUSE_VCLIP

RAPID = ${CD}/RAPID
RAPID_INC = -I${RAPID}
RAPID_LIB = -L${RAPID} -lRAPID
RAPID_DEF = -DUSE_RAPID

PQP = ${CD}/PQP
PQP_INC = -I${PQP}/include
PQP_LIB = -L${PQP}/lib -lPQP
PQP_DEF = -DUSE_PQP

INCLUDE = ${OBPRM_INC} ${GRAPH_INC} ${VCLIP_INC} ${RAPID_INC} ${PQP_INC}
LIB = ${OBPRM_LIB} ${VCLIP_LIB} ${RAPID_LIB} ${PQP_LIB}
#INCLUDE = ${OBPRM_INC} ${GRAPH_INC} ${RAPID_INC}
#LIB = ${OBPRM_LIB} ${RAPID_LIB}

DEFS_HP = ${VCLIP_DEF} ${RAPID_DEF}
DEFS_LINUX = ${DEFS_HP}
DEFS_SGI = ${DEFS_HP} 
DEFS_SUN = ${RAPID_DEF} ${PQP_DEF} -DSOLARIS

#Change after writing the c files (instead of having everything in h files)
HEADERS = RayCSpace.h RayTracer.h
PROGRAM_NAME = rmpulpc
OBJS = RayCSpace.o RayTracer.o

$(PROGRAM_NAME) :	$(HEADERS) $(OBJS) $(PROGRAM_NAME).cpp
	$(CC) $(CFLAGS) -o $(PROGRAM_NAME) $(PROGRAM_NAME).cpp $(OBJS) $(INCLUDE) $(LIB) $(DEFS_SUN)

main_obprm :       $(HEADERS) $(OBJS) main_obprm.cpp
	$(CC) $(CFLAGS) -o main_obprm main_obprm.cpp $(OBJS) $(INCLUDE) $(LIB) $(DEFS_SUN)

main_query :       $(HEADERS) $(OBJS) main_query.cpp
	$(CC) $(CFLAGS) -o main_query main_query.cpp $(OBJS) $(INCLUDE) $(LIB) $(DEFS_SUN)

%.o :	%.cpp
	$(CC) -c $(CFLAGS) -o $@ $< $(INCLUDE) $(DEFS_SUN)

clean :
	rm -f $(PROGRAM_NAME) $(OBJS)

