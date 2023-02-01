# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

# if STAPL is defined, use it.

#STAPL=/opt/stapl

ifdef STAPL
ifdef RUNNING_VALIDATION
 # STAPL was set at the top level when validation was launched.
 include $(STAPL)/GNUmakefile.STAPLdefaults
 COMP=$(CC)
 STAPL_LIBS= $(STAPL_LIBRARIES) $(LIB)
else
	STAPL_FLAGS= -I$(STAPL)/tools/libstdc++/4.8.2 -I$(STAPL)/tools      \
	-I$(STAPL)/include -I$(STAPL) -I/usr/local/boost/boost-1.63/include \
	-D_STAPL -DBOOST_RESULT_OF_USE_TR1_WITH_DECLTYPE_FALLBACK -std=c++11 -Wall -Wno-unused-local-typedefs

	STAPL_LIBS= -L$(STAPL)/lib -lstapl -lrt \
	-L/usr/local/boost/boost-1.63/lib64 -lboost_serialization -lboost_system
	
	COMP= /usr/lib64/openmpi/bin/mpic++

.cc.o:
	$(COMP) -c $(STAPL_FLAGS) $<
endif
else
#if stapl/runtime.hpp does not exist two directories up,
#assume STAPL is installed.
ifeq ($(wildcard ../../stapl/runtime.hpp),)

	STAPL= /opt/stapl

	STAPL_FLAGS= -I$(STAPL)/tools/libstdc++/4.8.2 -I$(STAPL)/tools      \
	-I$(STAPL)/include -I$(STAPL) -I/usr/local/boost/boost-1.63/include \
	-D_STAPL -DBOOST_RESULT_OF_USE_TR1_WITH_DECLTYPE_FALLBACK -std=c++11

	STAPL_LIBS= -lstapl -lrt -lboost_serialization -lboost_system
	
	COMP= /usr/lib64/openmpi/bin/mpic++

.cc.o:
	$(COMP) -c $(STAPL_FLAGS) $<

else
#otherwise, we're inside a working copy.
 STAPL = $(shell echo "$(PWD)" | sed 's,/docs/tutorial_guide,,')
 include $(STAPL)/GNUmakefile.STAPLdefaults
 COMP=$(CC)
 STAPL_LIBS= $(STAPL_LIBRARIES) $(LIB)
endif

endif


# for numprocs <= 2
DATA_SRC= tiny_data.cc
DATA_OBJ= tiny_data.o

# for numprocs >2 && <= 8
#DATA_SRC= small_data.cc
#DATA_OBJ= small_data.o

# for numprocs >8
#DATA_SRC= medium_data.cc
#DATA_OBJ= medium_data.o

# for numprocs <= 2
GRAF_SRC= tiny_graf.cc
GRAF_OBJ= tiny_graf.o

# for numprocs >2 && <= 8
#GRAF_SRC= small_graf.cc
#GRAF_OBJ= small_graf.o

# for numpros >8
#GRAF_SRC= medium_graf.cc
#GRAF_OBJ= medium_graf.o

OBJ2= ex_200.o ex_201.o ex_202.o ex_203.o ex_204.o ex_205.o ex_206.o \
	ex_208.o ex_209.o ex_210.o ex_211.o ex_207.o ex_212.o

OBJ3= ex_301.o ex_302.o ex_303.o ex_304.o ex_305.o ex_306.o 

OBJ4= ex_401.o ex_402.o ex_403.o ex_404.o ex_405.o ex_406.o 

OBJ5= ex_501.o ex_502.o ex_503.o ex_504.o ex_505.o ex_506.o ex_507.o ex_508.o

OBJ6= ex_601.o ex_602.o ex_603.o ex_604.o ex_605.o ex_606.o ex_607.o


ch2:	$(OBJ2) $(DATA_OBJ)
	

ch3:	$(OBJ3) $(DATA_OBJ)
	

ch4:	$(OBJ4) $(DATA_OBJ)
	

ch5:	$(OBJ5) $(DATA_OBJ)
	

ch6:	$(OBJ6) $(GRAF_OBJ)
	

EXECUTABLES= 200.exe 201.exe 202.exe 203.exe 204.exe 205.exe 206.exe 207.exe \
	 208.exe 209.exe 210.exe 211.exe 212.exe \
	 301.exe 302.exe 303.exe 304.exe 305.exe 306.exe \
	 401.exe 402.exe 403.exe 404.exe 405.exe 406.exe \
	 501.exe 502.exe 503.exe 504.exe 505.exe 506.exe 507.exe 508.exe \
	 601.exe 602.exe 603.exe 604.exe 605.exe 606.exe 607.exe

default: compile

compile: $(EXECUTABLES)

test: compile
	echo "examples are compile-only at this point"

200.exe: ex_200.o
	$(COMP) $(CXXFLAGS) ex_200.o $(STAPL_LIBS) -o 200.exe

201.exe: ex_201.o
	$(COMP) $(CXXFLAGS) ex_201.o $(STAPL_LIBS) -o 201.exe

202.exe: ex_202.o
	$(COMP) $(CXXFLAGS) ex_202.o $(STAPL_LIBS) -o 202.exe

203.exe: ex_203.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_203.o $(DATA_OBJ) $(STAPL_LIBS) -o 203.exe

204.exe: ex_204.o
	$(COMP) $(CXXFLAGS) ex_204.o $(STAPL_LIBS) -o 204.exe

205.exe: ex_205.o
	$(COMP) $(CXXFLAGS) ex_205.o $(STAPL_LIBS) -o 205.exe

206.exe: ex_206.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_206.o $(DATA_OBJ) $(STAPL_LIBS) -o 206.exe

207.exe: ex_207.o
	$(COMP) $(CXXFLAGS) ex_207.o $(STAPL_LIBS) -o 207.exe

208.exe: ex_208.o
	$(COMP) $(CXXFLAGS) ex_208.o $(STAPL_LIBS) -o 208.exe

209.exe: ex_209.o
	$(COMP) $(CXXFLAGS) ex_209.o $(STAPL_LIBS) -o 209.exe

210.exe: ex_210.o
	$(COMP) $(CXXFLAGS) ex_210.o $(STAPL_LIBS) -o 210.exe

211.exe: ex_211.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_211.o $(DATA_OBJ) $(STAPL_LIBS) -o 211.exe

212.exe: ex_212.o
	$(COMP) $(CXXFLAGS) ex_212.o $(STAPL_LIBS) -o 212.exe

301.exe: ex_301.o
	$(COMP) $(CXXFLAGS) ex_301.o $(STAPL_LIBS) -o 301.exe

302.exe: ex_302.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_302.o $(DATA_OBJ) $(STAPL_LIBS) -o 302.exe

303.exe: ex_303.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_303.o $(DATA_OBJ) $(STAPL_LIBS) -o 303.exe

304.exe: ex_304.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_304.o $(DATA_OBJ) $(STAPL_LIBS) -o 304.exe

305.exe: ex_305.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_305.o $(DATA_OBJ) $(STAPL_LIBS) -o 305.exe

306.exe: ex_306.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_306.o $(DATA_OBJ) $(STAPL_LIBS) -o 306.exe

401.exe: ex_401.o
	$(COMP) $(CXXFLAGS) ex_401.o $(STAPL_LIBS) -o 401.exe

402.exe: ex_402.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_402.o $(STAPL_LIBS) -o 402.exe

403.exe: ex_403.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_403.o $(STAPL_LIBS) -o 403.exe

404.exe: ex_404.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_404.o $(STAPL_LIBS) -o 404.exe

405.exe: ex_405.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_405.o $(DATA_OBJ) $(STAPL_LIBS) -o 405.exe

406.exe: ex_406.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_406.o $(DATA_OBJ) $(STAPL_LIBS) -o 406.exe

407.exe: ex_407.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_407.o $(GRAF_OBJ) $(STAPL_LIBS) -o 407.exe

408.exe: ex_408.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_408.o $(GRAF_OBJ) $(STAPL_LIBS) -o 408.exe

501.exe: ex_501.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_501.o $(DATA_OBJ) $(STAPL_LIBS) -o 501.exe

502.exe: ex_502.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_502.o $(DATA_OBJ) $(STAPL_LIBS) -o 502.exe

503.exe: ex_503.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_503.o $(DATA_OBJ) $(STAPL_LIBS) -o 503.exe

504.exe: ex_504.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_504.o $(DATA_OBJ) $(STAPL_LIBS) -o 504.exe

505.exe: ex_505.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_505.o $(DATA_OBJ) $(STAPL_LIBS) -o 505.exe

506.exe: ex_506.o $(DATA_OBJ)
	$(COMP) $(CXXFLAGS) ex_506.o $(DATA_OBJ) $(STAPL_LIBS) -o 506.exe

507.exe: ex_507.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_507.o $(GRAF_OBJ) $(STAPL_LIBS) -o 507.exe

508.exe: ex_508.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_508.o $(GRAF_OBJ) $(STAPL_LIBS) -o 508.exe

601.exe: ex_601.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_601.o $(GRAF_OBJ) $(STAPL_LIBS) -o 601.exe

602.exe: ex_602.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_602.o $(GRAF_OBJ) $(STAPL_LIBS) -o 602.exe

603.exe: ex_603.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_603.o $(GRAF_OBJ) $(STAPL_LIBS) -o 603.exe

604.exe: ex_604.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_604.o $(GRAF_OBJ) $(STAPL_LIBS) -o 604.exe

605.exe: ex_605.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_605.o $(GRAF_OBJ) $(STAPL_LIBS) -o 605.exe

606.exe: ex_606.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_606.o $(GRAF_OBJ) $(STAPL_LIBS) -o 606.exe

607.exe: ex_607.o $(GRAF_OBJ)
	$(COMP) $(CXXFLAGS) ex_607.o $(GRAF_OBJ) $(STAPL_LIBS) -o 607.exe

clean:
	rm -f *.o *.exe *.out

