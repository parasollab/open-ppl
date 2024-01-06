# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

ifndef STAPL
  STAPL = $(shell echo "$(PWD)" | sed 's,/benchmarks/data_mining/frequent_itemset,,')
endif

include $(STAPL)/GNUmakefile.STAPLdefaults

OBJ = weights.o driver.o freqmine.o eclat1d.o eclat1d_step2.o eclat1d_step3.o eclat1d_step4.o eclat1d_step5.o

#eclat2d.o eclat2d_step2.o eclat2d_step3.o eclat2d_step4.o eclat2d_step5.o

compile: $(OBJ)
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} ${OBJ} -o freqmine.exe ${STAPL_LIBRARIES} ${LIB} ${LIB_EPILOGUE}

execute: compile
	export STAPL_NUM_THREADS=1 ; \
	echo '########################################' ; \
	echo '# look at sample.dbg for debug output  #' ; \
	echo '########################################' ; \
	$(EXEC) -n 2 ./freqmine.exe sample.meta sample.txt freq.out t 0.05

clean:
	rm -f *.o *.exe *.out *.dbg

weights.o: weights.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} weights.cc -c $<

driver.o: driver.cc
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} driver.cc -c $<

freqmine.o: freqmine.cc freqmine.hpp algo.hpp algo_supp.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} freqmine.cc -c $<

eclat1d.o: eclat1d.cc freqmine.hpp algo.hpp algo_supp.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} eclat1d.cc -c $<

eclat1d_step2.o: eclat1d_step2.cc freqmine.hpp eclat1d.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} eclat1d_step2.cc -c $<

eclat1d_step3.o: eclat1d_step3.cc freqmine.hpp eclat1d.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} eclat1d_step3.cc -c $<

eclat1d_step4.o: eclat1d_step4.cc freqmine.hpp eclat1d.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} eclat1d_step4.cc -c $<

eclat1d_step5.o: eclat1d_step4.cc freqmine.hpp eclat1d.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} eclat1d_step5.cc -c $<


eclat2d.o: eclat2d.cc freqmine.hpp algo.hpp algo_supp.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} eclat2d.cc -c $<

eclat2d_step2.o: eclat2d_step2.cc freqmine.hpp eclat2d.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} eclat2d_step2.cc -c $<

eclat2d_step3.o: eclat2d_step3.cc freqmine.hpp eclat2d.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} eclat2d_step3.cc -c $<

eclat2d_step4.o: eclat2d_step4.cc freqmine.hpp eclat2d.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} eclat2d_step4.cc -c $<

eclat2d_step5.o: eclat2d_step5.cc freqmine.hpp eclat2d.hpp
	${CC} ${STAPL_CXXFLAGS} ${CXXFLAGS} eclat2d_step5.cc -c $<
