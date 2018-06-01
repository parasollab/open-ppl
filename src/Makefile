# set default platform
ifndef platform
  platform = LINUX_gcc
endif

# set default debug
ifndef debug
  debug = 0
endif

ifndef parallel
  parallel = 0
endif

# set directory paths
ifndef PMPL_DIR
  PMPL_DIR = .
endif

ifndef UTILS_DIR
  UTILS_DIR = ../utils
endif

ifndef STAPL_DIR
  STAPL_DIR = ${UTILS_DIR}/stapl_release
endif

ifndef STL_LIB
  STL_LIB = 4.8.2
endif

ifndef ROBOT_DEF
  # define robot type
  ROBOT_DEF = PMPCfg
  #ROBOT_DEF = PMPCfgMultiRobot
  #ROBOT_DEF = PMPCfgSurface
  #ROBOT_DEF = PMPState
  #ROBOT_DEF = PMPSSSurfaceMult
  #ROBOT_DEF = PMPReachDistCC
  #ROBOT_DEF = PMPReachDistCCFixed
  #ROBOT_DEF = PMPReachableVolume
endif

# include makefile defaults
include ${UTILS_DIR}/makefile_includes/Makefile.defaults
include ${PMPL_DIR}/Makefile.PMPLdefaults

# macros
DEFS += -D${ROBOT_DEF} ${UTILS_DEF} ${CGAL_DEF} ${STAPL_DEF} ${BOOST_DEF}
INCL =  ${PMPL_INCL} ${UTILS_INCL} ${STAPL_INCL} ${BOOST_INCL}
LIBS =  ${PMPL_LIB} ${UTILS_LIB} ${STAPL_LIB} ${BOOST_LIB}

# targets
.DEFAULT_GOAL := library

${PMPL_OBJS}: ${UTILS_LIBFILE} ${STAPL_LIBFILE}

${PMPL_LIBFILE}: ${PMPL_OBJS}
	${AR} ${PMPL_LIBFILE} ${PMPL_OBJS}

library: ${PMPL_LIBFILE}

ifeq (${parallel}, 1)
  MAIN = parallel_main.o
else
  MAIN = main.o
endif
${MAIN}: ${PMPL_LIBFILE}

pmpl: ${PMPL_LIBFILE} ${MAIN}
	${CXX} ${CXXFLAGS} ${OPTS} ${MAIN} ${LIBS} -o $@

%.o: %.cpp
	@${MAKE} -s check_platform
	${CXX} -c ${CXXFLAGS} ${OPTS} ${DEFS} ${INCL} ${DEPS} $< -o $@
	cat $*.d >> Dependencies
	rm -f $*.d


# clean targets
clean:
	rm -rf ${PMPL_LIBFILE} ${MAIN} pmpl

reallyclean:
	@${MAKE} clean
	rm -f ${PMPL_OBJS} *.d
	rm -f Dependencies
	touch Dependencies

reallyreallyclean:
	@${MAKE} reallyclean
	cd ${STAPL_DIR}; ${MAKE} platform=${platform} stl=${STL_LIB} clean
	cd ${CD_DIR}; ${MAKE} clean
	cd ${TINYXML_DIR}; ${MAKE} clean
	cd ${MPNN_SRC}; ${MAKE} clean
	cd ${MODELLOADER_DIR}; ${MAKE} clean
	cd ${KMEANS_DIR}; ${MAKE} clean
	cd ${TETGEN_DIR}; ${MAKE} clean


# include automatically generated dependencies
-include Dependencies
