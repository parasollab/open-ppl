# set default platform
ifndef platform
  platform = LINUX_gcc
endif


# set default debug
ifndef debug
  debug = 1
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
  STL_LIB = 4.6.2
endif

ifndef ROBOT_DEF
  # define robot type
  ROBOT_DEF = PMPCfg
  #ROBOT_DEF = PMPCfgSurface
  #ROBOT_DEF = PMPSSSurfaceMult
  #ROBOT_DEF = PMPReachDistCC
  #ROBOT_DEF = PMPReachDistCCFixed
endif

# include makefile defaults
include ${UTILS_DIR}/makefile_includes/Makefile.defaults
include ${PMPL_DIR}/Makefile.PMPLdefaults

# macros
DEFS += ${UTILS_DEF} -D${ROBOT_DEF} ${CGAL_DEF} ${STAPL_DEF}
INCL =  ${PMPL_INCL} ${UTILS_INCL} ${STAPL_INCL} ${BOOST_INCL} ${CGAL_INCL} ${MPNN_INCL}
LIBS =  ${PMPL_LIB} ${UTILS_LIB} ${MPNN_LIB} ${STAPL_LIB}

# targets
default_target: libs

utils: stapl_lib cd_libs xml_libs nn_libs modelloader_lib kmeans_lib
stapl_lib: ${STAPL_LIBFILE}
cd_libs: ${CD_LIBFILE}
xml_libs: ${XML_LIBFILE}
nn_libs: ${MPNN_LIBFILE}
modelloader_lib: ${MODELLOADER_LIBFILE}
kmeans_lib: ${KMEANS_LIBFILE}

objs: utils ${PMPL_OBJS}

libs: utils ${PMPL_LIBFILE}

${PMPL_LIBFILE}: objs
	${AR} ${PMPL_LIBFILE} ${PMPL_OBJS}

%: main_%.cpp libs
	${CXX} ${CXXFLAGS} ${OPTS} ${DEFS} ${INCL} $< ${LIBS} -o $@

%.o: %.cpp
	@${MAKE} -s check_platform
	${CXX} -c ${CXXFLAGS} ${OPTS} ${DEFS} ${INCL} ${DEPS} $< -o $@
	cat $*.d >> Dependencies
	rm -f $*.d


# clean targets
CLEAN = pmpl

reallyclean:
	@${MAKE} clean
	rm -f *.a ${PMPL_OBJS} *.d
	rm -f Dependencies
	touch Dependencies

reallyreallyclean:
	@${MAKE} reallyclean
	cd ${STAPL_DIR}; ${MAKE} platform=${platform} stl=${STL_LIB} clean
	cd ${CD_DIR}; ${MAKE} clean
	cd ${TINYXML_DIR}; ${MAKE} clean
	cd ${MPNN_SRC}; ${MAKE} clean
	cd ${MATHTOOL_DIR}; ${MAKE} clean
	cd ${MODELLOADER_DIR}; ${MAKE} clean
	cd ${KMEANS_DIR}; ${MAKE} clean


# include automatically generated dependencies
-include Dependencies
