# set default platform
ifndef platform
  platform = LINUX_64_gcc
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
  STL_LIB = 4.4.4
endif 

# include makefile defaults
include ${UTILS_DIR}/makefile_includes/Makefile.defaults
include ${PMPL_DIR}/Makefile.PMPLdefaults

ifndef ROBOT_DEF
  # define robot type
  ROBOT_DEF = -DPMPRigid
  #ROBOT_DEF = -DPMPSerial
  #ROBOT_DEF = -DPMPSerial2DOF
  #ROBOT_DEF = -DPMPFixedTree
  #ROBOT_DEF = -DPMPCfg2D
  #ROBOT_DEF = -DPMPCfg2DWithRot
  #ROBOT_DEF = -DPMPCfgSurface
  #ROBOT_DEF = -DPMPReachDistCC
  #ROBOT_DEF = -DPMPReachDistCCFixed
  #ROBOT_DEF = -DPMPRigidMulti
  #ROBOT_DEF = -DPMPManifold
endif

# macros
DEFS += ${UTILS_DEF} ${ROBOT_DEF} ${CGAL_DEF} ${STAPL_DEF} 
INCL =  ${PMPL_INCL} ${UTILS_INCL} ${STAPL_INCL} ${BOOST_INCL} ${CGAL_INCL} ${MPNN_INCL}
LIBS =  ${PMPL_LIB} ${UTILS_LIB} ${MPNN_LIB} ${STAPL_LIB}

# targets
default_target: libs

utils: cd_libs xml_libs nn_libs mathtool_lib modelloader_lib kmeans_lib
cd_libs: ${CD_LIBFILE}
xml_libs: ${XML_LIBFILE}
nn_libs: ${MPNN_LIBFILE}
mathtool_lib: ${MATHTOOL_LIBFILE}
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
	cd ${CD_DIR}; ${MAKE} clean
	cd ${TINYXML_DIR}; ${MAKE} clean
	cd ${MPNN_SRC}; ${MAKE} clean
	cd ${MATHTOOL_DIR}; ${MAKE} clean
	cd ${MODELLOADER_DIR}; ${MAKE} clean
	cd ${KMEANS_DIR}; ${MAKE} clean


# include automatically generated dependencies
-include Dependencies
