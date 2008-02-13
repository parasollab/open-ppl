# set default platform
ifndef platform
  platform = LINUX_gcc
endif

# set default debug
ifndef debug
  debug = 1
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


# include makefile defaults
include ${UTILS_DIR}/Makefile.defaults
include ${PMPL_DIR}/Makefile.PMPLdefaults


# set default CHECKIFSAMECC
ifndef CHECKIFSAMECC
  CHECKIFSAMECC = 1
endif


# define robot type
#ROBOT_DEF = -DPMPRigid
ROBOT_DEF = -DPMPSerial
#ROBOT_DEF = -DPMPSerial2DOF
#ROBOT_DEF = -DPMPCfg2D
#ROBOT_DEF = -DPMPReachDistCC
#ROBOT_DEF = -DPMPReachDistCCFixed
#ROBOT_DEF = -DPMPRigidMulti


# macros
DEFS += ${UTILS_DEF} ${ROBOT_DEF} -DCHECKIFSAMECC=${CHECKIFSAMECC} -DVERBOSE=0
INCL = -I. ${PMPL_INCL} ${UTILS_INCL} ${STAPL_INCL} ${BOOST_INCL}
LIBS =  ${PMPL_LIB} ${UTILS_LIB} ${BOOST_LIB}


# targets
default_target: libs

utils: cd_libs xml_libs 
cd_libs: ${CD_LIBFILE}
xml_libs: ${XML_LIBFILE}

objs: ${OBJS}

libs: utils libobprm.a 

libobprm.a: ${OBJS}
	${AR} libobprm.a ${OBJS}

%: main_%.cpp libs 
	@${MAKE} -s check_platform
	${CXX} ${CXXFLAGS} ${OPTS} ${DEFS} ${INCL} $< ${LIBS} -o $@

#metaplanner: main_metaplanner.cpp libs
#	@${MAKE} -s check_platform
#	${CXX} ${CXXFLAGS} ${OPTS} ${DEFS} -DCOLLISIONCFG ${INCL} $< ${LIBS} -o $@

%.o: %.cpp
	@${MAKE} -s check_platform
	${CXX} -c ${CXXFLAGS} ${OPTS} ${DEFS} ${INCL} ${DEPS} $< -o $@
	cat $*.d >> Dependencies
	rm -f $*.d


# clean targets
CLEAN = obprm query pmpl metaplanner

reallyclean:
	@${MAKE} clean
	rm -f *.a ${OBJS} *.d
	rm -f Dependencies
	touch Dependencies

# include automatically generated dependencies
include Dependencies
