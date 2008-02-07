ifndef STAPL
  STAPL=../utils/stapl_release
endif

ifndef OBPRM
  OBPRM=.
endif

#set default platform
ifndef platform
  platform=LINUX_gcc
endif

#set default rts
ifndef rts
  rts=MPI
endif

#set default stl
ifndef stl
  stl=STLport/stlport
endif

#set default library type
ifndef library
  library=static
endif

#set default debug
ifndef debug
  debug=1
endif

#set default CHECKIFSAMECC
ifndef CHECKIFSAMECC
  CHECKIFSAMECC=1
endif

#ROBOT = 
#ROBOT_DEF = -DPMPRigid
ROBOT_DEF = -DPMPSerial
#ROBOT_DEF = -DPMPCfg2D
#ROBOT_DEF = -DPMPReachDistCC
#ROBOT_DEF = -DPMPReachDistCCFixed
#ROBOT_DEF = -DPMPRigidMulti

include $(STAPL)/GNUmakefile.STAPLdefaults
include $(OBPRM)/GNUmakefile.OBPRMdefaults

#common misc.
DEFS = $(DEBUG_DEF) $(CD_DEF) -DCHECKIFSAMECC=$(CHECKIFSAMECC) -DVERBOSE=0 $(STAPLFLAGS) $(DEPS) $(ROBOT_DEF) -DTIXML_USE_STL -Wno-deprecated 
INCLUDES = -I. $(STAPL_INCL) $(CD_INCL) $(PMPL_INCL) -I/usr/include/boost
LIBS = $(OBPRMLIB) $(CD_LIB) $(STAPLLIB) $(TINYXMLLIB) -L/usr/lib -lboost_program_options

#compile targest
default: libs

utils: 
	cd ../utils/geom/CD-libs/; make 
	cd ../utils/xml/tinyxml/;make libs


include $(OBPRM)/GNUmakefile.OBPRMobjs
objs: $(OBJS)

libs: $(OBPRM_LIBFILE) 

libobprm.a: $(OBJS) $(XML_LIBFILE) $(CD_LIBFILE) 
	$(AR) libobprm.a $(OBJS)

#libobprm.so: objs
#	$(LD) $(OBJS) -o libobprm.so

libobprm.dylib: objs
	$(LD) $(OBJS) -o libobprm.dylib -lc -lgcc -lstdc++ $(CD_LIB)

%: main_%.cpp $(OBPRM_LIBFILE) 
	$(CC) $(OPTS) $(LIBFLAGS) $(INCLUDES) $(DEFS) main_$@.cpp $(LIBS) -o $@
	rm -f $@.d

metaplanner: main_metaplanner.cpp $(OBPRM_LIBFILE)
	$(CC) $(OPTS) $(LIBFLAGS) $(INCLUDES) $(DEFS) -DCOLLISIONCFG main_metaplanner.cpp $(LIBS) -o metaplanner

%.o: %.cpp
ifeq ($(platform2), SUN_CC)
	$(CC) $(OPTS) $(INCLUDES) $(DEFS) -xM1 -c $< > $*.d
endif
	$(CC) $(OPTS) $(INCLUDES) $(DEFS) -c $< -o $*.o
	touch $*.d
	cat $*.d >> Dependencies
	rm -f $*.d

clean:
	rm -f obprm
	rm -f query
	rm -f metaplanner

reallyclean:
	rm -f obprm
	rm -f query
	rm -f metaplanner
	rm -f pmp
	rm -f *.a *.so *.o *.dylib
	rm -f $(OBJS)
	rm -R -f ii_files
	rm -R -f SunWS_cache
	rm -f so_locations
	rm -f Dependencies
	touch Dependencies

-include Dependencies
