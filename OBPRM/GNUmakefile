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

include $(STAPL)/GNUmakefile.STAPLdefaults
include $(OBPRM)/GNUmakefile.OBPRMdefaults

#common misc.
DEFS = $(DEBUG_DEF) $(CD_DEF) -DCHECKIFSAMECC=$(CHECKIFSAMECC) -DVERBOSE=0 $(STAPLFLAGS) $(DEPS) -DTIXML_USE_STL -Wno-deprecated -D_LOG
INCLUDES = -I. $(STAPL_INCL) $(CD_INCL)
LIBS = $(OBPRMLIB) $(CD_LIB) $(STAPLLIB) $(TINYXMLLIB)

#compile targest
default: libs

utils: 
	cd ../utils/geom/CD-libs/RAPID/;make libRAPID.a
	cd ../utils/xml/tinyxml/;make libs
	cd ../utils/geom/CD-libs/PQP_v1.2;make library
	cd ../utils/geom/CD-libs/qhull3.0/src;make all
	cd ../utils/geom/CD-libs/VCLIP;make all

libs: utils objs $(OBPRM_LIBFILE)

libobprm.a: utils objs
	$(AR) libobprm.a $(OBJS)

#libobprm.so: objs
#	$(LD) $(OBJS) -o libobprm.so

libobprm.dylib: objs
	$(LD) $(OBJS) -o libobprm.dylib -lc -lgcc -lstdc++ $(CD_LIB)

%: main_%.cpp libs 
	$(CC) $(OPTS) $(LIBFLAGS) $(INCLUDES) $(DEFS) main_$@.cpp $(LIBS) -o $@
	rm -f $@.d


pmp: main_pmp.cpp libs
	$(CC) $(OPTS) $(LIBFLAGS) $(INCLUDES) $(DEFS)  main_pmp.cpp $(LIBS)  -o main_pmp
	rm -f $@.d



metaplanner: main_metaplanner.cpp libs 

	$(CC) $(OPTS) $(LIBFLAGS) $(INCLUDES) $(DEFS) -DCOLLISIONCFG main_metaplanner.cpp $(LIBS) -o metaplanner

%.o: %.cpp
ifeq ($(platform2), SUN_CC)
	$(CC) $(OPTS) $(INCLUDES) $(DEFS) -xM1 -c $< > $*.d
endif
	$(CC) $(OPTS) $(INCLUDES) $(DEFS) -c $<
	touch $*.d
	cat $*.d >> Dependencies
	rm -f $*.d

OBJS = \
	FreeBody.o \
	FixedBody.o \
	Connection.o \
	Contact.o \
	DHparameters.o \
	Transformation.o \
	Orientation.o \
	GMSPolyhedron.o \
	Matrix.o \
	Vectors.o \
	Body.o \
	MultiBody.o \
	Parameters.o \
	Input.o \
	Cfg.o \
	Cfg_free.o \
	Cfg_fixed_PRR.o \
	Cfg_free_tree.o \
	Cfg_fixed_tree.o \
	Cfg_2D.o \
	Environment.o \
	util.o \
	Clock_Class.o \
	Clock_Elapsed.o \
	Stat_Class.o \
	DistanceMetrics.o \
	CollisionDetection.o \
	QueryCmds.o \
	Weight.o \
	Boundary.o \
	MPStrategy.o \
	MPProblem.o 
#	Haptic.o \
#	Push.o \


objs: $(OBJS)

clean:
	rm -f obprm
	rm -f query
	rm -f metaplanner

reallyclean:
	rm -f obprm
	rm -f query
	rm -f metaplanner
	rm -f *.a *.so *.o *.dylib
	rm -R -f ii_files
	rm -R -f SunWS_cache
	rm -f so_locations
	rm -f Dependencies
	touch Dependencies


include Dependencies

