ifndef STAPL
  STAPL=../stapl_release
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
  library=dynamic
endif

#set default debug
ifndef debug
  debug=0
endif

#set default CHECKIFSAMECC
ifndef CHECKIFSAMECC
  CHECKIFSAMECC=1
endif

include $(STAPL)/GNUmakefile.STAPLdefaults
include $(OBPRM)/GNUmakefile.OBPRMdefaults

#common misc.
DEFS = $(DEBUG_DEF) $(CD_DEF) -DCHECKIFSAMECC=$(CHECKIFSAMECC) -DVERBOSE=0 $(STAPLFLAGS) $(DEPS)
INCLUDES = -I. $(STAPL_INCL) $(CD_INCL)
LIBS = $(OBPRMLIB) $(CD_LIB) $(STAPLLIB)

#compile targest
default: libs

libs: objs $(OBPRM_LIBFILE)

libobprm.a: objs
	$(AR) libobprm.a $(OBJS)

libobprm.so: objs
	$(CC) -shared $(OPTS) $(INCLUDES) $(DEFS) $(OBJS) -o libobprm.so

%: main_%.cpp libs
	$(CC) $(OPTS) $(LIBFLAGS) $(INCLUDES) $(DEFS) main_$@.cpp $(LIBS) -o $@
	rm -f $@.d

%.o: %.cpp
	$(CC) $(OPTS) $(INCLUDES) $(DEFS) -c $<
	touch $*.d
	cat $*.d >> Dependencies
	rm -f $*.d

OBJS = \
	Environment.o \
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
	util.o \
	Clock_Class.o \
	Clock_Elapsed.o \
	Stat_Class.o \
	DistanceMetrics.o \
	CollisionDetection.o \
	QueryCmds.o \
	Weight.o
#	Haptic.o \
#	Push.o \

objs: $(OBJS)

clean:
	rm -f obprm
	rm -f query

reallyclean:
	rm -f obprm
	rm -f query
	rm -f *.a *.so *.o
	rm -R -f ii_files
	rm -f so_locations
	rm -f Dependencies
	touch Dependencies


include Dependencies

