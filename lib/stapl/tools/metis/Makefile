

default:
	(cd Lib ; gmake ) 
	(cd Programs ; gmake ) 
	(cd Test ; gmake ) 

clean:
	(cd Lib ; gmake clean )
	(cd Programs ; gmake clean )
	(cd Test ; gmake clean )
	rm -rf libmetis.a libmetis.a++
	rm -rf pmetis kmetis oemetis onmetis partnmesh partdmesh mesh2nodal mesh2dual graphchk Graphs/mtest ../../lib/libmetis.a

realclean:
	(cd Lib ; make realclean )
	(cd Programs ; make realclean )
	(cd Test ; make realclean )

install: default
	cp libmetis.a ../../lib
