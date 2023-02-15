#
CXX := /usr/lib64/mpich/bin/mpic++

FLAGS := -std=c++11 -O3

INC := -D_STAPL -I/opt/stapl/trunk/./tools/libstdc++/4.8.3 -I/opt/stapl/trunk/tools -I/opt/stapl/trunk -I/usr/local/boost/boost-1.63/include -DBOOST_RESULT_OF_USE_TR1_WITH_DECLTYPE_FALLBACK -fno-color-diagnostics -DSTAPL__GNUC__=4 -DSTAPL__GNUC_MINOR__=8 -DSTAPL__GNUC_PATCHLEVEL__=3 -gcc-toolchain /usr/ -ftemplate-depth=512 -Wno-constexpr-not-const

OPT := -g -std=c++11 -O3

LINK := -g -L/opt/stapl/lib -lstapl_debug -lrt -L/usr/local/boost/boost-1.63/lib64 -lboost_serialization -lboost_thread -lboost_system

MPICH_CLINKER := clang
MPICH_CCC := clang++
MPICH_CXX := clang++
MPICH_CC := clang
CCLINKER := clang++

DATA := medium_data

#

all: 	algo1 algo2 algo3 algo4 algo5 algo6

algo1:	algo1_obj findfirst.exe findif.exe find.exe findend.exe mismatch.exe \
	partpt.exe adjfind.exe search.exe searchn.exe

algo2: 	algo2_obj ispart.exe isperm.exe equal.exe maxelement.exe maxvalue.exe \
	minelement.exe minvalue.exe allanynone.exe count.exe  

algo3:	algo3_obj copy.exe generate.exe replace.exe replcopy.exe fill.exe \
	iota.exe 
#transform.exe

algo4:	algo4_obj keepif.exe remove.exe rmvcopy.exe uniqcopy.exe unique.exe \
	reverse.exe rotate.exe stpart.exe partcp.exe partition.exe randshuf.exe

algo5:	algo5_obj sampsort.exe sort.exe partsort.exe partsortcp.exe \
	radsort.exe binsrch.exe eqrange.exe uplobnd.exe lexcomp.exe \
	issort.exe merge.exe 

algo6:	algo6_obj accum.exe adjdiff.exe partsum.exe inprod.exe \
	wtinprod.exe wtnorm.exe 

#

all_obj:  algo1_obj algo2_obj algo3_obj algo4_obj algo5_obj algo6_obj data_obj

algo1_obj: findfirst.o findif.o find.o findend.o mismatch.o \
	partpt.o adjfind.o search.o searchn.o  

algo2_obj: ispart.o isperm.o equal.o maxelement.o maxvalue.o \
	minelement.o minvalue.o allanynone.o count.o  

algo3_obj: copy.o generate.o replace.o replcopy.o fill.o iota.o 
#transform.o 

algo4_obj: keepif.o remove.o rmvcopy.o uniqcopy.o unique.o reverse.o \
	rotate.o stpart.o partcp.o partition.o randshuf.o 

algo5_obj: sampsort.o sort.o partsort.o partsortcp.o radsort.o binsrch.o \
	eqrange.o uplobnd.o lexcomp.o issort.o merge.o 

algo6_obj: accum.o adjdiff.o partsum.o inprod.o wtinprod.o wtnorm.o 

data_obj: tiny_data.o small_data.o medium_data.o big_data.o huge_data.o

#

%.o: %.cc
	$(CXX) $(INC) $(FLAGS) -c $< -o $@

%.exe:	%.o
	$(CXX) $< $(LINK) -o $@


issort.exe: issort.o
	$(CXX) issort.o $(DATA).o $(LINK) -o $@

radsort.exe: radsort.o
	$(CXX) radsort.o $(DATA).o $(LINK) -o $@

sampsort.exe: sampsort.o
	$(CXX) sampsort.o $(DATA).o $(LINK) -o $@

sort.exe: sort.o
	$(CXX) sort.o $(DATA).o $(LINK) -o $@

uplobnd.exe: uplobnd.o
	$(CXX) uplobnd.o $(DATA).o $(LINK) -o $@
