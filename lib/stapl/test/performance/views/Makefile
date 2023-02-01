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

all: 	view1 view2 

view1:	view1_obj vw_array.exe vw_vector.exe vw_list.exe vw_arrayro.exe \
	 vw_multi.exe vw_map.exe vw_graph.exe vw_matrix.exe vw_set.exe

view2:	view2_obj vw_counting.exe vw_filter.exe vw_native.exe vw_overlap.exe \
	vw_repeat.exe vw_reverse.exe vw_segment.exe vw_strided.exe


all_obj:  view1_obj view2_obj 

view1_obj: vw_array.o vw_vector.o vw_list.o vw_arrayro.o vw_multi.o \
	vw_map.o vw_graph.o vw_matrix.o vw_set.o

view2_obj: vw_counting.o vw_filter.o vw_native.o vw_overlap.o \
	vw_repeat.o vw_reverse.o vw_segment.o vw_strided.o

data_obj: tiny_data.o small_data.o medium_data.o big_data.o huge_data.o

#

%.o: %.cc
	$(CXX) $(INC) $(FLAGS) -c $< -o $@

%.exe:	%.o
	$(CXX) $< $(LINK) -o $@

vw_list.exe: vw_list.o
	$(CXX) vw_list.o $(DATA).o $(LINK) -o $@

vw_map.exe: vw_map.o
	$(CXX) vw_map.o $(DATA).o $(LINK) -o $@

vw_set.exe: vw_set.o
	$(CXX) vw_set.o $(DATA).o $(LINK) -o $@

