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

all: 	cont 

cont:	cont_obj ct_array.exe ct_vector.exe ct_list.exe ct_starray.exe \
	ct_matrix.exe ct_multi.exe ct_set.exe ct_unordset.exe \
	ct_map.exe ct_unordmap.exe ct_dygraf.exe ct_stgraf.exe

#

all_obj:  cont_obj data_obj

cont_obj: ct_array.o ct_vector.o ct_list.o ct_starray.o ct_matrix.o \
	ct_multi.o ct_set.o ct_unordset.o ct_map.o ct_unordmap.o  \
	ct_dygraf.o ct_stgraf.o

data_obj: tiny_data.o small_data.o medium_data.o big_data.o huge_data.o

#

%.o: %.cc
	$(CXX) $(INC) $(FLAGS) -c $< -o $@

%.exe:	%.o
	$(CXX) $< $(LINK) -o $@

ct_list.exe: ct_list.o
	$(CXX) ct_list.o $(DATA).o $(LINK) -o $@

ct_set.exe: ct_set.o
	$(CXX) ct_set.o $(DATA).o $(LINK) -o $@

ct_unordset.exe: ct_unordset.o
	$(CXX) ct_unordset.o $(DATA).o $(LINK) -o $@

ct_map.exe: ct_map.o
	$(CXX) ct_map.o $(DATA).o $(LINK) -o $@

ct_unordmap.exe: ct_unordmap.o
	$(CXX) ct_unordmap.o $(DATA).o $(LINK) -o $@

ct_dygraf.exe: ct_dygraf.o
	$(CXX) ct_dygraf.o $(DATA).o $(LINK) -o $@

ct_stgraf.exe: ct_stgraf.o
	$(CXX) ct_stgraf.o $(DATA).o $(LINK) -o $@

