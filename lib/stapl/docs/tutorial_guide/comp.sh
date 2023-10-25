#
###='-I/opt/stapl/tools/libstdc++/4.8.2 -I/opt/stapl/tools -I/opt/stapl -I/usr/local/boost/boost-1.63/include'
###=-L/opt/stapl/lib -lstapl -lrt -L/usr/local/boost/boost-1.63/lib64 -lboost_serialization -lboost_system'
#INC='-DSTAPL_NDEBUG -I/opt/stapl/tools/libstdc++/4.8.2 -I/opt/stapl/tools -I/opt/stapl/include -I/opt/stapl -I/usr/local/boost/boost-1.63/include'
#LINK='-L/opt/stapl/lib -lstapl -lrt -lboost_serialization -lboost_system'
#COMP='/usr/lib64/openmpi/bin/mpic++'
#OPT='-D_STAPL -std=c++11 -O3'

INC='-D_STAPL -I/users/rmetzger/stapl/trunk/./tools/libstdc++/4.8.3 -I/users/rmetzger/stapl/trunk/tools -I/users/rmetzger/stapl/trunk -I/usr/local/boost/boost-1.63/include -DBOOST_RESULT_OF_USE_TR1_WITH_DECLTYPE_FALLBACK -fno-color-diagnostics -DSTAPL__GNUC__=4 -DSTAPL__GNUC_MINOR__=8 -DSTAPL__GNUC_PATCHLEVEL__=3 -gcc-toolchain /usr/ -ftemplate-depth=512 -Wno-constexpr-not-const'

OPT='-g -std=c++11 -O3' # -DSTAPL_NDEBUG 
LINK='-g -L/opt/stapl/lib -lstapl_debug -lrt -L/usr/local/boost/boost-1.63/lib64 -lboost_serialization -lboost_thread -lboost_system'

COMP=/usr/lib64/mpich/bin/mpic++

export MPICH_CLINKER=clang
export MPICH_CCC=clang++
export MPICH_CXX=clang++
export MPICH_CC=clang
export CCLINKER=clang++

rm -f ex*.o *.exe

$COMP -c $INC $OPT tiny_data.cc
case "$1"
in
200) $COMP -c $INC $OPT ex_200.cc ;;
201) $COMP -c $INC $OPT ex_201.cc ;;
202) $COMP -c $INC $OPT ex_202.cc ;;
203) $COMP -c $INC $OPT ex_203.cc ;;
204) $COMP -c $INC $OPT ex_204.cc ;;
205) $COMP -c $INC $OPT ex_205.cc ;;
206) $COMP -c $INC $OPT ex_206.cc ;;
207) $COMP -c $INC $OPT ex_207.cc ;;
208) $COMP -c $INC $OPT ex_208.cc ;;
209) $COMP -c $INC $OPT ex_209.cc ;;
210) $COMP -c $INC $OPT ex_210.cc ;;
211) $COMP -c $INC $OPT ex_211.cc ;;
212) $COMP -c $INC $OPT ex_212.cc ;;

301) $COMP -c $INC $OPT ex_301.cc ;;
302) $COMP -c $INC $OPT ex_302.cc ;;
303) $COMP -c $INC $OPT ex_303.cc ;;
304) $COMP -c $INC $OPT ex_304.cc ;;
305) $COMP -c $INC $OPT ex_305.cc ;;
306) $COMP -c $INC $OPT ex_306.cc ;;

401) $COMP -c $INC $OPT ex_401.cc ;;
402) $COMP -c $INC $OPT ex_402.cc ;;
403) $COMP -c $INC $OPT ex_403.cc ;;
404) $COMP -c $INC $OPT ex_404.cc ;;
405) $COMP -c $INC $OPT ex_405.cc ;;
406) $COMP -c $INC $OPT ex_406.cc ;;

501) $COMP -c $INC $OPT ex_501.cc ;;
502) $COMP -c $INC $OPT ex_502.cc ;;
503) $COMP -c $INC $OPT ex_503.cc ;;
504) $COMP -c $INC $OPT ex_504.cc ;;
505) $COMP -c $INC $OPT ex_505.cc ;;
506) $COMP -c $INC $OPT ex_506.cc ;;
507) $COMP -c $INC $OPT ex_507.cc ;;
508) $COMP -c $INC $OPT ex_508.cc ;;

601) $COMP -c $INC $OPT ex_601.cc ;;
602) $COMP -c $INC $OPT ex_602.cc ;;
603) $COMP -c $INC $OPT ex_603.cc ;;
604) $COMP -c $INC $OPT ex_604.cc ;;
605) $COMP -c $INC $OPT ex_605.cc ;;
606) $COMP -c $INC $OPT ex_606.cc ;;
607) $COMP -c $INC $OPT ex_607.cc ;;
esac

$COMP ex*.o tiny_data.o $LINK -o $1.exe
date

