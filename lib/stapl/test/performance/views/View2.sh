#
NP=0
case "$1"
in
2) NP=$1 ;;
4) NP=$1 ;;
8) NP=$1 ;;
16) NP=$1 ;;
esac
if [ "$NP" -eq 0 ]
then
  echo "Invalid or missing parallelism specification: " $1
  exit
fi

EXEC='/usr/lib64/mpich/bin/mpiexec'
export STAPL_NUM_THREADS=1
MODEL=medium
OK=1
date

if [ "$OK" -eq 1 ]
then
$EXEC -n $NP ./vw_counting.exe  -data $MODEL
$EXEC -n $NP ./vw_reverse.exe   -data $MODEL # atom 5x slow
$EXEC -n $NP ./vw_repeat.exe    -data $MODEL
$EXEC -n $NP ./vw_strided.exe   -data $MODEL
$EXEC -n $NP ./vw_overlap.exe   -data $MODEL
$EXEC -n $NP ./vw_native.exe    -data $MODEL
else
$EXEC -n $NP ./vw_segment.exe   -data $MODEL
#$EXEC -n $NP ./vw_filter.exe    -data $MODEL
fi

##gnuplot -e "load view2_plots"
