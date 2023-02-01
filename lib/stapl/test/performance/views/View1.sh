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
$EXEC -n $NP ./vw_array.exe    -data $MODEL 
$EXEC -n $NP ./vw_vector.exe   -data $MODEL 
$EXEC -n $NP ./vw_arrayro.exe  -data $MODEL 
$EXEC -n $NP ./vw_map.exe      -data $MODEL
$EXEC -n $NP ./vw_set.exe      -data $MODEL
$EXEC -n $NP ./vw_graph.exe    -data $MODEL
else
$EXEC -n $NP ./vw_multi.exe    -data $MODEL
$EXEC -n $NP ./vw_list.exe     -data $MODEL
fi

##gnuplot -e "load view1_plots"
