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
$EXEC -n $NP ./ct_array.exe     -data $MODEL 
$EXEC -n $NP ./ct_vector.exe    -data $MODEL
$EXEC -n $NP ./ct_starray.exe   -data $MODEL
$EXEC -n $NP ./ct_set.exe       -data $MODEL
$EXEC -n $NP ./ct_unordset.exe  -data $MODEL
$EXEC -n $NP ./ct_map.exe       -data $MODEL
$EXEC -n $NP ./ct_unordmap.exe  -data $MODEL
else
$EXEC -n $NP ./ct_dygraf.exe     -data small
$EXEC -n $NP ./ct_stgraf.exe     -data small
$EXEC -n $NP ./ct_multi.exe      -data $MODEL
$EXEC -n $NP ./ct_list.exe      -data $MODEL
fi

##gnuplot -e "load cont_plots"
