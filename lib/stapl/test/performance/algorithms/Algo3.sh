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
MODEL=small
OK=1

if [ "$OK" -eq 1 ]
then
$EXEC -n $NP ./copy.exe     -data $MODEL
$EXEC -n $NP ./replace.exe  -data $MODEL 
$EXEC -n $NP ./replcopy.exe -data $MODEL 
$EXEC -n $NP ./fill.exe     -data $MODEL
$EXEC -n $NP ./iota.exe     -data $MODEL 
else
$EXEC -n $NP ./generate.exe -data $MODEL # random_sequence extremely slow
$EXEC -n $NP ./foreach.exe  -data $MODEL
$EXEC -n $NP ./transform.exe -data $MODEL
fi

##gnuplot -e "load algo3_plots"
