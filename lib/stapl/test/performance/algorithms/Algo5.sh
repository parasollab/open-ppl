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
date

if [ "$OK" -eq 1 ]
then
$EXEC -n $NP ./sampsort.exe   -data $MODEL
$EXEC -n $NP ./sort.exe       -data $MODEL
$EXEC -n $NP ./partsortcp.exe -data $MODEL
$EXEC -n $NP ./radsort.exe    -data $MODEL
$EXEC -n $NP ./binsrch.exe    -data $MODEL
$EXEC -n $NP ./eqrange.exe    -data $MODEL
$EXEC -n $NP ./uplobnd.exe    -data $MODEL
$EXEC -n $NP ./lexcomp.exe    -data $MODEL
$EXEC -n $NP ./issort.exe     -data $MODEL
$EXEC -n $NP ./merge.exe      -data $MODEL
$EXEC -n $NP ./partsort.exe   -data $MODEL
else
$EXEC -n $NP ./npart.exe     -data $MODEL
fi

##gnuplot -e "load algo5_plots"
