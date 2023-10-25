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
$EXEC -n $NP ./keepif.exe    -data $MODEL
$EXEC -n $NP ./remove.exe    -data $MODEL
$EXEC -n $NP ./rmvcopy.exe   -data $MODEL
$EXEC -n $NP ./uniqcopy.exe  -data $MODEL
$EXEC -n $NP ./unique.exe    -data $MODEL
$EXEC -n $NP ./reverse.exe   -data $MODEL
$EXEC -n $NP ./rotate.exe    -data $MODEL
$EXEC -n $NP ./randshuf.exe  -data $MODEL
else
$EXEC -n $NP ./stpart.exe    -data $MODEL # very slow
$EXEC -n $NP ./partcp.exe    -data $MODEL # very slow
$EXEC -n $NP ./partition.exe -data $MODEL # extremely slow
fi

##gnuplot -e "load algo4_plots"
