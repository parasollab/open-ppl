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
date

$EXEC -n $NP ./inprod.exe   -data $MODEL
$EXEC -n $NP ./accum.exe    -data $MODEL
$EXEC -n $NP ./adjdiff.exe  -data $MODEL
$EXEC -n $NP ./partsum.exe  -data $MODEL
$EXEC -n $NP ./wtinprod.exe -data $MODEL
$EXEC -n $NP ./wtnorm.exe   -data $MODEL

##gnuplot -e "load algo6_plots"
