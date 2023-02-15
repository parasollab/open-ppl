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
$EXEC -n $NP ./findif.exe    -data $MODEL 
$EXEC -n $NP ./find.exe      -data $MODEL 
$EXEC -n $NP ./mismatch.exe  -data $MODEL 
$EXEC -n $NP ./partpt.exe    -data $MODEL 
$EXEC -n $NP ./adjfind.exe   -data $MODEL 
$EXEC -n $NP ./search.exe    -data $MODEL 
$EXEC -n $NP ./searchn.exe   -data $MODEL
else
$EXEC -n $NP ./findfirst.exe -data small # extremely slow
$EXEC -n $NP ./findend.exe   -data small # extremely slow
fi

##gnuplot -e "load algo1_plots"

