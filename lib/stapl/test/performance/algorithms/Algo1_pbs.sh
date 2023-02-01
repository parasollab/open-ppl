#PBS -q batch
#PBS -N algo1_32
#PBS -j oe
#PBS -l mppnppn=32
#PBS -l mppwidth=SED
NP=SED
EXEC='/usr/bin/aprun'
export STAPL_NUM_THREADS=1
MODEL=small
OK=1

export PATH=/mnt/lustre/lus0/rmetzger/Perform
cd $PATH

if [ "$OK" -eq 1 ]
then
$EXEC -n $NP $PATH/findif.exe -data $MODEL
$EXEC -n $NP $PATH/find.exe -data $MODEL
$EXEC -n $NP $PATH/mismatch.exe -data $MODEL
$EXEC -n $NP $PATH/partpt.exe -data $MODEL
$EXEC -n $NP $PATH/adjfind.exe -data $MODEL
$EXEC -n $NP $PATH/search.exe -data $MODEL
$EXEC -n $NP $PATH/searchn.exe -data $MODEL
else
$EXEC -n $NP $PATH/findfirst.exe -data $MODEL
$EXEC -n $NP $PATH/findend.exe -data $MODEL
fi

##gnuplot -e "load algo1_plots"

