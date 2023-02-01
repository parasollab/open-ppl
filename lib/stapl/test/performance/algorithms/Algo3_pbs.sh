#PBS -q batch
#PBS -N algo3_32
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
$EXEC -n $NP $PATH/copy.exe -data $MODEL
$EXEC -n $NP $PATH/generate.exe -data $MODEL
$EXEC -n $NP $PATH/replace.exe -data $MODEL
$EXEC -n $NP $PATH/replcopy.exe -data $MODEL
$EXEC -n $NP $PATH/fill.exe -data $MODEL
$EXEC -n $NP $PATH/iota.exe -data $MODEL
else
$EXEC -n $NP $PATH/foreach.exe -data $MODEL
$EXEC -n $NP $PATH/transform.exe -data $MODEL
fi

##gnuplot -e "load algo3_plots"
