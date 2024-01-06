#PBS -q batch
#PBS -N algo5_32
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
$EXEC -n $NP $PATH/sampsort.exe -data $MODEL
$EXEC -n $NP $PATH/sort.exe -data $MODEL
$EXEC -n $NP $PATH/partsortcp.exe -data $MODEL
$EXEC -n $NP $PATH/radsort.exe -data $MODEL
$EXEC -n $NP $PATH/binsrch.exe -data $MODEL
$EXEC -n $NP $PATH/eqrange.exe -data $MODEL
$EXEC -n $NP $PATH/uplobnd.exe -data $MODEL
$EXEC -n $NP $PATH/lexcomp.exe -data $MODEL
$EXEC -n $NP $PATH/issort.exe -data $MODEL
$EXEC -n $NP $PATH/merge.exe -data $MODEL
$EXEC -n $NP $PATH/partsort.exe -data $MODEL
else
$EXEC -n $NP $PATH/npart.exe -data $MODEL
fi

##gnuplot -e "load algo5_plots"
