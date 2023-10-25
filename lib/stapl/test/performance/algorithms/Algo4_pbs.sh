#PBS -q batch
#PBS -l mppnppn=32
#PBS -N algo4_32
#PBS -j oe
#PBS -l mppnppn=32
#PBS -l mppwidth=SED
NP=SED
EXEC='/usr/bn/aprun'
export STAPL_NUM_THREADS=1
MODEL=small

export PATH=/mnt/lustre/lus0/rmetzger/Perform
cd $PATH

$EXEC -n $NP $PATH/keepif.exe -data $MODEL
$EXEC -n $NP $PATH/remove.exe -data $MODEL
$EXEC -n $NP $PATH/rmvcopy.exe -data $MODEL
$EXEC -n $NP $PATH/uniqcopy.exe -data $MODEL
$EXEC -n $NP $PATH/unique.exe -data $MODEL
$EXEC -n $NP $PATH/reverse.exe -data $MODEL
$EXEC -n $NP $PATH/rotate.exe -data $MODEL
$EXEC -n $NP $PATH/stpart.exe -data $MODEL
$EXEC -n $NP $PATH/partcp.exe -data $MODEL
$EXEC -n $NP $PATH/partition.exe -data $MODEL
$EXEC -n $NP $PATH/randshuf.exe -data $MODEL

##gnuplot -e "load algo4_plots"
