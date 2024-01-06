#PBS -q batch
#PBS -N algo2_32
#PBS -j oe
#PBS -l mppnppn=32
#PBS -l mppwidth=SED
NP=SED
EXEC='/usr/bin/aprun'
export STAPL_NUM_THREADS=1
MODEL=medium

export PATH=/mnt/lustre/lus0/rmetzger/Perform
cd $PATH

$EXEC -n $NP $PATH/ispart.exe -data $MODEL 
$EXEC -n $NP $PATH/equal.exe -data $MODEL 
$EXEC -n $NP $PATH/maxelement.exe -data $MODEL 
$EXEC -n $NP $PATH/maxvalue.exe -data $MODEL 
$EXEC -n $NP $PATH/minelement.exe -data $MODEL 
$EXEC -n $NP $PATH/minvalue.exe -data $MODEL 
$EXEC -n $NP $PATH/allanynone.exe -data $MODEL 
$EXEC -n $NP $PATH/count.exe -data $MODEL 
$EXEC -n $NP $PATH/isperm.exe -data $MODEL

##gnuplot -e "load algo2_plots"
