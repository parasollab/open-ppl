#PBS -q batch
#PBS -N algo6_32
#PBS -j oe
#PBS -l mppnppn=32
#PBS -l mppwidth=SED
NP=SED
EXEC='usr/bin/aprun'
export STAPL_NUM_THREADS=1
MODEL=medium
date

export PATH=/mnt/lustre/lus0/rmetzger/Perform
cd $PATH

$EXEC -n $NP $PATH/accum.exe -data $MODEL
$EXEC -n $NP $PATH/adjdiff.exe -data $MODEL
$EXEC -n $NP $PATH/partsum.exe -data $MODEL
$EXEC -n $NP $PATH/inprod.exe -data $MODEL
$EXEC -n $NP $PATH/wtinprod.exe -data $MODEL
$EXEC -n $NP $PATH/wtnorm.exe -data $MODEL

##gnuplot -e "load algo6_plots"
