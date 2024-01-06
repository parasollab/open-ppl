#PBS -q batch
#PBS -N view2_32
#PBS -j oe
#PBS -l mppnppn=32
#PBS -l mppwidth=SED
NP=SED
EXEC='/usr/bin/aprun'
export STAPL_NUM_THREADS=1
MODEL=medium
OK=1

export PATH=/mnt/lustre/lus0/rmetzger/Perform
cd $PATH

if [ "$OK" -eq 1 ]
then
$EXEC -n $NP $PATH/vw_counting.exe -data $MODEL
$EXEC -n $NP $PATH/vw_reverse.exe -data $MODEL
$EXEC -n $NP $PATH/vw_repeat.exe -data $MODEL
$EXEC -n $NP $PATH/vw_strided.exe -data $MODEL
$EXEC -n $NP $PATH/vw_overlap.exe -data $MODEL
$EXEC -n $NP $PATH/vw_native.exe -data $MODEL
else
$EXEC -n $NP $PATH/vw_segment.exe -data $MODEL
$EXEC -n $NP $PATH/vw_filter.exe -data $MODEL
fi

##gnuplot -e "load view2_plots"
