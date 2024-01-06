#PBS -q batch
#PBS -N view1_32
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
$EXEC -n $NP $PATH/vw_array.exe -data $MODEL
$EXEC -n $NP $PATH/vw_vector.exe -data $MODEL
$EXEC -n $NP $PATH/vw_arrayro.exe -data $MODEL
$EXEC -n $NP $PATH/vw_map.exe -data $MODEL
$EXEC -n $NP $PATH/vw_set.exe -data $MODEL
$EXEC -n $NP $PATH/vw_graph.exe -data $MODEL
else
$EXEC -n $NP $PATH/vw_multi.exe -data $MODEL
$EXEC -n $NP $PATH/vw_list.exe -data $MODEL
fi

##gnuplot -e "load view1_plots"
