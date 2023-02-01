#PBS -q batch
#PBS -N cont_32
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
$EXEC -n $NP $PATH/ct_array.exe -data $MODEL 
$EXEC -n $NP $PATH/ct_vector.exe -data $MODEL 
$EXEC -n $NP $PATH/ct_starray.exe -data $MODEL 
$EXEC -n $NP $PATH/ct_set.exe -data $MODEL
$EXEC -n $NP $PATH/ct_unordset.exe -data $MODEL 
$EXEC -n $NP $PATH/ct_map.exe -data $MODEL 
$EXEC -n $NP $PATH/ct_unordmap.exe -data $MODEL 
else
$EXEC -n $NP $PATH/ct_dygraf.exe -data small
$EXEC -n $NP $PATH/ct_stgraf.exe -data small
$EXEC -n $NP $PATH/ct_multi.exe -data $MODEL
$EXEC -n $NP $PATH/ct_list.exe -data $MODEL 
fi

##gnuplot -e "load cont_plots"
