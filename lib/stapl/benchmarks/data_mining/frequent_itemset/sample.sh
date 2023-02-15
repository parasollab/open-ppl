#

export STAPL_NUM_THREADS=1

rm -f freqmine.dbg sample.out
mpiexec -n 2 ./freqmine.exe sample.meta sample.txt sample.out t 0.05

