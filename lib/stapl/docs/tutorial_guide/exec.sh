#
EXEC='/usr/lib64/openmpi/bin/mpiexec'

# if you use larger processor counts, you must use larger data sizes
# see the Makefile for details
export PROC=2
export STAPL_NUM_THREADS=1

if [ "$#" -ne 1 ]
then
  echo "Usage: exec N"
  exit 1
fi

rm -f *.zout *.tmp *.out *.txt.*

case "$1"
in
200) $EXEC -n $PROC ./200.exe ;;
201) $EXEC -n $PROC ./201.exe ;;
202) $EXEC -n $PROC ./202.exe ;;
203) $EXEC -n $PROC ./203.exe ;;
204) $EXEC -n $PROC ./204.exe ;;
205) $EXEC -n $PROC ./205.exe ;;
206) $EXEC -n $PROC ./206.exe ;;
207) $EXEC -n $PROC ./207.exe ;;
208) $EXEC -n $PROC ./208.exe ;;
209) $EXEC -n $PROC ./209.exe ;;
210) $EXEC -n $PROC ./210.exe ;;
211) $EXEC -n $PROC ./211.exe ;;
212) $EXEC -n $PROC ./212.exe ;;

301) $EXEC -n $PROC ./301.exe ;;
302) $EXEC -n $PROC ./302.exe ;;
303) $EXEC -n $PROC ./303.exe ;;
304) $EXEC -n $PROC ./304.exe ;;
305) $EXEC -n $PROC ./305.exe ;;
306) $EXEC -n $PROC ./306.exe ;;

401) $EXEC -n $PROC ./401.exe ;;
402) $EXEC -n $PROC ./402.exe ;;
403) $EXEC -n $PROC ./403.exe ;;
404) $EXEC -n $PROC ./404.exe ;;
405) $EXEC -n $PROC ./405.exe ;;
406) $EXEC -n $PROC ./406.exe ;;

501) $EXEC -n $PROC ./501.exe ;;
502) $EXEC -n $PROC ./502.exe ;;
503) $EXEC -n $PROC ./503.exe ;;
504) $EXEC -n $PROC ./504.exe ;;
505) $EXEC -n $PROC ./505.exe ;;
506) $EXEC -n $PROC ./506.exe ;;
507) $EXEC -n $PROC ./507.exe ;;
508) $EXEC -n $PROC ./508.exe ;;

601) $EXEC -n $PROC ./601.exe ;;
602) $EXEC -n $PROC ./602.exe ;;
603) $EXEC -n $PROC ./603.exe ;;
604) $EXEC -n $PROC ./604.exe ;;
605) $EXEC -n $PROC ./605.exe ;;
606) $EXEC -n $PROC ./606.exe ;;
607) $EXEC -n $PROC ./607.exe ;;

esac

