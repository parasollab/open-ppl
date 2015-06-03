#!/bin/bash

#Each line of the input file encodes a test xml
while read p; do
  #first token is the test xml
  tokens=($p)
  input=${tokens[0]}
  base=`basename $input .xml`
  #ensure the program runs successfully
  if mpirun -np 4 ../pmpl -f XMLs/$input >& Scratch/$base.pmpl; then
    #Every other token is a file to actually diff. If no files are stated, then
    #report suceess
    num=`expr ${#tokens[@]} - 1`
    if [ $num == 0 ]; then
      echo "Success: Test $input"
      rm Scratch/$base*
    else
      #Diff each file and report error if output is not null
      for i in `seq 1 $num`; do
        file=${tokens[$i]}
        diffOut=$(diff Scratch/$file Outputs/$file)
        if [ "$diffOut" == "" ]; then
          echo "Success: Test $input"
          rm Scratch/$base*
        else
          echo "Error: Test $input failed to match correct output"
          printf "Diff: \n\t $diffOut\n"
        fi
      done
    fi
  else
    echo "Error: Test $input failed to run"
  fi
done <$1
