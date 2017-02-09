#!/bin/bash

executable="../pmpl"
if [ ! -z "$2" ]  && [ $2 == "1" ]; then #&& $2 eq "1" ]; then
  executable="mpirun -np 4 ../pmpl"
fi
echo "Executable: ${executable}"

#Each line of the input file encodes a test xml
while read p; do
  #first token is the test xml
  tokens=($p)
  input=${tokens[0]}
  base=`basename ${input} .xml`
  #ensure the program runs successfully
  if ${executable} -f XMLs/${input} >& Scratch/${base}.pmpl; then
    #Every other token is a file to actually diff. If no files are stated, then
    #report suceess
    num=`expr ${#tokens[@]} - 1`
    if [ ${num} == 0 ]; then
      echo "Success: Test ${input}"
      rm Scratch/${base}*
    else
      # Diff each file and report error if output is not null.
      pass='true'
      for i in `seq 1 ${num}`; do
        # First check that this output file was generated.
        file=${tokens[$i]}
        if  [ ! -f Scratch/${file} ] ; then
          # If expected output file isn't generated, test failed.
          pass='false'
          echo "Error: Test ${input} failed to generate expected output ${file}"
          continue
        fi

        # Next, check that the generated file matches the gold standard in Outputs.
        diffOut=$(diff Scratch/${file} Outputs/${file})
        if [ "${diffOut}" == "" ]; then
          # Diff is empty so test passed.
          echo "Success: Test ${input} on output ${file}"
        else
          # Diff !empty, so test failed.
          pass='false'
          echo "Error: Test ${input} failed to match correct output for ${file}"
        fi
      done
    fi
    # If the test passed, clear the output files.
    if [ "${pass}" == 'true' ] ; then
      rm Scratch/${base}*
    fi
  else
    echo "Error: Test ${input} failed to run"
  fi
done <$1
