#!/bin/bash

cd $1

filename=../$1.results
tests=(UniformRandom)

echo $1 > $filename
for t in ${tests[@]}
do
  echo $t >> $filename
  find -name $t.*.stat | xargs ../filter $2 | xargs grep "Number of Nodes" | xargs ../avg Nodes 4 0 >> $filename
  find -name $t.*.stat | xargs ../filter $2 | xargs grep "Total Cfg"       | xargs ../avg CD 4 0 >> $filename
  find -name $t.*.stat | xargs ../filter $2 | xargs grep "RoadmapAvgClr" | xargs ../avg RoadmapAvgClr 2 0 >> $filename
  find -name $t.*.stat | xargs ../filter $2 | xargs grep "RoadmapMinClr" | xargs ../avg RoadmapMinClr 2 0 >> $filename
  find -name $t.*.stat | xargs ../filter $2 | xargs grep "RoadmapVarClr" | xargs ../avg RoadmapVarClr 2 0 >> $filename
  find -name $t.*.stat | xargs ../filter $2 | xargs grep "PathAvgClr" | xargs ../avg PathAvgClr 2 0 >> $filename
  find -name $t.*.stat | xargs ../filter $2 | xargs grep "PathMinClr" | xargs ../avg PathMinClr 2 0 >> $filename
  find -name $t.*.stat | xargs ../filter $2 | xargs grep "PathVarClr" | xargs ../avg PathVarClr 2 0 >> $filename
  #find -name $t.*.stat | xargs ../filter $2 | xargs grep "Map Generation:" | xargs ../avg Time 4 1 >> $filename
  find -name $t.*.map | xargs ../parsemap
  find -name $t.*.coords | xargs ../dist *.env.dist
  find -name $t.*.dist | xargs grep "standard deviation:" | xargs ../avg Uniformity 5 0 >> $filename
done

