#!/bin/bash
# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

rm -f $1.dot
echo "digraph outgraph {" > $1.dot
echo "node [shape=record];" >> $1.dot
for f in $1.*.dot
do
  cat $f | grep -vE "subgraph|^}$|^label=" >> $1.dot
done

echo "}" >> $1.dot

dot -Tpdf $1.dot > $1.pdf
#rm $1.dot

