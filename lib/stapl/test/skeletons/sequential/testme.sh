#!/bin/bash

# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

eval ./stl_stencil 256
if test $? != 0
then
  echo "ERROR:: while testing stl_stencil"
fi
eval ./stl_stencil_2d_2p 16
if test $? != 0
then
  echo "ERROR:: while testing stl_stencil_2d_2p"
fi
