#!/bin/bash
# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

if [[ "x$1" = "x" ]]; then
  echo -e "Usage: install.sh DESTINATION_DIRECTORY"
  exit 1
fi

cp -R "build" "$1"
chmod -R g+rX,o+rX $1
