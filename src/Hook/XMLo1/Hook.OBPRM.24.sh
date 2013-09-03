#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.OBPRM.24 -V
cd ./Hook/
../pmpl -f Hook.OBPRM.24.xml >& Hook.OBPRM.24.log
