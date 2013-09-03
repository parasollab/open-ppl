#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.OBPRM.31 -V
cd ./Hook/
../pmpl -f Hook.OBPRM.31.xml >& Hook.OBPRM.31.log
