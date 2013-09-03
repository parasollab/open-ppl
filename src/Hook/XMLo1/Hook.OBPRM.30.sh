#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.OBPRM.30 -V
cd ./Hook/
../pmpl -f Hook.OBPRM.30.xml >& Hook.OBPRM.30.log
