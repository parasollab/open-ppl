#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.OBPRM.6 -V
cd ./Hook/
../pmpl -f Hook.OBPRM.6.xml >& Hook.OBPRM.6.log
