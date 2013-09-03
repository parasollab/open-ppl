#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.OBPRM.15 -V
cd ./Hook/
../pmpl -f Hook.OBPRM.15.xml >& Hook.OBPRM.15.log
