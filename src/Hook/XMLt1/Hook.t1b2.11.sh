#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.t1b2.11 -V
cd ./Hook/
../pmpl -f Hook.t1b2.11.xml >& Hook.t1b2.11.log
