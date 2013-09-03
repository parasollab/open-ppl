#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.g2b2.22 -V
cd ./Hook/
../pmpl -f Hook.g2b2.22.xml >& Hook.g2b2.22.log
