#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Uniform.11 -V
cd ./Hook/
../pmpl -f Hook.Uniform.11.xml >& Hook.Uniform.11.log
