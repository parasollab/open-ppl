#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel4.Uniform_RRT.35 -V
cd ./ZTunnel4/
../pmpl -f ZTunnel4.Uniform_RRT.35.xml >& ZTunnel4.Uniform_RRT.35.log
