#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel3.RRT.47 -V
cd ./ZTunnel3/
../pmpl -f ZTunnel3.RRT.47.xml >& ZTunnel3.RRT.47.log
