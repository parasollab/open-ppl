#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel3.RRT.5 -V
cd ./ZTunnel3/
../pmpl -f ZTunnel3.RRT.5.xml >& ZTunnel3.RRT.5.log
