#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel4.RRT.40 -V
cd ./ZTunnel4/
../pmpl -f ZTunnel4.RRT.40.xml >& ZTunnel4.RRT.40.log
