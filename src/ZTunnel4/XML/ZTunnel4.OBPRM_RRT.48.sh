#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel4.OBPRM_RRT.48 -V
cd ./ZTunnel4/
../pmpl -f ZTunnel4.OBPRM_RRT.48.xml >& ZTunnel4.OBPRM_RRT.48.log
