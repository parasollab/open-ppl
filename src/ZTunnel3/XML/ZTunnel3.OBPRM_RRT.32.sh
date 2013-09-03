#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel3.OBPRM_RRT.32 -V
cd ./ZTunnel3/
../pmpl -f ZTunnel3.OBPRM_RRT.32.xml >& ZTunnel3.OBPRM_RRT.32.log
