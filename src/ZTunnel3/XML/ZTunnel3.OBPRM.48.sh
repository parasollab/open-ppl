#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel3.OBPRM.48 -V
cd ./ZTunnel3/
../pmpl -f ZTunnel3.OBPRM.48.xml >& ZTunnel3.OBPRM.48.log
