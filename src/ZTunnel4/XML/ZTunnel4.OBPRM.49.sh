#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel4.OBPRM.49 -V
cd ./ZTunnel4/
../pmpl -f ZTunnel4.OBPRM.49.xml >& ZTunnel4.OBPRM.49.log
