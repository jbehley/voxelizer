#!/bin/bash

line1="#!/bin/bash"
line2="#block(name=xtrct_00, threads=1, memory=2000, gpus=1, hours=72)"

inDir1=/home/garbade/datasets/kitti/odometry_snap4/sequences/00/
outDir1=/home/garbade/datasets/kitti/odometry_snap4/sequences/00/0.20_256x256x32_dense/
cmd_gen_data="./gen_data 0.20_256x256x32_dense.cfg "
qsubDir=qsub_0.20_256x256x32_snap4_dense

mkdir $qsubDir

for i in $(seq -f "%02g" 00 21); \
	do \
	inDir=$(echo $inDir1 | sed -e "s|00|$(echo $i)|g" ); \
	outDir=$(echo $outDir1 | sed -e "s|00|$(echo $i)|g" ); \
	line2_mod=$(echo $line2 | sed -e "s|00|$(echo $i)|g" ); \
	echo -e $line1 '\n' >> ${qsubDir}/qsub_${i}.sh ;\
	echo -e $line2_mod '\n' >> ${qsubDir}/qsub_${i}.sh ;\
	echo -e $cmd_gen_data $inDir $outDir  '\n' >> ${qsubDir}/qsub_${i}.sh ;\
done













