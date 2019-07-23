#!/bin/bash

srcDir_raw=/home/garbade/datasets/kitti/odometry_snap4/sequences/00/0.20_256x256x32_dense/
dstDir=/home/garbade/datasets/kitti/odometry_snap4/filelists/filelist_0.20_256x256x32_snap4_dense


echo "Filling ${dstDir}_train.txt"
for i in $(seq -f "%02g" 00 07); \
	do \
	srcDir=$(echo $srcDir_raw | sed -e "s|00|$(echo $i)|g" ); \
	ls ${srcDir}/input/ >> ${dstDir}_train.txt ;\
done

for i in $(seq -f "%02g" 09 10); \
	do \
	srcDir=$(echo $srcDir_raw | sed -e "s|00|$(echo $i)|g" ); \
	ls ${srcDir}/input/ >> ${dstDir}_train.txt ;\
done




echo "Filling ${dstDir}_valid.txt"
for i in {08..08}; \
	do \
	srcDir=$(echo $srcDir_raw | sed -e "s|00|$(echo $i)|g" ); \
	ls ${srcDir}/input/ >> ${dstDir}_valid.txt ;\
done


echo "Filling ${dstDir}_test.txt"
for i in {11..21}; \
	do \
	srcDir=$(echo $srcDir_raw | sed -e "s|00|$(echo $i)|g" ); \
	ls ${srcDir}/input/ >> ${dstDir}_test.txt ;\
done


echo "Done"
