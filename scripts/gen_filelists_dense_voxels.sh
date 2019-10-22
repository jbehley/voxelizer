#!/bin/bash

# srcDir_raw=/home/garbade/datasets/kitti/odometry_snap5/voxels/00/target_voxel_gt/
srcDir_raw=/home/garbade/datasets/kitti/odometry_snap5/voxels/00/input_voxel/
dstDir=/home/garbade/datasets/kitti/odometry_snap5/filelists/filelist_voxels_input


echo "Filling ${dstDir}_train.txt"
for i in $(seq -f "%02g" 00 07); \
	do \
	srcDir=$(echo $srcDir_raw | sed -e "s|00|$(echo $i)|g" ); \
	cd ${srcDir};\
	ls *.mat > tmp.list; \
	while read p;do echo ${srcDir}$p >> ${dstDir}_train.txt; done < tmp.list ;\
	rm tmp.list; \
	cd -;\
done

for i in $(seq -f "%02g" 09 10); \
	do \
	srcDir=$(echo $srcDir_raw | sed -e "s|00|$(echo $i)|g" ); \
	cd ${srcDir};\
	ls *.mat > tmp.list; \
	while read p;do echo ${srcDir}$p >> ${dstDir}_train.txt; done < tmp.list ;\
	rm tmp.list; \
	cd -;\
done




echo "Filling ${dstDir}_valid.txt"
for i in {08..08}; \
	do \
	srcDir=$(echo $srcDir_raw | sed -e "s|00|$(echo $i)|g" ); \
	cd ${srcDir};\
	ls *.mat > tmp.list; \
	while read p;do echo ${srcDir}$p >> ${dstDir}_valid.txt; done < tmp.list ;\
	rm tmp.list; \
	cd -;\
done


echo "Filling ${dstDir}_test.txt"
for i in {11..21}; \
	do \
	srcDir=$(echo $srcDir_raw | sed -e "s|00|$(echo $i)|g" ); \
	cd ${srcDir};\
	ls *.mat > tmp.list; \
	while read p;do echo ${srcDir}$p >> ${dstDir}_test.txt; done < tmp.list ;\
	rm tmp.list; \
	cd -;\
done


echo "Done"


