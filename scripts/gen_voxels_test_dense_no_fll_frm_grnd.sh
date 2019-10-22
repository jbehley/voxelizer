#!/bin/bash


# Execute this from within the "./scripts" folder


line1="#!/bin/bash"
line2="#block(name=xtrct_00, threads=1, memory=2000, gpus=0, hours=72)"


voxelizer_root=/home/garbade/catkin/14_JensVoxelizer128/src/voxelizer/
voxelizer_binary=$voxelizer_root"bin/gen_data "
config_file=$voxelizer_root"scripts/config_dense.cfg "
qsubDir=snap5_qsubs_test_dense_no_fll_frm_grnd
mkdir $qsubDir

seqDirRaw=/home/garbade/datasets/kitti/odometry_snap4/sequences/00/
inLabelDirRaw=/home/garbade/datasets/kitti/odometry_snap5/test_labels/sequences/00/
outVoxelDirRaw=/home/garbade/datasets/kitti/odometry_snap5/voxels_no_fll_frm_grnd/00/


for i in $(seq -f "%02g" 11 21); \
	do \
	seqDir=$(echo $seqDirRaw | sed -e "s|00|$(echo $i)|g" ); \
	inLabelDir=$(echo $inLabelDirRaw | sed -e "s|00|$(echo $i)|g" ); \
	outVoxelDir=$(echo $outVoxelDirRaw | sed -e "s|00|$(echo $i)|g" ); \
	line2_mod=$(echo $line2 | sed -e "s|00|$(echo $i)|g" ); \
	echo -e $line1 '\n' >> ${qsubDir}/qsub_${i}.sh ;\
	echo -e $line2_mod '\n' >> ${qsubDir}/qsub_${i}.sh ;\
	echo -e $voxelizer_binary $config_file $seqDir $inLabelDir $outVoxelDir '\n' >> ${qsubDir}/qsub_${i}.sh ;\
done


