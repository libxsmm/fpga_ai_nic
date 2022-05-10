###############################################################################
# Copyright (c) Intel Corporation - All rights reserved.                      #
# This file is part of the LIBXSMM library.                                   #
#                                                                             #
# For information on the license, see the LICENSE file.                       #
# Further information: https://github.com/libxsmm/libxsmm/                    #
# SPDX-License-Identifier: BSD-3-Clause                                       #
###############################################################################
# Rui Ma (Intel Corp.)
###############################################################################
export OMP_NUM_THREADS=28
export KMP_AFFINITY=granularity=fine,compact,1,0
export I_MPI_DEBUG=4
#export LIBXSMM_VERBOSE=-1
mpirun -f hostlist -n 3 -ppn 1 numactl --interleave=all ./mlp_mpi_example_f32 3 20 5376 0 A 32 32 32 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048
#for node in 6
#do
#	ln -f -s ./config/ikl_config_n6 ikl_config
#	iko breset
#	ln -f -s ./config/hostlist_n${node} hostlist
#	ln -f -s ./config/ikl_config_n${node} ikl_config
#	#ln -f -s ./sw/mlp_mpi_example_f32_n${node} mlp_mpi_example_f32
#	for mb in 1792 448
#	do
#		mode="bfp_${mb}"
#		MB=$(($node*$mb))
#		echo "mpirun -f hostlist -n $node -ppn 1 numactl --interleave=all ./mlp_mpi_example_f32 $node 20 $MB 0 A 32 32 32 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 > ./log/${node}_${mode}.log"
#		mpirun -f hostlist -n $node -ppn 1 numactl --interleave=all ./mlp_mpi_example_f32 $node 20 $MB 0 A 32 32 32 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 > ./log/${node}_${mode}.log
#		for iter in 0 1 2 3
#		do
#			echo "mpirun -f hostlist -n $node -ppn 1 numactl --interleave=all ./mlp_mpi_example_f32 $node 20 $MB 0 A 32 32 32 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 >> ./log/${node}_${mode}.log"
#			mpirun -f hostlist -n $node -ppn 1 numactl --interleave=all ./mlp_mpi_example_f32 $node 20 $MB 0 A 32 32 32 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 >> ./log/${node}_${mode}.log
#		done
#	done
#done

