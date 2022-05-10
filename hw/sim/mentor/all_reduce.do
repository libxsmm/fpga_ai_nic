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
# TOP-LEVEL TEMPLATE - BEGIN
#
# QSYS_SIMDIR is used in the Quartus-generated IP simulation script to
# construct paths to the files required to simulate the IP in your Quartus
# project. By default, the IP script assumes that you are launching the
# simulator from the IP script location. If launching from another
# location, set QSYS_SIMDIR to the output directory you specified when you
# generated the IP script, relative to the directory from which you launch
# the simulator.
#
set QSYS_SIMDIR <project dir>/hw/sim
#
# Source the generated IP simulation script.
source $QSYS_SIMDIR/mentor/msim_setup.tcl
#
# Set any compilation options you require (this is unusual).
#set USER_DEFINED_COMPILE_OPTIONS <compilation options>
#set USER_DEFINED_VHDL_COMPILE_OPTIONS <compilation options for VHDL>
#set USER_DEFINED_VERILOG_COMPILE_OPTIONS=-suppress 12110
#
# Call command to compile the Quartus EDA simulation library.
dev_com
#
# Call command to compile the Quartus-generated IP simulation files.
com
#
# Add commands to compile all design files and testbench files, including
# the top level. (These are all the files required for simulation other
# than the files compiled by the Quartus-generated IP simulation script)
#
vlog -work work -stats=none $QSYS_SIMDIR/../alt_sync_regs_m2.v
vlog -work work -sv -stats=none $QSYS_SIMDIR/../a10_leading_zero_counter.sv
vlog -work work -sv -stats=none $QSYS_SIMDIR/../barrel_shifter.sv
vlog -work work -sv -stats=none $QSYS_SIMDIR/../max_u.sv
vlog -work work -sv -stats=none $QSYS_SIMDIR/../bf16_to_bfp_core.sv
vlog -work work -sv -stats=none $QSYS_SIMDIR/../bfp_to_bf16_core.sv
vlog -work work -sv -stats=none $QSYS_SIMDIR/../bfp_adapter.sv
vlog -work work  -stats=none $QSYS_SIMDIR/../fifo.v
vlog -work work -sv -stats=none $QSYS_SIMDIR/../all_reduce.sv
vlog -work work -sv -stats=none $QSYS_SIMDIR/all_reduce_tb.sv
#
# Set the top-level simulation or testbench module/entity name, which is
# used by the elab command to elaborate the top level.
#
set TOP_LEVEL_NAME all_reduce_tb
#
# Set any elaboration options you require.
#set USER_DEFINED_ELAB_OPTIONS <elaboration options>
#
# Call command to elaborate your design and testbench.
elab_debug
#
# Run the simulation.
add wave *
#add wave -position insertpoint sim:/bfp_adapter_tb/din_fp
add wave -position insertpoint sim:/all_reduce_tb/all_reduce_inst0/*
#add wave -position insertpoint sim:/all_reduce_tb/all_reduce_inst0/bfp_inst/*
view structure
view signals
run -a
#
# Report success to the shell.
#exit -code 0
#
# TOP-LEVEL TEMPLATE - END

