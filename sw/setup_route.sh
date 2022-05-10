#!/bin/bash
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
iko setup_sled

if [ $1 = 3 ]
then
	iko setup_chan f0 p0 f1 p1
	iko setup_chan f1 p0 f2 p1
	iko setup_chan f2 p0 f0 p1
elif [ $1 = 4 ]
then
	iko setup_chan f0 p0 f1 p1
	iko setup_chan f1 p0 f2 p1
	iko setup_chan f2 p0 f3 p1
	iko setup_chan f3 p0 f0 p1
elif [ $1 = 5 ]
then
	iko setup_chan f0 p0 f1 p1
	iko setup_chan f1 p0 f2 p1
	iko setup_chan f2 p0 f3 p1
	iko setup_chan f3 p0 f4 p1
	iko setup_chan f4 p0 f0 p1
elif [ $1 = 6 ]
then
	iko setup_chan f0 p0 f1 p1
	iko setup_chan f1 p0 f2 p1
	iko setup_chan f2 p0 f3 p1
	iko setup_chan f3 p0 f4 p1
	iko setup_chan f4 p0 f5 p1
	iko setup_chan f5 p0 f0 p1
fi