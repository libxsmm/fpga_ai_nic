# ***********************************************************************
# Intel Confidential
# Copyright 2019-2020 Intel Corporation
#
# This software (including Verilog code) and the related documents are
# Intel copyrighted materials, and your use of them is governed by the
# express license under which they were provided to you
# ("License"). Unless the License provides otherwise, you may not use,
# modify, copy, publish, distribute, disclose or transmit this software
# or the related documents without Intel's prior written permission.
#
# This software and the related documents are provided as is, with no
# express or implied warranties, other than those that are expressly
# stated in the License.
#
# Unless otherwise agreed by Intel in writing, you may not remove or
# alter this notice or any other notice embedded in Materials by Intel
# or Intel's suppliers or licensors in any way.
# ***********************************************************************
# The ccip_std_afu.sv file is confidential and missing. Please contact the owner of the repo if you need access to it
all: address_decode
	afu_synth_setup --source filelist.txt build
	\cd build; $(OPAE_PLATFORM_ROOT)/bin/run.sh

address_decode:
	\cp -r ${IKL_FPGA_ROOT}/../hardware/pac_a10_40g/build/ikl/a10_specific/address_decode .
clean:
	\rm -rf build
	\rm -rf address_decode
