/******************************************************************************
* Copyright (c) Intel Corporation - All rights reserved.                      *
* This file is part of the LIBXSMM library.                                   *
*                                                                             *
* For information on the license, see the LICENSE file.                       *
* Further information: https://github.com/libxsmm/libxsmm/                    *
* SPDX-License-Identifier: BSD-3-Clause                                       *
******************************************************************************/
/* Rui Ma (Intel Corp.)
******************************************************************************/

`timescale 1 ps / 1 ps

module sync_regs #(
	parameter WIDTH = 32,
)(
	input clk,
	input [WIDTH-1:0] din,
	output [WIDTH-1:0] dout
);

reg [WIDTH-1:0] din_meta = 0 /* synthesis preserve dont_replicate */
/* synthesis ALTERA_ATTRIBUTE = "-name SDC_STATEMENT \"set_multicycle_path -to [get_keepers *sync_regs*din_meta\[*\]] 2\" " */ ;

reg [WIDTH-1:0] sync_sr = 0 /* synthesis preserve dont_replicate */
/* synthesis ALTERA_ATTRIBUTE = "-name SDC_STATEMENT \"set_false_path -hold -to [get_keepers *sync_regs*din_meta\[*\]]\" " */ ;

always @(posedge clk) begin
	din_meta <= din;
	sync_sr <= din_meta;
end
assign dout = sync_sr;

endmodule
