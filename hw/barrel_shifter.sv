/******************************************************************************
* Copyright (c) Intel Corporation - All rights reserved.                      *
* This file is part of the LIBXSMM library.                                   *
*                                                                             *
* For information on the license, see the LICENSE file.                       *
* Further information: https://github.com/libxsmm/libxsmm/                    *
* SPDX-License-Identifier: BSD-3-Clause                                       *
******************************************************************************/
// Copyright 2021 Intel Corporation. 
//
// This reference design file is subject licensed to you by the terms and 
// conditions of the applicable License Terms and Conditions for Hardware 
// Reference Designs and/or Design Examples (either as signed by you or 
// found at https://www.altera.com/common/legal/leg-license_agreement.html ).  
//
// As stated in the license, you agree to only use this reference design 
// solely in conjunction with Intel FPGAs or Intel CPLDs.  
//
// THE REFERENCE DESIGN IS PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY OF ANY KIND INCLUDING WARRANTIES OF MERCHANTABILITY, 
// NONINFRINGEMENT, OR FITNESS FOR A PARTICULAR PURPOSE. Intel does not 
// warrant or assume responsibility for the accuracy or completeness of any
// information, links or other items within the Reference Design and any 
// accompanying materials.
//
// In the event that you do not agree with such terms and conditions, do not
// use the reference design file.
/////////////////////////////////////////////////////////////////////////////

// (* altera_attribute = "-name FRACTAL_SYNTHESIS ON" *) 
module barrel_shifter #(
    parameter SIZE = 32,
    parameter SHIFT_SIZE = 8,
    parameter SHIFT_LEFT = 1
) (
    input clk,
    input [SIZE-1:0] din,
    input [SHIFT_SIZE-1:0] shift,
    output [SIZE-1:0] dout
);
genvar i;
generate

wire clear_flag;

localparam LOG_SIZE = $clog2(SIZE);
localparam SHIFT_SIZE1 = (SHIFT_SIZE > LOG_SIZE) ? LOG_SIZE : SHIFT_SIZE;
if (SHIFT_SIZE1 < SHIFT_SIZE)
    assign clear_flag = (shift[SHIFT_SIZE-1:SHIFT_SIZE1] != 0); 
else
    assign clear_flag = 0;
    
wire [SHIFT_SIZE1-1:0] shift1;

assign shift1 = shift[SHIFT_SIZE1-1:0];

if (SHIFT_SIZE1 < 3) begin
    reg [SIZE-1:0] data_r;
    always @(posedge clk) begin
        if (SHIFT_LEFT)
            data_r <= clear_flag ? 0 : (din << shift1);
        else
            data_r <= clear_flag ? 0 : (din >> shift1);
        
    end
    assign dout = data_r;
end else begin
    localparam STAGE_NUM = (SHIFT_SIZE1 + 1) / 2;

    reg [SIZE-1:0] data_r[0:STAGE_NUM-1];
    reg [SHIFT_SIZE1-1:0] shift_r[0:STAGE_NUM-2];

    always @(posedge clk)
    begin
        if (SHIFT_LEFT)
            data_r[0] <= clear_flag ? 0 : (din << shift1[1:0]);
        else
            data_r[0] <= clear_flag ? 0 : (din >> shift1[1:0]);
        
        shift_r[0] <= shift1;
    end
    
    for (i = 1; i < STAGE_NUM-1; i=i+1) begin : stage_loop
        always @(posedge clk) begin
            if (SHIFT_LEFT)
                data_r[i] <= data_r[i-1] << {shift_r[i-1][2*i+1:2*i], {2*i {1'b0}}};
            else
                data_r[i] <= data_r[i-1] >> {shift_r[i-1][2*i+1:2*i], {2*i {1'b0}}};
            shift_r[i] <= shift_r[i-1];
        end
    end

    always @(posedge clk) begin
        if (SHIFT_LEFT)
            data_r[STAGE_NUM-1] <= data_r[STAGE_NUM-2] << {shift_r[STAGE_NUM-2][SHIFT_SIZE1-1:2*(STAGE_NUM-1)], {2*(STAGE_NUM-1) {1'b0}}};
        else
            data_r[STAGE_NUM-1] <= data_r[STAGE_NUM-2] >> {shift_r[STAGE_NUM-2][SHIFT_SIZE1-1:2*(STAGE_NUM-1)], {2*(STAGE_NUM-1) {1'b0}}};
    end
    
    assign dout = data_r[STAGE_NUM-1];
end
endgenerate
endmodule

