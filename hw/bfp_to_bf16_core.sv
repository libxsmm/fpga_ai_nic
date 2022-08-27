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

module bfp_to_bf16_core #(
    parameter NUM = 10,
    parameter EXPONENT_SIZE = 8,
    parameter MANTISSA_SIZE = 8,
    parameter NX_MODE = 1,
    parameter FAMILY = "S10" // S10 or A10
)(
    input clk,
        
    input [EXPONENT_SIZE-1:0] shared_exponent_in,
    input signed [MANTISSA_SIZE-1:0] sdata_in[0:NUM-1],

    output sign_out[0:NUM-1],
    output [EXPONENT_SIZE-1:0] exponent_out[0:NUM-1],
    output [MANTISSA_SIZE-1:0] mantissa_out[0:NUM-1]
);

localparam SHIFT_SIZE = $clog2(MANTISSA_SIZE+1);
localparam SHIFT_LATENCY = (SHIFT_SIZE + 1) / 2;
localparam LATENCY = SHIFT_LATENCY + 2;


reg [EXPONENT_SIZE-1:0] exponent_r;
reg [EXPONENT_SIZE-1:0] exponent_rr;

always @(posedge clk) begin
    exponent_r <= shared_exponent_in;
    
    if (NX_MODE)
        exponent_rr <= exponent_r + 7;
    else 
        exponent_rr <= exponent_r + 1;
end

genvar index;
generate

for (index = 0; index < NUM; index=index+1) begin : loop

    reg sign_r[0:LATENCY-1];
    
    reg [EXPONENT_SIZE-1:0] exponent_out_r[0:SHIFT_LATENCY-1];
    
    reg [MANTISSA_SIZE-1:0] sdata_r;
    reg [MANTISSA_SIZE-1:0] sdata_rr;

    reg [SHIFT_SIZE-1:0] zero_count;
    
    integer i;
    always @(posedge clk) begin
        sign_r[0] <= sdata_in[index][MANTISSA_SIZE-1];
        for (i = 1; i < LATENCY; i=i+1) begin
            sign_r[i] <= sign_r[i-1];
        end

        sdata_r <= (sdata_in[index][MANTISSA_SIZE-2:0] ^ {(MANTISSA_SIZE-1){sdata_in[index][MANTISSA_SIZE-1]}}) + sdata_in[index][MANTISSA_SIZE-1];
        sdata_rr <= sdata_r;

        exponent_out_r[0] <= exponent_rr - zero_count;
        
        //$display("%d: %b = %b - %b", index, exponent_rr - zero_count, exponent_rr, zero_count);
        for (i = 1; i < SHIFT_LATENCY; i=i+1) begin
            exponent_out_r[i] <= exponent_out_r[i-1];
        end
    end
    
    if (FAMILY == "S10")
        s10_leading_zero_counter #(.SIZE(MANTISSA_SIZE), .OUT_SIZE(SHIFT_SIZE)) zero_counter (
            .clk(clk),
            .din(sdata_r),
            .dout(zero_count)
        );
    else
        a10_leading_zero_counter #(.SIZE(MANTISSA_SIZE), .OUT_SIZE(SHIFT_SIZE)) zero_counter (
            .clk(clk),
            .din(sdata_r),
            .dout(zero_count)
        );
    
    wire tmp;
    
    barrel_shifter #(.SIZE(MANTISSA_SIZE), .SHIFT_SIZE(SHIFT_SIZE), .SHIFT_LEFT(1)) shifter (
        .clk(clk),
        .din(sdata_rr),
        .shift(zero_count),
        .dout({mantissa_out[index][MANTISSA_SIZE-1:1], tmp})
    );
    assign mantissa_out[index][0] = 0;
    
    assign sign_out[index] = sign_r[LATENCY-1];
    assign exponent_out[index] = exponent_out_r[SHIFT_LATENCY-1];
end

endgenerate

endmodule
