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

module bf16_to_bfp_core #(
    parameter NUM = 10,
    parameter EXPONENT_SIZE = 8,
    parameter MANTISSA_SIZE = 8,
    parameter NX_MODE = 1
)(
    input clk,
    
    input sign_in[0:NUM-1],
    input [EXPONENT_SIZE-1:0] exponent_in[0:NUM-1],
    input [MANTISSA_SIZE-1:0] mantissa_in[0:NUM-1],
    
    output [EXPONENT_SIZE-1:0] shared_exponent_out,
    output signed [MANTISSA_SIZE-1:0] sdata_out[0:NUM-1]
);

wire sign_out[0:NUM-1];
reg [MANTISSA_SIZE-1:0] mantissa_out[0:NUM-1];

localparam LOG_SIZE = $clog2(MANTISSA_SIZE);
localparam SHIFT_SIZE = (EXPONENT_SIZE > LOG_SIZE) ? LOG_SIZE : EXPONENT_SIZE;
localparam SHIFT_LATENCY = (SHIFT_SIZE + 1) / 2;
localparam MAX_LATENCY = (NUM <= 4) ? 2 : (
                                (NUM <= 12) ? 4 : (
                                  (NUM <= 36) ? 6 : (
                                    (NUM <= 108) ? 8 : 10
                                  )
                                )
                              );
                                                  
localparam SHARED_EXPONENT_LATENCY = SHIFT_LATENCY + 2;
localparam LATENCY = MAX_LATENCY + SHIFT_LATENCY + 2;

wire [EXPONENT_SIZE-1:0] exponent_max;

max_u #(.NUM(NUM), .SIZE(EXPONENT_SIZE)) max_inst(
    .clk(clk),
    .din(exponent_in),
    .dout(exponent_max)
);

reg [EXPONENT_SIZE-1:0] exponent_r[0:MAX_LATENCY-1][0:NUM-1];
reg sign_r[0:LATENCY-1][0:NUM-1];
reg [EXPONENT_SIZE-1:0] exponent_diff[0:NUM-1];
reg [EXPONENT_SIZE-1:0] shared_exponent_r[0:SHARED_EXPONENT_LATENCY-1];
reg [MANTISSA_SIZE-1:0] mantissa_r[0:MAX_LATENCY][0:NUM-1];
reg [MANTISSA_SIZE:0] sdata[0:NUM-1];

wire [MANTISSA_SIZE-1:0] mantissa_out_w[0:NUM-1];

integer i;
always @(posedge clk) begin

    sign_r[0] <= sign_in;
    for (i = 1; i < LATENCY; i=i+1) begin
        sign_r[i] <= sign_r[i-1];
    end

    mantissa_r[0] <= mantissa_in;
    for (i = 1; i < MAX_LATENCY+1; i=i+1) begin
        mantissa_r[i] <= mantissa_r[i-1];
    end
    
    exponent_r[0] <= exponent_in;
    for (i = 1; i < MAX_LATENCY; i=i+1) begin
        exponent_r[i] <= exponent_r[i-1];
    end
    for (i = 0; i < NUM; i=i+1) begin
        exponent_diff[i] <= exponent_max - exponent_r[MAX_LATENCY-1][i];
    end
    if (NX_MODE)
        shared_exponent_r[0] <= exponent_max - 6;
    else 
        shared_exponent_r[0] <= exponent_max;
    
    for (i = 1; i < SHARED_EXPONENT_LATENCY; i=i+1)
        shared_exponent_r[i] <= shared_exponent_r[i-1];        

    for (i = 0; i < NUM; i=i+1) begin
        sdata[i] <= ({1'b0, mantissa_out_w[i]} ^ {(MANTISSA_SIZE+1) {sign_r[LATENCY-2][i]}}) + sign_r[LATENCY-2][i];
    end
    
    mantissa_out <= mantissa_out_w;
end

genvar index;
generate
for (index = 0; index < NUM; index=index+1) begin : shifter
    barrel_shifter #(.SIZE(MANTISSA_SIZE), .SHIFT_SIZE(EXPONENT_SIZE), .SHIFT_LEFT(0)) shifter (
        .clk(clk),
        .din(mantissa_r[MAX_LATENCY][index]),
        .shift(exponent_diff[index]),
        .dout(mantissa_out_w[index])
    );
    
    assign sdata_out[index] = sdata[index][MANTISSA_SIZE:1];
end
endgenerate

assign sign_out = sign_r[LATENCY-1];
assign shared_exponent_out = shared_exponent_r[SHARED_EXPONENT_LATENCY-1];

endmodule
