/******************************************************************************
* Copyright (c) Intel Corporation - All rights reserved.                      *
* This file is part of the LIBXSMM library.                                   *
*                                                                             *
* For information on the license, see the LICENSE file.                       *
* Further information: https://github.com/libxsmm/libxsmm/                    *
* SPDX-License-Identifier: BSD-3-Clause                                       *
******************************************************************************/
// Copyright 2020 Intel Corporation. 
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

module a10_leading_zero_counter #(
    parameter SIZE = 48,
    parameter OUT_SIZE = $clog2(SIZE+1)
) (
    input clk,
    input [SIZE-1:0] din,
    output reg [OUT_SIZE-1:0] dout
);

wire [OUT_SIZE-1:0] dout_w;

genvar i, j;
generate

if (SIZE == 1) begin
    assign dout_w = din[0] ? 0 : 1;
end else if (SIZE == 2) begin
    assign dout_w = din[1] ? 0 : (din[0] ? 1 : 2);
end else if (SIZE == 3) begin
    assign dout_w = din[2] ? 0 : (din[1] ? 1 : (din[0] ? 2 : 3 ) );
end else if (SIZE == 4) begin
    assign dout_w = din[3] ? 0 : (din[2] ? 1 : (din[1] ? 2 : (din[0] ? 3 : 4) ) );
end else if (SIZE == 5) begin
    assign dout_w = din[4] ? 0 : (din[3] ? 1 : (din[2] ? 2 : (din[1] ? 3 : (din[0] ? 4 : 5) ) ) );
end else if (SIZE == 6) begin
    assign dout_w = din[5] ? 0 : (din[4] ? 1 : (din[3] ? 2 : (din[2] ? 3 : (din[1] ? 4 : (din[0] ? 5 : 6) ) ) ) );
end else begin

    localparam [SIZE:0] C[0:5] = {64'b1010101010101010101010101010101010101010101010101010101010101010,
                                  64'b1100110011001100110011001100110011001100110011001100110011001100,
                                  64'b1111000011110000111100001111000011110000111100001111000011110000,
                                  64'b1111111100000000111111110000000011111111000000001111111100000000,
                                  64'b1111111111111111000000000000000011111111111111110000000000000000,
                                  64'b1111111111111111111111111111111100000000000000000000000000000000};

    localparam LOOP_NUM = (SIZE - 6) / 6;

    for (j = 0; j < OUT_SIZE; j=j+1) begin : loop_j
        wire [2*LOOP_NUM+3:0] cout_w;
        wire [2*LOOP_NUM+3:0] shareout_w;

        twentynm_lcell_comb #(
            .lut_mask({16'h0, {8{C[j][SIZE-4]}}, {4{C[j][SIZE-3]}}, {2{C[j][SIZE-2]}}, C[j][SIZE-1], C[j][SIZE], 32'h0}),
            .shared_arith("on")
        ) c0 (
            .dataa(din[0]), 
            .datab(din[1]), 
            .datac(din[2]), 
            .datad(din[3]), 
            .cin(1'b0),
            .sharein(1'b0),
            .sumout(), 
            .cout(cout_w[0]),
            .shareout(shareout_w[0]),
            .datae(), .dataf(), .datag(), .combout()
        );

        twentynm_lcell_comb #(
            .lut_mask({16'h0, {8{C[j][SIZE-6]}}, {4{C[j][SIZE-5]}}, 4'h0, 16'h0, 16'h000F}),
            .shared_arith("on")
        ) c1 (
            .dataa(), 
            .datab(), 
            .datac(din[4]), 
            .datad(din[5]), 
            .cin(cout_w[0]),
            .sharein(shareout_w[0]),
            .sumout(), 
            .cout(cout_w[1]),
            .shareout(shareout_w[1]),
            .datae(), .dataf(), .datag(), .combout()
        );


        for (i = 0; i < LOOP_NUM; i = i + 1)
        begin: loop
            twentynm_lcell_comb #(
                .lut_mask({16'h0, {8{C[j][SIZE-6*i-10]}}, {4{C[j][SIZE-6*i-9]}}, {2{C[j][SIZE-6*i-8]}}, C[j][SIZE-6*i-7], 1'b0, 16'h0, 16'h0001}),
                .shared_arith("on")
            ) c2 (
                .dataa(din[6*i+6]), 
                .datab(din[6*i+7]), 
                .datac(din[6*i+8]), 
                .datad(din[6*i+9]), 
                .cin(cout_w[2*i+1]),
                .sharein(shareout_w[2*i+1]),
                .sumout(), 
                .cout(cout_w[2*i+2]),
                .shareout(shareout_w[2*i+2]),
                .datae(), .dataf(), .datag(), .combout()
            );
            
            // cout[2] = cout[13] & sum[4] & sum[5]
            twentynm_lcell_comb #(
                .lut_mask({16'h0, {8{C[j][SIZE-6*i-12]}}, {4{C[j][SIZE-6*i-11]}}, 4'h0, 16'h0, 16'h000F}),
                .shared_arith("on")
            ) c3 (
                .dataa(), 
                .datab(), 
                .datac(din[6*i+10]), 
                .datad(din[6*i+11]), 
                .cin(cout_w[2*i+2]),
                .sharein(shareout_w[2*i+2]),
                .sumout(), 
                .cout(cout_w[2*i+3]),
                .shareout(shareout_w[2*i+3]),
                .datae(), .dataf(), .datag(), .combout()
            );
        end

        if (SIZE % 6 == 0)
        begin
            // Last empty cell
            twentynm_lcell_comb #(
                .lut_mask(64'h0000_0000_0000_0000),
                .shared_arith("on")
            ) cN (
                .dataa(), 
                .datab(), 
                .datac(), 
                .datad(), 
                .cin(cout_w[2*LOOP_NUM+1]),
                .sharein(shareout_w[2*LOOP_NUM+1]),
                .sumout(dout_w[j]), 
                .cout(),
                .shareout(),
                .datae(), .dataf(), .datag(), .combout()
            );
        end
        else if (SIZE % 6 == 1)
        begin	
            // Second to last cell
            twentynm_lcell_comb #(
                .lut_mask({16'h0, {8{C[j][0]}}, 8'h0, 16'h0, 16'h00FF}),
                .shared_arith("on")
            ) cN_minus_1 (
                .dataa(), 
                .datab(), 
                .datac(), 
                .datad(din[SIZE-1]), 
                .cin(cout_w[2*LOOP_NUM+1]),
                .sharein(shareout_w[2*LOOP_NUM+1]),
                .sumout(), 
                .cout(cout_w[2*LOOP_NUM+2]),
                .shareout(shareout_w[2*LOOP_NUM+2]),
                .datae(), .dataf(), .datag(), .combout()
            );

            // Last empty cell
            twentynm_lcell_comb #(
                .lut_mask(64'h0000_0000_0000_0000),
                .shared_arith("on")
            ) cN (
                .dataa(), 
                .datab(), 
                .datac(), 
                .datad(), 
                .cin(cout_w[2*LOOP_NUM+2]),
                .sharein(shareout_w[2*LOOP_NUM+2]),
                .sumout(dout_w[j]), 
                .cout(),
                .shareout(),
                .datae(), .dataf(), .datag(), .combout()
            );
        end
        else if (SIZE % 6 == 2)
        begin	
            // Second to last cell
            twentynm_lcell_comb #(
                .lut_mask({16'h0, {8{C[j][0]}}, {4{C[j][1]}}, 4'h0, 16'h0, 16'h000F}),
                .shared_arith("on")
            ) cN_minus_1 (
                .dataa(), 
                .datab(), 
                .datac(din[SIZE-2]), 
                .datad(din[SIZE-1]), 
                .cin(cout_w[2*LOOP_NUM+1]),
                .sharein(shareout_w[2*LOOP_NUM+1]),
                .sumout(), 
                .cout(cout_w[2*LOOP_NUM+2]),
                .shareout(shareout_w[2*LOOP_NUM+2]),
                .datae(), .dataf(), .datag(), .combout()
            );

            // Last empty cell
            twentynm_lcell_comb #(
                .lut_mask(64'h0000_0000_0000_0000),
                .shared_arith("on")
            ) cN (
                .dataa(), 
                .datab(), 
                .datac(), 
                .datad(), 
                .cin(cout_w[2*LOOP_NUM+2]),
                .sharein(shareout_w[2*LOOP_NUM+2]),
                .sumout(dout_w[j]), 
                .cout(),
                .shareout(),
                .datae(), .dataf(), .datag(), .combout()
            );
        end
        else if (SIZE % 6 == 3)
        begin	
            // Second to last cell
            twentynm_lcell_comb #(
                .lut_mask({16'h0, {8{C[j][0]}}, {4{C[j][1]}}, {2{C[j][2]}}, 2'h0, 16'h0, 16'h0003}),
                .shared_arith("on")
            ) cN_minus_1 (
                .dataa(), 
                .datab(din[SIZE-3]), 
                .datac(din[SIZE-2]), 
                .datad(din[SIZE-1]), 
                .cin(cout_w[2*LOOP_NUM+1]),
                .sharein(shareout_w[2*LOOP_NUM+1]),
                .sumout(), 
                .cout(cout_w[2*LOOP_NUM+2]),
                .shareout(shareout_w[2*LOOP_NUM+2]),
                .datae(), .dataf(), .datag(), .combout()
            );

            // Last empty cell
            twentynm_lcell_comb #(
                .lut_mask(64'h0000_0000_0000_0000),
                .shared_arith("on")
            ) cN (
                .dataa(), 
                .datab(), 
                .datac(), 
                .datad(), 
                .cin(cout_w[2*LOOP_NUM+2]),
                .sharein(shareout_w[2*LOOP_NUM+2]),
                .sumout(dout_w[j]), 
                .cout(),
                .shareout(),
                .datae(), .dataf(), .datag(), .combout()
            );
        end
        else if (SIZE % 6 == 4)
        begin	
            // Second to last cell
            twentynm_lcell_comb #(
                .lut_mask({16'h0, {8{C[j][0]}}, {4{C[j][1]}}, {2{C[j][2]}}, C[j][3], 1'h0, 16'h0, 16'h0001}),
                .shared_arith("on")
            ) cN_minus_1 (
                .dataa(din[SIZE-4]), 
                .datab(din[SIZE-3]), 
                .datac(din[SIZE-2]), 
                .datad(din[SIZE-1]), 
                .cin(cout_w[2*LOOP_NUM+1]),
                .sharein(shareout_w[2*LOOP_NUM+1]),
                .sumout(), 
                .cout(cout_w[2*LOOP_NUM+2]),
                .shareout(shareout_w[2*LOOP_NUM+2]),
                .datae(), .dataf(), .datag(), .combout()
            );

            // Last empty cell
            twentynm_lcell_comb #(
                .lut_mask(64'h0000_0000_0000_0000),
                .shared_arith("on")
            ) cN (
                .dataa(), 
                .datab(), 
                .datac(), 
                .datad(), 
                .cin(cout_w[2*LOOP_NUM+2]),
                .sharein(shareout_w[2*LOOP_NUM+2]),
                .sumout(dout_w[j]), 
                .cout(),
                .shareout(),
                .datae(), .dataf(), .datag(), .combout()
            );
        end
        else if (SIZE % 6 == 5)
        begin	
            // Third to last cell
            twentynm_lcell_comb #(
                .lut_mask({16'h0, {8{C[j][1]}}, {4{C[j][2]}}, {2{C[j][3]}}, C[j][4], 1'h0, 16'h0, 16'h0001}),
                .shared_arith("on")
            ) cN_minus_2 (
                .dataa(din[SIZE-5]), 
                .datab(din[SIZE-4]), 
                .datac(din[SIZE-3]), 
                .datad(din[SIZE-2]), 
                .cin(cout_w[2*LOOP_NUM+1]),
                .sharein(shareout_w[2*LOOP_NUM+1]),
                .sumout(), 
                .cout(cout_w[2*LOOP_NUM+2]),
                .shareout(shareout_w[2*LOOP_NUM+2]),
                .datae(), .dataf(), .datag(), .combout()
            );

            // Second to last cell
            twentynm_lcell_comb #(
                .lut_mask({16'h0, {8{C[j][0]}}, 8'h0, 16'h0, 16'h00FF}),
                .shared_arith("on")
            ) cN_minus_1 (
                .dataa(), 
                .datab(), 
                .datac(), 
                .datad(din[SIZE-1]), 
                .cin(cout_w[2*LOOP_NUM+2]),
                .sharein(shareout_w[2*LOOP_NUM+2]),
                .sumout(), 
                .cout(cout_w[2*LOOP_NUM+3]),
                .shareout(shareout_w[2*LOOP_NUM+3]),
                .datae(), .dataf(), .datag(), .combout()
            );

            // Last empty cell
            twentynm_lcell_comb #(
                .lut_mask(64'h0000_0000_0000_0000),
                .shared_arith("on")
            ) cN (
                .dataa(), 
                .datab(), 
                .datac(), 
                .datad(), 
                .cin(cout_w[2*LOOP_NUM+3]),
                .sharein(shareout_w[2*LOOP_NUM+3]),
                .sumout(dout_w[j]), 
                .cout(),
                .shareout(),
                .datae(), .dataf(), .datag(), .combout()
            );
        end
    end
end
endgenerate

always @(posedge clk) begin
    dout <= dout_w;
end

endmodule

