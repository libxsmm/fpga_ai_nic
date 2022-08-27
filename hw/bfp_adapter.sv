/******************************************************************************
* Copyright (c) Intel Corporation - All rights reserved.                      *
* This file is part of the LIBXSMM library.                                   *
*                                                                             *
* For information on the license, see the LICENSE file.                       *
* Further information: https://github.com/libxsmm/libxsmm/                    *
* SPDX-License-Identifier: BSD-3-Clause                                       *
******************************************************************************/
// ***********************************************************************
// Intel Confidential
// Copyright 2019-2020 Intel Corporation
//
// This software (including Verilog code) and the related documents are
// Intel copyrighted materials, and your use of them is governed by the
// express license under which they were provided to you
// ("License"). Unless the License provides otherwise, you may not use,
// modify, copy, publish, distribute, disclose or transmit this software
// or the related documents without Intel's prior written permission.
//
// This software and the related documents are provided as is, with no
// express or implied warranties, other than those that are expressly
// stated in the License.
//
// Unless otherwise agreed by Intel in writing, you may not remove or
// alter this notice or any other notice embedded in Materials by Intel
// or Intel's suppliers or licensors in any way.
// ***********************************************************************
//
// Module Name: bfp_adapter
// Number of flits must be multiple of 32


module bfp_adapter #(
    parameter NUM_FP = 16,
    // parameter EXP_SIZE = 8,
    parameter MANT_SIZE = 8
    )(
    input [255:0]      snic_in,
    input              snic_in_valid,
    output             snic_in_ready,
    output [255:0]     snic_out,
    output             snic_out_valid,
    input              snic_out_ready,

    input [255:0]      eth_in,
    input              eth_in_valid,
    output             eth_in_ready,
    output reg [255:0] eth_out,
    output reg         eth_out_valid,
    input              eth_out_ready,

    output [127:0]      debug_status,
    input              clk,
    input              rst_n
);

localparam EXP_SIZE = 8;

localparam FIFO_128_EN = 1;
localparam FIFO_64_EN = 0;
localparam FIFO_16_EN = 0;
localparam FIFO_8_EN = 1;
localparam FIFO_128_IN_POS = 0;
localparam FIFO_64_IN_POS = FIFO_128_EN * 128 + FIFO_128_IN_POS;
localparam FIFO_16_IN_POS = FIFO_64_EN * 64 + FIFO_64_IN_POS;
localparam FIFO_8_IN_POS = FIFO_16_EN * 16 + FIFO_16_IN_POS;
localparam FIFO_8_COUNT = 1;
localparam FIFO_16_COUNT = 2;
localparam FIFO_64_COUNT = 8;
localparam FIFO_128_COUNT = 16;
localparam FP_SIZE = EXP_SIZE + MANT_SIZE;
localparam NUM_B = (NUM_FP < 8) ? (8/NUM_FP) : 1;
localparam NUM_C = (NUM_FP < 8) ? 1 : (NUM_FP/8);
localparam C_SIZE = 8 * (EXP_SIZE + MANT_SIZE);
localparam BUF_SIZE = NUM_FP * 32;
localparam BFP_SIZE = EXP_SIZE + NUM_FP * MANT_SIZE;
localparam BFP_BUF_SIZE = NUM_B * BFP_SIZE;

localparam LOG_SIZE = $clog2(24);
localparam SHIFT_SIZE = (EXP_SIZE > LOG_SIZE) ? LOG_SIZE : EXP_SIZE;
localparam SHIFT_LATENCY = (SHIFT_SIZE + 1) / 2;
localparam MAX_LATENCY = (NUM_FP <= 4) ? 2 : (
                                (NUM_FP <= 12) ? 4 : (
                                  (NUM_FP <= 36) ? 6 : (
                                    (NUM_FP <= 108) ? 8 : 10
                                  )
                                )
                              );
localparam SHARED_EXPONENT_LATENCY = SHIFT_LATENCY + 2;
localparam LATENCY = MAX_LATENCY + SHIFT_LATENCY + 2;

logic [BUF_SIZE-1:0] input_buf;
logic [LATENCY+1:0] valid_vec;
logic send_fifo_ok;
logic [3:0] n_count;
integer i, j;

assign snic_in_ready = send_fifo_ok;

// Fill input buffer
always @(posedge clk) begin
    if (!rst_n) begin
        valid_vec <= 'b0;
        n_count <= 'b0;
    end else begin
        valid_vec[LATENCY+1] <= 'b0;
        valid_vec[LATENCY:0] <= valid_vec[LATENCY+1:1];
        input_buf[n_count*256+:256] <= snic_in;
        if (send_fifo_ok && snic_in_valid) begin
            if (n_count == NUM_C-1) begin
                n_count <= 'd0;
                valid_vec[LATENCY+1] <= 'b1;
            end else n_count <= n_count + 4'd1;
        end
    end
end

// FP to BFP converter
wire sign_in[0:NUM_B-1][0:NUM_FP-1];
wire [7:0] exponent_in[0:NUM_B-1][0:NUM_FP-1];
wire [23:0] mantissa_in[0:NUM_B-1][0:NUM_FP-1];
wire [23:0] dout[0:NUM_B-1][0:NUM_FP-1];
wire [7:0] shared_exponent_out[0:NUM_B-1];
genvar m, n;

generate
    for (m = 0; m < NUM_B; m = m+1) begin: converter_inst
        for (n = 0; n < NUM_FP; n = n+1) begin : din_assigns
            assign sign_in[m][n] = input_buf[(m*NUM_FP+n)*32+31];
            assign exponent_in[m][n] = input_buf[((m*NUM_FP+n)*32+23)+:8];
            assign mantissa_in[m][n] = {1'b1, input_buf[((m*NUM_FP+n)*32)+:23]};
        end

        bf16_to_bfp_core #(.NUM(NUM_FP), .EXPONENT_SIZE(8), .MANTISSA_SIZE(24), .NX_MODE(0)) compress (
            .clk(clk),
            .sign_in(sign_in[m]),
            .exponent_in(exponent_in[m]),
            .mantissa_in(mantissa_in[m]),
            .shared_exponent_out(shared_exponent_out[m]),
            .sdata_out(dout[m])
        );
    end
endgenerate

// Truncate
reg [BFP_BUF_SIZE-1:0] bfp_buf;
always @(posedge clk) begin
    for (i = 0; i < NUM_B; i = i+1) begin
        for (j = 0; j < NUM_FP; j = j+1) begin
            bfp_buf[i*BFP_SIZE+j*MANT_SIZE+:MANT_SIZE] <= dout[i][j][23:(24-MANT_SIZE)];
        end
        bfp_buf[i*BFP_SIZE+NUM_FP*MANT_SIZE+:EXP_SIZE] <= shared_exponent_out[i];
    end
end

// Output FIFOs
logic [7:0] o_fifo_8_in;
logic o_fifo_8_in_valid;
logic [7:0] o_fifo_8_in_usedw;
logic o_fifo_8_in_ready;
logic [255:0] o_fifo_8_out;
logic o_fifo_8_out_empty;
logic o_fifo_8_out_deq;

logic [15:0] o_fifo_16_in;
logic o_fifo_16_in_valid;
logic [7:0] o_fifo_16_in_usedw;
logic o_fifo_16_in_ready;
logic [255:0] o_fifo_16_out;
logic o_fifo_16_out_empty;
logic o_fifo_16_out_deq;

logic [63:0] o_fifo_64_in;
logic o_fifo_64_in_valid;
logic [7:0] o_fifo_64_in_usedw;
logic o_fifo_64_in_ready;
logic [255:0] o_fifo_64_out;
logic o_fifo_64_out_empty;
logic o_fifo_64_out_deq;

logic [127:0] o_fifo_128_in;
logic o_fifo_128_in_valid;
logic [7:0] o_fifo_128_in_usedw;
logic o_fifo_128_in_ready;
logic [255:0] o_fifo_128_out;
logic o_fifo_128_out_empty;
logic o_fifo_128_out_deq;

generate
    if (FIFO_8_EN) begin
        async_fifo_mixed #(.IDW(8), .ODW(256), .IAW(8), .OAW(3), .DEPTH(256)) o_fifo8 (
            .aclr(~rst_n),
            .data(o_fifo_8_in),
            .rdclk(clk),
            .rdreq(o_fifo_8_out_deq),
            .wrclk(clk),
            .wrreq(o_fifo_8_in_valid),
            .q(o_fifo_8_out),
            .rdempty(o_fifo_8_out_empty),
            .wrfull(),
            .wrusedw(o_fifo_8_in_usedw)
        );
        assign o_fifo_8_in_ready = ~o_fifo_8_in_usedw[7];
        assign o_fifo_8_in = bfp_buf[FIFO_8_IN_POS+:8];
        assign o_fifo_8_in_valid = valid_vec[0];
    end else begin
        assign o_fifo_8_in_ready = 'b1;
    end

    if (FIFO_16_EN) begin
        async_fifo_mixed #(.IDW(16), .ODW(256), .IAW(8), .OAW(4), .DEPTH(256)) o_fifo16 (
            .aclr(~rst_n),
            .data(o_fifo_16_in),
            .rdclk(clk),
            .rdreq(o_fifo_16_out_deq),
            .wrclk(clk),
            .wrreq(o_fifo_16_in_valid),
            .q(o_fifo_16_out),
            .rdempty(o_fifo_16_out_empty),
            .wrfull(),
            .wrusedw(o_fifo_16_in_usedw)
        );
        assign o_fifo_16_in_ready = ~o_fifo_16_in_usedw[7];
        assign o_fifo_16_in = bfp_buf[FIFO_16_IN_POS+:16];
        assign o_fifo_16_in_valid = valid_vec[0];
    end else begin
        assign o_fifo_16_in_ready = 'b1;
    end

    if (FIFO_64_EN) begin
        async_fifo_mixed #(.IDW(64), .ODW(256), .IAW(8), .OAW(6), .DEPTH(256)) o_fifo64 (
            .aclr(~rst_n),
            .data(o_fifo_64_in),
            .rdclk(clk),
            .rdreq(o_fifo_64_out_deq),
            .wrclk(clk),
            .wrreq(o_fifo_64_in_valid),
            .q(o_fifo_64_out),
            .rdempty(o_fifo_64_out_empty),
            .wrfull(),
            .wrusedw(o_fifo_64_in_usedw)
        );
        assign o_fifo_64_in_ready = ~o_fifo_64_in_usedw[7];
        assign o_fifo_64_in = bfp_buf[FIFO_64_IN_POS+:64];
        assign o_fifo_64_in_valid = valid_vec[0];
    end else begin
        assign o_fifo_64_in_ready = 'b1;
    end

    if (FIFO_128_EN) begin
        async_fifo_mixed #(.IDW(128), .ODW(256), .IAW(8), .OAW(7), .DEPTH(256)) o_fifo128 (
            .aclr(~rst_n),
            .data(o_fifo_128_in),
            .rdclk(clk),
            .rdreq(o_fifo_128_out_deq),
            .wrclk(clk),
            .wrreq(o_fifo_128_in_valid),
            .q(o_fifo_128_out),
            .rdempty(o_fifo_128_out_empty),
            .wrfull(),
            .wrusedw(o_fifo_128_in_usedw)
        );
        assign o_fifo_128_in_ready = ~o_fifo_128_in_usedw[7];
        assign o_fifo_128_in = bfp_buf[FIFO_128_IN_POS+:128];
        assign o_fifo_128_in_valid = valid_vec[0];
    end else begin
        assign o_fifo_128_in_ready = 'b1;
    end
endgenerate

always @(posedge clk) begin
    if (!rst_n) begin
        send_fifo_ok <= 'b0;
    end else begin
        send_fifo_ok <= o_fifo_8_in_ready && o_fifo_16_in_ready && o_fifo_64_in_ready && o_fifo_128_in_ready;
    end
end

// Eth out
logic [2:0] state_out;
logic [4:0] eth_out_count;
assign o_fifo_128_out_deq = (state_out == 3'd1) && eth_out_ready && (!o_fifo_128_out_empty);
assign o_fifo_64_out_deq = (state_out == 3'd2) && eth_out_ready && (!o_fifo_64_out_empty);
assign o_fifo_16_out_deq = (state_out == 3'd3) && eth_out_ready && (!o_fifo_16_out_empty);
assign o_fifo_8_out_deq = (state_out == 3'd4) && eth_out_ready && (!o_fifo_8_out_empty);
always @(posedge clk) begin
    if (!rst_n) begin
        state_out <= 3'd0;
        eth_out_valid <= 'b0;
        eth_out_count <= 'b0;
    end else begin
        case (state_out)
            // IDLE
            3'd0: begin
                eth_out_count <= 'b0;
                if (FIFO_128_EN) state_out <= 3'd1;
                else if (FIFO_64_EN) state_out <= 3'd2;
                else if (FIFO_16_EN) state_out <= 3'd3;
                else if (FIFO_8_EN) state_out <= 3'd4;
            end
            // output FIFO_128
            3'd1: begin
                if (eth_out_ready) begin
                    eth_out_valid <= 'b0;
                    if (!o_fifo_128_out_empty) begin
                        eth_out_valid <= 'b1;
                        eth_out <= o_fifo_128_out;
                        if (eth_out_count == FIFO_128_COUNT-1) begin
                            if (FIFO_64_EN) state_out <= 3'd2;
                            else if (FIFO_16_EN) state_out <= 3'd3;
                            else if (FIFO_8_EN) state_out <= 3'd4;
                            eth_out_count <= 'd0;
                        end else begin
                            eth_out_count <= eth_out_count + 4'd1;
                        end
                    end
                end
            end
            // output FIFO_64
            3'd2: begin
                if (eth_out_ready) begin
                    eth_out_valid <= 'b0;
                    if (!o_fifo_64_out_empty) begin
                        eth_out_valid <= 'b1;
                        eth_out <= o_fifo_64_out;
                        if (eth_out_count == FIFO_64_COUNT-1) begin
                            if (FIFO_16_EN) state_out <= 3'd3;
                            else if (FIFO_8_EN) state_out <= 3'd4;
                            else if (FIFO_128_EN) state_out <= 3'd1;
                            eth_out_count <= 'd0;
                        end else begin
                            eth_out_count <= eth_out_count + 4'd1;
                        end
                    end
                end
            end
            // output FIFO_16
            3'd3: begin
                if (eth_out_ready) begin
                    eth_out_valid <= 'b0;
                    if (!o_fifo_16_out_empty) begin
                        eth_out_valid <= 'b1;
                        eth_out <= o_fifo_16_out;
                        if (eth_out_count == FIFO_16_COUNT-1) begin
                            if (FIFO_8_EN) state_out <= 3'd4;
                            else if (FIFO_128_EN) state_out <= 3'd1;
                            else if (FIFO_64_EN) state_out <= 3'd2;
                            eth_out_count <= 'd0;
                        end else begin
                            eth_out_count <= eth_out_count + 4'd1;
                        end
                    end
                end
            end
            // output FIFO_8
            3'd4: begin
                if (eth_out_ready) begin
                    eth_out_valid <= 'b0;
                    if (!o_fifo_8_out_empty) begin
                        eth_out_valid <= 'b1;
                        eth_out <= o_fifo_8_out;
                        if (eth_out_count == FIFO_8_COUNT-1) begin
                            if (FIFO_128_EN) state_out <= 3'd1;
                            else if (FIFO_64_EN) state_out <= 3'd2;
                            else if (FIFO_16_EN) state_out <= 3'd3;
                            eth_out_count <= 'd0;
                        end else begin
                            eth_out_count <= eth_out_count + 4'd1;
                        end
                    end
                end
            end
            default: begin
                eth_out_valid <= 'b0;
                state_out <= 'b0;
            end
        endcase
    end
end

// Eth in
// Eth input FIFOs
logic [255:0] i_fifo_8_in;
logic i_fifo_8_in_valid;
logic i_fifo_8_in_ready;
logic i_fifo_8_in_full;
logic [7:0] i_fifo_8_out;
logic i_fifo_8_out_empty;
logic i_fifo_8_out_deq;

logic [255:0] i_fifo_16_in;
logic i_fifo_16_in_valid;
logic i_fifo_16_in_ready;
logic i_fifo_16_in_full;
logic [15:0] i_fifo_16_out;
logic i_fifo_16_out_empty;
logic i_fifo_16_out_deq;

logic [255:0] i_fifo_64_in;
logic i_fifo_64_in_valid;
logic i_fifo_64_in_ready;
logic i_fifo_64_in_full;
logic [63:0] i_fifo_64_out;
logic i_fifo_64_out_empty;
logic i_fifo_64_out_deq;

logic [255:0] i_fifo_128_in;
logic i_fifo_128_in_valid;
logic i_fifo_128_in_ready;
logic i_fifo_128_in_full;
logic [127:0] i_fifo_128_out;
logic i_fifo_128_out_empty;
logic i_fifo_128_out_deq;

generate
    if (FIFO_8_EN) begin
        async_fifo_mixed #(.IDW(256), .ODW(8), .IAW(3), .OAW(8), .DEPTH(8)) i_fifo8 (
            .aclr(~rst_n),
            .data(i_fifo_8_in),
            .rdclk(clk),
            .rdreq(i_fifo_8_out_deq),
            .wrclk(clk),
            .wrreq(i_fifo_8_in_valid),
            .q(i_fifo_8_out),
            .rdempty(i_fifo_8_out_empty),
            .wrfull(i_fifo_8_in_full)
        );
        assign i_fifo_8_in_ready = ~i_fifo_8_in_full;
    end else begin
        assign i_fifo_8_in_ready = 'b1;
        assign i_fifo_8_out_empty = 'b0;
    end

    if (FIFO_16_EN) begin
        async_fifo_mixed #(.IDW(256), .ODW(16), .IAW(4), .OAW(8), .DEPTH(16)) i_fifo16 (
            .aclr(~rst_n),
            .data(i_fifo_16_in),
            .rdclk(clk),
            .rdreq(i_fifo_16_out_deq),
            .wrclk(clk),
            .wrreq(i_fifo_16_in_valid),
            .q(i_fifo_16_out),
            .rdempty(i_fifo_16_out_empty),
            .wrfull(i_fifo_16_in_full)
        );
        assign i_fifo_16_in_ready = ~i_fifo_16_in_full;
    end else begin
        assign i_fifo_16_in_ready = 'b1;
        assign i_fifo_16_out_empty = 'b0;
    end

    if (FIFO_64_EN) begin
        async_fifo_mixed #(.IDW(256), .ODW(64), .IAW(6), .OAW(8), .DEPTH(64)) i_fifo64 (
            .aclr(~rst_n),
            .data(i_fifo_64_in),
            .rdclk(clk),
            .rdreq(i_fifo_64_out_deq),
            .wrclk(clk),
            .wrreq(i_fifo_64_in_valid),
            .q(i_fifo_64_out),
            .rdempty(i_fifo_64_out_empty),
            .wrfull(i_fifo_64_in_full)
        );
        assign i_fifo_64_in_ready = ~i_fifo_64_in_full;
    end else begin
        assign i_fifo_64_in_ready = 'b1;
        assign i_fifo_64_out_empty = 'b0;
    end

    if (FIFO_128_EN) begin
        async_fifo_mixed #(.IDW(256), .ODW(128), .IAW(7), .OAW(8), .DEPTH(128)) i_fifo128 (
            .aclr(~rst_n),
            .data(i_fifo_128_in),
            .rdclk(clk),
            .rdreq(i_fifo_128_out_deq),
            .wrclk(clk),
            .wrreq(i_fifo_128_in_valid),
            .q(i_fifo_128_out),
            .rdempty(i_fifo_128_out_empty),
            .wrfull(i_fifo_128_in_full)
        );
        assign i_fifo_128_in_ready = ~i_fifo_128_in_full;
    end else begin
        assign i_fifo_128_in_ready = 'b1;
        assign i_fifo_128_out_empty = 'b0;
    end
endgenerate

// Fill Eth input FIFOs
logic [2:0] state_in;
logic [4:0] eth_in_count;
assign eth_in_ready = i_fifo_8_in_ready && i_fifo_16_in_ready && i_fifo_64_in_ready && i_fifo_128_in_ready;
always @(posedge clk) begin
    if (!rst_n) begin
        state_in <= 'd0;
        eth_in_count <= 'd0;
        i_fifo_128_in_valid <= 'b0;
        i_fifo_64_in_valid <= 'b0;
        i_fifo_16_in_valid <= 'b0;
        i_fifo_8_in_valid <= 'b0;
    end else begin
        if (!i_fifo_128_in_full) i_fifo_128_in_valid <= 'b0;
        if (!i_fifo_64_in_full) i_fifo_64_in_valid <= 'b0;
        if (!i_fifo_16_in_full) i_fifo_16_in_valid <= 'b0;
        if (!i_fifo_8_in_full) i_fifo_8_in_valid <= 'b0;
        case (state_in)
            3'd0: begin
                eth_in_count <= 'b0;
                if (FIFO_128_EN) state_in <= 3'd1;
                else if (FIFO_64_EN) state_in <= 3'd2;
                else if (FIFO_16_EN) state_in <= 3'd3;
                else if (FIFO_8_EN) state_in <= 3'd4;
            end
            3'd1: begin
                if (eth_in_valid && eth_in_ready) begin
                    i_fifo_128_in_valid <= 'b1;
                    i_fifo_128_in <= eth_in;
                    if (eth_in_count == FIFO_128_COUNT-1) begin
                        if (FIFO_64_EN) state_in <= 3'd2;
                        else if (FIFO_16_EN) state_in <= 3'd3;
                        else if (FIFO_8_EN) state_in <= 3'd4;
                        eth_in_count <= 'd0;
                    end else begin
                        eth_in_count <= eth_in_count + 4'd1;
                    end
                end
            end
            3'd2: begin
                if (eth_in_valid && eth_in_ready) begin
                    i_fifo_64_in_valid <= 'b1;
                    i_fifo_64_in <= eth_in;
                    if (eth_in_count == FIFO_64_COUNT-1) begin
                        if (FIFO_16_EN) state_in <= 3'd3;
                        else if (FIFO_8_EN) state_in <= 3'd4;
                        else if (FIFO_128_EN) state_in <= 3'd1;
                        eth_in_count <= 'd0;
                    end else begin
                        eth_in_count <= eth_in_count + 4'd1;
                    end
                end
            end
            3'd3: begin
                if (eth_in_valid && eth_in_ready) begin
                    i_fifo_16_in_valid <= 'b1;
                    i_fifo_16_in <= eth_in;
                    if (eth_in_count == FIFO_16_COUNT-1) begin
                        if (FIFO_8_EN) state_in <= 3'd4;
                        else if (FIFO_128_EN) state_in <= 3'd1;
                        else if (FIFO_64_EN) state_in <= 3'd2;
                        eth_in_count <= 'd0;
                    end else begin
                        eth_in_count <= eth_in_count + 4'd1;
                    end
                end
            end
            3'd4: begin
                if (eth_in_valid && eth_in_ready) begin
                    i_fifo_8_in_valid <= 'b1;
                    i_fifo_8_in <= eth_in;
                    if (eth_in_count == FIFO_8_COUNT-1) begin
                        if (FIFO_128_EN) state_in <= 3'd1;
                        else if (FIFO_64_EN) state_in <= 3'd2;
                        else if (FIFO_16_EN) state_in <= 3'd3;
                        eth_in_count <= 'd0;
                    end else begin
                        eth_in_count <= eth_in_count + 4'd1;
                    end
                end
            end
            default: begin
                state_in <= 'b0;
            end
        endcase // state_in
    end
end

// Output FIFO
logic [255:0] o_fifo_256_in;
logic o_fifo_256_in_valid;
logic [5:0] o_fifo_256_usedw;
logic o_fifo_256_in_almFull;
logic [255:0] o_fifo_256_out;
logic o_fifo_256_out_empty;
logic o_fifo_256_out_deq;

assign o_fifo_256_in_almFull = o_fifo_256_usedw[5];

fifo #(.DW(256), .AW(6), .DEPTH(64)) fifo256 (
    .clock(clk),
    .data(o_fifo_256_in),
    .rdreq(o_fifo_256_out_deq),
    .sclr(!rst_n),
    .wrreq(o_fifo_256_in_valid),
    .usedw(o_fifo_256_usedw),
    .empty(o_fifo_256_out_empty),
    .q(o_fifo_256_out)
);

// decompress
localparam DECOMPRESS_LAT = (($clog2(25) + 1) / 2) + 2 + 1;
reg [DECOMPRESS_LAT-1:0] fp_out_valid_vec;
logic [BFP_BUF_SIZE-1:0] bfp_in;
logic [BUF_SIZE-1:0] fp_out;
logic bfp_in_valid;
logic fp_out_ready;
logic [3:0] snic_out_count_in;
logic [3:0] snic_out_count_out;

always @(posedge clk) begin
    if (FIFO_8_EN) begin
        bfp_in[FIFO_8_IN_POS+:8] <= i_fifo_8_out;
    end
    if (FIFO_16_EN) begin
        bfp_in[FIFO_16_IN_POS+:16] <= i_fifo_16_out;
    end
    if (FIFO_64_EN) begin
        bfp_in[FIFO_64_IN_POS+:64] <= i_fifo_64_out;
    end
    if (FIFO_128_EN) begin
        bfp_in[FIFO_128_IN_POS+:128] <= i_fifo_128_out;
    end
end

assign bfp_in_valid = (!i_fifo_8_out_empty) && (!i_fifo_16_out_empty) && (!i_fifo_64_out_empty) && (!i_fifo_128_out_empty);
assign i_fifo_128_out_deq = bfp_in_valid && fp_out_ready;
assign i_fifo_64_out_deq = bfp_in_valid && fp_out_ready;
assign i_fifo_16_out_deq = bfp_in_valid && fp_out_ready;
assign i_fifo_8_out_deq = bfp_in_valid && fp_out_ready;

assign fp_out_ready = (!o_fifo_256_in_almFull) && (snic_out_count_in == NUM_C-1);
always @(posedge clk) begin
    if (!rst_n) begin
        snic_out_count_in <= 'b0;
        fp_out_valid_vec <= 'b0;
    end else begin
        if (!o_fifo_256_in_almFull) begin
            fp_out_valid_vec[DECOMPRESS_LAT-1] <= bfp_in_valid;
            if (bfp_in_valid) begin
                if (snic_out_count_in == NUM_C-1) begin
                    snic_out_count_in <= 'b0;
                end else begin
                    snic_out_count_in <= snic_out_count_in + 'd1;
                end
            end
        end else begin
            fp_out_valid_vec[DECOMPRESS_LAT-1] <= 'b0;
        end
        fp_out_valid_vec[DECOMPRESS_LAT-2:0] <= fp_out_valid_vec[DECOMPRESS_LAT-1:1];
    end
end

always @(posedge clk) begin
    if (!rst_n) begin
        snic_out_count_out <= 'b0;
        o_fifo_256_in_valid <= 'b0;
    end else begin
        o_fifo_256_in_valid <= 'b0;
        o_fifo_256_in <= fp_out[snic_out_count_out*256+:256];
        if (fp_out_valid_vec[0]) begin
            o_fifo_256_in_valid <= 'b1;
            if (snic_out_count_out == NUM_C-1) begin
                snic_out_count_out <= 'b0;
            end else begin
                snic_out_count_out <= snic_out_count_out + 4'd1;
            end
        end
    end
end

wire [23:0] sdata_in[0:NUM_B-1][0:NUM_FP-1];
wire sign_out[0:NUM_B-1][0:NUM_FP-1];
wire [7:0] exponent_out[0:NUM_B-1][0:NUM_FP-1];
wire [23:0] mantissa_out[0:NUM_B-1][0:NUM_FP-1];

generate
    for (m = 0; m < NUM_B; m = m+1) begin: decompress_inst
        for (n = 0; n < NUM_FP; n = n+1) begin : sdata_in_assigns
            assign sdata_in[m][n] = {bfp_in[m*BFP_SIZE+n*MANT_SIZE+:MANT_SIZE], {(24-MANT_SIZE){1'b0}}};

            assign fp_out[(m*NUM_FP+n)*32+31] = sign_out[m][n];
            assign fp_out[(m*NUM_FP+n)*32+:23] = mantissa_out[m][n][22:0];
            assign fp_out[(m*NUM_FP+n)*32+23+:8] = exponent_out[m][n];
        end

        bfp_to_bf16_core #(.NUM(NUM_FP), .EXPONENT_SIZE(8), .MANTISSA_SIZE(24), .NX_MODE(0), .FAMILY("A10")) decompress (
            .clk(clk),
            .shared_exponent_in(bfp_in[m*BFP_SIZE+NUM_FP*MANT_SIZE+:EXP_SIZE]),
            .sdata_in(sdata_in[m]),
            .sign_out(sign_out[m]),
            .exponent_out(exponent_out[m]),
            .mantissa_out(mantissa_out[m])
        );
    end
endgenerate


assign snic_out = o_fifo_256_out;
assign o_fifo_256_out_deq = (!o_fifo_256_out_empty) && snic_out_ready;
assign snic_out_valid = (!o_fifo_256_out_empty);

// logic [15:0] o_fifo_8_out_cnt;
// logic [15:0] o_fifo_128_out_cnt;
// logic [15:0] i_fifo_8_in_cnt;
// logic [15:0] i_fifo_128_in_cnt;
logic [31:0] eth_in_count_dbg;
logic [31:0] eth_out_count_dbg;
logic [31:0] eth_in_stall;
logic [31:0] eth_out_stall;
always_ff @(posedge clk) begin
    if (!rst_n) begin
        // o_fifo_8_out_cnt <= 'b0;
        // o_fifo_128_out_cnt <= 'b0;
        // i_fifo_8_in_cnt <= 'b0;
        // i_fifo_128_in_cnt <= 'b0;
        eth_in_count_dbg <= 'b0;
        eth_out_count_dbg <= 'b0;
        eth_in_stall <= 'b0;
        eth_out_stall <= 'b0;
    end else begin
        // if (o_fifo_8_out_deq) o_fifo_8_out_cnt <= o_fifo_8_out_cnt + 'd1;
        // if (o_fifo_128_out_deq) o_fifo_128_out_cnt <= o_fifo_128_out_cnt + 'd1;
        // if (i_fifo_8_in_valid && (!i_fifo_8_in_full)) i_fifo_8_in_cnt <= i_fifo_8_in_cnt + 'd1;
        // if (i_fifo_128_in_valid && (!i_fifo_128_in_full)) i_fifo_128_in_cnt <= i_fifo_128_in_cnt + 'd1;
        if (eth_in_valid && eth_in_ready) eth_in_count_dbg <= eth_in_count_dbg + 'd1;
        if (eth_out_valid && eth_out_ready) eth_out_count_dbg <= eth_out_count_dbg + 'd1;
        if (!eth_in_ready) eth_in_stall <= eth_in_stall + 'd1;
        if ((!eth_out_ready) && eth_out_valid) eth_out_stall <= eth_out_stall + 'd1;
    end
end

// dbg0
// assign debug_status = {i_fifo_128_out_empty, i_fifo_128_in_ready, i_fifo_8_out_empty, i_fifo_8_in_ready, 
//     o_fifo_128_out_empty, o_fifo_128_in_ready, o_fifo_8_out_empty, o_fifo_8_in_ready, 
//     o_fifo_256_out_empty, state_in, o_fifo_256_in_almFull, state_out,
//     3'd0, eth_in_count, 3'd0, eth_out_count,
// assign debug_status = {eth_out_stall, eth_in_stall, 
// eth_out_count_dbg, eth_in_count_dbg, o_fifo_128_out_cnt, o_fifo_8_out_cnt, i_fifo_128_in_cnt, i_fifo_8_in_cnt};
assign debug_status = {eth_out_stall, eth_in_stall, 
eth_out_count_dbg, eth_in_count_dbg};

endmodule // bfp_adapter
