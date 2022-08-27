/******************************************************************************
* Copyright (c) Intel Corporation - All rights reserved.                      *
* This file is part of the LIBXSMM library.                                   *
*                                                                             *
* For information on the license, see the LICENSE file.                       *
* Further information: https://github.com/libxsmm/libxsmm/                    *
* SPDX-License-Identifier: BSD-3-Clause                                       *
******************************************************************************/
/* Rui Ma (Intel Corp.)
	Mario Doumet (Intel Corp.)
******************************************************************************/
`define DEBUG 1
`define BFP_EN 1

import local_mem_cfg_pkg::*;

typedef enum logic [1:0] {
   RD_IDLE,
	RD_WEIGHTS,
   RD_SLICE,
   RD_NEXT_SLICE
} st_read_t;

typedef enum logic [1:0] {
   WR_IDLE,
   WR_SLICE,
   WR_NEXT_SLICE,
   WR_DONE
} st_write_t;

typedef enum logic [2:0] {
   ETH_IDLE,
	SKIP_WEIGHTS,
   SEND_LOCAL,
   REDUCE,
   REDUCE_OUTPUT,
   FORWARD_OUTPUT,
   OUTPUT_SEND,
   OUTPUT
} st_eth_t;

module all_reduce (
   input logic krn_clk,    // Clock
   input logic krn_reset_n,  // Asynchronous reset active low
   input logic reset_e0_n,

   // CCIP interface
   input logic ccip_clk,
   input logic ccip_rd_rqst_ready,
   output logic ccip_rd_rqst_valid,
   output logic [41:0] ccip_rd_rqst_data,

   input logic ccip_rd_rsp_valid,
   input logic [511:0] ccip_rd_rsp_data,
   output logic ccip_rd_rsp_ready,

   input logic ccip_wr_rqst_ready,
   output logic ccip_wr_rqst_valid,
   output logic [555:0] ccip_wr_rqst_data,   // [555]: cl_len; [554]: sop; [553:512]: address; [511:0]: data

   // ETH interface
   input logic [255:0] cti_ing_data,
   input logic cti_ing_valid,
   output logic ing_cti_ready,

   output logic [255:0] egr_cte_data,
   output logic egr_cte_valid,
   input logic cte_egr_ready,
	
   // MEM interface
   output  t_local_mem_data  avs_writedata,
   input   t_local_mem_data  avs_readdata,
   output  t_local_mem_addr  avs_address,
   input   logic             avs_waitrequest,
   output  logic             avs_write,
   output  logic             avs_read,
   output  t_local_mem_byte_mask  avs_byteenable,
   output  t_local_mem_burst_cnt  avs_burstcount,
   input                     avs_readdatavalid,

   // CSR signals
   input logic start_syn_e0,
   input logic kill_syn_e0,
   input [41:0] mem_addr,
   input [41:0] done_addr,
   input [41:0] weight_addr,
	input logic flags,
   input logic [31:0] total_length,
   input logic [15:0] node_id,
   input logic [15:0] num_node,

   output logic [63:0] lpbk_latency,
   output logic [31:0] num_rqst_sent,
   output logic [47:0] stall_host_in,
   output logic [47:0] stall_host_out,
   output logic [47:0] stall_eth_in,
   output logic [47:0] stall_eth_out,
   output logic [127:0] debug_status
);

   localparam SUM_DELAY = 3'd3;
   localparam BUF_SIZE = 512;
   localparam LG_BUF_SIZE = 9;

   //======================================================================================
   //
   //   Command Queue
   //
   //======================================================================================
   logic [84:0] cmd_fifo_rd_in;
   logic [84:0] cmd_fifo_rd_out;
   logic cmd_fifo_rd_enq;
   logic cmd_fifo_rd_deq;
   logic cmd_fifo_rd_full;
   logic cmd_fifo_rd_empty;

   // show-ahead fifo
	//This fifo holds the requests for synchronization starting at the address at the data port
   fifo #(.DW(85), .AW(8), .DEPTH(256)) cmd_fifo_rd (
      .data  (cmd_fifo_rd_in),
      .wrreq (cmd_fifo_rd_enq & (~cmd_fifo_rd_full)),
      .rdreq (cmd_fifo_rd_deq),
      .clock (krn_clk),
      .sclr  (~krn_reset_n),
      .q     (cmd_fifo_rd_out),
      .full  (cmd_fifo_rd_full),
      .empty (cmd_fifo_rd_empty)
   );

   logic cmd_fifo_eth_in;
   logic cmd_fifo_eth_out;
   logic cmd_fifo_eth_enq;
   logic cmd_fifo_eth_deq;
   logic cmd_fifo_eth_full;
   logic cmd_fifo_eth_empty;

   fifo #(.DW(1), .AW(8), .DEPTH(256)) cmd_fifo_eth (
      .data  (cmd_fifo_eth_in),
      .wrreq (cmd_fifo_eth_enq & (~cmd_fifo_eth_full)),
      .rdreq (cmd_fifo_eth_deq),
      .clock (krn_clk),
      .sclr  (~krn_reset_n),
      .q     (cmd_fifo_eth_out),
      .full  (cmd_fifo_eth_full),
      .empty (cmd_fifo_eth_empty)
   );

   logic [41:0] cmd_fifo_wr_in;
   logic [41:0] cmd_fifo_wr_out;
   logic cmd_fifo_wr_enq;
   logic cmd_fifo_wr_deq;
   logic cmd_fifo_wr_full;
   logic cmd_fifo_wr_empty;

   // show-ahead fifo
   async_fifo #(.DW(42), .AW(8), .DEPTH(256)) cmd_fifo_wr (
      .data    (cmd_fifo_wr_in),
      .wrreq   (cmd_fifo_wr_enq),
      .rdreq   (cmd_fifo_wr_deq),
      .wrclk   (krn_clk),
      .rdclk   (ccip_clk),
      .aclr    (~krn_reset_n),
      .q       (cmd_fifo_wr_out),
      .rdempty (cmd_fifo_wr_empty),
      .wrfull  (cmd_fifo_wr_full)
   );

	//Fifo for the weight update
   logic wu_cmd_fifo_in;
   logic wu_cmd_fifo_out;
   logic wu_cmd_fifo_enq;
   logic wu_cmd_fifo_deq;
   logic wu_cmd_fifo_full;
   logic wu_cmd_fifo_empty;

   async_fifo #(.DW(1), .AW(8), .DEPTH(256)) wu_cmd_fifo (
      .data    (wu_cmd_fifo_in),
      .wrreq   (wu_cmd_fifo_enq),
      .rdreq   (wu_cmd_fifo_deq),
      .wrclk   (krn_clk),
      .rdclk   (ccip_clk),
      .aclr    (~krn_reset_n),
      .q       (wu_cmd_fifo_out),
      .rdempty (wu_cmd_fifo_empty),
      .wrfull  (wu_cmd_fifo_full)
   );

	//Fifo for the weight update
   logic wu_cmd_rd_fifo_in;
   logic wu_cmd_rd_fifo_out;
   logic wu_cmd_rd_fifo_enq;
   logic wu_cmd_rd_fifo_deq;
   logic wu_cmd_rd_fifo_full;
   logic wu_cmd_rd_fifo_empty;

   async_fifo #(.DW(1), .AW(8), .DEPTH(256)) wu_cmd_rd_fifo (
      .data    (wu_cmd_rd_fifo_in),
      .wrreq   (wu_cmd_rd_fifo_enq),
      .rdreq   (wu_cmd_rd_fifo_deq),
      .wrclk   (krn_clk),
      .rdclk   (ccip_clk),
      .aclr    (~krn_reset_n),
      .q       (wu_cmd_rd_fifo_out),
      .rdempty (wu_cmd_rd_fifo_empty),
      .wrfull  (wu_cmd_rd_fifo_full)
   );



   always_ff @(posedge krn_clk)
   begin
      if (~krn_reset_n) begin
         cmd_fifo_rd_enq <= 'b0;
         cmd_fifo_eth_enq <= 'b0;
         cmd_fifo_wr_enq <= 'b0;
			wu_cmd_fifo_enq <= 'b0;
			wu_cmd_rd_fifo_enq <= 'b0;
      end else begin
         if (start_syn_e0) begin
			//The start signal above is sent from the CPU as a pulse to the FPGA. 
			//You can check in ccip_xxxxx.sv how start_e0 is set 
			//to 1 only once when the C++ kernel initiates all_reduce
            cmd_fifo_rd_enq <= 'b1;
            cmd_fifo_eth_enq <= 'b1;
            cmd_fifo_wr_enq <= 'b1;
				wu_cmd_fifo_enq <= 'b1;
				wu_cmd_rd_fifo_enq <= 'b1;
         end else begin
            cmd_fifo_rd_enq <= 'b0;
            cmd_fifo_eth_enq <= 'b0;
            cmd_fifo_wr_enq <= 'b0;
				wu_cmd_fifo_enq <= 'b0;
				wu_cmd_rd_fifo_enq <= 'b0;
         end
      end
      if (start_syn_e0) begin
			//The memory address at which to start reading is enqueued in the Read Cmd fifo
         cmd_fifo_rd_in <= {flags, weight_addr[41:0], mem_addr[41:0]};
         cmd_fifo_eth_in <= flags;
         cmd_fifo_wr_in <= mem_addr;
			wu_cmd_fifo_in <= flags;
			wu_cmd_rd_fifo_in <= flags;
      end
   end

   //======================================================================================
   //
   //   CCIP read logic
   //   The entire vector is first partitioned in blocks. Each block is further 
   //   partioned in #node slices. The size of each slice is BUF_SIZE*64 bytes.
   //	  I believe that at any time, only one block is present on the FPGAs at once
   //   I believe slices are swapped like in the arxiv paper
   //======================================================================================
   	

	logic [41:0] rd_rqst_fifo_in;
   logic [41:0] rd_rqst_fifo_in_n;
   logic [41:0] rd_rqst_fifo_out;
   logic rd_rqst_fifo_enq;
   logic rd_rqst_fifo_enq_n;
   logic rd_rqst_fifo_deq;
   logic rd_rqst_fifo_full;
   logic rd_rqst_fifo_empty;
	logic [7:0] rd_rqst_fifo_usedw;
   logic rd_rqst_fifo_almfull;

   logic [255:0] input_fifo_in;
   logic [255:0] input_fifo_out;
   logic input_fifo_enq;
   logic input_fifo_deq;
   logic input_fifo_full;
   logic input_fifo_empty;

   logic [31:0] rd_rqst_stall;

   // show-ahead fifo
   async_fifo #(.DW(42), .AW(8), .DEPTH(256)) rd_rqst_fifo (
      .data    (rd_rqst_fifo_in),
      .wrreq   (rd_rqst_fifo_enq),
      .rdreq   (rd_rqst_fifo_deq),
      .wrclk   (krn_clk),
      .rdclk   (ccip_clk),
      .aclr    (~reset_e0_n),
      .q       (rd_rqst_fifo_out),
      .rdempty (rd_rqst_fifo_empty),
      .wrfull  (rd_rqst_fifo_full),
		.wrusedw (rd_rqst_fifo_usedw)
   );

   always_ff @(posedge krn_clk) begin
      if (rd_rqst_fifo_usedw < 8'd253) begin
         rd_rqst_fifo_almfull <= 'b0;
      end else begin
         rd_rqst_fifo_almfull <= 'b1;
      end
   end

   // show-ahead fifo
   async_fifo_mixed #(.IDW(512), .ODW(256), .IAW(10), .OAW(11), .DEPTH(1024)) input_fifo (
      .data    (ccip_rd_rsp_data),
      .wrreq   (ccip_rd_rsp_valid),
      .rdreq   (input_fifo_deq),
      .wrclk   (ccip_clk),
      .rdclk   (krn_clk),
      .aclr    (~reset_e0_n),
      .q       (input_fifo_out),
      .rdempty (input_fifo_empty),
      .wrfull  (input_fifo_full)
   );

   assign ccip_rd_rqst_data = rd_rqst_fifo_out;
   assign ccip_rd_rqst_valid = !rd_rqst_fifo_empty;
   assign rd_rqst_fifo_deq = ccip_rd_rqst_ready && (!rd_rqst_fifo_empty);
   assign ccip_rd_rsp_ready = 'b1;

   st_read_t st_read, st_read_n;
   logic [31:0] num_rqst_sent_n;
   logic [31:0] block_offset, block_offset_n;
   logic [31:0] block_offset_next, block_offset_next_n;
   logic [31:0] mem_offset, mem_offset_n;
	logic [31:0] weight_mem_offset, weight_mem_offset_n;
	logic [31:0] weight_max, weight_max_n;
   logic [11:0] rd_rqst_credit, rd_rqst_credit_n;
   logic [41:0] rd_rqst_addr, rd_rqst_addr_n;
   logic [41:0] block_base_addr, block_base_addr_n;
   logic [41:0] block_base_addr_next, block_base_addr_next_n;
   logic [7:0] slice_id, slice_id_n;
   logic [31:0] block_size;

   assign block_size = (num_node << LG_BUF_SIZE);

   always_comb begin
      st_read_n = st_read;
      cmd_fifo_rd_deq = 'b0;
      rd_rqst_fifo_enq_n = rd_rqst_fifo_enq;
      rd_rqst_fifo_in_n = rd_rqst_fifo_in;
      block_base_addr_n = block_base_addr;
      block_base_addr_next_n = block_base_addr_next;
      rd_rqst_addr_n = rd_rqst_addr;
      mem_offset_n = mem_offset;
		weight_mem_offset_n = weight_mem_offset;
		weight_max_n = weight_max;
      num_rqst_sent_n = num_rqst_sent;
      block_offset_n = block_offset;
      block_offset_next_n = block_offset_next;
      slice_id_n = slice_id;

		//Note: The FSM below only inserts addresses that must be read into the rd_rqst_fifo
		//These requests go straight to CCIP and the response come back in the rd_rsp_fifo, aka input_fifo here
      case (st_read)
         RD_IDLE: begin
            num_rqst_sent_n = 'd0;
            block_offset_n = 'd0;
            if (!rd_rqst_fifo_almfull) rd_rqst_fifo_enq_n = 'b0;
            if (!cmd_fifo_rd_empty) begin
					//If we have a request in the read command buffer
               // Start reading data from the host, initialize the offsets and addresses
               // addresses are offsets plus the base address
               // The unit of offsets and addresses is "cache line" (64B)
               cmd_fifo_rd_deq = 'b1;
               mem_offset_n = (node_id << LG_BUF_SIZE);
					weight_mem_offset_n = 'b0;
					weight_max_n = {(total_length[31:9] + 23'd1), 9'b0}; 
               //rd_rqst_addr_n = cmd_fifo_rd_out[84]?cmd_fifo_rd_out[83:42]:cmd_fifo_rd_out[41:0];
               block_offset_next_n = block_size;
               block_base_addr_n = cmd_fifo_rd_out[41:0];
               block_base_addr_next_n = cmd_fifo_rd_out[41:0] + block_offset_next_n;
               if (node_id == num_node - 8'd1) begin
                  slice_id_n = 8'd0;
               end else begin
                  slice_id_n = node_id + 8'd1;
               end
					if (cmd_fifo_rd_out[84] == 1'b1) begin
               	st_read_n = RD_WEIGHTS;
						rd_rqst_addr_n = cmd_fifo_rd_out[83:42];
					end else begin 
						st_read_n = RD_SLICE;
						rd_rqst_addr_n = cmd_fifo_rd_out[41:0] + mem_offset_n;
					end
            end
         end
			RD_WEIGHTS: begin
				if (!rd_rqst_fifo_almfull && (rd_rqst_credit > 12'd15)) begin
					//The read request credit system is used below
					if (weight_mem_offset < weight_max) begin
						rd_rqst_fifo_enq_n = 'b1;
						rd_rqst_fifo_in_n = rd_rqst_addr;
						rd_rqst_addr_n = rd_rqst_addr + 42'd4;
					end else begin
						rd_rqst_fifo_enq_n = 'b0;
						rd_rqst_addr_n = block_base_addr[41:0] + mem_offset;
						st_read_n = RD_SLICE;
					end
					weight_mem_offset_n = weight_mem_offset + 32'd4;
				end else if (!rd_rqst_fifo_almfull) begin
					rd_rqst_fifo_enq_n = 'b0;
				end
			end
         RD_SLICE: begin
            if (!rd_rqst_fifo_almfull && (rd_rqst_credit > 12'd15)) begin
					//The read request credit system is used below
               rd_rqst_fifo_enq_n = 'b1;
               if (mem_offset < total_length) begin
                  rd_rqst_fifo_in_n = rd_rqst_addr;
               end else begin
						//If we are at the end of the data, and it's not a multiple of block size
                  // Padding data, just assign a random valid address
                  rd_rqst_fifo_in_n = block_base_addr;
               end
               // Each request reads 4 cache lines
               rd_rqst_addr_n = rd_rqst_addr + 42'd4;
               num_rqst_sent_n = num_rqst_sent + 32'd1;
               mem_offset_n = mem_offset + 32'd4;
					//Since BUFF_SIZE = #cache_lines = #requests * 4 (since each rqst is 4 cache lines)
					//If we are the second to last request
               if (num_rqst_sent == (BUF_SIZE/4)-2) begin
                  st_read_n = RD_NEXT_SLICE;
               end
            end else if (!rd_rqst_fifo_almfull) begin
               rd_rqst_fifo_enq_n = 'b0;
            end
         end
         RD_NEXT_SLICE: begin
				//We are at the last request of one slice inside the block. We still have to complete all the others
				//The first time we reach here, each node has one different slice in their buffer, and will now start loading a second slice
            if (!rd_rqst_fifo_almfull && (rd_rqst_credit > 12'd15)) begin
               rd_rqst_fifo_enq_n = 'b1;
               if (mem_offset < total_length) begin
                  rd_rqst_fifo_in_n = rd_rqst_addr;
               end else begin
                  // Padding data, just assign a random valid address
                  rd_rqst_fifo_in_n = block_base_addr;
               end
               num_rqst_sent_n = 'd0;
               if (slice_id == num_node - 8'd1) begin
                  slice_id_n = 8'd0;
               end else begin
                  slice_id_n = slice_id + 8'd1;
               end

               if (slice_id != node_id) begin
                  // Next slice in the same block
                  mem_offset_n = block_offset + (slice_id << LG_BUF_SIZE);
                  rd_rqst_addr_n = block_base_addr + (slice_id << LG_BUF_SIZE);
                  st_read_n = RD_SLICE;
               end else begin
                  // New block
                  block_offset_n = block_offset_next;
                  block_offset_next_n = block_offset_next + block_size;
                  block_base_addr_n = block_base_addr_next;
                  block_base_addr_next_n = block_base_addr_next + block_size;
                  mem_offset_n = block_offset_next + (slice_id << LG_BUF_SIZE);
                  rd_rqst_addr_n = block_base_addr_next + (slice_id << LG_BUF_SIZE);
						//Once we're done loading a slice, we then start loading other blocks until we run out
                  if (block_offset_next < total_length) begin
                     st_read_n = RD_SLICE;
                  end else begin
                     st_read_n = RD_IDLE;
                  end
               end
            end else if (!rd_rqst_fifo_almfull) begin
               rd_rqst_fifo_enq_n = 'b0;
            end
         end
      endcase // st_read
   end

   // Credit based backpressure mechanism to make sure the input fifo doesn't overflow
	logic [31:0] rqst_count, deq_count;
   
	always_ff @(posedge krn_clk) begin
      if (!krn_reset_n) begin
         rd_rqst_credit <= 12'd2048;
      end else begin
         if (rd_rqst_fifo_enq && input_fifo_deq) begin
            rd_rqst_credit <= rd_rqst_credit - 12'd7;
         end else if (rd_rqst_fifo_enq ) begin
            rd_rqst_credit <= rd_rqst_credit - 12'd8;
         end else if (input_fifo_deq) begin
            rd_rqst_credit <= rd_rqst_credit + 12'd1;
         end
      end
   end

	always_ff @(posedge krn_clk) begin
      if (!krn_reset_n) begin
			rqst_count <= 32'd0;
			deq_count <= 32'd0;
      end else begin
         if (rd_rqst_fifo_enq) begin
				rqst_count <= rqst_count + 32'd1;
         end 
			if (input_fifo_deq) begin
				deq_count <= deq_count + 32'd1;
         end
      end
   end



   //ccip read rqst (krn_clk domain)
   always_ff @(posedge krn_clk) begin
      if (~krn_reset_n) begin
         st_read <= RD_IDLE;
         rd_rqst_fifo_enq <= 'b0;
         rd_rqst_fifo_in <= 'b0;
         block_base_addr <= 'b0;
         block_base_addr_next <= 'b0;
         rd_rqst_addr <= 'b0;
         mem_offset <= 'b0;
			weight_mem_offset <= 'b0;
			weight_max <= 'b0;
         num_rqst_sent <= 'b0;
         block_offset <= 'b0;
         block_offset_next <= 'b0;
         slice_id <= 'b0;
      end else begin
         st_read <= st_read_n;
         rd_rqst_fifo_enq <= rd_rqst_fifo_enq_n;
         rd_rqst_fifo_in <= rd_rqst_fifo_in_n;
         block_base_addr <= block_base_addr_n;
         block_base_addr_next <= block_base_addr_next_n;
         rd_rqst_addr <= rd_rqst_addr_n;
         mem_offset <= mem_offset_n;
			weight_mem_offset <= weight_mem_offset_n;
			weight_max <= weight_max_n;
         num_rqst_sent <= num_rqst_sent_n;
         block_offset <= block_offset_n;
         block_offset_next <= block_offset_next_n;
         slice_id <= slice_id_n;
      end
   end

   //======================================================================================
   //
   //   Output FIFO
   //
   //======================================================================================
   logic [511:0] output_fifo_in;
   logic [511:0] output_fifo_out;
   logic output_fifo_enq;
   logic output_fifo_deq;
   logic output_fifo_full;
   logic output_fifo_empty;
   fifo #(.DW(512), .AW(10), .DEPTH(1024)) output_fifo (
      .data    (output_fifo_in),
      .wrreq   (output_fifo_enq),
      .rdreq   (output_fifo_deq),
      .clock   (ccip_clk),
      .sclr    (~krn_reset_n),
      .q       (output_fifo_out),
      .empty (output_fifo_empty),
      .full  (output_fifo_full)
   );


   //======================================================================================
   //
   //   Weight update data FIFO
   //
   //======================================================================================
   //This fifo contains the initial weights at the beginning of training
	logic [255:0] wu_weight_fifo_in;
   logic [511:0] wu_weight_fifo_out;
   logic wu_weight_fifo_enq;
   logic wu_weight_fifo_deq;
   logic wu_weight_fifo_full;
   logic wu_weight_fifo_empty;
   async_fifo_mixed #(.IDW(256), .ODW(512), .IAW(11), .OAW(10), .DEPTH(2048)) wu_weight_fifo (
      .data    (wu_weight_fifo_in),
      .wrreq   (wu_weight_fifo_enq),
      .rdreq   (wu_weight_fifo_deq),
      .wrclk   (krn_clk),
      .rdclk   (ccip_clk),
      .aclr    (~reset_e0_n),
      .q       (wu_weight_fifo_out),
      .rdempty (wu_weight_fifo_empty),
      .wrfull  (wu_weight_fifo_full)
   );

	assign wu_weight_fifo_in = input_fifo_out;

	//the wu_gradient fifo will receive the output of the eth FSM below. 
	//All new gradients are sent to the weight update module
   logic [255:0] wu_gradient_fifo_in;
   logic [511:0] wu_gradient_fifo_out;
   logic wu_gradient_fifo_enq;
   logic wu_gradient_fifo_deq;
   logic wu_gradient_fifo_full;
   logic wu_gradient_fifo_empty;
	logic wu_gradient_fifo_almfull;
	logic [10:0] wu_gradient_fifo_usedw;
   async_fifo_mixed #(.IDW(256), .ODW(512), .IAW(11), .OAW(10), .DEPTH(2048)) wu_gradient_fifo (
      .data    (wu_gradient_fifo_in),
      .wrreq   (wu_gradient_fifo_enq),
      .rdreq   (wu_gradient_fifo_deq),
      .wrclk   (krn_clk),
      .rdclk   (ccip_clk),
      .aclr    (~reset_e0_n),
      .q       (wu_gradient_fifo_out),
      .rdempty (wu_gradient_fifo_empty),
      .wrfull  (wu_gradient_fifo_full),
	   .wrusedw (wu_gradient_fifo_usedw)
   );


   always_ff @(posedge krn_clk) begin
      if (wu_gradient_fifo_usedw < 11'd2040) wu_gradient_fifo_almfull <= 'b0;
      else wu_gradient_fifo_almfull <= 'b1;
   end


   reg [31:0] total_length_out;
	reg [15:0] node_id_out;
	reg [15:0] num_node_out;
   //reg [31:0] lrate_out;
	
   alt_sync_regs_m2 #(.WIDTH(32), .DEPTH(2)) s0 (
      .clk(ccip_clk), 
      .din(total_length), 
      .dout(total_length_out)
   );

   alt_sync_regs_m2 #(.WIDTH(16), .DEPTH(2)) s1 (
      .clk(ccip_clk), 
      .din(node_id_minus_1), 
      .dout(node_id_out)
   );


   alt_sync_regs_m2 #(.WIDTH(16), .DEPTH(2)) s2 (
      .clk(ccip_clk), 
      .din(num_node), 
      .dout(num_node_out)
   );


   //alt_sync_regs_m2 #(.WIDTH(32), .DEPTH(2)) (
   //   .clk(ccip_clk), 
   //   .din(lrate), 
   //   .dout(lrate_out)
   //);
	wire [31:0] debug_length;
	wire [31:0] debug_length_two;
	wire [4:0] debug_states;
	wire [7:0] debug_fifos;
   weight_update wu_inst(
      .clk(ccip_clk),
      .reset(!krn_reset_n_out),
      .cmd_in(wu_cmd_fifo_out),
      .cmd_valid(!wu_cmd_fifo_empty),
      .cmd_ready(wu_cmd_fifo_deq),
		.cmd_rd_fifo_out(wu_cmd_rd_fifo_out),
		.cmd_rd_fifo_deq(wu_cmd_rd_fifo_deq),
		.cmd_rd_fifo_empty(wu_cmd_rd_fifo_empty),
      .weight_in(wu_weight_fifo_out),
      .weight_valid(!wu_weight_fifo_empty),
      .weight_ready(wu_weight_fifo_deq),
      .gradient_in(wu_gradient_fifo_out),
      .gradient_valid(!wu_gradient_fifo_empty),
      .gradient_ready(wu_gradient_fifo_deq),
		.result_out(output_fifo_in),
		.result_ready(!output_fifo_full),
		.result_enq(output_fifo_enq),
      .avs_writedata(avs_writedata),
      .avs_readdata(avs_readdata),
      .avs_address(avs_address),
      .avs_waitrequest(avs_waitrequest),
      .avs_write(avs_write),
      .avs_read(avs_read),
      .avs_byteenable(avs_byteenable),
      .avs_burstcount(avs_burstcount),
      .avs_readdatavalid(avs_readdatavalid),
      .total_length(total_length_out),
		.node_id(node_id_out),
		.num_node(num_node_out),
		.debug_length(debug_length),
		.debug_length_two(debug_length_two),
		.debug_states(debug_states),
		.debug_fifos(debug_fifos)
     // .lrate(lrate_out)
   );



   //======================================================================================
   //
   //   Send message FSM
   //
   //======================================================================================
   st_eth_t st_eth, st_eth_n;
   logic [7:0] wait_latency_n, wait_latency;
   logic [63:0] lpbk_latency_n;
   logic [47:0] stall_host_in_n;
   logic [47:0] stall_host_out_n;
   logic [47:0] stall_eth_in_n;
   logic [31:0] stall_eth_in_wait_n, stall_eth_in_wait;
   logic [47:0] stall_eth_out_n;

   logic [255:0] eth_in;
   logic eth_in_valid;
   logic eth_in_ready;
   logic [255:0] eth_out;
   logic eth_out_valid;
   logic eth_out_ready;

   logic [255:0] forward_fifo_in;
   logic [255:0] forward_fifo_out;
   logic forward_fifo_enq;
   logic forward_fifo_deq;
   logic forward_fifo_full;
   logic forward_fifo_empty;
   logic [9:0] forward_fifo_usedw;

   logic [255:0] send_fifo_in;
   logic [255:0] send_fifo_out;
   logic send_fifo_enq;
   logic send_fifo_deq;
   logic send_fifo_full;
   logic send_fifo_empty;
   logic [8:0] send_fifo_usedw;
   logic send_fifo_almfull;

   logic [255:0] iochan_o_data;
   logic iochan_o_valid;
   logic [15:0] send_count, send_count_n;
   logic [31:0] msg_sent, msg_sent_n;
   logic [31:0] forward_id;
   logic [255:0] sum_in0, sum_in0_n;
   logic [255:0] sum_in1, sum_in1_n;
   logic [255:0] sum_out;
	logic [31:0] weights_read, weights_read_n;
	logic [31:0] rd_weight_max, rd_weight_max_n;
   logic sum_valid_in, sum_valid_in_n;
   logic output_valid_in, output_valid_in_n;
   logic output_slct_in, output_slct_in_n;
   logic [255:0] forward_out_vec_in;
   logic [SUM_DELAY-1:0] sum_valid_vec;
   logic [SUM_DELAY-1:0] output_valid_vec;
   logic [SUM_DELAY-1:0] output_slct_vec;
   logic [255:0] forward_out_vec [0:SUM_DELAY-1];
   logic [31:0] block_reduce_size;
   integer i;

`ifdef BFP_EN
   bfp_adapter#(.NUM_FP(16), .MANT_SIZE(8)) bfp_inst(
      .snic_in(eth_out),
      .snic_in_valid(eth_out_valid),
      .snic_in_ready(eth_out_ready),
      .snic_out(eth_in),
      .snic_out_valid(eth_in_valid),
      .snic_out_ready(eth_in_ready),

      .eth_in(cti_ing_data),
      .eth_in_valid(cti_ing_valid),
      .eth_in_ready(ing_cti_ready),
      .eth_out(egr_cte_data),
      .eth_out_valid(egr_cte_valid),
      .eth_out_ready(cte_egr_ready),

      // .debug_status(debug_status),
      .clk(krn_clk),
      .rst_n(krn_reset_n)
   );
`else
   assign eth_in = cti_ing_data;
   assign eth_in_valid = cti_ing_valid;
   assign ing_cti_ready = eth_in_ready;
   assign egr_cte_data = eth_out;
   assign egr_cte_valid = eth_out_valid;
   assign eth_out_ready = cte_egr_ready;
`endif

   fifo #(.DW(256), .AW(10), .DEPTH(1024)) forward_fifo (
      .data  (forward_fifo_in),
      .wrreq (forward_fifo_enq),
      .rdreq (forward_fifo_deq),
      .clock (krn_clk),
      .sclr  (~reset_e0_n),
      .q     (forward_fifo_out),
      .usedw (forward_fifo_usedw),
      .full  (forward_fifo_full),
      .empty (forward_fifo_empty)
   );

   fifo #(.DW(256), .AW(9), .DEPTH(512)) send_fifo (
      .data  (send_fifo_in),
      .wrreq (send_fifo_enq),
      .rdreq (send_fifo_deq),
      .clock (krn_clk),
      .sclr  (~reset_e0_n),
      .q     (send_fifo_out),
      .usedw (send_fifo_usedw),
      .full  (send_fifo_full),
      .empty (send_fifo_empty)
   );

   always_ff @(posedge krn_clk) begin
      if (send_fifo_usedw < 9'd504) send_fifo_almfull <= 'b0;
      else send_fifo_almfull <= 'b1;
   end

   assign forward_fifo_in = eth_in;
   assign forward_fifo_enq = eth_in_valid;
   assign eth_in_ready = ~forward_fifo_full;

   always_comb
   begin
      st_eth_n = st_eth;
      cmd_fifo_eth_deq = 'b0;
      send_count_n = send_count;
      msg_sent_n = msg_sent;
      sum_in0_n = forward_fifo_out;
      sum_in1_n = input_fifo_out;
      sum_valid_in_n = 'b0;
      input_fifo_deq = 'b0;
      forward_fifo_deq = 'b0;
		wu_weight_fifo_enq = 'b0;
      output_valid_in_n = 'b0;
      output_slct_in_n = output_slct_in;
      lpbk_latency_n = lpbk_latency;
      wait_latency_n = wait_latency;
      stall_eth_in_wait_n = stall_eth_in_wait;
      stall_host_in_n = stall_host_in;
      stall_host_out_n = stall_host_out;
      stall_eth_in_n = stall_eth_in;
      stall_eth_out_n = stall_eth_out;
		weights_read_n = weights_read;
		rd_weight_max_n = rd_weight_max;
      case (st_eth)
			//After getting the start signal, go to send_local
         ETH_IDLE: begin
            send_count_n = 'b0;
            if (!cmd_fifo_eth_empty) begin
               cmd_fifo_eth_deq = 'b1;
					if(cmd_fifo_eth_out == 1'b1) begin
               	st_eth_n = SKIP_WEIGHTS;
					end else begin
						st_eth_n = SEND_LOCAL;
					end
               msg_sent_n = block_size;
					weights_read_n = 32'd0;
					rd_weight_max_n = {(total_length[30:9] + 22'd1), 10'b0};
            end
         end
		
			//We skip the first values put into the input fifo as they are the weights
			//They are inserted into the wu_weight_fifo	
			SKIP_WEIGHTS: begin
            // ----
				if (!input_fifo_empty && !wu_weight_fifo_full) begin
					//hi
					weights_read_n = weights_read + 32'd1;
					if (weights_read == rd_weight_max) begin
						st_eth_n = SEND_LOCAL;
						input_fifo_deq = 'b0;
						wu_weight_fifo_enq = 'b0;
					end else begin
						input_fifo_deq = 'b1;
						wu_weight_fifo_enq = 'b1;
					end
				end else begin
					if (weights_read == rd_weight_max) begin
						st_eth_n = SEND_LOCAL;
					end
				end
			end

			//What's happening in this stage is the following:
			
			//Stay in send local until you emptied the input fifo 2*BUFF_SIZE-1 times
			
			//The output of input_fifo (slice from the CPU) is technically being added to the output of forward fifo
			// (forward fifo comes from the eth_in port, from another FPGA)
			// but here sum_in0 is considered to be 0 so it is not added to anything 
			
			//The output is then sent into  send_fifo_in if the sum is considered valid.
			
			//Sum valid is set to 1 below but is delayed in a vector since the sum has a delay to it
			
			//Effectively, we empty 2*BUFF_SIZE-1 times the input_fifo into the send_fifo
			
			//Moreover, notice input fifo has an input width of 512, but an output width of 256, meaning that for each 
			// entry that goes in, there are two entries that go out. Specifically, the 2*BUFF_SIZE-1 
			// only represent one slice (and not two!), because each slice takes up that much space
			
			//Technically send_fifo is of size buffer size, but it is constantly being emptied and sent
			//through eth_out to the neighboring FPGAs

			//In conclusion, this state sends one slice to the next FPGA
         SEND_LOCAL: begin
            // -- performance conter --
            lpbk_latency_n = lpbk_latency + 64'd1;
            if (!send_fifo_almfull && input_fifo_empty)
               stall_host_in_n = stall_host_in + 48'd1;
            if (send_fifo_almfull && (~input_fifo_empty))
               stall_eth_out_n = stall_eth_out + 48'd1;
            // ----
            if (!send_fifo_almfull) begin
               if (!input_fifo_empty) begin
                  if (send_count == 2*BUF_SIZE-1) begin
                     send_count_n = 'd0;
                     st_eth_n = REDUCE;
                  end else begin
                     send_count_n = send_count + 16'd1;
                  end
                  input_fifo_deq = 'b1;
                  sum_in0_n = 'd0;
                  sum_valid_in_n = 1'b1;
               end
            end
         end

			//The next stage mainly empties the input queue and adds it to elements with the forward queue
			// which come from the neighboring FPGA, and stores them in the send buffer to send to the next FPGA
		
			//Notice this is done a total of block_reduce_size times, where block_reduce_size is defined 
			// as (num_nodes-2) << (LG_BUFF_SIZE + 1). First off, since each slice requires 2*BUFF_SIZE,
			// then it makes sense to shift whatever value we use by log of buff_size + 1. Moreover, in total
			// the gradients are added num_nodes-1 times in total. In this state we do the first num_nodes-2, and
			// the last one is kept for the next stage where it's output will actually be valid 

         REDUCE: begin
            // -- performance conter --
            lpbk_latency_n = lpbk_latency + 64'd1;
            if (!send_fifo_almfull && (!input_fifo_empty) && forward_fifo_empty)
               stall_eth_in_n = stall_eth_in + 48'd1;
            if (send_fifo_almfull && (!input_fifo_empty) && (~forward_fifo_empty))
               stall_eth_out_n = stall_eth_out + 48'd1;
            if (!send_fifo_almfull && input_fifo_empty && (~forward_fifo_empty))
               stall_host_in_n = stall_host_in + 48'd1;
            if (forward_fifo_empty) begin
               if (wait_latency[7]) wait_latency_n = 8'd1;
               else wait_latency_n = wait_latency + 8'd1;
            end else begin
               wait_latency_n = 8'd0;
            end
            if (wait_latency[7]) stall_eth_in_wait_n = stall_eth_in_wait + 32'd128;
            // ----
            if (!send_fifo_almfull) begin
               if (!input_fifo_empty && !forward_fifo_empty) begin
                  if (send_count == block_reduce_size - 1) begin
                     send_count_n = 'd0;
                     st_eth_n = REDUCE_OUTPUT;
                  end else begin
                     send_count_n = send_count + 16'd1;
                  end
                  input_fifo_deq = 'b1;
                  forward_fifo_deq = 'b1;
                  sum_valid_in_n = 'b1;
               end
            end
         end


			//In this next stage, we mainly do as in the previous stage but for only 1 slice (2*BUFF_SIZE almost)
			// but this time the output of the sum is valid so we mark it as such. output_slct_in is delayed in 
			// the same way the valid signal is delayed, by sum_delay. Note that the forward_output is also entered
			// in a vector where it is delayed in case it is selected
         REDUCE_OUTPUT: begin
            // -- performance conter --
            lpbk_latency_n = lpbk_latency + 64'd1;
            if (!send_fifo_almfull && (!wu_gradient_fifo_almfull) && (!input_fifo_empty) && forward_fifo_empty)
               stall_eth_in_n = stall_eth_in + 48'd1;
            if (send_fifo_almfull && (!wu_gradient_fifo_almfull) && (!input_fifo_empty) && (~forward_fifo_empty))
               stall_eth_out_n = stall_eth_out + 48'd1;
            if (!send_fifo_almfull && (!wu_gradient_fifo_almfull) && input_fifo_empty && (~forward_fifo_empty))
               stall_host_in_n = stall_host_in + 48'd1;
            if (!send_fifo_almfull && wu_gradient_fifo_almfull && (!input_fifo_empty) && (~forward_fifo_empty))
               stall_host_out_n = stall_host_out + 48'd1;
            // ----
            if (!send_fifo_almfull && !wu_gradient_fifo_almfull) begin
               if (!input_fifo_empty && !forward_fifo_empty) begin
                  if (send_count == 2*BUF_SIZE - 1) begin
                     send_count_n = 'd0;
                     st_eth_n = FORWARD_OUTPUT;
                  end else begin
                     send_count_n = send_count + 16'd1;
                  end
                  input_fifo_deq = 'b1;
                  forward_fifo_deq = 'b1;
                  sum_valid_in_n = 'b1;
                  output_valid_in_n = 'b1;
                  output_slct_in_n = 'b1; // select sum
               end
            end
         end

			//At this point, we reduced everything and don't need to sum anymore. We thus set the second operand
			// of the sum to 0, and stop dequeuing from the input fifo as we don't need it anymore. We still save
			// the output as valid and coming from the forward fifo (from the neighboring FPGA)
			//We do this block_reduce_size times, or equivalently for num_nodes - 2 slices, as the first time is 
			// already done in the previous state, and the last time will be done next. Then we go to output if 
			// we're done or output_send if there's more to do

         FORWARD_OUTPUT: begin
            // -- performance conter --
            lpbk_latency_n = lpbk_latency + 64'd1;
            if (!send_fifo_almfull && (!wu_gradient_fifo_almfull) && forward_fifo_empty)
               stall_eth_in_n = stall_eth_in + 48'd1;
            if (send_fifo_almfull && (!wu_gradient_fifo_almfull) && (~forward_fifo_empty))
               stall_eth_out_n = stall_eth_out + 48'd1;
            if (!send_fifo_almfull && wu_gradient_fifo_almfull && (~forward_fifo_empty))
               stall_host_out_n = stall_host_out + 48'd1;
            // ----
            if (!send_fifo_almfull && !wu_gradient_fifo_almfull) begin
               if (!forward_fifo_empty) begin
                  if (send_count == block_reduce_size - 1) begin
                     send_count_n = 'd0;
                     if (msg_sent >= total_length) begin
                        st_eth_n = OUTPUT;
                     end else begin
                        msg_sent_n = msg_sent + block_size;
                        st_eth_n = OUTPUT_SEND;
                     end
                  end else begin
                     send_count_n = send_count + 16'd1;
                  end
                  sum_in1_n = 'd0;
                  forward_fifo_deq = 'b1;
                  sum_valid_in_n = 'b1;
                  output_valid_in_n = 'b1;
                  output_slct_in_n = 'b0; // select forward
               end
            end
         end

			//This next state is a mix of the previous stage and send local
			// First it sends the local weights from the input to the next FPGA by adding them to 0
			// It also marks the output from the previous FPGA as valid because we're sharing the last slice
			//Finally we go back to the REDUCE state and go again

         OUTPUT_SEND: begin
            // -- performance conter --
            lpbk_latency_n = lpbk_latency + 64'd1;
            if (!send_fifo_almfull && (!wu_gradient_fifo_almfull) && (!input_fifo_empty) && forward_fifo_empty)
               stall_eth_in_n = stall_eth_in + 48'd1;
            if (send_fifo_almfull && (!wu_gradient_fifo_almfull) && (!input_fifo_empty) && (~forward_fifo_empty))
               stall_eth_out_n = stall_eth_out + 48'd1;
            if (!send_fifo_almfull && (!wu_gradient_fifo_almfull) && input_fifo_empty && (~forward_fifo_empty))
               stall_host_in_n = stall_host_in + 48'd1;
            if (!send_fifo_almfull && wu_gradient_fifo_almfull && (!input_fifo_empty) && (~forward_fifo_empty))
               stall_host_out_n = stall_host_out + 48'd1;
            // ----
            if (!send_fifo_almfull && !wu_gradient_fifo_almfull) begin
               if (!input_fifo_empty && !forward_fifo_empty) begin
                  if (send_count == 2*BUF_SIZE-1) begin
                     send_count_n = 'd0;
                     st_eth_n = REDUCE;
                  end else begin
                     send_count_n = send_count + 16'd1;
                  end
                  input_fifo_deq = 'b1;
                  forward_fifo_deq = 'b1;
                  sum_in0_n = 'd0;
                  sum_valid_in_n = 1'b1;
                  output_valid_in_n = 'b1;
                  output_slct_in_n = 'b0; // select forward
               end
            end
         end

			//In the next state we simply mark the last slice as valid and as coming from the 'forward' fifo
			// and then go back to being idle.
         OUTPUT: begin
            // -- performance conter --
            lpbk_latency_n = lpbk_latency + 64'd1;
            if ((!wu_gradient_fifo_almfull) && forward_fifo_empty)
               stall_eth_in_n = stall_eth_in + 48'd1;
            if (wu_gradient_fifo_almfull && (~forward_fifo_empty))
               stall_host_out_n = stall_host_out + 48'd1;
            // ----
            if (!wu_gradient_fifo_almfull) begin
               if (!forward_fifo_empty) begin
                  if (send_count == 2*BUF_SIZE-1) begin
                     send_count_n = 'd0;
                     st_eth_n = ETH_IDLE;
                  end else begin
                     send_count_n = send_count + 16'd1;
                  end
                  forward_fifo_deq = 'b1;
                  output_valid_in_n = 'b1;
                  output_slct_in_n = 'b0; // select forward
               end
            end
         end
      endcase
   end

   genvar k;
   generate
      for (k = 0; k < 8; k=k+1) begin
         fadd fadd_inst (
            .a      (sum_in0[k*32+:32]),      //   input,  width = 32,      a.a
            .areset (~krn_reset_n), //   input,   width = 1, areset.reset
            .b      (sum_in1[k*32+:32]),      //   input,  width = 32,      b.b
            .clk    (krn_clk),    //   input,   width = 1,    clk.clk
            .q      (sum_out[k*32+:32])       //  output,  width = 32,      q.q
         );
      end
   endgenerate

   always_ff @(posedge krn_clk)
   begin
      if (~krn_reset_n) begin
         st_eth <= ETH_IDLE;
         send_count <= 'd0;
         msg_sent <= 'd0;
         sum_valid_in <= 'b0;
         output_valid_in <= 'b0;
         output_slct_in <= 'b0;
         sum_valid_vec <= 'b0;
         output_valid_vec <= 'b0;
         output_slct_vec <= 'b0;
         lpbk_latency <= 'd0;
         stall_eth_in <= 'b0;
         stall_eth_out <= 'b0;
         stall_host_in <= 'b0;
         stall_host_out <= 'b0;
         wait_latency <= 'b0;
         stall_eth_in_wait <= 'b0;
			weights_read <= 'b0;
			rd_weight_max <= 'b0;
      end else begin
         st_eth <= st_eth_n;
         send_count <= send_count_n;
         msg_sent <= msg_sent_n;
         sum_valid_in <= sum_valid_in_n;
         output_valid_in <= output_valid_in_n;
         output_slct_in <= output_slct_in_n;
         sum_valid_vec[SUM_DELAY-1] <= sum_valid_in;
         sum_valid_vec[SUM_DELAY-2:0] <= sum_valid_vec[SUM_DELAY-1:1];
         output_valid_vec[SUM_DELAY-1] <= output_valid_in;
         output_valid_vec[SUM_DELAY-2:0] <= output_valid_vec[SUM_DELAY-1:1];
         output_slct_vec[SUM_DELAY-1] <= output_slct_in;
         output_slct_vec[SUM_DELAY-2:0] <= output_slct_vec[SUM_DELAY-1:1];
         lpbk_latency <= lpbk_latency_n;
         stall_eth_in <= stall_eth_in_n;
         stall_eth_out <= stall_eth_out_n;
         stall_host_in <= stall_host_in_n;
         stall_host_out <= stall_host_out_n;
         wait_latency <= wait_latency_n;
         stall_eth_in_wait <= stall_eth_in_wait_n;
			weights_read <= weights_read_n;
			rd_weight_max <= rd_weight_max_n;
      end
      sum_in0 <= sum_in0_n;
      sum_in1 <= sum_in1_n;
      forward_out_vec_in <= forward_fifo_out;
      forward_out_vec[SUM_DELAY-1] <= forward_out_vec_in;
      forward_out_vec[0:SUM_DELAY-2] <= forward_out_vec[1:SUM_DELAY-1]; 
      block_reduce_size <= (num_node - 2) << (LG_BUF_SIZE + 1);
   end

   always_ff @(posedge krn_clk) begin
      if (~krn_reset_n) begin
         send_fifo_enq <= 'b0;
      end else begin
         if (sum_valid_vec[0]) begin
            send_fifo_enq <= 'b1;
            send_fifo_in <= sum_out;
         end else begin
            send_fifo_enq <= 'b0;
         end
      end
   end

   always_ff @(posedge krn_clk) begin
      if (~krn_reset_n) begin
			wu_gradient_fifo_enq <= 'b0;
      end else begin
         if (output_valid_vec[0]) begin
				wu_gradient_fifo_enq <= 'b1;
            if (output_slct_vec[0]) begin
					wu_gradient_fifo_in <= sum_out;
            end else begin
					wu_gradient_fifo_in <= forward_out_vec[0];
            end
         end else begin
				wu_gradient_fifo_enq <= 'b0;
         end
      end
   end

   //======================================================================================
   //
   //   ETH out logic
   //
   //======================================================================================
   always_ff @(posedge krn_clk) begin
      if (~krn_reset_n) begin
         iochan_o_valid <= 'b0;
      end else if (eth_out_ready) begin
         if (!send_fifo_empty) begin
            iochan_o_data <= send_fifo_out;
            iochan_o_valid <= 'b1;
         end else begin
            iochan_o_valid <= 'b0;
         end
      end
   end
   assign send_fifo_deq = eth_out_ready && (!send_fifo_empty);
   assign eth_out = iochan_o_data;
   assign eth_out_valid = iochan_o_valid;

   //======================================================================================
   //
   //   CCIP write logic
   //
   //======================================================================================
   st_write_t st_write, st_write_n;
   logic [41:0] write_addr, write_addr_n;
   logic [31:0] num_write_sent, num_write_sent_n;
   logic [31:0] wr_block_offset, wr_block_offset_n;
   logic [31:0] wr_block_offset_next, wr_block_offset_next_n;
   logic [31:0] wr_mem_offset, wr_mem_offset_n;
   logic [41:0] wr_block_base_addr, wr_block_base_addr_n;
   logic [41:0] wr_block_base_addr_next, wr_block_base_addr_next_n;
   logic [7:0] wr_slice_id, wr_slice_id_n;
   logic ccip_wr_rqst_valid_n;
   logic [555:0] ccip_wr_rqst_data_n;
   logic [1:0] wr_sop, wr_sop_n;
   logic [15:0] node_id_ccip;
   logic [15:0] num_node_ccip;
   logic [15:0] node_id_minus_1;
   logic [31:0] block_size_ccip;
   logic [31:0] total_length_ccip;
   logic [2:0] done_id, done_id_n;

   assign node_id_minus_1 = (node_id_ccip == 16'd0) ? (num_node_ccip - 16'd1) : (node_id_ccip - 16'd1);

   alt_sync_regs_m2 #(.WIDTH(32), .DEPTH(2)) sync0(
      .clk(ccip_clk),
      .din(block_size),
      .dout(block_size_ccip)
   );

   alt_sync_regs_m2 #(.WIDTH(16), .DEPTH(2)) sync1(
      .clk(ccip_clk),
      .din(node_id),
      .dout(node_id_ccip)
   );

   alt_sync_regs_m2 #(.WIDTH(16), .DEPTH(2)) sync2(
      .clk(ccip_clk),
      .din(num_node),
      .dout(num_node_ccip)
   );

   alt_sync_regs_m2 #(.WIDTH(32), .DEPTH(2)) sync3(
      .clk(ccip_clk),
      .din(total_length),
      .dout(total_length_ccip)
   );

   always_comb begin
      st_write_n = st_write;
      cmd_fifo_wr_deq = 'b0;
      output_fifo_deq = 'b0;
      ccip_wr_rqst_valid_n = ccip_wr_rqst_valid;
      ccip_wr_rqst_data_n = ccip_wr_rqst_data;
      wr_block_base_addr_n = wr_block_base_addr;
      wr_block_base_addr_next_n = wr_block_base_addr_next;
      write_addr_n = write_addr;
      wr_mem_offset_n = wr_mem_offset;
      num_write_sent_n = num_write_sent;
      wr_block_offset_n = wr_block_offset;
      wr_block_offset_next_n = wr_block_offset_next;
      wr_slice_id_n = wr_slice_id;
      done_id_n = done_id;
      wr_sop_n = wr_sop;
      case (st_write)

			//This next stage sets up the write addresses for writeback of the final gradients.
			// FPGA 3 obtains as a first final output the slice 2. In general FPGA x gets slice x-1 % num_nodes
			// That is why the wr_mem_offset is taken as node_id_minus_1 shifted by slice size
			// We also note that the output fifo is back to having full length data (512) so a slice is only
			// BUFF_SIZE requests, and not 2 times that as in the main FSM seen above.
         WR_IDLE: begin
            num_write_sent_n = 'd0;
            wr_block_offset_n = 'd0;
            if (ccip_wr_rqst_ready) ccip_wr_rqst_valid_n = 'b0;
            if (!cmd_fifo_wr_empty) begin
               cmd_fifo_wr_deq = 'b1;
               wr_mem_offset_n = (node_id_minus_1 << LG_BUF_SIZE);
               write_addr_n = cmd_fifo_wr_out[41:0] + wr_mem_offset_n;
               wr_sop_n = 'b0;
               wr_block_offset_next_n = block_size_ccip;
               wr_block_base_addr_n = cmd_fifo_wr_out[41:0];
               wr_block_base_addr_next_n = cmd_fifo_wr_out[41:0] + wr_block_offset_next_n;
               wr_slice_id_n = node_id_ccip;
               st_write_n = WR_SLICE;
            end
         end

			//In this next state we send the data via the ccip output ports of the all_reduce module
			// the data from the output fifo (if it's not empty of course) is sent via ccip_wr_rqst_data along with other signals
			// the valid signal is assert and the data dequeued from the output fifo. The write address is then incremented
			// We go to write the final slice in the next stage
         WR_SLICE: begin
            if (ccip_wr_rqst_ready) begin
               if (!output_fifo_empty) begin
                  ccip_wr_rqst_data_n = {1'b1, (~(|wr_sop)), write_addr, output_fifo_out};
                  output_fifo_deq = 'b1;
                  wr_sop_n = wr_sop + 2'd1;
                  if (wr_mem_offset < total_length_ccip) begin
                     ccip_wr_rqst_valid_n = 'b1;
                  end else begin
                     ccip_wr_rqst_valid_n = 'b0;
                  end
                  write_addr_n = write_addr + 42'd1;
                  num_write_sent_n = num_write_sent + 32'd1;
                  wr_mem_offset_n = wr_mem_offset + 32'd1;
                  if (num_write_sent == BUF_SIZE-2) begin
                     st_write_n = WR_NEXT_SLICE;
                  end
               end else begin
                  ccip_wr_rqst_valid_n = 'b0;
               end
            end
         end

			// In this next state we also write back to memory like above for the last write of the slice
			// The main difference is that if there are more blocks or more slices then we go back up to WR_SLICE

         WR_NEXT_SLICE: begin
            if (ccip_wr_rqst_ready) begin
               if (!output_fifo_empty) begin
                  ccip_wr_rqst_data_n = {1'b1, 1'b0, write_addr, output_fifo_out};
                  output_fifo_deq = 'b1;
                  num_write_sent_n = 'd0;
                  wr_sop_n = 'b0;
                  if (wr_mem_offset < total_length_ccip) begin
                     ccip_wr_rqst_valid_n = 'b1;
                  end else begin
                     ccip_wr_rqst_valid_n = 'b0;
                  end
                  if (wr_slice_id == num_node_ccip - 8'd1) begin
                     wr_slice_id_n = 8'd0;
                  end else begin
                     wr_slice_id_n = wr_slice_id + 8'd1;
                  end

                  if (wr_slice_id != node_id_minus_1) begin
                     wr_mem_offset_n = wr_block_offset + (wr_slice_id << LG_BUF_SIZE);
                     write_addr_n = wr_block_base_addr + (wr_slice_id << LG_BUF_SIZE);
                     st_write_n = WR_SLICE;
                  end else begin
                     // New block
                     wr_block_offset_n = wr_block_offset_next;
                     wr_block_offset_next_n = wr_block_offset_next + block_size_ccip;
                     wr_block_base_addr_n = wr_block_base_addr_next;
                     wr_block_base_addr_next_n = wr_block_base_addr_next + block_size_ccip;
                     wr_mem_offset_n = wr_block_offset_next + (wr_slice_id << LG_BUF_SIZE);
                     if (wr_block_offset_next < total_length_ccip) begin
                        write_addr_n = wr_block_base_addr_next + (wr_slice_id << LG_BUF_SIZE);
                        st_write_n = WR_SLICE;
                     end else begin
                        write_addr_n = done_addr + done_id;
                        st_write_n = WR_DONE;
                     end
                  end
               end else begin
                  ccip_wr_rqst_valid_n = 'b0;
               end
            end
         end
         WR_DONE: begin
            if (ccip_wr_rqst_ready) begin
               ccip_wr_rqst_data_n = {1'b0, 1'b1, write_addr, 512'd1};
               ccip_wr_rqst_valid_n = 'b1;
               st_write_n = WR_IDLE;
               done_id_n = done_id + 3'd1;
            end
         end
      endcase // st_read
   end

   reg krn_reset_n_out;
   alt_sync_regs_m2 #(.WIDTH(1), .DEPTH(2)) reset_sync(
      .clk(ccip_clk), 
      .din(krn_reset_n), 
      .dout(krn_reset_n_out)
   );

   always_ff @(posedge ccip_clk) begin
      if (~krn_reset_n_out) begin
         st_write <= WR_IDLE;
         ccip_wr_rqst_valid <= 'b0;
         num_write_sent <= 'd0;
         wr_slice_id <= 'd0;
         done_id <= 'd0;
         rd_rqst_stall <= 'd0;
         wr_sop <= 'b0;
      end else begin
         st_write <= st_write_n;
         ccip_wr_rqst_valid <= ccip_wr_rqst_valid_n;
         num_write_sent <= num_write_sent_n;
         wr_slice_id <= wr_slice_id_n;
         done_id <= done_id_n;
         wr_sop <= wr_sop_n;
         if (ccip_rd_rqst_valid && (!ccip_rd_rqst_ready)) begin
            rd_rqst_stall <= rd_rqst_stall + 32'd1;
         end
      end
      ccip_wr_rqst_data <= ccip_wr_rqst_data_n;
      wr_block_base_addr <= wr_block_base_addr_n;
      wr_block_base_addr_next <= wr_block_base_addr_next_n;
      write_addr <= write_addr_n;
      wr_mem_offset <= wr_mem_offset_n;
      wr_block_offset <= wr_block_offset_n;
      wr_block_offset_next <= wr_block_offset_next_n;
   end

`ifdef DEBUG
   assign debug_status = {20'd0, rd_rqst_credit, rqst_count, deq_count, 
      debug_fifos, 
      eth_out_ready, eth_in_valid,  debug_states, wu_gradient_fifo_empty, 
      output_fifo_empty, wu_gradient_fifo_almfull, st_write, eth_in_ready, eth_out_valid, send_fifo_empty, send_fifo_almfull, 
      forward_fifo_empty, st_eth, input_fifo_empty, rd_rqst_fifo_almfull, st_read};
`endif

endmodule
