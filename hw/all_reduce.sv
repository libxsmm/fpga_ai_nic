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
`define DEBUG 0
// `define BFP_EN 0

typedef enum logic [1:0] {
   RD_IDLE,
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

   // CSR signals
   input logic start_syn_e0,
   input logic kill_syn_e0,
   input [41:0] mem_addr,
   input [41:0] done_addr,
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
   logic [41:0] cmd_fifo_rd_in;
   logic [41:0] cmd_fifo_rd_out;
   logic cmd_fifo_rd_enq;
   logic cmd_fifo_rd_deq;
   logic cmd_fifo_rd_full;
   logic cmd_fifo_rd_empty;

   // show-ahead fifo
   fifo #(.DW(42), .AW(8), .DEPTH(256)) cmd_fifo_rd (
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

   always_ff @(posedge krn_clk)
   begin
      if (~krn_reset_n) begin
         cmd_fifo_rd_enq <= 'b0;
         cmd_fifo_eth_enq <= 'b0;
         cmd_fifo_wr_enq <= 'b0;
      end else begin
         if (start_syn_e0) begin
            cmd_fifo_rd_enq <= 'b1;
            cmd_fifo_eth_enq <= 'b1;
            cmd_fifo_wr_enq <= 'b1;
         end else begin
            cmd_fifo_rd_enq <= 'b0;
            cmd_fifo_eth_enq <= 'b0;
            cmd_fifo_wr_enq <= 'b0;
         end
      end
      if (start_syn_e0) begin
         cmd_fifo_rd_in <= mem_addr;
         cmd_fifo_eth_in <= 1'b1;
         cmd_fifo_wr_in <= mem_addr;
      end
   end

   //======================================================================================
   //
   //   CCIP read logic
   //   The entire vector is first partitioned in blocks. Each block is further 
   //   partioned in #node slices. The size of each slice is BUF_SIZE*64 bytes.
   //
   //======================================================================================
   logic [41:0] rd_rqst_fifo_in;
   logic [41:0] rd_rqst_fifo_in_n;
   logic [41:0] rd_rqst_fifo_out;
   logic rd_rqst_fifo_enq;
   logic rd_rqst_fifo_enq_n;
   logic rd_rqst_fifo_deq;
   logic rd_rqst_fifo_full;
   logic rd_rqst_fifo_empty;

   logic [255:0] input_fifo_in;
   logic [255:0] input_fifo_out;
   logic input_fifo_enq;
   logic input_fifo_deq;
   logic input_fifo_full;
   logic input_fifo_empty;

   logic [31:0] rd_rqst_stall;

   // show-ahead fifo
   async_fifo #(.DW(42), .AW(3), .DEPTH(8)) rd_rqst_fifo (
      .data    (rd_rqst_fifo_in),
      .wrreq   (rd_rqst_fifo_enq),
      .rdreq   (rd_rqst_fifo_deq),
      .wrclk   (krn_clk),
      .rdclk   (ccip_clk),
      .aclr    (~reset_e0_n),
      .q       (rd_rqst_fifo_out),
      .rdempty (rd_rqst_fifo_empty),
      .wrfull  (rd_rqst_fifo_full)
   );

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
      num_rqst_sent_n = num_rqst_sent;
      block_offset_n = block_offset;
      block_offset_next_n = block_offset_next;
      slice_id_n = slice_id;
      case (st_read)
         RD_IDLE: begin
            num_rqst_sent_n = 'd0;
            block_offset_n = 'd0;
            if (!rd_rqst_fifo_full) rd_rqst_fifo_enq_n = 'b0;
            if (!cmd_fifo_rd_empty) begin
               // Start reading data from the host, initialize the offsets and addresses
               // addresses are offsets plus the base address
               // The unit of offsets and addresses is "cache line" (64B)
               cmd_fifo_rd_deq = 'b1;
               mem_offset_n = (node_id << LG_BUF_SIZE);
               rd_rqst_addr_n = cmd_fifo_rd_out[41:0] + mem_offset_n;
               block_offset_next_n = block_size;
               block_base_addr_n = cmd_fifo_rd_out[41:0];
               block_base_addr_next_n = cmd_fifo_rd_out[41:0] + block_offset_next_n;
               if (node_id == num_node - 8'd1) begin
                  slice_id_n = 8'd0;
               end else begin
                  slice_id_n = node_id + 8'd1;
               end
               st_read_n = RD_SLICE;
            end
         end
         RD_SLICE: begin
            if (!rd_rqst_fifo_full && (rd_rqst_credit > 12'd15)) begin
               rd_rqst_fifo_enq_n = 'b1;
               if (mem_offset < total_length) begin
                  rd_rqst_fifo_in_n = rd_rqst_addr;
               end else begin
                  // Padding data, just assign a random valid address
                  rd_rqst_fifo_in_n = block_base_addr;
               end
               // Each request reads 4 cache lines
               rd_rqst_addr_n = rd_rqst_addr + 42'd4;
               num_rqst_sent_n = num_rqst_sent + 32'd1;
               mem_offset_n = mem_offset + 32'd4;
               if (num_rqst_sent == (BUF_SIZE/4)-2) begin
                  st_read_n = RD_NEXT_SLICE;
               end
            end else if (!rd_rqst_fifo_full) begin
               rd_rqst_fifo_enq_n = 'b0;
            end
         end
         RD_NEXT_SLICE: begin
            if (!rd_rqst_fifo_full && (rd_rqst_credit > 12'd15)) begin
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
                  if (block_offset_next < total_length) begin
                     st_read_n = RD_SLICE;
                  end else begin
                     st_read_n = RD_IDLE;
                  end
               end
            end else if (!rd_rqst_fifo_full) begin
               rd_rqst_fifo_enq_n = 'b0;
            end
         end
      endcase // st_read
   end

   // Credit based backpressure mechanism to make sure the input fifo doesn't overflow
   always_ff @(posedge krn_clk) begin
      if (!krn_reset_n) begin
         rd_rqst_credit <= 12'd2048;
      end else begin
         if (rd_rqst_fifo_enq && (!rd_rqst_fifo_full) && input_fifo_deq) begin
            rd_rqst_credit <= rd_rqst_credit - 12'd7;
         end else if (rd_rqst_fifo_enq && (!rd_rqst_fifo_full)) begin
            rd_rqst_credit <= rd_rqst_credit - 12'd8;
         end else if (input_fifo_deq) begin
            rd_rqst_credit <= rd_rqst_credit + 12'd1;
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
   logic [255:0] output_fifo_in;
   logic [511:0] output_fifo_out;
   logic output_fifo_enq;
   logic output_fifo_deq;
   logic output_fifo_full;
   logic output_fifo_almfull;
   logic output_fifo_empty;
   logic [10:0] output_fifo_usedw;
   async_fifo_mixed #(.IDW(256), .ODW(512), .IAW(11), .OAW(10), .DEPTH(2048)) output_fifo (
      .data    (output_fifo_in),
      .wrreq   (output_fifo_enq),
      .rdreq   (output_fifo_deq),
      .wrclk   (krn_clk),
      .rdclk   (ccip_clk),
      .aclr    (~reset_e0_n),
      .q       (output_fifo_out),
      .rdempty (output_fifo_empty),
      .wrfull  (output_fifo_full),
      .wrusedw (output_fifo_usedw)
   );

   always_ff @(posedge krn_clk) begin
      if (output_fifo_usedw < 11'd2040) output_fifo_almfull <= 'b0;
      else output_fifo_almfull <= 'b1;
   end

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

// `ifdef BFP_EN
//    bfp_adapter#(.NUM_FP(16), .MANT_SIZE(8)) bfp_inst(
//       .snic_in(eth_out),
//       .snic_in_valid(eth_out_valid),
//       .snic_in_ready(eth_out_ready),
//       .snic_out(eth_in),
//       .snic_out_valid(eth_in_valid),
//       .snic_out_ready(eth_in_ready),

//       .eth_in(cti_ing_data),
//       .eth_in_valid(cti_ing_valid),
//       .eth_in_ready(ing_cti_ready),
//       .eth_out(egr_cte_data),
//       .eth_out_valid(egr_cte_valid),
//       .eth_out_ready(cte_egr_ready),

//       // .debug_status(debug_status),
//       .clk(krn_clk),
//       .rst_n(krn_reset_n)
//    );
// `else
   assign eth_in = cti_ing_data;
   assign eth_in_valid = cti_ing_valid;
   assign ing_cti_ready = eth_in_ready;
   assign egr_cte_data = eth_out;
   assign egr_cte_valid = eth_out_valid;
   assign eth_out_ready = cte_egr_ready;
// `endif

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
      output_valid_in_n = 'b0;
      output_slct_in_n = output_slct_in;
      lpbk_latency_n = lpbk_latency;
      wait_latency_n = wait_latency;
      stall_eth_in_wait_n = stall_eth_in_wait;
      stall_host_in_n = stall_host_in;
      stall_host_out_n = stall_host_out;
      stall_eth_in_n = stall_eth_in;
      stall_eth_out_n = stall_eth_out;
      case (st_eth)
         ETH_IDLE: begin
            send_count_n = 'b0;
            if (!cmd_fifo_eth_empty) begin
               cmd_fifo_eth_deq = 'b1;
               st_eth_n = SEND_LOCAL;
               msg_sent_n = block_size;
            end
         end
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
         REDUCE_OUTPUT: begin
            // -- performance conter --
            lpbk_latency_n = lpbk_latency + 64'd1;
            if (!send_fifo_almfull && (!output_fifo_almfull) && (!input_fifo_empty) && forward_fifo_empty)
               stall_eth_in_n = stall_eth_in + 48'd1;
            if (send_fifo_almfull && (!output_fifo_almfull) && (!input_fifo_empty) && (~forward_fifo_empty))
               stall_eth_out_n = stall_eth_out + 48'd1;
            if (!send_fifo_almfull && (!output_fifo_almfull) && input_fifo_empty && (~forward_fifo_empty))
               stall_host_in_n = stall_host_in + 48'd1;
            if (!send_fifo_almfull && output_fifo_almfull && (!input_fifo_empty) && (~forward_fifo_empty))
               stall_host_out_n = stall_host_out + 48'd1;
            // ----
            if (!send_fifo_almfull && !output_fifo_almfull) begin
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
         FORWARD_OUTPUT: begin
            // -- performance conter --
            lpbk_latency_n = lpbk_latency + 64'd1;
            if (!send_fifo_almfull && (!output_fifo_almfull) && forward_fifo_empty)
               stall_eth_in_n = stall_eth_in + 48'd1;
            if (send_fifo_almfull && (!output_fifo_almfull) && (~forward_fifo_empty))
               stall_eth_out_n = stall_eth_out + 48'd1;
            if (!send_fifo_almfull && output_fifo_almfull && (~forward_fifo_empty))
               stall_host_out_n = stall_host_out + 48'd1;
            // ----
            if (!send_fifo_almfull && !output_fifo_almfull) begin
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
         OUTPUT_SEND: begin
            // -- performance conter --
            lpbk_latency_n = lpbk_latency + 64'd1;
            if (!send_fifo_almfull && (!output_fifo_almfull) && (!input_fifo_empty) && forward_fifo_empty)
               stall_eth_in_n = stall_eth_in + 48'd1;
            if (send_fifo_almfull && (!output_fifo_almfull) && (!input_fifo_empty) && (~forward_fifo_empty))
               stall_eth_out_n = stall_eth_out + 48'd1;
            if (!send_fifo_almfull && (!output_fifo_almfull) && input_fifo_empty && (~forward_fifo_empty))
               stall_host_in_n = stall_host_in + 48'd1;
            if (!send_fifo_almfull && output_fifo_almfull && (!input_fifo_empty) && (~forward_fifo_empty))
               stall_host_out_n = stall_host_out + 48'd1;
            // ----
            if (!send_fifo_almfull && !output_fifo_almfull) begin
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
         OUTPUT: begin
            // -- performance conter --
            lpbk_latency_n = lpbk_latency + 64'd1;
            if ((!output_fifo_almfull) && forward_fifo_empty)
               stall_eth_in_n = stall_eth_in + 48'd1;
            if (output_fifo_almfull && (~forward_fifo_empty))
               stall_host_out_n = stall_host_out + 48'd1;
            // ----
            if (!output_fifo_almfull) begin
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
         output_fifo_enq <= 'b0;
      end else begin
         if (output_valid_vec[0]) begin
            output_fifo_enq <= 'b1;
            if (output_slct_vec[0]) begin
               output_fifo_in <= sum_out;
            end else begin
               output_fifo_in <= forward_out_vec[0];
            end
         end else begin
            output_fifo_enq <= 'b0;
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

   sync_regs #(.WIDTH(32)) sync0(
      .clk(ccip_clk),
      .din(block_size),
      .dout(block_size_ccip)
   );

   sync_regs #(.WIDTH(16)) sync1(
      .clk(ccip_clk),
      .din(node_id),
      .dout(node_id_ccip)
   );

   sync_regs #(.WIDTH(16)) sync2(
      .clk(ccip_clk),
      .din(num_node),
      .dout(num_node_ccip)
   );

   sync_regs #(.WIDTH(32)) sync3(
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
   sync_regs #(.WIDTH(1)) reset_sync(
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
   assign debug_status = {stall_eth_in_wait, rd_rqst_stall, num_write_sent, 
      wr_slice_id, 
      7'd0, ccip_wr_rqst_ready, 
      output_fifo_empty, output_fifo_almfull, st_write, 2'b00, send_fifo_empty, send_fifo_almfull, 
      forward_fifo_empty, st_eth, input_fifo_empty, rd_rqst_fifo_full, st_read};
`endif

endmodule
