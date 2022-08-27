/******************************************************************************
* Copyright (c) Intel Corporation - All rights reserved.                      *
* This file is part of the LIBXSMM library.                                   *
*                                                                             *
* For information on the license, see the LICENSE file.                       *
* Further information: https://github.com/libxsmm/libxsmm/                    *
* SPDX-License-Identifier: BSD-3-Clause                                       *
******************************************************************************/
/*   Mario Doumet (Intel Corp.)
******************************************************************************/



`include "platform_if.vh"
`include "cci_mpf_if.vh"
`include "cci_mpf_platform.vh"
`include "cci_mpf_app_conf_default.vh"
`include "cci_csr_if.vh"


import local_mem_cfg_pkg::*;

typedef enum logic[2:0] { WR_IDLE,
                          MEM_INIT,
                          WR_SLICE,
                          WR_NEXT_SLICE
} wr_state_t;

typedef enum logic[1:0] { IDLE,
                          LOAD_SLICE,
                          LOAD_NEXT_SLICE
} rd_state_t;

module weight_update #(parameter BUF_SIZE=512, parameter LG_BUF_SIZE=9) (
   input clk,
   input reset,
   
   //Wires responsible for communicating with all_reduce.sv
   input cmd_in,
   output logic cmd_ready,
   input cmd_valid,

   input cmd_rd_fifo_out,
   output logic cmd_rd_fifo_deq,
   input cmd_rd_fifo_empty,

   input [511:0] weight_in,
   output logic weight_ready,
   input weight_valid,

   input [511:0] gradient_in,
   output logic gradient_ready,
   input gradient_valid,

   output logic [511:0] result_out,
   input result_ready,
   output logic result_enq, 

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

   input [31:0] total_length,
   input [15:0] node_id,
   input [15:0] num_node,
   output [31:0] debug_length,
      output [31:0] debug_length_two,
   output [4:0] debug_states,
   output [7:0] debug_fifos
);

   //======================================================================================
   //
   //   Mem write FIFO
   //
   //======================================================================================
   logic [543:0] mem_wr_fifo_in;
   logic [543:0] mem_wr_fifo_out;
   logic mem_wr_fifo_enq;
   logic mem_wr_fifo_enq_n;
   logic mem_wr_fifo_deq;
   logic mem_wr_fifo_full;
   logic [3:0] mem_wr_fifo_usedw;
   logic mem_wr_fifo_empty;

   fifo #(.DW(544), .AW(4), .DEPTH(16)) mem_wr_fifo(
      .data(mem_wr_fifo_in),
      .wrreq(mem_wr_fifo_enq),
      .rdreq(mem_wr_fifo_deq),
      .clock(clk),
      .sclr(reset),
      .q(mem_wr_fifo_out),
      .full(mem_wr_fifo_full),
      .empty(mem_wr_fifo_empty),
      .usedw(mem_wr_fifo_usedw)
   );


   //======================================================================================
   //
   //   Mem read FIFO
   //
   //======================================================================================
   logic [31:0] mem_rd_rqst_fifo_in;
   logic [31:0] mem_rd_rqst_fifo_out;
   logic mem_rd_rqst_fifo_enq;
   logic mem_rd_rqst_fifo_deq;
   logic mem_rd_rqst_fifo_full;
   logic mem_rd_rqst_fifo_empty;

   fifo #(.DW(32), .AW(9), .DEPTH(512)) mem_rd_rqst_fifo(
      .data(mem_rd_rqst_fifo_in),
      .wrreq(mem_rd_rqst_fifo_enq),
      .rdreq(mem_rd_rqst_fifo_deq),
      .clock(clk),
      .sclr(reset),
      .q(mem_rd_rqst_fifo_out),
      .full(mem_rd_rqst_fifo_full),
      .empty(mem_rd_rqst_fifo_empty)
   );


   logic [511:0] mem_rd_rsp_fifo_in;
   logic [511:0] mem_rd_rsp_fifo_out;
   logic mem_rd_rsp_fifo_enq;
   logic mem_rd_rsp_fifo_deq;
   logic mem_rd_rsp_fifo_full;
   logic mem_rd_rsp_fifo_empty;
   logic [8:0] mem_rd_rsp_fifo_usedw;
   logic mem_rd_rsp_fifo_almfull;
   logic [10:0] rd_rqst_credit;
   logic [31:0] num_rqst_sent;
   logic [31:0] rd_rqst_addr;
   logic [31:0] weight_start_addr;

   fifo #(.DW(512), .AW(9), .DEPTH(512)) mem_rd_rsp_fifo(
      .data(mem_rd_rsp_fifo_in),
      .wrreq(mem_rd_rsp_fifo_enq),
      .rdreq(mem_rd_rsp_fifo_deq),
      .clock(clk),
      .sclr(reset),
      .q(mem_rd_rsp_fifo_out),
      .usedw(mem_rd_rsp_fifo_usedw),
      .full(mem_rd_rsp_fifo_full),
      .empty(mem_rd_rsp_fifo_empty)
   );

   // mem read response
   assign mem_rd_rsp_fifo_enq = avs_readdatavalid;
   assign mem_rd_rsp_fifo_in = avs_readdata;

   always_ff @(posedge clk) begin
      if (mem_rd_rsp_fifo_usedw < 9'd504) mem_rd_rsp_fifo_almfull <= 'b0;
      else mem_rd_rsp_fifo_almfull <= 'b1;
   end


   //======================================================================================
   //
   //   Mem read/write logic
   //
   //======================================================================================
   logic [31:0] enq_count;
   //This next block (8 lines) is just debugging purposes, feel free to remove it   
   always_ff @ (posedge clk) begin
      if (reset) begin
         enq_count <= 'b0;
      end else begin
         if (new_weights_valid == 1'b1) begin
            enq_count <= enq_count + 32'd1;
         end
      end
   end


   //We've defined 3 fifos for memory. The rd_rqst, rd_rsp and mem_wr
   //What happens in the always block below is the following:
   //If avs_waitrequest is low, we can queue up a read or a write and wait for avs_waitrequest to be low
   //We take the writes from the mem_wr_fifo and the reads from the mem_rd_rqst_fifo  
   //I don't think the implementation is optimal, but I wanted to be safe with memory writes. 
   //Another implementation can be found below with faster times but this one here works for sure

   assign avs_burstcount = 12'd1;
   assign avs_byteenable = {64{1'b1}};
 
   logic started, started_n; 
   logic rw, rw_n;
   logic avs_write_n, avs_read_n;
   logic [31:0] avs_address_n;

   assign mem_wr_fifo_deq = (!avs_waitrequest && (!mem_wr_fifo_empty) && rw && started);
   assign mem_rd_rqst_fifo_deq = (!avs_waitrequest && (!mem_rd_rqst_fifo_empty) && !rw && started);
   
   always_comb begin
      avs_address_n = avs_address;
      avs_write_n = avs_write;
      avs_read_n = avs_read;
      started_n = started;
      rw_n = rw;
      if (!started) begin
         if (!mem_wr_fifo_empty) begin
            avs_write_n = 'b1;
            started_n = 'b1;
            rw_n = 'b1;
            avs_address_n = t_local_mem_addr'(mem_wr_fifo_out[543:512]);   
         end else if (!mem_rd_rqst_fifo_empty) begin
            avs_read_n = 'b1;
            avs_address_n = t_local_mem_addr'(mem_wr_fifo_out[543:512]);
            started_n = 'b1;
            rw_n = 'b0;
         end
      end else if (started && !avs_waitrequest) begin
         avs_write_n = 'b0;
         avs_read_n = 'b0;
         started_n = 'b0;
      end
   end
            

 
   always_ff @(posedge clk) begin
      if (reset) begin
         avs_address <= 'd0;
         avs_write <= 'b0;
         avs_read <= 'b0;
         started <= 'b0;
         rw <= 'b0;
      end else begin
         avs_address <= avs_address_n;
         avs_write <= avs_write_n;
         avs_read <= avs_read_n;
         started <= started_n;
         rw <= rw_n;
      end
   end

//  //This is the alternative way mentioned earlier          
//   assign mem_wr_fifo_deq = (~avs_waitrequest && (!mem_wr_fifo_empty));
//   assign mem_rd_rqst_fifo_deq = (~avs_waitrequest && (!mem_rd_rqst_fifo_empty) && (mem_wr_fifo_empty) && !mem_rd_rsp_fifo_almfull);
//
//   logic [31:0] enq_count;
//   always_ff @ (posedge clk) begin
//      if (reset) begin
//         enq_count <= 'b0;
//      end else begin
//         if (new_weights_valid == 1'b1) begin
//            enq_count <= enq_count + 32'd1;
//         end
//      end
//   end
//
//   always_ff @(posedge clk) begin
//      if (reset) begin
//         avs_address <= 'd0;
//         avs_write <= 'b0;
//         avs_read <= 'b0;
//         mem_count <= 'd0;
//      end else begin
//         if (~avs_waitrequest) begin
//            avs_write <= 'b0;
//            avs_read <= 'b0;
//            avs_writedata <= mem_wr_fifo_out[511:0];
//            if ((!mem_wr_fifo_empty) && (!mem_rd_rqst_fifo_empty)) begin
//                  avs_write <= 'b1;
//                  avs_address <= t_local_mem_addr'(mem_wr_fifo_out[543:512]);
//            end else if (!mem_wr_fifo_empty) begin
//               avs_write <= 'b1;
//               avs_address <= t_local_mem_addr'(mem_wr_fifo_out[543:512]);
//            end else if (!mem_rd_rqst_fifo_empty && !mem_rd_rsp_fifo_almfull) begin
//               avs_read <= 'b1;
//               avs_address <= t_local_mem_addr'(mem_rd_rqst_fifo_out);
//            end
//         end
//      end
//   end
//


/////////////////////////////////////////////////////////////
//             Mem read FSM                                //
/////////////////////////////////////////////////////////////

//In this part, our goal is to request the correct weights from memory
//We load them in the correct slice order to facilitate the weight update
//Notice the load update is exactly that of the read in all_reduce but with the node_id
//passed being decreased by 1 (because that's the first slice that's ready)

//These two signals are for debugging purposes
   logic [31:0] count_count_n, count_count;
   logic [31:0] save_count_n, save_count;
   

   rd_state_t rd_state, rd_state_n;
   logic [31:0] block_size;
   assign block_size = (num_node << LG_BUF_SIZE);

   logic [31:0] block_offset_next_n, block_offset_next;
   logic [31:0] mem_rd_slice_addr_n, mem_rd_slice_addr;
   logic [31:0] block_base_addr_n, block_base_addr;
   logic [31:0] block_base_addr_next_n, block_base_addr_next;
   logic [31:0] rd_slice_count_n, rd_slice_count;
   logic [7:0] slice_id_n, slice_id;
   

   assign mem_rd_rqst_fifo_in = {mem_rd_slice_addr[25:0], 6'b0};

   always_comb begin
      rd_state_n = rd_state;
      cmd_rd_fifo_deq = 'b0;
      block_offset_next_n = block_offset_next;
      mem_rd_slice_addr_n = mem_rd_slice_addr;
      block_base_addr_n = block_base_addr;
      block_base_addr_next_n = block_base_addr_next;
      rd_slice_count_n = rd_slice_count;
      slice_id_n = slice_id;
      count_count_n = count_count;
      save_count_n = save_count;
      mem_rd_rqst_fifo_enq = 'b0;
      case (rd_state)
         IDLE:
         begin
            if (!cmd_rd_fifo_empty) begin
               cmd_rd_fifo_deq = 'b1;
               rd_state_n = LOAD_SLICE;
            end

            //Signals for reading weights from memory
            block_offset_next_n = block_size;
            mem_rd_slice_addr_n = (node_id << LG_BUF_SIZE);
            block_base_addr_n = 'b0;
            block_base_addr_next_n = block_offset_next_n;
            rd_slice_count_n = 'b0;
            count_count_n = 'b0;
            if (node_id == num_node - 8'd1) begin
               slice_id_n = 8'd0;
            end else begin
               slice_id_n = node_id + 8'd1;
            end
         end

         LOAD_SLICE:
         begin
            if (!mem_rd_rqst_fifo_full) begin
               count_count_n = count_count + 32'd1;
               mem_rd_rqst_fifo_enq = 'b1;
               if (mem_rd_slice_addr < total_length) begin
                  mem_rd_slice_addr_n = mem_rd_slice_addr + 32'd1;
               end else begin
                  mem_rd_slice_addr_n = block_base_addr;
               end
               rd_slice_count_n = rd_slice_count + 32'd1;   
               if (rd_slice_count == BUF_SIZE -2) begin
                  rd_state_n = LOAD_NEXT_SLICE;
               end
            end
         end

         LOAD_NEXT_SLICE:
         begin
            if (!mem_rd_rqst_fifo_full) begin
               count_count_n = count_count + 32'd1;
               mem_rd_rqst_fifo_enq = 'b1;
               //The correct value for mem_rd_slice_addr has already been set so there's no point to set it now.
               rd_slice_count_n = 'b0; 
               if (slice_id == num_node - 8'd1) begin
                  slice_id_n = 8'd0;
               end else begin
                  slice_id_n = slice_id + 8'd1;
               end
               
               if (slice_id != node_id) begin
                  //Next slice in the same block
                  mem_rd_slice_addr_n = block_base_addr + (slice_id << LG_BUF_SIZE);
                  rd_state_n = LOAD_SLICE;
               end else begin
                  block_offset_next_n = block_offset_next + block_size;
                  block_base_addr_n = block_base_addr_next;
                  block_base_addr_next_n = block_base_addr_next + block_size;
                  mem_rd_slice_addr_n = block_base_addr_next + (slice_id << LG_BUF_SIZE);
                  if (block_offset_next < total_length) begin
                     rd_state_n = LOAD_SLICE;
                  end else begin
                     rd_state_n = IDLE;
                     save_count_n = save_count + 32'd1;
                  end
               end
            end
         end

      endcase
   end

   always_ff @ (posedge clk) begin
      if (reset) begin
         rd_state <= IDLE;
         block_offset_next <= 'b0;
         mem_rd_slice_addr <= 'b0;
         block_base_addr <= 'b0;
         block_base_addr_next <= 'b0;
         rd_slice_count <= 'b0;
         slice_id <= 'b0;
         save_count <= 'b0;
         count_count <= 'b0;
      end else begin
         rd_state <= rd_state_n;
         block_offset_next <= block_offset_next_n;
         mem_rd_slice_addr <= mem_rd_slice_addr_n;
         block_base_addr <= block_base_addr_n;
         block_base_addr_next <= block_base_addr_next_n;
         rd_slice_count <= rd_slice_count_n;
         slice_id <= slice_id_n;
         save_count <= save_count_n;
         count_count <= count_count_n;
      end
   end



/////////////////////////////////////////////////////////////
//             Weight update                               //
/////////////////////////////////////////////////////////////

//In this part, we take the weights and gradients from gradient_in and
// mem_rd_rsp_fifo_out and compute the new weights. 

   localparam WU_DELAY = 4;
   logic [WU_DELAY-1:0] wu_valid_vec;
   logic [511:0] new_weights;
   logic new_weights_valid;

   //We compute 16 weights at a time
   //The learning rate used is -0.1, feel free to change it below 
   genvar k;
   generate
      for (k = 0; k < 16; k=k+1) begin : fused_multiply_add
         ffma ffma_inst (
            .a      (32'hBDCCCCCD),
            .areset (reset),
            .b      (gradient_in[k*32+:32]),
            .c      (mem_rd_rsp_fifo_out[k*32+:32]),
            .clk    (clk),
            .q      (new_weights[k*32+:32])
         );
      end
   endgenerate
   

   //We need to insert the outputs of the computation into a FIFO
   logic [511:0] new_weights_fifo_in;
   logic [511:0] new_weights_fifo_out;
   logic new_weights_fifo_enq;
   logic new_weights_fifo_deq;
   logic new_weights_fifo_full;
   logic [4:0] new_weights_fifo_usedw;
   logic new_weights_fifo_empty;
   logic new_weights_fifo_almfull;

   fifo #(.DW(512), .AW(5), .DEPTH(32)) new_weights_fifo(
      .data(new_weights_fifo_in),
      .wrreq(new_weights_fifo_enq),
      .rdreq(new_weights_fifo_deq),
      .clock(clk),
      .sclr(reset),
      .q(new_weights_fifo_out),
      .full(new_weights_fifo_full),
      .empty(new_weights_fifo_empty),
      .usedw(new_weights_fifo_usedw)
   );

   always_ff @(posedge clk) begin
      if (new_weights_fifo_usedw < 5'd26) new_weights_fifo_almfull <= 'b0;
      else new_weights_fifo_almfull <= 'b1;
   end

   //we tie the input of the FIFO to the output of the fused multiply add
   assign new_weights_fifo_in = new_weights;
   assign result_out = new_weights_fifo_out;

   //We dequeue the input FIFOs if both arguments are valid and the output fifo has space
   assign gradient_ready = gradient_valid && !mem_rd_rsp_fifo_empty && !new_weights_fifo_almfull;
   assign mem_rd_rsp_fifo_deq = gradient_valid && !mem_rd_rsp_fifo_empty && !new_weights_fifo_almfull;
   
   //Since the weight update takes WU_DELAY cycles to complete, we need to insert the valid bit into a vector that gets shifted
   assign new_weights_valid = gradient_valid && !mem_rd_rsp_fifo_empty && !new_weights_fifo_almfull;

   always_ff @ (posedge clk) begin
      if (reset) begin
         wu_valid_vec <= 'b0;
      end else begin
         wu_valid_vec[WU_DELAY-2:0] <= wu_valid_vec[WU_DELAY-1:1];
         wu_valid_vec[WU_DELAY-1] <= new_weights_valid;
      end
   end
   
   assign new_weights_fifo_enq = wu_valid_vec[0];


/////////////////////////////////////////////////////////////
//             Mem write FSM                               //
/////////////////////////////////////////////////////////////


//We queue the updated weights back to memory in the same order that they were read
   wr_state_t wr_state, wr_state_n;
   logic [31:0] mem_wr_addr_n, mem_wr_addr;
   logic [31:0] total_send_count_n, total_send_count;
   logic [31:0] max_length_n, max_length; 
   logic [31:0] wr_block_offset_next_n, wr_block_offset_next;
   logic [31:0] wr_block_base_addr_n, wr_block_base_addr;
   logic [31:0] wr_block_base_addr_next_n, wr_block_base_addr_next;
   logic [7:0] wr_slice_id_n, wr_slice_id;
   logic [31:0] save_n, save;
   logic [31:0] cnt, cnt_n;
   logic mem_wr_fifo_slct;
   assign debug_length = total_send_count;   
   assign debug_states = {rd_state, wr_state};
   assign debug_fifos = {cmd_rd_fifo_empty, mem_wr_fifo_empty, mem_rd_rqst_fifo_empty, mem_rd_rsp_fifo_empty, new_weights_fifo_empty, mem_wr_fifo_full, gradient_valid, new_weights_fifo_almfull};
   assign mem_wr_fifo_in[543:512] = {mem_wr_addr[25:0], 6'b0};
   assign mem_wr_fifo_in[511:0] = mem_wr_fifo_slct?(weight_in):(new_weights_fifo_out);
   assign debug_length_two = max_length;
   always_comb begin
      wr_state_n = wr_state;
      cmd_ready = 'b0;
      wr_block_offset_next_n = wr_block_offset_next;
      wr_block_base_addr_n = wr_block_base_addr;
      wr_block_base_addr_next_n = wr_block_base_addr_next;
      wr_slice_id_n = wr_slice_id;
      weight_ready = 'b0;
      mem_wr_addr_n = mem_wr_addr;
      mem_wr_fifo_enq = 'b0;
      mem_wr_fifo_slct = 'b0;
      new_weights_fifo_deq = 'b0;
      total_send_count_n = total_send_count;
      max_length_n = max_length;
      result_enq = 'b0;
      cnt_n = cnt;
      save_n = save;

      case (wr_state)
         WR_IDLE:
         begin
            if (cmd_valid) begin
               cmd_ready = 'b1;
               if (cmd_in) begin
                  wr_state_n = MEM_INIT;
               end else begin 
                  wr_state_n = WR_SLICE;
               end
            end

            //Signals for the memory initialization
            mem_wr_addr_n = (node_id << LG_BUF_SIZE);
            max_length_n = {(total_length[31:9] + 23'd1), 9'b0};
            total_send_count_n = 'b0;
            wr_block_offset_next_n = block_size;
            wr_block_base_addr_n = 'b0;
            wr_block_base_addr_next_n = wr_block_offset_next_n;
            if (node_id == num_node - 8'd1) begin
               wr_slice_id_n = 8'd0;
            end else begin
               wr_slice_id_n = node_id + 8'd1;
            end
         end

         //We select the write data as being the output of the ffma, not the weight in buffer
         //Enqueue it to the result buffer (send it back to all_reduce)
         WR_SLICE:
         begin
            if (!mem_wr_fifo_full && !new_weights_fifo_empty && result_ready) begin
               mem_wr_fifo_enq = 'b1;
               result_enq = 'b1;
               mem_wr_fifo_slct = 'b0;
               new_weights_fifo_deq = 'b1;
               mem_wr_addr_n = mem_wr_addr + 32'd1;
               total_send_count_n = total_send_count + 32'd1;  
               if (total_send_count == BUF_SIZE -2) begin
                  wr_state_n = WR_NEXT_SLICE;
               end
            end
         end

         WR_NEXT_SLICE:
         begin
            if (!mem_wr_fifo_full && !new_weights_fifo_empty && result_ready) begin
               mem_wr_fifo_enq = 'b1;
               result_enq = 'b1;
               //The correct value for mem_wr_addr has already been set so there's no point to set it now.
               mem_wr_fifo_slct = 'b0;
               new_weights_fifo_deq = 'b1;
               total_send_count_n = 'b0;  
               if (wr_slice_id == num_node - 8'd1) begin
                  wr_slice_id_n = 8'd0;
               end else begin
                  wr_slice_id_n = wr_slice_id + 8'd1;
               end
               
               if (wr_slice_id != node_id) begin
                  //Next slice in the same block
                  mem_wr_addr_n = wr_block_base_addr + (wr_slice_id << LG_BUF_SIZE);
                  wr_state_n = WR_SLICE;
                     save_n = save + 32'd1;
               end else begin
                  wr_block_offset_next_n = wr_block_offset_next + block_size;
                  wr_block_base_addr_n = wr_block_base_addr_next;
                  wr_block_base_addr_next_n = wr_block_base_addr_next + block_size;
                  mem_wr_addr_n = wr_block_base_addr_next + (wr_slice_id << LG_BUF_SIZE);
                  if (wr_block_offset_next < total_length) begin
                     wr_state_n = WR_SLICE;
                  end else begin
                     wr_state_n = WR_IDLE;
                  end
               end
            end
         end

         //In this state we send all the initial weights to memory
         MEM_INIT:
         begin
            if (weight_valid && !mem_wr_fifo_full) begin
                  mem_wr_fifo_slct = 'b1;
                  mem_wr_fifo_enq = 'b1;
                  weight_ready = 'b1;
                  mem_wr_addr_n = mem_wr_addr + 32'd1;
                  total_send_count_n = total_send_count + 32'd1;  
               end
            if (total_send_count == max_length) begin
               wr_state_n = WR_SLICE;
               total_send_count_n = 'b0;
            end 
         end   

      endcase
   end

   always_ff @ (posedge clk) begin
      if (reset) begin
         wr_state <= WR_IDLE;
         wr_block_offset_next <= 'b0;
         wr_block_base_addr <= 'b0;
         wr_block_base_addr_next <= 'b0;
         wr_slice_id <= 'b0;
         mem_wr_addr <= 'b0;
         total_send_count <= 'b0;
         max_length <= 'b0;
         save <= 'b0;
      end else begin
         wr_state <= wr_state_n;
         wr_block_offset_next <= wr_block_offset_next_n;
         wr_block_base_addr <= wr_block_base_addr_n;
         wr_block_base_addr_next <= wr_block_base_addr_next_n;
         wr_slice_id <= wr_slice_id_n;
         mem_wr_addr <= mem_wr_addr_n;
         total_send_count <= total_send_count_n;
         max_length <= max_length_n;
         save <= save_n;
      end
   end




endmodule
