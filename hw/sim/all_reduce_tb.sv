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
`timescale 1ps/1ps

module all_reduce_tb;

   logic krn_clk = 1'b0;
   logic krn_reset_n;
   logic reset_e0_n;
   logic ccip_clk = 1'b0;
   logic start_syn_e0 = 1'b0;
   logic kill_syn_e0 = 1'b0;

   always begin
      #1600 krn_clk = ~krn_clk;
   end

   always begin
      #5000 ccip_clk = ~ccip_clk;
   end
   localparam HOST_MEM_DELAY = 20;
   localparam CCIP_EFFICIENCY = 90; // 100% efficiency is 51.2 Gbps
   localparam ETH_DELAY = 20;
   localparam ETH_EFFICIENCY = 50; // 100% efficiency is 80 Gbps
   localparam TOTAL_LENGTH = 2048;
   localparam DONE_ADDR = TOTAL_LENGTH;

   //======================================================================================
   //
   //   Node 0
   //
   //======================================================================================
   logic n0_c0TxAlmFull;
   logic n0_ccip_rd_rqst_valid;
   logic n0_ccip_rd_rqst_ready;
   logic [41:0] n0_ccip_rd_rqst_data;
   logic [41:0] n0_rd_addr;
   logic n0_ccip_rd_rsp_valid;
   logic [511:0] n0_ccip_rd_rsp_data;
   logic n0_ccip_rd_rsp_ready;
   logic n0_c1TxAlmFull;
   logic n0_ccip_wr_rqst_valid;
   logic [553:0] n0_ccip_wr_rqst_data;
   logic [255:0] n0_cti_ing_data;
   logic n0_cti_ing_valid;
   logic n0_ing_cti_ready;
   logic [255:0] n0_egr_cte_data;
   logic n0_egr_cte_valid;
   logic n0_cte_egr_ready;

   // CCIP MEM model
   reg [511:0] n0_host_mem [0:TOTAL_LENGTH];
   logic [31:0] n0_rand_num2;
   logic [31:0] n0_rand_num3;

   logic [511:0] n0_ccip_rd_rsp_data_vec [0:HOST_MEM_DELAY-1];
   logic [HOST_MEM_DELAY-1:0] n0_ccip_rd_rsp_valid_vec = 0;
   logic [1:0] n0_burst = 0;
   assign n0_ccip_rd_rsp_valid = n0_ccip_rd_rsp_valid_vec[0];
   assign n0_ccip_rd_rsp_data = n0_ccip_rd_rsp_data_vec[0];
   assign n0_ccip_rd_rqst_ready = (!n0_c0TxAlmFull) && (n0_burst == 2'd0);
   always @(posedge ccip_clk) begin
      n0_rand_num2 = $urandom_range(0, 100);
      n0_rand_num3 = $urandom_range(0, 100);
      if (n0_rand_num2 < CCIP_EFFICIENCY) begin
         n0_c0TxAlmFull <= 'b0;
      end else begin
         n0_c0TxAlmFull <= 'b1;
      end
      if (n0_rand_num3 < CCIP_EFFICIENCY) begin
         n0_c1TxAlmFull <= 'b0;
      end else begin
         n0_c1TxAlmFull <= 'b1;
      end
      n0_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1] <= 'b0;
      if (n0_burst == 0) begin
         if (!n0_c0TxAlmFull && n0_ccip_rd_rqst_valid) begin
            n0_burst <= n0_burst + 2'd1;
            n0_rd_addr <= n0_ccip_rd_rqst_data;
            n0_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1] <= 'b1;
            n0_ccip_rd_rsp_data_vec[HOST_MEM_DELAY-1] <= n0_host_mem[{n0_ccip_rd_rqst_data[41:2], n0_burst}];
         end
      end else begin
         n0_burst <= n0_burst + 2'd1;
         n0_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1] <= 'b1;
         n0_ccip_rd_rsp_data_vec[HOST_MEM_DELAY-1] <= n0_host_mem[{n0_rd_addr[41:2], n0_burst}];
      end
      n0_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-2:0] <= n0_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1:1];
      n0_ccip_rd_rsp_data_vec[0:HOST_MEM_DELAY-2] <= n0_ccip_rd_rsp_data_vec[1:HOST_MEM_DELAY-1];

      if (!n0_c1TxAlmFull && n0_ccip_wr_rqst_valid) begin
         n0_host_mem[n0_ccip_wr_rqst_data[553:512]] <= n0_ccip_wr_rqst_data[511:0];
      end
   end

   all_reduce all_reduce_inst0(
      .krn_clk(krn_clk),
      .krn_reset_n(krn_reset_n),
      .reset_e0_n(reset_e0_n),
      .ccip_clk(ccip_clk),
      .ccip_rd_rqst_ready(n0_ccip_rd_rqst_ready),
      .ccip_rd_rqst_valid(n0_ccip_rd_rqst_valid),
      .ccip_rd_rqst_data(n0_ccip_rd_rqst_data),
      .ccip_rd_rsp_valid(n0_ccip_rd_rsp_valid),
      .ccip_rd_rsp_data(n0_ccip_rd_rsp_data),
      .ccip_rd_rsp_ready(n0_ccip_rd_rsp_ready),
      .ccip_wr_rqst_ready(!n0_c1TxAlmFull),
      .ccip_wr_rqst_valid(n0_ccip_wr_rqst_valid),
      .ccip_wr_rqst_data(n0_ccip_wr_rqst_data),
      .cti_ing_data(n0_cti_ing_data),
      .cti_ing_valid(n0_cti_ing_valid),
      .ing_cti_ready(n0_ing_cti_ready),
      .egr_cte_data(n0_egr_cte_data),
      .egr_cte_valid(n0_egr_cte_valid),
      .cte_egr_ready(n0_cte_egr_ready),

      .start_syn_e0(start_syn_e0),
      .kill_syn_e0(kill_syn_e0),
      .mem_addr(42'd0),
      .done_addr(DONE_ADDR),
      .total_length(TOTAL_LENGTH),
      .node_id(16'd0),
      .num_node(16'd3)
   );

   //======================================================================================
   //
   //   Node 1
   //
   //======================================================================================
   logic n1_c0TxAlmFull;
   logic n1_ccip_rd_rqst_valid;
   logic n1_ccip_rd_rqst_ready;
   logic [41:0] n1_ccip_rd_rqst_data;
   logic [41:0] n1_rd_addr;
   logic n1_ccip_rd_rsp_valid;
   logic [511:0] n1_ccip_rd_rsp_data;
   logic n1_ccip_rd_rsp_ready;
   logic n1_c1TxAlmFull;
   logic n1_ccip_wr_rqst_valid;
   logic [553:0] n1_ccip_wr_rqst_data;
   logic [255:0] n1_cti_ing_data;
   logic n1_cti_ing_valid;
   logic n1_ing_cti_ready;
   logic [255:0] n1_egr_cte_data;
   logic n1_egr_cte_valid;
   logic n1_cte_egr_ready;

   // CCIP MEM model
   reg [511:0] n1_host_mem [0:TOTAL_LENGTH];
   logic [31:0] n1_rand_num2;
   logic [31:0] n1_rand_num3;

   logic [511:0] n1_ccip_rd_rsp_data_vec [0:HOST_MEM_DELAY-1];
   logic [HOST_MEM_DELAY-1:0] n1_ccip_rd_rsp_valid_vec = 0;
   logic [1:0] n1_burst = 0;
   assign n1_ccip_rd_rsp_valid = n1_ccip_rd_rsp_valid_vec[0];
   assign n1_ccip_rd_rsp_data = n1_ccip_rd_rsp_data_vec[0];
   assign n1_ccip_rd_rqst_ready = (!n1_c0TxAlmFull) && (n1_burst == 2'd0);
   always @(posedge ccip_clk) begin
      n1_rand_num2 = $urandom_range(0, 100);
      n1_rand_num3 = $urandom_range(0, 100);
      if (n1_rand_num2 < CCIP_EFFICIENCY) begin
         n1_c0TxAlmFull <= 'b0;
      end else begin
         n1_c0TxAlmFull <= 'b1;
      end
      if (n1_rand_num3 < CCIP_EFFICIENCY) begin
         n1_c1TxAlmFull <= 'b0;
      end else begin
         n1_c1TxAlmFull <= 'b1;
      end
      n1_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1] <= 'b0;
      if (n1_burst == 0) begin
         if (!n1_c0TxAlmFull && n1_ccip_rd_rqst_valid) begin
            n1_burst <= n1_burst + 2'd1;
            n1_rd_addr <= n1_ccip_rd_rqst_data;
            n1_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1] <= 'b1;
            n1_ccip_rd_rsp_data_vec[HOST_MEM_DELAY-1] <= n1_host_mem[{n1_ccip_rd_rqst_data[41:2], n1_burst}];
         end
      end else begin
         n1_burst <= n1_burst + 2'd1;
         n1_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1] <= 'b1;
         n1_ccip_rd_rsp_data_vec[HOST_MEM_DELAY-1] <= n1_host_mem[{n1_rd_addr[41:2], n1_burst}];
      end
      n1_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-2:0] <= n1_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1:1];
      n1_ccip_rd_rsp_data_vec[0:HOST_MEM_DELAY-2] <= n1_ccip_rd_rsp_data_vec[1:HOST_MEM_DELAY-1];

      if (!n1_c1TxAlmFull && n1_ccip_wr_rqst_valid) begin
         n1_host_mem[n1_ccip_wr_rqst_data[553:512]] <= n1_ccip_wr_rqst_data[511:0];
      end
   end

   all_reduce all_reduce_inst1(
      .krn_clk(krn_clk),
      .krn_reset_n(krn_reset_n),
      .reset_e0_n(reset_e0_n),
      .ccip_clk(ccip_clk),
      .ccip_rd_rqst_ready(n1_ccip_rd_rqst_ready),
      .ccip_rd_rqst_valid(n1_ccip_rd_rqst_valid),
      .ccip_rd_rqst_data(n1_ccip_rd_rqst_data),
      .ccip_rd_rsp_valid(n1_ccip_rd_rsp_valid),
      .ccip_rd_rsp_data(n1_ccip_rd_rsp_data),
      .ccip_rd_rsp_ready(n1_ccip_rd_rsp_ready),
      .ccip_wr_rqst_ready(!n1_c1TxAlmFull),
      .ccip_wr_rqst_valid(n1_ccip_wr_rqst_valid),
      .ccip_wr_rqst_data(n1_ccip_wr_rqst_data),
      .cti_ing_data(n1_cti_ing_data),
      .cti_ing_valid(n1_cti_ing_valid),
      .ing_cti_ready(n1_ing_cti_ready),
      .egr_cte_data(n1_egr_cte_data),
      .egr_cte_valid(n1_egr_cte_valid),
      .cte_egr_ready(n1_cte_egr_ready),

      .start_syn_e0(start_syn_e0),
      .kill_syn_e0(kill_syn_e0),
      .mem_addr(42'd0),
      .done_addr(DONE_ADDR),
      .total_length(TOTAL_LENGTH),
      .node_id(16'd1),
      .num_node(16'd3)
   );

   //======================================================================================
   //
   //   Node 2
   //
   //======================================================================================
   logic n2_c0TxAlmFull;
   logic n2_ccip_rd_rqst_valid;
   logic n2_ccip_rd_rqst_ready;
   logic [41:0] n2_ccip_rd_rqst_data;
   logic [41:0] n2_rd_addr;
   logic n2_ccip_rd_rsp_valid;
   logic [511:0] n2_ccip_rd_rsp_data;
   logic n2_ccip_rd_rsp_ready;
   logic n2_c1TxAlmFull;
   logic n2_ccip_wr_rqst_valid;
   logic [553:0] n2_ccip_wr_rqst_data;
   logic [255:0] n2_cti_ing_data;
   logic n2_cti_ing_valid;
   logic n2_ing_cti_ready;
   logic [255:0] n2_egr_cte_data;
   logic n2_egr_cte_valid;
   logic n2_cte_egr_ready;

   // CCIP MEM model
   reg [511:0] n2_host_mem [0:TOTAL_LENGTH];
   logic [31:0] n2_rand_num2;
   logic [31:0] n2_rand_num3;

   logic [511:0] n2_ccip_rd_rsp_data_vec [0:HOST_MEM_DELAY-1];
   logic [HOST_MEM_DELAY-1:0] n2_ccip_rd_rsp_valid_vec = 0;
   logic [1:0] n2_burst = 0;
   assign n2_ccip_rd_rsp_valid = n2_ccip_rd_rsp_valid_vec[0];
   assign n2_ccip_rd_rsp_data = n2_ccip_rd_rsp_data_vec[0];
   assign n2_ccip_rd_rqst_ready = (!n2_c0TxAlmFull) && (n2_burst == 2'd0);
   always @(posedge ccip_clk) begin
      n2_rand_num2 = $urandom_range(0, 100);
      n2_rand_num3 = $urandom_range(0, 100);
      if (n2_rand_num2 < CCIP_EFFICIENCY) begin
         n2_c0TxAlmFull <= 'b0;
      end else begin
         n2_c0TxAlmFull <= 'b1;
      end
      if (n2_rand_num3 < CCIP_EFFICIENCY) begin
         n2_c1TxAlmFull <= 'b0;
      end else begin
         n2_c1TxAlmFull <= 'b1;
      end
      n2_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1] <= 'b0;
      if (n2_burst == 0) begin
         if (!n2_c0TxAlmFull && n2_ccip_rd_rqst_valid) begin
            n2_burst <= n2_burst + 2'd1;
            n2_rd_addr <= n2_ccip_rd_rqst_data;
            n2_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1] <= 'b1;
            n2_ccip_rd_rsp_data_vec[HOST_MEM_DELAY-1] <= n2_host_mem[{n2_ccip_rd_rqst_data[41:2], n2_burst}];
         end
      end else begin
         n2_burst <= n2_burst + 2'd1;
         n2_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1] <= 'b1;
         n2_ccip_rd_rsp_data_vec[HOST_MEM_DELAY-1] <= n2_host_mem[{n2_rd_addr[41:2], n2_burst}];
      end
      n2_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-2:0] <= n2_ccip_rd_rsp_valid_vec[HOST_MEM_DELAY-1:1];
      n2_ccip_rd_rsp_data_vec[0:HOST_MEM_DELAY-2] <= n2_ccip_rd_rsp_data_vec[1:HOST_MEM_DELAY-1];

      if (!n2_c1TxAlmFull && n2_ccip_wr_rqst_valid) begin
         n2_host_mem[n2_ccip_wr_rqst_data[553:512]] <= n2_ccip_wr_rqst_data[511:0];
      end
   end

   all_reduce all_reduce_inst2(
      .krn_clk(krn_clk),
      .krn_reset_n(krn_reset_n),
      .reset_e0_n(reset_e0_n),
      .ccip_clk(ccip_clk),
      .ccip_rd_rqst_ready(n2_ccip_rd_rqst_ready),
      .ccip_rd_rqst_valid(n2_ccip_rd_rqst_valid),
      .ccip_rd_rqst_data(n2_ccip_rd_rqst_data),
      .ccip_rd_rsp_valid(n2_ccip_rd_rsp_valid),
      .ccip_rd_rsp_data(n2_ccip_rd_rsp_data),
      .ccip_rd_rsp_ready(n2_ccip_rd_rsp_ready),
      .ccip_wr_rqst_ready(!n2_c1TxAlmFull),
      .ccip_wr_rqst_valid(n2_ccip_wr_rqst_valid),
      .ccip_wr_rqst_data(n2_ccip_wr_rqst_data),
      .cti_ing_data(n2_cti_ing_data),
      .cti_ing_valid(n2_cti_ing_valid),
      .ing_cti_ready(n2_ing_cti_ready),
      .egr_cte_data(n2_egr_cte_data),
      .egr_cte_valid(n2_egr_cte_valid),
      .cte_egr_ready(n2_cte_egr_ready),

      .start_syn_e0(start_syn_e0),
      .kill_syn_e0(kill_syn_e0),
      .mem_addr(42'd0),
      .done_addr(DONE_ADDR),
      .total_length(TOTAL_LENGTH),
      .node_id(16'd2),
      .num_node(16'd3)
   );


   //======================================================================================
   //
   //   ETH
   //
   //======================================================================================
   logic [31:0] rand_num4;
   logic [31:0] rand_num5;
   logic [31:0] rand_num6;
   logic [255:0] eth_data_vec0 [0:ETH_DELAY-1];
   logic [255:0] eth_data_vec1 [0:ETH_DELAY-1];
   logic [255:0] eth_data_vec2 [0:ETH_DELAY-1];
   logic [ETH_DELAY-1:0] eth_valid_vec0;
   logic [ETH_DELAY-1:0] eth_valid_vec1;
   logic [ETH_DELAY-1:0] eth_valid_vec2;

   assign n0_cti_ing_data = eth_data_vec0[0];
   assign n0_cti_ing_valid = eth_valid_vec0[0];
   assign n1_cti_ing_data = eth_data_vec1[0];
   assign n1_cti_ing_valid = eth_valid_vec1[0];
   assign n2_cti_ing_data = eth_data_vec2[0];
   assign n2_cti_ing_valid = eth_valid_vec2[0];
   // n0 -> n2
   always @(posedge krn_clk) begin
      rand_num4 = $urandom_range(0, 100);
      if (rand_num4 < ETH_EFFICIENCY) begin
         n0_cte_egr_ready <= 'b1;
      end else begin
         n0_cte_egr_ready <= 'b0;
      end
      if (n2_ing_cti_ready) begin
         eth_valid_vec2[ETH_DELAY-1] <= 'b0;
         if (n0_cte_egr_ready && n0_egr_cte_valid) begin
            eth_valid_vec2[ETH_DELAY-1] <= 'b1;
            eth_data_vec2[ETH_DELAY-1] <= n0_egr_cte_data;
         end
         eth_valid_vec2[ETH_DELAY-2:0] <= eth_valid_vec2[ETH_DELAY-1:1];
         eth_data_vec2[0:ETH_DELAY-2] <= eth_data_vec2[1:ETH_DELAY-1];
      end
   end
   // n1 -> n0
   always @(posedge krn_clk) begin
      rand_num5 = $urandom_range(0, 100);
      if (rand_num5 < ETH_EFFICIENCY) begin
         n1_cte_egr_ready <= 'b1;
      end else begin
         n1_cte_egr_ready <= 'b0;
      end
      if (n0_ing_cti_ready) begin
         eth_valid_vec0[ETH_DELAY-1] <= 'b0;
         if (n1_cte_egr_ready && n1_egr_cte_valid) begin
            eth_valid_vec0[ETH_DELAY-1] <= 'b1;
            eth_data_vec0[ETH_DELAY-1] <= n1_egr_cte_data;
         end
         eth_valid_vec0[ETH_DELAY-2:0] <= eth_valid_vec0[ETH_DELAY-1:1];
         eth_data_vec0[0:ETH_DELAY-2] <= eth_data_vec0[1:ETH_DELAY-1];
      end
   end
   // n2 -> n1
   always @(posedge krn_clk) begin
      rand_num6 = $urandom_range(0, 100);
      if (rand_num6 < ETH_EFFICIENCY) begin
         n2_cte_egr_ready <= 'b1;
      end else begin
         n2_cte_egr_ready <= 'b0;
      end
      if (n1_ing_cti_ready) begin
         eth_valid_vec1[ETH_DELAY-1] <= 'b0;
         if (n2_cte_egr_ready && n2_egr_cte_valid) begin
            eth_valid_vec1[ETH_DELAY-1] <= 'b1;
            eth_data_vec1[ETH_DELAY-1] <= n2_egr_cte_data;
         end
         eth_valid_vec1[ETH_DELAY-2:0] <= eth_valid_vec1[ETH_DELAY-1:1];
         eth_data_vec1[0:ETH_DELAY-2] <= eth_data_vec1[1:ETH_DELAY-1];
      end
   end

   //======================================================================================
   //
   //   Sim
   //
   //======================================================================================
   int i, j;
   real error = 0.0;
   int fail = 0;
   initial begin
      start_syn_e0 = 'b0;
      kill_syn_e0 = 'b0;
      krn_reset_n = 'b0;
      reset_e0_n = 'b0;

      for (i = 0; i < TOTAL_LENGTH; i = i+1) begin
         for (j = 0; j < 16; j = j+1) begin
            n0_host_mem[i][j*32+:32] = $shortrealtobits((i*16+j+1.0));
         end
      end
      for (i = 0; i < TOTAL_LENGTH; i = i+1) begin
         for (j = 0; j < 16; j = j+1) begin
            n1_host_mem[i][j*32+:32] = $shortrealtobits((i*16+j+1.0));
         end
      end
      for (i = 0; i < TOTAL_LENGTH; i = i+1) begin
         for (j = 0; j < 16; j = j+1) begin
            n2_host_mem[i][j*32+:32] = $shortrealtobits((i*16+j+1.0));
         end
      end

      #100000;
      krn_reset_n = 'b1;
      reset_e0_n = 'b1;

      #20000;
      start_syn_e0 = 'b1;
      #3200;
      start_syn_e0 = 'b0;
      #80000000;
      for (i = 0; i < TOTAL_LENGTH; i = i+1) begin
         for (j = 0; j < 16; j = j+1) begin
            error = ($bitstoshortreal(n0_host_mem[i][j*32+:32]) - 3*(i*16+j+1.0));
            if (error < -1e-6 || error > 1e-6) begin
               if (fail < 100) begin
                  $display("got %f, should be %f, error is %f\n", $bitstoshortreal(n0_host_mem[i][j*32+:32]), 3*(i*16+j+1.0), error);
               end
               fail = fail + 1;
            end
         end
      end
      for (i = 0; i < TOTAL_LENGTH; i = i+1) begin
         for (j = 0; j < 16; j = j+1) begin
            error = ($bitstoshortreal(n1_host_mem[i][j*32+:32]) - 3*(i*16+j+1.0));
            if (error < -1e-6 || error > 1e-6) begin
               fail = fail + 1;
            end
         end
      end
      for (i = 0; i < TOTAL_LENGTH; i = i+1) begin
         for (j = 0; j < 16; j = j+1) begin
            error = ($bitstoshortreal(n2_host_mem[i][j*32+:32]) - 3*(i*16+j+1.0));
            if (error < -1e-6 || error > 1e-6) begin
               fail = fail + 1;
            end
         end
      end
      if (fail == 0) begin
         $display("PASSED\n");
      end else begin
         $display("FAILED, number of errors%d\n", fail);
      end
      $finish;
   end

endmodule