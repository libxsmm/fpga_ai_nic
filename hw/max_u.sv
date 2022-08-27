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

module max_u #(
    parameter NUM = 10,
    parameter SIZE = 8
)(
    input clk,
    input [SIZE-1:0] din[0:NUM-1],
    output [SIZE-1:0] dout
);
genvar i;
generate
if (NUM == 1) begin
    assign dout = din[0];
end
else if (NUM == 2) begin
    reg greater_flag; 
    reg [SIZE-1:0] din_r[0:NUM-1];
    reg [SIZE-1:0] dout_r;
    
    always @(posedge clk) begin
        din_r <= din;
        greater_flag <= (din[1] > din[0]);
        dout_r <= din_r[greater_flag];
    end
    assign dout = dout_r;
end
else if (NUM == 3) begin
    reg gr01, gr02, gr12; 
    reg [SIZE-1:0] din_r[0:NUM-1];
    reg [SIZE-1:0] dout_r;
    
    always @(posedge clk) begin
        din_r <= din;
        gr01 <= (din[0] > din[1]);
        gr02 <= (din[0] > din[2]);
        gr12 <= (din[1] > din[2]);
        dout_r <= din_r[0] & {SIZE { gr01 & gr02}} 
              | din_r[1] & {SIZE {~gr01 & gr12}}
              | din_r[2] & {SIZE {~gr02 & ~gr12}};
    end
    assign dout = dout_r;
end
else if (NUM == 4) begin
    reg gr01, gr02, gr03, gr12, gr13, gr23; 
    reg [SIZE-1:0] din_r[0:NUM-1];
    reg [SIZE-1:0] dout_r;
    
    always @(posedge clk) begin
        din_r <= din;
        gr01 <= (din[0] > din[1]);
        gr02 <= (din[0] > din[2]);
        gr03 <= (din[0] > din[3]);
        gr12 <= (din[1] > din[2]);
        gr13 <= (din[1] > din[3]);
        gr23 <= (din[2] > din[3]);
        dout_r <= din_r[0] & {SIZE { gr01 &  gr02 &  gr03}} 
              | din_r[1] & {SIZE {~gr01 &  gr12 &  gr13}}
              | din_r[2] & {SIZE {~gr02 & ~gr12 &  gr23}}
              | din_r[3] & {SIZE {~gr03 & ~gr13 & ~gr23}};
    end
    assign dout = dout_r;
end
else if (NUM % 3 == 0) begin
    localparam GROUP_NUM = NUM / 3;
    wire [SIZE-1:0] din2[0:GROUP_NUM-1];
    for (i = 0; i < GROUP_NUM; i=i+1) begin : loop
        max_u #(.SIZE(SIZE), .NUM(3)) inst(.clk(clk), .din(din[3*i:3*i+2]), .dout(din2[i]));
    end
    max_u #(.SIZE(SIZE), .NUM(GROUP_NUM)) inst_final(.clk(clk), .din(din2), .dout(dout));
end
else if (NUM % 3 == 1) begin
    localparam GROUP_NUM = NUM / 3 + 1;
    wire [SIZE-1:0] din2[0:GROUP_NUM-1];
    for (i = 0; i < GROUP_NUM-2; i=i+1) begin : loop
        max_u #(.SIZE(SIZE), .NUM(3)) inst(.clk(clk), .din(din[3*i:3*i+2]), .dout(din2[i]));
    end
    max_u #(.SIZE(SIZE), .NUM(2)) inst2(.clk(clk), .din(din[NUM-4:NUM-3]), .dout(din2[GROUP_NUM-2]));
    max_u #(.SIZE(SIZE), .NUM(2)) inst3(.clk(clk), .din(din[NUM-2:NUM-1]), .dout(din2[GROUP_NUM-1]));
    max_u #(.SIZE(SIZE), .NUM(GROUP_NUM)) inst_final(.clk(clk), .din(din2), .dout(dout));
end
else begin // if (NUM % 3 == 2)
    localparam GROUP_NUM = NUM / 3 + 1;
    wire [SIZE-1:0] din2[0:GROUP_NUM-1];
    for (i = 0; i < GROUP_NUM-1; i=i+1) begin : loop
        max_u #(.SIZE(SIZE), .NUM(3)) inst(.clk(clk), .din(din[3*i:3*i+2]), .dout(din2[i]));
    end
    max_u #(.SIZE(SIZE), .NUM(2)) inst2(.clk(clk), .din(din[NUM-2:NUM-1]), .dout(din2[GROUP_NUM-1]));
    max_u #(.SIZE(SIZE), .NUM(GROUP_NUM)) inst_final(.clk(clk), .din(din2), .dout(dout));
end

endgenerate
endmodule
