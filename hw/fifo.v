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

// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module  fifo #(
    parameter DW = 256,
    parameter AW = 9,
    parameter DEPTH = 512)  (
    clock,
    data,
    rdreq,
    sclr,
    wrreq,
    empty,
    full,
    q,
    usedw);

    input    clock;
    input  [DW-1:0]  data;
    input    rdreq;
    input    sclr;
    input    wrreq;
    output   empty;
    output   full;
    output [DW-1:0]  q;
    output [AW-1:0]  usedw;

    wire  sub_wire0;
    wire  sub_wire1;
    wire [DW-1:0] sub_wire2;
    wire [AW-1:0] sub_wire3;
    wire  empty = sub_wire0;
    wire  full = sub_wire1;
    wire [DW-1:0] q = sub_wire2[DW-1:0];
    wire [AW-1:0] usedw = sub_wire3[AW-1:0];

    scfifo  scfifo_component (
                .clock (clock),
                .data (data),
                .rdreq (rdreq),
                .sclr (sclr),
                .wrreq (wrreq),
                .empty (sub_wire0),
                .full (sub_wire1),
                .q (sub_wire2),
                .usedw (sub_wire3),
                .aclr (),
                .almost_empty (),
                .almost_full (),
                .eccstatus ());
    defparam
        scfifo_component.add_ram_output_register  = "ON",
        scfifo_component.enable_ecc  = "FALSE",
        scfifo_component.intended_device_family  = "Arria 10",
        scfifo_component.lpm_numwords  = DEPTH,
        scfifo_component.lpm_showahead  = "ON",
        scfifo_component.lpm_type  = "scfifo",
        scfifo_component.lpm_width  = DW,
        scfifo_component.lpm_widthu  = AW,
        scfifo_component.overflow_checking  = "ON",
        scfifo_component.underflow_checking  = "ON",
        scfifo_component.use_eab  = "ON";


endmodule


module  async_fifo_mixed #(
    parameter IDW = 256,
    parameter ODW = 512,
    parameter IAW = 5,
    parameter OAW = 4,
    parameter DEPTH = 32)  (
    aclr,
    data,
    rdclk,
    rdreq,
    wrclk,
    wrreq,
    q,
    rdempty,
    wrfull,
    wrusedw);

    input    aclr;
    input  [IDW-1:0]  data;
    input    rdclk;
    input    rdreq;
    input    wrclk;
    input    wrreq;
    output [ODW-1:0]  q;
    output   rdempty;
    output   wrfull;
    output [IAW-1:0]  wrusedw;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
    tri0     aclr;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

    wire [ODW-1:0] sub_wire0;
    wire  sub_wire1;
    wire  sub_wire2;
    wire [IAW-1:0] sub_wire3;
    wire [ODW-1:0] q = sub_wire0[ODW-1:0];
    wire  rdempty = sub_wire1;
    wire  wrfull = sub_wire2;
    wire [IAW-1:0] wrusedw = sub_wire3[IAW-1:0];

    dcfifo_mixed_widths  dcfifo_mixed_widths_component (
                .aclr (aclr),
                .data (data),
                .rdclk (rdclk),
                .rdreq (rdreq),
                .wrclk (wrclk),
                .wrreq (wrreq),
                .q (sub_wire0),
                .rdempty (sub_wire1),
                .wrfull (sub_wire2),
                .wrusedw (sub_wire3),
                .eccstatus (),
                .rdfull (),
                .rdusedw (),
                .wrempty ());
    defparam
        dcfifo_mixed_widths_component.enable_ecc  = "FALSE",
        dcfifo_mixed_widths_component.intended_device_family  = "Arria 10",
        dcfifo_mixed_widths_component.lpm_hint  = "DISABLE_DCFIFO_EMBEDDED_TIMING_CONSTRAINT=TRUE",
        dcfifo_mixed_widths_component.lpm_numwords  = DEPTH,
        dcfifo_mixed_widths_component.lpm_showahead  = "ON",
        dcfifo_mixed_widths_component.lpm_type  = "dcfifo_mixed_widths",
        dcfifo_mixed_widths_component.lpm_width  = IDW,
        dcfifo_mixed_widths_component.lpm_widthu  = IAW,
        dcfifo_mixed_widths_component.lpm_widthu_r  = OAW,
        dcfifo_mixed_widths_component.lpm_width_r  = ODW,
        dcfifo_mixed_widths_component.overflow_checking  = "ON",
        dcfifo_mixed_widths_component.rdsync_delaypipe  = 4,
        dcfifo_mixed_widths_component.read_aclr_synch  = "OFF",
        dcfifo_mixed_widths_component.underflow_checking  = "ON",
        dcfifo_mixed_widths_component.use_eab  = "ON",
        dcfifo_mixed_widths_component.write_aclr_synch  = "OFF",
        dcfifo_mixed_widths_component.wrsync_delaypipe  = 4;


endmodule

module  async_fifo #(
    parameter DW = 256,
    parameter AW = 9,
    parameter DEPTH = 512) (
    aclr,
    data,
    rdclk,
    rdreq,
    wrclk,
    wrreq,
    q,
    rdempty,
    wrfull,
    wrusedw);

    input    aclr;
    input  [DW-1:0]  data;
    input    rdclk;
    input    rdreq;
    input    wrclk;
    input    wrreq;
    output [DW-1:0]  q;
    output   rdempty;
    output   wrfull;
    output [AW-1:0]  wrusedw;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
    tri0     aclr;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

    wire [DW-1:0] sub_wire0;
    wire  sub_wire1;
    wire  sub_wire2;
    wire [AW-1:0] sub_wire3;
    wire [DW-1:0] q = sub_wire0[DW-1:0];
    wire  rdempty = sub_wire1;
    wire  wrfull = sub_wire2;
    wire [AW-1:0] wrusedw = sub_wire3[AW-1:0];

    dcfifo  dcfifo_component (
                .aclr (aclr),
                .data (data),
                .rdclk (rdclk),
                .rdreq (rdreq),
                .wrclk (wrclk),
                .wrreq (wrreq),
                .q (sub_wire0),
                .rdempty (sub_wire1),
                .wrfull (sub_wire2),
                .wrusedw (sub_wire3),
                .eccstatus (),
                .rdfull (),
                .rdusedw (),
                .wrempty ());
    defparam
        dcfifo_component.enable_ecc  = "FALSE",
        dcfifo_component.intended_device_family  = "Arria 10",
        dcfifo_component.lpm_hint  = "DISABLE_DCFIFO_EMBEDDED_TIMING_CONSTRAINT=TRUE",
        dcfifo_component.lpm_numwords  = DEPTH,
        dcfifo_component.lpm_showahead  = "ON",
        dcfifo_component.lpm_type  = "dcfifo",
        dcfifo_component.lpm_width  = DW,
        dcfifo_component.lpm_widthu  = AW,
        dcfifo_component.overflow_checking  = "ON",
        dcfifo_component.rdsync_delaypipe  = 4,
        dcfifo_component.read_aclr_synch  = "OFF",
        dcfifo_component.underflow_checking  = "ON",
        dcfifo_component.use_eab  = "ON",
        dcfifo_component.write_aclr_synch  = "OFF",
        dcfifo_component.wrsync_delaypipe  = 4;


endmodule
