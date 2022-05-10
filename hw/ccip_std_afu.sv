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

`include "platform_if.vh"
`include "cci_mpf_if.vh"
`include "cci_mpf_platform.vh"
`include "cci_mpf_app_conf_default.vh"
`include "cci_csr_if.vh"

import local_mem_cfg_pkg::*;

module ccip_std_afu
  #(
    parameter NUM_LOCAL_MEM_BANKS = 2
    )
   (
    // CCI-P Clocks and Resets
    input logic       pClk, // Primary CCI-P interface clock. 200.00 MHZ
    input logic       pClkDiv2, // Aligned, pClk divided by 2.    100.00 MHZ
    input logic       pClkDiv4, // Aligned, pClk divided by 4.     50.00 MHZ
    input logic       uClk_usr, // User clock domain. Refer to clock programming guide.
    input logic       uClk_usrDiv2, // Aligned, user clock divided by 2.
    
    input logic       pck_cp2af_softReset, // CCI-P ACTIVE HIGH Soft. Reset Relaesed only when ALL clocks are stable
    input logic [1:0] pck_cp2af_pwrState, // CCI-P AFU Power State
    input logic       pck_cp2af_error, // CCI-P Protocol Error Detected
      
    // Raw HSSI interface
    pr_hssi_if.to_fiu hssi,
    
    // CCI-P structures
    input 	      t_if_ccip_Rx pck_cp2af_sRx, // CCI-P Rx Port
    output 	      t_if_ccip_Tx pck_af2cp_sTx // CCI-P Tx Port
    );
   
`ifdef nct
   localparam NCT = `nct; // Number of CITAS's
`else
   localparam NCT = 1;    // Number of CITAS's
`endif
`ifdef ppc
   localparam PPC = `ppc; // Ports Per CITAS.
`else
   localparam PPC = 4;    // Ports Per CITAS.
`endif

   //------------------------------------------------------------------------------
   // Pick the proper clk, as chosen by the AFU's JSON file
   //------------------------------------------------------------------------------
   
   // The platform may transform the CCI-P clock from pClk to a clock
   // chosen in the AFU's JSON file.
   logic              ccip_clk;
   assign ccip_clk = `PLATFORM_PARAM_CCI_P_CLOCK; // set to hssi.f2a_prmgmt_ctrl_clk in ikl_fpga.json
   
   logic              reset;
   assign reset = `PLATFORM_PARAM_CCI_P_RESET;
   
   logic              pm_clk;
   assign pm_clk = hssi.f2a_prmgmt_ctrl_clk;    // 100.00 MHZ

   // ====================================================================
   //
   //  Convert the external wires to an MPF interface.
   //
   // ====================================================================

   //
   // The AFU exposes the primary AFU device feature header (DFH) at MMIO
   // address 0.  MPF defines a set of its own DFHs.  The AFU must
   // build its feature chain to point to the MPF chain.  The AFU must
   // also tell the MPF module the MMIO address at which MPF should start
   // its feature chain.
   //
   localparam MPF_DFH_MMIO_ADDR = 'h1000;

   //
   // MPF represents CCI as a SystemVerilog interface, derived from the
   // same basic types defined in ccip_if_pkg.  Interfaces reduce the
   // number of internal MPF module parameters, since each internal MPF
   // shim has a bus connected toward the AFU and a bus connected toward
   // the FIU.
   //

   //
   // Expose FIU as an MPF interface
   //
   cci_mpf_if#(.ENABLE_LOG(1)) fiu(.clk(ccip_clk));

   // The CCI wires to MPF mapping connections have identical naming to
   // the standard AFU.  The module exports an interface named "fiu".
   ccip_wires_to_mpf
   #(
   // All inputs and outputs in PR region (AFU) must be registered!
   .REGISTER_INPUTS(1),
   .REGISTER_OUTPUTS(1)
   )
   map_ifc
   (
      .pClk(ccip_clk),
      .pck_cp2af_softReset(reset),
      .fiu,
      .*
   );

   // ====================================================================
   //
   //  Manage CSRs at the lowest level so they can observe the edge state
   //  and to keep them available even when other code fails.
   //
   // ====================================================================

   cci_mpf_if afu_csrs(.clk(ccip_clk));
   assign afu_csrs.reset = fiu.reset;

   // Most wires flow straight through and are, at most, read in this shim.
   assign fiu.c0Tx = afu_csrs.c0Tx;
   assign afu_csrs.c0TxAlmFull = fiu.c0TxAlmFull;
   assign fiu.c1Tx = afu_csrs.c1Tx;
   assign afu_csrs.c1TxAlmFull = fiu.c1TxAlmFull;

   assign afu_csrs.c0Rx = fiu.c0Rx;
   assign afu_csrs.c1Rx = fiu.c1Rx;

   t_if_cci_c2_Tx csr2cp_tx;
   always_ff @(posedge ccip_clk) begin
      // Forward MMIO read responses from other modules closer to the AFU.
      fiu.c2Tx <= afu_csrs.c2Tx;

      // Pass local read responses toward the host.  This code has a race
      // if two MMIO read requests are outstanding at the same time, since
      // the local response will overwrite the value forwarded from afu.c2Tx.
      // Our sample applications never have two MMIO reads outstanding
      // simulatenously, so we leave the hardware here simple.
      if (csr2cp_tx.mmioRdValid) begin
         fiu.c2Tx <= csr2cp_tx;
      end
   end

   // app_csrs csrs();

   // csr_mgr
   // #(
   //    .NEXT_DFH_BYTE_OFFSET(MPF_DFH_MMIO_ADDR)
   //    )
   // csr_io
   // (
   //    .clk(afu_clk),
   //    .fiu,
   //    .afu(afu_csrs),
   //    .pck_cp2af_pwrState(pck_cp2af_pwrState_q),
   //    .pck_cp2af_error(pck_cp2af_error_q),
   //    .csrs
   //    );


   // ====================================================================
   //
   //  Instantiate a memory properties factory (MPF) between the external
   //  interface and the AFU, optionally adding support for virtual
   //  memory and control over memory ordering.
   //
   // ====================================================================

   cci_mpf_if#(.ENABLE_LOG(1)) afu(.clk(ccip_clk));

   logic c0NotEmpty;
   logic c1NotEmpty;

   cci_mpf
   #(
   // Should read responses be returned in the same order that
   // the reads were requested?
   .SORT_READ_RESPONSES(`MPF_CONF_SORT_READ_RESPONSES),

   // Should the Mdata from write requests be returned in write
   // responses?  If the AFU is simply counting write responses
   // and isn't consuming Mdata, then setting this to 0 eliminates
   // the memory and logic inside MPF for preserving Mdata.
   .PRESERVE_WRITE_MDATA(`MPF_CONF_PRESERVE_WRITE_MDATA),

   // Enable virtual to physical translation?  When enabled, MPF
   // accepts requests with either virtual or physical addresses.
   // Virtual addresses are indicated by setting the
   // addrIsVirtual flag in the MPF extended Tx channel
   // request header.
   .ENABLE_VTP(`MPF_CONF_ENABLE_VTP),
   // Two implementations of physical to virtual page translation are
   // available in VTP. Pick mode "HARDWARE_WALKER" to walk the VTP
   // page table using AFU-generated memory reads. Pick mode
   // "SOFTWARE_SERVICE" to send translation requests to software.
   // In HARDWARE_WALKER mode it is the user code's responsibility to
   // pin all pages that may be touched by the FPGA. The SOFTWARE_SERVICE
   // mode may pin pages automatically on demand.
`ifdef MPF_CONF_VTP_PT_MODE_HARDWARE_WALKER
   .VTP_PT_MODE("HARDWARE_WALKER"),
`elsif MPF_CONF_VTP_PT_MODE_SOFTWARE_SERVICE
   .VTP_PT_MODE("SOFTWARE_SERVICE"),
`endif

   // Enable mapping of eVC_VA to physical channels?  AFUs that both use
   // eVC_VA and read back memory locations written by the AFU must either
   // emit WrFence on VA or use explicit physical channels and enforce
   // write/read order.  Each method has tradeoffs.  WrFence VA is expensive
   // and should be emitted only infrequently.  Memory requests to eVC_VA
   // may have higher bandwidth than explicit mapping.  The MPF module for
   // physical channel mapping is optimized for each CCI platform.
   //
   // If you set ENFORCE_WR_ORDER below you probably also want to set
   // ENABLE_VC_MAP.
   //
   // The mapVAtoPhysChannel extended header bit must be set on each
   // request to enable mapping.
   .ENABLE_VC_MAP(`MPF_CONF_ENABLE_VC_MAP),
   // When ENABLE_VC_MAP is set the mapping is either static for the entire
   // run or dynamic, changing in response to traffic patterns.  The mapper
   // guarantees synchronization when the mapping changes by emitting a
   // WrFence on eVC_VA and draining all reads.  Ignored when ENABLE_VC_MAP
   // is 0.
   .ENABLE_DYNAMIC_VC_MAPPING(`MPF_CONF_ENABLE_DYNAMIC_VC_MAPPING),

   // Manage traffic to reduce latency without sacrificing bandwidth?
   // The blue bitstream buffers for a given channel may be larger than
   // necessary to sustain full bandwidth.  Allowing more requests beyond
   // this threshold to enter the channel increases latency without
   // increasing bandwidth.  While this is fine for some applications,
   // those with multiple kernels connected to the CCI memory interface
   // may see performance gains when a kernel's performance is a
   // latency-sensitive.
   .ENABLE_LATENCY_QOS(`MPF_CONF_ENABLE_LATENCY_QOS),

   // Should write/write and write/read ordering within a cache
   // be enforced?  By default CCI makes no guarantees on the order
   // in which operations to the same cache line return.  Setting
   // this to 1 adds logic to filter reads and writes to ensure
   // that writes retire in order and the reads correspond to the
   // most recent write.
   //
   // ***  Even when set to 1, MPF guarantees order only within
   // ***  a given virtual channel.  There is no guarantee of
   // ***  order across virtual channels and no guarantee when
   // ***  using eVC_VA, since it spreads requests across all
   // ***  channels.  Synchronizing writes across virtual channels
   // ***  can be accomplished only by requesting a write fence on
   // ***  eVC_VA.  Syncronizing writes across virtual channels
   // ***  and then reading back the same data requires both
   // ***  requesting a write fence on eVC_VA and waiting for the
   // ***  corresponding write fence response.
   //
   .ENFORCE_WR_ORDER(`MPF_CONF_ENFORCE_WR_ORDER),

   // Enable partial write emulation.  CCI has no support for masked
   // writes that merge new data with existing data in a line.  MPF
   // adds byte-level masks to the write request header and emulates
   // partial writes as a read-modify-write operation.  When coupled
   // with ENFORCE_WR_ORDER, partial writes are free of races on the
   // FPGA side.  There are no guarantees of atomicity and there is
   // no protection against races with CPU-generates writes.
   .ENABLE_PARTIAL_WRITES(`MPF_CONF_ENABLE_PARTIAL_WRITES),

   // Experimental:  Merge nearby reads from the same address?  Some
   // applications generate reads to the same line within a few cycles
   // of each other.  This module reduces the requests to single host
   // read and replicates the result.  The module requires a wide
   // block RAM FIFO, so should not be enabled without some thought.
   .MERGE_DUPLICATE_READS(`MPF_CONF_MERGE_DUPLICATE_READS),

   // Address of the MPF feature header.  See comment above.
   .DFH_MMIO_BASE_ADDR(MPF_DFH_MMIO_ADDR)
   )
   mpf
   (
      .clk(ccip_clk),
      .fiu(afu_csrs),
      .afu,
      .c0NotEmpty,
      .c1NotEmpty
      );
   
   //------------------------------------------------------------------------------
   // Internal signals
   //------------------------------------------------------------------------------
   reg                init_done_r;
   reg [63:0] 	      afu_scratch;
   reg [63:0] 	      afu_init;
   
   reg                csr_rst = 1'b1;
   reg                rx_rst  = 1'b1;
   reg                tx_rst  = 1'b1;
   reg [3:0] 	      sloop;
   
   //------------------------------------------------------------------------------
   // CSR Address Map
   //------------------------------------------------------------------------------
   
   localparam AFU_DFH       = 16'h0000; // 0
   localparam AFU_ID_L      = 16'h0008; // 1
   localparam AFU_ID_H      = 16'h0010; // 2
   localparam AFU_RST_CTRL  = 16'h0018; // 3
   localparam LOOPBK_CTRL   = 16'h0020; // 4
   localparam AFU_INIT      = 16'h0028; // 5
   localparam ETH_CTRL_ADDR = 16'h0030; // 6
   localparam ETH_WR_DATA   = 16'h0038; // 7
   localparam ETH_RD_DATA   = 16'h0040; // 8
   localparam AFU_SCRATCH   = 16'h0048; // 9
   
   // genvar 	      i;
   
   //Base mem address is offset 0x140000 from BAR2
   // Offset Register    Bit(s) Access    Description
   //  0x00  AFU_DFH     63:0    RO       returns 0x1000000010000001
   //  0x08  AFU_ID_L    63:0    RO       returns 0xB74F291AF34E1783
   //  0x10  AFU_ID_H    63:0    RO       returns 0x05189FE40676DD24
   //
   // 0x18  RST_CTRL       0     RW       tx_rst  - resets Streaming Agent, CITAS and E-Net MAC tx path
   //                      1     RW       rx_rst  - resets E-Net MAC rx path
   //                      2     RW       csr_rst - reset CSRs in the MAC
   //
   // 0x20 LOOPBK_CTRL     0     RW       loopback tx data to rx data. (a debug mode)
   //                      1     RW       stratch
   //                      2     RW       stratch
   //                      3     RW       stratch
   //                     16     RO       hssi.f2a_rx_is_lockedtodata[0]
   //                     17     RO       hssi.f2a_rx_is_lockedtodata[1]
   //                     18     RO       hssi.f2a_rx_is_lockedtodata[2]
   //                     19     RO       hssi.f2a_rx_is_lockedtodata[3]
   //                     20     RO       hssi.f2a_rx_enh_blk_lock[0]
   //                     21     RO       hssi.f2a_rx_enh_blk_lock[1]
   //                     22     RO       hssi.f2a_rx_enh_blk_lock[2]
   //                     23     RO       hssi.f2a_rx_enh_blk_lock[3]
   //
   // 0x28 AFU_INIT        0     RW       afu_init      - sets hssi.a2f_init_start
   //                      1     RO       afu_init_done - reads state of hssi.f2a_init_done
   //                   63:2     RW       scratch
   //
   // 0x30 CTRL_ADDR    15:0     RW       Offset (0x33C0 is the 32-bit word address offset to the beginning of the
   //                                     streaming agent address space)
   //                     16     RW       Write bit - Writes the data in reg 0x38 to the address in bits[15:0]
   //                     17     RW       Read bit  - Read the address in bits[15:0], place data in register 0x40.
   //                     18     RO       Read busy - The data in the RD_DATA register is not valid until this bit clears.
   //
   // 0x38 WR_DATA       31:0    RW       write data
   // 0x40 RD_DATA       31:0    RO       read data
   // 0x48 AFU_SCRATCH   63:0    RW       scratch
   
   
   //------------------------------------------------------------------------------
   // CSR registers
   //------------------------------------------------------------------------------
   reg 		      csr_read      = 1'b0;
   reg 		      csr_write     = 1'b0;
   reg [31:0] 	      csr_writedata = 32'h0 /* synthesis preserve */;
   reg [18:0] 	      ctrl_addr     = 18'h0 /* synthesis preserve */;
   reg [15:0] 	      csr_address   = 16'h0 /* synthesis preserve */;
   wire [31:0] 	      csr_readdata;
   reg [31:0] 	      csr_readdata_r;
   
   logic 	      init_start;
   wire 	      init_done;
   
   //------------------------------------------------------------------------------
   // Register PR <--> PR signals near interface before consuming it
   //------------------------------------------------------------------------------
   
   (* noprune *) logic [1:0]  pck_cp2af_pwrState_T1;
   (* noprune *) logic        pck_cp2af_error_T1;
   
   logic              pck_cp2af_softReset_T1;
   // t_if_ccip_Rx pck_cp2af_sRx_T1;
   // t_if_ccip_Tx pck_af2cp_sTx_T0;
   
   // ccip_interface_reg inst_green_ccip_interface_reg
   //   (
   //    .pClk                   (ccip_clk),
   //    .pck_cp2af_softReset_T0 (reset),
   //    .pck_cp2af_pwrState_T0  (pck_cp2af_pwrState),
   //    .pck_cp2af_error_T0     (pck_cp2af_error),
   //    .pck_cp2af_sRx_T0       (pck_cp2af_sRx),
   //    .pck_af2cp_sTx_T0       (pck_af2cp_sTx_T0),
      
   //    .pck_cp2af_softReset_T1 (pck_cp2af_softReset_T1),
   //    .pck_cp2af_pwrState_T1  (pck_cp2af_pwrState_T1),
   //    .pck_cp2af_error_T1     (pck_cp2af_error_T1),
   //    .pck_cp2af_sRx_T1       (pck_cp2af_sRx_T1),
   //    .pck_af2cp_sTx_T1       (pck_af2cp_sTx)
   //    );
   always@(posedge ccip_clk)
   begin
       pck_cp2af_softReset_T1   <= reset;
       pck_cp2af_pwrState_T1    <= pck_cp2af_pwrState;
       pck_cp2af_error_T1       <= pck_cp2af_error;
   end
   
   //------------------------------------------------------------------------------
   // extracting/setting signals on CCIP interface structure
   //------------------------------------------------------------------------------
   
   t_ccip_c0_ReqMmioHdr    cp2csr_MmioHdr;
   logic              cp2csr_MmioWrEn;
   logic              cp2csr_MmioRdEn;
   t_ccip_mmioData         cp2csr_MmioDin;
   
   t_ccip_c2_RspMmioHdr    csr2cp_MmioHdr;
   t_ccip_mmioData         csr2cp_MmioDout;
   logic              csr2cp_MmioDout_v;
   
   // always_comb
   //   begin
   //      // Extract Cfg signals from C0 channel
   //      cp2csr_MmioHdr   = t_ccip_c0_ReqMmioHdr'(pck_cp2af_sRx_T1.c0.hdr);
   //      cp2csr_MmioWrEn  = pck_cp2af_sRx_T1.c0.mmioWrValid;
   //      cp2csr_MmioRdEn  = pck_cp2af_sRx_T1.c0.mmioRdValid;
   //      cp2csr_MmioDin   = pck_cp2af_sRx_T1.c0.data[CCIP_MMIODATA_WIDTH-1:0];
   //      // Setting Rsp signals to C2 channel
   //      pck_af2cp_sTx_T0                  = 'b0;
   //      pck_af2cp_sTx_T0.c2.hdr           = csr2cp_MmioHdr;
   //      pck_af2cp_sTx_T0.c2.data          = csr2cp_MmioDout;
   //      pck_af2cp_sTx_T0.c2.mmioRdValid   = csr2cp_MmioDout_v;
   //   end

   always_comb
     begin
        // Extract Cfg signals from C0 channel
        cp2csr_MmioHdr   = t_ccip_c0_ReqMmioHdr'(fiu.c0Rx.hdr);
        cp2csr_MmioWrEn  = fiu.c0Rx.mmioWrValid;
        cp2csr_MmioRdEn  = fiu.c0Rx.mmioRdValid;
        cp2csr_MmioDin   = fiu.c0Rx.data[CCIP_MMIODATA_WIDTH-1:0];
        // Setting Rsp signals to C2 channel
        csr2cp_tx                  = 'b0;
        csr2cp_tx.hdr              = csr2cp_MmioHdr;
        csr2cp_tx.data             = csr2cp_MmioDout;
        csr2cp_tx.mmioRdValid      = csr2cp_MmioDout_v;
     end
   
   //Note; According to the Acceleration Specification, There is no flow
   //control on the Rx channels.  and we must accept all reads and wites
   //on this interface.  This presents a problem. We do pipeline writes
   //to the ILK logic but they can only be performed at the frequency of
   //the IKL clock.  Additionally, however reads taks some time.  This
   //FIFO was added but if there was a read followed by many writes it
   //is possiable for this FIFO to overrun.
   //This issue is fixed by adding the read_bust bit. SW must spin on
   //this bit to prevent FIFO overrun.
   
   localparam RWQ_WIDTH = 5;
   
   wire               rwq_empty;
   wire               rwq_rdfifo;
   wire [89:0] 	      rddata;
   wire [14:0] 	      rwq_address = rddata[89:75];
   wire [8:0] 	      rwq_tid     = rddata[74:66];
   wire               rwq_wrvld   = rddata[65] & rwq_rdfifo;
   wire               rwq_rdvld   = rddata[64] & rwq_rdfifo;
   wire [63:0] 	      rwq_data    = rddata[63:0];
   wire [RWQ_WIDTH-1:0] rwq_count;
   wire                 csr_waitrequest;
   reg                  rwq_afull;

   logic [31:0] num_rqst_sent;
   logic [31:0] num_rsp_rcvd;
   logic [31:0] num_rsp_send;
   logic [63:0]         debug0, debug1, debug2, debug3;
   
   // sw_should_never_do_this because SW should have made sure that the read busy bit was clear before doing this.
   // Doing this could eventually overrun the RWQ FIFO.
   wire                 sw_should_never_do_this = (csr_read | csr_write) & ((rwq_address[3:0] == ETH_CTRL_ADDR[6:3]) & rddata[65]);
   assign rwq_rdfifo = ~rwq_empty & ~sw_should_never_do_this;
   
   bypass_fifo #(.DATA_WIDTH(90),
                 .ADDR_WIDTH(RWQ_WIDTH),
                 .BYPASS(1'b1))
   rwq (
        .clk                                   (ccip_clk),                                         // input            clk
        .reset_n                               (~pck_cp2af_softReset_T1),                          // input            reset_n
        .write                                 (~rwq_afull & (cp2csr_MmioWrEn | cp2csr_MmioRdEn)), // input            write
        .write_data                            ({
                                                 cp2csr_MmioHdr.address[15:1],
                                                 cp2csr_MmioHdr.tid,
                                                 cp2csr_MmioWrEn,
                                                 cp2csr_MmioRdEn,
                                                 cp2csr_MmioDin[63:0]
                                                 }),                                               // input  [DATA_WIDTH-1:000] write_data
        .read                                  (rwq_rdfifo),                                       // input            read
        .read_data                             (rddata),                                           // output [DATA_WIDTH-1:000] read_data
        .fifo_empty                            (rwq_empty),                                        // output           fifo_empty
        .nafull                                (nafull),                                           // output           nafull
        .fifo_count                            (rwq_count),                                        // output           fifo_count
        .write_pointer                         (),
        .read_pointer                          ()
        );
   
   
   always @(posedge ccip_clk) begin
      if (pck_cp2af_softReset_T1) begin
         rwq_afull <= 1'b0;
      end else begin
         rwq_afull <= rwq_count[RWQ_WIDTH-1:4] |
                      rwq_afull; // make this FATAL
      end
   end
   
   //------------------------------------------------------------------------------
   // CSR registers
   //------------------------------------------------------------------------------
   t_ccip_mmioData csr_rd_data;
   
   always @(posedge ccip_clk) begin
      init_start    <= afu_init[0];
      init_done_r   <= init_done;
   end
   
   // csr write interface...
   always @(posedge ccip_clk or posedge pck_cp2af_softReset_T1) begin
      if (pck_cp2af_softReset_T1) begin
         sloop           <= 'b0;
         afu_init        <= 64'h1;
         ctrl_addr[17:0] <= 'b0;
         csr_writedata   <= 'b0;
         afu_scratch     <= 'b0;
      end else begin
         if (rwq_wrvld)
           case (rwq_address[3:0])
             AFU_RST_CTRL [6:3]: {csr_rst,rx_rst,tx_rst} <= rwq_data[2:0];
             LOOPBK_CTRL  [6:3]: sloop           <= rwq_data[3:0];
             AFU_INIT     [6:3]: afu_init        <= rwq_data;
             ETH_CTRL_ADDR[6:3]: ctrl_addr[17:0] <= rwq_data[17:0];
             ETH_WR_DATA  [6:3]: csr_writedata   <= rwq_data[31:0];
             AFU_SCRATCH  [6:3]: afu_scratch     <= rwq_data;
             default: ;
           endcase
      end
   end
   
   // csr read interface...
   always @(posedge ccip_clk) begin
      case (rwq_address[3:0])
        AFU_DFH      [6:3]: begin
                           t_ccip_dfh afu_dfh;
                           afu_dfh = ccip_dfh_defaultDFH();
                           afu_dfh.f_type = eFTYP_AFU;
                           afu_dfh.eol = 'b0;
                           afu_dfh.nextFeature = MPF_DFH_MMIO_ADDR;

                           csr_rd_data <= afu_dfh;
                        end
        // For E2E e10
        AFU_ID_L     [6:3]: csr_rd_data <= 'hB74F291AF34E1783;
        AFU_ID_H     [6:3]: csr_rd_data <= 'h05189FE40676DD24;
        LOOPBK_CTRL  [6:3]: csr_rd_data <= {40'h0, hssi.f2a_rx_enh_blk_lock, hssi.f2a_rx_is_lockedtodata, 12'h0, sloop};
        AFU_RST_CTRL [6:3]: csr_rd_data <= {61'h0, csr_rst,rx_rst,tx_rst};
        AFU_INIT     [6:3]: csr_rd_data <= {afu_init[63:2], init_done_r, afu_init[0]};
        ETH_CTRL_ADDR[6:3]: csr_rd_data <= 64'b0 | ctrl_addr;
        ETH_WR_DATA  [6:3]: csr_rd_data <= 64'b0 | csr_writedata;
        ETH_RD_DATA  [6:3]: csr_rd_data <= 64'b0 | csr_readdata_r;
        AFU_SCRATCH  [6:3]: csr_rd_data <= afu_scratch;

                     4'ha : csr_rd_data <= {num_rsp_send, num_rsp_rcvd};
                     4'hb : csr_rd_data <= debug0;
                     4'hc : csr_rd_data <= debug1;
                     4'hd : csr_rd_data <= debug2;
                     4'he : csr_rd_data <= debug3;
        default:            csr_rd_data <= 64'b0;
      endcase
   end
   
   //------------------------------------------------------------------------------
   // build the response signals for CCIP interface
   //------------------------------------------------------------------------------
   logic           csr_ren_T1;
   t_ccip_tid      csr_tid_T1;
   
   always @(posedge ccip_clk or posedge pck_cp2af_softReset_T1) begin
      if (pck_cp2af_softReset_T1) begin
         csr_ren_T1        <= 1'b0;
         csr2cp_MmioDout_v <= 1'b0;
      end else begin
         // Pipe Stage T1
         csr_ren_T1 <= rwq_rdvld  && (rwq_address < 15'h200);
         // Pipe Stage T2
         csr2cp_MmioDout_v <= csr_ren_T1;
      end
   end
   
   always @(posedge ccip_clk) begin
      // Pipe Stage T1
      csr_tid_T1 <= rwq_tid;
      // Pipe Stage T2
      csr2cp_MmioHdr      <= csr_tid_T1;
      csr2cp_MmioDout     <= csr_rd_data;
   end
   
   reg flagged_afull = 0;
   
   always @(posedge ccip_clk) begin
      if (pck_cp2af_softReset_T1) begin
         csr_read      <= 1'b0;
         csr_write     <= 1'b0;
         ctrl_addr[18] <= 1'b0;
      end else begin
         csr_read    <= (rwq_address[3:0] == ETH_CTRL_ADDR[6:3]) & rwq_wrvld & rwq_data[17] & ~csr_read |
                        csr_read & csr_waitrequest;
         csr_write   <= (rwq_address[3:0] == ETH_CTRL_ADDR[6:3]) & rwq_wrvld & rwq_data[16] & ~csr_write |
                        csr_write & csr_waitrequest;
         csr_address <= (csr_waitrequest & (csr_read | csr_write)) ? csr_address : ((rwq_address[3:0] == ETH_CTRL_ADDR[6:3]) & rwq_wrvld) ? rwq_data[15:0] : ctrl_addr[15:0];
	 
         ctrl_addr[18] <= (rwq_address[3:0] == ETH_CTRL_ADDR[6:3]) & rwq_wrvld & rwq_data[17] & ~csr_read |
                          csr_read & csr_waitrequest |
                          (rwq_address[3:0] == ETH_CTRL_ADDR[6:3]) & rwq_wrvld & rwq_data[16] & ~csr_write  |
                          csr_write & csr_waitrequest;
         
         //synopsys translate_off
         if ((rwq_address[3:0] == ETH_CTRL_ADDR[6:3]) & rwq_wrvld & (rwq_data[16] | rwq_data[17]) & (csr_read | csr_write)) begin
            $display ("Attempted CSR write while a CSR transaction was in progress ck1");
            $stop;
         end
         if ((rwq_address[3:0] == ETH_RD_DATA[6:3]) & rwq_rdvld & (csr_read | csr_write)) begin
            $display ("Attempted to read the CSR read data register while a CSR transaction was in progress ck2");
            $stop;
         end
         if (rwq_afull & ~flagged_afull) begin
            $display ("rwq is almost full");
            $stop;
            flagged_afull = 1;
         end
         //synopsys translate_on
      end
      if (csr_read) csr_readdata_r <= csr_readdata;
   end
   
   // IKL stuff...
   // egress
   wire [NCT*PPC-1:0] egr_cte_valid;
   wire [255:0]       egr_cte_data[NCT*PPC-1:0];
   wire [NCT*PPC-1:0] cte_egr_ready;
   
   // ingress
   wire [NCT*PPC-1:0] cti_ing_valid;
   wire [255:0]       cti_ing_data[NCT*PPC-1:0];
   wire [NCT*PPC-1:0] ing_cti_ready;
   
   wire 	      str_reset_n;
   wire 	      krn_reset_n;
   wire 	      str_clk; // Streaming agent clock. pb_ikl logic. Must be greater than 151.25 MHZ.
   wire 	      krn_clk; // Same clock the kernels run on. Any frequency supported.
   
   wire [11:0] 	      pb_csr_address;
   wire 	      pb_csr_read;
   wire [31:0] 	      pb_csr_readdata_scr;
   wire 	      pb_csr_write;
   wire [31:0] 	      pb_csr_writedata;
   wire 	      pb_csr_waitrequest_scr;
   
   //   assign krn_clk = pClkDiv4;      //  50.00 MHZ
   //   assign krn_clk = pClkDiv2;      // 100.00 MHZ
   //   assign krn_clk = pClk;          // 200.00 MHZ
   //   assign krn_clk = hssi.f2a_tx_clk; // 312.50 MHZ MAC TX cloc
   //   assign str_clk = pClk;            // 200.00 MHZ
   
   // krn_clk and str_clk can be asynchronous to each other
   // They can be connected to any clock.
   // str_clk must be faster than 156.25 MHZ.
   // krn_clk can be any speed.
   assign krn_clk = hssi.f2a_tx_clk; // 312.50 MHZ MAC TX clock
   assign str_clk = pClk;            // 200.00 MHZ
   
   pb_mac_40g #(.NCT(NCT), .PPC(PPC))
   pb_mac_40g (
               .csr_clk                (ccip_clk),
               .csr_rst_n              (~csr_rst),
               .tx_rst_n               (~tx_rst),
               .rx_rst_n               (~rx_rst),
	       
               .krn_clk                (krn_clk),             // Same clock the kernels run on. Any frequency supported.
               .str_clk                (str_clk),             // Streaming agent clock. pb_ikl logic. Must be greater than 151.25 MHZ.
               .tx_clk_312             (hssi.f2a_tx_clk),     // MAC TX clock 312.5 MHZ
               .rx_clk_312             (hssi.f2a_rx_clk_ln0), // MAC RX clock 312.5 MHZ (recovered from rx)
	       
               // added ports for 40G mac...
               .f2a_rx_locked_ln0      (hssi.f2a_rx_locked_ln0),
               .f2a_tx_locked          (hssi.f2a_tx_locked),
	       
               .tx_full                (hssi.f2a_tx_enh_fifo_full[3:0]),
               .tx_empty               (hssi.f2a_tx_enh_fifo_empty[3:0]),
               .tx_pfull               (hssi.f2a_tx_enh_fifo_pfull[3:0]),
               .tx_pempty              (hssi.f2a_tx_enh_fifo_pempty[3:0]),
	       
               .rx_full                (hssi.f2a_rx_enh_fifo_full[3:0]),
               .rx_empty               (hssi.f2a_rx_enh_fifo_empty[3:0]),
               .rx_pfull               (hssi.f2a_rx_enh_fifo_pfull[3:0]),
               .rx_pempty              (hssi.f2a_rx_enh_fifo_pempty[3:0]),
	       
               .a2f_rx_seriallpbken    (), // (hssi.a2f_rx_seriallpbken[3:0]), // causes data to lookback in the PHY. Input signal to the PHY.
               .a2f_rx_set_locktodata  (hssi.a2f_rx_set_locktodata[3:0]),
               .a2f_rx_set_locktoref   (hssi.a2f_rx_set_locktoref[3:0]),
               .f2a_prmgmt_ram_ena     (hssi.f2a_prmgmt_ram_ena),
	       
               // serdes controls
               .tx_analogreset         (hssi.a2f_tx_analogreset[3:0]),
               .tx_digitalreset        (hssi.a2f_tx_digitalreset[3:0]),
               .rx_analogreset         (hssi.a2f_rx_analogreset[3:0]),
               .rx_digitalreset        (hssi.a2f_rx_digitalreset[3:0]),
	       
               .tx_cal_busy            (hssi.f2a_tx_cal_busy),
               .rx_cal_busy            (hssi.f2a_rx_cal_busy),
               .rx_is_lockedtodata     (hssi.f2a_rx_is_lockedtodata[3:0]),
               .atx_pll_locked         (hssi.f2a_tx_pll_locked),
	       
               // serdes data pipe
               .xgmii_tx_valid         (hssi.a2f_tx_enh_data_valid[3:0]),
               .xgmii_tx_control       (hssi.a2f_tx_control[71:0]),
               .xgmii_tx_data          (hssi.a2f_tx_parallel_data[511:0]),
	       
               .xgmii_rx_valid         (hssi.f2a_rx_enh_data_valid[0]),
               .xgmii_rx_control       (hssi.f2a_rx_control[7:0]),
               .xgmii_rx_data          (hssi.f2a_rx_parallel_data[511:0]),
               .rx_enh_fifo_rd_en      (hssi.a2f_rx_enh_fifo_rd_en[3:0]),
	       
               // csr interface synchronous to csr_clk
               .csr_read               (csr_read),
               .csr_write              (csr_write),
               .csr_writedata          (csr_writedata),
               .csr_readdata           (csr_readdata),
               .csr_address            (csr_address),
               .csr_waitrequest        (csr_waitrequest),
	       
               // IKL stuff...
               .egr_cte_valid                         (egr_cte_valid),               // input  [(NCT*PPC)-1:0]     egr_cte_valid
               .egr_cte_data                          (egr_cte_data),                // input  [(NCT*PPC*256)-1:0] egr_cte_data
               .cte_egr_ready                         (cte_egr_ready),               // output [(NCT*PPC)-1:0]     cte_egr_ready
	       
               .cti_ing_valid                         (cti_ing_valid),               // output [(NCT*PPC)-1:0]      cti_ing_valid
               .cti_ing_data                          (cti_ing_data),                // output [(NCT*PPC*256)-1:0]  cti_ing_data
               .ing_cti_ready                         (ing_cti_ready),               // input  [(NCT*PPC)-1:0]      ing_cti_ready
	       
               // additional csr interface. synchronous to str_clk. NOT AV-MM SPEC COMPILANT!
               .pb_csr_address                        (pb_csr_address),              // output  [11:0]   pb_csr_address
               .pb_csr_read                           (pb_csr_read),                 // output           pb_csr_read
               .pb_csr_readdata_scr                   (pb_csr_readdata_scr),         // input [031:000]  pb_csr_readdata_scr
               .pb_csr_write                          (pb_csr_write),                // output           pb_csr_write
               .pb_csr_writedata                      (pb_csr_writedata),            // output [031:000] pb_csr_writedata
               .pb_csr_waitrequest_scr                (pb_csr_waitrequest_scr),      // input            pb_csr_waitrequest
	       
               .str_reset_n                           (str_reset_n),                 // output tx_rst_n synchronized to str_clk
               .krn_reset_n                           (krn_reset_n)                  // output tx_rst_n synchronized to krn_clk
               );
   
   
   // end IKL stuff...
   
   assign hssi.a2f_prmgmt_fatal_err     = 1'b0;
   assign hssi.a2f_prmgmt_dout          = 32'h5aa562cc;
   assign hssi.a2f_rx_seriallpbken[3:0] = sloop; // causes data to lookback in the PHY. Input signal to the PHY.
   assign hssi.a2f_init_start           = init_start;
   assign init_done                     = hssi.f2a_init_done;
   
   reg [31:00] 	      pb_csr_readdata_00;
   wire 	      pb_scr_waitrequest_00;
   
   
   assign pb_csr_readdata_scr    = pb_csr_readdata_00;
   assign pb_csr_waitrequest_scr = pb_scr_waitrequest_00;
   
   // amm address decode...
   wire 	      amm_hit_00;
   wire 	      pb_csr_read_00;
   wire 	      pb_csr_write_00;
   assign amm_hit_00  = (pb_csr_address[11:8] == 4'h0);
   assign pb_csr_read_00  = amm_hit_00 & pb_csr_read;
   assign pb_csr_write_00 = amm_hit_00 & pb_csr_write;
   
   // start_e0: start FSM
   // start_e1: probe one result
   // kill_e0: end
   reg                start_e0,start_e1,start_e2,start_e3;
   reg                start_i0,start_i1,start_i2,start_i3;
   reg                kill_e0, kill_e1, kill_e2, kill_e3;
   reg                kill_i0, kill_i1, kill_i2, kill_i3;
   
   wire               start_syn_e0,start_syn_e1,start_syn_e2,start_syn_e3;
   wire               start_syn_i0,start_syn_i1,start_syn_i2,start_syn_i3;
   wire               kill_syn_e0, kill_syn_e1, kill_syn_e2, kill_syn_e3;
   wire               kill_syn_i0, kill_syn_i1, kill_syn_i2, kill_syn_i3;
   
   reg                pb_csr_read_00_r1;
   
   wire [63:0]        idata_0,  idata_1,  idata_2,  idata_3;
   wire               ivalid_0, ivalid_1, ivalid_2, ivalid_3;
   reg [7:0]          status_0, status_1, status_2, status_3;
   logic [15:0]       node_id;
   logic [15:0]       num_node;
   logic [31:0]       total_length;
   logic [63:0]       lpbk_latency;
   logic [47:0]       stall_host_in;
   logic [47:0]       stall_host_out;
   logic [47:0]       stall_eth_in;
   logic [47:0]       stall_eth_out;
   logic [127:0]       debug_status;
   t_ccip_clAddr      mem_addr;
   t_ccip_clAddr      done_addr;
   ///* synthesis preserve dont_replicate */

   always @(posedge str_clk) begin
      if (~str_reset_n) begin
         start_e0 <= 1'b0;
         start_e1 <= 1'b0;
         start_e2 <= 1'b0;
         start_e3 <= 1'b0;
         start_i0 <= 1'b0;
         start_i1 <= 1'b0;
         start_i2 <= 1'b0;
         start_i3 <= 1'b0;
         kill_e0 <= 1'b0;
         kill_e1 <= 1'b0;
         kill_e2 <= 1'b0;
         kill_e3 <= 1'b0;
         kill_i0 <= 1'b0;
         kill_i1 <= 1'b0;
         kill_i2 <= 1'b0;
         kill_i3 <= 1'b0;
         pb_csr_readdata_00 <= 32'h0;
         node_id <= 16'd0;
         num_node <= 16'd0;
         total_length <= 32'd0;
      end else begin
         start_e0 <= 1'b0;
         start_e1 <= 1'b0;
         start_e2 <= 1'b0;
         start_e3 <= 1'b0;
         start_i0 <= 1'b0;
         start_i1 <= 1'b0;
         start_i2 <= 1'b0;
         start_i3 <= 1'b0;
         kill_e0 <= 1'b0;
         kill_e1 <= 1'b0;
         kill_e2 <= 1'b0;
         kill_e3 <= 1'b0;
         kill_i0 <= 1'b0;
         kill_i1 <= 1'b0;
         kill_i2 <= 1'b0;
         kill_i3 <= 1'b0;
         pb_csr_readdata_00 <= 32'h0;
	 
         if (pb_csr_address[7:2] == 0) begin
            if (pb_csr_write_00) begin
               start_e0 <= pb_csr_writedata[0];
               start_e1 <= pb_csr_writedata[1];
               start_e2 <= pb_csr_writedata[2];
               start_e3 <= pb_csr_writedata[3];
               start_i0 <= pb_csr_writedata[4];
               start_i1 <= pb_csr_writedata[5];
               start_i2 <= pb_csr_writedata[6];
               start_i3 <= pb_csr_writedata[7];
               kill_e0 <= pb_csr_writedata[16];
               kill_e1 <= pb_csr_writedata[17];
               kill_e2 <= pb_csr_writedata[18];
               kill_e3 <= pb_csr_writedata[19];
               kill_i0 <= pb_csr_writedata[20];
               kill_i1 <= pb_csr_writedata[21];
               kill_i2 <= pb_csr_writedata[22];
               kill_i3 <= pb_csr_writedata[23];
            end
	    
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= 32'h0; // nothing to read, only pusles.
            end
         // end else if (pb_csr_address[7:2] == 1) begin
         //    if (pb_csr_read_00) begin
         //       //pb_csr_readdata_00 <= {status_3, status_2, status_1, status_0};
         //       pb_csr_readdata_00 <= {31'd0, result_valid};
         //    end
         end else if (pb_csr_address[7:2] == 2) begin
            if (pb_csr_write_00) begin
               node_id <= pb_csr_writedata[15:0];
               num_node <= pb_csr_writedata[31:16]; // vec_length*(NUM_NODE-2)
            end
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= {num_node, node_id};
            end
         end else if (pb_csr_address[7:2] == 3) begin
            if (pb_csr_write_00) begin
               total_length <= pb_csr_writedata;
            end
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= total_length;
            end
         end else if (pb_csr_address[7:2] == 4) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= lpbk_latency[31:0];
            end
         end else if (pb_csr_address[7:2] == 5) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= lpbk_latency[63:32];
            end
         end else if (pb_csr_address[7:2] == 6) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= num_rqst_sent;
            end
         end else if (pb_csr_address[7:2] == 7) begin
            if (pb_csr_write_00) begin
               done_addr[31:0] <= pb_csr_writedata;
            end
            if (pb_csr_read_00) begin
               // pb_csr_readdata_00 <= num_rsp_rcvd;
               pb_csr_readdata_00 <= done_addr[31:0];
            end
         end else if (pb_csr_address[7:2] == 8) begin
            if (pb_csr_write_00) begin
               done_addr[41:32] <= pb_csr_writedata[9:0];
            end
            if (pb_csr_read_00) begin
               // pb_csr_readdata_00 <= num_rsp_send;
               pb_csr_readdata_00 <= done_addr[41:32];
            end
         end else if (pb_csr_address[7:2] == 9) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= stall_host_in[31:0];
            end
         end else if (pb_csr_address[7:2] == 10) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= stall_host_out[31:0];
            end
         end else if (pb_csr_address[7:2] == 11) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00[15:0] <= stall_host_in[47:32];
               pb_csr_readdata_00[31:16] <= stall_host_out[47:32];
            end
         end else if (pb_csr_address[7:2] == 12) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= stall_eth_in[31:0];
            end
         end else if (pb_csr_address[7:2] == 13) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= stall_eth_out[31:0];
            end
         end else if (pb_csr_address[7:2] == 14) begin
            if (pb_csr_read_00) begin
               // pb_csr_readdata_00 <= total_send_count;
               pb_csr_readdata_00[15:0] <= stall_eth_in[47:32];
               pb_csr_readdata_00[31:16] <= stall_eth_out[47:32];
            end
         end else if (pb_csr_address[7:2] == 15) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= debug_status[31:0];
            end
         end else if (pb_csr_address[7:2] == 16) begin
            if (pb_csr_write_00) begin
               mem_addr[31:0] <= pb_csr_writedata;
            end
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= mem_addr[31:0];
            end
         end else if (pb_csr_address[7:2] == 17) begin
            if (pb_csr_write_00) begin
               mem_addr[41:32] <= pb_csr_writedata[9:0];
            end
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= mem_addr[41:32];
            end
         end else if (pb_csr_address[7:2] == 18) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= debug_status[63:32];
            end
         end else if (pb_csr_address[7:2] == 19) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= debug_status[95:64];
            end
         end else if (pb_csr_address[7:2] == 20) begin
            if (pb_csr_read_00) begin
               pb_csr_readdata_00 <= debug_status[127:96];
            end
         end
      end // else: !if(~krn_reset_n)
      pb_csr_read_00_r1 <= pb_csr_read_00;
   end // always @ (posedge krn_clk)
   
   assign pb_scr_waitrequest_00 = pb_csr_read_00 & ~pb_csr_read_00_r1; // exactly one cycle wait.
   
   pulse_sync seps0 (.clkin(str_clk), .clkout(krn_clk), .in(start_e0), .out(start_syn_e0));
   pulse_sync seps1 (.clkin(str_clk), .clkout(krn_clk), .in(start_e1), .out(start_syn_e1));
   pulse_sync seps2 (.clkin(str_clk), .clkout(krn_clk), .in(start_e2), .out(start_syn_e2));
   pulse_sync seps3 (.clkin(str_clk), .clkout(krn_clk), .in(start_e3), .out(start_syn_e3));
   
   pulse_sync sips0 (.clkin(str_clk), .clkout(krn_clk), .in(start_i0), .out(start_syn_i0));
   pulse_sync sips1 (.clkin(str_clk), .clkout(krn_clk), .in(start_i1), .out(start_syn_i1));
   pulse_sync sips2 (.clkin(str_clk), .clkout(krn_clk), .in(start_i2), .out(start_syn_i2));
   pulse_sync sips3 (.clkin(str_clk), .clkout(krn_clk), .in(start_i3), .out(start_syn_i3));
   
   pulse_sync keps0 (.clkin(str_clk), .clkout(krn_clk), .in(kill_e0), .out(kill_syn_e0));
   pulse_sync keps1 (.clkin(str_clk), .clkout(krn_clk), .in(kill_e1), .out(kill_syn_e1));
   pulse_sync keps2 (.clkin(str_clk), .clkout(krn_clk), .in(kill_e2), .out(kill_syn_e2));
   pulse_sync keps3 (.clkin(str_clk), .clkout(krn_clk), .in(kill_e3), .out(kill_syn_e3));
   
   pulse_sync kips0 (.clkin(str_clk), .clkout(krn_clk), .in(kill_i0), .out(kill_syn_i0));
   pulse_sync kips1 (.clkin(str_clk), .clkout(krn_clk), .in(kill_i1), .out(kill_syn_i1));
   pulse_sync kips2 (.clkin(str_clk), .clkout(krn_clk), .in(kill_i2), .out(kill_syn_i2));
   pulse_sync kips3 (.clkin(str_clk), .clkout(krn_clk), .in(kill_i3), .out(kill_syn_i3));
   
   reg reset_e0_n;
   reg reset_e1_n;
   reg reset_e2_n;
   reg reset_e3_n;
   reg reset_i0_n;
   reg reset_i1_n;
   reg reset_i2_n;
   reg reset_i3_n;
      
   always @(posedge krn_clk) begin
      if (~krn_reset_n) begin
         status_0 <= 8'b0;
         status_1 <= 8'b0;
         status_2 <= 8'b0;
         status_3 <= 8'b0;
      end else begin
         if (kill_syn_i0) begin
            status_0 <= 8'h0;
         end else if (ivalid_0) begin
            status_0 <= idata_0[7:0];
         end
         if (kill_syn_i1) begin
            status_1 <= 8'h0;
         end else if (ivalid_1) begin
            status_1 <= idata_1[7:0];
         end
         if (kill_syn_i2) begin
            status_2 <= 8'h0;
         end else if (ivalid_2) begin
            status_2 <= idata_2[7:0];
         end
         if (kill_syn_i3) begin
            status_3 <= 8'h0;
         end else if (ivalid_3) begin
            status_3 <= idata_3[7:0];
         end
      end // else: !if(~krn_reset_n)
      
      reset_e0_n <= krn_reset_n & ~kill_syn_e0;
      reset_e1_n <= krn_reset_n & ~kill_syn_e1;
      reset_e2_n <= krn_reset_n & ~kill_syn_e2;
      reset_e3_n <= krn_reset_n & ~kill_syn_e3;
      reset_i0_n <= krn_reset_n & ~kill_syn_i0;
      reset_i1_n <= krn_reset_n & ~kill_syn_i1;
      reset_i2_n <= krn_reset_n & ~kill_syn_i2;
      reset_i3_n <= krn_reset_n & ~kill_syn_i3;
      
   end // always @ (posedge krn_clk)

   // All reduce logic
   logic ccip_rd_rqst_valid;
   logic [41:0] ccip_rd_rqst_data;
   logic ccip_rd_rsp_ready;
   logic ccip_wr_rqst_valid;
   logic [555:0] ccip_wr_rqst_data;
   logic [127:0] debug_status_in;

   logic [255:0] eth_in;
   logic eth_in_valid;
   logic eth_in_ready;
   logic eth_in_fifo_full;
   logic eth_in_fifo_empty;
   logic [255:0] eth_out;
   logic eth_out_valid;
   logic eth_out_ready;
   logic eth_out_fifo_full;
   logic eth_out_fifo_empty;

   logic [15:0] num_node_out;
   logic [15:0] node_id_out;
   logic [31:0] total_length_out;
   t_ccip_clAddr mem_addr_out;
   t_ccip_clAddr done_addr_out;

   alt_sync_regs_m2 #(.WIDTH(127), .DEPTH(2)) sync0(
      .clk(str_clk),
      .din(debug_status_in),
      .dout(debug_status)
   );

   alt_sync_regs_m2 #(.WIDTH(16), .DEPTH(2)) sync1(
      .clk(krn_clk),
      .din(num_node),
      .dout(num_node_out)
   );

   alt_sync_regs_m2 #(.WIDTH(16), .DEPTH(2)) sync2(
      .clk(krn_clk),
      .din(node_id),
      .dout(node_id_out)
   );

   alt_sync_regs_m2 #(.WIDTH(32), .DEPTH(2)) sync3(
      .clk(krn_clk),
      .din(total_length),
      .dout(total_length_out)
   );

   alt_sync_regs_m2 #(.WIDTH(42), .DEPTH(2)) sync4(
      .clk(krn_clk),
      .din(mem_addr),
      .dout(mem_addr_out)
   );

   alt_sync_regs_m2 #(.WIDTH(42), .DEPTH(2)) sync5(
      .clk(ccip_clk),
      .din(done_addr),
      .dout(done_addr_out)
   );

   all_reduce all_reduce_inst(
      .krn_clk(krn_clk),
      .krn_reset_n(krn_reset_n),
      .reset_e0_n(reset_e0_n),
      .ccip_clk(ccip_clk),
      .ccip_rd_rqst_ready(!afu.c0TxAlmFull),
      .ccip_rd_rqst_valid(ccip_rd_rqst_valid),
      .ccip_rd_rqst_data(ccip_rd_rqst_data),
      .ccip_rd_rsp_valid(cci_c0Rx_isReadRsp(afu.c0Rx)),
      .ccip_rd_rsp_data(afu.c0Rx.data),
      .ccip_rd_rsp_ready(ccip_rd_rsp_ready),
      .ccip_wr_rqst_ready(!afu.c1TxAlmFull),
      .ccip_wr_rqst_valid(ccip_wr_rqst_valid),
      .ccip_wr_rqst_data(ccip_wr_rqst_data),
      .cti_ing_data(cti_ing_data[1]),
      .cti_ing_valid(cti_ing_valid[1]),
      .ing_cti_ready(ing_cti_ready[1]),
      .egr_cte_data(egr_cte_data[0]),
      .egr_cte_valid(egr_cte_valid[0]),
      .cte_egr_ready(cte_egr_ready[0]),

      .start_syn_e0(start_syn_e0),
      .kill_syn_e0(kill_syn_e0),
      .mem_addr(mem_addr_out),
      .done_addr(done_addr_out),
      .total_length(total_length_out),
      .node_id(node_id_out),
      .num_node(num_node_out),
      .lpbk_latency(lpbk_latency),
      .num_rqst_sent(num_rqst_sent),
      .stall_host_in(stall_host_in),
      .stall_host_out(stall_host_out),
      .stall_eth_in(stall_eth_in),
      .stall_eth_out(stall_eth_out),
      .debug_status(debug_status_in)
   );

   //======================================================================================
   //
   //   CCIP read logic
   //
   //======================================================================================
   //ccip read rqst (ccip_clk domain)
   t_cci_mpf_c0_ReqMemHdr rd_hdr;
   t_cci_mpf_ReqMemHdrParams rd_hdr_params;
   always_comb begin
      // Use virtual addresses
      rd_hdr_params = cci_mpf_defaultReqHdrParams(1);
      // Let the FIU pick the channel
      rd_hdr_params.vc_sel = eVC_VA;
      // Read 4 lines (the size of an entry in the list)
      rd_hdr_params.cl_len = eCL_LEN_4;

      // Generate the header
      rd_hdr = cci_mpf_c0_genReqHdr(eREQ_RDLINE_I,
                                   ccip_rd_rqst_data,
                                   t_cci_mdata'(0),
                                   rd_hdr_params);
   end

   always_ff @(posedge ccip_clk) begin
      if (pck_cp2af_softReset_T1) begin
         afu.c0Tx.valid <= 'b0;
      end else begin
         afu.c0Tx <= cci_mpf_genC0TxReadReq(rd_hdr, (!afu.c0TxAlmFull) && ccip_rd_rqst_valid);
      end
   end

   always_ff @(posedge ccip_clk) begin
      if (pck_cp2af_softReset_T1) begin
         num_rsp_rcvd <= 'b0;
      end else begin
         if (afu.c0Tx.valid) begin
            num_rsp_rcvd <= num_rsp_rcvd + 32'd1;
         end
      end
   end

   //======================================================================================
   //
   //   CCIP write logic
   //
   //======================================================================================
   // ccip_clk domain
   t_cci_mpf_c1_ReqMemHdr wr_hdr;
   t_cci_mpf_ReqMemHdrParams wr_hdr_params;
   always_comb begin
      // Use virtual addresses
      wr_hdr_params = cci_mpf_defaultReqHdrParams(1);
      // Let the FIU pick the channel
      wr_hdr_params.vc_sel = eVC_VA;
      // Read 4 lines (the size of an entry in the list)
      wr_hdr_params.cl_len = (ccip_wr_rqst_data[555]) ? eCL_LEN_4 : eCL_LEN_1;
      wr_hdr_params.sop = ccip_wr_rqst_data[554];

      wr_hdr = cci_mpf_c1_genReqHdr(eREQ_WRLINE_I,
                                    ccip_wr_rqst_data[553:512],
                                    t_cci_mdata'(0),
                                    wr_hdr_params);
   end

   always_ff @(posedge ccip_clk) begin
      if (pck_cp2af_softReset_T1) begin
         afu.c1Tx.valid <= 'b0;
      end else begin
         afu.c1Tx.valid <= ccip_wr_rqst_valid && (!afu.c1TxAlmFull);
      end
      afu.c1Tx.hdr <= wr_hdr;
      afu.c1Tx.data <= ccip_wr_rqst_data[511:0];
   end

   always_ff @(posedge ccip_clk) begin
      if (pck_cp2af_softReset_T1) begin
         num_rsp_send <= 'b0;
         debug0 <= 'b0;
         debug1 <= 'b0;
         debug2 <= 'b0;
         debug3 <= 'b0;
      end else begin
         if (afu.c1Tx.valid) begin
            num_rsp_send <= num_rsp_send + 32'd1;
            if (debug0 == 64'd0) debug0[41:0] <= afu.c1Tx.hdr.base.address;
            if (debug0 != 64'd0) debug1[41:0] <= afu.c1Tx.hdr.base.address;
            debug2 <= afu.c1Tx.data[63:0];
            debug3 <= afu.c1Tx.data[319:256];
         end
      end
   end

   //======================================================================================
   //
   //   ETH logic
   //
   //======================================================================================
   // assign ing_cti_ready[1] = eth_in_ready;
   // assign eth_in_valid = cti_ing_valid[1];
   // assign eth_in = cti_ing_data[1];

   // assign eth_out_ready = cte_egr_ready[0];
   // assign egr_cte_valid[0] = eth_out_valid;
   // assign egr_cte_data[0] = eth_out;

   assign egr_cte_data[1] = 'd0;
   assign egr_cte_data[2] = 'd0;
   assign egr_cte_data[3] = 'd0;
   assign egr_cte_valid[1] = 'b0;
   assign egr_cte_valid[2] = 'b0;
   assign egr_cte_valid[3] = 'b0;
   assign ing_cti_ready[0] = 'b1;
   assign ing_cti_ready[2] = 'b1;
   assign ing_cti_ready[3] = 'b1;
   
endmodule
