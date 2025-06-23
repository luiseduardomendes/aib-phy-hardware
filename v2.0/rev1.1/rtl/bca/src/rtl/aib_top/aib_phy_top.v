// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2022 HCL Technologies Ltd.
// Copyright (C) 2022 Blue Cheetah Analog Design, Inc.
// Copyright (C) 2021 Intel Corporation. 

module aib_phy_top #(
    parameter ACTIVE_CHNLS = 1,     // <<< FIX: Added to control active channels
    parameter NBR_CHNLS = 24,       // Total number of channels 
    parameter NBR_BUMPS = 102,      // Number of BUMPs
    parameter NBR_PHASES = 4,       // Number of phases
    parameter NBR_LANES = 40,       // Number of lanes
    parameter MS_SSR_LEN = 81,      // Data size for leader side band
    parameter SL_SSR_LEN = 73,      // Data size for follower side band
    parameter SCAN_STR_PER_CH = 10 ,// Scan data input and data output size
    parameter [0:0] BERT_BUF_MODE_EN = 1  // Enables Buffer mode for BERT
    )
 ( 

// Power supply pins
inout vddc1,  // vddc1 power supply pin (low noise for clock circuits)
inout vddc2,  // vddc2 power supply pin for IOs circuits
inout vddtx,  // vddtx power supply pin for high-speed data
inout vss,    // Ground

// IO PADs
inout wire [NBR_BUMPS-1:0]   iopad_ch0_aib,  // IO pad channel 0
inout wire [NBR_BUMPS-1:0]   iopad_ch1_aib,  // IO pad channel 1
inout wire [NBR_BUMPS-1:0]   iopad_ch2_aib,  // IO pad channel 2
inout wire [NBR_BUMPS-1:0]   iopad_ch3_aib,  // IO pad channel 3
inout wire [NBR_BUMPS-1:0]   iopad_ch4_aib,  // IO pad channel 4
inout wire [NBR_BUMPS-1:0]   iopad_ch5_aib,  // IO pad channel 5
inout wire [NBR_BUMPS-1:0]   iopad_ch6_aib,  // IO pad channel 6
inout wire [NBR_BUMPS-1:0]   iopad_ch7_aib,  // IO pad channel 7
inout wire [NBR_BUMPS-1:0]   iopad_ch8_aib,  // IO pad channel 8
inout wire [NBR_BUMPS-1:0]   iopad_ch9_aib,  // IO pad channel 9
inout wire [NBR_BUMPS-1:0]   iopad_ch10_aib, // IO pad channel 10
inout wire [NBR_BUMPS-1:0]   iopad_ch11_aib, // IO pad channel 11
inout wire [NBR_BUMPS-1:0]   iopad_ch12_aib, // IO pad channel 12
inout wire [NBR_BUMPS-1:0]   iopad_ch13_aib, // IO pad channel 13
inout wire [NBR_BUMPS-1:0]   iopad_ch14_aib, // IO pad channel 14
inout wire [NBR_BUMPS-1:0]   iopad_ch15_aib, // IO pad channel 15
inout wire [NBR_BUMPS-1:0]   iopad_ch16_aib, // IO pad channel 16
inout wire [NBR_BUMPS-1:0]   iopad_ch17_aib, // IO pad channel 17
inout wire [NBR_BUMPS-1:0]   iopad_ch18_aib, // IO pad channel 18
inout wire [NBR_BUMPS-1:0]   iopad_ch19_aib, // IO pad channel 19
inout wire [NBR_BUMPS-1:0]   iopad_ch20_aib, // IO pad channel 20
inout wire [NBR_BUMPS-1:0]   iopad_ch21_aib, // IO pad channel 21
inout wire [NBR_BUMPS-1:0]   iopad_ch22_aib, // IO pad channel 22
inout wire [NBR_BUMPS-1:0]   iopad_ch23_aib, // IO pad channel 23

inout  iopad_device_detect,  // Indicates the presence of a valid leader
inout  iopad_power_on_reset, // Perfoms a power-on-reset in the adapter

input  [NBR_LANES*NBR_PHASES*2*NBR_CHNLS-1 :0] data_in_f,  // FIFO mode input
output [NBR_LANES*NBR_PHASES*2*NBR_CHNLS-1 :0] data_out_f, // FIFO mode output

input  [NBR_LANES*2*NBR_CHNLS-1 :0] data_in,  // Register mode data input
output [NBR_LANES*2*NBR_CHNLS-1 :0] data_out, // Register mode data output

input  [NBR_CHNLS-1:0]    m_ns_fwd_clk, // Clock for transmitting data from the
                                        // near side to the far side
input  [NBR_CHNLS-1:0]    m_ns_rcv_clk, // Receive-domain clock forwarded from
                                        // the near side to the far side for
                                        // transmitting data from the far side
output [NBR_CHNLS-1:0]    m_fs_rcv_clk, // Received from the far side and
                                        // converted from quasi-differential to
                                        // single-ended
output [NBR_CHNLS-1:0]    m_fs_fwd_clk, // Received from the far side and
                                        // converted from quasi-differential to 
                                        // single-ended
input  [NBR_CHNLS-1:0]    m_wr_clk,     // Clocks data_in_f
input  [NBR_CHNLS-1:0]    m_rd_clk,     // Clocks data_out_f
output [NBR_CHNLS-1:0]    ns_fwd_clk,   // Near side forwarded clock to SOC/MAC
output [NBR_CHNLS-1:0]    ns_fwd_clk_div, // Divided near side forwarded clock
                                          // to SoC/MAC
output [NBR_CHNLS-1:0]    fs_fwd_clk,     // Far side forwarded clock to TX MAC
output [NBR_CHNLS-1:0]    fs_fwd_clk_div, // Divided far side forwarded clock to
                                          // SoC/MAC

input  [NBR_CHNLS-1:0]    ns_adapter_rstn, // Resets the AIB Adapter
input  [NBR_CHNLS-1:0]    ns_mac_rdy,   // Indicates that the near side is ready
                                        // for calibration and data transfer
output [NBR_CHNLS-1:0]    fs_mac_rdy,   // Indicates that the far-side MAC is
                                        // ready to transmit data
input                     i_conf_done,  // Single control to reset all AIB
                                        // channels in the interface. LO=reset,
                                        // HI=out of reset.
input                     i_osc_clk,    // Free running oscillator clock for a
                                        // leader interface

input [NBR_CHNLS-1:0]  ms_rx_dcc_dll_lock_req, // Initiates calibration of      
                                               // receive path for a leader
                                               // interface
input  [NBR_CHNLS-1:0] ms_tx_dcc_dll_lock_req, // Initiates calibration of      
                                               // transmit path for a leader
                                               // interface
input  [NBR_CHNLS-1:0] sl_tx_dcc_dll_lock_req, // Initiates calibration of      
                                               // receive path for a follower
                                               // interface
input  [NBR_CHNLS-1:0] sl_rx_dcc_dll_lock_req, // Initiates calibration of      
                                               // transmit path for a follower
                                               // interface

output [NBR_CHNLS-1:0] ms_tx_transfer_en, // Indicates that leader has completed
                                          // its TX path calibration and is
                                          // ready to receive data.
output [NBR_CHNLS-1:0] ms_rx_transfer_en, // Indicates that leader has completed
                                          // its RX path calibration and is
                                          // ready to receive data.
output [NBR_CHNLS-1:0] sl_tx_transfer_en, // Indicates that follower has
                                          // completed its TX path calibration
                                          // and is ready to receive data
output [NBR_CHNLS-1:0] sl_rx_transfer_en, // Indicates that follower has
                                          // completed its RX path calibration
                                          // and is ready to receive data
output [NBR_CHNLS-1:0] m_rx_align_done,   // Indicates that the receiving AIB
                                          // Adapter is aligned to incoming word
                                          // marked data

output [MS_SSR_LEN*NBR_CHNLS-1:0] sr_ms_tomac, // Leader  sideband data
output [SL_SSR_LEN*NBR_CHNLS-1:0] sr_sl_tomac, // Follower  sideband data

input dual_mode_select, // Low: AIB interface is a Follower.
                        // High: AIB interface is a Leadee.
input m_gen2_mode,      // If LO when the AIB interface is released from reset, 
                        // the AIB interface is in Gen1 mode. If HI when the AIB
                        // interface is released from reset, the AIB interface
                        // is in Gen2 mode.

//Aux channel
input  m_por_ovrd, // Intended for standalone test without an AIB partner. 
                   // If LO, the Leader is not in reset. If HI and the AIB
                   // interface signal power_on_reset input is 1, the Leader is 
                   // held in reset. The Leader's power_on_reset input is
                   // required to have a weak pullup so that a no connect on
                   // that input will result in power_on_reset=HI.

input  m_device_detect_ovrd, // Intended for standalone test without an AIB
                             // partner. If LO, the Follower uses the AIB 
                             // interface device_detect input. If HI, the
                             // Follower outputs m_device_detect=1.
input  i_m_power_on_reset,   // Controls the power_on_reset signal sent to the
                             // Leader. Must be stable at power-up.

output m_device_detect, // A copy of the AIB interface signal device_detect from
                        // the Leader, qualified by m_device_detect_ovrd.

output o_m_power_on_reset, // A copy of the power_on_reset signal from the
                           // Follower, qualified by override from m_por_ovrd.

//JTAG signals
input  i_jtag_clkdr,     // Test clock
input  i_jtag_clksel,    // Select the JTAG clock or the operational clock for
                         // the register in the I/O block
input  i_jtag_intest,    // Enable testing of data path
input  i_jtag_mode,      // Teste mode select
input  i_jtag_rstb,      // JTAG reset
input  i_jtag_rstb_en,   // JTAG reset enable
input  i_jtag_tdi,       // Test data input
input  i_jtag_tx_scanen, // JTAG shift DR, active high
input  i_jtag_weakpdn,   // Enable weak pull-down on all AIB IO blocks
input  i_jtag_weakpu,    // Enable weak pull-up on all AIB IO blocks
output o_jtag_tdo,       // last boundary scan chain output, TDO

input  i_scan_clk,       // Scan clock
input  i_scan_clk_500m,  // Scan clock 500m
input  i_scan_clk_1000m, // Scan clock 1000m
input  i_scan_en,        // Scan enable
input  i_scan_mode,      // Scan mode
input  [(NBR_CHNLS*SCAN_STR_PER_CH):0] i_scan_din,  // Scan data input
output [(NBR_CHNLS*SCAN_STR_PER_CH):0] i_scan_dout, // Scan data output


//AVMM interface
input        i_cfg_avmm_clk,      // Avalon interface clock
input        i_cfg_avmm_rst_n,    // Avalon interface reset
input [16:0] i_cfg_avmm_addr,     // address to be programmed
input [3:0]  i_cfg_avmm_byte_en,  // byte enable
input        i_cfg_avmm_read,     // Asserted to indicate the Cfg read access
input        i_cfg_avmm_write,    // Asserted to indicate the Cfg write access
input [31:0] i_cfg_avmm_wdata,    // data to be programmed
output       o_cfg_avmm_rdatavld, // Assert to indicate data available for Cfg 
                                  // read access
output [31:0] o_cfg_avmm_rdata,   // data returned for Cfg read access
output        o_cfg_avmm_waitreq, // asserted to indicate not ready for Cfg
                                  // access


//Redundancy control signals for IO buffers
input [27*NBR_CHNLS-1:0]  sl_external_cntl_26_0,  // user defined bits 26:0 for
                                                  // slave shift register
input [3*NBR_CHNLS-1:0]   sl_external_cntl_30_28, // user defined bits 30:28 for
                                                  // slave shift register
input [26*NBR_CHNLS-1:0]  sl_external_cntl_57_32, // user defined bits 57:32 for
                                                  // slave shift register
input [5*NBR_CHNLS-1:0]   ms_external_cntl_4_0,   // user defined bits 4:0 for 
                                                  // master shift register
input [58*NBR_CHNLS-1:0]  ms_external_cntl_65_8   // user defined bits 65:8 for
                                                  // master shift register
);

wire [NBR_CHNLS-1:0] o_cfg_avmm_rdatavld_ch;
wire [NBR_CHNLS-1:0] o_cfg_avmm_waitreq_ch;
wire [NBR_CHNLS-1:0] o_jtag_tdo_ch; 

// Avalon read data bus of each channel
wire [31:0]  o_rdata_ch_0;
wire [31:0]  o_rdata_ch_1;
wire [31:0]  o_rdata_ch_2;
wire [31:0]  o_rdata_ch_3;
wire [31:0]  o_rdata_ch_4;
wire [31:0]  o_rdata_ch_5;
wire [31:0]  o_rdata_ch_6;
wire [31:0]  o_rdata_ch_7;
wire [31:0]  o_rdata_ch_8;
wire [31:0]  o_rdata_ch_9;
wire [31:0]  o_rdata_ch_10;
wire [31:0]  o_rdata_ch_11;
wire [31:0]  o_rdata_ch_12;
wire [31:0]  o_rdata_ch_13;
wire [31:0]  o_rdata_ch_14;
wire [31:0]  o_rdata_ch_15;
wire [31:0]  o_rdata_ch_16;
wire [31:0]  o_rdata_ch_17;
wire [31:0]  o_rdata_ch_18;
wire [31:0]  o_rdata_ch_19;
wire [31:0]  o_rdata_ch_20;
wire [31:0]  o_rdata_ch_21;
wire [31:0]  o_rdata_ch_22;
wire [31:0]  o_rdata_ch_23;
wire [ 3:0]  o_aux_spare_nc;

wire [1:0] clkdiv_adc0;
wire [2:0] adc0_muxsel;
wire       adc0_en;
wire       adc0_start;
wire       adc0_chopen;
wire       adc0_dfx_en;
wire [9:0] adc0_out;
wire       adc0_done;
wire       adc0_anaviewout_nc;

wire [1:0] clkdiv_adc1;
wire [2:0] adc1_muxsel;
wire       adc1_en;
wire       adc1_start;
wire       adc1_chopen;
wire       adc1_dfx_en;
wire [9:0] adc1_out;
wire       adc1_done;
wire       adc1_anaviewout_nc;

wire [1:0] clkdiv_adc2;
wire [2:0] adc2_muxsel;
wire       adc2_en;
wire       adc2_start;
wire       adc2_chopen;
wire       adc2_dfx_en;
wire [9:0] adc2_out;
wire       adc2_done;
wire       adc2_anaviewout_nc;

wire [1:0] clkdiv_adc3;
wire [2:0] adc3_muxsel;
wire       adc3_en;
wire       adc3_start;
wire       adc3_chopen;
wire       adc3_dfx_en;
wire [9:0] adc3_out;
wire       adc3_done;
wire       adc3_anaviewout_nc;

wire [1:0] clkdiv_adc4;
wire [2:0] adc4_muxsel;
wire       adc4_en;
wire       adc4_start;
wire       adc4_chopen;
wire       adc4_dfx_en;
wire [9:0] adc4_out;
wire       adc4_done;
wire       adc4_anaviewout_nc;

wire        pvtmona_en;
wire        pvtmona_dfx_en;
wire [ 2:0] pvtmona_digview_sel;
wire [ 2:0] pvtmona_ref_clkdiv;
wire [ 2:0] pvtmona_osc_clkdiv;
wire [ 2:0] pvtmona_osc_sel;
wire        pvtmona_count_start;
wire [ 9:0] pvtmona_osc_clk_count;
wire        pvtmona_count_done;
wire        pvtmona_digviewout_nc;
wire [ 3:0] o_pvtmona_spare_nc;

wire        pvtmonb_en;
wire        pvtmonb_dfx_en;
wire [ 2:0] pvtmonb_digview_sel;
wire [ 2:0] pvtmonb_ref_clkdiv;
wire [ 2:0] pvtmonb_osc_clkdiv;
wire [ 2:0] pvtmonb_osc_sel;
wire        pvtmonb_count_start;
wire [ 9:0] pvtmonb_osc_clk_count;
wire        pvtmonb_count_done;
wire        pvtmonb_digviewout_nc;
wire [ 3:0] o_pvtmonb_spare_nc;

wire        pvtmonc_en;
wire        pvtmonc_dfx_en;
wire [ 2:0] pvtmonc_digview_sel;
wire [ 2:0] pvtmonc_ref_clkdiv;
wire [ 2:0] pvtmonc_osc_clkdiv;
wire [ 2:0] pvtmonc_osc_sel;
wire        pvtmonc_count_start;
wire [ 9:0] pvtmonc_osc_clk_count;
wire        pvtmonc_count_done;
wire        pvtmonc_digviewout_nc;
wire [ 3:0] o_pvtmonc_spare_nc;

wire        pvtmond_en;
wire        pvtmond_dfx_en;
wire [ 2:0] pvtmond_digview_sel;
wire [ 2:0] pvtmond_ref_clkdiv;
wire [ 2:0] pvtmond_osc_clkdiv;
wire [ 2:0] pvtmond_osc_sel;
wire        pvtmond_count_start;
wire [ 9:0] pvtmond_osc_clk_count;
wire        pvtmond_count_done;
wire        pvtmond_digviewout_nc;
wire [ 3:0] o_pvtmond_spare_nc;

wire        pvtmone_en;
wire        pvtmone_dfx_en;
wire [ 2:0] pvtmone_digview_sel;
wire [ 2:0] pvtmone_ref_clkdiv;
wire [ 2:0] pvtmone_osc_clkdiv;
wire [ 2:0] pvtmone_osc_sel;
wire        pvtmone_count_start;
wire [ 9:0] pvtmone_osc_clk_count;
wire        pvtmone_count_done;
wire        pvtmone_digviewout_nc;
wire [ 3:0] o_pvtmone_spare_nc;

wire [2:0] auxch_rxbuf;

wire        avmm_rdatavld_top;
wire [31:0] avmm_rdata_top;
wire        avmm_waitreq_top;
wire [NBR_CHNLS*4-1:0] txrx_anaviewout;
wire [NBR_CHNLS-1:0]   tx_dll_anaviewout;
wire [NBR_CHNLS-1:0]   rx_dll_anaviewout;
wire [NBR_CHNLS-1:0]   dcs_anaviewout;
wire [(NBR_CHNLS*2)-1:0]   digviewout;
wire [6:0] adc_anain_0;
wire [6:0] adc_anain_1;
wire [6:0] adc_anain_2;
wire [6:0] adc_anain_3;
wire [6:0] adc_anain_4;
assign o_jtag_tdo = o_jtag_tdo_ch[23];


// <<< FIX: Re-declared individual channel wires as arrays for use in generate blocks
wire [NBR_CHNLS-1:0] o_cfg_avmm_rdatavld_ch;
wire [NBR_CHNLS-1:0] o_cfg_avmm_waitreq_ch;
wire [NBR_CHNLS-1:0] o_jtag_tdo_ch; 
wire [32*NBR_CHNLS-1:0] o_rdata_ch_packed; // Packed array for glue logic

// <<< FIX: Create a local packed array for the iopad ports to use in the generate loop
wire [NBR_BUMPS-1:0] iopad_aib_ports [0:NBR_CHNLS-1];
assign iopad_aib_ports[0] = iopad_ch0_aib;
assign iopad_aib_ports[1] = iopad_ch1_aib;
assign iopad_aib_ports[2] = iopad_ch2_aib;
assign iopad_aib_ports[3] = iopad_ch3_aib;
assign iopad_aib_ports[4] = iopad_ch4_aib;
assign iopad_aib_ports[5] = iopad_ch5_aib;
assign iopad_aib_ports[6] = iopad_ch6_aib;
assign iopad_aib_ports[7] = iopad_ch7_aib;
assign iopad_aib_ports[8] = iopad_ch8_aib;
assign iopad_aib_ports[9] = iopad_ch9_aib;
assign iopad_aib_ports[10] = iopad_ch10_aib;
assign iopad_aib_ports[11] = iopad_ch11_aib;
assign iopad_aib_ports[12] = iopad_ch12_aib;
assign iopad_aib_ports[13] = iopad_ch13_aib;
assign iopad_aib_ports[14] = iopad_ch14_aib;
assign iopad_aib_ports[15] = iopad_ch15_aib;
assign iopad_aib_ports[16] = iopad_ch16_aib;
assign iopad_aib_ports[17] = iopad_ch17_aib;
assign iopad_aib_ports[18] = iopad_ch18_aib;
assign iopad_aib_ports[19] = iopad_ch19_aib;
assign iopad_aib_ports[20] = iopad_ch20_aib;
assign iopad_aib_ports[21] = iopad_ch21_aib;
assign iopad_aib_ports[22] = iopad_ch22_aib;
assign iopad_aib_ports[23] = iopad_ch23_aib;
// Assign back for inout behavior
assign iopad_ch0_aib = iopad_aib_ports[0];
assign iopad_ch1_aib = iopad_aib_ports[1];
assign iopad_ch2_aib = iopad_aib_ports[2];
assign iopad_ch3_aib = iopad_aib_ports[3];
assign iopad_ch4_aib = iopad_aib_ports[4];
assign iopad_ch5_aib = iopad_aib_ports[5];
assign iopad_ch6_aib = iopad_aib_ports[6];
assign iopad_ch7_aib = iopad_aib_ports[7];
assign iopad_ch8_aib = iopad_aib_ports[8];
assign iopad_ch9_aib = iopad_aib_ports[9];
assign iopad_ch10_aib = iopad_aib_ports[10];
assign iopad_ch11_aib = iopad_aib_ports[11];
assign iopad_ch12_aib = iopad_aib_ports[12];
assign iopad_ch13_aib = iopad_aib_ports[13];
assign iopad_ch14_aib = iopad_aib_ports[14];
assign iopad_ch15_aib = iopad_aib_ports[15];
assign iopad_ch16_aib = iopad_aib_ports[16];
assign iopad_ch17_aib = iopad_aib_ports[17];
assign iopad_ch18_aib = iopad_aib_ports[18];
assign iopad_ch19_aib = iopad_aib_ports[19];
assign iopad_ch20_aib = iopad_aib_ports[20];
assign iopad_ch21_aib = iopad_aib_ports[21];
assign iopad_ch22_aib = iopad_aib_ports[22];
assign iopad_ch23_aib = iopad_aib_ports[23];


// The final top-level TDO is the end of the JTAG chain
assign o_jtag_tdo = o_jtag_tdo_ch[NBR_CHNLS-1];

// <<< FIX: Instantiate the corrected, scalable glue logic
aib_avmm_glue_logic #(
    .NBR_CHNLS(NBR_CHNLS)
) aib_avmm_glue_logic_inst (
    .i_waitreq_ch (o_cfg_avmm_waitreq_ch),
    .i_rdatavld_ch(o_cfg_avmm_rdatavld_ch),
    .i_rdata_ch   (o_rdata_ch_packed),
    .o_waitreq    (o_cfg_avmm_waitreq),
    .o_rdatavld   (o_cfg_avmm_rdatavld),
    .o_rdata      (o_cfg_avmm_rdata)
);


// <<< FIX: Removed all hard-coded channel instantiations and replaced with generate blocks
genvar i;

// <<< FIX: Generate loop for the active channels
generate
  for (i = 0; i < ACTIVE_CHNLS; i = i + 1) begin : gen_active_channels
    aib_channel_n #(
      .NBR_LANES(NBR_LANES),
      .SCAN_STR_PER_CH(SCAN_STR_PER_CH),
      .BERT_BUF_MODE_EN(BERT_BUF_MODE_EN)
    ) aib_channel_inst (
      .vddc2(vddc2),
      .vddc1(vddc1),
      .vddtx(vddtx),
      .vss(vss),

      .iopad_aib(iopad_aib_ports[i]),

      .data_in_f(data_in_f[320 * (i + 1) - 1:320 * i]),
      .data_out_f(data_out_f[320 * (i + 1) - 1:320 * i]),
      .data_in(data_in[80 * (i + 1) - 1:80 * i]),
      .data_out(data_out[80 * (i + 1) - 1:80 * i]),

      .m_ns_fwd_clk(m_ns_fwd_clk[i]),
      .m_fs_rcv_clk(m_fs_rcv_clk[i]),
      .m_fs_fwd_clk(m_fs_fwd_clk[i]),
      .m_ns_rcv_clk(m_ns_rcv_clk[i]),
      .m_wr_clk(m_wr_clk[i]),
      .m_rd_clk(m_rd_clk[i]),
      .ns_fwd_clk(ns_fwd_clk[i]),
      .ns_fwd_clk_div(ns_fwd_clk_div[i]),
      .fs_fwd_clk(fs_fwd_clk[i]),
      .fs_fwd_clk_div(fs_fwd_clk_div[i]),

      .i_conf_done(i_conf_done),
      .ms_rx_dcc_dll_lock_req(ms_rx_dcc_dll_lock_req[i]),
      .ms_tx_dcc_dll_lock_req(ms_tx_dcc_dll_lock_req[i]),
      .sl_tx_dcc_dll_lock_req(sl_tx_dcc_dll_lock_req[i]),
      .sl_rx_dcc_dll_lock_req(sl_rx_dcc_dll_lock_req[i]),
      .ms_tx_transfer_en(ms_tx_transfer_en[i]),
      .ms_rx_transfer_en(ms_rx_transfer_en[i]),
      .sl_tx_transfer_en(sl_tx_transfer_en[i]),
      .sl_rx_transfer_en(sl_rx_transfer_en[i]),
      .m_rx_align_done(m_rx_align_done[i]),

      .sr_ms_tomac(sr_ms_tomac[MS_SSR_LEN * (i + 1) - 1:MS_SSR_LEN * i]),
      .sr_sl_tomac(sr_sl_tomac[SL_SSR_LEN * (i + 1) - 1:SL_SSR_LEN * i]),

      .dual_mode_select(dual_mode_select),
      .m_gen2_mode(m_gen2_mode),

      .sl_external_cntl_26_0(sl_external_cntl_26_0[27 * (i + 1) - 1:27 * i]),
      .sl_external_cntl_30_28(sl_external_cntl_30_28[3 * (i + 1) - 1:3 * i]),
      .sl_external_cntl_57_32(sl_external_cntl_57_32[26 * (i + 1) - 1:26 * i]),

      .ms_external_cntl_4_0(ms_external_cntl_4_0[5 * (i + 1) - 1:5 * i]),
      .ms_external_cntl_65_8(ms_external_cntl_65_8[58 * (i + 1) - 1:58 * i]),

      .ns_adapter_rstn(ns_adapter_rstn[i]),
      .ns_mac_rdy(ns_mac_rdy[i]),
      .fs_mac_rdy(fs_mac_rdy[i]),
      .por(o_m_power_on_reset),
      .i_osc_clk(i_osc_clk),
      .txrx_anaviewout(txrx_anaviewout[4 * (i + 1) - 1:4 * i]),
      .tx_dll_anaviewout(tx_dll_anaviewout[i]),
      .rx_dll_anaviewout(rx_dll_anaviewout[i]),
      .dcs_anaviewout(dcs_anaviewout[i]),
      .digviewout(digviewout[2 * (i + 1) - 1:2 * i]),

      //JTAG interface
      .jtag_clkdr_in(i_jtag_clkdr),
      .i_jtag_clksel(i_jtag_clksel),
      .scan_out(o_jtag_tdo_ch[i]),
      .jtag_intest(i_jtag_intest),
      .jtag_mode_in(i_jtag_mode),
      .jtag_rstb(i_jtag_rstb),
      .jtag_rstb_en(i_jtag_rstb_en),
      .jtag_weakpdn(i_jtag_weakpdn),
      .jtag_weakpu(i_jtag_weakpu),
      .jtag_tx_scanen_in(i_jtag_tx_scanen),
      .scan_in((i == 0) ? i_jtag_tdi : o_jtag_tdo_ch[i - 1]),
      .i_scan_clk(i_scan_clk),
      .i_scan_clk_500m(i_scan_clk_500m),
      .i_scan_clk_1000m(i_scan_clk_1000m),
      .i_scan_en(i_scan_en),
      .i_scan_mode(i_scan_mode),
      .i_scan_din(i_scan_din[(SCAN_STR_PER_CH * (i + 1)) - 1:(SCAN_STR_PER_CH * i)]),
      .i_scan_dout(i_scan_dout[(SCAN_STR_PER_CH * (i + 1)) - 1:(SCAN_STR_PER_CH * i)]),

      .i_channel_id(i),
      .i_cfg_avmm_clk(i_cfg_avmm_clk),
      .i_cfg_avmm_rst_n(i_cfg_avmm_rst_n),
      .i_cfg_avmm_addr(i_cfg_avmm_addr),
      .i_cfg_avmm_byte_en(i_cfg_avmm_byte_en),
      .i_cfg_avmm_read(i_cfg_avmm_read),
      .i_cfg_avmm_write(i_cfg_avmm_write),
      .i_cfg_avmm_wdata(i_cfg_avmm_wdata),

      .o_cfg_avmm_rdatavld(o_cfg_avmm_rdatavld_ch[i]),
      .o_cfg_avmm_rdata(o_rdata_ch_packed[(i + 1) * 32 - 1 -: 32]),
      .o_cfg_avmm_waitreq(o_cfg_avmm_waitreq_ch[i])
    );
  end
endgenerate

// <<< FIX: Generate loop to tie off all outputs of inactive channels
generate
    for (i = ACTIVE_CHNLS; i < NBR_CHNLS; i = i + 1) begin : gen_inactive_channels
        // Tie off AVMM signals
        assign o_cfg_avmm_rdatavld_ch[i] = 1'b0;
        assign o_cfg_avmm_waitreq_ch[i]  = 1'b1;
        assign o_rdata_ch_packed[(i+1)*32-1 -: 32] = 32'b0;

        // Tie off JTAG chain
        assign o_jtag_tdo_ch[i] = (i == 0) ? 1'b0 : o_jtag_tdo_ch[i-1];

        // Tie off main data and control outputs
        assign data_out_f[320*(i+1)-1:320*i] = '0;
        assign data_out[80*(i+1)-1:80*i] = '0;
        assign m_fs_rcv_clk[i] = 1'b0;
        assign m_fs_fwd_clk[i] = 1'b0;
        assign ns_fwd_clk[i] = 1'b0;
        assign ns_fwd_clk_div[i] = 1'b0;
        assign fs_fwd_clk[i] = 1'b0;
        assign fs_fwd_clk_div[i] = 1'b0;
        assign fs_mac_rdy[i] = 1'b0;
        assign ms_tx_transfer_en[i] = 1'b0;
        assign ms_rx_transfer_en[i] = 1'b0;
        assign sl_tx_transfer_en[i] = 1'b0;
        assign sl_rx_transfer_en[i] = 1'b0;
        assign m_rx_align_done[i] = 1'b0;
        assign sr_ms_tomac[MS_SSR_LEN*(i+1)-1:MS_SSR_LEN*i] = '0;
        assign sr_sl_tomac[SL_SSR_LEN*(i+1)-1:SL_SSR_LEN*i] = '0;
        assign i_scan_dout[(SCAN_STR_PER_CH*(i+1))-1:(SCAN_STR_PER_CH*i)] = '0;
    end
endgenerate

// Auxiliary channel
aibio_auxch_cbb aibio_auxch_cbb
(
//----supply pins----//
.vddc  (vddc2),
.vss   (vss),
//-----input pins-------------//
.dual_mode_sel        (dual_mode_select),
.i_m_power_on_reset   (i_m_power_on_reset),
.m_por_ovrd           (m_por_ovrd),
.m_device_detect_ovrd (m_device_detect_ovrd),
.rxbuf_cfg            (auxch_rxbuf[2:0]),             
.powergood            (vddc2),      
.gen1mode_en          (~m_gen2_mode),
//-----inout pins-----------------//
.xx_power_on_reset    (iopad_power_on_reset),
.xx_device_detect     (iopad_device_detect),
//-------output pins--------------//
.o_m_power_on_reset   (o_m_power_on_reset),
.m_device_detect      (m_device_detect),
//--------spare pins---------------//
.i_aux_spare          (4'h0),                // Spares pins
.o_aux_spare          (o_aux_spare_nc[3:0])  // Spares pins
);
assign adc_anain_0[6:0] = {dcs_anaviewout[0],  rx_dll_anaviewout[0],  tx_dll_anaviewout[0],  txrx_anaviewout[4*1-1:4*0]};
assign adc_anain_1[6:0] = {dcs_anaviewout[5],  rx_dll_anaviewout[5],  tx_dll_anaviewout[5],  txrx_anaviewout[4*6-1:4*5]};
assign adc_anain_2[6:0] = {dcs_anaviewout[11], rx_dll_anaviewout[11], tx_dll_anaviewout[11], txrx_anaviewout[4*12-1:4*11]};
assign adc_anain_3[6:0] = {dcs_anaviewout[17], rx_dll_anaviewout[17], tx_dll_anaviewout[17], txrx_anaviewout[4*18-1:4*17]};
assign adc_anain_4[6:0] = {dcs_anaviewout[23], rx_dll_anaviewout[23], tx_dll_anaviewout[23], txrx_anaviewout[4*24-1:4*23]};
//------------------------------------------------------------------------------
//                          ADC0 for channel 0-3
//------------------------------------------------------------------------------
aibio_adc_cbb i_adc_cbb_0(  
//------Supply pins------//
.vddc(vddc2),  
.vssx(vss), 
//------Input pins------//
.adcclk(i_cfg_avmm_clk), 
.clkdiv(clkdiv_adc0[1:0]),            
.adc_anain({1'b0,adc_anain_0}),//ch0             
.adc_anamux_sel(adc0_muxsel[2:0]),    
.adc_en(adc0_en),            
.reset(i_cfg_avmm_rst_n),             
.adc_start(adc0_start),         
.chopen(adc0_chopen),            
.adc_dfx_en(adc0_dfx_en),        
.adc_anaviewsel(3'b000),    
//------Output pins------//
.adcout(adc0_out[9:0]),  
.adcdone(adc0_done),
.adc_anaviewout(adc0_anaviewout_nc),
//------Spare pins------//
.i_adc_spare(4'b0000),  // Spare pins
.o_adc_spare()   
);

//------------------------------------------------------------------------------
//                          ADC1 for channel 5-7
//------------------------------------------------------------------------------
aibio_adc_cbb i_adc_cbb_1(  
//------Supply pins------//
.vddc(vddc2),  
.vssx(vss), 
//------Input pins------//
.adcclk(i_cfg_avmm_clk), 
.clkdiv(clkdiv_adc1[1:0]),            
.adc_anain({1'b0,adc_anain_1}),//ch5, we should keep LSB as constant value             
.adc_anamux_sel(adc1_muxsel[2:0]),    
.adc_en(adc1_en),            
.reset(i_cfg_avmm_rst_n),             
.adc_start(adc1_start),         
.chopen(adc1_chopen),            
.adc_dfx_en(adc1_dfx_en),        
.adc_anaviewsel(3'b000),    
//------Output pins------//
.adcout(adc1_out[9:0]),  
.adcdone(adc1_done),
.adc_anaviewout(adc1_anaviewout_nc),
//------Spare pins------//
.i_adc_spare(4'b0000),  // Spare pins
.o_adc_spare()   
);

//------------------------------------------------------------------------------
//                          ADC2 for channel 8-11
//------------------------------------------------------------------------------
aibio_adc_cbb i_adc_cbb_2(  
//------Supply pins------//
.vddc(vddc2),  
.vssx(vss), 
//------Input pins------//
.adcclk(i_cfg_avmm_clk), 
.clkdiv(clkdiv_adc2[1:0]),            
.adc_anain({1'b0,adc_anain_2}),//ch11             
.adc_anamux_sel(adc2_muxsel[2:0]),    
.adc_en(adc2_en),            
.reset(i_cfg_avmm_rst_n),             
.adc_start(adc2_start),         
.chopen(adc2_chopen),            
.adc_dfx_en(adc2_dfx_en),        
.adc_anaviewsel(3'b000),    
//------Output pins------//
.adcout(adc2_out[9:0]),  
.adcdone(adc2_done),
.adc_anaviewout(adc2_anaviewout_nc),
//------Spare pins------//
.i_adc_spare(4'b0000),  // Spare pins
.o_adc_spare()   
);

//------------------------------------------------------------------------------
//                          ADC3 for channel 12-15
//------------------------------------------------------------------------------
aibio_adc_cbb i_adc_cbb_3(  
//------Supply pins------//
.vddc(vddc2),  
.vssx(vss), 
//------Input pins------//
.adcclk(i_cfg_avmm_clk), 
.clkdiv(clkdiv_adc3[1:0]),            
.adc_anain({1'b0,adc_anain_3}),//ch17             
.adc_anamux_sel(adc3_muxsel[2:0]),    
.adc_en(adc3_en),            
.reset(i_cfg_avmm_rst_n),             
.adc_start(adc3_start),         
.chopen(adc3_chopen),            
.adc_dfx_en(adc3_dfx_en),        
.adc_anaviewsel(3'b000),    
//------Output pins------//
.adcout(adc3_out[9:0]),  
.adcdone(adc3_done),
.adc_anaviewout(adc3_anaviewout_nc),
//------Spare pins------//
.i_adc_spare(4'b0000),  // Spare pins
.o_adc_spare()   
);
//------------------------------------------------------------------------------
//                          ADC4 for channel 16-19
//------------------------------------------------------------------------------
aibio_adc_cbb i_adc_cbb_4(  
//------Supply pins------//
.vddc(vddc2),  
.vssx(vss), 
//------Input pins------//
.adcclk(i_cfg_avmm_clk), 
.clkdiv(clkdiv_adc4[1:0]),            
.adc_anain({1'b0,adc_anain_4}),//ch23             
.adc_anamux_sel(adc4_muxsel[2:0]),    
.adc_en(adc4_en),            
.reset(i_cfg_avmm_rst_n),             
.adc_start(adc4_start),         
.chopen(adc4_chopen),            
.adc_dfx_en(adc4_dfx_en),        
.adc_anaviewsel(3'b000),    
//------Output pins------//
.adcout(adc4_out[9:0]),  
.adcdone(adc4_done),
.adc_anaviewout(adc4_anaviewout_nc),
//------Spare pins------//
.i_adc_spare(4'b0000),  // Spare pins
.o_adc_spare()   
);    



aib_avmm_top aib_avmm_top_i(
// Inputs
.i_cfg_avmm_clk     (i_cfg_avmm_clk),
.i_cfg_avmm_rst_n   (i_cfg_avmm_rst_n),
.i_cfg_avmm_addr    (i_cfg_avmm_addr[16:0]), 
.i_cfg_avmm_byte_en (i_cfg_avmm_byte_en[3:0]), 
.i_cfg_avmm_read    (i_cfg_avmm_read), 
.i_cfg_avmm_write   (i_cfg_avmm_write), 
.i_cfg_avmm_wdata   (i_cfg_avmm_wdata[31:0]),
// Scan mode input 
.i_scan_mode (i_scan_mode),
// Outputs
// ADC0 signals
.clkdiv_adc0    (clkdiv_adc0[1:0]),
.adc0_muxsel    (adc0_muxsel[2:0]),
.adc0_en        (adc0_en),
.adc0_start     (adc0_start),
.adc0_chopen    (adc0_chopen),
.adc0_dfx_en    (adc0_dfx_en),
.adc0_out       (adc0_out[9:0]),
.adc0_done      (adc0_done),
// ADC1 signals
.clkdiv_adc1    (clkdiv_adc1[1:0]),
.adc1_muxsel    (adc1_muxsel[2:0]),
.adc1_en        (adc1_en),
.adc1_start     (adc1_start),
.adc1_chopen    (adc1_chopen),
.adc1_dfx_en    (adc1_dfx_en),
.adc1_out       (adc1_out[9:0]),
.adc1_done      (adc1_done),
// ADC2 signals
.clkdiv_adc2    (clkdiv_adc2[1:0]),
.adc2_muxsel    (adc2_muxsel[2:0]),
.adc2_en        (adc2_en),
.adc2_start     (adc2_start),
.adc2_chopen    (adc2_chopen),
.adc2_dfx_en    (adc2_dfx_en),
.adc2_out       (adc2_out[9:0]),
.adc2_done      (adc2_done),
// ADC3 signals
.clkdiv_adc3    (clkdiv_adc3[1:0]),
.adc3_muxsel    (adc3_muxsel[2:0]),
.adc3_en        (adc3_en),
.adc3_start     (adc3_start),
.adc3_chopen    (adc3_chopen),
.adc3_dfx_en    (adc3_dfx_en),
.adc3_out       (adc3_out[9:0]),
.adc3_done      (adc3_done),
// ADC4 signals
.clkdiv_adc4    (clkdiv_adc4[1:0]),
.adc4_muxsel    (adc4_muxsel[2:0]),
.adc4_en        (adc4_en),
.adc4_start     (adc4_start),
.adc4_chopen    (adc4_chopen),
.adc4_dfx_en    (adc4_dfx_en),
.adc4_out       (adc4_out[9:0]),
.adc4_done      (adc4_done),
// AIBIO PVTMONA CBB
.pvtmona_en                (pvtmona_en),
.pvtmona_dfx_en            (pvtmona_dfx_en),
.pvtmona_digview_sel       (pvtmona_digview_sel[2:0]),
.pvtmona_ref_clkdiv        (pvtmona_ref_clkdiv[2:0]),
.pvtmona_osc_clkdiv        (pvtmona_osc_clkdiv[2:0]),
.pvtmona_osc_sel           (pvtmona_osc_sel[2:0]),
.pvtmona_count_start       (pvtmona_count_start),
.pvtmona_osc_clk_count     (pvtmona_osc_clk_count[9:0]),
.pvtmona_count_done        (pvtmona_count_done),
// AIBIO PVTMONB CBB
.pvtmonb_en                (pvtmonb_en),
.pvtmonb_dfx_en            (pvtmonb_dfx_en),
.pvtmonb_digview_sel       (pvtmonb_digview_sel[2:0]),
.pvtmonb_ref_clkdiv        (pvtmonb_ref_clkdiv[2:0]),
.pvtmonb_osc_clkdiv        (pvtmonb_osc_clkdiv[2:0]),
.pvtmonb_osc_sel           (pvtmonb_osc_sel[2:0]),
.pvtmonb_count_start       (pvtmonb_count_start),
.pvtmonb_osc_clk_count     (pvtmonb_osc_clk_count[9:0]),
.pvtmonb_count_done        (pvtmonb_count_done),
// AIBIO PVTMONC CBB
.pvtmonc_en                (pvtmonc_en),
.pvtmonc_dfx_en            (pvtmonc_dfx_en),
.pvtmonc_digview_sel       (pvtmonc_digview_sel[2:0]),
.pvtmonc_ref_clkdiv        (pvtmonc_ref_clkdiv[2:0]),
.pvtmonc_osc_clkdiv        (pvtmonc_osc_clkdiv[2:0]),
.pvtmonc_osc_sel           (pvtmonc_osc_sel[2:0]),
.pvtmonc_count_start       (pvtmonc_count_start),
.pvtmonc_osc_clk_count     (pvtmonc_osc_clk_count[9:0]),
.pvtmonc_count_done        (pvtmonc_count_done),
// AIBIO PVTMONB CBB
.pvtmond_en                (pvtmond_en),
.pvtmond_dfx_en            (pvtmond_dfx_en),
.pvtmond_digview_sel       (pvtmond_digview_sel[2:0]),
.pvtmond_ref_clkdiv        (pvtmond_ref_clkdiv[2:0]),
.pvtmond_osc_clkdiv        (pvtmond_osc_clkdiv[2:0]),
.pvtmond_osc_sel           (pvtmond_osc_sel[2:0]),
.pvtmond_count_start       (pvtmond_count_start),
.pvtmond_osc_clk_count     (pvtmond_osc_clk_count[9:0]),
.pvtmond_count_done        (pvtmond_count_done),
// AIBIO PVTMONB CBB
.pvtmone_en                (pvtmone_en),
.pvtmone_dfx_en            (pvtmone_dfx_en),
.pvtmone_digview_sel       (pvtmone_digview_sel[2:0]),
.pvtmone_ref_clkdiv        (pvtmone_ref_clkdiv[2:0]),
.pvtmone_osc_clkdiv        (pvtmone_osc_clkdiv[2:0]),
.pvtmone_osc_sel           (pvtmone_osc_sel[2:0]),
.pvtmone_count_start       (pvtmone_count_start),
.pvtmone_osc_clk_count     (pvtmone_osc_clk_count[9:0]),
.pvtmone_count_done        (pvtmone_count_done),
// Auxiliary channel register output 
.auxch_rxbuf    (auxch_rxbuf[2:0]),
// Avalon outputs
.avmm_rdatavld_top (avmm_rdatavld_top),
.avmm_rdata_top    (avmm_rdata_top[31:0]), 
.avmm_waitreq_top  (avmm_waitreq_top)
);

aibio_pvtmon_cbb i_pvtmon_cbb_0(
.vddc(vddc2),
.vss(vss),
.clk_sys(i_cfg_avmm_clk),
.count_start(pvtmona_count_start),
.en(pvtmona_en),
.osc_sel(pvtmona_osc_sel[2:0]), //3-bits
.oscclkdiv(pvtmona_osc_clkdiv[2:0]), //3-bits
.refclkdiv(pvtmona_ref_clkdiv[2:0]), //3-bits
.pvtmon_dfx_en(pvtmona_dfx_en),
.pvtmon_digview_sel(pvtmona_digview_sel[2:0]), //3-bits
.count_done(pvtmona_count_done),
.codeout(pvtmona_osc_clk_count[9:0]), //10bits
.pvtmon_digviewout(pvtmona_digviewout_nc),
.i_pvtmon_spare(4'b0000), //4-bits
.o_pvtmon_spare(o_pvtmona_spare_nc[3:0]) //4-bits
);

aibio_pvtmon_cbb i_pvtmon_cbb_1(
.vddc(vddc2),
.vss(vss),
.clk_sys(i_cfg_avmm_clk),
.count_start(pvtmonb_count_start),
.en(pvtmonb_en),
.osc_sel(pvtmonb_osc_sel[2:0]), //3-bits
.oscclkdiv(pvtmonb_osc_clkdiv[2:0]), //3-bits
.refclkdiv(pvtmonb_ref_clkdiv[2:0]), //3-bits
.pvtmon_dfx_en(pvtmonb_dfx_en),
.pvtmon_digview_sel(pvtmonb_digview_sel[2:0]), //3-bits
.count_done(pvtmonb_count_done),
.codeout(pvtmonb_osc_clk_count[9:0]), //10bits
.pvtmon_digviewout(pvtmonb_digviewout_nc),
.i_pvtmon_spare(4'b0000), //4-bits
.o_pvtmon_spare(o_pvtmonb_spare_nc[3:0]) //4-bits
);

aibio_pvtmon_cbb i_pvtmon_cbb_2(
.vddc(vddc2),
.vss(vss),
.clk_sys(i_cfg_avmm_clk),
.count_start(pvtmonc_count_start),
.en(pvtmonc_en),
.osc_sel(pvtmonc_osc_sel[2:0]), //3-bits
.oscclkdiv(pvtmonc_osc_clkdiv[2:0]), //3-bits
.refclkdiv(pvtmonc_ref_clkdiv[2:0]), //3-bits
.pvtmon_dfx_en(pvtmonc_dfx_en),
.pvtmon_digview_sel(pvtmonc_digview_sel[2:0]), //3-bits
.count_done(pvtmonc_count_done),
.codeout(pvtmonc_osc_clk_count[9:0]), //10bits
.pvtmon_digviewout(pvtmonc_digviewout_nc),
.i_pvtmon_spare(4'b0000), //4-bits should be grounded
.o_pvtmon_spare(o_pvtmonc_spare_nc[3:0]) //4-bits
);

aibio_pvtmon_cbb i_pvtmon_cbb_3(
.vddc(vddc2),
.vss(vss),
.clk_sys(i_cfg_avmm_clk),
.count_start(pvtmond_count_start),
.en(pvtmond_en),
.osc_sel(pvtmond_osc_sel[2:0]), //3-bits
.oscclkdiv(pvtmond_osc_clkdiv[2:0]), //3-bits
.refclkdiv(pvtmond_ref_clkdiv[2:0]), //3-bits
.pvtmon_dfx_en(pvtmond_dfx_en),
.pvtmon_digview_sel(pvtmond_digview_sel[2:0]), //3-bits
.count_done(pvtmond_count_done),
.codeout(pvtmond_osc_clk_count[9:0]), //10bits
.pvtmon_digviewout(pvtmond_digviewout_nc),
.i_pvtmon_spare(4'b0000), //4-bits
.o_pvtmon_spare(o_pvtmond_spare_nc[3:0]) //4-bits
);

aibio_pvtmon_cbb i_pvtmon_cbb_4(
.vddc(vddc2),
.vss(vss),
.clk_sys(i_cfg_avmm_clk),
.count_start(pvtmone_count_start),
.en(pvtmone_en),
.osc_sel(pvtmone_osc_sel[2:0]), //3-bits
.oscclkdiv(pvtmone_osc_clkdiv[2:0]), //3-bits
.refclkdiv(pvtmone_ref_clkdiv[2:0]), //3-bits
.pvtmon_dfx_en(pvtmone_dfx_en),
.pvtmon_digview_sel(pvtmone_digview_sel[2:0]), //3-bits
.count_done(pvtmone_count_done),
.codeout(pvtmone_osc_clk_count[9:0]), //10bits
.pvtmon_digviewout(pvtmone_digviewout_nc),
.i_pvtmon_spare(4'b0000), //4-bits
.o_pvtmon_spare(o_pvtmone_spare_nc[3:0]) //4-bits
);


endmodule // aib_phy_top
