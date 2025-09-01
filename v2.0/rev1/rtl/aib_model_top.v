module aib_model_top #(
     parameter MAX_SCAN_LEN = 200,
     parameter DATAWIDTH = 40,
     parameter TOTAL_CHNL_NUM = 24,
     parameter ACTIVE_CHNL_NUM = 24 // Number of active channels
 ) (
     // NOTE: The interface has been changed to use a single, consolidated
     // 1D vector for the iopad signals, which is then decoded internally.
     inout [TOTAL_CHNL_NUM*102-1:0]          iopad_aib,

     inout                                   iopad_device_detect,
     inout                                   iopad_power_on_reset,

     input  [DATAWIDTH*8*TOTAL_CHNL_NUM-1:0]   data_in_f,
     output [DATAWIDTH*8*TOTAL_CHNL_NUM-1:0]   data_out_f,

     input  [DATAWIDTH*2*TOTAL_CHNL_NUM-1:0]   data_in,
     output [DATAWIDTH*2*TOTAL_CHNL_NUM-1:0]   data_out,

     input  [TOTAL_CHNL_NUM-1:0]             m_ns_fwd_clk,
     input  [TOTAL_CHNL_NUM-1:0]             m_ns_rcv_clk,
     output [TOTAL_CHNL_NUM-1:0]             m_fs_rcv_clk,
     output [TOTAL_CHNL_NUM-1:0]             m_fs_fwd_clk,
     input  [TOTAL_CHNL_NUM-1:0]             m_wr_clk,
     input  [TOTAL_CHNL_NUM-1:0]             m_rd_clk,
     output [TOTAL_CHNL_NUM-1:0]             tclk_phy,

     input  [TOTAL_CHNL_NUM-1:0]             ns_adapter_rstn,
     input  [TOTAL_CHNL_NUM-1:0]             ns_mac_rdy,
     output [TOTAL_CHNL_NUM-1:0]             fs_mac_rdy,
     input                                   i_conf_done,
     input                                   i_osc_clk,

     input  [TOTAL_CHNL_NUM-1:0]             ms_rx_dcc_dll_lock_req,
     input  [TOTAL_CHNL_NUM-1:0]             ms_tx_dcc_dll_lock_req,
     input  [TOTAL_CHNL_NUM-1:0]             sl_tx_dcc_dll_lock_req,
     input  [TOTAL_CHNL_NUM-1:0]             sl_rx_dcc_dll_lock_req,
     output [TOTAL_CHNL_NUM-1:0]             ms_tx_transfer_en,
     output [TOTAL_CHNL_NUM-1:0]             ms_rx_transfer_en,
     output [TOTAL_CHNL_NUM-1:0]             sl_tx_transfer_en,
     output [TOTAL_CHNL_NUM-1:0]             sl_rx_transfer_en,
     output [TOTAL_CHNL_NUM-1:0]             m_rx_align_done,
     output [81*TOTAL_CHNL_NUM-1:0]          sr_ms_tomac,
     output [73*TOTAL_CHNL_NUM-1:0]          sr_sl_tomac,
     input                                   dual_mode_select,
     input                                   m_gen2_mode,

     //Aux channel
     input                                   m_por_ovrd,
     input                                   m_device_detect_ovrd,
     input                                   i_m_power_on_reset,
     output                                  m_device_detect,
     output                                  o_m_power_on_reset,
     //JTAG signals
     input                                   i_jtag_clkdr,
     input                                   i_jtag_clksel,
     input                                   i_jtag_intest,
     input                                   i_jtag_mode,
     input                                   i_jtag_rstb,
     input                                   i_jtag_rstb_en,
     input                                   i_jtag_tdi,
     input                                   i_jtag_tx_scanen,
     input                                   i_jtag_weakpdn,
     input                                   i_jtag_weakpu,

     input                                   i_scan_clk,
     input                                   i_scan_clk_500m,
     input                                   i_scan_clk_1000m,
     input                                   i_scan_en,
     input                                   i_scan_mode,
     input  [TOTAL_CHNL_NUM-1:0][MAX_SCAN_LEN-1:0] i_scan_din,
     output [TOTAL_CHNL_NUM-1:0][MAX_SCAN_LEN-1:0] i_scan_dout,

     //AVMM interface
     input                                   i_cfg_avmm_clk,
     input                                   i_cfg_avmm_rst_n,
     input [16:0]                            i_cfg_avmm_addr,
     input [3:0]                             i_cfg_avmm_byte_en,
     input                                   i_cfg_avmm_read,
     input                                   i_cfg_avmm_write,
     input [31:0]                            i_cfg_avmm_wdata,
     output                                  o_cfg_avmm_rdatavld,
     output reg [31:0]                       o_cfg_avmm_rdata,
     output                                  o_cfg_avmm_waitreq,
     output                                  o_jtag_tdo,

     //Redundancy control signals for IO buffers
     input [27*TOTAL_CHNL_NUM-1:0]           sl_external_cntl_26_0,
     input [3*TOTAL_CHNL_NUM-1:0]            sl_external_cntl_30_28,
     input [26*TOTAL_CHNL_NUM-1:0]           sl_external_cntl_57_32,

     input [5*TOTAL_CHNL_NUM-1:0]            ms_external_cntl_4_0,
     input [58*TOTAL_CHNL_NUM-1:0]           ms_external_cntl_65_8
 );

     // Internal wires for connecting to each channel instance
     wire por_ms, osc_clk;
     wire [TOTAL_CHNL_NUM-1:0] o_cfg_avmm_rdatavld_ch;
     wire [TOTAL_CHNL_NUM-1:0] o_cfg_avmm_waitreq_ch;
     wire [TOTAL_CHNL_NUM-1:0] o_jtag_tdo_ch;
     wire [31:0] o_rdata_ch[0:TOTAL_CHNL_NUM-1];

     wire  vccl_aib = 1'b1;
     wire vssl_aib = 1'b0;

     // Assign the final TDO from the last channel in the daisy chain
     assign o_jtag_tdo = o_jtag_tdo_ch[23];

     /////////////////////////////////////////////////////////////////////////////////
     //
     // AVMM output generation
     //
     /////////////////////////////////////////////////////////////////////////////////

     // Wait request is asserted if any channel asserts it (AND reduction)
     assign o_cfg_avmm_waitreq =  &o_cfg_avmm_waitreq_ch;
     // Read data valid is asserted if any channel asserts it (OR reduction)
     assign o_cfg_avmm_rdatavld = |o_cfg_avmm_rdatavld_ch;

     // Combine read data from all channels (bitwise OR)
     assign o_cfg_avmm_rdata    = o_rdata_ch[23]|o_rdata_ch[22]|o_rdata_ch[21]|o_rdata_ch[20]|o_rdata_ch[19]|o_rdata_ch[18]|
                              o_rdata_ch[17]|o_rdata_ch[16]|o_rdata_ch[15]|o_rdata_ch[14]|o_rdata_ch[13]|o_rdata_ch[12]|
                              o_rdata_ch[11]|o_rdata_ch[10]|o_rdata_ch[9 ]|o_rdata_ch[8] |o_rdata_ch[7] |o_rdata_ch[6] |
                              o_rdata_ch[5 ]|o_rdata_ch[4 ]|o_rdata_ch[3 ]|o_rdata_ch[2] |o_rdata_ch[1] |o_rdata_ch[0];
                          

     /////////////////////////////////////////////////////////////////////////////////
     //
     // Channel Instantiation Generate Block
     //
     /////////////////////////////////////////////////////////////////////////////////
     genvar i;
     generate
         for (i = 0; i < TOTAL_CHNL_NUM; i = i + 1) begin : aib_channel_gen
             // Define constants for bus slicing to improve readability
             localparam IOPAD_WIDTH = 102;
             localparam DATA_F_WIDTH = DATAWIDTH * 8;
             localparam DATA_WIDTH = DATAWIDTH * 2;
             localparam MS_SSR_WIDTH = 81;
             localparam SL_SSR_WIDTH = 73;

             if (i < ACTIVE_CHNL_NUM) begin : active_channel_inst
                 aib_channel #(
                     .DATAWIDTH(DATAWIDTH),
                     .MAX_SCAN_LEN(MAX_SCAN_LEN)
                 ) aib_channel_inst (
                     // Decode/slice the 1D iopad_aib vector for the current channel instance
                     .iopad_aib(iopad_aib[IOPAD_WIDTH*(i+1)-1 : IOPAD_WIDTH*i]),

                     // Slicing data buses based on the channel index 'i'
                     .data_in_f(data_in_f[DATA_F_WIDTH*(i+1)-1 : DATA_F_WIDTH*i]),
                     .data_out_f(data_out_f[DATA_F_WIDTH*(i+1)-1 : DATA_F_WIDTH*i]),
                     .data_in(data_in[DATA_WIDTH*(i+1)-1 : DATA_WIDTH*i]),
                     .data_out(data_out[DATA_WIDTH*(i+1)-1 : DATA_WIDTH*i]),

                     // Selecting single-bit signals based on the channel index 'i'
                     .m_ns_fwd_clk(m_ns_fwd_clk[i]),
                     .m_fs_rcv_clk(m_fs_rcv_clk[i]),
                     .m_fs_fwd_clk(m_fs_fwd_clk[i]),
                     .m_ns_rcv_clk(m_ns_rcv_clk[i]),
                     .m_wr_clk(m_wr_clk[i]),
                     .m_rd_clk(m_rd_clk[i]),
                     .tclk_phy(tclk_phy[i]),

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

                     .sr_ms_tomac(sr_ms_tomac[MS_SSR_WIDTH*(i+1)-1 : MS_SSR_WIDTH*i]),
                     .sr_sl_tomac(sr_sl_tomac[SL_SSR_WIDTH*(i+1)-1 : SL_SSR_WIDTH*i]),

                     .dual_mode_select(dual_mode_select),
                     .m_gen2_mode(m_gen2_mode),

                     .sl_external_cntl_26_0(sl_external_cntl_26_0[27*(i+1)-1 : 27*i]),
                     .sl_external_cntl_30_28(sl_external_cntl_30_28[3*(i+1)-1 : 3*i]),
                     .sl_external_cntl_57_32(sl_external_cntl_57_32[26*(i+1)-1 : 26*i]),

                     .ms_external_cntl_4_0(ms_external_cntl_4_0[5*(i+1)-1 : 5*i]),
                     .ms_external_cntl_65_8(ms_external_cntl_65_8[58*(i+1)-1 : 58*i]),

                     .ns_adapter_rstn(ns_adapter_rstn[i]),
                     .ns_mac_rdy(ns_mac_rdy[i]),
                     .fs_mac_rdy(fs_mac_rdy[i]),
                     .por(o_m_power_on_reset),
                     .i_osc_clk(i_osc_clk),

                     // JTAG interface
                     .jtag_clkdr_in(i_jtag_clkdr),
                     .scan_out(o_jtag_tdo_ch[i]),
                     .jtag_intest(i_jtag_intest),
                     .jtag_mode_in(i_jtag_mode),
                     .jtag_rstb(i_jtag_rstb),
                     .jtag_rstb_en(i_jtag_rstb_en),
                     .jtag_weakpdn(i_jtag_weakpdn),
                     .jtag_weakpu(i_jtag_weakpu),
                     .jtag_tx_scanen_in(i_jtag_tx_scanen),
                     // Daisy-chain the scan path: input is either top-level TDI or previous channel's TDO
                     .scan_in((i == 0) ? i_jtag_tdi : o_jtag_tdo_ch[i-1]),
                     .i_scan_clk(i_scan_clk),
                     .i_scan_clk_500m(i_scan_clk_500m),
                     .i_scan_clk_1000m(i_scan_clk_1000m),
                     .i_scan_en(i_scan_en),
                     .i_scan_mode(i_scan_mode),
                     .i_scan_din(i_scan_din[i]),
                     .i_scan_dout(i_scan_dout[i]),

                     .i_channel_id(6'(i)),
                     .i_cfg_avmm_clk(i_cfg_avmm_clk),
                     .i_cfg_avmm_rst_n(i_cfg_avmm_rst_n),
                     .i_cfg_avmm_addr(i_cfg_avmm_addr),
                     .i_cfg_avmm_byte_en(i_cfg_avmm_byte_en),
                     .i_cfg_avmm_read(i_cfg_avmm_read),
                     .i_cfg_avmm_write(i_cfg_avmm_write),
                     .i_cfg_avmm_wdata(i_cfg_avmm_wdata),

                     .o_cfg_avmm_rdatavld(o_cfg_avmm_rdatavld_ch[i]),
                     .o_cfg_avmm_rdata(o_rdata_ch[i]),
                     .o_cfg_avmm_waitreq(o_cfg_avmm_waitreq_ch[i])
                 );
             end else begin : inactive_channel_stub
                 // This block handles unused channels by tying off their outputs to a safe,
                 // low-power standby state.

                 // Tie off data and clock outputs to logic LOW
                 assign data_out_f[DATA_F_WIDTH*(i+1)-1 : DATA_F_WIDTH*i] = '0;
                 assign data_out[DATA_WIDTH*(i+1)-1 : DATA_WIDTH*i] = '0;
                 assign m_fs_rcv_clk[i] = 1'b0;
                 assign m_fs_fwd_clk[i] = 1'b0;
                 assign tclk_phy[i] = 1'b0;

                 // Tie off status and control signals to logic LOW (inactive state)
                 assign fs_mac_rdy[i] = 1'b0;
                 assign ms_tx_transfer_en[i] = 1'b0;
                 assign ms_rx_transfer_en[i] = 1'b0;
                 assign sl_tx_transfer_en[i] = 1'b0;
                 assign sl_rx_transfer_en[i] = 1'b0;
                 assign m_rx_align_done[i] = 1'b0;

                 // Tie off sideband outputs to logic LOW
                 assign sr_ms_tomac[MS_SSR_WIDTH*(i+1)-1 : MS_SSR_WIDTH*i] = '0;
                 assign sr_sl_tomac[SL_SSR_WIDTH*(i+1)-1 : SL_SSR_WIDTH*i] = '0;

                 // Tristate the main channel IO pads for this slice of the vector
                 assign iopad_aib[IOPAD_WIDTH*(i+1)-1 : IOPAD_WIDTH*i] = {IOPAD_WIDTH{1'bz}};

                 // Tie off scan output
                 assign i_scan_dout[i] = {MAX_SCAN_LEN{1'b0}};

                 // Bypass the JTAG chain for inactive channels
                 assign o_jtag_tdo_ch[i] = (i == 0) ? i_jtag_tdi : o_jtag_tdo_ch[i-1];

                 // Tie off AVMM signals for this channel
                 assign o_cfg_avmm_rdatavld_ch[i] = 1'b0;   // No valid data
                 assign o_cfg_avmm_waitreq_ch[i]  = 1'b1;   // Always busy
                 assign o_rdata_ch[i]             = 32'b0;  // Read data is zero
             end
         end
     endgenerate

aib_aux_channel  aib_aux_channel
   (
    // AIB IO Bidirectional
    .iopad_dev_dect(iopad_device_detect),
    .iopad_dev_dectrdcy(iopad_device_detect),
    .iopad_dev_por(iopad_power_on_reset),
    .iopad_dev_porrdcy(iopad_power_on_reset),

//  .device_detect_ms(ms_device_detect),
    .m_por_ovrd(m_por_ovrd),
    .m_device_detect_ovrd(m_device_detect_ovrd),
    .por_ms(o_m_power_on_reset),
    .m_device_detect(m_device_detect),
    .por_sl(i_m_power_on_reset),
//  .osc_clk(i_osc_clk),
    .ms_nsl(dual_mode_select),
    .irstb(1'b1) // Output buffer tri-state enable
    );


endmodule // aib
