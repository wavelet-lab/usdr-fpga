module usb2_ulpi_usbeps #(
    parameter AUX_BUS_RX = 1,
    parameter AUX_BUS_TX = 1,
    parameter USDR_PID = 0,
    parameter ULTRA_SCALE = 0
) (
    //ULPI ports
    input            phy_rst,
    input            phy_clk,

    output     [7:0] phy_do,
    input      [7:0] phy_di,
    output           phy_doe,

    input            phy_dir,
    input            phy_nxt,
    output           phy_stp,

    // Control
    input            cfg_usb2_en,

    // RB
    // Streaming [tag] OUT
    // Streaming [tag] IN
    output [31:0]    usb2_stat,
    output [31:0]    usb2_stat2,
    output           usb_bus_reset,

    //Data fifo IF
    input            clk,
    input            rst,


    // AXIS_M
    input  [AUX_BUS_RX-1:0]                   sn_axis_endpoint_rx_valid,
    output [AUX_BUS_RX-1:0]                   sn_axis_endpoint_rx_ready,
    input  [AUX_BUS_RX-1:0]                   sn_axis_endpoint_rx_last,
    input  [AUX_BUS_RX * 32-1:0]              sn_axis_endpoint_rx_data,
    input  [AUX_BUS_RX * 4-1:0]               sn_axis_endpoint_rx_keep,
    input  [AUX_BUS_RX-1:0]                   sn_axis_endpoint_rx_stall,

    // AXIS_M
    input  [AUX_BUS_TX-1:0]                   mn_axis_endpoint_tx_busy, //(busy flag -- do not try to push data)
    output [AUX_BUS_TX-1:0]                   mn_axis_endpoint_tx_valid,
    input  [AUX_BUS_TX-1:0]                   mn_axis_endpoint_tx_ready,
    output                                    m_axis_endpoint_tx_last,
    output [32-1:0]                           m_axis_endpoint_tx_data,
    output [4-1:0]                            m_axis_endpoint_tx_keep
);

// Output 32 bit streams are in little-endian format (both TX & RX)
//
wire usb_phy_reset;
wire async_reset;
synchronizer  #(.INIT(1), .ASYNC_RESET(1))  reset_phy_to_sys(.clk(clk), .rst(phy_rst), .a_in(usb_phy_reset /*1'b0*/), .s_out(async_reset));

wire usb_phy_ready;
wire async_usb_reset;
synchronizer  #(.INIT(1), .ASYNC_RESET(1))  resetusb_phy_to_sys(.clk(clk), .rst(usb_phy_ready), .a_in(1'b0), .s_out(async_usb_reset));


localparam EP_WIDTH = 5;
localparam MAX_EPSZ_WIDTH = 11;
localparam RAM_PARAGRAPH_WIDTH = 6;
localparam RAM_WIDTH = 12;

wire [6:0] self_usb_addr;

assign     usb_bus_reset = usb_phy_reset;

wire [7:0] axis_usb_rx_data;
wire       axis_usb_rx_valid;
wire       axis_usb_rx_last;
wire       axis_usb_rx_error;

// Streaming OUT (AXI stream)
wire [7:0]      usb_out_data;
wire            usb_out_valid;
wire            usb_out_txerror; //TX error sampled on the LAST byte
wire            usb_out_last;
wire            usb_out_ready;
wire [9:0] usb2_phy_chirp_debug;

wire            host_disconnected;
wire            cfg_usb_highspeed;

usb2_phy_chirp #(.NO_ADDR_CHECK(0)) usb2_phy_chirp(
    // PHY
    .phy_clk(phy_clk),

    .phy_do(phy_do),
    .phy_di(phy_di),
    .phy_doe(phy_doe),

    .phy_dir(phy_dir),
    .phy_nxt(phy_nxt),
    .phy_stp(phy_stp),

    // Control iface
    .reset(phy_rst),

    // Streaming IN (AXI stream)
    .axis_usb_rx_data(axis_usb_rx_data),
    .axis_usb_rx_valid(axis_usb_rx_valid),
    .axis_usb_rx_last(axis_usb_rx_last),
    .axis_usb_rx_error(axis_usb_rx_error),

    // Streaming OUT (AXI stream)
    .usb_out_data(usb_out_data),
    .usb_out_valid(usb_out_valid),
    .usb_out_txerror(usb_out_txerror), //TX error sampled on the LAST byte
    .usb_out_last(usb_out_last),
    .usb_out_ready(usb_out_ready),

    // Configuration
    .cfg_usb2_en(cfg_usb2_en),
    .cfg_usb_addr(self_usb_addr),

    // Status
    .stat_usb_hs(cfg_usb_highspeed),        // Entered HS mode succesfully
    .stat_phy_ready(usb_phy_ready),      // All initializations are done, phy is ready
    .stat_usb_reset(usb_phy_reset),

    .host_disconnected(host_disconnected),

    .debug_state(usb2_phy_chirp_debug)
);

wire                 usb_configured;
wire [RAM_WIDTH-1:0] mmfifo_addr;
wire [7:0]           mmfifo_data;
wire                 mmfifo_wr;
wire                 mmfifo_valid;
wire                 mmfifo_tag;
wire                 mmfifo_ready;

wire                 mmfifo_rb_ready;
reg                  mmfifo_rb_valid;
wire [7:0]           mmfifo_rb_data;
reg                  mmfifo_rb_tag;

localparam EP_CFG_WIDTH = 2*(RAM_WIDTH - RAM_PARAGRAPH_WIDTH) - 1 + (MAX_EPSZ_WIDTH - 2);


wire                                       updfifo_valid;
wire [EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:0] updfifo_data;
wire                                       updfifo_ready;

wire                                       cnffifo_valid;
wire [EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:0] cnffifo_data;
wire                                       cnffifo_ready;


wire [EP_WIDTH - 1:0]     tranif_ep_addr;
wire [EP_CFG_WIDTH - 1:0] tranif_ep_data;

wire [19:0] usb2_tran_if_debug;

usb2_tran_if #(
    .HWENUMERATOR(1),
    .USDR_PID(USDR_PID),
    .ULTRA_SCALE(ULTRA_SCALE)
) usb2_tran_if (
    .clk(phy_clk),
    .reset(usb_phy_reset),

    .cfg_usb_highspeed(cfg_usb_highspeed),
    .cfg_usb_addr(self_usb_addr),
    .cfg_usb_configured(usb_configured),

    .cfg_ep_addr(tranif_ep_addr),
    .cfg_ep_ramoffset(tranif_ep_data[EP_CFG_WIDTH - 1 : MAX_EPSZ_WIDTH - 2 + (RAM_WIDTH - RAM_PARAGRAPH_WIDTH - 1)]),
    .cfg_ep_fifozsz(tranif_ep_data[MAX_EPSZ_WIDTH - 2 + (RAM_WIDTH - RAM_PARAGRAPH_WIDTH - 1) - 1: MAX_EPSZ_WIDTH - 2]),

    .axis_usb_rx_data(axis_usb_rx_data),
    .axis_usb_rx_valid(axis_usb_rx_valid),
    .axis_usb_rx_last(axis_usb_rx_last),
    .axis_usb_rx_error(axis_usb_rx_error),

    .axis_usb_tx_data(usb_out_data),
    .axis_usb_tx_valid(usb_out_valid),
    .axis_usb_tx_last(usb_out_last),
    .axis_usb_tx_error(usb_out_txerror),
    .axis_usb_tx_ready(usb_out_ready),

    .mmfifo_addr(mmfifo_addr),
    .mmfifo_data(mmfifo_data),
    .mmfifo_wr(mmfifo_wr),
    .mmfifo_valid(mmfifo_valid),
    .mmfifo_tag(mmfifo_tag),
    .mmfifo_ready(mmfifo_ready),

    .mmfifo_rb_ready(mmfifo_rb_ready),
    .mmfifo_rb_valid(mmfifo_rb_valid),
    .mmfifo_rb_data(mmfifo_rb_data),
    .mmfifo_rb_tag(mmfifo_rb_tag),

    //Update interface format [ext][size][epaddr]
    .updfifo_valid(updfifo_valid),
    .updfifo_data(updfifo_data),
    .updfifo_ready(updfifo_ready),

    .cnffifo_valid(cnffifo_valid),
    .cnffifo_data(cnffifo_data),
    .cnffifo_ready(cnffifo_ready),

    .debug_state(usb2_tran_if_debug),
    .debug2(usb2_stat2)
);

// FIFO Backend

localparam DATA_BITS = 2;
localparam ID_WIDTH  = 1;

wire [RAM_WIDTH-1:2] bram_addr;
wire                 bram_en;
wire                 bram_we;
wire [31:0]          bram_data_wr;
wire [31:0]          bram_data_rd;
    
blk_mem_gen_usb blk_mem_gen_usb(
    .clka(phy_clk),
    .ena(mmfifo_valid && mmfifo_ready),
    .wea(mmfifo_valid && mmfifo_ready && mmfifo_wr),
    .addra(mmfifo_addr),
    .dina(mmfifo_data),
    .douta(mmfifo_rb_data),

    .clkb(clk),
    .enb(bram_en),
    .web({4{bram_we}}),
    .addrb(bram_addr),
    .dinb(bram_data_wr),
    .doutb(bram_data_rd)
);

assign mmfifo_ready = (mmfifo_wr) ? 1'b1 : (!mmfifo_rb_valid || mmfifo_rb_ready);

always @(posedge phy_clk) begin
    if (usb_phy_reset /*phy_rst*/) begin
        mmfifo_rb_valid <= 0;
    end else begin
        if (mmfifo_rb_valid && mmfifo_rb_ready) begin
            mmfifo_rb_valid <= 0;
        end
        if (mmfifo_valid && mmfifo_ready) begin
            if (!mmfifo_wr) begin
                mmfifo_rb_valid <= 1'b1;
                mmfifo_rb_tag   <= mmfifo_tag;
            end
        end
    end
end


wire [RAM_WIDTH-1:2] h_mmfifo_waddr;
wire [31:0]          h_mmfifo_wdata;
wire                 h_mmfifo_wvalid;
wire                 h_mmfifo_wready;

wire [RAM_WIDTH-1:2] h_mmfifo_araddr;
wire                 h_mmfifo_arvalid;
wire                 h_mmfifo_arready;
wire                 h_mmfifo_arid;

wire                 h_mmfifo_rready;
wire                 h_mmfifo_rvalid;
wire [31:0]          h_mmfifo_rdata;
wire                 h_mmfifo_rid;

alrdwr_to_bram #(
    .ADDR_WIDTH(RAM_WIDTH),
    .DATA_BITS(DATA_BITS),
    .BRAM_STAGES(1),
    .ID_WIDTH(ID_WIDTH),
    .RD_PRIO(1)
) alrdwr_to_bram (
    .clk(clk),
    .rst(rst),

    // AL Write address channel
    .s_al_waddr(h_mmfifo_waddr),
    .s_al_wvalid(h_mmfifo_wvalid),
    .s_al_wdata(h_mmfifo_wdata),
    .s_al_wready(h_mmfifo_wready),

    // AL Read address channel
    .s_al_araddr(h_mmfifo_araddr),
    .s_al_arvalid(h_mmfifo_arvalid),
    .s_al_arid(h_mmfifo_arid),
    .s_al_arready(h_mmfifo_arready),

    // AL Read data channel signals
    .s_al_rdata(h_mmfifo_rdata),
    .s_al_rvalid(h_mmfifo_rvalid),
    .s_al_rid(h_mmfifo_rid),
    .s_al_rready(h_mmfifo_rready),

    // BRAM interface
    .bram_addr(bram_addr),
    .bram_en(bram_en),
    .bram_we(bram_we),
    .bram_data_wr(bram_data_wr),
    .bram_data_rd(bram_data_rd)
);

wire [EP_WIDTH - 1:0]     axisrx_ep_addr;
wire [EP_CFG_WIDTH - 1:0] axisrx_ep_data;

wire [EP_WIDTH - 1:0]     axistx_ep_addr;
wire [EP_CFG_WIDTH - 1:0] axistx_ep_data;

wire [EP_WIDTH - 1:0]     ep_addr2;
wire [EP_CFG_WIDTH - 1:0] cfg_ep_size2;

//////////////////////////////////////////////////
// TODO HIGH SPEED / FULL SPEED EP size selection
// Endpoint configurtion  OFFSET_BUFEERSZ_EPSIZE
// FIFO RAM format
// [ RAM_offset / 64][ BUFFER_size / 64 - 1 ][ MAX_EP_SIZE / 4 - 1]
// 4kB RAM = 64 paragraphs total

(* rom_style = "distributed" *)
wire [EP_CFG_WIDTH-1:0] fifo_ep_cfg_rom[0:31];

// EP configuration in RAM                                offset + size         MaxEP
assign fifo_ep_cfg_rom[0]  = { 6'd0,   5'd2,   9'd15};   // 0    +  192b  FIFO   64b
assign fifo_ep_cfg_rom[1]  = { 6'd3,   5'd8,   9'd127};  // 192  +  576b  FIFO  512b
assign fifo_ep_cfg_rom[2]  = { 6'd0,   5'd0,   9'd0};    //     invalid
assign fifo_ep_cfg_rom[3]  = { 6'd12,  5'd19,  9'd127};  // 768  + 1280b  FIFO  512b

assign fifo_ep_cfg_rom[16] = { 6'd0,  5'd2,  9'd15};     // 0    + 192b   FIFO   64b
assign fifo_ep_cfg_rom[17] = { 6'd32, 5'd8,  9'd127};    // 2048 + 576b   FIFO  512b
assign fifo_ep_cfg_rom[18] = { 6'd41, 5'd2,  9'd15};     // 2624 + 192b   FIFO   64b
assign fifo_ep_cfg_rom[19] = { 6'd44, 5'd19, 9'd127};    // 2816 + 1280b  FIFO  512b

assign cfg_ep_size2   = fifo_ep_cfg_rom[ep_addr2];
assign tranif_ep_data = fifo_ep_cfg_rom[tranif_ep_addr];
assign axistx_ep_data = fifo_ep_cfg_rom[axistx_ep_addr];
assign axisrx_ep_data = fifo_ep_cfg_rom[axisrx_ep_addr];

wire                                       axis_updfifo_valid;
wire [EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:0] axis_updfifo_data;
wire                                       axis_updfifo_ready;

wire                                       axis_cnffifo_valid;
wire [EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:0] axis_cnffifo_data;
wire                                       axis_cnffifo_ready;

axis_opt_cc_fifo #(.WIDTH(EP_WIDTH + MAX_EPSZ_WIDTH - 2), .CLK_CC(1)) async_cnffifo_q (
  .rx_clk(clk),
  .rx_rst(async_reset),
  .s_rx_tdata(axis_cnffifo_data),
  .s_rx_tvalid(axis_cnffifo_valid),
  .s_rx_tready(axis_cnffifo_ready),

  .tx_clk(phy_clk),
  .tx_rst(usb_phy_reset /*phy_rst*/),
  .m_tx_tdata(cnffifo_data),
  .m_tx_tvalid(cnffifo_valid),
  .m_tx_tready(cnffifo_ready)
);

axis_opt_cc_fifo #(.WIDTH(EP_WIDTH + MAX_EPSZ_WIDTH - 2), .CLK_CC(1)) async_updfifo_q (
  .rx_clk(phy_clk),
  .rx_rst(usb_phy_reset/*phy_rst*/),
  .s_rx_tdata(updfifo_data),
  .s_rx_tvalid(updfifo_valid),
  .s_rx_tready(updfifo_ready),

  .tx_clk(clk),
  .tx_rst(async_reset),
  .m_tx_tdata(axis_updfifo_data),
  .m_tx_tvalid(axis_updfifo_valid),
  .m_tx_tready(axis_updfifo_ready)
);


wire axisrx_upd_selector = axis_updfifo_data[4];
wire axisrx_cnf_selector;

wire                                       axistx_updfifo_valid  = !axisrx_upd_selector && axis_updfifo_valid;
wire [EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:0] axistx_updfifo_data   = axis_updfifo_data;
wire                                       axistx_updfifo_ready;

wire                                       axistx_cnffifo_valid;
wire [EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:0] axistx_cnffifo_data;
wire                                       axistx_cnffifo_ready  = !axisrx_cnf_selector && axis_cnffifo_ready;


wire                                       axisrx_updfifo_valid  = axisrx_upd_selector && axis_updfifo_valid;
wire [EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:0] axisrx_updfifo_data   = axis_updfifo_data;
wire                                       axisrx_updfifo_ready;

wire                                       axisrx_cnffifo_valid;
wire [EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:0] axisrx_cnffifo_data;
wire                                       axisrx_cnffifo_ready  = axisrx_cnf_selector && axis_cnffifo_ready;

assign axisrx_cnf_selector = axisrx_cnffifo_valid;
assign axis_updfifo_ready = (axisrx_upd_selector) ?  axisrx_updfifo_ready : axistx_updfifo_ready;

assign axis_cnffifo_valid = axistx_cnffifo_valid || axisrx_cnffifo_valid;
assign axis_cnffifo_data  = (axisrx_cnf_selector) ? axisrx_cnffifo_data : axistx_cnffifo_data;


usb_fe_axis_tx #(.AUX_BUS_TX(AUX_BUS_TX), .ULTRA_SCALE(ULTRA_SCALE)) usb_fe_axis_tx(
    .clk(clk),
    .reset(async_reset /*async_usb_reset*/ /*async_reset*/),

    .mmfifo_addr(h_mmfifo_araddr),
    .mmfifo_valid(h_mmfifo_arvalid),
    .mmfifo_tag(h_mmfifo_arid),
    .mmfifo_ready(h_mmfifo_arready),

    .mmfifo_rb_ready(h_mmfifo_rready),
    .mmfifo_rb_valid(h_mmfifo_rvalid),
    .mmfifo_rb_data(h_mmfifo_rdata),
    .mmfifo_rb_tag(h_mmfifo_rid),

    //Update interface format [ext][size][epaddr]
    .updfifo_valid(axistx_updfifo_valid),
    .updfifo_data(axistx_updfifo_data),
    .updfifo_ready(axistx_updfifo_ready),

    .cnffifo_valid(axistx_cnffifo_valid),
    .cnffifo_data(axistx_cnffifo_data),
    .cnffifo_ready(axistx_cnffifo_ready),

    // TODO EP0 control IF for MCU handling
    .ep_addr(axistx_ep_addr),
    .cfg_ep_ramoffset(axistx_ep_data[EP_CFG_WIDTH - 1 : MAX_EPSZ_WIDTH - 2 + (RAM_WIDTH - RAM_PARAGRAPH_WIDTH - 1)]),
    .cfg_ep_fifozsz(axistx_ep_data[MAX_EPSZ_WIDTH - 2 + (RAM_WIDTH - RAM_PARAGRAPH_WIDTH - 1) - 1: MAX_EPSZ_WIDTH - 2]),
    .cfg_ep_size(axistx_ep_data[MAX_EPSZ_WIDTH - 2 - 1:0]),

    // AXIS_M
    .mn_axis_endpoint_tx_busy(mn_axis_endpoint_tx_busy), //(busy flag -- do not try to push data)
    .mn_axis_endpoint_tx_valid(mn_axis_endpoint_tx_valid),
    .mn_axis_endpoint_tx_ready(mn_axis_endpoint_tx_ready),
    .m_axis_endpoint_tx_last(m_axis_endpoint_tx_last),
    .m_axis_endpoint_tx_data(m_axis_endpoint_tx_data),
    .m_axis_endpoint_tx_keep(m_axis_endpoint_tx_keep)

    // AXIS stall interface
);

usb_fe_axis_rx #(.AUX_BUS_RX(AUX_BUS_RX), .ULTRA_SCALE(ULTRA_SCALE)) usb_fe_axis_rx(
    .clk(clk),
    .reset(async_reset),

    .mmfifo_addr(h_mmfifo_waddr),
    .mmfifo_data(h_mmfifo_wdata),
    .mmfifo_valid(h_mmfifo_wvalid),
    .mmfifo_ready(h_mmfifo_wready),

    //Update interface format [ext][size][epaddr]
    .updfifo_valid(axisrx_updfifo_valid),
    .updfifo_data(axisrx_updfifo_data),
    .updfifo_ready(axisrx_updfifo_ready),

    .cnffifo_valid(axisrx_cnffifo_valid),
    .cnffifo_data(axisrx_cnffifo_data),
    .cnffifo_ready(axisrx_cnffifo_ready),

    // TODO EP0 control IF for MCU handling
    .ep_addr(axisrx_ep_addr),
    .cfg_ep_ramoffset(axisrx_ep_data[EP_CFG_WIDTH - 1 : MAX_EPSZ_WIDTH - 2 + (RAM_WIDTH - RAM_PARAGRAPH_WIDTH - 1)]),
    .cfg_ep_fifozsz(axisrx_ep_data[MAX_EPSZ_WIDTH - 2 + (RAM_WIDTH - RAM_PARAGRAPH_WIDTH - 1) - 1: MAX_EPSZ_WIDTH - 2]),
    .cfg_ep_size(axisrx_ep_data[MAX_EPSZ_WIDTH - 2 - 1:0]),

    .ep_addr2(ep_addr2),
    .cfg_ep_size2(cfg_ep_size2),

    // AXIS_M
    .sn_axis_endpoint_rx_valid(sn_axis_endpoint_rx_valid),
    .sn_axis_endpoint_rx_ready(sn_axis_endpoint_rx_ready),
    .sn_axis_endpoint_rx_last(sn_axis_endpoint_rx_last),
    .sn_axis_endpoint_rx_data(sn_axis_endpoint_rx_data),
    .sn_axis_endpoint_rx_keep(sn_axis_endpoint_rx_keep),

    // AXIS stall interface
    .sn_axis_endpoint_rx_stall(sn_axis_endpoint_rx_stall)
);

assign usb2_stat[19:0]  = usb2_tran_if_debug;
assign usb2_stat[31:20] = { host_disconnected, 1'b0, usb2_phy_chirp_debug};


endmodule
