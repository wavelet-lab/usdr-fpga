module ddr_reciver #(
    parameter       DATA_WIDTH      = 12,
    parameter       DELAY_REFCLK    = 200,
    parameter [0:0] WITH_FIFO       = 0,
    parameter [0:0] NO_CLK_DELAY    = 0,
    parameter [0:0] EXTERNAL_CLOCKS = 0,
    parameter [0:0] ALT_PS          = 1'b0,
    parameter [0:0] LML7_LFSR_CHK   = 1'b1,

    parameter [47:0] PHASE_IQAB_ALIGN_0 = 48'h5A5_969_BBE_441,
    parameter [47:0] PHASE_IQAB_ALIGN_1 = 48'hF0F_356_C42_676,

    parameter [0:0]  ULTRA_SCALE        = 0,
    parameter [0:0]  IDDR_STYLE         = 1'b0,
    parameter        CLK_CFG_RATE       = 125_000_000,
    parameter [0:0]  HAS_CAPTURE        = 0,
    parameter [0:0]  HAS_DC_ESTIMATOR   = 1,
    parameter        CAPTURE_DEEP       = 128
)(
    // DDR interface
    input  ddr_clk_ref,
    output ddr_clk_fwd,

    input                         ddr_frame,
    input [DATA_WIDTH-1:0]        ddr_data,

    // Out logic interface
    output                        l_clk,
    output                        l_valid,
    output [4 * DATA_WIDTH - 1:0] l_data,

    // External clocking for iserdese2
    input         extclk_buf_io,
    input         extclk_bufr,
    input         extclk_fwd,
    input         extclk_usr,

    output        extclk_psen,
    output        extclk_psclk,
    input         extclk_psdone,

    // Control
    input         cfg_clk,
    input         cfg_rst,

    input         cfg_wvalid,
    output        cfg_wready,
    input [31:0]  cfg_wdata,

    output        cfg_rvalid,
    input         cfg_rready,
    output [31:0] cfg_rdata,

    output        rxd_rst_gclk
);

localparam       LML7_LFSR_POLY       = 15'h6000;
localparam       LML7_LFSR_WIDTH      = 15;
localparam [3:0] LML7_L_XOR           = 4'b1010;
localparam       LML7_LFSR_STAT_WIDTH = 32;

wire [3:0] lml7_lfsr_sel_rb;
wire [3:0] lml7_lfsr_rst;
wire [3:0] lml7_lfsr_en;
wire       lml7_iqab_en;
wire       lml7_abmm_en;

reg [LML7_LFSR_STAT_WIDTH - 1:0] lml7_lfsr_data_rb;

reg [LML7_LFSR_STAT_WIDTH-1:0]     iq_cnt;
reg [LML7_LFSR_STAT_WIDTH-1:0]     ab_mm_cnt;

wire [4*LML7_LFSR_STAT_WIDTH-1:0]  sync_error;
wire [4*LML7_LFSR_STAT_WIDTH-1:0]  lost_error;
wire [4*LML7_LFSR_STAT_WIDTH-1:0]  ber_error;
wire [4*LML7_LFSR_STAT_WIDTH-1:0]  iqs_error = { ab_mm_cnt, ab_mm_cnt, iq_cnt, iq_cnt };

wire [4*LML7_LFSR_STAT_WIDTH-1:0]  vdsel_error =
    lml7_lfsr_sel_rb[3:2] == 0 ? sync_error :
    lml7_lfsr_sel_rb[3:2] == 1 ? lost_error :
    lml7_lfsr_sel_rb[3:2] == 2 ? ber_error  : iqs_error;

wire [LML7_LFSR_STAT_WIDTH-1:0]  dsel_error =
    lml7_lfsr_sel_rb[1:0] == 0 ? vdsel_error[LML7_LFSR_STAT_WIDTH * (0 + 1) - 1:LML7_LFSR_STAT_WIDTH * 0] :
    lml7_lfsr_sel_rb[1:0] == 1 ? vdsel_error[LML7_LFSR_STAT_WIDTH * (1 + 1) - 1:LML7_LFSR_STAT_WIDTH * 1] :
    lml7_lfsr_sel_rb[1:0] == 2 ? vdsel_error[LML7_LFSR_STAT_WIDTH * (2 + 1) - 1:LML7_LFSR_STAT_WIDTH * 2] :
                                 vdsel_error[LML7_LFSR_STAT_WIDTH * (3 + 1) - 1:LML7_LFSR_STAT_WIDTH * 3];

reg [47:0] ph0;
reg [47:0] ph1;
reg        ph_s;

/*
vio_1 vio_1(
    .clk(cfg_clk),
    .probe_in0(sync_error),
    .probe_in1(lost_error),
    .probe_in2(ber_error),
    .probe_in3(lml7_lfsr_rst),
    .probe_in4(lml7_lfsr_en),
    .probe_in5(lml7_lfsr_sel_rb),
    .probe_in6(iq_cnt),
    .probe_in7({ ph1, ph0 })
);
*/

generate
for (genvar j = 0; j < 4; j=j+1) begin
lfsr_checker #(
    .DATA_WIDTH(DATA_WIDTH),
    .LFSR_POLY(LML7_LFSR_POLY),
    .LFSR_LEN(LML7_LFSR_WIDTH),
    .STAT_WIDTH(LML7_LFSR_STAT_WIDTH),
    .XOR_CONST(LML7_L_XOR[j])
) lml7_lfsr (
    .clk(l_clk),
    .rst(lml7_lfsr_rst[j]),
    .en(lml7_lfsr_en[j]),

    .data_i(l_data[DATA_WIDTH * (j + 1) - 1:DATA_WIDTH * j]),
    .alarm_o(),

    .sync_error(sync_error[LML7_LFSR_STAT_WIDTH * (j + 1) - 1:LML7_LFSR_STAT_WIDTH * j]),
    .lost_error(lost_error[LML7_LFSR_STAT_WIDTH * (j + 1) - 1:LML7_LFSR_STAT_WIDTH * j]),
    .ber_error(ber_error[LML7_LFSR_STAT_WIDTH * (j + 1) - 1:LML7_LFSR_STAT_WIDTH * j])
);
end
endgenerate

wire cfg_siso_ddr_skip;
wire iqab_chk_0 = cfg_siso_ddr_skip ? (l_data[2 * DATA_WIDTH - 1:0] == PHASE_IQAB_ALIGN_0[2 * DATA_WIDTH - 1:0]) : (l_data == PHASE_IQAB_ALIGN_0);
wire iqab_chk_1 = cfg_siso_ddr_skip ? (l_data[2 * DATA_WIDTH - 1:0] == PHASE_IQAB_ALIGN_1[2 * DATA_WIDTH - 1:0]) : (l_data == PHASE_IQAB_ALIGN_1);

always @(posedge l_clk) begin
    lml7_lfsr_data_rb <= dsel_error;

    if (&lml7_lfsr_en) begin
      if ((l_data[DATA_WIDTH * (1 + 1) - 1:DATA_WIDTH * 1] != ~l_data[DATA_WIDTH * (0 + 1) - 1:DATA_WIDTH * 0]) ||
            (l_data[DATA_WIDTH * (3 + 1) - 1:DATA_WIDTH * 3] != ~l_data[DATA_WIDTH * (2 + 1) - 1:DATA_WIDTH * 2]))  begin
        iq_cnt <= iq_cnt + 1'b1;
      end
    end else if (lml7_iqab_en) begin
      ph_s <= ~ph_s;

      if (ph_s)
        ph1 <= l_data;
      else
        ph0 <= l_data;

      if (iqab_chk_0 == 1'b0 && iqab_chk_1 == 1'b0) begin
        iq_cnt <= iq_cnt + 1'b1;
      end
    end else if (lml7_abmm_en) begin
        if (l_data[DATA_WIDTH * 4 - 1:DATA_WIDTH * 2] != l_data[DATA_WIDTH * 2 - 1:DATA_WIDTH * 0]) begin
            ab_mm_cnt <= ab_mm_cnt + 1'b1;
        end
    end else begin
      iq_cnt    <= 0;
      ph_s      <= 0;
      ab_mm_cnt <= 0;
    end
end



localparam ASYNC_RESETS = 1'b0;

// [r/w addr14 data16 ]

// addr14..addr8 -- Banks
localparam [6:0]
    ADDR_BANK_CTRL = 0,    // Control registers
    ADDR_BANK_MMCM = 1,    // MMCM registers
    ADDR_BANK_CLKMEAS = 2,
    ADDR_BANK_LFSRCHK = 3, // LFSR cntrol
    ADDR_BANK_IQABCHK = 4,
    ADDR_BANK_CAPTURE = 5, // IQ capture
    ADDR_BANK_DC_EST  = 6, // DC estimator
    ADDR_BANK_CLKDLY = 14, // Clock delay
    ADDR_BANK_FRMDLY = 15, // Frame delay
    ADDR_BANK_D0DLY  = 16, // D0 through D15 delay
    ADDR_BANK_D15DLY = 31;

localparam [7:0]
    ADDR_CTRL_DESER = 0,
    ADDR_CTRL_MMCM = 1,
    ADDR_CTRL_MAN = 2; // Manual control for fine tune phase & bitslip

wire        cfg_wwr   = cfg_wdata[31];
wire [6:0]  cfg_wbank = cfg_wdata[30:24];
wire [7:0]  cfg_baddr = cfg_wdata[23:16];

reg [14:0] rb_addr;

always @(posedge cfg_clk) begin
    if (cfg_rst) begin
        rb_addr <= 0;
    end else if (cfg_wvalid && cfg_wready && !cfg_wwr) begin
        rb_addr <= { cfg_wbank, cfg_baddr };
    end
end

assign lml7_lfsr_sel_rb = rb_addr[3:0];

wire rb_bank_ctrl = (rb_addr[14:8] == ADDR_BANK_CTRL);
wire rb_bank_mmcm = (rb_addr[14:8] == ADDR_BANK_MMCM);
wire rb_meas_clk  = (rb_addr[14:8] == ADDR_BANK_CLKMEAS);
wire rb_lfsr_reg  = (rb_addr[14:8] == ADDR_BANK_LFSRCHK);
wire rb_capture   = (rb_addr[14:8] == ADDR_BANK_CAPTURE);
wire rb_dc_estim  = (rb_addr[14:8] == ADDR_BANK_DC_EST);

wire rb_capture_h =  rb_addr[7];
wire [6:0] rb_capture_addr = rb_addr[6:0];
wire [2:0] rb_dc_estim_w   = rb_addr[2:0];

wire wr_bank_ctrl   = (cfg_wbank == ADDR_BANK_CTRL);
wire wr_bank_mmcm   = (cfg_wbank == ADDR_BANK_MMCM);
wire wr_bank_clkdly = (cfg_wbank == ADDR_BANK_CLKDLY);
wire wr_bank_frmdly = (cfg_wbank == ADDR_BANK_FRMDLY);
wire wr_bank_lfsr   = (cfg_wbank == ADDR_BANK_LFSRCHK);
wire wr_bank_iqab   = (cfg_wbank == ADDR_BANK_IQABCHK);
wire wr_bank_captrue= (cfg_wbank == ADDR_BANK_CAPTURE);
wire wr_bank_dc_estim= (cfg_wbank == ADDR_BANK_DC_EST);

reg  [3:0] cfg_lml7_lfsr_rst;
reg  [3:0] cfg_lml7_lfsr_en;
reg        cfg_lml7_iqab_en;
reg        cfg_lml7_abmm_en;

wire [4 * DATA_WIDTH - 1:0] rb_capture_data;
wire [2 * DATA_WIDTH - 1:0] rb_capture_data_half = rb_capture_h ? rb_capture_data[4 * DATA_WIDTH - 1:2 * DATA_WIDTH] : rb_capture_data[2 * DATA_WIDTH - 1:0];
wire                        rb_capture_busy;
wire [6:0]                  rb_capture_progress;

generate
for (genvar k = 0; k < 4; k = k + 1) begin
synchronizer  #(.INIT(1), .ASYNC_RESET(1'b0)) s_lml7_lfsr_rst_x    (.clk(l_clk), .rst(1'b0), .a_in(cfg_lml7_lfsr_rst[k]),   .s_out(lml7_lfsr_rst[k]));
synchronizer  #(.INIT(0), .ASYNC_RESET(1'b0)) s_lml7_lfsr_en_x     (.clk(l_clk), .rst(1'b0), .a_in(cfg_lml7_lfsr_en[k]),    .s_out(lml7_lfsr_en[k]));
end
endgenerate

synchronizer  #(.INIT(0), .ASYNC_RESET(1'b0)) s_lml7_iqab_en_x     (.clk(l_clk), .rst(1'b0), .a_in(cfg_lml7_iqab_en),   .s_out(lml7_iqab_en));
synchronizer  #(.INIT(0), .ASYNC_RESET(1'b0)) s_lml7_abmm_en_x     (.clk(l_clk), .rst(1'b0), .a_in(cfg_lml7_abmm_en),   .s_out(lml7_abmm_en));


always @(posedge cfg_clk) begin
    if (wr_bank_lfsr && cfg_wvalid && cfg_wwr) begin
        cfg_lml7_lfsr_rst    <= cfg_wdata[3:0];
        cfg_lml7_lfsr_en     <= cfg_wdata[7:4];
    end

    if (wr_bank_iqab && cfg_wvalid && cfg_wwr) begin
        cfg_lml7_iqab_en     <= cfg_wdata[0];
        cfg_lml7_abmm_en     <= cfg_wdata[1];
    end
end

// MMCM port
wire cfg_wready_mmcm;
wire cfg_rvalid_mmcm;
wire [31:0] cfg_rdata_mmcm;

// Calibration data
wire [7:0] data_clkdly;

wire [31:0] bitslip_data;

wire [31:0] clk_meas;

wire [31:0] rb_dc_estim_meas;

assign cfg_rdata =
    rb_dc_estim  ? rb_dc_estim_meas :
    rb_capture   ? { rb_capture_busy, rb_capture_progress, rb_capture_data_half } :
    rb_lfsr_reg  ? lml7_lfsr_data_rb :
    rb_bank_mmcm ? cfg_rdata_mmcm :
    rb_bank_ctrl ? bitslip_data :
    rb_meas_clk  ? clk_meas : { clk_meas[31:8], data_clkdly };
assign cfg_rvalid =
    rb_bank_mmcm ? cfg_rvalid_mmcm : 1'b1;

assign cfg_wready = (wr_bank_mmcm /*&& cfg_wwr*/) ? cfg_wready_mmcm : 1'b1;



// Clocks
wire ddr_clk_io;
wire ddr_clk_div;
wire ddr_clk_div_logic;

wire tune_phase_valid;
wire tune_phase_ready;

wire ddr_reset;


reg [27:0] clk125_cnt = 0;
reg        clk125_pulse = 0;
always @(posedge cfg_clk) begin
    clk125_cnt <= clk125_cnt + 1'b1;
    if (clk125_cnt == (CLK_CFG_RATE - 1)) begin
        clk125_cnt   <= 0;
        clk125_pulse <= ~clk125_pulse;
    end
end

clock_measurement #(.POSEDGE_ONLY(1'b0), .GEN_WIDTH(4), .CNT_WIDTH(28)) clk_ref_meas(
	.meas_clk_i(ddr_clk_div_logic),
	.ref_pulse_i(clk125_pulse),
	.ref_rst_i(0),
	.meas_data_o(clk_meas)
);


generate
if (EXTERNAL_CLOCKS) begin

    reg tune_phase_busy = 1'b0;
    if (ALT_PS) begin
        always @(posedge extclk_psclk) begin
            if (extclk_psen) begin
                tune_phase_busy <= 1'b1;
            end
            if (extclk_psdone) begin
                tune_phase_busy <= 1'b0;
            end
        end
    end

    assign ddr_clk_io         = extclk_buf_io;
    assign ddr_clk_div        = (ULTRA_SCALE && IDDR_STYLE) ? extclk_buf_io : extclk_bufr;

    assign ddr_clk_div_logic  = extclk_usr; //extclk_bufr; //extclk_usr; // extclk_bufr
    assign ddr_clk_fwd        = extclk_fwd;

    assign extclk_psen        = (ALT_PS) ? tune_phase_valid : 1'b0;
    assign extclk_psclk       = (ALT_PS) ? ddr_clk_div      : 1'b0;
    assign tune_phase_ready   = ~tune_phase_busy;

    assign data_clkdly        = clk_meas;

end else if (ULTRA_SCALE) begin
    assign tune_phase_ready = 1'b1;
    assign cfg_rdata_mmcm = 32'hffff_ffff;
    assign cfg_wready_mmcm = 1'b1;
    assign cfg_rvalid_mmcm = 1'b1;

    if (IDDR_STYLE) begin
        BUFG clk_ref_bufg( .I(ddr_clk_ref), .O(ddr_clk_div));
        assign ddr_clk_io = ddr_clk_div;
    end else begin
        BUFGCE_DIV #(
            .BUFGCE_DIVIDE(1)
        ) bufg_clk_i (
            .I   (ddr_clk_ref),
            .CE  (1'b1),
            .CLR (1'b0),
            .O   (ddr_clk_io)
        );

        BUFGCE_DIV #(
            .BUFGCE_DIVIDE(2)
        ) bufg_clkdiv_i (
            .I   (ddr_clk_ref),
            .CE  (1'b1),
            .CLR (1'b0),
            .O   (ddr_clk_div)
        );
    end

    assign ddr_clk_div_logic = ddr_clk_div;

end else begin
    assign tune_phase_ready = 1'b1;
    assign cfg_rdata_mmcm = 32'hffff_ffff;
    assign cfg_wready_mmcm = 1'b1;
    assign cfg_rvalid_mmcm = 1'b1;

    xlnx_ddr_clock #(.DELAY_REFCLK(DELAY_REFCLK), .NO_CLK_DELAY(NO_CLK_DELAY)) clk (
        .ref_clk(ddr_clk_ref),

        .lvds_data_clk_div(ddr_clk_div),
        .lvds_in_bufio(ddr_clk_io),

        .lvds_delay_ld(wr_bank_clkdly && cfg_wvalid && cfg_wwr),
        .lvds_delay_data(cfg_wdata),
        .clk_data_l(data_clkdly)
    );

    ODDR #(
      .DDR_CLK_EDGE("OPPOSITE_EDGE"),
      .SRTYPE("SYNC"),
      .INIT(0)
    ) oddr_fclk (
      .Q(ddr_clk_fwd),
      .C(1'b0),
      .CE(1'b1),
      .D1(1'b1),
      .D2(1'b0),
      .R(1'b0)
    );

    // todo add option to bypass th buffer
    BUFG logic_buf(.I(ddr_clk_div), .O(ddr_clk_div_logic));

    // TODO: add phase tuner
end
endgenerate


localparam
    CFG_DESER_DDR_RESET     = 0,
    CFG_DESER_BITSLIP_RESET = 1,
    CFG_DESER_SISODDR       = 3,
    CFG_DESER_SISODDR_SKIP  = 4;  // ???

// Sync regs
reg [5:0] ctrl_deser;
reg [1:0] ctrl_man;
reg       ctrl_capture;
reg       ctrl_dc_est_rst;
reg [7:0] ctrl_dc_est_acc;

assign cfg_siso_ddr_skip = ctrl_deser[4];

always @(posedge cfg_clk) begin
    if (cfg_rst) begin
        ctrl_deser   <= 6'b000111;
        ctrl_man     <= 2'b00;
        ctrl_capture <= 1'b0;
    end else begin
        if (wr_bank_ctrl && cfg_wvalid && cfg_wwr && cfg_baddr == ADDR_CTRL_DESER) begin
            ctrl_deser <= cfg_wdata;
        end
        if (wr_bank_ctrl && cfg_wvalid && cfg_wwr && cfg_baddr == ADDR_CTRL_MAN) begin
            ctrl_man   <= cfg_wdata;
        end
        if (wr_bank_captrue && cfg_wvalid && cfg_wwr && cfg_baddr == 0) begin
            ctrl_capture <= cfg_wdata;
        end
        if (wr_bank_dc_estim && cfg_wvalid && cfg_wwr && cfg_baddr == 0) begin
            ctrl_dc_est_rst <= cfg_wdata;
        end
        if (wr_bank_dc_estim && cfg_wvalid && cfg_wwr && cfg_baddr == 1) begin
            ctrl_dc_est_acc <= cfg_wdata;
        end
    end
end


wire sisoddrmode_clkdiv;
wire sisoddrmode_clkdiv_skip; // Use x2 receiver frequency in odrer to use ISERDESEx in SISO_DDR mode
wire capture_lclk;
wire dc_est_rst_lclk;

synchronizer  #(.INIT(0), .ASYNC_RESET(ASYNC_RESETS)) sisoddrmode_man(.clk(ddr_clk_div), .rst(ddr_reset), .a_in(ctrl_deser[3]), .s_out(sisoddrmode_clkdiv));
synchronizer  #(.INIT(0), .ASYNC_RESET(ASYNC_RESETS)) sisoddrmode_skp(.clk(ddr_clk_div), .rst(ddr_reset), .a_in(ctrl_deser[4]), .s_out(sisoddrmode_clkdiv_skip));
synchronizer  #(.INIT(1), .ASYNC_RESET(ASYNC_RESETS)) ddr_reset_reg  (.clk(ddr_clk_div), .rst(ctrl_deser[0]), .a_in(1'b0), .s_out(ddr_reset));
assign rxd_rst_gclk = ctrl_deser[0];

synchronizer  #(.INIT(0), .ASYNC_RESET(ASYNC_RESETS)) capture_start   (.clk(l_clk), .rst(ddr_reset), .a_in(ctrl_capture), .s_out(capture_lclk));
synchronizer  #(.INIT(0), .ASYNC_RESET(ASYNC_RESETS)) dc_estim_start  (.clk(l_clk), .rst(ddr_reset), .a_in(ctrl_dc_est_rst), .s_out(dc_est_rst_lclk));


wire [4*DATA_WIDTH-1:0] ddr_data_realigned;
wire                    ddr_valid_realigned;

genvar i, j, k;

generate
if (ULTRA_SCALE) begin
    wire [4*DATA_WIDTH-1:0] ddr_data_deser;
    wire [3:0]              ddr_frame_deser;
    wire                    ddr_data_valid;

    if (IDDR_STYLE) begin
        wire [2 * DATA_WIDTH - 1:0] ddr_in_2_data;
        wire [1:0]                  ddr_in_2_frame;

        for (k = 0; k < DATA_WIDTH; k = k + 1) begin: lml_bufs
            IDDRE1 #(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED")) iddr_data (
                .D(ddr_data[k]),
                .Q1(ddr_in_2_data[0          + k]),
                .Q2(ddr_in_2_data[DATA_WIDTH + k]),
                .C(ddr_clk_div),
                .CB(!ddr_clk_div),
                .R(1'b0)
            );
        end

        IDDRE1 #(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED")) iddr_data (
            .D(ddr_frame),
            .Q1(ddr_in_2_frame[0]),
            .Q2(ddr_in_2_frame[1]),
            .C(ddr_clk_div),
            .CB(!ddr_clk_div),
            .R(1'b0)
        );

        reg [4 * DATA_WIDTH - 1:0] ddr_in_4_data;
        reg [3:0]                  ddr_in_4_frame;
        reg                        ddr_in_4_valid;
        reg                        ddr_ch_switch;

        always @(posedge ddr_clk_div) begin
            if (ddr_reset) begin
                ddr_in_4_valid <= 1'b0;
                ddr_ch_switch  <= 1'b0;
            end else begin
                ddr_ch_switch  <= ~ddr_ch_switch || sisoddrmode_clkdiv;
                ddr_in_4_valid <= ddr_ch_switch || sisoddrmode_clkdiv;

                if (ddr_ch_switch) begin
                    ddr_in_4_data[47:24] <= ddr_in_2_data;
                    ddr_in_4_frame[3:2]  <= ddr_in_2_frame;
                end else begin
                    ddr_in_4_data[23:0]  <= ddr_in_2_data;
                    ddr_in_4_frame[1:0]  <= ddr_in_2_frame;
                end

                if (sisoddrmode_clkdiv) begin
                    ddr_in_4_data[23:0]  <= ddr_in_4_data[47:24];
                    ddr_in_4_frame[1:0]  <= ddr_in_4_frame[3:2];
                end
            end
        end

        assign ddr_data_deser = ddr_in_4_data;
        assign ddr_frame_deser = ddr_in_4_frame;
        assign ddr_data_valid = ddr_in_4_valid;

    end else begin

        for (i = 0; i < DATA_WIDTH; i=i+1) begin: dlvds
            wire [3:0] bit_deser;

            (* keep = "TRUE" *)
            ISERDESE3 #(
                .DATA_WIDTH(4),                 // Parallel data width (4,8)
                .FIFO_ENABLE("FALSE"),          // Enables the use of the FIFO
                .FIFO_SYNC_MODE("FALSE"),       // Always set to FALSE. TRUE is reserved for later use.
                .IS_CLK_B_INVERTED(1'b1),       // Optional inversion for CLK_B
                .IS_CLK_INVERTED(1'b0),         // Optional inversion for CLK
                .IS_RST_INVERTED(1'b0),         // Optional inversion for RST
                .SIM_DEVICE("ULTRASCALE_PLUS")
            ) iserdese3_data_inst (
                .FIFO_EMPTY(),                     // 1-bit output: FIFO empty flag
                .INTERNAL_DIVCLK(),
                .Q(bit_deser),                     // 8-bit registered output
                .CLK(ddr_clk_io),                  // 1-bit input: High-speed clock
                .CLKDIV(ddr_clk_div),              // 1-bit input: Divided Clock
                .CLK_B(ddr_clk_io),                // 1-bit input: Inversion of High-speed clock CLK
                .D(ddr_data[i]),                   // 1-bit input: Serial Data Input
                .FIFO_RD_CLK(1'b0),                // 1-bit input: FIFO read clock
                .FIFO_RD_EN(1'b0),                 // 1-bit input: Enables reading the FIFO when asserted
                .RST(ddr_reset)                    // 1-bit input: Asynchronous Reset
            );

            for (j = 0; j < 4; j = j + 1) begin
                //assign ddr_data_deser[i + j*DATA_WIDTH] = bit_deser[3 - j];
                assign ddr_data_deser[i + j*DATA_WIDTH] = bit_deser[j];
            end
        end

        wire [3:0] frame_bit_deser;

        (* keep = "TRUE" *)
        ISERDESE3 #(
            .DATA_WIDTH(4),                 // Parallel data width (4,8)
            .FIFO_ENABLE("FALSE"),          // Enables the use of the FIFO
            .FIFO_SYNC_MODE("FALSE"),       // Always set to FALSE. TRUE is reserved for later use.
            .IS_CLK_B_INVERTED(1'b1),       // Optional inversion for CLK_B
            .IS_CLK_INVERTED(1'b0),         // Optional inversion for CLK
            .IS_RST_INVERTED(1'b0),         // Optional inversion for RST
            .SIM_DEVICE("ULTRASCALE_PLUS")
        ) iserdese3_frame_inst (
            .FIFO_EMPTY(),                     // 1-bit output: FIFO empty flag
            .INTERNAL_DIVCLK(),
            .Q(frame_bit_deser),               // 8-bit registered output
            .CLK(ddr_clk_io),                  // 1-bit input: High-speed clock
            .CLKDIV(ddr_clk_div),              // 1-bit input: Divided Clock
            .CLK_B(ddr_clk_io),                // 1-bit input: Inversion of High-speed clock CLK
            .D(ddr_frame),                     // 1-bit input: Serial Data Input
            .FIFO_RD_CLK(1'b0),                // 1-bit input: FIFO read clock
            .FIFO_RD_EN(1'b0),                 // 1-bit input: Enables reading the FIFO when asserted
            .RST(ddr_reset)                    // 1-bit input: Asynchronous Reset
        );
        //assign ddr_frame_deser = { frame_bit_deser[0], frame_bit_deser[1], frame_bit_deser[2], frame_bit_deser[3] };
        assign ddr_frame_deser = frame_bit_deser;
        assign ddr_data_valid = 1'b1;
    end

    wire [4*DATA_WIDTH-1:0] ddr_data_realigned_chk;
    ddr_realign #(.WIDTH(DATA_WIDTH)) realign (
        .clk(ddr_clk_div),
        .rst(ddr_reset),
        .s_demux_frame(ddr_frame_deser),
        .s_demux_data(ddr_data_deser),
        .s_demux_valid(ddr_data_valid),
        .m_align_data(ddr_data_realigned_chk),
        .m_align_valid(ddr_valid_realigned),
        .cfg_sisoddr(sisoddrmode_clkdiv),
        .cfg_sisoddr_skip(sisoddrmode_clkdiv_skip && !IDDR_STYLE),
        .stat(bitslip_data)
    );

    wire [4*DATA_WIDTH-1:0] ddr_data_deser_swap = {
        ddr_data_realigned_chk[4*DATA_WIDTH-1:3*DATA_WIDTH],  ddr_data_realigned_chk[2*DATA_WIDTH-1:DATA_WIDTH],
        ddr_data_realigned_chk[3*DATA_WIDTH-1:2*DATA_WIDTH],  ddr_data_realigned_chk[DATA_WIDTH-1:0] };

    assign ddr_data_realigned = (sisoddrmode_clkdiv_skip && !IDDR_STYLE) ? ddr_data_deser_swap : ddr_data_realigned_chk;

    assign tune_phase_valid = 1'b0; // No precise phase shift at the moment

end else begin
    // Specific configurations
    wire bitslip_man_clkdiv;
    wire tunephase_man_clkdiv;
    wire reset_bitslip_fsm_clkdiv;


    synchronizer  #(.INIT(0), .ASYNC_RESET(ASYNC_RESETS)) bitslip_man    (.clk(ddr_clk_div), .rst(ddr_reset), .a_in(ctrl_man[1]),   .s_out(bitslip_man_clkdiv));
    synchronizer  #(.INIT(0), .ASYNC_RESET(ASYNC_RESETS)) tunephase_man  (.clk(ddr_clk_div), .rst(ddr_reset), .a_in(ctrl_man[0]),   .s_out(tunephase_man_clkdiv));
    synchronizer  #(.INIT(1), .ASYNC_RESET(ASYNC_RESETS)) bitslip_reset  (.clk(ddr_clk_div), .rst(ddr_reset), .a_in(ctrl_deser[1]), .s_out(reset_bitslip_fsm_clkdiv));


    wire                    ddr_bitslip;

    // Frame + Data Deserializer
    wire                    ddr_frame_delayed;
    wire [3:0]              ddr_frame_deser;
    wire [DATA_WIDTH-1:0]   ddr_data_delayed;

    wire [4*DATA_WIDTH-1:0] ddr_data_deser;

    (* keep = "TRUE" *)
    IDELAYE2 #(
        .CINVCTRL_SEL           ( "FALSE"        ),  // TRUE, FALSE
        .DELAY_SRC              ( "IDATAIN"      ),  // IDATAIN, DATAIN
        .HIGH_PERFORMANCE_MODE  ( "TRUE"         ),  // TRUE, FALSE
        .IDELAY_TYPE            ( "VAR_LOAD"     ),  // FIXED, VARIABLE, or VAR_LOADABLE
        .IDELAY_VALUE           ( 0              ),  // 0 to 31
        .REFCLK_FREQUENCY       ( DELAY_REFCLK   ),
        .PIPE_SEL               ( "FALSE"        ),
        .SIGNAL_PATTERN         ( "CLOCK"        )   // CLOCK, DATA
    ) idelaye2_frameclk (
        .DATAOUT                (  ddr_frame_delayed   ),  // Delayed clock
        .DATAIN                 (  1'b0                ),  // Data from FPGA logic
        .C                      (  cfg_clk             ),
        .CE                     (  1'b0                ),
        .INC                    (  1'b0                ),
        .IDATAIN                (  ddr_frame           ),
        .LD                     (  wr_bank_frmdly && cfg_wvalid && cfg_wwr),
        .LDPIPEEN               (  1'b0                ),
        .REGRST                 (  1'b0                ),
        .CNTVALUEIN             (  cfg_wdata    ),
        .CNTVALUEOUT            (                      ),
        .CINVCTRL               (  1'b0                )
    );

    (* keep = "TRUE" *)
    ISERDESE2 #(
      .DATA_RATE("DDR"),
      .DATA_WIDTH(4),
      .INTERFACE_TYPE("NETWORKING"),
      //.IOBDELAY("NONE")
      .IOBDELAY("BOTH")
    ) iserdese2_lvds_frame (
      .RST(ddr_reset),
      .D(ddr_frame),
      .DDLY(ddr_frame_delayed),
      .Q1(ddr_frame_deser[3]),
      .Q2(ddr_frame_deser[2]),
      .Q3(ddr_frame_deser[1]),
      .Q4(ddr_frame_deser[0]),
      .Q5(),
      .Q6(),
      .Q7(),
      .Q8(),
      .CLK(ddr_clk_io),
      .CLKB(~ddr_clk_io),
      .OCLK(1'b0),
      .OCLKB(1'b0),
      .CLKDIV(ddr_clk_div),
      .CLKDIVP(1'b0),
      .DYNCLKSEL(1'b0),
      .DYNCLKDIVSEL(1'b0),
      .CE1(1'b1),
      .CE2(1'b1),
      .BITSLIP(ddr_bitslip),
      .OFB(1'b0),
      .SHIFTIN1(1'b0),
      .SHIFTIN2(1'b0),
      .SHIFTOUT1(),
      .SHIFTOUT2(),
      .O()
    );

    for (i = 0; i < DATA_WIDTH; i=i+1) begin: dlvds
        wire wr_bank_d_i_strb = (cfg_wbank == (ADDR_BANK_D0DLY + i)) && cfg_wvalid && cfg_wwr;

        (* keep = "TRUE" *)
        IDELAYE2 #(
          .CINVCTRL_SEL           ( "FALSE"        ),  // TRUE, FALSE
          .DELAY_SRC              ( "IDATAIN"      ),  // IDATAIN, DATAIN
          .HIGH_PERFORMANCE_MODE  ( "TRUE"         ),  // TRUE, FALSE
          .IDELAY_TYPE            ( "VAR_LOAD"     ),  // FIXED, VARIABLE, or VAR_LOADABLE
          .IDELAY_VALUE           ( 0              ),  // 0 to 31
          .REFCLK_FREQUENCY       ( DELAY_REFCLK   ),
          .PIPE_SEL               ( "FALSE"        ),
          .SIGNAL_PATTERN         ( "DATA"         )   // CLOCK, DATA
        ) idelaye2_data (
          .DATAOUT                (  ddr_data_delayed[i] ),  // Delayed clock
          .DATAIN                 (  1'b0                ),  // Data from FPGA logic
          .C                      (  cfg_clk             ),
          .CE                     (  1'b0                ),
          .INC                    (  1'b0                ),
          .IDATAIN                (  ddr_data[i]         ),
          .LD                     (  wr_bank_d_i_strb    ),
          .LDPIPEEN               (  1'b0                ),
          .REGRST                 (  1'b0                ),
          .CNTVALUEIN             (  cfg_wdata    ),
          .CNTVALUEOUT            (                      ),
          .CINVCTRL               (  1'b0                )
        );

      (* keep = "TRUE" *)
      ISERDESE2 #(
        .DATA_RATE("DDR"),
        .DATA_WIDTH(4),
        .INTERFACE_TYPE("NETWORKING"),
        .IOBDELAY("BOTH")
      ) iserdese2_lvds_data (
        .RST(ddr_reset),
        .D(ddr_data[i]),
        .DDLY(ddr_data_delayed[i]),
        .Q1(ddr_data_deser[i + 3*DATA_WIDTH]),
        .Q2(ddr_data_deser[i + 2*DATA_WIDTH]),
        .Q3(ddr_data_deser[i + 1*DATA_WIDTH]),
        .Q4(ddr_data_deser[i + 0*DATA_WIDTH]),
        .Q5(),
        .Q6(),
        .Q7(),
        .Q8(),
        .CLK(ddr_clk_io),
        .CLKB(~ddr_clk_io),
        .OCLK(1'b0),
        .OCLKB(1'b0),
        .CLKDIV(ddr_clk_div),
        .CLKDIVP(1'b0),
        .DYNCLKSEL(1'b0),
        .DYNCLKDIVSEL(1'b0),
        .CE1(1'b1),
        .CE2(1'b1),
        .BITSLIP(ddr_bitslip),
        .OFB(1'b0),
        .SHIFTIN1(1'b0),
        .SHIFTIN2(1'b0),
        .SHIFTOUT1(),
        .SHIFTOUT2(),
        .O()
      );
    end

    // Bitslip & Phase calibration -- Frame align
    ddr_bitslip_fsm bitslip (
        .cfg_sisoddr(sisoddrmode_clkdiv),
        .cfg_sisoddr_skp(sisoddrmode_clkdiv_skip),

        .lvds_data_clk_div_buf(ddr_clk_div),
        .lvds_data_clk_reset(ddr_reset),

        .reset_bitslip_fsm(reset_bitslip_fsm_clkdiv),
        .bitslip_man(bitslip_man_clkdiv),
        .tunephase_man(tunephase_man_clkdiv),

        .lvds_frame_div(ddr_frame_deser),
        .lvds_bitslip(ddr_bitslip),

        .lvds_tunephase(tune_phase_valid),
        .lvds_tunephase_ready(tune_phase_ready),

        .lvds_stat(bitslip_data)
    );

    wire [4*DATA_WIDTH-1:0] ddr_data_deser_swap = {
        ddr_data_deser[4*DATA_WIDTH-1:3*DATA_WIDTH],  ddr_data_deser[2*DATA_WIDTH-1:DATA_WIDTH],
        ddr_data_deser[3*DATA_WIDTH-1:2*DATA_WIDTH],  ddr_data_deser[DATA_WIDTH-1:0] };

    assign ddr_data_realigned = sisoddrmode_clkdiv_skip ? ddr_data_deser_swap : ddr_data_deser;
    assign ddr_valid_realigned = 1'b1;
end
endgenerate

`ifdef USE_VIO
reg [4*DATA_WIDTH-1:0] ddr_data_l0;
reg [4*DATA_WIDTH-1:0] ddr_data_l1;
reg [4*DATA_WIDTH-1:0] ddr_data_l2;
reg [4*DATA_WIDTH-1:0] ddr_data_l3;
reg [1:0]              ddr_data_p = 0;

vio_2 vio_2 (
    .clk(l_clk),
    .probe_in0(ddr_data_l0),
    .probe_in1(ddr_data_l1),
    .probe_in2(ddr_data_l2),
    .probe_in3(ddr_data_l3)
);
`endif


// FIFO
assign l_clk = ddr_clk_div_logic;
generate
    if (WITH_FIFO) begin
        xlnx_in_fifo fifo(
            // Async reset to both ports
            .rst(ctrl_deser[2]),

            .a_clk(ddr_clk_div),
            .a_valid(!ddr_reset),
            .a_data(ddr_data_deser),

            .b_clk(l_clk),
            .b_data(l_data),
            .b_ready(1'b1)
        );
    end else begin
        // Phase realign
        reg [4 * DATA_WIDTH - 1:0] buf_data;
        reg                        buf_valid;

        always @(posedge l_clk) begin
            buf_valid <= 1'b0;
            if (!ddr_reset) begin
                buf_data  <= ddr_data_realigned;
                buf_valid <= ddr_valid_realigned;

`ifdef USE_VIO
                case (ddr_data_p)
                0: ddr_data_l0 <= buf_data;
                1: ddr_data_l1 <= buf_data;
                2: ddr_data_l2 <= buf_data;
                3: ddr_data_l3 <= buf_data;
                endcase

                ddr_data_p <= ddr_data_p + 1'b1;
`endif
            end
        end

        assign l_data  = buf_data;
        assign l_valid = buf_valid;
    end

if (HAS_CAPTURE) begin
    data_snapshoter #(.DEEP(CAPTURE_DEEP), .WIDTH(DATA_WIDTH * 4)) capture (
        .clk(l_clk),
        .rst(ddr_reset),

        // 0 - 1 transition start capturing the data, overwritting all previous
        .capture_start(capture_lclk),

        .capture_busy(rb_capture_busy),
        .capture_data(l_data),
        .capture_valid(l_valid),
        .capture_progress(rb_capture_progress),

        // Async interface
        .rb_addr(rb_capture_addr),
        .rb_data(rb_capture_data)
    );
end


if (HAS_DC_ESTIMATOR) begin
    wire [7:0]         count;
    wire [32 * 4 -1:0] dc_meas;

    assign rb_dc_estim_meas =
        rb_dc_estim_w == 4 ? dc_meas[31:0] :
        rb_dc_estim_w == 5 ? dc_meas[63:32] :
        rb_dc_estim_w == 6 ? dc_meas[95:64] :
        rb_dc_estim_w == 7 ? dc_meas[127:96] : count;

    dc_estimator #(.CHANS(4), .IN_WIDTH(DATA_WIDTH), .OUT_WIDTH(32), .CNT_WIDTH(8)) dc_est (
        .clk(l_clk),
        .rst(dc_est_rst_lclk),

        .in_data(l_data),
        .in_valid(l_valid),
        .in_acc(ctrl_dc_est_acc),

        .out_data(dc_meas),
        .out_cnt(count)
    );
end

endgenerate

endmodule
