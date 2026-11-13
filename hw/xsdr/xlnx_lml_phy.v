module xlnx_lml_phy #(
  parameter [0:0] MODE_MMCM     = 1'b1,
  parameter [0:0] OSERDESE2_CLK = 1'b1,
  parameter [0:0] LMLE_CLK      = 1'b1,
  parameter [0:0] ALT_PS        = 1'b0,
  parameter [0:0] ULTRA_SCALE   = 1'b0,
  parameter [0:0] ODDR_STYLE    = 1'b0,
  parameter [0:0] USE_SPLIT_PAHSE = 1'b0,
  parameter [0:0] NO_BUFG_LMLE    = 1'b0,
  parameter [0:0] DEBUG         = 1'b0,
  parameter       IQAB_GEN_DEEP = 6
)(
  // LMS7 LML port
  output        lms_i_txnrx,
  inout         lms_io_iqsel,
  input         lms_o_mclk,
  output        lms_i_fclk,
  inout [11:0]  lms_io_diq,

  // Control
  input         phy_cfg_clk,
  input         phy_cfg_rst,
  input [7:0]   phy_cfg_addr,
  input [23:0]  phy_cfg_data,
  input         phy_cfg_strobe,
  output [31:0] phy_cfg_status,
  output        phy_cfg_busy,

  // external
  input         lmle_lms_o_mclk,
  output        lmle_bufio,
  //output        lmle_bufio_b,
  output        lmle_bufr,
  output        lmle_fclk,
  output        lmle_usrclk,

  input         lmle_psen,
  input         lmle_psclk,
  output        lmle_psdone,

  output reg    lfsr_gen_en,
  output reg    iqalgn_gen_en,
  output [47:0]              iqalgn_gen_push_data,
  input [IQAB_GEN_DEEP-1:0]  iqalgn_gen_push_addr,

  // DATA Out
  output        tx_data_clk,
  input         tx_data_valid,
  output        tx_data_ready,
  input [11:0]  tx_data_s0,
  input [11:0]  tx_data_s1,
  input [11:0]  tx_data_s2,
  input [11:0]  tx_data_s3,

  output        txd_rst_gclk
);


localparam [3:0]
    PHY_REG_PORT_CTRL    = 0,
    PHY_REG_MMCM_DRP     = 1,
    PHY_REG_MMCM_CTRL    = 2,
    PHY_REG_MMCM_PSINC   = 3,
    PHY_REG_DLY_VALUE    = 4,
    PHY_REG_PORT_IQSEL   = 5,
    PHY_REG_LFSR_GEN     = 6,
    PHY_REG_IQAB_GEN     = 7,
    PHY_REG_IQAB_PSHA    = 8,
    PHY_REG_IQAB_PSHB    = 9;


// PHY configuration port
reg [15:0] drp_rdata;
reg        drp_busy;
reg [3:0]  cfg_port_iqsel = 4'b1100;

  wire         cfg_mmcm_drp_dclk  = phy_cfg_clk;
  wire [15:0]  cfg_mmcm_drp_di    = phy_cfg_data[15:0];
  wire [6:0]   cfg_mmcm_drp_daddr = phy_cfg_data[22:16];
  wire         cfg_mmcm_drp_dwe   = phy_cfg_data[23];
  wire         cfg_mmcm_drp_den   = (phy_cfg_addr == PHY_REG_MMCM_DRP) && phy_cfg_strobe;

  wire [15:0]  cfg_mmcm_drp_do;
  wire         cfg_mmcm_drp_drdy;

  reg          cfg_mmcm_drp_gpio_out_pd;
  reg          cfg_mmcm_drp_gpio_out_rst;
  reg          cfg_mmcm_drp_gpio_out_insel;

  reg          cfg_port_enable;
  reg          cfg_sisosdr_skip;
  wire         cfg_rx_idelay_clk   = phy_cfg_clk;
  wire [4:0]   cfg_rx_idelay_data  = phy_cfg_data[4:0];
  wire         cfg_rx_idelay_valid = (phy_cfg_addr == PHY_REG_DLY_VALUE);

  wire         cfg_mmcm_drp_gpio_fbstopped;
  wire         cfg_mmcm_drp_gpio_instopped;

wire mmcm_ready;

reg  mmcm_psbusy;
wire mmcm_psdone;
wire mmcm_psclk = phy_cfg_clk;
wire mmcm_psen  = (phy_cfg_addr == PHY_REG_MMCM_PSINC);

assign phy_cfg_status[31:16] = drp_rdata;
assign phy_cfg_status[15:8]  = { MODE_MMCM, 2'b00, mmcm_psbusy, cfg_mmcm_drp_gpio_fbstopped, cfg_mmcm_drp_gpio_instopped, drp_busy, mmcm_ready};
assign phy_cfg_status[7:0]   = 0;

assign phy_cfg_busy          = (phy_cfg_addr == PHY_REG_MMCM_DRP) ? drp_busy : (phy_cfg_addr == PHY_REG_MMCM_PSINC) ? mmcm_psbusy : 1'b0;

reg [47:0]               iqalgn_fifo_push_data;
reg [IQAB_GEN_DEEP-1:0]  iqalgn_fifo_push_addr;
reg                      iqalgn_fifo_push_we;

always @(posedge phy_cfg_clk) begin
    if (phy_cfg_rst) begin
        drp_busy                    <= 1'b0;
        mmcm_psbusy                 <= 1'b0;
        cfg_mmcm_drp_gpio_out_pd    <= 1'b1;
        cfg_mmcm_drp_gpio_out_rst   <= 1'b1;
        cfg_port_enable             <= 1'b0;
        lfsr_gen_en                 <= 1'b0;
        iqalgn_gen_en               <= 1'b0;
        cfg_mmcm_drp_gpio_out_insel <= 1'b1;
    end else begin
        iqalgn_fifo_push_we         <= 1'b0;
        
      if (phy_cfg_strobe) begin
        case (phy_cfg_addr)
        PHY_REG_PORT_CTRL: begin
            cfg_port_enable  <= phy_cfg_data[0];
            cfg_sisosdr_skip <= phy_cfg_data[4];
            
            iqalgn_fifo_push_addr <= 0;
        end
        PHY_REG_MMCM_DRP:  drp_busy        <= 1'b1;
        PHY_REG_MMCM_CTRL: begin
            cfg_mmcm_drp_gpio_out_rst   <= phy_cfg_data[0];
            cfg_mmcm_drp_gpio_out_pd    <= phy_cfg_data[1];
            cfg_mmcm_drp_gpio_out_insel <= phy_cfg_data[2];
        end
        PHY_REG_MMCM_PSINC: begin
            mmcm_psbusy <= 1'b1;
        end
        PHY_REG_PORT_IQSEL: cfg_port_iqsel <= phy_cfg_data[3:0];
        PHY_REG_LFSR_GEN:   lfsr_gen_en    <= phy_cfg_data[0];
        PHY_REG_IQAB_GEN:   iqalgn_gen_en <=  phy_cfg_data[0];
        
        PHY_REG_IQAB_PSHA:  iqalgn_fifo_push_data[23:0]  <= phy_cfg_data[23:0];
        PHY_REG_IQAB_PSHB:  begin
            iqalgn_fifo_push_data[47:24] <= phy_cfg_data[23:0];
            iqalgn_fifo_push_addr        <= iqalgn_fifo_push_addr + 1'b1;
            iqalgn_fifo_push_we          <= 1'b1;
        end
        endcase
      end

      if (MODE_MMCM && cfg_mmcm_drp_drdy) begin
        drp_rdata <= cfg_mmcm_drp_do;
        drp_busy  <= 1'b0;
      end

      if (MODE_MMCM && mmcm_psdone) begin
        mmcm_psbusy <= 1'b0;
      end
    end
end
assign txd_rst_gclk = ~cfg_port_enable;

// 64-point continous generator
ram_sxp #(
    .DATA_WIDTH(48),
    .ADDR_WIDTH(IQAB_GEN_DEEP),
    .ULTRA_SCALE(ULTRA_SCALE),
    .MODE_SDP(1)
) iqalgn_gen_buffer (
   .wclk(phy_cfg_clk),
   .we(iqalgn_fifo_push_we),
   .waddr(iqalgn_fifo_push_addr),
   .wdata(iqalgn_fifo_push_data),

   .raddr(iqalgn_gen_push_addr),
   .rdata(iqalgn_gen_push_data)
);


/////////////////////////////////////////////////////////////
// IO buffers
wire        phy_lml_tmode           = ~cfg_port_enable;
wire [11:0] phy_lml_data_in;
wire [11:0] phy_lml_data_out;
wire [11:0] phy_lml_data_out_tmode;
wire        phy_lml_iqsel_in;
wire        phy_lml_iqsel_out;
wire        phy_lml_iqsel_out_tmode;

wire        phy_lml_fclk_out_tmode;
wire        phy_lml_fclk_out;
wire        phy_lml_txnrx_out       = 1'b1;
wire        phy_lml_mclk_in;
wire        phy_lml_mclk_in2;


genvar i;
genvar j;
generate
  for (j = 0; j < 12; j = j + 1) begin: lml_bufs
    OBUFT phy_data_buf_o(
      .O(lms_io_diq[j]),
      .I(phy_lml_data_out[j]),
      .T(phy_lml_data_out_tmode[j])
    );
  end
endgenerate


OBUFT phy_iqsel_buf_o(
  .O(lms_io_iqsel),
  .I(phy_lml_iqsel_out),
  .T(phy_lml_iqsel_out_tmode)
);

OBUFT phy_fclk_obuf(
  .O(lms_i_fclk),
  .I(phy_lml_fclk_out),
  .T(phy_lml_fclk_out_tmode)
);

//OBUF phy_txnrx_obuf(
//  .O(lms_i_txnrx),
//  .I(phy_lml_txnrx_out)
//);
assign lms_i_txnrx = phy_lml_txnrx_out;

wire lms_o_mclk_bufg;
IBUFG phy_mclk_ibuf(
  .O(lms_o_mclk_bufg),
  .I(lms_o_mclk)
);

generate
if (ULTRA_SCALE) begin
    BUFG lml_bufg (.I(lms_o_mclk_bufg), .O(phy_lml_mclk_in));
end else begin
    assign phy_lml_mclk_in = lms_o_mclk_bufg;
end
endgenerate


generate
if (LMLE_CLK) begin
  wire lmle_lms_o_mclk_buf;

  if (ULTRA_SCALE) begin
    IBUFG phy_mclk_ibuf2(
    .O(lmle_lms_o_mclk_buf),
    .I(lmle_lms_o_mclk)
    );
    assign phy_lml_mclk_in2 = lmle_lms_o_mclk_buf;

  end else begin
    IBUFG phy_mclk_ibuf2(
    .O(lmle_lms_o_mclk_buf),
    .I(lmle_lms_o_mclk)
    );

    assign phy_lml_mclk_in2 = lmle_lms_o_mclk_buf;
  end

end else begin
  assign phy_lml_mclk_in2 = 1'b0;
end
endgenerate

wire [11:0]   phy_lml_data_in_d;
wire          phy_lml_iqsel_in_d;

wire phy_tx_reset_clkdiv;
wire phy_tx_reset_clkdiv_iq;
wire phy_tx_data_clk;
wire phy_tx_data_clk_div;
wire phy_tx_data_clk_iq;
wire phy_tx_data_clk_div_iq;
wire phy_tx_ce = 1'b1;

wire [11:0] b_tx_data_s0 = tx_data_s0;
wire [11:0] b_tx_data_s1 = tx_data_s1;
wire [11:0] b_tx_data_s2 = tx_data_s2;
wire [11:0] b_tx_data_s3 = tx_data_s3;

generate
if (DEBUG) begin
ila_0 ila_0(
    .clk(tx_data_clk),
    .probe0(b_tx_data_s0),
    .probe1(b_tx_data_s1),
    .probe2(b_tx_data_s2),
    .probe3(b_tx_data_s3),
    .probe4(tx_data_ready)
);
end
endgenerate


generate
  if (ULTRA_SCALE && ODDR_STYLE) begin
       // Cross clock 4bit -> 2bit

       wire [4 + 47:0] data_in_wide;
       wire [4 + 47:0] data_out_data;
       wire            data_out_valid;
       reg             data_out_ready;

       wire            data_out_rst;

       reg             data_out_narrow_precharge;
       wire [2 + 23:0] data_out_narrow = (!data_out_ready) ? data_out_data[25:0] : data_out_data[51:26];

       synchronizer #(.INIT(1)) data_out_rst_sync (.clk(phy_tx_data_clk),    .rst(phy_tx_reset_clkdiv),    .a_in(1'b0), .s_out(data_out_rst));

       axis_cc_fifo #(.WIDTH(48 + 4), .DEEP(16), .ULTRA_SCALE(ULTRA_SCALE)) oddr_fifo_i (
            .rx_clk(phy_tx_data_clk_div),
            .rx_rst(phy_tx_reset_clkdiv),

            .s_rx_tdata(data_in_wide),
            .s_rx_tvalid(1'b1),
            .s_rx_tready(),

            .tx_clk(phy_tx_data_clk),
            .tx_rst(data_out_rst),

            .m_tx_tdata(data_out_data),
            .m_tx_tvalid(data_out_valid),
            .m_tx_tready(data_out_ready)
       );

       always @(posedge phy_tx_data_clk) begin
        if (data_out_rst) begin
            data_out_ready            <= 1'b0;
            data_out_narrow_precharge <= 1'b0;
        end else begin
            data_out_narrow_precharge <= data_out_valid;

            if (data_out_narrow_precharge) begin
                data_out_ready <= ~data_out_ready;
            end
        end
       end

       for (i = 0; i < 12; i = i + 1) begin
        assign data_in_wide[0 + i] = b_tx_data_s0[i];
        assign data_in_wide[13 + i] = b_tx_data_s1[i];
        assign data_in_wide[26 + i] = b_tx_data_s2[i];
        assign data_in_wide[39 + i] = b_tx_data_s3[i];

        ODDRE1 data_oddr(.D1(data_out_narrow[i]), .D2(data_out_narrow[13 + i]), .Q(phy_lml_data_out[i]), .C(phy_tx_data_clk), .SR(1'b0));
       end

       assign data_in_wide[12] = cfg_port_iqsel[0];
       assign data_in_wide[25] = cfg_port_iqsel[1];
       assign data_in_wide[38] = cfg_port_iqsel[2];
       assign data_in_wide[51] = cfg_port_iqsel[3];

       ODDRE1 iq_sel_oddr(.D1(data_out_narrow[12]), .D2(data_out_narrow[25]), .Q(phy_lml_iqsel_out), .C(phy_tx_data_clk), .SR(1'b0));

  end else if (ULTRA_SCALE) begin
    for (i = 0; i < 12; i=i+1) begin
        wire [3:0] data_out = { b_tx_data_s3[i], b_tx_data_s2[i], b_tx_data_s1[i], b_tx_data_s0[i] };

        OSERDESE3 #(
           .DATA_WIDTH(4),                 // Parallel Data Width (4-8)
           .INIT(1'b0),                    // Initialization value of the OSERDES flip-flops
           .IS_CLKDIV_INVERTED(1'b0),      // Optional inversion for CLKDIV
           .IS_CLK_INVERTED(1'b0),         // Optional inversion for CLK
           .IS_RST_INVERTED(1'b0),         // Optional inversion for RST
           .SIM_DEVICE("ULTRASCALE_PLUS")  // Set the device version for simulation functionality (ULTRASCALE)
        ) OSERDESE3_inst (
           .OQ(phy_lml_data_out[i]),                // 1-bit output: Serial Output Data
           .T_OUT(phy_lml_data_out_tmode[i]),       // 1-bit output: 3-state control output to IOB
           .CLK(phy_tx_data_clk),                   // 1-bit input: High-speed clock
           .CLKDIV(phy_tx_data_clk_div),            // 1-bit input: Divided Clock
           .D(data_out),                // 8-bit input: Parallel Data Input
           .RST(phy_tx_reset_clkdiv),   // 1-bit input: Asynchronous Reset
           .T(phy_lml_tmode)            // 1-bit input: Tristate input from fabric
        );
    end

    wire [3:0] iqsel_out_data = { cfg_port_iqsel[3], cfg_port_iqsel[2], cfg_port_iqsel[1], cfg_port_iqsel[0] };
    OSERDESE3 #(
       .DATA_WIDTH(4),                 // Parallel Data Width (4-8)
       .INIT(1'b0),                    // Initialization value of the OSERDES flip-flops
       .IS_CLKDIV_INVERTED(1'b0),      // Optional inversion for CLKDIV
       .IS_CLK_INVERTED(1'b0),         // Optional inversion for CLK
       .IS_RST_INVERTED(1'b0),         // Optional inversion for RST
       .SIM_DEVICE("ULTRASCALE_PLUS")  // Set the device version for simulation functionality (ULTRASCALE)
    ) OSERDESE3_inst (
       .OQ(phy_lml_iqsel_out),                // 1-bit output: Serial Output Data
       .T_OUT(phy_lml_iqsel_out_tmode),       // 1-bit output: 3-state control output to IOB
       .CLK(phy_tx_data_clk_iq),              // 1-bit input: High-speed clock
       .CLKDIV(phy_tx_data_clk_div_iq),       // 1-bit input: Divided Clock
       .D(iqsel_out_data),                    // 8-bit input: Parallel Data Input
       .RST(phy_tx_reset_clkdiv_iq),   // 1-bit input: Asynchronous Reset
       .T(phy_lml_tmode)               // 1-bit input: Tristate input from fabric
    );

  end else begin
    reg [11:0] b_tx_data_s0_r;
    reg [11:0] b_tx_data_s1_r;
    reg [11:0] b_tx_data_s2_r;
    reg [11:0] b_tx_data_s3_r;

    reg [3:0]  cfg_port_iqsel_r;

    always @(posedge phy_tx_data_clk_div) begin
        if (cfg_sisosdr_skip) begin
            b_tx_data_s0_r <= b_tx_data_s0;
            b_tx_data_s1_r <= b_tx_data_s0;
            b_tx_data_s2_r <= b_tx_data_s1;
            b_tx_data_s3_r <= b_tx_data_s1;
        end else begin
            b_tx_data_s0_r <= b_tx_data_s0;
            b_tx_data_s1_r <= b_tx_data_s1;
            b_tx_data_s2_r <= b_tx_data_s2;
            b_tx_data_s3_r <= b_tx_data_s3;
        end
    end


    for (i = 0; i < 12; i=i+1) begin
      OSERDESE2 #(
        .DATA_RATE_OQ("DDR"),
        .DATA_RATE_TQ("DDR"),
        .DATA_WIDTH(4)
      ) oserdese2_lms_diq_tx(
        .RST(phy_tx_reset_clkdiv),
        .D1(b_tx_data_s0_r[i]),
        .D2(b_tx_data_s1_r[i]),
        .D3(b_tx_data_s2_r[i]),
        .D4(b_tx_data_s3_r[i]),
        .D5(),
        .D6(),
        .D7(),
        .D8(),
        .CLK(phy_tx_data_clk),
        .CLKDIV(phy_tx_data_clk_div),
        .OCE(phy_tx_ce),
        .T1(phy_lml_tmode),
        .T2(phy_lml_tmode),
        .T3(phy_lml_tmode),
        .T4(phy_lml_tmode),
        .TCE(1'b1),
        .TBYTEIN(),
        .TBYTEOUT(),
        .OQ(phy_lml_data_out[i]),
        .TQ(phy_lml_data_out_tmode[i]),
        .OFB(),
        .TFB(),
        .SHIFTOUT1(),
        .SHIFTOUT2(),
        .SHIFTIN1(),
        .SHIFTIN2()
      );
    end

    OSERDESE2 #(
      .DATA_RATE_OQ("DDR"),
      .DATA_RATE_TQ("DDR"),
      .DATA_WIDTH(4)
    ) oserdese2_lms_iqsel_tx(
      // .RST(phy_tx_reset_clkdiv),
      .RST(phy_tx_reset_clkdiv_iq),
      .D1(cfg_port_iqsel[0]),
      .D2(cfg_port_iqsel[1]),
      .D3(cfg_port_iqsel[2]),
      .D4(cfg_port_iqsel[3]),
      .D5(),
      .D6(),
      .D7(),
      .D8(),
      .CLK(phy_tx_data_clk_iq),
      .CLKDIV(phy_tx_data_clk_div_iq),
      .OCE(phy_tx_ce),
      .T1(phy_lml_tmode),
      .T2(phy_lml_tmode),
      .T3(phy_lml_tmode),
      .T4(phy_lml_tmode),
      .TCE(1'b1),
      .TBYTEIN(),
      .TBYTEOUT(),
      .OQ(phy_lml_iqsel_out),
      .TQ(phy_lml_iqsel_out_tmode),
      .OFB(),
      .TFB(),
      .SHIFTOUT1(),
      .SHIFTOUT2(),
      .SHIFTIN1(),
      .SHIFTIN2()
    );
  end

endgenerate


wire phy_fclk_clk;

generate
  //TODO add for US+
  if (!MODE_MMCM) begin: fclk_phase
    assign phy_fclk_clk = phy_lml_mclk_in;
    wire phy_fclk_clk_delayed;

        (* keep = "TRUE" *)
        IDELAYE2 #(
          .CINVCTRL_SEL           ( "FALSE"        ),  // TRUE, FALSE
          .DELAY_SRC              ( "DATAIN"      ),   // IDATAIN, DATAIN
          .HIGH_PERFORMANCE_MODE  ( "TRUE"         ),  // TRUE, FALSE
          .IDELAY_TYPE            ( "VAR_LOAD"     ),  // FIXED, VARIABLE, or VAR_LOADABLE
          .IDELAY_VALUE           ( 31             ),  // 0 to 31
          .REFCLK_FREQUENCY       ( 200   ),
          .PIPE_SEL               ( "FALSE"        ),
          .SIGNAL_PATTERN         ( "CLOCK"        )   // CLOCK, DATA
        ) idelaye2_clk (
          .DATAOUT                (  phy_fclk_clk_delayed  ),  // Delayed clock
          .DATAIN                 (  phy_fclk_clk          ),  // Data from FPGA logic
          .C                      (  cfg_rx_idelay_clk     ),
          .CE                     (  1'b0                  ),
          .INC                    (  1'b0                  ),
          .IDATAIN                (  1'b0                  ),
          .LD                     (  cfg_rx_idelay_valid   ),
          .LDPIPEEN               (  1'b0                  ),
          .REGRST                 (  1'b0                  ),
          .CNTVALUEIN             (  cfg_rx_idelay_data    ),
          .CNTVALUEOUT            (                        ),
          .CINVCTRL               (  1'b0                  )
        );

    ODDR #(
      .DDR_CLK_EDGE("OPPOSITE_EDGE"),
      .SRTYPE("SYNC"),
      .INIT(0)
    ) oddr_lms_clk_tx (
      .Q(phy_lml_fclk_out),
      .C(phy_fclk_clk_delayed),
      .CE(1'b1),
      .D1(1'b1),
      .D2(1'b0),
      .R(1'b0)
    );

    assign phy_lml_fclk_out_tmode = ~cfg_port_enable;
  end
endgenerate

/////////////////////////////////////////////////////////////
// Clocking
wire mmcm_gen_data_clk;
wire mmcm_gen_data_clk_div;
wire mmcm_gen_data_clk_div_usr;

generate
  if (MODE_MMCM) begin
    assign phy_tx_data_clk     = mmcm_gen_data_clk;
    assign phy_tx_data_clk_div = mmcm_gen_data_clk_div;

    if (ULTRA_SCALE) begin
        assign tx_data_clk = phy_tx_data_clk_div;
    end else begin
        BUFG refbuf(.I(mmcm_gen_data_clk_div), .O(tx_data_clk));
    end
  end else begin
    xlnx_serdes_clk #(.ULTRA_SCALE(ULTRA_SCALE), .DIV(1)) mrcc_div2 (
        .clk(phy_lml_mclk_in),
        .ce(1'b1),
        .clr(1'b0),
        .clk_o(phy_tx_data_clk),
        .clk_div_o(phy_tx_data_clk_div)
    );

    if (ULTRA_SCALE) begin
        assign tx_data_clk = phy_tx_data_clk_div;
    end else begin
        BUFG refbuf(.I(phy_tx_data_clk_div), .O(tx_data_clk));
    end
  end
endgenerate


`define NO_IQ_SHIFT

generate
  if (MODE_MMCM) begin: mmcm_gen
    wire mmcm_gen_fclk;
    wire clkfb_buf;
    wire mmcm_gen_data_clk_io;
    wire mmcm_gen_data_clk_io2;
    wire mmcm_gen_data_clk_div_usr2;
    wire mmcm_gen_fclk2;
    wire mmcm_gen_data_clk_io_iq;

    MMCME2_ADV #(
      .BANDWIDTH("LOW"),    // "HIGH"
      .DIVCLK_DIVIDE(1),    // (1 to 106)

      .CLKFBOUT_MULT_F(2),
      .CLKFBOUT_PHASE(0.0),
      .CLKFBOUT_USE_FINE_PS("FALSE"),

      .CLKIN1_PERIOD(2.5),
      .REF_JITTER1(0.010),

      .CLKIN2_PERIOD(2.5),
      .REF_JITTER2(0.010),

      .CLKOUT0_DIVIDE_F(USE_SPLIT_PAHSE ? 4 : 2),
      .CLKOUT0_DUTY_CYCLE(0.5),
      .CLKOUT0_PHASE(0.0),
      .CLKOUT0_USE_FINE_PS("FALSE"),

      .CLKOUT1_DIVIDE(2),
      .CLKOUT1_DUTY_CYCLE(0.5),
      .CLKOUT1_PHASE(0.0),
      //.CLKOUT1_USE_FINE_PS(ALT_PS ? "TRUE" : "FALSE"),
      .CLKOUT1_USE_FINE_PS("FALSE"),

      .CLKOUT2_DIVIDE(2),
      .CLKOUT2_DUTY_CYCLE(0.5),
      .CLKOUT2_PHASE(0.0),
      .CLKOUT2_USE_FINE_PS("FALSE"),

      .CLKOUT3_DIVIDE(4),
      .CLKOUT3_DUTY_CYCLE(0.5),
      .CLKOUT3_PHASE(0.0),
      .CLKOUT3_USE_FINE_PS("FALSE"),

      .CLKOUT4_DIVIDE(4),
      .CLKOUT4_DUTY_CYCLE(0.5),
      .CLKOUT4_PHASE(0.0),
      .CLKOUT4_USE_FINE_PS("FALSE"),
      .CLKOUT4_CASCADE("FALSE"),

      .CLKOUT5_DIVIDE(2),
      .CLKOUT5_DUTY_CYCLE(0.5),
      .CLKOUT5_PHASE(0.0),
      .CLKOUT5_USE_FINE_PS("FALSE"),

      .CLKOUT6_DIVIDE(2),
      .CLKOUT6_DUTY_CYCLE(0.5),
      .CLKOUT6_PHASE(90.0),
      .CLKOUT6_USE_FINE_PS("FALSE"),

      //.COMPENSATION("INTERNAL"), // "ZHOLD"
      .COMPENSATION("ZHOLD"), // "ZHOLD"
      .STARTUP_WAIT("FALSE")
   ) mmcme2 (
      .CLKFBOUT(clkfb_buf),
      .CLKFBOUTB(),

      .CLKFBSTOPPED(cfg_mmcm_drp_gpio_fbstopped),
      .CLKINSTOPPED(cfg_mmcm_drp_gpio_instopped),

      // Clock outputs
      .CLKOUT0(mmcm_gen_data_clk_io_iq),    // TX IQSEL frame phase
      .CLKOUT0B(),
      .CLKOUT1(mmcm_gen_data_clk_io),       // TX DATA phase
      .CLKOUT1B(),
      .CLKOUT2(mmcm_gen_data_clk_io2),      // RX DATA / IQSEL phase
      .CLKOUT2B(),
      .CLKOUT3(mmcm_gen_data_clk_div_usr),  // Not used
      .CLKOUT3B(),
      .CLKOUT4(mmcm_gen_data_clk_div_usr2), // Not used
      .CLKOUT5(mmcm_gen_fclk2),
      .CLKOUT6(mmcm_gen_fclk),

      // DRP Ports
      .DO(cfg_mmcm_drp_do), // (16-bits)
      .DRDY(cfg_mmcm_drp_drdy),
      .DADDR(cfg_mmcm_drp_daddr), // 5 bits
      .DCLK(cfg_mmcm_drp_dclk),
      .DEN(cfg_mmcm_drp_den),
      .DI(cfg_mmcm_drp_di), // 16 bits
      .DWE(cfg_mmcm_drp_dwe),

      .LOCKED(mmcm_ready),
      .CLKFBIN(clkfb_buf),

      // Clock inputs
      .CLKIN1(phy_lml_mclk_in),
      .CLKIN2(phy_lml_mclk_in2),
      .CLKINSEL(cfg_mmcm_drp_gpio_out_insel /*1'b1*/), //High = CLKIN1, Low = CLKIN2

      // Fine phase shifting
      .PSDONE(ALT_PS ? lmle_psdone : mmcm_psdone),
      .PSCLK(ALT_PS ?  lmle_psclk  : mmcm_psclk),
      .PSEN(ALT_PS ?   lmle_psen   : mmcm_psen),
      .PSINCDEC(1'b0),

      .PWRDWN(cfg_mmcm_drp_gpio_out_pd),
      .RST(cfg_mmcm_drp_gpio_out_rst)
    );

    if (USE_SPLIT_PAHSE) begin
        // In spil phase mode we use CLK and CLKDIV sourcing directly from
        // MMCM, this is not recommended in US+. Remove this feature in futuere
        // releases.
        if (ULTRA_SCALE) begin
            BUFG mrcc_div2(.I(mmcm_gen_data_clk_io_iq), .O(mmcm_gen_data_clk_div));
            BUFG in_bufio (.I(mmcm_gen_data_clk_io),    .O(mmcm_gen_data_clk));
        end else begin
            BUFR #(.BUFR_DIVIDE(1)) mrcc_div2(
            .I(mmcm_gen_data_clk_io_iq),
            .O(mmcm_gen_data_clk_div),
            .CE(1'b1),
            .CLR(1'b0)
            );
            BUFIO in_bufio(.I(mmcm_gen_data_clk_io), .O(mmcm_gen_data_clk));
        end
    end else begin
        xlnx_serdes_clk #(.ULTRA_SCALE(ULTRA_SCALE)) mrcc_div2 (
            .clk(mmcm_gen_data_clk_io),
            .ce(1'b1),
            .clr(1'b0),
            .clk_o(mmcm_gen_data_clk),
            .clk_div_o(mmcm_gen_data_clk_div)
        );
    end

`ifdef NO_IQ_SHIFT
    assign phy_tx_data_clk_div_iq = mmcm_gen_data_clk_div;
    assign phy_tx_data_clk_iq     = mmcm_gen_data_clk;
`else
    xlnx_serdes_clk #(.ULTRA_SCALE(ULTRA_SCALE)) mrcc_div2_iq (
        .clk(mmcm_gen_data_clk_io_iq),
        .ce(1'b1),
        .clr(1'b0),
        .clk_o(phy_tx_data_clk_iq),
        .clk_div_o(phy_tx_data_clk_div_iq)
    );
`endif

    if (LMLE_CLK) begin
        xlnx_serdes_clk #(.ULTRA_SCALE(ULTRA_SCALE)) lmle_clk (
            .clk(mmcm_gen_data_clk_io2),
            .ce(1'b1),
            .clr(1'b0),
            .clk_o(lmle_bufio),
            .clk_div_o(lmle_bufr)
        );

        //assign lmle_usrclk  = lmle_bufr;
        if (ULTRA_SCALE || NO_BUFG_LMLE) begin
            assign lmle_usrclk  = lmle_bufr;
        end else begin
            BUFG refbuf(.I(lmle_bufr), .O(lmle_usrclk));
        end

        assign lmle_fclk    = mmcm_gen_fclk2;
    end

    if (OSERDESE2_CLK) begin
        /*
        wire mmcm_gen_fclk_b;

        BUFG fclk_b(.I(mmcm_gen_fclk), .O(mmcm_gen_fclk_b));

        ODDR #(
          .DDR_CLK_EDGE("OPPOSITE_EDGE"),
          .SRTYPE("SYNC"),
          .INIT(0)
        ) oddr_lms_clk_tx (
          .Q(phy_lml_fclk_out),
          .C(mmcm_gen_fclk_b),
          .CE(1'b1),
          .D1(1'b1),
          .D2(1'b0),
          .R(1'b0)
        );
        */
        assign phy_lml_fclk_out = mmcm_gen_fclk;
        assign phy_lml_fclk_out_tmode = ~cfg_port_enable;
    end else begin
        assign phy_lml_fclk_out = mmcm_gen_fclk;
        assign phy_lml_fclk_out_tmode = ~cfg_port_enable;
    end

  end else begin
    assign cfg_mmcm_drp_gpio_in = 4'b0000;
    assign cfg_mmcm_drp_do = 0;
    assign cfg_mmcm_drp_drdy = 1'b1;
  end
endgenerate



synchronizer #(.INIT(1)) phy_tx_reset_clkdiv_sync    (.clk(phy_tx_data_clk_div),    .rst(~cfg_port_enable),    .a_in(1'b0), .s_out(phy_tx_reset_clkdiv));
`ifdef NO_IQ_SHIFT
assign phy_tx_reset_clkdiv_iq = phy_tx_reset_clkdiv;
`else
synchronizer #(.INIT(1)) phy_tx_reset_clkdiv_iq_sync (.clk(phy_tx_data_clk_div_iq), .rst(phy_tx_reset_clkdiv), .a_in(1'b0), .s_out(phy_tx_reset_clkdiv_iq));
`endif


assign tx_data_ready = ~phy_tx_reset_clkdiv;

endmodule
