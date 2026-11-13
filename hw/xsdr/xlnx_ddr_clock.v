module xlnx_ddr_clock #(
    parameter DELAY_REFCLK = 200,
    parameter NO_CLK_DELAY = 0,
    parameter [0:0] VIO_CONTROL = 1'b0
)(
    input  ref_clk,
    
    output lvds_data_clk_div,
    output lvds_in_bufio,
    
    // Control
    input         lvds_delay_ld,
    input [4:0]   lvds_delay_data,
    output [7:0]  clk_data_l
);

assign ilvds_fclk = ref_clk;

wire ilvds_fclk_delayed;
wire internal_clk_dly;
wire internal_clk = (NO_CLK_DELAY) ? ilvds_fclk : internal_clk_dly;

BUFR #(.BUFR_DIVIDE(2)) mrcc_div2(
  .I(internal_clk),
  .O(lvds_data_clk_div),
  .CE(1'b1),
  .CLR(1'b0) //olvds_reset
);
BUFIO in_bufio(.I(internal_clk), .O(lvds_in_bufio));


wire [4:0] lvds_delay_data_o_vio;
wire       lvds_delay_ld_vio;
wire [4:0] lvds_delay_data_vio;   

generate
if (VIO_CONTROL) begin
vio_1 vio_1(.clk(lvds_data_clk_div),
    .probe_in0(clk_data_l),
    .probe_in1(lvds_delay_data_o_vio),
    .probe_out0(lvds_delay_ld_vio),
    .probe_out1(lvds_delay_data_vio)
);
end else begin
    assign lvds_delay_ld_vio = 1'b0;
    assign lvds_delay_data_vio = 5'h0;
end
endgenerate

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
) idelaye2_clk (
  .DATAOUT                (  ilvds_fclk_delayed  ),  // Delayed clock
  .DATAIN                 (  1'b0                ),  // Data from FPGA logic
  .C                      (  lvds_data_clk_div   ),
  .CE                     (  1'b0                ),
  .INC                    (  1'b0                ),
  .IDATAIN                (  ilvds_fclk          ),
  .LD                     (  lvds_delay_ld  | lvds_delay_ld_vio     ),
  .LDPIPEEN               (  1'b0                ),
  .REGRST                 (  1'b0                ),
  .CNTVALUEIN             (  lvds_delay_ld_vio ? lvds_delay_data_vio : lvds_delay_data     ),
  .CNTVALUEOUT            (  lvds_delay_data_o_vio ),
  .CINVCTRL               (  1'b0                )
);

// "IBUF": O => DDLY | Q1-Q6 => D

(* keep = "TRUE" *)
ISERDESE2 #(
      .DATA_RATE("DDR"),
      .DATA_WIDTH(4),
      .INTERFACE_TYPE("NETWORKING"),
      .IOBDELAY(NO_CLK_DELAY ? "NONE" : "IBUF")
) iserdese2_clk (
      .RST(1'b0),
      .D(ilvds_fclk),
      .DDLY(ilvds_fclk_delayed),
      .Q1(clk_data_l[7]),
      .Q2(clk_data_l[6]),
      .Q3(clk_data_l[5]),
      .Q4(clk_data_l[4]),
      .Q5(clk_data_l[3]),
      .Q6(clk_data_l[2]),
      .Q7(clk_data_l[1]),
      .Q8(clk_data_l[0]),
      .CLK(lvds_in_bufio),
      .CLKB(~lvds_in_bufio),
      .OCLK(1'b0),
      .OCLKB(1'b0),
      .CLKDIV(lvds_data_clk_div),
      .CLKDIVP(1'b0),
      .DYNCLKSEL(1'b0),
      .DYNCLKDIVSEL(1'b0),
      .CE1(1'b1),
      .CE2(1'b1),
      .BITSLIP(1'b0),
      .OFB(1'b0),
      .SHIFTIN1(1'b0),
      .SHIFTIN2(1'b0),
      .SHIFTOUT1(),
      .SHIFTOUT2(),
      .O(internal_clk_dly)
);



endmodule
