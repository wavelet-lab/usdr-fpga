// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module axis_fifo #(
	parameter         WIDTH = 32,
	parameter         DEEP = 32,
	parameter         DEEP_BITS = (DEEP <= 16) ? 4 : (DEEP <= 32) ? 5 : (DEEP <= 64) ? 6 : 7,
	parameter         IN_WIDTH = WIDTH,
	parameter   [0:0] EXTRA_REG = 1'b1,
	parameter   [0:0] FIFO_USED_EXACT = 1'b1,
	parameter   [7:0] INIT_AVAIL = 0,
	parameter [511:0] INIT_POS0 = 0,
	parameter [511:0] INIT_POS1 = 0,
	parameter [511:0] INIT_POS2 = 0,
	parameter [511:0] INIT_POS3 = 0,
	parameter [511:0] INIT_POS4 = 0,
	parameter [511:0] INIT_POS5 = 0,
	parameter [511:0] INIT_POS6 = 0,
	parameter [511:0] INIT_POS7 = 0,
	parameter [511:0] INIT_POS8 = 0,
	parameter [511:0] INIT_POS9 = 0,
	parameter [511:0] INIT_POS10 = 0,
	parameter [511:0] INIT_POS11 = 0,
	parameter [511:0] INIT_POS12 = 0,
	parameter [511:0] INIT_POS13 = 0,
	parameter [511:0] INIT_POS14 = 0,
	parameter [511:0] INIT_POS15 = 0,
	parameter [511:0] INIT_POS16 = 0,
	parameter [511:0] INIT_POS17 = 0,
	parameter [511:0] INIT_POS18 = 0,
	parameter [511:0] INIT_POS19 = 0,
	parameter [511:0] INIT_POS20 = 0,
	parameter [511:0] INIT_POS21 = 0,
	parameter [511:0] INIT_POS22 = 0,
	parameter [511:0] INIT_POS23 = 0,
	parameter [511:0] INIT_POS24 = 0,
	parameter [511:0] INIT_POS25 = 0,
	parameter [511:0] INIT_POS26 = 0,
	parameter [511:0] INIT_POS27 = 0,
	parameter [511:0] INIT_POS28 = 0,
	parameter [511:0] INIT_POS29 = 0,
	parameter [511:0] INIT_POS30 = 0,
	parameter [511:0] INIT_POS31 = 0,
	parameter [511:0] INIT_POS32 = 0,
	parameter [511:0] INIT_POS33 = 0,
	parameter [511:0] INIT_POS34 = 0,
	parameter [511:0] INIT_POS35 = 0,
	parameter [511:0] INIT_POS36 = 0,
	parameter [511:0] INIT_POS37 = 0,
	parameter [511:0] INIT_POS38 = 0,
	parameter [511:0] INIT_POS39 = 0,
	parameter [511:0] INIT_POS40 = 0,
	parameter [511:0] INIT_POS41 = 0,
	parameter [511:0] INIT_POS42 = 0,
	parameter [511:0] INIT_POS43 = 0,
	parameter [511:0] INIT_POS44 = 0,
	parameter [511:0] INIT_POS45 = 0,
	parameter [511:0] INIT_POS46 = 0,
	parameter [511:0] INIT_POS47 = 0,
	parameter [511:0] INIT_POS48 = 0,
	parameter [511:0] INIT_POS49 = 0,
	parameter [511:0] INIT_POS50 = 0,
	parameter [511:0] INIT_POS51 = 0,
	parameter [511:0] INIT_POS52 = 0,
	parameter [511:0] INIT_POS53 = 0,
	parameter [511:0] INIT_POS54 = 0,
	parameter [511:0] INIT_POS55 = 0,
	parameter [511:0] INIT_POS56 = 0,
	parameter [511:0] INIT_POS57 = 0,
	parameter [511:0] INIT_POS58 = 0,
	parameter [511:0] INIT_POS59 = 0,
	parameter [511:0] INIT_POS60 = 0,
	parameter [511:0] INIT_POS61 = 0,
	parameter [511:0] INIT_POS62 = 0,
	parameter [511:0] INIT_POS63 = 0,
	parameter [511:0] INIT_POS64 = 0,
	parameter [511:0] INIT_POS65 = 0,
	parameter [511:0] INIT_POS66 = 0,
	parameter [511:0] INIT_POS67 = 0,
	parameter [511:0] INIT_POS68 = 0,
	parameter [511:0] INIT_POS69 = 0,
	parameter [511:0] INIT_POS70 = 0,
	parameter [511:0] INIT_POS71 = 0,
	parameter [511:0] INIT_POS72 = 0,
	parameter [511:0] INIT_POS73 = 0,
	parameter [511:0] INIT_POS74 = 0,
	parameter [511:0] INIT_POS75 = 0,
	parameter [511:0] INIT_POS76 = 0,
	parameter [511:0] INIT_POS77 = 0,
	parameter [511:0] INIT_POS78 = 0,
	parameter [511:0] INIT_POS79 = 0,
	parameter [511:0] INIT_POS80 = 0,
	parameter [511:0] INIT_POS81 = 0,
	parameter [511:0] INIT_POS82 = 0,
	parameter [511:0] INIT_POS83 = 0,
	parameter [511:0] INIT_POS84 = 0,
	parameter [511:0] INIT_POS85 = 0,
	parameter [511:0] INIT_POS86 = 0,
	parameter [511:0] INIT_POS87 = 0,
	parameter [511:0] INIT_POS88 = 0,
	parameter [511:0] INIT_POS89 = 0,
	parameter [511:0] INIT_POS90 = 0,
	parameter [511:0] INIT_POS91 = 0,
	parameter [511:0] INIT_POS92 = 0,
	parameter [511:0] INIT_POS93 = 0,
	parameter [511:0] INIT_POS94 = 0,
	parameter [511:0] INIT_POS95 = 0,
	parameter [511:0] INIT_POS96 = 0,
	parameter [511:0] INIT_POS97 = 0,
	parameter [511:0] INIT_POS98 = 0,
	parameter [511:0] INIT_POS99 = 0,
	parameter [511:0] INIT_POS100 = 0,
	parameter [511:0] INIT_POS101 = 0,
	parameter [511:0] INIT_POS102 = 0,
	parameter [511:0] INIT_POS103 = 0,
	parameter [511:0] INIT_POS104 = 0,
	parameter [511:0] INIT_POS105 = 0,
	parameter [511:0] INIT_POS106 = 0,
	parameter [511:0] INIT_POS107 = 0,
	parameter [511:0] INIT_POS108 = 0,
	parameter [511:0] INIT_POS109 = 0,
	parameter [511:0] INIT_POS110 = 0,
	parameter [511:0] INIT_POS111 = 0,
	parameter [511:0] INIT_POS112 = 0,
	parameter [511:0] INIT_POS113 = 0,
	parameter [511:0] INIT_POS114 = 0,
	parameter [511:0] INIT_POS115 = 0,
	parameter [511:0] INIT_POS116 = 0,
	parameter [511:0] INIT_POS117 = 0,
	parameter [511:0] INIT_POS118 = 0,
	parameter [511:0] INIT_POS119 = 0,
	parameter [511:0] INIT_POS120 = 0,
	parameter [511:0] INIT_POS121 = 0,
	parameter [511:0] INIT_POS122 = 0,
	parameter [511:0] INIT_POS123 = 0,
	parameter [511:0] INIT_POS124 = 0,
	parameter [511:0] INIT_POS125 = 0,
	parameter [511:0] INIT_POS126 = 0,
	parameter [511:0] INIT_POS127 = 0,
	parameter [511:0] INIT_REG = 0
) (
  input                            clk,
  input                            rst,

  input [WIDTH-1:0]                s_rx_tdata,
  input                            s_rx_tvalid,
  output                           s_rx_tready,

  output [WIDTH-1:0]               m_tx_tdata,
  output                           m_tx_tvalid,
  input                            m_tx_tready,

  output [DEEP_BITS-1+EXTRA_REG:0] fifo_used,
  output                           fifo_full,
  output                           fifo_empty
);

wire [WIDTH-1:0]     srl_data;
wire [WIDTH-1:0]     srl_dataq;
wire                 srl_ce;
wire                 srl_we;
wire [DEEP_BITS-1:0] srl_addr;

srl_ra #(
	.WIDTH(WIDTH),
	.DEEP(DEEP),
	.DEEP_BITS(DEEP_BITS),
    .INIT_POS0(INIT_POS0),
    .INIT_POS1(INIT_POS1),
    .INIT_POS2(INIT_POS2),
    .INIT_POS3(INIT_POS3),
    .INIT_POS4(INIT_POS4),
    .INIT_POS5(INIT_POS5),
    .INIT_POS6(INIT_POS6),
    .INIT_POS7(INIT_POS7),
    .INIT_POS8(INIT_POS8),
    .INIT_POS9(INIT_POS9),
    .INIT_POS10(INIT_POS10),
    .INIT_POS11(INIT_POS11),
    .INIT_POS12(INIT_POS12),
    .INIT_POS13(INIT_POS13),
    .INIT_POS14(INIT_POS14),
    .INIT_POS15(INIT_POS15),
    .INIT_POS16(INIT_POS16),
    .INIT_POS17(INIT_POS17),
    .INIT_POS18(INIT_POS18),
    .INIT_POS19(INIT_POS19),
    .INIT_POS20(INIT_POS20),
    .INIT_POS21(INIT_POS21),
    .INIT_POS22(INIT_POS22),
    .INIT_POS23(INIT_POS23),
    .INIT_POS24(INIT_POS24),
    .INIT_POS25(INIT_POS25),
    .INIT_POS26(INIT_POS26),
    .INIT_POS27(INIT_POS27),
    .INIT_POS28(INIT_POS28),
    .INIT_POS29(INIT_POS29),
    .INIT_POS30(INIT_POS30),
    .INIT_POS31(INIT_POS31),
    .INIT_POS32(INIT_POS32),
    .INIT_POS33(INIT_POS33),
    .INIT_POS34(INIT_POS34),
    .INIT_POS35(INIT_POS35),
    .INIT_POS36(INIT_POS36),
    .INIT_POS37(INIT_POS37),
    .INIT_POS38(INIT_POS38),
    .INIT_POS39(INIT_POS39),
    .INIT_POS40(INIT_POS40),
    .INIT_POS41(INIT_POS41),
    .INIT_POS42(INIT_POS42),
    .INIT_POS43(INIT_POS43),
    .INIT_POS44(INIT_POS44),
    .INIT_POS45(INIT_POS45),
    .INIT_POS46(INIT_POS46),
    .INIT_POS47(INIT_POS47),
    .INIT_POS48(INIT_POS48),
    .INIT_POS49(INIT_POS49),
    .INIT_POS50(INIT_POS50),
    .INIT_POS51(INIT_POS51),
    .INIT_POS52(INIT_POS52),
    .INIT_POS53(INIT_POS53),
    .INIT_POS54(INIT_POS54),
    .INIT_POS55(INIT_POS55),
    .INIT_POS56(INIT_POS56),
    .INIT_POS57(INIT_POS57),
    .INIT_POS58(INIT_POS58),
    .INIT_POS59(INIT_POS59),
    .INIT_POS60(INIT_POS60),
    .INIT_POS61(INIT_POS61),
    .INIT_POS62(INIT_POS62),
    .INIT_POS63(INIT_POS63),
    .INIT_POS64(INIT_POS64),
    .INIT_POS65(INIT_POS65),
    .INIT_POS66(INIT_POS66),
    .INIT_POS67(INIT_POS67),
    .INIT_POS68(INIT_POS68),
    .INIT_POS69(INIT_POS69),
    .INIT_POS70(INIT_POS70),
    .INIT_POS71(INIT_POS71),
    .INIT_POS72(INIT_POS72),
    .INIT_POS73(INIT_POS73),
    .INIT_POS74(INIT_POS74),
    .INIT_POS75(INIT_POS75),
    .INIT_POS76(INIT_POS76),
    .INIT_POS77(INIT_POS77),
    .INIT_POS78(INIT_POS78),
    .INIT_POS79(INIT_POS79),
    .INIT_POS80(INIT_POS80),
    .INIT_POS81(INIT_POS81),
    .INIT_POS82(INIT_POS82),
    .INIT_POS83(INIT_POS83),
    .INIT_POS84(INIT_POS84),
    .INIT_POS85(INIT_POS85),
    .INIT_POS86(INIT_POS86),
    .INIT_POS87(INIT_POS87),
    .INIT_POS88(INIT_POS88),
    .INIT_POS89(INIT_POS89),
    .INIT_POS90(INIT_POS90),
    .INIT_POS91(INIT_POS91),
    .INIT_POS92(INIT_POS92),
    .INIT_POS93(INIT_POS93),
    .INIT_POS94(INIT_POS94),
    .INIT_POS95(INIT_POS95),
    .INIT_POS96(INIT_POS96),
    .INIT_POS97(INIT_POS97),
    .INIT_POS98(INIT_POS98),
    .INIT_POS99(INIT_POS99),
    .INIT_POS100(INIT_POS100),
    .INIT_POS101(INIT_POS101),
    .INIT_POS102(INIT_POS102),
    .INIT_POS103(INIT_POS103),
    .INIT_POS104(INIT_POS104),
    .INIT_POS105(INIT_POS105),
    .INIT_POS106(INIT_POS106),
    .INIT_POS107(INIT_POS107),
    .INIT_POS108(INIT_POS108),
    .INIT_POS109(INIT_POS109),
    .INIT_POS110(INIT_POS110),
    .INIT_POS111(INIT_POS111),
    .INIT_POS112(INIT_POS112),
    .INIT_POS113(INIT_POS113),
    .INIT_POS114(INIT_POS114),
    .INIT_POS115(INIT_POS115),
    .INIT_POS116(INIT_POS116),
    .INIT_POS117(INIT_POS117),
    .INIT_POS118(INIT_POS118),
    .INIT_POS119(INIT_POS119),
    .INIT_POS120(INIT_POS120),
    .INIT_POS121(INIT_POS121),
    .INIT_POS122(INIT_POS122),
    .INIT_POS123(INIT_POS123),
    .INIT_POS124(INIT_POS124),
    .INIT_POS125(INIT_POS125),
    .INIT_POS126(INIT_POS126),
    .INIT_POS127(INIT_POS127),
	.INIT_REG(INIT_REG)
) srl (
    .clk(clk),
    .we(srl_we),
    .data_i(s_rx_tdata),

    .addr_i(srl_addr),
    .data_o(srl_data),
    .ce(srl_ce),
    .rstq(rst),
    .dataq_o(srl_dataq)
);

assign m_tx_tdata = (EXTRA_REG) ? srl_dataq : srl_data;

localparam FIFO_FULL       = (EXTRA_REG) ? DEEP : DEEP - 1;
localparam FIFO_INIT_ADDR  = (EXTRA_REG) ? ((INIT_AVAIL > 2) ? INIT_AVAIL - 2 : 0) : ((INIT_AVAIL > 1) ? INIT_AVAIL - 1 : 0);
localparam FIFO_INIT_FULL  = (INIT_AVAIL == FIFO_FULL + 1);

reg [DEEP_BITS-1:0] addr   = FIFO_INIT_ADDR;
reg                 full   = FIFO_INIT_FULL;
reg                 empty  = (INIT_AVAIL > (EXTRA_REG ? 1 : 0)) ? 1'b0 : 1'b1;
reg                 rvalid = (INIT_AVAIL > 0) ? 1'b1 : 1'b0;

wire fifo_wr_strobe = s_rx_tvalid && s_rx_tready;
wire fifo_rd_strobe = (EXTRA_REG) ? ~empty && (~rvalid || m_tx_tready) : (m_tx_tvalid && m_tx_tready);

assign srl_addr     = addr;
assign srl_ce       = (EXTRA_REG) ? fifo_rd_strobe : 1'b0;
assign srl_we       = fifo_wr_strobe;

assign fifo_used    = (EXTRA_REG && FIFO_USED_EXACT) ? srl_addr + rvalid : srl_addr;
assign fifo_full    = full;
assign fifo_empty   = (EXTRA_REG) ? ~rvalid && empty : empty;

assign m_tx_tvalid  = (EXTRA_REG) ? rvalid : ~empty;
assign s_rx_tready  = ~fifo_full;


always @(posedge clk) begin
  if (rst) begin
    empty  <= 1'b1;
    full   <= 1'b0;
    addr   <= 0;
    rvalid <= 1'b0;
  end else begin
    if (EXTRA_REG) begin
       rvalid <= ~empty || rvalid && ~m_tx_tready;
    end

    if (fifo_wr_strobe && fifo_rd_strobe) begin
    end else if (fifo_wr_strobe) begin
      full  <= (addr == DEEP - 2);
      empty <= 1'b0;
      addr  <= addr + !empty;
    end else if (fifo_rd_strobe) begin
      full  <= 1'b0;
      empty <= (addr == 0);
      addr  <= addr - (addr != 0);
    end
  end
end

endmodule


