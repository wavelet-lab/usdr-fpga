// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
// SRL with random access and cyclic load for ROM-like applications
// supported FIFO DEEP
// 16, 32, 64, 96, 128
// maximum width 512 bits
module srl_ra #(
	parameter         WIDTH = 32,
	parameter         DEEP = 32,
	parameter         DEEP_BITS = (DEEP < 32) ? 4 : (DEEP < 64) ? 5 : (DEEP < 128) ? 6 : 7,
	parameter         IN_WIDTH = WIDTH,
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
	parameter [511:0] INIT_REG = 0,
	parameter [511:0] RST_REG = 0
)(
	input                    clk,
	input                    we,
	input [IN_WIDTH - 1:0]   data_i,

	input [DEEP_BITS - 1:0]  addr_i,
	output [WIDTH - 1:0]     data_o,
	input                    ce,
	input                    rstq,
	output [WIDTH - 1:0]     dataq_o,
	
	output [WIDTH - 1:0]     datasrl_o
);

wire [WIDTH - 1:0] srlout_0;
wire [WIDTH - 1:0] srlout_1;
wire [WIDTH - 1:0] srlout_2;
wire [WIDTH - 1:0] srlout_3;

wire [WIDTH - 1:0] srldata_0;
wire [WIDTH - 1:0] srldata_1;
wire [WIDTH - 1:0] srldata_2;
wire [WIDTH - 1:0] srldata_3;

wire [WIDTH - 1:0] last_out = (DEEP <= 32) ? srlout_0 :
                              (DEEP <= 64) ? srlout_1 :
  		                      (DEEP <= 96) ? srlout_2 : srlout_3;
assign datasrl_o = last_out;

wire [WIDTH - 1:0] data_i_cyc;
generate
//assign data_i_cyc = (IN_WIDTH < WIDTH) ? { last_out[WIDTH - IN_WIDTH - 1:0], data_i } : data_i;
if (IN_WIDTH < WIDTH) begin
	assign data_i_cyc =  { last_out[WIDTH - IN_WIDTH - 1:0], data_i };
end else begin
	assign data_i_cyc =  data_i;
end

endgenerate

reg [WIDTH - 1:0] dataq_o_r = INIT_REG;
assign dataq_o = dataq_o_r;

always @(posedge clk) begin
	if (rstq) begin
		dataq_o_r <= RST_REG[WIDTH - 1:0];
	end else if (ce) begin
		dataq_o_r <= data_o;
	end
end


genvar i;
generate

if (DEEP_BITS == 4) begin: gen_bits_4
  for (i = 0; i < WIDTH; i=i+1) begin : srl16
   SRL16E #(
     .INIT({INIT_POS15[i], INIT_POS14[i], INIT_POS13[i], INIT_POS12[i], INIT_POS11[i], INIT_POS10[i], INIT_POS9[i], INIT_POS8[i],
            INIT_POS7[i], INIT_POS6[i], INIT_POS5[i], INIT_POS4[i], INIT_POS3[i], INIT_POS2[i], INIT_POS1[i], INIT_POS0[i]})
   ) fifo16 (
        .CLK(clk),
        .CE(we),
        .D(data_i[i]),
        .A0(addr_i[0]),
        .A1(addr_i[1]),
        .A2(addr_i[2]),
        .A3(addr_i[3]),
        .Q(srldata_0[i])
   );
  end
end 

for (i = 0; i < WIDTH; i=i+1) begin : srl32
  if (DEEP > 16) begin
    SRLC32E #(
     .INIT({INIT_POS31[i], INIT_POS30[i], INIT_POS29[i], INIT_POS28[i], INIT_POS27[i], INIT_POS26[i], INIT_POS25[i], INIT_POS24[i],
            INIT_POS23[i], INIT_POS22[i], INIT_POS21[i], INIT_POS20[i], INIT_POS19[i], INIT_POS18[i], INIT_POS17[i], INIT_POS16[i],
            INIT_POS15[i], INIT_POS14[i], INIT_POS13[i], INIT_POS12[i], INIT_POS11[i], INIT_POS10[i], INIT_POS9[i], INIT_POS8[i],
            INIT_POS7[i], INIT_POS6[i], INIT_POS5[i], INIT_POS4[i], INIT_POS3[i], INIT_POS2[i], INIT_POS1[i], INIT_POS0[i]})
    ) fifo32_0 (
        .CLK(clk),
        .CE(we),
        .D(data_i_cyc[i]),
        .A(addr_i[4:0]),
        .Q(srldata_0[i]),
        .Q31(srlout_0[i])
    );
  end
  
  if (DEEP > 32) begin
    SRLC32E #(
     .INIT({INIT_POS63[i], INIT_POS62[i], INIT_POS61[i], INIT_POS60[i], INIT_POS59[i], INIT_POS58[i], INIT_POS57[i], INIT_POS56[i],
            INIT_POS55[i], INIT_POS54[i], INIT_POS53[i], INIT_POS52[i], INIT_POS51[i], INIT_POS50[i], INIT_POS49[i], INIT_POS48[i],
            INIT_POS47[i], INIT_POS46[i], INIT_POS45[i], INIT_POS44[i], INIT_POS43[i], INIT_POS42[i], INIT_POS41[i], INIT_POS40[i],
            INIT_POS39[i], INIT_POS38[i], INIT_POS37[i], INIT_POS36[i], INIT_POS35[i], INIT_POS34[i], INIT_POS33[i], INIT_POS32[i]})
    ) fifo32_1 (
        .CLK(clk),
        .CE(we),
        .D(data_i_cyc[i]),
        .A(addr_i[4:0]),
        .Q(srldata_1[i]),
        .Q31(srlout_1[i])
    );
  end
 
  if (DEEP > 64) begin
    SRLC32E #(
     .INIT({INIT_POS95[i], INIT_POS94[i], INIT_POS93[i], INIT_POS92[i], INIT_POS91[i], INIT_POS90[i], INIT_POS89[i], INIT_POS88[i],
            INIT_POS87[i], INIT_POS86[i], INIT_POS85[i], INIT_POS84[i], INIT_POS83[i], INIT_POS82[i], INIT_POS81[i], INIT_POS80[i],
            INIT_POS79[i], INIT_POS78[i], INIT_POS77[i], INIT_POS76[i], INIT_POS75[i], INIT_POS74[i], INIT_POS73[i], INIT_POS72[i],
            INIT_POS71[i], INIT_POS70[i], INIT_POS69[i], INIT_POS68[i], INIT_POS67[i], INIT_POS66[i], INIT_POS65[i], INIT_POS64[i]})
    ) fifo32_2 (
        .CLK(clk),
        .CE(we),
        .D(data_i_cyc[i]),
        .A(addr_i[4:0]),
        .Q(srldata_2[i]),
        .Q31(srlout_2[i])
    );
  end

  if (DEEP > 96) begin
    SRLC32E #(
     .INIT({INIT_POS127[i], INIT_POS126[i], INIT_POS125[i], INIT_POS124[i], INIT_POS123[i], INIT_POS122[i], INIT_POS121[i], INIT_POS120[i],
            INIT_POS119[i], INIT_POS118[i], INIT_POS117[i], INIT_POS116[i], INIT_POS115[i], INIT_POS114[i], INIT_POS113[i], INIT_POS112[i],
            INIT_POS111[i], INIT_POS110[i], INIT_POS109[i], INIT_POS108[i], INIT_POS107[i], INIT_POS106[i], INIT_POS105[i], INIT_POS104[i],
            INIT_POS103[i], INIT_POS102[i], INIT_POS101[i], INIT_POS100[i], INIT_POS99[i], INIT_POS98[i], INIT_POS97[i], INIT_POS96[i]})
    ) fifo32_3 (
        .CLK(clk),
        .CE(we),
        .D(data_i_cyc[i]),
        .A(addr_i[4:0]),
        .Q(srldata_3[i]),
        .Q31(srlout_3[i])
    );
  end
end 

endgenerate


assign data_o = (DEEP <= 32) ? srldata_0 :
                (DEEP <= 64) ? ( addr_i[5] == 0 ? srldata_0 : srldata_1 ) :
  				(DEEP <= 96) ? ( addr_i[6:5] == 0 ? srldata_0 : addr_i[6:5] == 1 ? srldata_1 : srldata_2 ) :
  				               ( addr_i[6:5] == 0 ? srldata_0 : addr_i[6:5] == 1 ? srldata_1 :  addr_i[6:5] == 2 ? srldata_2 : srldata_3 );


  
endmodule

