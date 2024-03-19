// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module dsp48e1_pipeline #(
	parameter TAG_WIDTH = 1'b1,
	parameter RESET_DSP48 = 1'b0,
	parameter [7:0] ID = 0
)(
	input                  clk,
	input                  rst,
	
	input [30+25+18-1:0]   s_adb_data,
	input                  s_adb_valid,
	input [14:0]           s_adb_cmd,
	input [TAG_WIDTH-1:0]  s_adb_tag,
	
	// Async C update
	input [47:0]           s_c_data,
	input                  s_c_valid,
	
	output [47:0]          m_p_data,
	output                 m_p_valid,
	output [TAG_WIDTH-1:0] m_p_tag,
	
	output [3:0]           m_p_svalid,
	
	// Cascase
	output [47:0]          dspcasc_p_o,
	input  [47:0]          dspcasc_p_i
);
//    0010010_0000_001    P = P + PCIN      0901
//    0100101_0000_010    P = P + (D+A)*B   1282
//    0110101_0000_010    P = C + (D+A)*B   1A82
//    0000101_0000_010    P = 0 + (D+A)*B   0282
//
//    0000101_0000_000    P = 0 + A*B       0280
//
//  0_0001100_0000_001    P = C + 0 + 0     0601
//  000_0110_0000_0001 
//
// inmode:
//                 000 : A
//                 001 : 0
//                 010 : D + A
//                 011 : D - A
// alu:
//            0000 : Z + X + Y + CIN
//            0001 : -Z + X + Y + CIN - 1
//            0010 : -Z - X - Y - CIN - 1
//            0011 : Z - (X + Y + CIN)
// opmode X:
//  xxx_xx_00 : 0
//  xxx_01_01 : M
//  xxx_xx_10 : P
//  xxx_xx_11 : {A,B}
// opmode Y:
//  xxx_00_xx : 0
//  xxx_01_01 : M
//  xxx_10_xx : ~0
//  xxx_11_xx : C
// opmode Z:
//  000_xx_xx : 0
//  001_xx_xx : PCIN
//  010_xx_xx : P
//  011_xx_xx : C
//  100_10_00 : P for MACC
//  101_xx_xx : PCIN >> 17
//  110_xx_xx : P >> 17
//
//    1         7           4        3
// { round_opmode[6:0]_alu[3:0]_inmode[2:0]
// 4 stages:
// e0:  A2  D  B1           INMODE    OPMODE_ALUMODE_e0
// e1:    AD   B2                     OPMODE_ALUMODE_e1
// e2:       M      C                 OPMODE    ALUMODE  CARRYINSEL CARRYIN
// e3            P


reg [4:1]  stage_valid_r;
wire [3:0] stage_valid = { stage_valid_r[3:1], s_adb_valid };
assign     m_p_valid = stage_valid_r[4];
assign     m_p_svalid = stage_valid;

//reg [2:0]  INMODE_e0;
reg [10:0] OPMODE_ALUMODE_e0;
reg [10:0] OPMODE_ALUMODE_e1;
reg        ROUND_MUL_e0;
reg        ROUND_MUL_e1;

reg [TAG_WIDTH-1:0] TAG_e0;
reg [TAG_WIDTH-1:0] TAG_e1;
reg [TAG_WIDTH-1:0] TAG_e2;
reg [TAG_WIDTH-1:0] TAG_e3;
assign m_p_tag = TAG_e3;

wire rst_dsp = rst & RESET_DSP48;

always @(posedge clk) begin
	if (rst) begin
		stage_valid_r <= 0;
	end else begin
		stage_valid_r <= stage_valid;
		
		if (stage_valid[0]) begin
			//INMODE_e0         <= s_adb_cmd[2:0];
			OPMODE_ALUMODE_e0 <= s_adb_cmd[13:3];
			ROUND_MUL_e0      <= s_adb_cmd[14];
			TAG_e0            <= s_adb_tag;
			//$display("%g: DSP48.%0d.0 IN=%b OP=%b  A=%x D=%x B=%x", $time, ID, s_adb_cmd[2:0], s_adb_cmd[13:3], s_adb_data[29+25+18:25+18], s_adb_data[24+18:18], s_adb_data[17:0]);
		end
		
		if (stage_valid[1]) begin
			OPMODE_ALUMODE_e1 <= OPMODE_ALUMODE_e0;
			ROUND_MUL_e1      <= ROUND_MUL_e0;
			TAG_e1            <= TAG_e0;
		end
		
		if (stage_valid[2]) begin
			TAG_e2            <= TAG_e1;
		end
		
		if (stage_valid[3]) begin
			TAG_e3            <= TAG_e2;
		end
	end
end



DSP48E1 #(
	.A_INPUT("DIRECT"),               // Selects A input source, "DIRECT" (A port) or "CASCADE" (ACIN port)
	.B_INPUT("DIRECT"),               // Selects B input source, "DIRECT" (B port) or "CASCADE" (BCIN port)
	.USE_DPORT("TRUE"),               // Select D port usage (TRUE or FALSE)
	.USE_MULT("MULTIPLY"),            // Select multiplier usage ("MULTIPLY", "DYNAMIC", or "NONE")
	.USE_SIMD("ONE48"),               // SIMD selection ("ONE48", "TWO24", "FOUR12")
	.AUTORESET_PATDET("NO_RESET"),    // "NO_RESET", "RESET_MATCH", "RESET_NOT_MATCH"
	.MASK(48'hffffffffffff),          // 48-bit mask value for pattern detect (1=ignore)
	.PATTERN(48'h000000000000),       // 48-bit pattern match for pattern detect
	.SEL_MASK("MASK"),                // "C", "MASK", "ROUNDING_MODE1", "ROUNDING_MODE2"
	.SEL_PATTERN("PATTERN"),          // Select pattern value ("PATTERN" or "C")
	.USE_PATTERN_DETECT("NO_PATDET"), // Enable pattern detect ("PATDET" or "NO_PATDET")
	.ACASCREG(1'b1),                  // Number of pipeline stages between A/ACIN and ACOUT (0, 1 or 2)
	.ADREG(1'b1),                     // Number of pipeline stages for pre-adder (0 or 1)
	.ALUMODEREG(1'b1),                // Number of pipeline stages for ALUMODE (0 or 1)
	.AREG(1'b1),                      // Number of pipeline stages for A (0, 1 or 2)
	.BCASCREG(1'b1),                  // Number of pipeline stages between B/BCIN and BCOUT (0, 1 or 2)
	.BREG(2'b10),                     // Number of pipeline stages for B (0, 1 or 2)
	.CARRYINREG(1'b1),                // Number of pipeline stages for CARRYIN (0 or 1)
	.CARRYINSELREG(1'b1),             // Number of pipeline stages for CARRYINSEL (0 or 1)
	.CREG(1'b1),                      // Number of pipeline stages for C (0 or 1)
	.DREG(1'b1),                      // Number of pipeline stages for D (0 or 1)
	.INMODEREG(1'b1),                 // Number of pipeline stages for INMODE (0 or 1)
	.MREG(1'b1),                      // Number of multiplier pipeline stages (0 or 1)
	.OPMODEREG(1'b1),                 // Number of pipeline stages for OPMODE (0 or 1)
	.PREG(1'b1)                       // Number of pipeline stages for P (0 or 1)
) dsp (
	.ACOUT(),                         // 30-bit output: A port cascade output
	.BCOUT(),                         // 18-bit output: B port cascade output
	.CARRYCASCOUT(),                  // 1-bit output: Cascade carry output
	.MULTSIGNOUT(),                   // 1-bit output: Multiplier sign cascade output
	.PCOUT(dspcasc_p_o),              // 48-bit output: Cascade output

	.OVERFLOW(),                      // 1-bit output: Overflow in add/acc output
	.PATTERNBDETECT(),                // 1-bit output: Pattern bar detect output
	.PATTERNDETECT(),                 // 1-bit output: Pattern detect output
	.UNDERFLOW(),                     // 1-bit output: Underflow in add/acc output

	.CARRYOUT(),                      // 4-bit output: Carry output
	.P(m_p_data),                     // 48-bit output: Primary data output
	
	.ACIN(),                          // 30-bit input: A cascade data input
	.BCIN(),                          // 18-bit input: B cascade input
	.CARRYCASCIN(),                   // 1-bit input: Cascade carry input
	.MULTSIGNIN(),                    // 1-bit input: Multiplier sign input
	.PCIN(dspcasc_p_i),               // 48-bit input: P cascade input

	.ALUMODE(OPMODE_ALUMODE_e1[3:0]), // 4-bit input: ALU control input
	.CARRYINSEL(ROUND_MUL_e1 ? 3'b110 : 3'b000),              // 3-bit input: Carry select input
	.CLK(clk),                        // 1-bit input: Clock input
	.INMODE({1'b0, s_adb_cmd[2:0], 1'b0}), // 5-bit input: INMODE control input
	.OPMODE(OPMODE_ALUMODE_e1[10:4]), // 7-bit input: Operation mode input

	.A(s_adb_data[29+25+18:25+18]),   // 30-bit input: A data input
	.B(s_adb_data[17:0]),             // 18-bit input: B data input
	.C(s_c_data),                     // 48-bit input: C data input
	.CARRYIN(1'b0),                   // 1-bit input: Carry input signal
	.D(s_adb_data[24+18:18]),         // 25-bit input: D data input

	.CEA1(1'b0),                      // 1-bit input: Clock enable input for 1st stage AREG
	.CEA2(stage_valid[0]),            // 1-bit input: Clock enable input for 2nd stage AREG
	.CEAD(stage_valid[1]),            // 1-bit input: Clock enable input for ADREG
	.CEALUMODE(stage_valid[2]),       // 1-bit input: Clock enable input for ALUMODE
	.CEB1(stage_valid[0]),            // 1-bit input: Clock enable input for 1st stage BREG
	.CEB2(stage_valid[1]),            // 1-bit input: Clock enable input for 2nd stage BREG
	.CEC(s_c_valid),                  // 1-bit input: Clock enable input for CREG
	.CECARRYIN(stage_valid[2]),       // 1-bit input: Clock enable input for CARRYINREG
	.CECTRL(stage_valid[2]),          // 1-bit input: Clock enable input for OPMODEREG and CARRYINSELREG
	.CED(stage_valid[0]),             // 1-bit input: Clock enable input for DREG
	.CEINMODE(stage_valid[0]),        // 1-bit input: Clock enable input for INMODEREG
	.CEM(stage_valid[2]),             // 1-bit input: Clock enable input for MREG
	.CEP(stage_valid[3]),             // 1-bit input: Clock enable input for PREG
                  
	.RSTA(rst_dsp),                   // 1-bit input: Reset input for AREG
	.RSTALLCARRYIN(rst_dsp),          // 1-bit input: Reset input for CARRYINREG
	.RSTALUMODE(rst_dsp),             // 1-bit input: Reset input for ALUMODEREG
	.RSTB(rst_dsp),                   // 1-bit input: Reset input for BREG
	.RSTC(rst_dsp),                   // 1-bit input: Reset input for CREG
	.RSTCTRL(rst_dsp),                // 1-bit input: Reset input for OPMODEREG and CARRYINSELREG
	.RSTD(rst_dsp),                   // 1-bit input: Reset input for DREG and ADREG
	.RSTINMODE(rst_dsp),              // 1-bit input: Reset input for INMODEREG
	.RSTM(rst_dsp),                   // 1-bit input: Reset input for MREG
	.RSTP(rst_dsp)                    // 1-bit input: Reset input for PREG
);


endmodule

