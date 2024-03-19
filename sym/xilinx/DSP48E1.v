// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module DSP48E1 #(
                                                // Feature Control Attributes: Data Path Selection
    parameter A_INPUT = "DIRECT",               // Selects A input source, "DIRECT" (A port) or "CASCADE" (ACIN port)
    parameter B_INPUT = "DIRECT",               // Selects B input source, "DIRECT" (B port) or "CASCADE" (BCIN port)
    parameter USE_DPORT = "FALSE",              // Select D port usage (TRUE or FALSE)
    parameter USE_MULT = "MULTIPLY",            // Select multiplier usage ("MULTIPLY", "DYNAMIC", or "NONE")
    parameter USE_SIMD = "ONE48",               // SIMD selection ("ONE48", "TWO24", "FOUR12")
                                                // Pattern Detector Attributes: Pattern Detection Configuration
    parameter AUTORESET_PATDET = "NO_RESET",    // "NO_RESET", "RESET_MATCH", "RESET_NOT_MATCH"
    parameter MASK = 48'h3fffffffffff,          // 48-bit mask value for pattern detect (1=ignore)
    parameter PATTERN = 48'h000000000000,       // 48-bit pattern match for pattern detect
    parameter SEL_MASK = "MASK",                // "C", "MASK", "ROUNDING_MODE1", "ROUNDING_MODE2"
    parameter SEL_PATTERN = "PATTERN",          // Select pattern value ("PATTERN" or "C")
    parameter USE_PATTERN_DETECT = "NO_PATDET", // Enable pattern detect ("PATDET" or "NO_PATDET")
                                                // Register Control Attributes: Pipeline Register Configuration
    parameter ACASCREG = 1'b1,                  // Number of pipeline stages between A/ACIN and ACOUT (0, 1 or 2)
    parameter ADREG = 1'b1,                     // Number of pipeline stages for pre-adder (0 or 1)
    parameter ALUMODEREG = 1'b1,                // Number of pipeline stages for ALUMODE (0 or 1)
    parameter AREG = 1'b1,                      // Number of pipeline stages for A (0, 1 or 2)
    parameter BCASCREG = 1'b1,                  // Number of pipeline stages between B/BCIN and BCOUT (0, 1 or 2)
    parameter BREG = 1'b1,                      // Number of pipeline stages for B (0, 1 or 2)
    parameter CARRYINREG = 1'b1,                // Number of pipeline stages for CARRYIN (0 or 1)
    parameter CARRYINSELREG = 1'b1,             // Number of pipeline stages for CARRYINSEL (0 or 1)
    parameter CREG = 1'b1,                      // Number of pipeline stages for C (0 or 1)
    parameter DREG = 1'b1,                      // Number of pipeline stages for D (0 or 1)
    parameter INMODEREG = 1'b1,                 // Number of pipeline stages for INMODE (0 or 1)
    parameter MREG      = 1'b1,                 // Number of multiplier pipeline stages (0 or 1)
    parameter OPMODEREG = 1'b1,                 // Number of pipeline stages for OPMODE (0 or 1)
    parameter PREG      = 1'b1                  // Number of pipeline stages for P (0 or 1)
) (
                  // Cascade: 30-bit (each) output: Cascade Ports
	output [29:0]  ACOUT,                       // 30-bit output: A port cascade output
	output [17:0]  BCOUT,                       // 18-bit output: B port cascade output
	output         CARRYCASCOUT,                // 1-bit output: Cascade carry output
	output         MULTSIGNOUT,                 // 1-bit output: Multiplier sign cascade output
	output [47:0]  PCOUT,                       // 48-bit output: Cascade output
                  // Control: 1-bit (each) output: Control Inputs/Status Bits
	output         OVERFLOW,                    // 1-bit output: Overflow in add/acc output
	output         PATTERNBDETECT,              // 1-bit output: Pattern bar detect output
	output         PATTERNDETECT,               // 1-bit output: Pattern detect output
	output         UNDERFLOW,                   // 1-bit output: Underflow in add/acc output
                  // Data&colon; 4-bit (each) output: Data Ports
	output [3:0]   CARRYOUT,                    // 4-bit output: Carry output
	output [47:0]  P,                           // 48-bit output: Primary data output
                  // Cascade: 30-bit (each) input: Cascade Ports
	input [29:0]   ACIN,                        // 30-bit input: A cascade data input
	input [17:0]   BCIN,                        // 18-bit input: B cascade input
	input          CARRYCASCIN,                 // 1-bit input: Cascade carry input
	input          MULTSIGNIN,                  // 1-bit input: Multiplier sign input
	input [47:0]   PCIN,                        // 48-bit input: P cascade input
                  // Control: 4-bit (each) input: Control Inputs/Status Bits
	input [3:0]    ALUMODE,                     // 4-bit input: ALU control input
	input [2:0]    CARRYINSEL,                  // 3-bit input: Carry select input
	input          CLK,                         // 1-bit input: Clock input
	input [4:0]    INMODE,                      // 5-bit input: INMODE control input
	input [6:0]    OPMODE,                      // 7-bit input: Operation mode input
                  // Data&colon; 30-bit (each) input: Data Ports
	input [29:0]   A,                           // 30-bit input: A data input
	input [17:0]   B,                           // 18-bit input: B data input
	input [47:0]   C,                           // 48-bit input: C data input
	input          CARRYIN,                     // 1-bit input: Carry input signal
	input [24:0]   D,                           // 25-bit input: D data input
                  // Reset/Clock Enable: 1-bit (each) input: Reset/Clock Enable Inputs
	input          CEA1,                     // 1-bit input: Clock enable input for 1st stage AREG
	input          CEA2,                     // 1-bit input: Clock enable input for 2nd stage AREG
	input          CEAD,                     // 1-bit input: Clock enable input for ADREG
	input          CEALUMODE,                // 1-bit input: Clock enable input for ALUMODE
	input          CEB1,                     // 1-bit input: Clock enable input for 1st stage BREG
	input          CEB2,                     // 1-bit input: Clock enable input for 2nd stage BREG
	input          CEC,                      // 1-bit input: Clock enable input for CREG
	input          CECARRYIN,                // 1-bit input: Clock enable input for CARRYINREG
	input          CECTRL,                   // 1-bit input: Clock enable input for OPMODEREG and CARRYINSELREG
	input          CED,                      // 1-bit input: Clock enable input for DREG
	input          CEINMODE,                 // 1-bit input: Clock enable input for INMODEREG
	input          CEM,                      // 1-bit input: Clock enable input for MREG
	input          CEP,                      // 1-bit input: Clock enable input for PREG
	input          RSTA,                     // 1-bit input: Reset input for AREG
	input          RSTALLCARRYIN,            // 1-bit input: Reset input for CARRYINREG
	input          RSTALUMODE,               // 1-bit input: Reset input for ALUMODEREG
	input          RSTB,                     // 1-bit input: Reset input for BREG
	input          RSTC,                     // 1-bit input: Reset input for CREG
	input          RSTCTRL,                  // 1-bit input: Reset input for OPMODEREG and CARRYINSELREG
	input          RSTD,                     // 1-bit input: Reset input for DREG and ADREG
	input          RSTINMODE,                // 1-bit input: Reset input for INMODEREG
	input          RSTM,                     // 1-bit input: Reset input for MREG
	input          RSTP                      // 1-bit input: Reset input for PREG
);

// Data registers
reg [29:0] A1_reg;
reg [29:0] A2_reg;
reg [17:0] B1_reg;
reg [17:0] B2_reg;
reg [47:0] C_reg;
reg [24:0] D_reg;
reg [24:0] AD_reg;
reg [42:0] M_reg;
reg        AXNORB_reg;

reg        CARRYIN_reg;
reg [3:0]  ALUMODE_reg;
reg [6:0]  OPMODE_reg;
reg [2:0]  CARRYINSEL_reg;
reg [4:0]  INMODE_reg;

reg [47:0] P_reg;
reg        CARRYOUT_reg;

wire [4:0] INMODE_IN   = (INMODEREG) ? INMODE_reg : INMODE;

wire [29:0] A_IN_ST0   = (A_INPUT == "DIRECT") ? A : ACIN;
wire [29:0] A_IN_ST1   = (AREG == 0 || AREG == 1) ?  A_IN_ST0 : A1_reg;
wire [29:0] A_IN_ST2   = (AREG == 0) ? A_IN_ST1 : A2_reg;
wire [29:0] X_AB_0     = A_IN_ST2;
// AREG = 0: ACASCREG must be 0
// AREG = 1: ACASCREG must be 1
// AREG = 2: ACASCREG can be 1 or 2 
assign ACOUT           = (AREG == 2 && ACASCREG == 1) ? A_IN_ST1 : A_IN_ST2;
wire [24:0] A_OUT      = (INMODE_IN[1]) ? 25'h0 : (~INMODE_IN[0] ? A_IN_ST2[24:0] : A_IN_ST1[24:0]);
wire [24:0] D_IN_ST0   = (DREG == 0) ?  D : D_reg;
wire [24:0] D_IN_ST1   = (~INMODE_IN[2]) ? 30'h0 : D_IN_ST0;
wire [24:0] AD_SUM     = INMODE_IN[3] ?  D_IN_ST1 - A_OUT : D_IN_ST1 + A_OUT;
wire [24:0] AD_SUM_ST0 = (ADREG) ? AD_reg : AD_SUM;
wire [24:0] A_MULT     = (USE_DPORT == "FALSE") ? A_OUT : AD_SUM_ST0;

wire [17:0] B_IN_ST0   = (B_INPUT == "DIRECT") ? B : BCIN;
wire [17:0] B_IN_ST1   = (BREG == 0 || BREG == 1) ?  B_IN_ST0 : B1_reg;
wire [17:0] B_IN_ST2   = (BREG == 0) ?  B_IN_ST1 : B2_reg;
wire [17:0] X_AB_1     = B_IN_ST2;
assign BCOUT           = (BREG == 2 && BCASCREG == 1) ? B_IN_ST1 : B_IN_ST2;
wire [17:0] B_MULT     = INMODE_IN[4] ? B_IN_ST1 : B_IN_ST2;
wire        AXNORB     = (A_MULT[24] == B_MULT[17]);

// it's partial product, mot MULT !!!!
wire [42:0] MULT       = $signed(A_MULT) * $signed(B_MULT);
wire [42:0] MULT_ST0   = MREG ? M_reg : MULT;
wire        AXNORB_ST0 = MREG ? AXNORB_reg : AXNORB;
wire [42:0] C_ST0      = CREG ? C_reg : C;

wire        CARRYIN_IN = CARRYINREG ? CARRYIN_reg : CARRYIN;
// X Y Z 
wire [3:0]  ALUMODE_IN = (ALUMODEREG) ? ALUMODE_reg : ALUMODE;
wire [6:0]  OPMODE_IN  = (OPMODEREG)  ? OPMODE_reg  : OPMODE;
wire [2:0]  CARRYINSEL_IN = (CARRYINSELREG) ? CARRYINSEL_reg : CARRYINSEL;

wire [47:0] MUL_EXT    = {{5{MULT_ST0[42]}}, MULT_ST0 };

wire [47:0] X_MUX = 
	(OPMODE_IN[1:0] == 0) ? 48'h0 :
	(OPMODE_IN[1:0] == 1) ? { MUL_EXT[47:24], 24'h000000 } :
	(OPMODE_IN[1:0] == 2) ? P_reg : { X_AB_0 , X_AB_1 };

wire [47:0] Y_MUX = 
	(OPMODE_IN[3:2] == 0) ? 48'h0 :
	(OPMODE_IN[3:2] == 1) ? { 24'h000000, MUL_EXT[23:0] } :
	(OPMODE_IN[3:2] == 2) ? 48'hffffffffffff : C_ST0;

wire [47:0] Z_MUX =
    (OPMODE_IN[6:4] == 0) ? 48'h0 :
    (OPMODE_IN[6:4] == 1) ? PCIN :
    (OPMODE_IN[6:4] == 2) ? P_reg :
    (OPMODE_IN[6:4] == 3) ? C_ST0 :
    (OPMODE_IN[6:4] == 4) ? P_reg :
    (OPMODE_IN[6:4] == 5) ? $signed(PCIN) >> 17 :
    (OPMODE_IN[6:4] == 6) ? $signed(P_reg) >> 17 : 48'hxxxxxxxxxxxx;
    
wire        CIN =
	(CARRYINSEL_IN == 0) ? CARRYIN_IN :
	(CARRYINSEL_IN == 1) ? ~PCIN[47] :
	(CARRYINSEL_IN == 2) ? CARRYCASCIN :
	(CARRYINSEL_IN == 3) ? PCIN[47] :
	(CARRYINSEL_IN == 4) ? CARRYCASCOUT :
	(CARRYINSEL_IN == 5) ? ~P_reg[47] :
	(CARRYINSEL_IN == 6) ? AXNORB_ST0 : P_reg[47];

wire [48:0] ALU_OUT =
	(ALUMODE_IN == 0) ? Z_MUX + X_MUX + Y_MUX + CIN :
	(ALUMODE_IN == 1) ? Z_MUX - (X_MUX + Y_MUX + CIN) :
	(ALUMODE_IN == 2) ? ~Z_MUX + X_MUX + Y_MUX + CIN : 
	(ALUMODE_IN == 3) ? ~(Z_MUX + X_MUX + Y_MUX + CIN) :
	(~OPMODE_IN[3] && ALUMODE_IN == 4) ?   X_MUX ^ Z_MUX :
	(~OPMODE_IN[3] && ALUMODE_IN == 5) ? ~(X_MUX ^ Z_MUX) :
	(~OPMODE_IN[3] && ALUMODE_IN == 6) ? ~(X_MUX ^ Z_MUX) :
	(~OPMODE_IN[3] && ALUMODE_IN == 7) ?   X_MUX ^ Z_MUX :
	(~OPMODE_IN[3] && ALUMODE_IN == 12)?   X_MUX & Z_MUX :
	(~OPMODE_IN[3] && ALUMODE_IN == 13)?   X_MUX & (~Z_MUX) :
	(~OPMODE_IN[3] && ALUMODE_IN == 14)? ~(X_MUX & Z_MUX) :
	(~OPMODE_IN[3] && ALUMODE_IN == 15)?  ~X_MUX | Z_MUX :
	(OPMODE_IN[3] && ALUMODE_IN == 4)  ? ~(X_MUX ^ Z_MUX) :
	(OPMODE_IN[3] && ALUMODE_IN == 5)  ?   X_MUX ^ Z_MUX :
	(OPMODE_IN[3] && ALUMODE_IN == 6)  ?   X_MUX ^ Z_MUX :
	(OPMODE_IN[3] && ALUMODE_IN == 7)  ? ~(X_MUX ^ Z_MUX) :
	(OPMODE_IN[3] && ALUMODE_IN == 12) ?   X_MUX | Z_MUX :
	(OPMODE_IN[3] && ALUMODE_IN == 13) ?   X_MUX | (~Z_MUX) :
	(OPMODE_IN[3] && ALUMODE_IN == 14) ? ~(X_MUX | Z_MUX) :
	(OPMODE_IN[3] && ALUMODE_IN == 15) ?  ~X_MUX & Z_MUX : 49'hxxxxxxxxxxxxx;

assign P     = (PREG) ? P_reg : ALU_OUT[47:0];
assign PCOUT = (PREG) ? P_reg : ALU_OUT[47:0];

assign CARRYCASCOUT = (PREG) ? CARRYOUT_reg : ALU_OUT[48];
assign CARRYOUT     = { (PREG) ? CARRYOUT_reg : ALU_OUT[48], 3'h0 };

assign OVERFLOW       = 1'bx; //TODO
assign PATTERNBDETECT = 1'bx; //TODO
assign PATTERNDETECT  = 1'bx; //TODO
assign UNDERFLOW      = 1'bx; //TODO

always @(posedge CLK) begin
	if (RSTA) begin
		A1_reg <= 0;
		A2_reg <= 0;
	end else begin
		if (CEA1)
			A1_reg <= A_IN_ST0;
		if (CEA2)
			A2_reg <= A_IN_ST1;
	end
	
	if (RSTB) begin
		B1_reg <= 0;
		B2_reg <= 0;
	end else begin
		if (CEB1)
			B1_reg <= B_IN_ST0;
		if (CEB2)
			B2_reg <= B_IN_ST1;
	end
	
	if (RSTC) begin
		C_reg <= 0;
	end else begin
		if (CEC)
			C_reg <= C;
	end
	
	if (RSTD) begin
		D_reg <= 0;
		AD_reg <= 0;
	end else begin
		if (CED)
			D_reg <= D;
		if (CEAD)
			AD_reg <= AD_SUM;
	end
	
	if (RSTINMODE) begin
		INMODE_reg <= 0;
	end else begin
		if (CEINMODE)
			INMODE_reg <= INMODE;
	end 
	
	
	if (RSTM) begin
		M_reg      <= 0;
	end else if (CEM) begin
		M_reg      <= MULT;
		AXNORB_reg <= AXNORB;
	end

	if (RSTALLCARRYIN) begin
		AXNORB_reg  <= 0;
		CARRYIN_reg <= 0;
	end else begin
		if (CEM)
			AXNORB_reg <= AXNORB;
		if (CECARRYIN)
			CARRYIN_reg <= CARRYIN;
	end
	
	if (RSTCTRL) begin
		OPMODE_reg     <= 0;
		CARRYINSEL_reg <= 0;
	end else begin
		if (CECTRL) begin
			OPMODE_reg <= OPMODE;
			CARRYINSEL_reg <= CARRYINSEL;
		end
	end
	
	if (RSTALUMODE) begin
		ALUMODE_reg <= 0;
	end else begin
		if (CEALUMODE) begin
			ALUMODE_reg <= ALUMODE;
		end
	end

	if (RSTP) begin
		P_reg <= 0;
		CARRYOUT_reg <= 0;
	end else begin
		if (CEP) begin
			P_reg        <= ALU_OUT[47:0];
			CARRYOUT_reg <= ALU_OUT[48];
		end
	end
end

endmodule


