// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module SRLC32E #(
	parameter INIT = 32'h00000000
) (
    input       CLK,
    input       CE,
    input       D,
    input [4:0] A,
    output      Q,
    output      Q31
);

reg [31:0] srcl32e = INIT;

assign Q   = srcl32e[A];
assign Q31 = srcl32e[31];

always @(posedge CLK) begin
	if (CE) begin
		srcl32e <= { srcl32e[30:0], D };
	end
end

endmodule

