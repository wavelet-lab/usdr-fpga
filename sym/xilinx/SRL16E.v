// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module SRL16E #(
	parameter INIT = 16'h0000
) (
    input       CLK,
    input       CE,
    input       D,
    input       A0,
    input       A1,
    input       A2,
    input       A3,
    output      Q
);

reg [15:0] src16e = INIT;

assign Q = src16e[{A3, A2, A1, A0}];

always @(posedge CLK) begin
	if (CE) begin
		src16e <= { src16e[14:0], D };
	end
end

endmodule

