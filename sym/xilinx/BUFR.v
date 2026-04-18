// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2026 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
`timescale 1ns / 1ps

module BUFR #(
    parameter BUFR_DIVIDE = "BYPASS"
) (
    input  I,
    output O,
    input  CE,
    input  CLR
);

localparam DIVIDE_BYPASS = (BUFR_DIVIDE == "BYPASS") || (BUFR_DIVIDE == "1");
localparam DIVIDE_BY_2   = (BUFR_DIVIDE == "2");

generate
if (DIVIDE_BYPASS) begin : gen_passthrough
    assign O = (CE && !CLR) ? I : 1'b0;
end else if (DIVIDE_BY_2) begin : gen_div2
    reg o_r = 1'b0;

    always @(posedge I or posedge CLR) begin
        if (CLR) begin
            o_r <= 1'b0;
        end else if (CE) begin
            o_r <= ~o_r;
        end
    end

    assign O = o_r;
end else begin : gen_passthrough
    initial begin
        $display("WARNING: BUFR behavioral model only supports BUFR_DIVIDE=BYPASS, \"1\", or \"2\" (got %0s)", BUFR_DIVIDE);
    end

    assign O = 1'bx;
end
endgenerate

endmodule
