// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2026 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
`timescale 1ns / 1ps

module ODDR(
    output Q,
    input  D1,
    input  D2,
    input  CE,
    input  C,
    input  R
);

reg q_r = 1'b0;

always @(posedge C or posedge R) begin
    if (R) begin
        q_r <= 1'b0;
    end else if (CE) begin
        q_r <= D1;
    end
end

always @(negedge C or posedge R) begin
    if (R) begin
        q_r <= 1'b0;
    end else if (CE) begin
        q_r <= D2;
    end
end

assign Q = q_r;

endmodule
