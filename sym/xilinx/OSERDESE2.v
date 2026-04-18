// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2026 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
`timescale 1ns / 1ps

module OSERDESE2 #(
    parameter DATA_RATE_OQ = "DDR",
    parameter DATA_RATE_TQ = "DDR",
    parameter integer DATA_WIDTH = 4
) (
    input RST,
    input D1,
    input D2,
    input D3,
    input D4,
    input D5,
    input D6,
    input D7,
    input D8,
    input CLK,
    input CLKDIV,
    input OCE,
    input T1,
    input T2,
    input T3,
    input T4,
    input TCE,
    input TBYTEIN,
    output TBYTEOUT,
    output OQ,
    output TQ,
    input OFB,
    output TFB,
    output SHIFTOUT1,
    output SHIFTOUT2,
    input SHIFTIN1,
    input SHIFTIN2
);

localparam integer SERDES_WIDTH = (DATA_WIDTH <= 2) ? 2 :
                                  (DATA_WIDTH <= 4) ? 4 : 8;
localparam integer OQ_IS_DDR = (DATA_RATE_OQ == "DDR");
localparam integer TQ_IS_DDR = (DATA_RATE_TQ == "DDR");

reg [7:0] oq_word = 8'd0;
reg [7:0] tq_word = 8'd0;
integer   serial_idx = 0;
reg       oq_r   = 1'b0;
reg       tq_r   = 1'b0;

function automatic select_serial_bit;
    input [7:0] word;
    input integer idx;
    begin
        case (idx)
            0: select_serial_bit = word[0];
            1: select_serial_bit = word[1];
            2: select_serial_bit = word[2];
            3: select_serial_bit = word[3];
            4: select_serial_bit = word[4];
            5: select_serial_bit = word[5];
            6: select_serial_bit = word[6];
            7: select_serial_bit = word[7];
            default: select_serial_bit = 1'b0;
        endcase
    end
endfunction

always @(posedge CLKDIV or posedge RST) begin
    if (RST) begin
        oq_word <= 8'd0;
        tq_word <= 8'd0;
    end else begin
        if (OCE) begin
            oq_word <= {D8, D7, D6, D5, D4, D3, D2, D1};
        end

        if (TCE) begin
            tq_word <= {4'b0000, T4, T3, T2, T1};
        end
    end
end

always @(posedge CLK or negedge CLK or posedge RST) begin
    if (RST) begin
        serial_idx <= 0;
        oq_r       <= 1'b0;
        tq_r       <= 1'b0;
    end else begin
        if (OQ_IS_DDR || CLK) begin
            if (OCE) begin
                oq_r <= select_serial_bit(oq_word, serial_idx);
            end
        end

        if (TQ_IS_DDR || CLK) begin
            if (TCE) begin
                tq_r <= select_serial_bit(tq_word, serial_idx);
            end
        end

        if ((OCE && (OQ_IS_DDR || CLK)) || (TCE && (TQ_IS_DDR || CLK))) begin
            if (serial_idx == SERDES_WIDTH - 1) begin
                serial_idx <= 0;
            end else begin
                serial_idx <= serial_idx + 1;
            end
        end
    end
end

assign OQ = oq_r;
assign TQ = tq_r;
assign TBYTEOUT = 1'b0;
assign TFB = tq_r;
assign SHIFTOUT1 = 1'b0;
assign SHIFTOUT2 = 1'b0;

endmodule
