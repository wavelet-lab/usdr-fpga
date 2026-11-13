// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2026 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module dc_estimator #(
    parameter CHANS = 4,
    parameter IN_WIDTH = 12,
    parameter OUT_WIDTH = 32,
    parameter ACC_WIDTH = 8,
    parameter CNT_WIDTH = 4
) (
    input clk,
    input rst,

    input [IN_WIDTH * CHANS - 1:0]    in_data,
    input                             in_valid,
    input [ACC_WIDTH - 1:0]           in_acc,

    output     [OUT_WIDTH * CHANS - 1:0]    out_data,
    output reg [CNT_WIDTH - 1:0]            out_cnt
);

localparam ACC_D_CNT = 16;
reg [ACC_WIDTH + ACC_D_CNT - 1:0] accumcnt;

wire data_acced = accumcnt[ACC_WIDTH + ACC_D_CNT - 1:ACC_D_CNT] == in_acc;

always @(posedge clk) begin
    if (rst) begin
        out_cnt  <= 0;
        accumcnt <= 0;
    end else begin
        if (in_valid) begin
            accumcnt <= accumcnt + 1'b1;
            if (data_acced) begin
                out_cnt  <= out_cnt + 1'b1;
                accumcnt <= 0;
            end
        end
    end
end

genvar i;
generate
for (i = 0; i < CHANS; i=i+1) begin
    reg [OUT_WIDTH - 1:0] acc_data;
    reg [OUT_WIDTH - 1:0] acc_data_l;
    wire [IN_WIDTH - 1:0] ain_data = in_data[IN_WIDTH * (i + 1) - 1:IN_WIDTH *i];
    always @(posedge clk) begin
        if (rst) begin
            acc_data   <= 0;
            acc_data_l <= ~0;
        end else begin
            if (in_valid) begin
                acc_data <= acc_data + { {(OUT_WIDTH - IN_WIDTH + 1){ain_data[IN_WIDTH - 1]}}, ain_data[IN_WIDTH - 2:0]};
                if (data_acced) begin
                    acc_data_l <= acc_data;
                    acc_data   <= 0;
                end
            end
        end
    end

    assign out_data[OUT_WIDTH * (i + 1) - 1:OUT_WIDTH *i] = acc_data_l;
end
endgenerate


endmodule
