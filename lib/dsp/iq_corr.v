module iq_corr #(
    parameter WIDTH     = 16,
    parameter CFG_WIDTH = 16,
    parameter OUT_WIDTH = 16
) (
    input clk,
    input rst,

    input [CFG_WIDTH * 2 - 1:0]  cfg_amp,
    input [CFG_WIDTH * 2 - 1:0]  cfg_tan,

    input [WIDTH * 2 - 1:0]      in_data,
    input                        in_valid,

    output     [OUT_WIDTH * 2 - 1:0] out_data,
    output reg                       out_valid

);

// Hardcoded for 2 channels for I/Q
//
//     stage_1          stage_2
// ------ | ---------| * [] + |---
//  \                 /
//   +--| * [] |-+ +-+
//                X
//   +--| * [] |-+ +-+
//  /                 \
// ------ | ---------| * [] + |---

localparam EXT_WIDTH = (CFG_WIDTH + WIDTH) - 1; // Sign-extended double


reg [WIDTH * 2 - 1:0]     data_s0_direct;

reg [EXT_WIDTH - 1:0]     data_s0_mtan_i;
reg [EXT_WIDTH - 1:0]     data_s0_mtan_q;

reg                       data_s0_valid;

reg [EXT_WIDTH - 1:0]     data_s1_corr_i;
reg [EXT_WIDTH - 1:0]     data_s1_corr_q;

always @(posedge clk) begin
    if (rst) begin
        data_s0_valid <= 0;
        out_valid     <= 0;
    end else begin
        data_s0_valid <= in_valid;
        out_valid     <= data_s0_valid;
    end
end

always @(posedge clk) begin
    if (in_valid) begin
        data_s0_direct <= in_data;
        data_s0_mtan_i <= $signed(in_data[WIDTH - 1:0])         * $signed(cfg_tan[CFG_WIDTH - 1:0]);
        data_s0_mtan_q <= $signed(in_data[2 * WIDTH - 1:WIDTH]) * $signed(cfg_tan[2 * CFG_WIDTH - 1:CFG_WIDTH]);
    end

    if (data_s0_valid) begin
        data_s1_corr_i <= $signed(data_s0_direct[WIDTH - 1:0])         * $signed(cfg_amp[CFG_WIDTH - 1:0])             + $signed(data_s0_mtan_q);
        data_s1_corr_q <= $signed(data_s0_direct[2 * WIDTH - 1:WIDTH]) * $signed(cfg_amp[2 * CFG_WIDTH - 1:CFG_WIDTH]) + $signed(data_s0_mtan_i);
    end
end

assign out_data = {
    data_s1_corr_q[EXT_WIDTH - 1:EXT_WIDTH - OUT_WIDTH],
    data_s1_corr_i[EXT_WIDTH - 1:EXT_WIDTH - OUT_WIDTH]
};

endmodule
