module iq_corr #(
    parameter WIDTH    = 12
) (
    input clk,
    input rst,

    input [WIDTH * 2 - 1:0]      cfg_amp,
    input [WIDTH * 2 - 1:0]      cfg_tan,

    input [WIDTH * 2 - 1:0]      in_data,
    input                        in_valid,

    output reg [WIDTH * 2 - 1:0] out_data,
    output reg                   out_valid

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

localparam EXT_WIDTH = 2 * WIDTH - 1; // Sign-extended double


reg [WIDTH * 2 - 1:0]     data_s0_direct;
reg [EXT_WIDTH * 2 - 1:0] data_s0_mtan;
reg                       data_s0_valid;

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
        data_s0_mtan <= in_data;
    end

    if (data_s0_valid) begin
        out_valid <= data_s0_mtan;
    end
end



endmodule
