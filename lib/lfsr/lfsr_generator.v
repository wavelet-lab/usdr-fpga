module lfsr_generator #(
    parameter DATA_WIDTH = 32,
    parameter LFSR_POLY = 15'h6000,
    parameter LFSR_LEN = 15,
    parameter _STATE_WIDTH = (LFSR_LEN > DATA_WIDTH) ? LFSR_LEN : DATA_WIDTH
)(
    input                         clk,
    input                         rst,
    input      [_STATE_WIDTH-1:0] init_i,
    output     [DATA_WIDTH-1:0]   data_o
);

reg [_STATE_WIDTH - 1:0] state;
assign data_o = state;

wire next_bit = ^(state & LFSR_POLY);

always @(posedge clk) begin
    if (rst) begin
        state <= init_i;
    end else begin
        state <= { state[_STATE_WIDTH - 2:0], next_bit };
    end
end

endmodule
