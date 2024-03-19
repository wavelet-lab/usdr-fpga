module simple_uart_tx #(
    parameter BITS_DATA = 8,
    parameter STOP_BITS = 1,
    parameter COUNTER_BITS = 16
)(
    input rst,
    input clk,

    input [COUNTER_BITS-1:0] cfg_counter_div,

    output txd,

    input [BITS_DATA-1:0] axis_data,
    input                 axis_valid,
    output reg            axis_ready
);

//                          0         XXXXXXX        1
//                       start bit | data bits | stop bits
localparam PACKET_BITS  = 1 + BITS_DATA + STOP_BITS;

reg [3:0]            bitpos; // 4 bits is enough for 1 + 9 + 2 (12bit) max packet
reg [BITS_DATA:0]    data;
wire                 clk_both_edge;

uart_divider #(.COUNTER_BITS(COUNTER_BITS)) txdiv (
    .clk(clk),
    .cfg_counter_div(cfg_counter_div),
    .idle(bitpos == 0),
    .strobe(clk_both_edge)
);

always @(posedge clk) begin
    if (rst) begin
        bitpos      <= 0;
        data[0]     <= 1;
        axis_ready  <= 1;
    end else begin
        if (bitpos == 0) begin
            if (axis_valid && axis_ready) begin
                data[BITS_DATA:0] <= { axis_data, 1'b0 };
                bitpos            <= 1;
                axis_ready        <= 1'b0;
            end
        end else if (clk_both_edge) begin
            bitpos            <= bitpos + 1'b1;
            data[BITS_DATA:0] <= {1'b1, data[BITS_DATA:1]};

            if (bitpos == PACKET_BITS) begin
                bitpos       <= 1'b0;
                axis_ready   <= 1'b1;
            end
        end
    end
end

assign txd = data[0];

endmodule
