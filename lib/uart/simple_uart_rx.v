module simple_uart_rx #(
    parameter BITS_DATA = 8,
    parameter COUNTER_BITS = 16
)(
    input rst,
    input clk,

    input [COUNTER_BITS-1:0] cfg_counter_div,

    input rxd,

    output     [BITS_DATA-1:0] axis_data,
    output reg                 axis_valid,
    output reg                 axis_fault,
    output                     axis_stop
);

wire rxd_sync;
synchronizer  #(.INIT(1)) rxd_sample_sync  (
    .clk(clk),
    .rst(rst),
    .a_in(rxd),
    .s_out(rxd_sync)
);

localparam DEBOUNCE_BITS = 3;
reg  [DEBOUNCE_BITS - 1:0] rxd_sync_db;
wire [DEBOUNCE_BITS:0]     rxd_sync_adb = { rxd_sync_db, rxd_sync };
always @(posedge clk) begin
    if (rst) begin
        rxd_sync_db <= ~0;
    end else begin
        rxd_sync_db <= rxd_sync_adb[DEBOUNCE_BITS-1:0];
    end
end

wire logic_lo = (rxd_sync_adb == {(DEBOUNCE_BITS + 1){1'b0}});
wire logic_hi = (rxd_sync_adb == {(DEBOUNCE_BITS + 1){1'b1}});

//                          0         XXXXXXX        1/2
//                       start bit | data bits | stop bits (at least 1)
localparam PACKET_BITS  = 1 + BITS_DATA + 1;
reg [BITS_DATA:0]    data;
reg [3:0]            bitpos; // 4 bits is enough for 1 + 9 + 2 (12bit) max packet
wire                 clk_both_edge_90;

uart_divider #(.COUNTER_BITS(COUNTER_BITS), .PHASE_90(1'b1)) rxdiv (
    .clk(clk),
    .cfg_counter_div(cfg_counter_div),
    .idle(bitpos == 0),
    .strobe(clk_both_edge_90)
);

always @(posedge clk) begin
    if (rst) begin
        bitpos     <= 0;
        axis_valid <= 1'b0;
        axis_fault <= 1'b0;
    end else begin
        axis_valid <= 1'b0;

        if (bitpos == 0) begin
            // Start bit
            if (logic_lo) begin
                bitpos     <= bitpos + 1'b1;
                axis_fault <= 1'b0;
            end
        end else if (clk_both_edge_90) begin
            data       <= { logic_hi, data[BITS_DATA:1] };
            axis_fault <= axis_fault || (!logic_lo && !logic_hi);
            bitpos     <= bitpos + 1'b1;

            if (bitpos == PACKET_BITS) begin
                axis_valid <= 1'b1;
                bitpos     <= 0;
            end
        end

    end
end

assign axis_data = data[BITS_DATA-1:0];
assign axis_stop = data[BITS_DATA]; // Stop bit, should always be 1, 0 -- indicates incorrect baud rate

endmodule
