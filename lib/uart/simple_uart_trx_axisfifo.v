module simple_uart_trx_axisfifo #(
    parameter UART_SPEED    = 9600,
    parameter BUS_SPEED     = 62500000,
    parameter BUS_ALT_SPEED = 62500000,
    parameter FIFO_DEPTH    = 32
)(
    input  clk,
    input  rst,

    input  cfg_alt_mode,

    input  rxd,
    output txd,

    output        axis_wready,
    input [31:0]  axis_wdata,
    input         axis_wvalid,

    input         axis_rready,
    output [31:0] axis_rdata,
    output        axis_rvalid,

    output reg    int_valid,
    input         int_ready
);

localparam BITS_DATA        = 8;
localparam COUNTER_BITS     = 16;
localparam COUNTER_VAL      = (((2*BUS_SPEED     + UART_SPEED) / UART_SPEED) / 2);
localparam COUNTER_VAL_ALT  = (((2*BUS_ALT_SPEED + UART_SPEED) / UART_SPEED) / 2);

reg [COUNTER_BITS - 1:0] cfg_counter_div;

always @(posedge clk) begin
    if (rst) begin
        cfg_counter_div  <= (cfg_alt_mode) ? COUNTER_VAL_ALT : COUNTER_VAL;
    end else begin
        if (axis_wvalid && axis_wdata[31]) begin
            cfg_counter_div <= axis_wdata[COUNTER_BITS - 1:0];
        end
    end
end


wire [BITS_DATA+1:0]   axis_rx_data;
wire                   axis_rx_valid;
wire                   axis_rx_read;

simple_uart_rx #(
    .COUNTER_BITS(COUNTER_BITS)
) simple_uart_rx (
    .clk(clk),
    .rst(rst),

    .cfg_counter_div(cfg_counter_div),

    .rxd(rxd),

    .axis_data(axis_rx_data[BITS_DATA-1:0]),
    .axis_valid(axis_rx_valid),
    .axis_fault(axis_rx_data[BITS_DATA]),
    .axis_stop(axis_rx_data[BITS_DATA + 1])
);

axis_fifo #(.WIDTH(2+BITS_DATA), .DEEP(FIFO_DEPTH)) rx_fifo (
    .clk(clk),
    .rst(rst),

    .s_rx_tdata(axis_rx_data),
    .s_rx_tvalid(axis_rx_valid),
    .s_rx_tready(),

    .m_tx_tdata(axis_rdata[BITS_DATA+1:0]),
    .m_tx_tvalid(),
    .m_tx_tready(axis_rready),

    .fifo_used(axis_rdata[14:10]),
    .fifo_empty(axis_rdata[15])
);


wire [BITS_DATA-1:0] axis_tx_data;
wire                 axis_tx_valid;
wire                 axis_tx_read;

simple_uart_tx #(
    .COUNTER_BITS(COUNTER_BITS)
) simple_uart_tx (
    .clk(clk),
    .rst(rst),

    .cfg_counter_div(cfg_counter_div),

    .txd(txd),

    .axis_data(axis_tx_data),
    .axis_valid(axis_tx_valid),
    .axis_ready(axis_tx_read)
);

axis_fifo #(.WIDTH(BITS_DATA), .DEEP(FIFO_DEPTH)) tx_fifo (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata(axis_wdata[7:0]),
  .s_rx_tvalid(axis_wvalid && !axis_wdata[31]),
  .s_rx_tready(),

  .m_tx_tdata(axis_tx_data),
  .m_tx_tvalid(axis_tx_valid),
  .m_tx_tready(axis_tx_read),

  .fifo_used(axis_rdata[6 - 1 + 16:16]),
  .fifo_empty(axis_rdata[31])
);

assign axis_rdata[30:22] = 0;
assign axis_wready = 1'b1;
assign axis_rvalid = 1'b1;

always @(posedge clk) begin
    if (rst) begin
        int_valid <= 1'b0;
    end else begin
        if (int_valid && int_ready) begin
            int_valid <= 1'b0;
        end
    end
end

endmodule
