// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module axis_opt_cc_fifo #(
  parameter CLK_CC = 1,
  parameter NO_DATA = 0,
  parameter WIDTH = 12,
  parameter DEEP = 32,
  parameter CC_SYNC_STAGES = 2,
  parameter ASYNC_RESET = 0,
  parameter ULTRA_SCALE = 0,
  parameter CC_DATA_PIPELINED = 1,
  parameter DIRECT_DATA_PIPELINED = 0,
  parameter CC_RX_VALID_PIPELINE_S = 1'b0
) (
  input                            rx_clk,
  input                            rx_rst,

  input [WIDTH-1:0]                s_rx_tdata,
  input                            s_rx_tvalid,
  output                           s_rx_tready,


  input                            tx_clk,
  input                            tx_rst,

  output [WIDTH-1:0]               m_tx_tdata,
  output                           m_tx_tvalid,
  input                            m_tx_tready
);

wire [WIDTH-1:0] tx_tdata;
wire             tx_tvalid;
wire             tx_tready;

generate
if (CLK_CC && (DEEP > 0)) begin
    axis_cc_fifo #(
        .WIDTH(WIDTH),
        .DEEP(DEEP),
        .DATA_PIPELINE(1'b0),
        .ULTRA_SCALE(ULTRA_SCALE),
        .ASYNC_RESET(ASYNC_RESET),
        .RX_VALID_PIPELINE_S(CC_RX_VALID_PIPELINE_S)
    ) cc_fifo (
        .rx_clk(rx_clk),
        .rx_rst(rx_rst),

        .s_rx_tdata(s_rx_tdata),
        .s_rx_tvalid(s_rx_tvalid),
        .s_rx_tready(s_rx_tready),

        .tx_clk(tx_clk),
        .tx_rst(tx_rst),

        .m_tx_tdata(tx_tdata),
        .m_tx_tvalid(tx_tvalid),
        .m_tx_tready(tx_tready)
    );
end else if (CLK_CC) begin
    // Simple syncronization. asuming data propagation path is less than syncronization time. In general this asumption might be wrong, so
    // use with caution! As a simple solution you can increase syncronization time (CC_SYNC_STAGES)
    reg  wa;
    wire wb;
    wire ra;
    reg  rb;

    synchronizer #(.SYNC_STAGES(CC_SYNC_STAGES), .ASYNC_RESET(ASYNC_RESET)) s_to_m(
        .clk(tx_clk),
        .rst(tx_rst),
        .a_in(wa),
        .s_out(wb)
    );

    synchronizer #(.ASYNC_RESET(ASYNC_RESET)) m_to_s(
        .clk(rx_clk),
        .rst(rx_rst),
        .a_in(rb),
        .s_out(ra)
    );

    assign s_rx_tready  = wa ^ ra ^ 1;
    assign tx_tvalid = wb ^ rb;

    always @(posedge rx_clk) begin
        if (rx_rst) begin
            wa <= 1'b0;
        end else if (s_rx_tvalid && s_rx_tready) begin
            wa <= ~wa;
        end
    end

    always @(posedge tx_clk) begin
        if (tx_rst) begin
            rb <= 1'b0;
        end else if (tx_tvalid && tx_tready) begin
            rb <= ~rb;
        end
    end

    if (!NO_DATA) begin
        reg [WIDTH-1:0] inlatch;
        always @(posedge rx_clk) begin
            if (s_rx_tvalid && s_rx_tready)
                inlatch <= s_rx_tdata;
        end
        assign tx_tdata = inlatch;
    end

end else begin
    if (!NO_DATA) begin
        assign tx_tdata    = s_rx_tdata;
    end
    assign tx_tvalid   = s_rx_tvalid;
    assign s_rx_tready = tx_tready;
end
endgenerate

localparam DATA_PIPELINE = CLK_CC && CC_DATA_PIPELINED || CLK_CC && (DEEP == 0) || !CLK_CC && DIRECT_DATA_PIPELINED;
axis_opt_pipeline #(.WIDTH(WIDTH), .PIPELINE(DATA_PIPELINE), .REG_READY(1'b0)) out (
    .clk(tx_clk),
    .rst(tx_rst),

    .s_rx_tdata(tx_tdata),
    .s_rx_tvalid(tx_tvalid),
    .s_rx_tready(tx_tready),

    .m_tx_tdata(m_tx_tdata),
    .m_tx_tvalid(m_tx_tvalid),
    .m_tx_tready(m_tx_tready)
);


endmodule

