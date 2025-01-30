// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
// Packet size < DEEP to not hang in ALLOW_ON_FULL==FALSE
module axis_fifo_pkt #(
    parameter         WIDTH = 32,
    parameter         DEEP = 32,
    parameter         DEEP_BITS = (DEEP <= 16) ? 4 : (DEEP <= 32) ? 5 : (DEEP <= 64) ? 6 : 7,
    parameter   [0:0] EXTRA_REG = 1'b1,
    parameter   [0:0] PASSTHROUGH = 1'b0,
    parameter   [0:0] ALLOW_ON_FULL = 1'b1
)(
  input                            clk,
  input                            rst,

  input [WIDTH-1:0]                s_rx_tdata,
  input                            s_rx_tvalid,
  input                            s_rx_tlast,
  output                           s_rx_tready,

  output [WIDTH-1:0]               m_tx_tdata,
  output                           m_tx_tvalid,
  output                           m_tx_tlast,
  input                            m_tx_tready,

  output [DEEP_BITS-1+EXTRA_REG:0] fifo_used,
  output                           fifo_full,
  output                           fifo_empty
);

wire fifo_tx_tready;
wire fifo_tx_tvalid;

generate
if (PASSTHROUGH) begin
    assign fifo_tx_tready = m_tx_tready;
    assign m_tx_tvalid    = fifo_tx_tvalid;
end else begin
    reg [DEEP_BITS:0]  in_cnt;
    reg [DEEP_BITS:0]  out_cnt;
    reg                busy;

    wire [DEEP_BITS:0] pkt_nonzero = in_cnt - out_cnt - 1'b1;

    always @(posedge clk) begin
        if (rst) begin
            in_cnt  <= 0;
            out_cnt <= 0;
            busy    <= 1'b1;
        end else begin
            busy <= (!ALLOW_ON_FULL || !fifo_full) && pkt_nonzero[DEEP_BITS];

            if (m_tx_tvalid && m_tx_tready && m_tx_tlast) begin
                out_cnt <= out_cnt + 1'b1;
                busy    <= 1'b1;  //need to update counter, TODO: auto
            end
            
            if (s_rx_tvalid && s_rx_tready && s_rx_tlast) begin
                in_cnt <= in_cnt + 1'b1;
                busy   <= 1'b0;
            end
        end
    end

    assign fifo_tx_tready = !busy && m_tx_tready;
    assign m_tx_tvalid    = !busy && fifo_tx_tvalid;
end
endgenerate

axis_fifo #(.WIDTH(WIDTH + 1), .DEEP(DEEP), .EXTRA_REG(EXTRA_REG)) fifo (
    .clk(clk),
    .rst(rst),

    .s_rx_tdata({s_rx_tlast, s_rx_tdata}),
    .s_rx_tvalid(s_rx_tvalid),
    .s_rx_tready(s_rx_tready),

    .m_tx_tdata({m_tx_tlast, m_tx_tdata}),
    .m_tx_tvalid(fifo_tx_tvalid),
    .m_tx_tready(fifo_tx_tready),
    

    .fifo_used(fifo_used),
    .fifo_full(fifo_full),
    .fifo_empty(fifo_empty)
);

endmodule

