// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
// Simple LUT-based FIFO with 2 slots
// use to relax backpressure on tready signal

module axis_fifo_trd #(
    parameter WIDTH = 32,
    parameter BYPASS = 1'b0
) (
  input                            clk,
  input                            rst,

  input [WIDTH-1:0]                s_rx_tdata,
  input                            s_rx_tvalid,
  output                           s_rx_tready,

  output [WIDTH-1:0]               m_tx_tdata,
  output                           m_tx_tvalid,
  input                            m_tx_tready
);

generate
if (BYPASS) begin
  assign m_tx_tdata  = s_rx_tdata;
  assign m_tx_tvalid = s_rx_tvalid;
  assign s_rx_tready = m_tx_tready;
end else begin
reg s_rx_tready_r;
reg m_tx_tvalid_r;
assign s_rx_tready = s_rx_tready_r;
assign m_tx_tvalid = m_tx_tvalid_r;

reg [1:0] rptr;
reg [1:0] wptr;

wire [1:0] used = wptr - rptr;

reg [WIDTH-1:0] s0;
reg [WIDTH-1:0] s1;

assign m_tx_tdata = rptr[0] ? s1: s0;

always @(posedge clk) begin
  if (rst) begin
    s_rx_tready_r <= 1'b1;
    m_tx_tvalid_r <= 1'b0;
    rptr          <= 2'b00;
    wptr          <= 2'b00;
  end else begin
    if (s_rx_tready && s_rx_tvalid) begin
        wptr <= wptr + 1'b1;
        if (wptr[0] == 1'b0)
            s0 <= s_rx_tdata;
        else
            s1 <= s_rx_tdata;

        s_rx_tready_r <= (used < 1);
        m_tx_tvalid_r <= 1'b1;
    end

    if (m_tx_tvalid && m_tx_tready) begin
        rptr          <= rptr + 1'b1;
        s_rx_tready_r <= 1'b1;
        m_tx_tvalid_r <= (s_rx_tready && s_rx_tvalid) ? 1'b1 : used[1];
    end
  end
end

`ifdef FORMAL
    initial restrict(rst);

    always @(posedge clk) begin
      if (!rst) begin
        assert(used != 3);

        cover(used == 0);
        cover(used == 1);
        cover(used == 2);

        cover(m_tx_tvalid && m_tx_tready && !$past(m_tx_tvalid));
        cover(m_tx_tvalid && m_tx_tready && $past(m_tx_tvalid) && $past(m_tx_tready));

        cover(s_rx_tvalid && s_rx_tready && !$past(s_rx_tvalid));
        cover(s_rx_tvalid && s_rx_tready && $past(s_rx_tvalid) && $past(s_rx_tready));
      end
    end
`endif


end
endgenerate

endmodule
