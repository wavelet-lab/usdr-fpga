module axis_shrink #(
    parameter WIDTH = 8,
    parameter SHRINK = 2
)(
  input                            clk,
  input                            rst,

  input [WIDTH-1:0]                s_rx_tdata,
  input                            s_rx_tvalid,
  output                           s_rx_tready,

  output [WIDTH / SHRINK-1:0]      m_tx_tdata,
  output                           m_tx_tvalid,
  input                            m_tx_tready
);

assign m_tx_tvalid = s_rx_tvalid;
localparam LOW_WIDTH     = WIDTH / SHRINK;
localparam EXP_REG_WIDTH = $clog2(SHRINK);

reg [EXP_REG_WIDTH - 1:0] state;

wire last_beat = (state == SHRINK - 1);

assign s_rx_tready = m_tx_tready && last_beat;

always @(posedge clk) begin
    if (rst) begin
        state       <= 0;
    end else begin
        if (m_tx_tvalid && m_tx_tready) begin
            state       <= last_beat ? 0 : state + 1'b1;
        end
    end
end

genvar i;
generate
for (i = 0; i < LOW_WIDTH; i = i + 1) begin
    assign m_tx_tdata[i] = s_rx_tdata[LOW_WIDTH * state + i];
end
endgenerate

endmodule
