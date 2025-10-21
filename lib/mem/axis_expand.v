module axis_expand #(
    parameter WIDTH = 8,
    parameter EXPAND = 2
)(
  input                            clk,
  input                            rst,

  input [WIDTH-1:0]                s_rx_tdata,
  input                            s_rx_tvalid,
  output                           s_rx_tready,

  output reg [EXPAND * WIDTH-1:0]  m_tx_tdata,
  output reg                       m_tx_tvalid,
  input                            m_tx_tready
);

assign s_rx_tready = m_tx_tready;

localparam EXP_REG_WIDTH = $clog2(EXPAND);

reg [EXP_REG_WIDTH - 1:0] state;
always @(posedge clk) begin
    if (rst) begin
        m_tx_tvalid <= 1'b0;
        state       <= 0;
    end else begin
        if (s_rx_tvalid && s_rx_tready) begin
            state       <= state + 1'b1;
            m_tx_tvalid <= 1'b0;

            if (state == EXPAND - 1) begin
                m_tx_tvalid <= 1'b1;
                state       <= 0;
            end
        end
    end
end

genvar i;
generate
for (i = 0; i < WIDTH; i = i + 1) begin
    always @(posedge clk) begin
        if (s_rx_tvalid && s_rx_tready) begin
            m_tx_tdata[WIDTH * state + i] <= s_rx_tdata[i];
        end
    end
end
endgenerate


endmodule
