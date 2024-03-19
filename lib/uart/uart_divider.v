module uart_divider #(
    parameter COUNTER_BITS = 16,
    parameter PHASE_90 = 0
)(
    input  clk,
    input  idle,

    input [COUNTER_BITS-1:0] cfg_counter_div,

    output strobe
);

reg [COUNTER_BITS:0] clk_counter;
reg                  clk_div;
reg                  clk_div_p;
assign               strobe = (clk_div_p != clk_div);

always @(posedge clk) begin
    if (idle) begin
        clk_counter  <= (PHASE_90) ? (cfg_counter_div >> 1) : cfg_counter_div;
        clk_div      <= 0;
        clk_div_p    <= 0;
    end else begin
        clk_counter <= clk_counter - 1'b1;
        clk_div_p   <= clk_div;

        if (clk_counter[COUNTER_BITS]) begin
            clk_counter <= cfg_counter_div;
            clk_div     <= ~clk_div;
        end
    end
end

endmodule
