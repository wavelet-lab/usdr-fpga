module xlnx_serdes_clk #(
    parameter [0:0] ULTRA_SCALE = 0,
    parameter       DIV = 2
) (
    input clk,
    input ce,
    input clr,
    output clk_o,
    output clk_div_o
);

generate
if (ULTRA_SCALE) begin
    BUFGCE_DIV #(.BUFGCE_DIVIDE(1)) bufg_clk_i (
        .I   (clk),
        .CE  (ce),
        .CLR (clr),
        .O   (clk_o)
    );
    BUFGCE_DIV #(.BUFGCE_DIVIDE(DIV)) bufg_clkdiv_i (
        .I   (clk),
        .CE  (ce),
        .CLR (clr),
        .O   (clk_div_o)
    );
end else begin
    BUFR #(.BUFR_DIVIDE(DIV)) mrcc_div2(
    .I(clk),
    .O(clk_div_o),
    .CE(ce),
    .CLR(clr)
    );
    BUFIO in_bufio(.I(clk), .O(clk_o));
end
endgenerate

endmodule
