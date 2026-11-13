module data_snapshoter #(
    parameter DEEP = 64,
    parameter _DEEP_BITS = $clog2(DEEP),
    parameter WIDTH = 32
) (
    input clk,
    input rst,

    // 0 - 1 transition start capturing the data, overwritting all previous
    input              capture_start,
    output             capture_busy,
    input [WIDTH-1:0]  capture_data,
    input              capture_valid,
    output [_DEEP_BITS-1:0] capture_progress,

    // Async interface
    input [_DEEP_BITS-1:0] rb_addr,
    output [WIDTH-1:0]     rb_data
);

reg [_DEEP_BITS-1:0] wr_addr;
reg                  wr_active;

always @(posedge clk) begin
    if (rst) begin
        wr_active <= 1'b0;
    end else begin
        if (!wr_active) begin
            if (capture_start) begin
                wr_active <= 1'b1;
                wr_addr   <= 0;
            end
        end else begin
            if (capture_valid) begin
                wr_addr <= wr_addr + 1'b1;
                if (wr_addr == DEEP - 1) begin
                    wr_active <= 1'b0;
                end
            end
        end
    end
end

assign capture_busy = wr_active;

srl_ra #(
    .WIDTH(WIDTH),
    .DEEP(DEEP)
) srl (
    .clk(clk),
    .we(wr_active && capture_valid),
    .data_i(capture_data),

    .addr_i(rb_addr),
    .data_o(rb_data),
    .ce(1'b0),
    .rstq(1'b0),
    .dataq_o()
);

assign capture_progress = wr_addr;

endmodule
