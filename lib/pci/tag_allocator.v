// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module tag_allocator #(
    parameter PCIE_TAG_BITS = 5  //32 TAGs in total by default
)(
    input clk,
    input rst,

    output core_ready,
    output notags,
    output [PCIE_TAG_BITS:0]   tag_fifo_used,

    output [PCIE_TAG_BITS-1:0] m_tag_alloc_data,
    input                      m_tag_alloc_ready,
    output                     m_tag_alloc_valid,

    input [PCIE_TAG_BITS-1:0]  s_tag_free_data,
    input                      s_tag_free_valid,
    output                     s_tag_free_ready
);

reg [PCIE_TAG_BITS:0] reset_idx;
wire                  tag_reset_ready = reset_idx[PCIE_TAG_BITS];
always @(posedge clk) begin
    if (rst) begin
        reset_idx <= 0;
    end else begin
        if (!tag_reset_ready) begin
            reset_idx <= reset_idx + 1'b1;
        end
    end
end
assign core_ready = tag_reset_ready;
assign notags     = (tag_fifo_used == (1 << PCIE_TAG_BITS));

axis_fifo #(.DEEP(1<<PCIE_TAG_BITS), .WIDTH(PCIE_TAG_BITS), .EXTRA_REG(1'b1)) tag_allocator_fifo (
    .clk(clk),
    .rst(rst),

    .s_rx_tdata(tag_reset_ready ?   s_tag_free_data  : reset_idx[PCIE_TAG_BITS-1:0]),
    .s_rx_tvalid(tag_reset_ready ?  s_tag_free_valid : 1'b1),
    .s_rx_tready(s_tag_free_ready),

    .m_tx_tdata(m_tag_alloc_data),
    .m_tx_tvalid(m_tag_alloc_valid),
    .m_tx_tready(m_tag_alloc_ready),

    .fifo_full(),
    .fifo_used(tag_fifo_used)
);

endmodule
