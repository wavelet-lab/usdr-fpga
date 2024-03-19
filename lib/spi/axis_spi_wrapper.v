// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module axis_spi_wrapper #(
    parameter DATA_WIDTH = 32,
    parameter WR_ONLY = 0,
    parameter FIXED_DIV = 2,
    parameter HOLD_SEN = 0,
    parameter USE_FIFO = 0,
    parameter DATA_WIDTH_F = (DATA_WIDTH == 0) ? 32 : DATA_WIDTH,
    parameter INVALID_DATA = 32'hcccccccc
)(
    input                       clk,
    input                       rst,

    input [31:0]                axis_tx_data,
    input                       axis_tx_valid,
    output                      axis_tx_ready,

    output [31:0]               axis_rx_data,
    output                      axis_rx_valid,
    input                       axis_rx_ready,

    output                      spi_mosi,
    input                       spi_miso,
    output                      spi_sclk,
    output                      spi_sen,

    output                      spi_interrupt_valid,
    input                       spi_interrupt_ready
);

localparam DATA_BITS = $clog2(DATA_WIDTH);

generate
if (DATA_WIDTH > 0) begin
        wire                      res_v;
        wire                      res_r = 1'b1;
        wire [DATA_WIDTH_F - 1:0] res_d;

        axis_tag_spi #(
            .DATA_WIDTH(DATA_WIDTH),
            .NO_TRANSACT(1),
            .SCLK_RESET_HIGH(0)
        ) spi_inst (
            .clk(clk),
            .rst(rst),
            .cfg_transfer_zsz(DATA_WIDTH - 1'b1),
            .cfg_div(FIXED_DIV),
            .cfg_spi_mode(2'd0),
            .cfg_bus(1'b0),
            .cfg_wo(1'b0),
            .axis_tx_data(axis_tx_data[DATA_WIDTH_F-1:0]),
            .axis_tx_valid(axis_tx_valid),
            .axis_tx_ready(axis_tx_ready),
            .axis_tx_id(),
            .axis_tx_last(1'b1),
            .axis_rx_data(res_d),
            .axis_rx_valid(res_v),
            .axis_rx_ready(res_r),
            .axis_rx_id(),
            .axis_rx_last(),
            .spi_mosi(spi_mosi),
            .spi_miso(spi_miso),
            .spi_sclk(spi_sclk),
            .spi_csn(spi_sen)
        );
        
        assign axis_rx_valid = 1'b1;
        if (USE_FIFO) begin
            axis_fifo #(.DEEP(16), .WIDTH(DATA_WIDTH)) res_fb (
                .clk(clk),
                .rst(rst),
                
                .s_rx_tdata(res_d),
                .s_rx_tvalid(res_v),
                .s_rx_tready(res_r),
                
                .m_tx_tdata(axis_rx_data),
                .m_tx_tvalid(),
                .m_tx_tready(axis_rx_ready),
                
                .fifo_used(),
                .fifo_full(),
                .fifo_empty()
            );
        end else begin
            reg [DATA_WIDTH_F - 1:0] res_d_reg;
            always @(posedge clk) begin
                if (res_v && res_r) begin
                    res_d_reg <= res_d;
                end
            end
            assign axis_rx_data = res_d_reg;
        end
  
        reg int_valid;
        always @(posedge clk) begin
            if (rst) begin
                int_valid <= 1'b0;
            end else begin
                if (int_valid && spi_interrupt_ready) begin
                    int_valid <= 1'b0;
                end
                
                if (res_v && res_r) begin
                    int_valid <= 1'b1;
                end
            end
        end
        assign spi_interrupt_valid = int_valid;
        
end else begin
    assign axis_tx_ready = 1'b1;
    assign axis_rx_valid = 1'b1;
    assign axis_rx_data  = INVALID_DATA;
    
    assign spi_mosi = 1'b1;
    assign spi_sclk = 1'b1;
    assign spi_sen = 1'b1;
    
    assign spi_interrupt_valid = 1'b0;
end
endgenerate

endmodule
