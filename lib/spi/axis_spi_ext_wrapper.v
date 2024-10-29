module axis_spi_ext_wrapper #(
    parameter DATA_WIDTH = 32,
    parameter WR_ONLY = 0,
    parameter DATA_WIDTH_F = (DATA_WIDTH == 0) ? 32 : DATA_WIDTH,
    parameter CS_COUNT = 8,
    parameter _CS_COUNT_BITS = $clog2(CS_COUNT),
    parameter USE_FIFO = 0,
    parameter INVALID_DATA = 32'hcccccccc
)(
    input                       clk,
    input                       rst,

    input [31:0]                axis_cfg_data,
    input                       axis_cfg_valid,
    output                      axis_cfg_ready,

    input [31:0]                axis_tx_data,
    input                       axis_tx_valid,
    output                      axis_tx_ready,

    output [31:0]               axis_rx_data,
    output                      axis_rx_valid,
    input                       axis_rx_ready,

    output                      spi_mosi,
    input                       spi_miso,
    output                      spi_sclk,
    output [CS_COUNT-1:0]       spi_sen,

    output                      spi_interrupt_valid,
    input                       spi_interrupt_ready
);

localparam DATA_BITS = $clog2(DATA_WIDTH);


reg [7:0]                clk_div = 0;
reg [_CS_COUNT_BITS:0]   bus_no = 0;
reg [1:0]                bcount = 0; // 0 - 8; 1 - 16; 2 - 24; 3 - 32

wire [4:0]               transfer_zsz = { bcount, 3'b111 };

assign axis_cfg_ready = 1'b1;
always @(posedge clk) begin
    if (axis_cfg_ready && axis_cfg_valid) begin
        clk_div <= axis_cfg_data[7:0];
        bus_no  <= axis_cfg_data[8 + _CS_COUNT_BITS - 1:8];
        bcount  <= axis_cfg_data[15:14];
    end
end

generate
if (DATA_WIDTH > 0) begin
    wire                      res_v;
    wire                      res_r = 1'b1;
    wire [DATA_WIDTH_F - 1:0] res_d;

    axis_tag_spi #(
        .DATA_WIDTH(DATA_WIDTH),
        .NO_TRANSACT(1),
        .SCLK_RESET_HIGH(0),
        .DIV_WIDTH(8),
        .BUS_COUNT(CS_COUNT)
    ) spi_inst (
        .clk(clk),
        .rst(rst),
        .cfg_transfer_zsz(transfer_zsz),
        .cfg_div(clk_div),
        .cfg_spi_mode(2'd0),
        .cfg_bus(bus_no),
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
    assign spi_sen = {CS_COUNT{1'b1}};

    assign spi_interrupt_valid = 1'b0;
end
endgenerate

endmodule



