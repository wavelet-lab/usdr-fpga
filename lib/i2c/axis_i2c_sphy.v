// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module axis_i2c_sphy #(
    parameter CLOCK_DIV = 256,
    parameter CLOCK_DIV_BITS = $clog2(CLOCK_DIV),
    parameter CLOCK_STRETCHING = 1'b1,
    parameter STRETCH_TIMEOUT_BITS = 15
)(
    input clk,
    input rst,

    // I2C drive
    input      sda_i,
    output reg sda_t,
    input      scl_i, // For clock stretching
    output reg scl_t,

    //
    input       s_tx_valid,
    output      s_tx_ready,
    input [2:0] s_tx_data,

    // Data bit
    output      m_rx_valid,
    output      m_rx_data,
    output      m_rx_user
);

localparam [2:0]
    PHY_D0    = 3'd0,
    PHY_D1    = 3'd1,
    PHY_START = 3'd2,
    PHY_STOP  = 3'd3,
    PHY_ACK   = 3'd4,
    PHY_RX    = 3'd5,
    PHY_RS    = 3'd6,
    PHY_IDLE  = 3'd7;

localparam [1:0]
    CLOCK_PH_0   = 2'd0,
    CLOCK_PH_90  = 2'd1,
    CLOCK_PH_180 = 2'd2,
    CLOCK_PH_270 = 2'd3;

// Clock phases                ENCODED
//      0 1 2 3  ________
// SDA  ________/        X    0000 | 1111
//         ___  |   ___  |                 => D0 | D1  example encoding
// SCL  __/   \_|__/   \_|    0110 | 0110
//
//                               D0   D1  STRT STOP ACK   RX   RS  IDLE
localparam [31:0] SDA_ROM = 32'b0000_1111_1100_0011_1111_1111_1100_1111;
localparam [31:0] SCL_ROM = 32'b0110_0110_1110_0111_0110_0110_0110_1111;

wire        valid = s_tx_valid;
wire [2:0]  state = s_tx_data;
reg  [1:0]  phase;
reg         clk_strobe;
wire [4:0]  rom_idx;
reg         active;

assign rom_idx[4:2] = 3'd7 - state;
assign rom_idx[1:0] = 2'd3 - phase;

always @(posedge clk) begin
    if (rst) begin
        sda_t <= 1'b1;
        scl_t <= 1'b1;
    end else begin
        if (clk_strobe && active) begin
            sda_t <= SDA_ROM[rom_idx];
            scl_t <= SCL_ROM[rom_idx];
        end
    end
end

reg [STRETCH_TIMEOUT_BITS:0] watchdog;
reg [CLOCK_DIV_BITS-1:0] clkdiv;
always @(posedge clk) begin
    if (rst) begin
        clkdiv      <= 0;
        phase       <= 2'b0;
        clk_strobe  <= 1'b0;
        active      <= 1'b0;

    end else begin
        clk_strobe  <= 1'b0;
        clkdiv      <= clkdiv + 1'b1;
        if (clkdiv == (CLOCK_DIV - 1)) begin
            clkdiv     <= 0;

            if (CLOCK_STRETCHING && phase == CLOCK_PH_0) begin
                watchdog    <= 0;
            end

            phase      <= phase + 1'b1;
            clk_strobe <= 1'b1;

            if (CLOCK_STRETCHING && state == PHY_ACK && valid && phase == CLOCK_PH_180 && sda_i == 1'b0 && scl_i == 1'b0) begin
                watchdog   <= watchdog + 1'b1;

                if (!watchdog[STRETCH_TIMEOUT_BITS]) begin
                    phase      <= CLOCK_PH_180;
                    clk_strobe <= 1'b0;
                end
            end
        end

        if ((clkdiv == (CLOCK_DIV - 1)) && phase == CLOCK_PH_270) begin
            active   <= valid;
        end
    end
end

wire transfer_end = (phase == CLOCK_PH_270) && clk_strobe;
assign s_tx_ready = active && transfer_end;

assign m_rx_data  = sda_i || (CLOCK_STRETCHING && watchdog[STRETCH_TIMEOUT_BITS]);
assign m_rx_valid = transfer_end && valid && (state == PHY_RX || state == PHY_ACK);
assign m_rx_user  = (state == PHY_ACK);


endmodule
