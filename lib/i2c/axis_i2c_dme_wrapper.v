// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
//
// Compatibility wrapper with old software, replace by modern I2C ABI
//
module axis_i2c_dme_wrapper #(
    parameter I2C_SPEED = 100000,
    parameter BUS_SPEED = 125000000,
    parameter REPORT_ERROR = 1,
    parameter CLOCK_STRETCHING = 1,
    parameter STRETCH_TIMEOUT_BITS = 15
)(
    input clk,
    input rst,

    input  [1:0]  sda_in,
    input  [1:0]  scl_in,
    output [1:0]  sda_out_eo,
    output [1:0]  scl_out_eo,

    input [31:0]  addr_lut,

    input         s_cmd_valid,
    input [31:0]  s_cmd_data,
    output        s_cmd_ready,

    output        m_rb_valid,
    output [31:0] m_rb_data,
    input         m_rb_ready,

    input         m_int_ready,
    output reg    m_int_valid

);

localparam OLD_I2C_CMD_RDVAL_OFF = 31;
localparam OLD_I2C_CMD_RDSZ_OFF  = 28;
localparam OLD_I2C_CMD_WRSZ_OFF  = 26;
localparam OLD_I2C_CMD_DEVNO_OFF = 24;

// TX[31]    - RD_VALID 0 - only WR, 1 - RD cycle after WR
// TX[30:28] - RDZ_SZ (0 - 1 byte -- skip read cycle, .. 7 - 8 bytes (but only 4 last will be stored in reg))
// TX[27:26] - WR_SZ (0 - 0 byte -- skip write cycle, 1 - 1 bytes, .. 3 - 3 bytes)

// TX[25:24] - Device Address Select from LUT (WR byte 0 -- address)

// TX[23:16] - WR byte 3
// TX[15:8]  - WR byte 2
// TX[7:0]   - WR byte 1

wire       old_cmd_rdval  = s_cmd_data[OLD_I2C_CMD_RDVAL_OFF];
wire [2:0] old_cmd_rdzsz  = s_cmd_data[OLD_I2C_CMD_RDSZ_OFF + 2:OLD_I2C_CMD_RDSZ_OFF];
wire [1:0] old_cmd_wrsz   = s_cmd_data[OLD_I2C_CMD_WRSZ_OFF + 1:OLD_I2C_CMD_WRSZ_OFF];
wire [1:0] old_cmd_devsel = s_cmd_data[OLD_I2C_CMD_DEVNO_OFF + 1:OLD_I2C_CMD_DEVNO_OFF];

wire [7:0] i2c_addr_bus = (old_cmd_devsel == 2'b00) ? addr_lut[7:0] :
                          (old_cmd_devsel == 2'b01) ? addr_lut[15:8] :
                          (old_cmd_devsel == 2'b10) ? addr_lut[23:16] : addr_lut[31:24];

wire [1:0] sda_out_t;
wire [1:0] scl_out_t;

assign sda_out_eo = ~sda_out_t;
assign scl_out_eo = ~scl_out_t;

wire rx_valid;

axis_i2c_smaster #(
    .BUS_COUNT(2),
    .CLOCK_DIV( (BUS_SPEED / I2C_SPEED) / 4 ),
    .CLOCK_STRETCHING(CLOCK_STRETCHING),
    .STRETCH_TIMEOUT_BITS(STRETCH_TIMEOUT_BITS),
    .ERROR_SET(REPORT_ERROR),
    .ERROR_VALUE(32'hbaadbeef),
    .DATA_WR_COUNT(3),
    .DATA_RD_COUNT(4),
    .ABORT_ON_ERROR(1'b1),
    .NOTIFY_ALL(1'b0)
) i2c_master (
    .clk(clk),
    .rst(rst),

    .sda_i(sda_in),
    .sda_t(sda_out_t),
    .scl_i(scl_in),
    .scl_t(scl_out_t),

    .s_busno(i2c_addr_bus[7]),
    .s_wrlen({1'b0, old_cmd_wrsz}),
    .s_rdlen(old_cmd_rdval ? old_cmd_rdzsz + 1'b1 : 1'b0),
    .s_addr(i2c_addr_bus[6:0]),
    .s_data(s_cmd_data[23:0]),
    .s_valid(s_cmd_valid),
    .s_ready(s_cmd_ready),

    .m_rx_data(m_rb_data),
    .m_rx_valid(rx_valid),
    .m_rx_ready(1'b1)
);
assign m_rb_valid = 1'b1;


always @(posedge clk) begin
    if (rst) begin
        m_int_valid  <= 1'b0;
    end else begin
        if (m_int_valid && m_int_ready) begin
            m_int_valid <= 1'b0;
        end

        if (rx_valid) begin
            m_int_valid <= 1'b1;
        end
    end
end

endmodule
