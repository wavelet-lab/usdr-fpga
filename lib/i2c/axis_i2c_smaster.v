// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module axis_i2c_smaster #(
    parameter BUS_COUNT = 1,
    parameter BUS_BITS  = (BUS_COUNT == 1) ? 1 : $clog2(BUS_COUNT),
    parameter CLOCK_DIV = 256,
    parameter CLOCK_STRETCHING = 1'b1,

    parameter ERROR_SET     = 1'b1,
    parameter ERROR_VALUE   = 32'hdeadbeef,

    parameter DATA_WR_COUNT = 4,
    parameter DATA_WR_BITS  = $clog2(DATA_WR_COUNT),
    parameter DATA_RD_COUNT = 4,
    parameter DATA_RD_BITS  = $clog2(DATA_RD_COUNT),

    parameter ABORT_ON_ERROR = 1'b1,
    parameter NOTIFY_ALL = 1'b0,

    parameter STRETCH_TIMEOUT_BITS = 15
)(
    input clk,
    input rst,

    input  [BUS_COUNT-1:0] sda_i,
    output [BUS_COUNT-1:0] sda_t,
    input  [BUS_COUNT-1:0] scl_i,
    output [BUS_COUNT-1:0] scl_t,

    //
    input  [BUS_BITS-1:0]             s_busno,
    input  [DATA_WR_BITS:0]           s_wrlen,
    input  [DATA_RD_BITS:0]           s_rdlen,
    input  [6:0]                      s_addr,
    input  [DATA_WR_COUNT * 8 - 1:0]  s_data,
    input                             s_valid,
    output                            s_ready,


    output reg [DATA_RD_COUNT * 8 - 1:0]  m_rx_data,
    output reg                            m_rx_valid,
    input                                 m_rx_ready
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

wire rx_valid;
wire rx_data;
wire rx_user;

reg [2:0]  tx_data;
reg        tx_valid;
wire       tx_ready;

reg [BUS_BITS-1:0]            busno;
wire                          sda_t_sel;
wire                          scl_t_sel;

assign sda_t = ({BUS_COUNT{1'b1}} ^ (1'b1 << busno)) | (sda_t_sel << busno);
assign scl_t = ({BUS_COUNT{1'b1}} ^ (1'b1 << busno)) | (scl_t_sel << busno);


axis_i2c_sphy #(
    .CLOCK_DIV(CLOCK_DIV),
    .CLOCK_STRETCHING(CLOCK_STRETCHING),
    .STRETCH_TIMEOUT_BITS(STRETCH_TIMEOUT_BITS)
) phy (
    .clk(clk),
    .rst(rst),

    // I2C drive
    .sda_i(sda_i[busno]),
    .sda_t(sda_t_sel),
    .scl_i(scl_i[busno]),
    .scl_t(scl_t_sel),

    .s_tx_valid(tx_valid),
    .s_tx_ready(tx_ready),
    .s_tx_data(tx_data),

    .m_rx_valid(rx_valid),
    .m_rx_data(rx_data),
    .m_rx_user(rx_user)
);

wire rx_err_status = rx_valid && rx_user && rx_data;

always @(posedge clk) begin
    if (rst) begin
    end else begin
        if (rx_valid) begin
            if (rx_user) begin
                if (ERROR_SET && rx_data) begin
                    m_rx_data <= ERROR_VALUE;
                end
            end else begin
                m_rx_data <= { m_rx_data[DATA_RD_COUNT * 8 - 2:0], rx_data };
            end
        end
    end
end

reg [6:0]                     out_addr;
reg [3:0]                     bit_count;
wire                          is_bit9 = bit_count[3];

reg [7:0]                     tmp_data;
reg [DATA_WR_COUNT * 8 - 1:0] out_data;  // .....  0..7

reg [DATA_WR_BITS:0]          wrlen;
reg [DATA_RD_BITS:0]          rdlen;


localparam [2:0]
    ST_IDLE = 0,
    ST_START = 1,
    ST_ADDRW = 2,
    ST_DATAW = 3,
    ST_RSTART = 4,
    ST_ADDRR = 5,
    ST_DATAR = 6,
    ST_STOP = 7;
reg [2:0] state;

wire [DATA_WR_COUNT * 8 - 1:0]  s_data_reverse;
genvar i, j;
generate
    for (j = 0; j < DATA_WR_COUNT; j = j + 1) begin
        for (i = 0; i < 8; i = i + 1) begin
            assign s_data_reverse[j * 8 + (7 - i)] = s_data[j * 8 + i];
        end
    end
endgenerate

assign s_ready = (state == ST_IDLE) && ~tx_valid;

reg read_access;

always @(posedge clk) begin
    if (rst) begin
        busno      <= 0;
        tx_valid   <= 1'b0;
        state      <= ST_IDLE;
        m_rx_valid <= 1'b0;
    end else begin

        if (m_rx_valid && m_rx_ready) begin
            m_rx_valid <= 1'b0;
        end

        if (tx_valid && tx_ready) begin
            tx_valid <= 1'b0;
        end

        if (!tx_valid || tx_ready) begin
            case (state)

            ST_IDLE: begin
                if (s_valid && s_ready) begin
                    out_addr <= s_addr;
                    wrlen    <= s_wrlen;
                    rdlen    <= s_rdlen;
                    out_data <= s_data_reverse;
                    busno    <= s_busno;
                    state    <= ST_START;
                end
            end

            ST_START: begin
                tx_data   <= PHY_START;
                tx_valid  <= 1'b1;

                state     <= (wrlen == 0) ? ST_ADDRR : ST_ADDRW;

                bit_count   <= 4'd0;
                tmp_data    <= { out_addr, 1'b0 };

                read_access <= (rdlen != 0);
            end

            ST_ADDRW, ST_ADDRR: begin
                bit_count <= bit_count + 1'b1;

                if (is_bit9) begin
                    tx_data   <= PHY_ACK;
                    bit_count <= 4'd0;

                    state     <= (state == ST_ADDRW) ? ST_DATAW : ST_DATAR;
                end else begin
                    tx_data   <= tmp_data[7] ? PHY_D1 : PHY_D0;
                    tmp_data  <= { tmp_data[6:0], 1'b1 };
                end
                tx_valid      <= 1'b1;
            end

            ST_DATAW: begin
                bit_count <= bit_count + 1'b1;

                if (is_bit9) begin
                    tx_data   <= PHY_ACK;
                    bit_count <= 4'd0;
                    wrlen     <= wrlen - 1'b1;

                    if (wrlen == 1) begin
                        if (rdlen == 0) begin
                            state     <= ST_STOP;
                        end else begin
                            state     <= ST_RSTART;
                        end
                    end

                end else begin
                    tx_data  <= out_data[0] ? PHY_D1 : PHY_D0;
                    out_data <= { 1'b1, out_data[DATA_WR_COUNT * 8 - 1:1] };
                end

                tx_valid    <= 1'b1;
            end

            ST_RSTART: begin
                tx_data   <= PHY_RS;
                tx_valid  <= 1'b1;

                state     <= ST_ADDRR;

                bit_count <= 4'd0;
                tmp_data  <= { out_addr, 1'b1 };
            end

            ST_DATAR: begin
                bit_count <= bit_count + 1'b1;

                if (is_bit9) begin
                    tx_data   <= (rdlen == 1) ? PHY_D1 : PHY_D0; // Nak / Ack
                    bit_count <= 4'd0;
                    rdlen     <= rdlen - 1'b1;

                    if (rdlen == 1) begin
                        state      <= ST_STOP;
                    end
                end else begin
                    tx_data   <= PHY_RX;
                end

                tx_valid    <= 1'b1;
            end

            ST_STOP: begin
                tx_data    <= PHY_STOP;
                tx_valid   <= 1'b1;

                m_rx_valid <= NOTIFY_ALL || read_access;
                state      <= ST_IDLE;
            end

            endcase
        end

        if (ABORT_ON_ERROR && rx_err_status) begin
            state <= ST_STOP;
        end
    end
end


endmodule
