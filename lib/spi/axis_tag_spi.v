// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//

module axis_tag_spi #(
    parameter DATA_WIDTH = 32,
    parameter DATA_WBITS = $clog2(DATA_WIDTH),
    parameter DIV_WIDTH  = 8,
    parameter TAG_WIDTH  = 1,
    parameter BUS_COUNT  = 1,
    parameter BUS_WIDTH  = $clog2(BUS_COUNT) == 0 ? 1 : $clog2(BUS_COUNT),
    parameter [0:0] SCLK_RESET_HIGH = 1'b1,
    parameter [0:0] MOSI_RESET_HIGH = 1'b1,
    parameter [0:0] NO_TRANSACT     = 0
) (
    input clk,
    input rst,

    // configuration values latches
    input [DATA_WBITS-1:0]      cfg_transfer_zsz, // 0 means 1 cycle
    input [DIV_WIDTH-1:0]       cfg_div,
    input [1:0]                 cfg_spi_mode,
    input [BUS_WIDTH-1:0]       cfg_bus,
    input                       cfg_wo,

    input                       axis_tx_valid,
    output                      axis_tx_ready,
    input [DATA_WIDTH-1:0]      axis_tx_data,
    input [TAG_WIDTH-1:0]       axis_tx_id,
    input                       axis_tx_last, //last in burst

    output reg                  axis_rx_valid,
    input                       axis_rx_ready, // set to 1 to disable readback backpressure
    output reg [DATA_WIDTH-1:0] axis_rx_data,
    output reg [TAG_WIDTH-1:0]  axis_rx_id,
    output reg                  axis_rx_last,

    output reg                  spi_sclk,
    output reg [BUS_COUNT-1:0]  spi_csn,
    output reg                  spi_mosi,
    input                       spi_miso
);


// cfg_spi_mode
// SPI mode | CPOL | CPHA | 
// 0        | 0    | 0    | falling SCLK, and when CS activates rising SCLK
// 1        | 0    | 1    | rising SCLK falling SCLK
// 2        | 1    | 0    | rising SCLK, and when CS activates falling SCLK
// 3        | 1    | 1    | falling SCLK  rising SCLK

reg [TAG_WIDTH-1:0]  reg_tag;
reg [TAG_WIDTH-1:0]  reg_tag_bf;
reg                  reg_last;
reg                  reg_last_bf;

reg [DIV_WIDTH-1:0]  reg_div;
reg                  reg_spi_cpol;
reg                  reg_spi_cpha;
reg                  reg_wo;

reg                  spi_active;
reg                  spi_ready;

// SPI PHY, drives SCL and provides read/writes strobe
reg [DIV_WIDTH-1:0]  div_cnt;
reg                  div_pol;
reg                  spi_in_valid;
wire                 spi_out_ready    = (div_cnt == 0) && (reg_spi_cpha == div_pol);
wire                 spi_in_valid_pre = (div_cnt == 0) && (reg_spi_cpha != div_pol);

reg                  terminate_strobe;

always @(posedge clk) begin
  if (rst) begin
    spi_sclk <= SCLK_RESET_HIGH;
    div_pol  <= 0;
  end else begin
    if (!spi_active) begin
        div_pol <= 1'b0;
        div_cnt <= 0;
    end else begin
    
      if ((div_cnt == 0) ? !axis_rx_valid || axis_rx_ready : 1'b1) begin
        div_cnt <= div_cnt + 1'b1;
        if (div_cnt == reg_div) begin
            div_pol <= ~div_pol;
            div_cnt <= 0;
        end

        spi_in_valid  <= spi_in_valid_pre;
        
        if (div_cnt == 0 && (!terminate_strobe)) begin
            spi_sclk <= reg_spi_cpol ^ div_pol;      
        end
      end
    end
  end
end

reg                  reg_transaction;
reg [DATA_WIDTH-1:0] reg_wr_data;
reg [DATA_WBITS:0]   reg_transfer_sz;

reg                  nxt_read_valid;

assign recharge_strobe = spi_out_ready && (reg_transfer_sz == 0);
assign axis_tx_ready   = spi_ready || !NO_TRANSACT && reg_transaction && recharge_strobe;

always @(posedge clk) begin
  if (rst) begin
    spi_csn            <= {BUS_COUNT{1'b1}};
    spi_active         <= 1'b0;
    spi_ready          <= 1'b1;
    axis_rx_valid      <= 1'b0;
    reg_transaction    <= 1'b0;
    terminate_strobe   <= 1'b0;
    
    spi_mosi           <= MOSI_RESET_HIGH;
  end else begin
    
    if (axis_rx_valid && axis_rx_ready) begin
        axis_rx_valid <= 1'b0;
    end
    
    if (spi_active) begin 
        if (spi_out_ready) begin
            spi_mosi        <= reg_wr_data[DATA_WIDTH - 1];
            reg_wr_data     <= { reg_wr_data[DATA_WIDTH - 2:0], 1'b1 };
            reg_transfer_sz <= reg_transfer_sz - 1'b1;
            
            nxt_read_valid  <= recharge_strobe;
        end
        
        if (spi_in_valid) begin
            if (!reg_wo) begin
                axis_rx_data      <= { axis_rx_data[DATA_WIDTH - 2:0], spi_miso };
                
                if (nxt_read_valid) begin
                    axis_rx_last  <= reg_last;
                    axis_rx_valid <= 1'b1;
                    axis_rx_id    <= reg_tag;
                    if (!NO_TRANSACT) begin
                      reg_tag     <= reg_tag_bf;
                      reg_last    <= reg_last_bf;
                    end
                end
            end
        end
        
        if (!NO_TRANSACT && recharge_strobe) begin
            spi_ready <= reg_transaction;
        end
        
        if (reg_spi_cpha == 1'b0 && spi_out_ready && reg_transfer_sz[DATA_WBITS] || reg_spi_cpha == 1'b1 && spi_in_valid_pre && reg_transfer_sz[DATA_WBITS]) begin
            terminate_strobe <= 1'b1;
        end
        
        // Terminate transaction
        if ((reg_spi_cpha == 1'b0 && spi_in_valid_pre && reg_transfer_sz[DATA_WBITS] && !reg_transfer_sz[0] || reg_spi_cpha == 1'b1 && spi_out_ready && reg_transfer_sz[DATA_WBITS]) && (!reg_transaction || NO_TRANSACT)) begin
            spi_active <= 1'b0;
            spi_ready  <= 1'b1;
            spi_csn    <= {BUS_COUNT{1'b1}};
        end
        
    end
    
    if (axis_tx_valid && axis_tx_ready) begin
        reg_wr_data      <= axis_tx_data << (DATA_WIDTH - cfg_transfer_zsz - 1'b1);
        // spi_mosi         <= axis_tx_data[cfg_transfer_zsz];
        
        if (!NO_TRANSACT && reg_transaction && recharge_strobe) begin
          reg_tag_bf     <= axis_tx_id;
          reg_last_bf    <= axis_tx_last;
        end else begin
          reg_tag        <= axis_tx_id;
          reg_last       <= axis_tx_last;
        end
        
        if (~reg_transaction || NO_TRANSACT) begin
            reg_spi_cpol     <= cfg_spi_mode[1];
            reg_spi_cpha     <= cfg_spi_mode[0];
            reg_div          <= cfg_div;
            reg_wo           <= cfg_wo;
            
            spi_csn          <= BUS_COUNT == 1 ? 1'b0 : {BUS_COUNT{1'b1}} ^ (1'b1 << cfg_bus);
        end
        
        reg_transfer_sz  <= { 1'b0, cfg_transfer_zsz };
        spi_active       <= 1'b1;
        spi_ready        <= 1'b0;
        
        reg_transaction  <= ~axis_tx_last;
        terminate_strobe <= 1'b0;
        
    end
    
  end
end


endmodule
