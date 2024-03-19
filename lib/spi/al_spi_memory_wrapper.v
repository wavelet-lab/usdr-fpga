// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
// ABI-compatibility module with old software
module al_spi_memory_wrapper #(
    parameter DUMMY_CYCLES_ESPI = 8,
    parameter DUMMY_CYCLES_QPI = 10,
    parameter CMD_STAT = 1,
    parameter FIXED_DIV = 0
)(
    input         clk,
    input         rst,

    input         flashspi_addr_valid,
    input  [31:0] flashspi_addr_data,
    output        flashspi_addr_ready,

    input         flashspi_cmd_valid,
    input  [31:0] flashspi_cmd_data,
    output        flashspi_cmd_ready,

    output        flashspi_rd_valid,
    output [31:0] flashspi_rd_data,
    input         flashspi_rd_ready,

    output        flashspi_stat_valid,
    output [31:0] flashspi_stat_data,
    input         flashspi_stat_ready,

    output [31:0]    m_mem_wdata,
    output [7:2]     m_mem_waddr,
    output           m_mem_wvalid,
    input            m_mem_wready,

    output [7:2]     m_mem_araddr,
    output           m_mem_arvalid,
    input            m_mem_arready,
        
    input [31:0]     m_mem_rdata,
    input            m_mem_rvalid,
    output           m_mem_rready,

    input   [3:0]    qphy_di,
    output  [3:0]    qphy_do,
    output  [3:0]    qphy_dt,
    output           qphy_dncs,
    output           qphy_clk
);

assign flashspi_stat_valid  = 1'b1;
assign flashspi_rd_valid    = 1'b1;


// Dummy clock ops
localparam FLASH_FR_1_1_1   = 8'h0B;  //    FAST READ 
localparam FLASH_FR4B_1_1_1 = 8'h0C;  //    4-BYTE FAST READ

// Flash SPI command          xxxxxxxx          1         10         1/0 
// [7:0] cmd | [7:0] size | [15:4] reserved | memop | [1:0] ecmdsz | wnr
//   cmd    -- FLASH command
//   size   -- memory transfer size 0 - no memory ops, 1 - 2 bytes, ... 255 - 256 bytes
//   memop  -- load or store data from RAM
//   ecmdsz -- extra command parameters 0 - none, 1 = 2 bytes, 2 = 3 bytes, 3 = 4 bytes
//   wnr    -- 0 - from SPI to RAM, 1 - from RAM to SPI

localparam WRNRD_OFF    = 0;
localparam EXCMDSZ_OFF  = 1;
localparam MEMVALID_OFF = 3;

localparam SZ_OFF       = 16;
localparam CMD_OFF      = 24;

reg [31:0] flashspi_addr_data_reg;
always @(posedge clk) begin
  if (rst) begin
    flashspi_addr_data_reg <= 0;
  end else begin
    if (flashspi_addr_valid && flashspi_addr_ready) begin
      flashspi_addr_data_reg <= flashspi_addr_data;
    end
  end
end

wire [7:0] flash_cmd = flashspi_cmd_data[CMD_OFF + 7:CMD_OFF];
wire [8:2] size_m1 = flashspi_cmd_data[SZ_OFF + 7:SZ_OFF + 2] - 1'b1;

wire      cmd_ready;
reg       cmd_valid;
reg [7:0] cmd_cmd;
reg [1:0] cmd_edata_wr_sz;
reg       cmd_edata_wr_valid;
reg       cmd_dummy_valid;
reg       cmd_mem_valid;
reg [7:2] cmd_mem_length;
reg       cmd_mem_wr;

// RD & not in memory and size > 0
wire register_read_mode = !flashspi_cmd_data[WRNRD_OFF] && !flashspi_cmd_data[MEMVALID_OFF] && (!size_m1[8] || flashspi_cmd_data[SZ_OFF + 2:SZ_OFF] != 2'b00);

reg [23:0] count;

always @(posedge clk) begin
  if (rst) begin
    // Precharge initialization
    count              <= ~0;
    
    cmd_valid          <= 1'b1;
    cmd_cmd            <= 8'h9f;
    cmd_mem_valid      <= 1'b0;
    cmd_mem_wr         <= 1'b0;
    cmd_dummy_valid    <= 1'b0;
    cmd_edata_wr_sz    <= 2'b10;
    cmd_edata_wr_valid <= 1'b1;
    
  end else begin
  
    if (cmd_valid && cmd_ready) begin
      cmd_valid <= 1'b0;
      count     <= count + 1'b1;
    end
  
    if (flashspi_cmd_valid && flashspi_cmd_ready) begin
      cmd_valid          <= 1'b1;
      cmd_cmd            <= flash_cmd;
      cmd_mem_length     <= size_m1[7:2];
      cmd_mem_valid      <= flashspi_cmd_data[MEMVALID_OFF] && ~size_m1[8];

      cmd_mem_wr         <= ~flashspi_cmd_data[WRNRD_OFF];
      cmd_edata_wr_sz    <= register_read_mode ? flashspi_cmd_data[SZ_OFF + 2:SZ_OFF] - 1'b1 :  flashspi_cmd_data[EXCMDSZ_OFF + 1:EXCMDSZ_OFF];
      cmd_edata_wr_valid <= register_read_mode ? 1'b1                                    :  (flashspi_cmd_data[EXCMDSZ_OFF + 1:EXCMDSZ_OFF] != 0);
      cmd_dummy_valid    <= (flash_cmd == FLASH_FR_1_1_1 || flash_cmd == FLASH_FR4B_1_1_1);
    end
  
  end
end

 
al_spi_memory #(
    .FIXED_DIV(FIXED_DIV)  // 0 = :2, 1 = :4, 2 = :6, etc.
) al_spi_memory (
    .clk(clk),
    .rst(rst),
    
    .s_edata(flashspi_addr_data_reg),
    
    // Command
    .s_valid(cmd_valid),
    .s_ready(cmd_ready),
    .s_cmd(cmd_cmd),
    .s_edata_wr_sz(cmd_edata_wr_sz),
    .s_edata_wr_valid(cmd_edata_wr_valid),
    .s_dummy_sz(4'd7),
    .s_dummy_valid(cmd_dummy_valid),
    
    .s_mem_valid(cmd_mem_valid),
    .s_mem_length(cmd_mem_length),
    .s_mem_wr(cmd_mem_wr),
    
    // Memory stream
    .m_mem_wdata({ m_mem_wdata[7:0], m_mem_wdata[15:8], m_mem_wdata[23:16], m_mem_wdata[31:24] }),
    .m_mem_waddr(m_mem_waddr),
    .m_mem_wvalid(m_mem_wvalid),
    .m_mem_wready(m_mem_wready),

    .m_mem_araddr(m_mem_araddr),
    .m_mem_arvalid(m_mem_arvalid),
    .m_mem_arready(m_mem_arready),
        
    .m_mem_rdata({ m_mem_rdata[7:0], m_mem_rdata[15:8], m_mem_rdata[23:16], m_mem_rdata[31:24] }),
    .m_mem_rvalid(m_mem_rvalid),
    .m_mem_rready(m_mem_rready),
    
    // SPI
    .spi_sclk(qphy_clk),
    .spi_csn(qphy_dncs),
    .spi_mosi(qphy_do[0]),
    .spi_miso(qphy_di[1])
);

// QSPI mapping to SPI
// IO0 - SI
// IO1 - SO
// IO2 - WP#
// IO3 - RESET#

assign qphy_dt      = 4'b0010;
assign qphy_do[2:1] = 2'b11;
assign qphy_do[3]   = !rst;

assign flashspi_rd_data    = m_mem_wdata;
assign flashspi_stat_data  = { CMD_STAT ? count : 24'h00_00_00, 7'h0, cmd_valid };

assign flashspi_addr_ready = !cmd_valid;
assign flashspi_cmd_ready  = !cmd_valid;


endmodule
