// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module al_spi_memory #(
    parameter [7:0] FIXED_DIV = 1
)(
    input clk,
    input rst,
    
    // Extended data (address & special parameters)
    input [31:0] s_edata,
    
    // Command
    input        s_valid,
    output       s_ready,
    input [7:0]  s_cmd,
    input [1:0]  s_edata_wr_sz,
    input        s_edata_wr_valid,
    input [3:0]  s_dummy_sz,
    input        s_dummy_valid,
    
    input        s_mem_valid,
    input [7:2]  s_mem_length,
    input        s_mem_wr,
    
    //Memory interface
    output [31:0] m_mem_wdata,
    output [7:2]  m_mem_waddr,
    output        m_mem_wvalid,
    input         m_mem_wready,

    output [7:2]  m_mem_araddr,
    output        m_mem_arvalid,
    input         m_mem_arready,
        
    input [31:0]  m_mem_rdata,
    input         m_mem_rvalid,
    output        m_mem_rready,
    
    //SPI
    output        spi_sclk,
    output        spi_csn,
    output        spi_mosi,
    input         spi_miso
);

localparam [2:0]
    ST_IDLE_CMD = 0,
    ST_EDATA = 1,
    ST_DUMMY = 2,
    ST_MEM_READ = 3,
    ST_MEM_WRITE = 4,
    ST_DONE = 5,
    ST_MEM_READ_DONE = 6,
    ST_ACK = 7;
reg [2:0] state;

reg [7:2] rem_length;
reg [7:2] rem_arlength;
reg [7:2] addr;

wire      transfer_last = (rem_length == 0);

wire      rx_valid;
wire      rx_ready;
wire      rx_last;
wire      rx_mem_op;

// 1:last | 5:size | 32:data
wire [32 + 5 + 1 - 1:0]   spi_out_data = 
    (state == ST_IDLE_CMD) ? { !s_edata_wr_valid && !s_dummy_valid && !s_mem_valid ? 1'b1 : 1'b0, 5'd7,                     { s_edata[31:8], s_cmd } } :
    (state == ST_EDATA)    ? { !s_dummy_valid && !s_mem_valid ? 1'b1 : 1'b0,                    { s_edata_wr_sz, 3'b111 },    s_edata } :
    (state == ST_DUMMY)    ? { !s_mem_valid ? 1'b1 : 1'b0,                                        s_dummy_sz,                 32'hffffffff } :
    (state == ST_MEM_READ) ? { transfer_last,                                                     5'd31,                      m_mem_rdata } :
                             { transfer_last,                                                     5'd31,                      32'hffffffff };

wire      spi_out_ready;
wire      spi_out_valid = 
    (state == ST_DONE || state == ST_ACK)               ? 1'b0 : 
    (state == ST_MEM_READ || state == ST_MEM_READ_DONE) ? m_mem_rvalid :
    (state == ST_MEM_WRITE)                             ? !m_mem_wvalid : s_valid; // s_valid for ST_IDLE_CMD, ST_EDATA, ST_DUMMY

assign    m_mem_rready    = (state == ST_MEM_READ || state == ST_MEM_READ_DONE) ? spi_out_ready : 1'b0;
assign    m_mem_wvalid    = rx_valid && rx_mem_op;

assign    rx_ready        = rx_valid && (rx_mem_op ? m_mem_wready : 1'b1);

assign    m_mem_waddr     = addr;
assign    m_mem_araddr    = addr;
assign    m_mem_arvalid   = (state == ST_MEM_READ);

always @(posedge clk) begin
    if (rst) begin
        state         <= ST_IDLE_CMD;
    end else begin
    
        if (m_mem_wvalid && m_mem_wready) begin
            addr       <= addr       + 1'b1;
        end
        
        case (state)
        ST_IDLE_CMD: begin
            if (s_valid) begin
                rem_length   <= s_mem_length;
                rem_arlength <= s_mem_length;
                addr         <= 0;
                
                if (s_edata_wr_valid) begin
                    state <= ST_EDATA;
                end else if (s_dummy_valid) begin
                    state <= ST_DUMMY;
                end else if (s_mem_valid) begin
                    state <= (s_mem_wr) ? ST_MEM_WRITE : ST_MEM_READ;
                end else begin
                    state <= ST_DONE;
                end
            end
        end
        
        ST_EDATA: begin
            if (spi_out_valid && spi_out_ready) begin
                if (s_dummy_valid) begin
                    state <= ST_DUMMY;
                end else if (s_mem_valid) begin
                    state <= (s_mem_wr) ? ST_MEM_WRITE : ST_MEM_READ;
                end else begin
                    state <= ST_DONE;
                end
            end
        end
        
        ST_DUMMY: begin
            if (spi_out_valid && spi_out_ready) begin
                if (s_mem_valid) begin
                    state <= (s_mem_wr) ? ST_MEM_WRITE : ST_MEM_READ;
                end else begin
                    state <= ST_DONE;
                end
            end
        end
        
        ST_MEM_WRITE: begin
            if (spi_out_valid && spi_out_ready) begin
                if (transfer_last) begin
                    state  <= ST_DONE;
                end
                rem_length <= rem_length - 1'b1;
            end
        end
        
        ST_MEM_READ, ST_MEM_READ_DONE: begin
            if (m_mem_arvalid && m_mem_arready) begin
                rem_arlength <= rem_arlength - 1'b1;
                addr         <= addr         + 1'b1;
                
                if (rem_arlength == 0) begin
                    state    <=  ST_MEM_READ_DONE;
                end
            end
        
            if (m_mem_rvalid && m_mem_rready) begin
                if (transfer_last) begin
                    state    <= ST_DONE;
                end
                rem_length   <= rem_length   - 1'b1;
            end
        end
        
        ST_DONE: begin
            if (spi_out_ready) begin
                state   <= ST_ACK;
            end
        end
        
        ST_ACK: begin
            state   <= ST_IDLE_CMD;
        end
        
        endcase
    end
end

assign s_ready = (state == ST_ACK);

axis_tag_spi #(
    .DATA_WIDTH(32),
    .NO_TRANSACT(0),
    .SCLK_RESET_HIGH(0)
) spi_inst (
    .clk(clk),
    .rst(rst),
    
    .cfg_transfer_zsz(spi_out_data[32 + 5 - 1:32]),
    .cfg_div(FIXED_DIV),
    .cfg_spi_mode(2'd0),
    .cfg_bus(1'b0),
    .cfg_wo(1'b0),
    
    .axis_tx_data(spi_out_data[31:0]),
    .axis_tx_valid(spi_out_valid),
    .axis_tx_ready(spi_out_ready),
    .axis_tx_id(state == ST_MEM_WRITE),
    .axis_tx_last(spi_out_data[32 + 5]),
    
    .axis_rx_data(m_mem_wdata),
    .axis_rx_valid(rx_valid),
    .axis_rx_ready(rx_ready),
    .axis_rx_id(rx_mem_op),
    .axis_rx_last(rx_last),
    
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_sclk(spi_sclk),
    .spi_csn(spi_csn)
);

endmodule
