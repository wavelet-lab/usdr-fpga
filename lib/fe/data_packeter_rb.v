// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2025 Wavelet Lab
//
// USDR PROJECT
//
//
// ID format
// { DAC_CMD_WIDTH , DATA_BITS, 2 }
//   id to pass for abort; last beat bytes count; more beats to come
module data_packeter_rb #(
    parameter       RAM_ADDR_WIDTH = 18,
    parameter [7:0] CHANS = 4,
    parameter DATA_WIDTH = 16 << CHANS,
    parameter FE_IDX_BITS = 5,
    parameter RAM_CHECK_BIT = 8,
    parameter ASYNC_CLK = 1,
    parameter DATA_BITS = $clog2(CHANS) + 1,
    parameter DAC_CMD_WIDTH = 1,
    parameter ID_WIDTH = DAC_CMD_WIDTH + DATA_BITS + 1
//    parameter PIPELINE_ABORT_READY = 1'b1
) (
    input clk,
    input rst,

    output                                  s_descr_ready,
    input                                   s_descr_valid,
    input  [RAM_ADDR_WIDTH+1-DATA_BITS:0]   s_descr_addr,
    input  [RAM_ADDR_WIDTH-1:0]             s_descr_data,

    // FIFO RB interface
    output     [RAM_ADDR_WIDTH-1:DATA_BITS] m_fifo_araddr,
    output reg                              m_fifo_arvalid,
    input                                   m_fifo_arready,
    output reg  [ID_WIDTH-1:0]              m_fifo_arid,

    input  [DAC_CMD_WIDTH-1:0]              s_abort_id,
    input                                   s_abort_valid,

    // CC-stat
    input                                   m_fedma_clk,
    input                                   m_fedma_rst,
    output [RAM_ADDR_WIDTH:RAM_CHECK_BIT]   m_fedma_ram_addr,   //Determines availability of RAM

    output                                  m_fedma_idx_valid,
    input                                   m_fedma_idx_ready
);


reg  m_proc_idx_valid_clk;
wire m_proc_idx_ready_clk;
axis_opt_cc_fifo #(.CLK_CC(ASYNC_CLK), .DEEP(8), .NO_DATA(1'b1)) proc_idx_cc (
  .rx_clk(clk),
  .rx_rst(rst),
  .s_rx_tvalid(m_proc_idx_valid_clk),
  .s_rx_tready(m_proc_idx_ready_clk),

  .tx_clk(m_fedma_clk),
  .tx_rst(m_fedma_rst),
  .m_tx_tvalid(m_fedma_idx_valid),
  .m_tx_tready(m_fedma_idx_ready)
);
////////////////////////////////////////////////////////////////////////////////

wire ram_addr_dac_clk_rst;

generate
if (ASYNC_CLK) begin
    synchronizer  dma_addr_rst_cc (.clk(clk), .rst(1'b0), .a_in(m_fedma_rst), .s_out(ram_addr_dac_clk_rst));
end else begin
    assign ram_addr_dac_clk_rst = m_fedma_rst;
end
endgenerate
////////////////////////////////////////////////////////////////////////////////

reg [RAM_ADDR_WIDTH:DATA_BITS]       m_fifo_araddr_e;
reg [RAM_ADDR_WIDTH:RAM_CHECK_BIT]   m_fifo_araddr_ep;

assign m_fifo_araddr = m_fifo_araddr_e[RAM_ADDR_WIDTH-1:DATA_BITS];

wire m_fifo_addr_changed = m_fifo_araddr_ep[RAM_ADDR_WIDTH:RAM_CHECK_BIT] != m_fifo_araddr_e[RAM_ADDR_WIDTH:RAM_CHECK_BIT];
wire m_fifo_addr_changed_rdy;

always @(posedge clk) begin
    if (ram_addr_dac_clk_rst) begin
        m_fifo_araddr_ep <= 0;
    end else begin
        if (m_fifo_addr_changed && m_fifo_addr_changed_rdy) begin
            m_fifo_araddr_ep <= m_fifo_araddr_e[RAM_ADDR_WIDTH:RAM_CHECK_BIT];
        end
    end
end

reg  [RAM_ADDR_WIDTH:RAM_CHECK_BIT] fedma_ram_addr_r;
wire [RAM_ADDR_WIDTH:RAM_CHECK_BIT] fedma_ram_addr_data;
wire                                fedma_ram_addr_ready = 1'b1;
wire                                fedma_ram_addr_valid;
assign                              m_fedma_ram_addr = fedma_ram_addr_r;
axis_opt_cc_fifo #( .CLK_CC(ASYNC_CLK), .CC_DATA_PIPELINED(0), .WIDTH(RAM_ADDR_WIDTH - RAM_CHECK_BIT + 1)) addr_check_c (
    .rx_clk(clk),
    .rx_rst(ram_addr_dac_clk_rst),

    .s_rx_tdata(m_fifo_araddr_e[RAM_ADDR_WIDTH:RAM_CHECK_BIT]),
    .s_rx_tvalid(m_fifo_addr_changed),
    .s_rx_tready(m_fifo_addr_changed_rdy),

    .tx_clk(m_fedma_clk),
    .tx_rst(m_fedma_rst),

    .m_tx_tdata(fedma_ram_addr_data),
    .m_tx_tvalid(fedma_ram_addr_valid),
    .m_tx_tready(fedma_ram_addr_ready)
);

always @(posedge m_fedma_clk) begin
    if (m_fedma_rst) begin
        fedma_ram_addr_r <= 0;
    end else begin
        if (fedma_ram_addr_ready && fedma_ram_addr_valid) begin
            fedma_ram_addr_r <= fedma_ram_addr_data;
        end
    end
end

////////////////////////////////////////////////////////////////////////////////


// First request in a burst
reg [RAM_ADDR_WIDTH - 1:DATA_BITS]  m_fifo_arbytes_rem;
wire                                m_fifo_last_req = (m_fifo_arbytes_rem == 1);


reg [DAC_CMD_WIDTH-1:0]          r_brst_id;
reg [DAC_CMD_WIDTH-1:0]          p_brst_id;

wire fail_burst = (p_brst_id == s_abort_id && s_abort_valid);

reg late_cnf;
assign s_descr_ready = (!m_fifo_arvalid && !late_cnf) || (m_fifo_arready && (m_fifo_arid[0]));

reg [39:0] stat_total_fe_req;
reg [39:0] stat_total_fe_dis;


always @(posedge clk) begin
    if (rst) begin
        m_proc_idx_valid_clk   <= 1'b0;

        m_fifo_arvalid         <= 1'b0;
        m_fifo_araddr_e        <= 0;

        r_brst_id              <= 0;
        p_brst_id              <= 0;

        late_cnf               <= 1'b0;

        stat_total_fe_req <= 0;
        stat_total_fe_dis <= 0;
    end else begin
        if (m_proc_idx_valid_clk && m_proc_idx_ready_clk) begin
            m_proc_idx_valid_clk <= 1'b0;
        end

        if (m_fifo_arvalid && m_fifo_arready) begin
            m_fifo_arid[0]      <= m_fifo_last_req;

            m_fifo_araddr_e     <= m_fifo_araddr_e + 1'b1;
            m_fifo_arvalid      <= !(m_fifo_arid[0]);
            m_fifo_arbytes_rem  <= m_fifo_arbytes_rem - 1'b1;

            if (m_fifo_arid[0] || fail_burst) begin
                if (m_fifo_arid[0]) begin
                    m_proc_idx_valid_clk <= 1'b1;
                    p_brst_id            <= p_brst_id + 1'b1;
                end
                if (fail_burst) begin
                    // End transfer
                    m_fifo_arid[0]     <= 1'b1;
                    m_fifo_araddr_e    <= m_fifo_araddr_e + m_fifo_arbytes_rem + 1'b1;
                    m_fifo_arbytes_rem <= 0;
                end
            end
        end

        if (late_cnf && m_proc_idx_ready_clk) begin
            m_proc_idx_valid_clk <= 1'b1;
            late_cnf             <= 1'b0;
        end

        if (s_descr_valid && s_descr_ready) begin
            m_fifo_arid[0]                        <= 1'b0; // 1+ beats
            m_fifo_arid[ID_WIDTH-1:1]             <= { r_brst_id, s_descr_data[DATA_BITS-1:0] };

            m_fifo_arvalid                        <= !s_descr_addr[0]; //1'b1;
            m_fifo_arbytes_rem                    <= s_descr_data[RAM_ADDR_WIDTH - 1:DATA_BITS];

            m_fifo_araddr_e                       <= s_descr_addr[RAM_ADDR_WIDTH+1-DATA_BITS:1];

            if (!s_descr_addr[0]) begin
                r_brst_id                         <= r_brst_id + 1'b1;
            end
            late_cnf                              <= s_descr_addr[0];

            stat_total_fe_req <= stat_total_fe_req + 1'b1;
            if (s_descr_addr[0]) begin
                stat_total_fe_dis <= stat_total_fe_dis + 1'b1;
            end
        end
    end
end

endmodule
