// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
//
module fe_simple_tx #(
    parameter TIMESTAMP_BITS = 48,
    parameter RAM_ADDR_WIDTH = 18,
    parameter DATA_BITS = 3,
    parameter DATA_WIDTH = 8 << DATA_BITS,
    parameter FE_IDX_BITS = 5,
    parameter SAMPLES_WIDTH = RAM_ADDR_WIDTH - 1,
    parameter FE_DESCR_WIDTH = TIMESTAMP_BITS + SAMPLES_WIDTH + (RAM_ADDR_WIDTH - DATA_BITS),
    parameter FRAME_LENGTH = 32, //Extra samples to flush beforehand to compensate distribution delay through extra pipeline stages
    parameter RAM_CHECK_BIT = 8,
    parameter SAMPLES_CHECK_BITS = (DATA_BITS - 1),
    parameter ID_WIDTH = 1 + SAMPLES_CHECK_BITS,
    parameter EXACT_SAMPLES_CHECK = 1,
    parameter INITIAL_TS_COMP = FRAME_LENGTH,
    parameter ASYNC_CLK = 1,  // clk & m_fedma_clk are async to each other
    parameter WITH_NOTS_BIT = 1
)(
    input clk,
    input rst,

    // Burst processed and is ready to be reused (clocked @m_fedma_clk)
    output                                m_proc_idx_valid,
    input                                 m_proc_idx_ready,

    output                                s_descr_ready,
    input                                 s_descr_valid,
    input  [FE_DESCR_WIDTH-1:0]           s_descr_data,

    // CC-stat
    input                                   m_fedma_clk,
    input                                   m_fedma_rst,
    output [TIMESTAMP_BITS - 1:0]           m_fedma_ts,         //Current TS to discard old buffer requests
    output [RAM_ADDR_WIDTH:RAM_CHECK_BIT]   m_fedma_ram_addr,   //Determines availability of RAM


    // Configurations
    input [1:0]                             cfg_mute,
    input                                   cfg_swap,
    input                                   cfg_format,

    output                                  sig_underrun,

    // FIFO RB interface
    output     [RAM_ADDR_WIDTH-1:DATA_BITS] m_fifo_araddr,
    output reg                              m_fifo_arvalid,
    input                                   m_fifo_arready,
    output reg  [ID_WIDTH-1:0]              m_fifo_arid,

    input [DATA_WIDTH-1:0]                  m_fifo_rdata,
    input                                   m_fifo_rvalid,
    input       [ID_WIDTH-1:0]              m_fifo_rid,
    output                                  m_fifo_rready,


    // DATA to DACs
    output reg [DATA_WIDTH-1:0]       dac_data,
    output reg                        dac_valid,
    input                             dac_ready,
    output                            dac_frame,  // LFMC clock
    input                             dac_sync    // crossing from 0->1 resets TX timer
);

localparam [TIMESTAMP_BITS-1:0] timestamp_advance = INITIAL_TS_COMP + 4;

// TX timer control
wire  [TIMESTAMP_BITS-1:0]    tx_timer;
assign                        dac_frame = tx_timer[$clog2(FRAME_LENGTH)];

reg  m_proc_idx_valid_clk;
wire m_proc_idx_ready_clk;
axis_opt_cc_fifo #(.CLK_CC(ASYNC_CLK), .DEEP(8), .NO_DATA(1'b1)) proc_idx_cc (
  .rx_clk(clk),
  .rx_rst(rst),
  .s_rx_tvalid(m_proc_idx_valid_clk),
  .s_rx_tready(m_proc_idx_ready_clk),

  .tx_clk(m_fedma_clk),
  .tx_rst(m_fedma_rst),
  .m_tx_tvalid(m_proc_idx_valid),
  .m_tx_tready(m_proc_idx_ready)
);

wire ram_addr_rst;

generate
if (ASYNC_CLK) begin
    wire dac_nsync_fedma_clk;

    cc_counter #(
        .WIDTH(TIMESTAMP_BITS),
        .GRAY_BITS(6) /*,
        .ASYNC_RESET(1) */
    ) cc_tx_timer (
        .in_clk(clk),
        .in_rst(!dac_sync),
        .in_increment( dac_valid && dac_ready ),
        .in_counter(tx_timer),

        .out_clk(m_fedma_clk),
        .out_rst(dac_nsync_fedma_clk),
        .out_counter(m_fedma_ts)
    );

    synchronizer #( .INIT(1) /*, .ASYNC_RESET(1) */ ) dac_sync_dma_cc (.clk(m_fedma_clk), .rst(1'b0), .a_in(!dac_sync), .s_out(dac_nsync_fedma_clk));
    synchronizer  dma_addr_rst_cc (.clk(m_fedma_clk), .rst(1'b0), .a_in(m_fedma_rst), .s_out(ram_addr_rst));
end else begin
    assign ram_addr_rst = m_fedma_rst;

    reg [TIMESTAMP_BITS-1:0] sync_tx_timer_reg;
    always @(posedge clk) begin
        if (!dac_sync) begin
            sync_tx_timer_reg <= 0;
        end else begin
            if (dac_valid && dac_ready) begin
                sync_tx_timer_reg <= sync_tx_timer_reg + 1'b1;
            end
        end
    end

    assign tx_timer   = sync_tx_timer_reg;
    assign m_fedma_ts = sync_tx_timer_reg;
end
endgenerate

reg [RAM_ADDR_WIDTH:DATA_BITS]       m_fifo_araddr_e;
reg [RAM_ADDR_WIDTH:RAM_CHECK_BIT]   m_fifo_araddr_ep;

wire m_fifo_addr_changed = m_fifo_araddr_ep[RAM_ADDR_WIDTH:RAM_CHECK_BIT] != m_fifo_araddr_e[RAM_ADDR_WIDTH:RAM_CHECK_BIT];
wire m_fifo_addr_changed_rdy;

always @(posedge clk) begin
    if (ram_addr_rst) begin
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
    .rx_rst(ram_addr_rst),

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
localparam FE_BYTES_OFF   = 0;
localparam FE_SAMPLES_OFF = FE_BYTES_OFF    + (RAM_ADDR_WIDTH - DATA_BITS);
localparam FE_TS_OFF      = FE_SAMPLES_OFF  + SAMPLES_WIDTH;


wire [TIMESTAMP_BITS-1:0]         descriptor_data_ts      = s_descr_data[FE_DESCR_WIDTH - 1:FE_TS_OFF];
wire [SAMPLES_WIDTH-1:0]          descriptor_data_samples = s_descr_data[FE_TS_OFF - 1     :FE_SAMPLES_OFF];
wire [RAM_ADDR_WIDTH-1:DATA_BITS] descriptor_data_bytes   = s_descr_data[FE_SAMPLES_OFF - 1:FE_BYTES_OFF];

reg                               descriptor_data_ts_reg_v;
wire                              descriptor_valid        = descriptor_data_ts_reg_v; //s_descr_valid;

reg                   state;
localparam [0:0]
    WAIT_TS  = 0,
    IN_BURST = 1;


reg [TIMESTAMP_BITS-1:0] descriptor_data_ts_reg;
wire [TIMESTAMP_BITS:0] timestamp_diff     = (descriptor_data_ts_reg - timestamp_advance) - tx_timer;
wire                    time_check         = timestamp_diff[TIMESTAMP_BITS];
reg                     time_check_valid;
reg                     time_check_prev;
wire                    time_notset        = WITH_NOTS_BIT && descriptor_data_ts_reg[TIMESTAMP_BITS-1];
wire                    time_check_late    = !time_notset && time_check_valid && (time_check_prev && time_check);
wire                    time_check_trigger = time_check_valid && (!time_check_prev && time_check || time_notset && dac_valid && dac_ready);

localparam DLY_CYCLES = 2;
reg [DLY_CYCLES-1:0]    time_check_trigger_pp;
wire                    time_check_trigger_data = time_check_trigger_pp[DLY_CYCLES - 1];
always @(posedge clk) begin
    if (!dac_sync) begin
        time_check_trigger_pp <= 0;
    end else begin
        time_check_trigger_pp <= { time_check_trigger_pp[DLY_CYCLES-2:0], time_check_trigger };
    end
end

reg [SAMPLES_WIDTH-1:0] burst_sample;
reg [31:0]              underruns;

// First request in a burst
reg [RAM_ADDR_WIDTH:DATA_BITS]  m_fifo_arbytes_rem;
wire                            m_fifo_arlast = (m_fifo_arbytes_rem == 0);

wire                    fifo_ar_sop      = (!m_fifo_arvalid || m_fifo_arvalid && m_fifo_arready && m_fifo_arlast);
wire                    descriptor_ready = fifo_ar_sop && (time_check_trigger || time_check_late);
assign                  s_descr_ready    = descriptor_ready;

reg [1:0]                    data_state;
reg [SAMPLES_CHECK_BITS-1:0] data_samples;
//reg [0:0]                    data_format;
wire [0:0]                   data_format;

// Supported data formats
localparam [0:0]
    DF_CI16_1 = 0,   // 1x Complex Chan 16bit
    DF_CI16_2 = 1;   // 2x Complex Chan 16bit
    //  DF_CI16_4    // 4x Complex Chan 16bit
    //  DF_CI16_8    // 8x Complex Chan 16bit
    //  DF_CI12_1
    //  DF_CI12_2
    //  DF_CI12_4
    //  DF_CI12_8
    //  DF_CI8_1
    //  DF_CI8_2
    //  DF_CI8_4
    //  DF_CI8_8

reg                             underrung_sig;


wire [31:0] data_dfci16_1 = (DATA_BITS == 4 && data_state == 2) ? m_fifo_rdata[95:64] :
                            (DATA_BITS == 4 && data_state == 3) ? m_fifo_rdata[127:96] :
                                                data_state == 1 ? m_fifo_rdata[63:32] :  m_fifo_rdata[31:0];
wire [63:0] data_dfci16_2 = (DATA_BITS == 4 && data_state == 1) ? m_fifo_rdata[127:64] : m_fifo_rdata[63:0];

always @(posedge clk) begin
    if (ram_addr_rst) begin
        state              <= 0;

        time_check_valid   <= 1'b0;
        time_check_prev    <= 1'b0;

        burst_sample       <= 0;
        underruns          <= 0;

        m_proc_idx_valid_clk   <= 1'b0;

        m_fifo_arvalid     <= 1'b0;

        underrung_sig      <= 1'b0;

        data_state         <= 0;
        data_samples       <= 0;

        dac_valid          <= 1'b0; //

        m_fifo_araddr_e    <= 0;

        descriptor_data_ts_reg_v <= 1'b0;

    end else begin
        underrung_sig      <= 1'b0;

        if (m_proc_idx_valid_clk && m_proc_idx_ready_clk) begin
            m_proc_idx_valid_clk <= 1'b0;
        end

        if (s_descr_valid && !descriptor_valid) begin
            descriptor_data_ts_reg   <= descriptor_data_ts;
            descriptor_data_ts_reg_v <= 1'b1;
        end

        if (descriptor_valid) begin ///////////////////////////////////////////////
            time_check_prev        <= time_check;
            time_check_valid       <= 1'b1;
            //descriptor_data_ts_reg <= descriptor_data_ts;
        end


        if (m_fifo_arvalid && m_fifo_arready) begin
            m_fifo_arid[0]      <= (m_fifo_arbytes_rem == 1'b1);
            m_fifo_araddr_e     <= m_fifo_araddr_e + 1'b1;
            m_fifo_arvalid      <= !m_fifo_arlast;
            m_fifo_arbytes_rem  <= m_fifo_arbytes_rem - 1'b1;
            if (m_fifo_arlast) begin
                m_proc_idx_valid_clk <= 1'b1;
            end
        end

        if (!time_check_valid && underrung_sig) begin
            m_proc_idx_valid_clk <= 1'b1;
        end

        if (descriptor_valid && descriptor_ready) begin
            descriptor_data_ts_reg_v <= 1'b0;

            time_check_valid   <= 1'b0;
            underrung_sig      <= time_check_late;

            if (time_check_late) begin
                // Ignore current burst
                m_fifo_araddr_e     <= m_fifo_araddr_e + descriptor_data_bytes + 1'b1;
                underruns           <= underruns + 1'b1;
            end else begin
                m_fifo_arid[0]               <= 1'b0; // EOP -indicator
                if (EXACT_SAMPLES_CHECK) begin
                    m_fifo_arid[SAMPLES_CHECK_BITS-1+1:1] <= descriptor_data_samples[SAMPLES_CHECK_BITS - 1:0];
                end
                m_fifo_arvalid               <= 1'b1;
                m_fifo_arbytes_rem           <= descriptor_data_bytes;
            end
        end

        // TODO latency compensator
        if (m_fifo_rvalid) begin
            if (dac_ready) begin

                data_state   <= data_state + 1'b1;
                if (EXACT_SAMPLES_CHECK) begin
                    data_samples <= data_samples + 1'b1;
                end

                case (data_format)
                DF_CI16_1: begin
                    if (DATA_BITS == 3 && data_state == 1)
                        data_state <= 0;

                    dac_data <= {(DATA_WIDTH / 32){data_dfci16_1}};
                end

                DF_CI16_2: begin
                    if (DATA_BITS == 3)
                        data_state <= 0;
                    else if (DATA_BITS == 4 && data_state == 1)
                        data_state <= 0;

                    dac_data[63:0] <= cfg_swap ? { data_dfci16_2[31:0], data_dfci16_2[63:32] } : data_dfci16_2;
                end
                endcase
            end

            if (m_fifo_rready) begin
                if (m_fifo_rid[0]) begin
                    data_state             <= 0;
                    if (EXACT_SAMPLES_CHECK) begin
                        data_samples       <= 0;
                    end
                end
            end
        end else begin
            if (dac_ready) begin
                dac_data    <= 0;
            end
        end

        if (cfg_mute[0]) begin
            dac_data[31:0] <= 0;
        end
        if ((DATA_WIDTH > 32) && cfg_mute[1]) begin
            dac_data[63:32] <= 0;
        end

        dac_valid       <= dac_sync;
    end
end


wire burst_samples_boundary = (EXACT_SAMPLES_CHECK && m_fifo_rid[0] && (m_fifo_rid[SAMPLES_CHECK_BITS-1+1:1] == data_samples[SAMPLES_CHECK_BITS - 1:0]));

assign m_fifo_rready   = dac_ready && (burst_samples_boundary || (
    DATA_BITS == 3 ?   ((data_format == DF_CI16_1 && data_state[0:0] == 1'd1) || (data_format == DF_CI16_2)) :
/*  DATA_BITS == 4 ?*/ ((data_format == DF_CI16_1 && data_state[1:0] == 2'd3) || (data_format == DF_CI16_2 && data_state[0:0] == 1'd1))));

assign m_fifo_araddr = m_fifo_araddr_e;
assign data_format   = cfg_format;
assign sig_underrun  = underrung_sig;

endmodule
