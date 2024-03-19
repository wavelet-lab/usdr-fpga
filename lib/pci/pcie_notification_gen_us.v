// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module pcie_notification_gen_us #(
    parameter EVENT_WIDTH = 32,
    parameter INT_NUM_BITS = 2,
    parameter ORIG_NUM_BITS = INT_NUM_BITS,
    parameter BUCKET_BITS = 1,
    parameter DATA_BITS = 4, // 3 - 64 bit, 4 - 128 bit, 5 - 256 bit 
    parameter DATA_WIDTH_ = 8 << DATA_BITS,
    parameter ULTRA_SCALE = 0,
    parameter KEEP_WIDTH_ = DATA_WIDTH_/32,
    parameter USER_WIDTH_ = ULTRA_SCALE ? 62 : 1,
    parameter EN64BIT = 0 // for 7-Series
)(
    input clk,
    input rst,

    input [1:0]                     cfg_pcie_attr,
    input [15:0]                    cfg_pcie_reqid,

    // PCIe TLP
    input                           m_axis_tx_tready,
    output [DATA_WIDTH_-1:0]        m_axis_tx_tdata,
    output [KEEP_WIDTH_-1:0]        m_axis_tx_tkeep,
    output                          m_axis_tx_tlast,
    output                          m_axis_tx_tvalid,
    output [USER_WIDTH_-1:0]        m_axis_tx_tuser,

    // Interrupt logic generator
    output     [INT_NUM_BITS - 1:0] m_int_data,
    output                          m_int_valid,
    input                           m_int_ready,

    // Configuration channel
    input [31:0]                    s_cfg_data,
    input                           s_cfg_valid,
    output                          s_cfg_ready,

    // ACK channel
    input [31:0]                    s_ack_data,
    input                           s_ack_valid,
    output                          s_ack_ready,

    // Event nfo
    input [EVENT_WIDTH - 1:0]       s_evd_data,
    input                           s_evd_valid,
    input                           s_evd_last,
    output                          s_evd_ready
);
// All records aligned to 128bit
wire [31:2]         bus_addr;

wire  cfg_addr_we    = s_cfg_valid && s_cfg_ready;
wire  bus_ready;
wire  bus_valid;
wire  update_addr_we = bus_valid && bus_ready;
wire  req_valid_asel = s_evd_valid;
 
wire [BUCKET_BITS - 1:0]   evd_bucket_no      = s_evd_data[BUCKET_BITS - 1 + 6:6];
assign s_cfg_ready = !s_evd_valid;

//TODO: optional bucket size, now it's fixed to 4KiB
//TODO: 64bit addr 
ram_sxp #(.DATA_WIDTH(20), .ADDR_WIDTH(BUCKET_BITS), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b0)) dmacfg_c (
    .wclk(clk),
    .we(cfg_addr_we),
    .waddr(req_valid_asel ? evd_bucket_no : s_cfg_data[BUCKET_BITS-1:0]),
    .wdata(s_cfg_data[31:12]),
    .raddr(),
    .rdata(bus_addr[31:12])
);

localparam ADDR_WIDTH = 12;
wire [ADDR_WIDTH:4] stored_addr;
wire [ADDR_WIDTH:4] upd_addr = req_valid_asel ? stored_addr + 1'b1 : 0;

ram_sxp #(.DATA_WIDTH(ADDR_WIDTH + 1 - 4), .ADDR_WIDTH(BUCKET_BITS), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b0)) dmacfg_w (
    .wclk(clk),
    .we(cfg_addr_we || update_addr_we),
    .waddr(req_valid_asel ? evd_bucket_no : s_cfg_data[BUCKET_BITS-1:0]),
    .wdata(upd_addr),
    .raddr(),
    .rdata(stored_addr)
);

assign bus_addr[11:2] = { stored_addr[11:4], 2'b00 };

// Flow control: it's 4096/16 = 256 events in notification host RAM. confirmed_addr is wrapped with 1 extra bit
// to control read / write pointers overlap. It's no need to confirm every event being read but should be done
// after several messages read to eliminate event stalls. Interrupt is fired only after the event is dispatched
wire [ADDR_WIDTH:4] confirmed_addr;
assign s_ack_ready = 1'b1;
 
ram_sxp #(.DATA_WIDTH(ADDR_WIDTH + 1 - 4), .ADDR_WIDTH(BUCKET_BITS), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b1)) dmacfg_ack (
    .wclk(clk),
    .we(s_ack_valid && s_ack_ready),
    .waddr(s_ack_data[16+4:16]),
    .wdata(s_ack_data[9:1]),
    .raddr(evd_bucket_no),
    .rdata(confirmed_addr)
);

wire [ADDR_WIDTH:4] delta = stored_addr - confirmed_addr;
wire                st_ovf = delta[ADDR_WIDTH];

reg s_evd_sop; // First beat of data
always @(posedge clk) begin
    if (rst) begin
        s_evd_sop <= 1'b1;
    end else if (s_evd_valid && s_evd_ready) begin
        s_evd_sop <= s_evd_last;
    end
end

reg ntfy_slots_full; // First beat and fifo size ok to push
reg ovf_flag;
reg inpacket;

always @(posedge clk) begin
    if (rst) begin
        ntfy_slots_full <= 1'b1;
        ovf_flag        <= 1'b0;
        inpacket        <= 1'b0;
    end else begin
        ntfy_slots_full <= !s_evd_sop || !s_evd_valid || s_evd_ready || st_ovf;
        
        inpacket        <= s_evd_valid && s_evd_ready && s_evd_last ? 1'b0 : s_evd_valid;
        if (inpacket == 1'b0 && s_evd_valid) begin
            ovf_flag    <= stored_addr[12];
        end
    end
end
// Add non-pipelined version of ntfy_slots_full & ovf_flag to reduce latency

reg s_evd_req_valid; // Bus request processed
assign bus_valid = ~ntfy_slots_full && s_evd_req_valid;

always @(posedge clk) begin
    if (rst) begin
        s_evd_req_valid <= 1'b1;
    end else if (bus_valid && bus_ready) begin
        s_evd_req_valid <= 1'b0;
    end else if (s_evd_valid && s_evd_ready && s_evd_last) begin
        s_evd_req_valid <= 1'b1;
    end
end

// First packet in event
// b31 .. b16  seqno
// b15 .. b12  eventsz
// b11 .. b6   aux
// b5  .. b0   eventno

wire                     addr_valid;
wire                     addr_last;

reg [DATA_WIDTH_-1:0]    rb_data;
reg                      rb_valid;
reg                      rb_id;
reg [DATA_BITS-1-2:0]    rb_widx;
reg                      rb_last;
wire                     rb_ready;

assign s_evd_ready       =  !s_evd_req_valid && addr_valid && (!rb_valid || rb_ready);

always @(posedge clk) begin
    if (rst) begin
        rb_valid <= 1'b0;
        rb_widx  <= 0;
    end else begin
        if (rb_valid && rb_ready) begin
            rb_valid <= 1'b0;
        end
        
        if (s_evd_valid && s_evd_ready) begin
            rb_widx <= rb_widx + 1'b1;
            rb_last <= addr_last;
            
            if (rb_widx == 0) begin
                rb_data[31:0]   <= s_evd_sop ? { ovf_flag, s_evd_data[30:0] } : s_evd_data;
            end else if (rb_widx == 1) begin
                rb_data[63:32]  <= s_evd_data;
            end else if (DATA_BITS >= 4 && rb_widx == 2) begin
                rb_data[95:64]  <= s_evd_data;
            end else if (DATA_BITS >= 4) begin
                rb_data[127:96] <= s_evd_data;
            end
            
            if (s_evd_last || (DATA_BITS == 3 ? rb_widx == 1 : rb_widx == 3)) begin
                rb_valid <= 1'b1;
                rb_widx  <= 0;
            end
        end
        
    end
end

wire [4-DATA_BITS:0]      bus_sz      = (DATA_BITS >= 4) ? 0 : s_evd_data[13:12] < 2 ? 0 : 1;
wire [INT_NUM_BITS - 1:0] bus_intno   = s_evd_data[INT_NUM_BITS - 1 + BUCKET_BITS + 6:BUCKET_BITS + 6];

wire                      req_valid;
wire                      req_ready;
wire [31:DATA_BITS]       req_bus_addr;
wire [4-DATA_BITS:0]      req_sendsz;
wire [INT_NUM_BITS - 1:0] req_intno;

axis_opt_pipeline #(
  .WIDTH(32 - DATA_BITS + INT_NUM_BITS + 5 - DATA_BITS),
  .PIPELINE(DATA_BITS < 4)
) addr_pipe (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata({bus_intno, bus_sz, bus_addr[31:DATA_BITS]}),
  .s_rx_tvalid(bus_valid),
  .s_rx_tready(bus_ready),

  .m_tx_tdata({req_intno, req_sendsz, req_bus_addr}),
  .m_tx_tvalid(req_valid),
  .m_tx_tready(req_ready)
);

wire valid_interrupt = (~m_int_data != 0);
wire event_sent;
assign m_int_valid = event_sent && valid_interrupt;

al_ram_to_pcie_memwr #(
    .LOCAL_ADDR_WIDTH(DATA_BITS + 1),
    .MEM_TAG(INT_NUM_BITS),
    .REQUEST_LEN_BITS(1),
    .ULTRA_SCALE(ULTRA_SCALE),
    .DATA_BITS(DATA_BITS),
    .EN64BIT(EN64BIT)
) memwr (
    .clk(clk),
    .rst(rst),

    .s_tcq_valid(req_valid),
    .s_tcq_ready(req_ready),
    .s_tcq_laddr(1'b0),
    .s_tcq_raddr(req_bus_addr[31:DATA_BITS]),
    .s_tcq_length(req_sendsz),
    .s_tcq_tag(req_intno),

    .s_tcq_cvalid(event_sent),
    .s_tcq_cready(m_int_ready || !valid_interrupt),
    .s_tcq_ctag(m_int_data),

    .cfg_pcie_attr(cfg_pcie_attr),
    .cfg_pcie_reqid(cfg_pcie_reqid),

    // AXIs PCIe TX
    .m_axis_tx_tready(m_axis_tx_tready),
    .m_axis_tx_tdata(m_axis_tx_tdata),
    .m_axis_tx_tkeep(m_axis_tx_tkeep),
    .m_axis_tx_tlast(m_axis_tx_tlast),
    .m_axis_tx_tvalid(m_axis_tx_tvalid),
    .m_axis_tx_tuser(m_axis_tx_tuser),

    // RAM request interface
    .m_al_rdata(rb_data),
    .m_al_rvalid(rb_valid),
    .m_al_rid(rb_last),
    .m_al_rready(rb_ready),

    // RAM address
    .m_al_araddr(),
    .m_al_arvalid(addr_valid),
    .m_al_arid(addr_last),
    .m_al_arready(rb_valid && rb_ready)
);



endmodule

