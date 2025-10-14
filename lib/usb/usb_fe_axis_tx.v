module usb_fe_axis_tx #(
    parameter ULTRA_SCALE = 1,
    parameter RAM_WIDTH = 12,
    parameter RAM_FIFO_ALIGH = 2,
    parameter DATA_WIDTH = 32,
    parameter EP_WIDTH = 5,
    parameter MAX_EPSZ_WIDTH = 11,
    parameter AUX_BUS_TX = 1,
    parameter RAM_PARAGRAPH_WIDTH = 6,
    parameter PUSH_ZERO_DP = 0, //Push zero data packets, with keep == 0
    parameter [63:0] AUX_BUS_IDX_TO_EP = 64'h0_f_e_d_c_b_a_9_8_7_6_5_4_3_2_1,
    parameter [63:0] AUX_BUS_EP_TO_IDX = 64'he_d_c_b_a_9_8_7_6_5_4_3_2_1_0_f
) (
    input          clk,
    input          reset,

    output [RAM_WIDTH-1:RAM_FIFO_ALIGH]   mmfifo_addr,
    output                                mmfifo_valid,
    output                                mmfifo_tag,
    input                                 mmfifo_ready,

    output                       mmfifo_rb_ready,
    input                        mmfifo_rb_valid,
    input [DATA_WIDTH-1:0]       mmfifo_rb_data,
    input                        mmfifo_rb_tag,

    //Update interface format [ext][size][epaddr]
    input                                                          updfifo_valid,
    input [EP_WIDTH + MAX_EPSZ_WIDTH - RAM_FIFO_ALIGH - 1:0]       updfifo_data,
    output                                                         updfifo_ready,

    // NON-Z format for now!!!! [size][epaddr]
    output reg                                                     cnffifo_valid,
    output     [EP_WIDTH + MAX_EPSZ_WIDTH - RAM_FIFO_ALIGH - 1:0]  cnffifo_data,
    input                                                          cnffifo_ready,

    // TODO EP0 control IF for MCU handling
    output [EP_WIDTH - 1:0]                   ep_addr,
    input  [RAM_WIDTH-1:RAM_PARAGRAPH_WIDTH]  cfg_ep_ramoffset,
    input  [RAM_WIDTH-2:RAM_PARAGRAPH_WIDTH]  cfg_ep_fifozsz,     // FIFO size
    input  [MAX_EPSZ_WIDTH-1:RAM_FIFO_ALIGH]  cfg_ep_size,        // EP maxsz

    // AXIS_M
    input  [AUX_BUS_TX-1:0]                   mn_axis_endpoint_tx_busy, //(busy flag -- do not try to push data)
    output [AUX_BUS_TX-1:0]                   mn_axis_endpoint_tx_valid,
    input  [AUX_BUS_TX-1:0]                   mn_axis_endpoint_tx_ready,
    output                                    m_axis_endpoint_tx_last,
    output [DATA_WIDTH-1:0]                   m_axis_endpoint_tx_data,
    output [4-1:0]                            m_axis_endpoint_tx_keep

    // AXIS stall interface
);
// RAM packet header
// b0 - x
// b1 - x
// b2 - sb, 0000, cnt[10:8]
// b3 - cnt[7:0]

localparam MAX_ENDPOINT_SZ_WIDTH = MAX_EPSZ_WIDTH - 1;
localparam EP_UPD_WIDTH = EP_WIDTH + MAX_EPSZ_WIDTH - RAM_FIFO_ALIGH;

wire [EP_WIDTH - 1:0]                    updfifo_ep     = updfifo_data[EP_WIDTH - 1:0];
wire [MAX_EPSZ_WIDTH - 1:RAM_FIFO_ALIGH] updfifo_length = updfifo_data[EP_UPD_WIDTH - 1:EP_WIDTH];
wire                                     updfifo_reset_fifo = (updfifo_data[EP_UPD_WIDTH - 1: EP_UPD_WIDTH - 2] == 2'b10);


wire [RAM_WIDTH - 1 : RAM_FIFO_ALIGH]    ep_sta_length;
reg [MAX_EPSZ_WIDTH - 1:RAM_FIFO_ALIGH]  transferred;     // count from in integer form
reg                                      transferred_upd;

// TODO: FIXME!! not in z-format
wire  [RAM_WIDTH:RAM_FIFO_ALIGH] updated_len_trans      = {1'b0, ep_sta_length } - {2'b0, transferred } - 1'b1;
wire                             updated_len_trans_zero = updated_len_trans[RAM_WIDTH];

wire [RAM_WIDTH - 1:RAM_FIFO_ALIGH] ep_stat_idata =
    (updfifo_valid && updfifo_ready && updfifo_reset_fifo) ? {(RAM_WIDTH - RAM_FIFO_ALIGH){1'b1}} :
    (updfifo_valid && updfifo_ready)                       ? ep_sta_length + updfifo_length + 2'b10 :
                                                             updated_len_trans[RAM_WIDTH - 1:RAM_FIFO_ALIGH];


reg  [EP_WIDTH - 1:0] tx_active_ep;
wire [EP_WIDTH - 1:0] ep_stat_addr = (transferred_upd) ? tx_active_ep : updfifo_ep;
assign updfifo_ready = !transferred_upd;

// Available fifo length
ram_sxp #(
    .DATA_WIDTH(RAM_WIDTH - RAM_FIFO_ALIGH),
    .ADDR_WIDTH(5),
    .ULTRA_SCALE(ULTRA_SCALE),
    .MODE_SDP(1)
) ep_fifo_stat (
  .wclk(clk),
  .we(transferred_upd || updfifo_valid),
  .waddr(ep_stat_addr),
  .wdata(ep_stat_idata),
  .raddr(ep_stat_addr),
  .rdata(ep_sta_length)
);

// EP current FIFO pointer
wire [RAM_WIDTH-2:RAM_FIFO_ALIGH]  ep_sta_offptr;
wire [RAM_WIDTH-2:RAM_FIFO_ALIGH]  ep_cur_offptr = ep_sta_offptr;
wire                               ep_cur_strobe_we;

localparam STUB_WIDTH = RAM_PARAGRAPH_WIDTH - RAM_FIFO_ALIGH;
wire  [RAM_WIDTH-1:RAM_FIFO_ALIGH]       ep_rampos     = {cfg_ep_ramoffset, {(STUB_WIDTH){1'b0}}} + {1'b0, ep_sta_offptr};
wire  [RAM_WIDTH-2:RAM_FIFO_ALIGH]       ep_nxt_offptr = (ep_cur_offptr == {cfg_ep_fifozsz, {STUB_WIDTH{1'b1}}}) ? 0 : ep_cur_offptr + 1'b1;
wire  [RAM_WIDTH-1:RAM_FIFO_ALIGH]       ep_nxtaddr    = {cfg_ep_ramoffset, {(STUB_WIDTH){1'b0}}} + {1'b0, ep_nxt_offptr};

wire [EP_WIDTH - 1:0] ep_stat_addr2 = (updfifo_valid && updfifo_ready && updfifo_reset_fifo) ? updfifo_ep : tx_active_ep;

ram_sxp #(
    .DATA_WIDTH(RAM_WIDTH - RAM_FIFO_ALIGH - 1),
    .ADDR_WIDTH(5),
    .ULTRA_SCALE(ULTRA_SCALE),
    .MODE_SDP(1)
) ep_off_stat (
  .wclk(clk),
  .we(ep_cur_strobe_we || updfifo_valid && updfifo_ready && updfifo_reset_fifo),
  .waddr(ep_stat_addr2),
  .wdata((updfifo_valid && updfifo_ready && updfifo_reset_fifo) ? 1'b0 : ep_nxt_offptr),
  .raddr(ep_stat_addr2),
  .rdata(ep_sta_offptr)
);

reg [AUX_BUS_TX-1:0] tx_fifo_avil; //at least some data is avilable in this FIFO
//

localparam [1:0]
    STX_IDLE     = 0,
    STX_LENGTH   = 1,
    STX_DATA     = 3,
    STX_LEN_DATA = 2;

reg [1:0]                       stx_state;
reg [MAX_EPSZ_WIDTH-1:0]        stx_length;

wire       idx_selected_valid;
wire [3:0] idx_selected;

wire [3:0] cnv_ep_to_idx = {
    AUX_BUS_EP_TO_IDX[4 * updfifo_ep[3:0] + 3],
    AUX_BUS_EP_TO_IDX[4 * updfifo_ep[3:0] + 2],
    AUX_BUS_EP_TO_IDX[4 * updfifo_ep[3:0] + 1],
    AUX_BUS_EP_TO_IDX[4 * updfifo_ep[3:0] + 0] };
wire [3:0] cnv_idx_to_ep = {
    AUX_BUS_IDX_TO_EP[4 * idx_selected + 3],
    AUX_BUS_IDX_TO_EP[4 * idx_selected + 2],
    AUX_BUS_IDX_TO_EP[4 * idx_selected + 1],
    AUX_BUS_IDX_TO_EP[4 * idx_selected + 0] };

wire [AUX_BUS_TX-1:0] valid_msk =  tx_fifo_avil & ~mn_axis_endpoint_tx_busy;
wire [15:0]           valid_msk_ext = valid_msk;
assign {idx_selected_valid, idx_selected} =
    (valid_msk_ext[15] ? 5'h1f : valid_msk_ext[14] ? 5'h1e : valid_msk_ext[13] ? 5'h1d : valid_msk_ext[12] ? 5'h1c :
     valid_msk_ext[11] ? 5'h1b : valid_msk_ext[10] ? 5'h1a : valid_msk_ext[9]  ? 5'h19 : valid_msk_ext[8]  ? 5'h18 :
     valid_msk_ext[7]  ? 5'h17 : valid_msk_ext[6]  ? 5'h16 : valid_msk_ext[5]  ? 5'h15 : valid_msk_ext[4]  ? 5'h14 :
     valid_msk_ext[3]  ? 5'h13 : valid_msk_ext[2]  ? 5'h12 : valid_msk_ext[1]  ? 5'h11 : valid_msk_ext[0]  ? 5'h10 : 5'h00);

reg [3:0] idx_selected_latch;

wire [MAX_EPSZ_WIDTH-1:RAM_FIFO_ALIGH] stx_length_nxt = stx_length[MAX_EPSZ_WIDTH-1:RAM_FIFO_ALIGH] - 1'b1;

reg mmfifo_req_nstop;

assign mmfifo_addr = ep_rampos;
assign mmfifo_tag  = stx_length_nxt[MAX_EPSZ_WIDTH-1];
assign mmfifo_valid = (stx_state == STX_LENGTH) || (stx_state == STX_DATA && mmfifo_req_nstop );

assign mmfifo_rb_ready  = (stx_state == STX_LEN_DATA) || (stx_state == STX_DATA && mn_axis_endpoint_tx_ready[idx_selected_latch]);

assign mn_axis_endpoint_tx_valid = ((stx_state == STX_DATA) && mmfifo_rb_valid) << idx_selected_latch;
assign m_axis_endpoint_tx_data  = mmfifo_rb_data;
assign m_axis_endpoint_tx_last  = mmfifo_rb_tag;
assign m_axis_endpoint_tx_keep  = (!mmfifo_rb_tag)           ? 4'b1111 :
                                  (stx_length[1:0] == 2'b11) ? 4'b1111 :
                                  (stx_length[1:0] == 2'b10) ? 4'b0111 :
                                  (stx_length[1:0] == 2'b01) ? 4'b0011 : 4'b0001;

assign ep_cur_strobe_we = (stx_state == STX_LENGTH) || (stx_state == STX_DATA && mmfifo_req_nstop && mmfifo_ready);

wire [MAX_EPSZ_WIDTH-1:0] len_pkt_ondata      = { mmfifo_rb_data[MAX_EPSZ_WIDTH-8-1+16:16], mmfifo_rb_data[31:24] };
wire                      len_pkt_ondata_zero = len_pkt_ondata[MAX_EPSZ_WIDTH-1];

assign cnffifo_data[EP_WIDTH + MAX_EPSZ_WIDTH - RAM_FIFO_ALIGH - 1:EP_WIDTH] = transferred;
assign cnffifo_data[EP_WIDTH - 1:0]                                          = tx_active_ep;
assign ep_addr                                                               = tx_active_ep;
// TX part
always @(posedge clk) begin
    if (reset) begin
        tx_fifo_avil       <= 1'b0;
        mmfifo_req_nstop   <= 1'b0;
        cnffifo_valid      <= 1'b0;
        transferred_upd    <= 1'b0;

        idx_selected_latch <= 0;
        //tx_active_ep_valid <= 1'b0;
        stx_state          <= STX_IDLE;
    end else begin
        transferred_upd    <= 1'b0;

        if (cnffifo_valid && cnffifo_ready) begin
            cnffifo_valid                    <= 1'b0;
        end

        if (transferred_upd) begin
            tx_fifo_avil[idx_selected_latch] <= !updated_len_trans_zero;
        end else if (updfifo_valid && updfifo_ready && !updfifo_reset_fifo) begin
            tx_fifo_avil[cnv_ep_to_idx]      <= 1'b1;
        end

        case (stx_state)
        STX_IDLE: begin
            mmfifo_req_nstop <= 1'b0;

            if (idx_selected_valid && (!cnffifo_valid || cnffifo_ready) && !transferred_upd) begin
                tx_active_ep       <= { 1'b0, cnv_idx_to_ep };
                idx_selected_latch <= idx_selected;
                stx_state          <= STX_LENGTH;
                mmfifo_req_nstop   <= 1'b1;
                // make simulation clear, but TAG is ignored on first beat
                // stx_length         <= 1 << RAM_FIFO_ALIGH; // Need mmfifo_tag == 1'b0;
            end
        end

        STX_LENGTH: begin
            if (mmfifo_ready) begin
                mmfifo_req_nstop <= 1'b0;
                stx_state        <= STX_LEN_DATA;
                transferred      <= 0;
            end
        end

        STX_LEN_DATA: begin
            if (mmfifo_rb_valid) begin
                stx_length <= len_pkt_ondata;
                if (len_pkt_ondata_zero) begin
                    //Zero data packet
                    stx_state       <= STX_IDLE;
                    transferred_upd <= 1'b1;
                    cnffifo_valid   <= 1'b1;
                end else begin
                    stx_state        <= STX_DATA;
                    mmfifo_req_nstop <= 1'b1;
                end
            end
        end

        STX_DATA: begin
            if (mmfifo_valid && mmfifo_ready) begin
                stx_length[MAX_EPSZ_WIDTH-1:RAM_FIFO_ALIGH] <= stx_length_nxt;
                mmfifo_req_nstop                            <= mmfifo_req_nstop && ~mmfifo_tag;
                transferred                                 <= transferred + 1'b1;
            end

            if (mmfifo_rb_tag && mmfifo_rb_valid && mmfifo_rb_ready) begin
                stx_state                        <= STX_IDLE;
                transferred_upd                  <= 1'b1;
                cnffifo_valid                    <= 1'b1;
            end
        end

        endcase

        if (updfifo_valid && updfifo_ready && updfifo_reset_fifo) begin
            tx_fifo_avil[cnv_ep_to_idx] <= 1'b0;
            stx_state                   <= STX_IDLE;
        end
    end
end



endmodule
