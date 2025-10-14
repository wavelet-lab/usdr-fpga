module usb_fe_axis_rx #(
    parameter ULTRA_SCALE = 0,
    parameter RAM_WIDTH = 12,
    parameter RAM_FIFO_ALIGH = 2,
    parameter DATA_WIDTH = 32,
    parameter EP_WIDTH = 5,
    parameter MAX_EPSZ_WIDTH = 11,
    parameter AUX_BUS_RX = 1,
    parameter RAM_PARAGRAPH_WIDTH = 6,
    parameter PUSH_ZERO_DP = 0, //Push zero data packets, with keep == 0
    parameter [63:0] AUX_BUS_IDX_TO_EP = 64'h0_f_e_d_c_b_a_9_8_7_6_5_4_3_2_1,
    parameter [63:0] AUX_BUS_EP_TO_IDX = 64'he_d_c_b_a_9_8_7_6_5_4_3_2_1_0_f
) (
    input          clk,
    input          reset,

    output [RAM_WIDTH-1:RAM_FIFO_ALIGH]   mmfifo_addr,
    output [DATA_WIDTH-1:0]               mmfifo_data,
    output                                mmfifo_valid,
    input                                 mmfifo_ready,

    //Update interface format [ext][size][epaddr]
    input                                                          updfifo_valid,
    input [EP_WIDTH + MAX_EPSZ_WIDTH - RAM_FIFO_ALIGH - 1:0]       updfifo_data,
    output                                                         updfifo_ready,

    output reg                                                     cnffifo_valid,
    output     [EP_WIDTH + MAX_EPSZ_WIDTH - RAM_FIFO_ALIGH - 1:0]  cnffifo_data,
    input                                                          cnffifo_ready,

    // TODO EP0 control IF for MCU handling
    output [EP_WIDTH - 1:0]                   ep_addr,
    input  [RAM_WIDTH-1:RAM_PARAGRAPH_WIDTH]  cfg_ep_ramoffset,
    input  [RAM_WIDTH-2:RAM_PARAGRAPH_WIDTH]  cfg_ep_fifozsz,     // FIFO size
    input  [MAX_EPSZ_WIDTH-1:RAM_FIFO_ALIGH]  cfg_ep_size,        // EP maxsz

    output [EP_WIDTH - 1:0]                   ep_addr2,
    input  [MAX_EPSZ_WIDTH-1:RAM_FIFO_ALIGH]  cfg_ep_size2,       // EP maxsz

    // AXIS_M
    input  [AUX_BUS_RX-1:0]                   sn_axis_endpoint_rx_valid,
    output [AUX_BUS_RX-1:0]                   sn_axis_endpoint_rx_ready,
    input  [AUX_BUS_RX-1:0]                   sn_axis_endpoint_rx_last,
    input  [AUX_BUS_RX * DATA_WIDTH - 1:0]    sn_axis_endpoint_rx_data,
    input  [AUX_BUS_RX * 4 - 1:0]             sn_axis_endpoint_rx_keep,

    // AXIS stall interface
    input  [AUX_BUS_RX-1:0]                   sn_axis_endpoint_rx_stall
);

// We need to monitor at least epmaxsz+8 space at FIFO before setting avail flag (to store DATA+LEN+ZDP)
localparam AUX_BUS_RX_BITS = (AUX_BUS_RX == 1) ? 1 : $clog2(AUX_BUS_RX);

reg [3:0]                  idx_selected_latch;
wire [AUX_BUS_RX_BITS-1:0] idx_sel = idx_selected_latch[AUX_BUS_RX_BITS-1:0];

wire                        s_axis_endpoint_rx_last = sn_axis_endpoint_rx_last[idx_sel];
wire  [DATA_WIDTH - 1:0]    s_axis_endpoint_rx_data = sn_axis_endpoint_rx_data >> (DATA_WIDTH * idx_sel);
wire  [4 - 1:0]             s_axis_endpoint_rx_keep = sn_axis_endpoint_rx_keep >> (4 * idx_sel);


localparam MAX_ENDPOINT_SZ_WIDTH = MAX_EPSZ_WIDTH - 1;
localparam EP_UPD_WIDTH = EP_WIDTH + MAX_EPSZ_WIDTH - RAM_FIFO_ALIGH;

wire [EP_WIDTH - 1:0]                    updfifo_ep     = updfifo_data[EP_WIDTH - 1:0];
wire [MAX_EPSZ_WIDTH - 1:RAM_FIFO_ALIGH] updfifo_length = updfifo_data[EP_UPD_WIDTH - 1:EP_WIDTH];
wire                                     updfifo_reset_fifo = (updfifo_data[EP_UPD_WIDTH - 1: EP_UPD_WIDTH - 2] == 2'b10);

wire [RAM_WIDTH - 1 : RAM_FIFO_ALIGH]    ep_sta_length;
reg [MAX_EPSZ_WIDTH - 1:RAM_FIFO_ALIGH]  transferred;
reg [RAM_FIFO_ALIGH-1:0]                 transferred_low;
wire                                     transferred_upd;
wire [MAX_EPSZ_WIDTH - 1:RAM_FIFO_ALIGH] transferred_nxt = transferred + 1'b1;

wire  [RAM_WIDTH - 1:RAM_FIFO_ALIGH] updated_len_trans = ep_sta_length - transferred - 2'b10;


// Updated available space in FIFO in DW in Z-format
// 1. In case of reset to the maximum available space
// 2. Increase buffer size when data in buffer is sent (by updfifo_length in z-format)
// 3. Decrease buffer size by stream data placed into RAM buffer

wire [RAM_WIDTH - 1:RAM_FIFO_ALIGH] ep_stat_idata =
    (updfifo_valid && updfifo_ready && updfifo_reset_fifo) ? { cfg_ep_fifozsz, {(RAM_PARAGRAPH_WIDTH - RAM_FIFO_ALIGH){1'b1}}} :
    (updfifo_valid && updfifo_ready)                       ? ep_sta_length + updfifo_length + 2'b10 :
                                                             updated_len_trans;

reg   [RAM_WIDTH - 1:RAM_FIFO_ALIGH]    ep_stat_idata_reg;
reg                                     ep_stat_idata_upd;
reg   [EP_WIDTH - 1:0]                  ep_stat_idata_ep;
reg  [MAX_EPSZ_WIDTH-1:RAM_FIFO_ALIGH]  cached_cfg_ep_size2;

// we need cached_cfg_ep_size2 + 1 to store maximum length packet
wire  [RAM_WIDTH:RAM_FIFO_ALIGH]     ep_stat_enough_delta = ep_stat_idata_reg - cached_cfg_ep_size2 - 2'b10;
wire                                 ep_stat_enough_flag  = !ep_stat_enough_delta[RAM_WIDTH];


reg  [EP_WIDTH - 1:0] rx_active_ep;
assign updfifo_ready = !transferred_upd && !ep_stat_idata_upd;

assign ep_addr2 = (transferred_upd) ? rx_active_ep : updfifo_ep;
wire update_avail_fifolen = transferred_upd && mmfifo_ready || updfifo_valid && updfifo_ready;
// Available fifo length

ram_sxp #(
    .DATA_WIDTH(RAM_WIDTH - RAM_FIFO_ALIGH),
    .ADDR_WIDTH(5),
    .ULTRA_SCALE(ULTRA_SCALE),
    .MODE_SDP(1)
) ep_fifo_stat (
  .wclk(clk),
  .we(update_avail_fifolen),
  .waddr(ep_addr2),
  .wdata(ep_stat_idata),
  .raddr(ep_addr2),
  .rdata(ep_sta_length)
);

// EP current FIFO pointer
reg  [RAM_WIDTH-1:RAM_FIFO_ALIGH]  ep_rampos_pktlen;

wire [RAM_WIDTH-2:RAM_FIFO_ALIGH]  ep_sta_offptr;
wire [RAM_WIDTH-2:RAM_FIFO_ALIGH]  ep_cur_offptr = ep_sta_offptr;
wire                               ep_cur_strobe_we;

localparam STUB_WIDTH = RAM_PARAGRAPH_WIDTH - RAM_FIFO_ALIGH;
wire  [RAM_WIDTH-1:RAM_FIFO_ALIGH]       ep_rampos     = {cfg_ep_ramoffset, {(STUB_WIDTH){1'b0}}} + {1'b0, ep_sta_offptr};
wire  [RAM_WIDTH-2:RAM_FIFO_ALIGH]       ep_nxt_offptr = (ep_cur_offptr == {cfg_ep_fifozsz, {STUB_WIDTH{1'b1}}}) ? 0 : ep_cur_offptr + 1'b1;
wire  [RAM_WIDTH-1:RAM_FIFO_ALIGH]       ep_nxtaddr    = {cfg_ep_ramoffset, {(STUB_WIDTH){1'b0}}} + {1'b0, ep_nxt_offptr};

wire [EP_WIDTH - 1:0] ep_stat_addr2 = (updfifo_valid && updfifo_ready && updfifo_reset_fifo) ? updfifo_ep : rx_active_ep;

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
  .raddr(rx_active_ep),
  .rdata(ep_sta_offptr)
);

wire                       idx_selected_valid;
wire [3:0]                 idx_selected;

wire       s_axis_endpoint_rx_valid = sn_axis_endpoint_rx_valid[idx_sel];
wire       s_axis_endpoint_rx_ready;
assign     sn_axis_endpoint_rx_ready = s_axis_endpoint_rx_ready << idx_sel;

wire [3:0] cnv_ep_to_idx = {
    AUX_BUS_EP_TO_IDX[4 * ep_stat_idata_ep[3:0] + 3],
    AUX_BUS_EP_TO_IDX[4 * ep_stat_idata_ep[3:0] + 2],
    AUX_BUS_EP_TO_IDX[4 * ep_stat_idata_ep[3:0] + 1],
    AUX_BUS_EP_TO_IDX[4 * ep_stat_idata_ep[3:0] + 0] };
wire [3:0] cnv_idx_to_ep = {
    AUX_BUS_IDX_TO_EP[4 * idx_selected + 3],
    AUX_BUS_IDX_TO_EP[4 * idx_selected + 2],
    AUX_BUS_IDX_TO_EP[4 * idx_selected + 1],
    AUX_BUS_IDX_TO_EP[4 * idx_selected + 0] };


reg  [AUX_BUS_RX-1:0] rx_fifo_avil; //at least some data is avilable in this FIFO
wire [AUX_BUS_RX-1:0] valid_msk =  rx_fifo_avil & sn_axis_endpoint_rx_valid;
wire [15:0]           valid_msk_ext = valid_msk;
assign {idx_selected_valid, idx_selected} =
    (valid_msk_ext[15] ? 5'h1f : valid_msk_ext[14] ? 5'h1e : valid_msk_ext[13] ? 5'h1d : valid_msk_ext[12] ? 5'h1c :
     valid_msk_ext[11] ? 5'h1b : valid_msk_ext[10] ? 5'h1a : valid_msk_ext[9]  ? 5'h19 : valid_msk_ext[8]  ? 5'h18 :
     valid_msk_ext[7]  ? 5'h17 : valid_msk_ext[6]  ? 5'h16 : valid_msk_ext[5]  ? 5'h15 : valid_msk_ext[4]  ? 5'h14 :
     valid_msk_ext[3]  ? 5'h13 : valid_msk_ext[2]  ? 5'h12 : valid_msk_ext[1]  ? 5'h11 : valid_msk_ext[0]  ? 5'h10 : 5'h00);


localparam [1:0]
    SRX_IDLE     = 0,
    SRX_SAVE_POS = 1,
    SRX_DATA     = 2,
    SRX_LEN_FILL = 3;

reg [1:0]                       srx_state;
reg                             srx_push_packet;

assign mmfifo_addr                       = (srx_state == SRX_LEN_FILL) ? ep_rampos_pktlen : ep_rampos;

assign mmfifo_data[31:24]  = (srx_state == SRX_LEN_FILL) ? ({transferred[7:2], transferred_low[1:0]})      : s_axis_endpoint_rx_data[31:24];
assign mmfifo_data[23:16]  = (srx_state == SRX_LEN_FILL) ? ({5'b00000, transferred[MAX_EPSZ_WIDTH - 1:8]}) : s_axis_endpoint_rx_data[23:16];
assign mmfifo_data[15:0]   = s_axis_endpoint_rx_data[15:0];

assign mmfifo_valid                      = (srx_state == SRX_DATA) && s_axis_endpoint_rx_valid || (srx_state == SRX_LEN_FILL);
assign s_axis_endpoint_rx_ready          = (srx_state == SRX_DATA) && mmfifo_ready;

assign ep_cur_strobe_we         =  (srx_state == SRX_SAVE_POS) ||
    (srx_state == SRX_DATA && mmfifo_valid && mmfifo_ready);

assign ep_addr = (updfifo_valid && updfifo_ready && updfifo_reset_fifo) ? updfifo_ep : rx_active_ep;

assign transferred_upd = (srx_state == SRX_LEN_FILL);

assign cnffifo_data[EP_WIDTH + MAX_EPSZ_WIDTH - RAM_FIFO_ALIGH - 1:EP_WIDTH] = transferred + 1'b1;
assign cnffifo_data[EP_WIDTH - 1:0]                                          = rx_active_ep;


reg [AUX_BUS_RX-1:0] ep_stall_state;

wire [AUX_BUS_RX-1:0] ep_stall_update = ep_stall_state ^ sn_axis_endpoint_rx_stall;
wire [15:0]           ep_stall_update_msk_ext = ep_stall_update;
wire                  idx_ep_stall_update_valid;
wire [3:0]            idx_ep_stall_update;

assign {idx_ep_stall_update_valid, idx_ep_stall_update} =
    (ep_stall_update_msk_ext[15] ? 5'h1f : ep_stall_update_msk_ext[14] ? 5'h1e : ep_stall_update_msk_ext[13] ? 5'h1d : ep_stall_update_msk_ext[12] ? 5'h1c :
     ep_stall_update_msk_ext[11] ? 5'h1b : ep_stall_update_msk_ext[10] ? 5'h1a : ep_stall_update_msk_ext[9]  ? 5'h19 : ep_stall_update_msk_ext[8]  ? 5'h18 :
     ep_stall_update_msk_ext[7]  ? 5'h17 : ep_stall_update_msk_ext[6]  ? 5'h16 : ep_stall_update_msk_ext[5]  ? 5'h15 : ep_stall_update_msk_ext[4]  ? 5'h14 :
     ep_stall_update_msk_ext[3]  ? 5'h13 : ep_stall_update_msk_ext[2]  ? 5'h12 : ep_stall_update_msk_ext[1]  ? 5'h11 : ep_stall_update_msk_ext[0]  ? 5'h10 : 5'h00);


wire [3:0] cnv_idx_ep_stall_update_to_ep = {
    AUX_BUS_IDX_TO_EP[4 * idx_ep_stall_update + 3],
    AUX_BUS_IDX_TO_EP[4 * idx_ep_stall_update + 2],
    AUX_BUS_IDX_TO_EP[4 * idx_ep_stall_update + 1],
    AUX_BUS_IDX_TO_EP[4 * idx_ep_stall_update + 0] };


always @(posedge clk) begin
    if (reset) begin
        rx_fifo_avil       <= 1'b0;
        cnffifo_valid      <= 1'b0;

        idx_selected_latch <= 0;

        ep_stat_idata_upd  <= 1'b0;
        srx_state          <= SRX_IDLE;

        ep_stall_state     <= 0;

    end else begin
        ep_stat_idata_upd  <= 1'b0;

        if (cnffifo_valid && cnffifo_ready) begin
            cnffifo_valid <= 1'b0;
        end

        //Update availability flags
        if (update_avail_fifolen) begin
            ep_stat_idata_reg   <= ep_stat_idata;
            ep_stat_idata_upd   <= 1'b1;
            ep_stat_idata_ep    <= ep_addr2;

            cached_cfg_ep_size2 <= cfg_ep_size2;
        end

        case (srx_state)
        SRX_IDLE: begin
            if (idx_selected_valid && (!cnffifo_valid || cnffifo_ready) && !transferred_upd && !ep_stat_idata_upd) begin
                rx_active_ep       <= {1'b1, cnv_idx_to_ep};
                idx_selected_latch <= idx_selected;
                transferred        <= ~0;

                srx_state          <= SRX_SAVE_POS;
            end else if (idx_ep_stall_update_valid && (!cnffifo_valid || cnffifo_ready)) begin
                cnffifo_valid                                           <= 1'b1;
                rx_active_ep                                            <= {1'b1, cnv_idx_ep_stall_update_to_ep};
                transferred[MAX_EPSZ_WIDTH - 1:MAX_EPSZ_WIDTH - 3]      <= (sn_axis_endpoint_rx_stall[idx_ep_stall_update]) ? 3'b101 : 3'b100;
                transferred[MAX_EPSZ_WIDTH - 4:RAM_FIFO_ALIGH]          <= ~1;
                srx_state                                               <= SRX_IDLE;

                ep_stall_state[idx_ep_stall_update]                     <= sn_axis_endpoint_rx_stall[idx_ep_stall_update];
            end
        end

        SRX_SAVE_POS: begin
            srx_state          <= SRX_DATA;
            ep_rampos_pktlen   <= ep_rampos;
        end

        SRX_DATA: begin
            if (mmfifo_valid && mmfifo_ready) begin
                transferred <= transferred_nxt;
                if (s_axis_endpoint_rx_last || (transferred_nxt == cfg_ep_size)) begin
                    srx_push_packet <= s_axis_endpoint_rx_last && (transferred_nxt == cfg_ep_size);
                    transferred_low <= (s_axis_endpoint_rx_keep == 4'b0111) ? 2'b10 :
                                       (s_axis_endpoint_rx_keep == 4'b0011) ? 2'b01 :
                                       (s_axis_endpoint_rx_keep == 4'b0001) ? 2'b00 : 2'b11;

                    srx_state       <= SRX_LEN_FILL;
                end
            end
        end

        // TODO fill zero packet right after
        // FIFO length decreased here
        SRX_LEN_FILL: begin
            if (mmfifo_valid && mmfifo_ready) begin
                srx_state         <= SRX_IDLE;
                cnffifo_valid     <= 1'b1;
                ep_rampos_pktlen  <= ep_rampos;

                rx_fifo_avil[idx_selected_latch] <= 1'b0;
            end
        end

        endcase

    end

    if (ep_stat_idata_upd) begin
        rx_fifo_avil[cnv_ep_to_idx] <= ep_stat_enough_flag;
    end
end

endmodule
