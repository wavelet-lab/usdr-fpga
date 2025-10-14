module usb2_tran_if #(
    parameter ROM_WIDTH = 8,
    parameter RAM_WIDTH = 12,
    parameter EP_WIDTH = 5,
    parameter MAX_EPSZ_WIDTH = 11,
    parameter HWENUMERATOR = 1,
    parameter RAM_PARAGRAPH_WIDTH = 6,
    parameter ULTRA_SCALE = 0,
    parameter USDR_PID = 0
)(
    input            clk,
    input            reset,

    input            cfg_usb_highspeed,
    output reg [6:0] cfg_usb_addr,
    output reg       cfg_usb_configured,

    output [EP_WIDTH-1:0]                         cfg_ep_addr,
    input [RAM_WIDTH - 1:RAM_PARAGRAPH_WIDTH]     cfg_ep_ramoffset,
    input [RAM_WIDTH - 2:RAM_PARAGRAPH_WIDTH]     cfg_ep_fifozsz,
    // input [MAX_EPSZ_WIDTH-1:2]                    cfg_ep_maxsize,

    input [7:0]    axis_usb_rx_data,
    input          axis_usb_rx_valid,
    input          axis_usb_rx_last,
    input          axis_usb_rx_error,

    output [7:0]   axis_usb_tx_data,
    output         axis_usb_tx_valid,
    output         axis_usb_tx_last,
    output         axis_usb_tx_error,
    input          axis_usb_tx_ready,

    //Memory mmaped ram
    output reg [RAM_WIDTH-1:0] mmfifo_addr,
    output reg [7:0]           mmfifo_data,
    output reg                 mmfifo_wr,
    output reg                 mmfifo_valid,
    output reg                 mmfifo_tag,
    input                      mmfifo_ready,

    output                     mmfifo_rb_ready,
    input                      mmfifo_rb_valid,
    input [7:0]                mmfifo_rb_data,
    input                      mmfifo_rb_tag,

    // Update interface format [ext][size][epaddr]; size of payload only in DW-z format
    output reg                                       updfifo_valid,
    output reg [EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:0] updfifo_data,
    input                                            updfifo_ready,

    // Cnf format [size][epaddr]; size in DW z-format  (inc. header)
    input                                            cnffifo_valid,
    input [EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:0]      cnffifo_data,
    output                                           cnffifo_ready,

    output [11+8:0] debug_state,
    output [31:0]   debug2
);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO NYET isn't handled right,
// check it and make it work properly when cfg_usb_highspeed is set!!!
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Asuming EP0 is 64b always

// TODOs:
// - active_ep / cur_ep
// - NYET disable in USB1.1
// - hold EP0 push for autoenumeration

`include "usb_enums.vh"

// The second case, known as “protocol stall,” is detailed in Section 8.5.3.  Protocol stall is unique to
// control pipes.  Protocol stall differs from functional stall in meaning and duration.  A protocol STALL
// is returned during the Data or Status stage of a control transfer, and the STALL condition terminates at
// the beginning of the next control transfer (Setup).  The remainder of this section refers to the general
// case of a functional stall


//9.1.1.5  Configured
//Before a USB device’s function may be used, the device must be configured.  From the device’s
//perspective, configuration involves correctly processing a SetConfiguration() request with a non-zero
//configuration value.  Configuring a device or changing an alternate setting causes all of the status and
//configuration values associated with endpoints in the affected interfaces to be set to their default values.
//This includes setting the data toggle of any endpoint using data toggles to the value DATA0.

// ROM-based enumerator
// MASK-ROM dictionary => size . offset
// FSM iterates over and try to find offset and size in ROM
// on success sends out data, otherwise STALLs request


// Hardware setup logic, mandatory SETUP transactions
// ClearFeature
// GetConfiguration
// GetDescriptor
// GetStatus
// SetAddress
// SetConfiguration
// SetFeature
// ----------- => MCU pass or STALL

// Endpoint configuration  (i.e. which endpoint is halted)

// 8 byte temp RX buffer (don't save token here)
reg [63:0]  temp_data;
reg         token_skipped;

// SETUP packet
wire [7:0]  setup_bmRequestType = temp_data[63:56];
wire [7:0]  setup_bRequest      = temp_data[55:48];
wire [15:0] setup_wValue        = { temp_data[39:32], temp_data[47:40] };
wire [15:0] setup_wIndex        = { temp_data[23:16], temp_data[31:24] };
wire [15:0] setup_wLength       = { temp_data[7:0],   temp_data[15:8]  };

reg setup_data_valid;

wire tran_state_setup;
always @(posedge clk) begin
    if (reset) begin
        token_skipped    <= 1'b0;
        setup_data_valid <= 1'b0;
    end else begin
        if (axis_usb_rx_valid) begin
            token_skipped   <= 1'b1;
            if (token_skipped && (tran_state_setup )) begin
                temp_data[63:0] <= {temp_data[55:0], axis_usb_rx_data};
            end
            if (axis_usb_rx_last) begin
                token_skipped   <= 1'b0;
            end
        end
        setup_data_valid <= axis_usb_rx_valid && axis_usb_rx_last && tran_state_setup;
    end
end


`ifndef SYM
/*
ila_0 ila_0(
    .clk(clk),
    .probe0(setup_data_valid),
    .probe1(setup_bmRequestType),
    .probe2(setup_bRequest),
    .probe3(setup_wValue),
    .probe4(setup_wIndex),
    .probe5(setup_wLength)
);
*/
`endif


reg [6:0] usb_addr;
reg       configured;

reg [ROM_WIDTH - 1:0] rom_dscr_len;
reg [ROM_WIDTH - 1:0] rom_dscr_off;

reg [7:0] tx_data;
reg       tx_valid;
reg       tx_last;
reg       tx_error;
wire      tx_ready;
assign axis_usb_tx_data  = tx_data;
assign axis_usb_tx_valid = tx_valid;
assign axis_usb_tx_last  = tx_last;
assign axis_usb_tx_error = tx_error;
assign tx_ready          = axis_usb_tx_ready;

localparam MAX_ENDPOINT_SZ_WIDTH = MAX_EPSZ_WIDTH - 1;
reg [MAX_ENDPOINT_SZ_WIDTH:0] bcnt;
reg [MAX_ENDPOINT_SZ_WIDTH:0] bcnt_sent;
wire [MAX_ENDPOINT_SZ_WIDTH:0] bcnt_sent_next = bcnt_sent - 1'b1;


// -- Endpoint configuration word
// ramoffset  [RAM_WIDTH-1:6]   --  RAM region offset in RAM (64b aligment)
// fifozsz    [RAM_WIDTH-2:6]   --  FIFO size in 64b
// epszmax    [2:0]             --  Endpoint MAX size

// -- Endpoint status
// offptr     [RAM_WIDTH-2:2]   --  Current pointer
// length     [RAM_WIDTH-1:2]   --  Current buffer length (used for IN / free for OUT)
// stallflag
// toggleflag
// nyetflag                     --  Tried to do OUT but got no space, mark this EP to respond NAK to PING, flash it once a packet advanced

// avail format
// 10xxx - Reset to 0 command
// 11xxx = 0
// 00000 = 1
// 00001 = 2
// 01111 = 16

wire [4:0] ep_addr;
wire [4:0] ep_waddr;

localparam RAM_FIFO_ALIGH      = 2;

wire  [RAM_WIDTH-1:RAM_FIFO_ALIGH]       ep_sta_length;
wire  [RAM_WIDTH-2:RAM_FIFO_ALIGH]       ep_sta_offptr;
wire                                     ep_toggle;
wire                                     ep_stall;
wire                                     ep_nyetflag;

//reg   [4:0]                              ep_cur_addr;
reg   [RAM_WIDTH-2:RAM_FIFO_ALIGH]       ep_cur_offptr;
reg   [RAM_FIFO_ALIGH-1:0]               ep_cur_offptr_low;
reg                                      ep_cur_toggle;
reg                                      ep_cur_stall_clear;
reg                                      ep_cur_nyetflag;
reg                                      ep_cur_strobe;
reg                                      ep_cur_strobe_we;
reg                                      ep_cur_strobe_rdy;
reg                                      ep_cur_rst_fifo_len;

wire                                     ep_update_fifo_stat;

localparam EP_FIFO_STAT_WIDTH = RAM_WIDTH - RAM_FIFO_ALIGH + 2;
wire [EP_FIFO_STAT_WIDTH - 1:0] ep_stat_idata;
wire [4:0]                      ep_stat_addr;

ram_sxp #(
    .DATA_WIDTH(EP_FIFO_STAT_WIDTH),
    .ADDR_WIDTH(5),
    .ULTRA_SCALE(ULTRA_SCALE),
    .MODE_SDP(1)
) ep_fifo_stat (
  .wclk(clk),
  .we(ep_update_fifo_stat),
  .waddr(ep_stat_addr),
  .wdata(ep_stat_idata),

  .raddr(ep_stat_addr),
  .rdata({ep_sta_length, ep_nyetflag, ep_stall})
);

/*
`ifndef SYM
ila_0 ila_0(
    .clk(clk),
    .probe0(ep_update_fifo_stat),
    .probe1(ep_stat_addr),
    .probe2(ep_stat_idata)
);
`endif
*/

/*
`ifndef SYM
ila_1 ila_1(
    .clk(clk),
    .probe0(updfifo_valid || cnffifo_valid || ep_update_fifo_stat),
    .probe1(updfifo_valid),
    .probe2(updfifo_ready),
    .probe3(updfifo_data),  //13
    .probe4(cnffifo_valid),
    .probe5(cnffifo_ready),
    .probe6(cnffifo_data),  //13
    .probe7(ep_update_fifo_stat),
    .probe8(ep_stat_addr),
    .probe9(ep_stat_idata)
);
`endif
*/

localparam EP_STAT_WIDTH = RAM_WIDTH - 1 - RAM_FIFO_ALIGH + 1;
ram_sxp #(
    .DATA_WIDTH(EP_STAT_WIDTH),
    .ADDR_WIDTH(5),
    .ULTRA_SCALE(ULTRA_SCALE),
    .MODE_SDP(1)
) ep_stat (
  .wclk(clk),
  .we(ep_cur_strobe_we /*&& ep_cur_strobe_rdy*/),
  .waddr(ep_waddr),
  .wdata({ep_cur_offptr[RAM_WIDTH-2:RAM_FIFO_ALIGH], ep_cur_toggle}),
  .raddr(ep_addr),
  .rdata({ep_sta_offptr, ep_toggle})
);

// End region (to reset ram addr)
localparam STUB_WIDTH = RAM_PARAGRAPH_WIDTH - RAM_FIFO_ALIGH;
wire  [RAM_WIDTH-1:RAM_FIFO_ALIGH]       ep_rampos     = {cfg_ep_ramoffset, {(STUB_WIDTH){1'b0}}} + {1'b0, ep_sta_offptr};
wire  [RAM_WIDTH-2:RAM_FIFO_ALIGH]       ep_nxt_offptr = (ep_cur_offptr == {cfg_ep_fifozsz, {STUB_WIDTH{1'b1}}}) ? 0 : ep_cur_offptr + 1'b1;
wire  [RAM_WIDTH-1:RAM_FIFO_ALIGH]       ep_nxtaddr    = {cfg_ep_ramoffset, {(STUB_WIDTH){1'b0}}} + {1'b0, ep_nxt_offptr};

wire                                     ep_no_avail   = ep_sta_length[RAM_WIDTH-1];

// Control request FSM for EP0
// ST_CTRL_NONE   - No control request in progress
// ST_CTRL_SETUP  - Setup phase
// ST_CTRL_DATA   - Data stage
// ST_CTRL_STATUS - Status stage
reg [1:0] ctrl_state;
localparam [1:0]
    FSM_CTRL_NONE   = 0,
    FSM_CTRL_SETUP  = 1,
    FSM_CTRL_DATA   = 2,
    FSM_CTRL_STATUS = 3;

// General rules for DATA transfers
//
// OUT -> DATA0/1          ->           -- {data errror}
//                         -> *STALL    -- Endpoint halted
//                         -> *NYET/NAK -- No space in buffer
//    toggle_bit == DATAx  -> *ACK      -- Do nothing, previous ACK was lost
//    toggle_bit != DATAx  -> *ACK      -- Accepted, update FIFO counters
//
// PING -> *STALL          -- Endpoint halted
//      -> *NAK            -- No space in buffer
//      -> *ACK            -- Space in buffer available for MAXEPSZ
//
// IN -> *STALL            -- Endpoint halted
//    -> *NAK              -- No data available
//    -> *DATA0/1   ->
//                  -> ACK -- Accepted, update FIFO counters, toggle_bit <= ~toggle_bit
//

// Transaction state information
reg [1:0] tran_state;
reg [4:0] active_ep;  // active_ep is valid when tran_state != NONE
reg [4:0] cur_ep;
reg [4:0] iter_ep;


reg       ep_type;    // Endpoint type 0 - Control; 1 - Bulk
localparam [0:0]
    EPTYPE_CONTROL = 0,
    EPTYPE_BULK    = 1;

localparam [1:0]
    FSM_TRAN_NONE  = 0,
    FSM_TRAN_IN    = 1,
    FSM_TRAN_OUT   = 2,
    FSM_TRAN_SETUP = 3;

assign tran_state_setup = (tran_state == FSM_TRAN_SETUP);

reg [3:0]  act_state;
localparam [3:0]
    ACT_NONE                = 0,
    ACT_CHECK_PING          = 1,
    ACT_CHECK_IN            = 2,
    ACT_UPD_LEN_ACK         = 3,
    ACT_UPD_LEN_CMT         = 4,
    ACT_RESET_EP0_SETUP_IN  = 5,
    ACT_RESET_EP0_SETUP_OUT = 6,
    ACT_LOAD_EDATA          = 7,
    ACT_LEN_BUFFER0         = 8,
    ACT_LEN_BUFFER1         = 9,
    ACT_IN_WAIT_DATA        = 10,
    ACT_FILL_BUFFER         = 11,
    ACT_SETUP_LOAD_EP0_IN   = 12,
    ACT_WAIT_DATA           = 13;


reg [RAM_WIDTH-1:2]              length_remaining;   // Length remaining
wire [RAM_WIDTH-1:2]             length_remaining_next = (length_remaining - 1'b1);
wire                             length_remaining_nxtzero = (length_remaining_next[RAM_WIDTH-1]);

wire [RAM_WIDTH - 1:RAM_FIFO_ALIGH] ep_len_rstz = (ep_waddr[4]) ?  {(RAM_WIDTH - RAM_FIFO_ALIGH){1'b1}} : { 1'b0, cfg_ep_fifozsz, {(RAM_PARAGRAPH_WIDTH - RAM_FIFO_ALIGH){1'b1}}};
wire [RAM_WIDTH - 1:RAM_FIFO_ALIGH] ep_len_updz = ep_sta_length - {bcnt[MAX_ENDPOINT_SZ_WIDTH], bcnt[MAX_ENDPOINT_SZ_WIDTH:2]} - 2'b10;
wire [EP_FIFO_STAT_WIDTH - 1:0] ep_sis = {ep_cur_rst_fifo_len ? ep_len_rstz : ep_len_updz , ep_cur_nyetflag, ep_cur_stall_clear ? 1'b0 : ep_stall };

wire [MAX_EPSZ_WIDTH - 1:2]         cnffifo_lenz       = cnffifo_data[EP_WIDTH + MAX_EPSZ_WIDTH - 2 - 1:EP_WIDTH];
wire                                cnffifo_stall_flag = (cnffifo_lenz[MAX_EPSZ_WIDTH - 1:MAX_EPSZ_WIDTH - 2] == 2'b10);
wire                                cnffifo_stall_bit  = (cnffifo_lenz[MAX_EPSZ_WIDTH - 3]);
wire [RAM_WIDTH - 1:RAM_FIFO_ALIGH] cnffifo_updz       = ep_sta_length + {cnffifo_lenz[MAX_EPSZ_WIDTH - 1], cnffifo_lenz} + 1'b1;
wire [EP_FIFO_STAT_WIDTH - 1:0]     cnffifo_idata      = { cnffifo_updz, 1'b0, (cnffifo_stall_flag) ? cnffifo_stall_bit /* 1'b1 */: ep_stall };

wire [6:2]                  setup_ep0upd_len;
wire                        setup_ep0upd_stall_flag;
reg                         setup_ep0upd_valid;
wire                        setup_ep0upd_ready = (!ep_cur_strobe) && (!cnffifo_valid);

wire [RAM_WIDTH - 1:RAM_FIFO_ALIGH]     setup_ep0upd_updz  = ep_sta_length + {5'h0, setup_ep0upd_len} + 1'b1;
wire [EP_FIFO_STAT_WIDTH - 1:0]         setup_ep0upd_idata = { setup_ep0upd_updz, 1'b0, (setup_ep0upd_stall_flag) ? 1'b1 : ep_stall };

assign ep_stat_idata       = (ep_cur_strobe) ? ep_sis           : cnffifo_valid ? cnffifo_idata                 : setup_ep0upd_idata;
assign ep_stat_addr        = (ep_cur_strobe) ? ep_waddr         : cnffifo_valid ? cnffifo_data[EP_WIDTH - 1:0]  : setup_ep0upd_valid ? 5'h10 : 5'hxx;
assign ep_update_fifo_stat = (ep_cur_strobe) ? ep_cur_strobe_we : cnffifo_valid ? 1'b1                          : setup_ep0upd_valid;
assign cnffifo_ready       = (!ep_cur_strobe);
assign ep_addr  = active_ep;
assign ep_waddr = cur_ep;


wire                             token_toggle_bit = (axis_usb_rx_data[3:0] == R_PID_DATA0 ? 1'b0 : 1'b1);
reg [2:0]                        dataout_state;
localparam [2:0]
    SENDOUT_OK            = 0, // Succesfull wite to FIFO, send ACK
    SENDOUT_REPEAT        = 1, // Repeated data, send ACK
    SENDOUT_STALL         = 2, // EP stalled, send STALL
    SENDOUT_OVERFLOW      = 3, // Buffer overflow, send NAK
    SENDOUT_CORRUPTED     = 4, // bad CRC, ignore packet
    SENDOUT_OVERFLOW_NYET = 5; // Buffer overflow, send NYET in HS or NAK FS/LS, TODO take HS into account!!!

reg [1:0] in_addr_pre;

assign mmfifo_rb_ready = (act_state == ACT_FILL_BUFFER)  ? tx_ready :
                         (act_state == ACT_IN_WAIT_DATA) ? 1'b0 : 1'b1;

localparam [2:0]
    CTRLSTAT_NONE = 0,
    CTRLSTAT_CHECK_REQUEST = 1,
    CTRLSTAT_WAIT_DESCR = 2,
    CTRLSTAT_LOAD_DATA = 3,
    CTRLSTAT_LOAD_LEN = 4,
    CTRLSTAT_LOAD_COMMIT = 5,
    CTRLSTAT_SET_CONFIGURATION = 6,
    CTRLSTAT_CLEAR_EP_HALT = 7;

reg [2:0] ctrlstat;

// Data to push to EP
wire [6:0]setup_new_dev_addr = setup_wValue[6:0];

reg [7:0]                        enum_in_data;
reg                              setup_update_addr;
reg                              setup_req_stall;
reg                              setup_configuration;
reg [6:0]                        setup_curpkt_sz;
reg [RAM_WIDTH-2:RAM_FIFO_ALIGH] setup_offptr;
reg [RAM_WIDTH-2:RAM_FIFO_ALIGH] setup_offptr_cur;
reg [RAM_FIFO_ALIGH-1:0]         setup_offptr_cur_low;
reg [RAM_WIDTH-2:RAM_FIFO_ALIGH] setup_remaining;


reg [16:0]                       setup_req_len;
reg                              setup_extra_zerodp;
reg                              setup_extra_data;
reg                              setup_data_rom;

wire [RAM_WIDTH-2:RAM_FIFO_ALIGH] setup_offptr_nxt    = setup_offptr_cur + 1'b1;
wire [6:0]                        setup_curpkt_sz_nxt = setup_curpkt_sz + 1'b1;
wire [RAM_WIDTH-2:RAM_FIFO_ALIGH] setup_remaining_nxt = setup_remaining - 1'b1;
wire [16:0]                       setup_req_len_nxt   = setup_req_len - 1'b1;

assign setup_ep0upd_len        = setup_curpkt_sz[6:2];
assign setup_ep0upd_stall_flag = setup_req_stall;

reg                 usbrom_req_valid;
reg [7:0]           usbrom_req_data;

wire                usbrom_out_valid;
wire [7:0]          usbrom_out_data;
wire                usbrom_out_last;
wire                usbrom_out_nomatch;
wire                usbrom_out_ready = !axis_usb_rx_valid &&
        (act_state == ACT_NONE) && (tran_state == FSM_TRAN_NONE) && (ctrlstat == CTRLSTAT_LOAD_DATA) && (setup_remaining[RAM_WIDTH-2] == 0);

reg [11:0]  seen_reg;

reg [9:0]  setup_cnt;
reg [9:0]  in_cnt;
reg [9:0]  data_cnt;

/*
`ifndef SYM
ila_0 ila_0(
    .clk(clk),
    .probe0(axis_usb_rx_data),
    .probe1(axis_usb_rx_valid),
    .probe2(axis_usb_rx_last),
    .probe3(axis_usb_rx_error),
    .probe4(act_state),
    .probe5(ctrlstat),
    .probe6(token_skipped),

    .probe7(tx_data),
    .probe8(tx_valid),
    .probe9(tx_last),
    .probe10(tx_error),
    .probe11(tx_ready),

    .probe12(updfifo_valid),
    .probe13(updfifo_ready),

    .probe14(cnffifo_valid),
    .probe15(cnffifo_ready),

    .probe16(mmfifo_valid)

);
`endif
*/

assign cfg_ep_addr = (act_state == ACT_NONE && ctrlstat == CTRLSTAT_SET_CONFIGURATION && tran_state == FSM_TRAN_NONE) ? cur_ep : ep_addr;

generate
if (HWENUMERATOR) begin
usb_rom_descriptors #(
    .USDR_PID(USDR_PID)
) usb_rom_descriptors(
    .clk(clk),
    //.reset(reset), //act_state <= ACT_RESET_EP0_SETUP_IN
    .reset(act_state == ACT_RESET_EP0_SETUP_IN),

    .req_valid(usbrom_req_valid),
    .req_data(usbrom_req_data),
    .req_par(~cfg_usb_highspeed), // 0 - HS (480mbit); 1 - FS (12mbit)

    .out_valid(usbrom_out_valid),
    .out_data(usbrom_out_data),
    .out_last(usbrom_out_last),
    .out_nomatch(usbrom_out_nomatch),
    .out_ready(usbrom_out_ready)
);
end
endgenerate

reg       dbg_valid;
reg       dbg_dir;
reg [3:0] dbg_ep;
reg [3:0] dbg_type;

reg       dbg_in_packet;

/*
`ifndef SYM
ila_0 ila_0(
    .clk(clk),
    .probe0(dbg_valid),
    .probe1(dbg_dir),
    .probe2(dbg_ep),
    .probe3(dbg_type)
);
`endif
*/


always @(posedge clk) begin
    if (reset) begin
        usb_addr    <= 7'h00;
        configured  <= 0;

        tx_valid    <= 1'b0;
//        tran_status <= ST_IDLE;

        act_state   <= ACT_NONE;

        ep_cur_strobe     <= 1'b0;
        ep_cur_strobe_we  <= 1'b0;
        ep_cur_strobe_rdy <= 1'b0;
        mmfifo_valid  <= 1'b0;
        updfifo_valid <= 1'b0;

        ctrlstat      <= CTRLSTAT_NONE;

        setup_configuration <= 1'b0;
        usbrom_req_valid    <= 1'b0;
        setup_ep0upd_valid  <= 1'b0;

        cfg_usb_addr        <= 7'h0;
        cfg_usb_configured  <= 1'b0;

        ep_cur_rst_fifo_len <= 1'b0;

        seen_reg  <= 0;
        setup_cnt <= 0;
        in_cnt    <= 0;
        data_cnt  <= 0;

        setup_update_addr <= 0;

        dbg_valid     <= 1'b0;
        dbg_in_packet <= 1'b0;

    end else begin
        dbg_valid <= 1'b0;

        usbrom_req_valid <= 1'b0;

        if (ep_cur_strobe_rdy) begin
            ep_cur_strobe     <= 1'b0;
            ep_cur_strobe_we  <= 1'b0;
            ep_cur_strobe_rdy <= 1'b0;
        end

        if (tx_ready && tx_valid)
            tx_valid <= 1'b0;

        if (mmfifo_valid && mmfifo_ready)
            mmfifo_valid <= 1'b0;

        if (updfifo_valid && updfifo_ready)
            updfifo_valid <= 1'b0;

        if (setup_ep0upd_ready && setup_ep0upd_valid)
            setup_ep0upd_valid <= 1'b0;

        if (tx_valid && tx_ready) begin
            dbg_in_packet <= 1'b1;
            if (tx_last)
                dbg_in_packet <= 1'b0;

            if (!dbg_in_packet) begin
                dbg_valid <= 1'b1;
                dbg_dir   <= 1'b1;
                dbg_ep    <= active_ep;
                dbg_type  <= tx_data[3:0];
            end
        end

        if (HWENUMERATOR) begin
            case (ctrlstat)
            CTRLSTAT_CHECK_REQUEST: begin
                setup_update_addr <= 1'b0;
                setup_req_stall   <= 1'b0;
                setup_data_rom    <= 1'b1;
                setup_req_len     <= {1'b0, setup_wLength};
                setup_extra_data  <= 1'b0;
                setup_extra_zerodp<= 1'b0;
                setup_offptr_cur_low <= 2'b00;
                iter_ep              <= 5'h1;

                if (setup_bmRequestType[7]) begin
                    // IN Requests
                    // SETUP -> IN .... -> OUT (zero)

                    ctrlstat         <= CTRLSTAT_LOAD_DATA;
                    if (setup_bmRequestType[6:0] == 7'h02 && setup_bRequest == GET_STATUS && setup_wValue == 0) begin
                        enum_in_data <= 8'h0;
                        seen_reg[1]  <= 1'b1;
                    end else if (setup_bmRequestType[6:0] == 7'h0 && setup_bRequest == GET_STATUS) begin
                        enum_in_data <= 8'h0;
                        seen_reg[1]  <= 1'b1;
                    end else if (setup_bmRequestType[6:0] == 7'h0 && setup_bRequest == GET_CONFIGURATION) begin
                        enum_in_data <= { 7'h0, setup_configuration };
                        seen_reg[2]  <= 1'b1;
                    end else if (setup_bmRequestType[6:0] == 7'h0 && setup_bRequest == GET_DESCRIPTOR) begin
                        ctrlstat         <= CTRLSTAT_WAIT_DESCR;
                        usbrom_req_valid <= 1'b1;
                        usbrom_req_data  <= { setup_wValue[11:8], setup_wValue[3:0] };
                        seen_reg[0]      <= 1'b1;
                    end else if (setup_bmRequestType[6:0] == 7'h40 && setup_bRequest == 254) begin //ms-specific vendor code
                        ctrlstat         <= CTRLSTAT_WAIT_DESCR;
                        usbrom_req_valid <= 1'b1;
                        usbrom_req_data  <= { 4'd12, setup_wIndex[3:0] };
                        seen_reg[0]      <= 1'b1;
                    end else if (setup_bmRequestType[6:0] == 7'h40 && setup_bRequest == 252) begin //webusb vendor code
                        ctrlstat         <= CTRLSTAT_WAIT_DESCR;
                        usbrom_req_valid <= 1'b1;
                        usbrom_req_data  <= { 4'd13, setup_wIndex[3:0] };
                        seen_reg[0]      <= 1'b1;
                    end else begin
                        ctrlstat           <= CTRLSTAT_LOAD_COMMIT;
                        setup_req_stall    <= 1'b1;
                        setup_ep0upd_valid <= 1'b1;
                        seen_reg[3] <= 1'b1;
                    end
                end else begin
                    // OUT requests
                    // SETUP -> OUT .... -> IN (zero)
                    // SETUP -> IN (zero)

                    // Put 0 length IN packet, assume 0-length packets
                    setup_offptr_cur_low <= 2'b10;
                    ctrlstat             <= CTRLSTAT_LOAD_LEN;

                    if (setup_bmRequestType[6:0] == 7'h02 && setup_bRequest == CLEAR_FEATURE && setup_wValue == 0) begin
                        // Clear HALT BIT, reset FIFO position, size && send tonification to remote part
                        // setup_wIndex -- endpoint
                        iter_ep            <= { setup_wIndex[7], setup_wIndex[3:0] };
                        ctrlstat           <= CTRLSTAT_CLEAR_EP_HALT;
                    end else if (setup_bmRequestType[6:0] == 7'h0 && setup_bRequest == CLEAR_FEATURE) begin
                        // No features
                    end else if (setup_bmRequestType[6:0] == 7'h0 && setup_bRequest == SET_FEATURE) begin
                        // No features
                        seen_reg[5] <= 1'b1;
                    end else if (setup_bmRequestType[6:0] == 7'h0 && setup_bRequest == SET_ADDRESS) begin
                        seen_reg[4] <= 1'b1;
                        setup_update_addr  <= 1'b1;
                    end else if (setup_bmRequestType[6:0] == 7'h0 && setup_bRequest == SET_CONFIGURATION && setup_wValue[15:1] == 0) begin
                        // TODO: Should reset all EP to DATA0 state
                        cfg_usb_configured <= setup_wValue[0];
                        ctrlstat           <= CTRLSTAT_SET_CONFIGURATION;
                    end else begin
                        // STALL requests that we don't support
                        setup_req_stall    <= 1'b1;
                        ctrlstat           <= CTRLSTAT_LOAD_COMMIT;
                        setup_ep0upd_valid <= 1'b1;
                        seen_reg[6] <= 1'b1;
                    end
                end
            end

            CTRLSTAT_WAIT_DESCR: begin
                if (usbrom_out_valid) begin
                    ctrlstat         <= CTRLSTAT_LOAD_DATA;
                    setup_data_rom   <= 1'b1;
                    setup_req_len    <= setup_req_len_nxt; //Reqlen isn't in Z format
                    seen_reg[8] <= 1'b1;
                end
                if (usbrom_out_nomatch) begin
                    setup_req_stall    <= 1'b1;
                    setup_ep0upd_valid <= 1'b1;
                    ctrlstat           <= CTRLSTAT_LOAD_COMMIT;

                    seen_reg[7] <= 1'b1;
                end
            end

            endcase
        end

        if (axis_usb_rx_valid) begin
        // Process packet header (token)
        if (axis_usb_rx_valid && !axis_usb_rx_error && !token_skipped) begin
            dbg_valid <= (axis_usb_rx_data[3:0] != R_PID_SOF); //Skip SOF
            dbg_dir   <= 1'b0;
            dbg_ep    <= axis_usb_rx_data[7:4];
            dbg_type  <= axis_usb_rx_data[3:0];

            case (axis_usb_rx_data[3:0])
            R_PID_SETUP, R_PID_OUT, R_PID_IN, R_PID_PING: begin
                active_ep[3:0] <= axis_usb_rx_data[7:4];
                active_ep[4]   <= (axis_usb_rx_data[3:0] == R_PID_IN) ? 1'b1 : 1'b0;
                cur_ep[3:0]    <= axis_usb_rx_data[7:4];
                cur_ep[4]      <= (axis_usb_rx_data[3:0] == R_PID_IN) ? 1'b1 : 1'b0;

                ep_type        <= (axis_usb_rx_data[7:4] == 0) ? EPTYPE_CONTROL : EPTYPE_BULK; //EP0 is for Control only
                tran_state     <= (axis_usb_rx_data[3:0] == R_PID_OUT)   ? FSM_TRAN_OUT   :
                                  (axis_usb_rx_data[3:0] == R_PID_IN)    ? FSM_TRAN_IN    :
                                  (axis_usb_rx_data[3:0] == R_PID_SETUP) ? FSM_TRAN_SETUP : FSM_TRAN_NONE;

                ep_cur_strobe       <= 1'b1;
                ep_cur_strobe_we    <= 1'b0;
                ep_cur_strobe_rdy   <= 1'b0;
                ep_cur_rst_fifo_len <= 1'b0;

                if (axis_usb_rx_data[3:0] == R_PID_SETUP) begin
                    // New control request: reset control request state, reset STALL state, reset toggle bit
                    act_state <= ACT_RESET_EP0_SETUP_IN;
                    setup_cnt <= setup_cnt + 1'b1;
                end else if (axis_usb_rx_data[3:0] == R_PID_IN) begin
                    act_state <= ACT_CHECK_IN;
                    in_cnt    <= in_cnt + 1'b1;
                end else if (axis_usb_rx_data[3:0] == R_PID_PING) begin
                    act_state <= ACT_CHECK_PING;
                end else begin
                    // Can't get OUT token from host!!! only possible in OTG mode
                    act_state <= ACT_WAIT_DATA;
                end
            end

            R_PID_ACK: begin
                if (tran_state == FSM_TRAN_IN) begin
                    // Update EP state
                    ep_cur_toggle      <= ~ep_toggle;
                    ep_cur_rst_fifo_len<= 1'b0;
                    ep_cur_stall_clear <= 1'b0;
                    ep_cur_nyetflag    <= 1'b0;
                    ep_cur_strobe      <= 1'b1;
                    ep_cur_strobe_we   <= 1'b1;
                    ep_cur_strobe_rdy  <= 1'b1; // autoclear
                    cur_ep             <= active_ep;

                    // Advance buffer
                    updfifo_valid      <= 1'b1;
                    updfifo_data       <= { bcnt[MAX_ENDPOINT_SZ_WIDTH:2], active_ep };

                    // TODO: This confirmation ACK in control transfer might be loss!!! add a workarond for proper filtering in the case.
                    // i.e. maximum timeout
                    if (ep_type == EPTYPE_CONTROL) begin
                        seen_reg[9] <= 1'b1;

                        if (setup_update_addr) begin
                            seen_reg[10] <= 1'b1;
                            cfg_usb_addr  <= setup_new_dev_addr;
                        end

                        setup_remaining <= setup_remaining + bcnt[MAX_ENDPOINT_SZ_WIDTH:2];
                    end
                end

                tran_state         <= FSM_TRAN_NONE;
            end

            R_PID_DATA0, R_PID_DATA1: begin
                length_remaining   <= ep_sta_length;
                ep_cur_offptr      <= ep_sta_offptr;
                ep_cur_offptr_low  <= {RAM_FIFO_ALIGH{1'b1}};
                mmfifo_addr        <= { ep_rampos,      {RAM_FIFO_ALIGH{1'b1}}};
                mmfifo_data        <= axis_usb_rx_data;
                mmfifo_valid       <= 1'b0;
                mmfifo_wr          <= 1'b1;
                mmfifo_tag         <= 1'b0;
                dataout_state      <= (ep_stall)                      ? SENDOUT_STALL    :
                                      (token_toggle_bit != ep_toggle) ? SENDOUT_REPEAT   :
                                      (ep_no_avail)                   ? SENDOUT_OVERFLOW :
                                      (axis_usb_rx_error)             ? SENDOUT_CORRUPTED : SENDOUT_OK;

                bcnt               <= ~0; //1st DW is control
                ep_cur_strobe_rdy  <= 1'b1;
                if (axis_usb_rx_last) begin
                    act_state       <= ACT_UPD_LEN_ACK;
                    data_cnt        <= data_cnt + 1'b1;
                end
            end

            endcase
        end else if (axis_usb_rx_valid && token_skipped && (tran_state == FSM_TRAN_OUT || tran_state == FSM_TRAN_SETUP)) begin
            mmfifo_data      <= axis_usb_rx_data;
            mmfifo_valid     <= (dataout_state == SENDOUT_OK);

            if (ep_cur_offptr_low == 2'b11) begin
                length_remaining <= length_remaining - 1'b1;
                ep_cur_offptr    <= ep_nxt_offptr;
                mmfifo_addr      <= { ep_nxtaddr, 2'b00 };
            end else begin
                mmfifo_addr[1:0] <= ep_cur_offptr_low + 1'b1;
            end

            ep_cur_offptr_low <= ep_cur_offptr_low + 1'b1;

            //////////////////////////////////////////////////////////////////////////  TODO check NYET/NAK ///////////////////////////
            if (dataout_state == SENDOUT_OK && length_remaining_nxtzero) begin
                dataout_state <= SENDOUT_OVERFLOW_NYET;
                mmfifo_valid  <= 1'b0; //Do not overflow buffer
            end
            if (dataout_state == SENDOUT_OK && mmfifo_valid && !mmfifo_ready) begin
                dataout_state <= SENDOUT_OVERFLOW; // Congestion on memory buffer
            end
            if (axis_usb_rx_error) begin
                dataout_state <= SENDOUT_CORRUPTED;
            end

            bcnt              <= bcnt + 1'b1;

            if (axis_usb_rx_last) begin
                act_state       <= ACT_UPD_LEN_ACK;
            end
        end
        end else case (act_state) //No active recieve function
        ACT_NONE: begin
            // No transaction activity, can occupy FIFO for HW enumeration
            if (HWENUMERATOR && tran_state == FSM_TRAN_NONE) begin
                if (ctrlstat == CTRLSTAT_SET_CONFIGURATION || ctrlstat == CTRLSTAT_CLEAR_EP_HALT) begin
                    // Reset all FIFOs and configure EPs except EP0
                    if (!updfifo_valid || updfifo_ready) begin
                        ep_cur_toggle       <= 1'b0;
                        ep_cur_rst_fifo_len <= 1'b1;
                        ep_cur_offptr       <= 0;
                        ep_cur_nyetflag     <= 1'b0;
                        ep_cur_stall_clear  <= 1'b1;
                        ep_cur_strobe       <= 1'b1;
                        ep_cur_strobe_we    <= (iter_ep != 5'h10);
                        ep_cur_strobe_rdy   <= 1'b1;
                        cur_ep              <= iter_ep;
                        iter_ep             <= iter_ep + 1'b1;

                        //Reset frontend FIFO (don't wait for ready FE might be in reset)
                        updfifo_valid      <= (iter_ep != 5'h10);
                        updfifo_data       <= { 2'b10, {(MAX_ENDPOINT_SZ_WIDTH-3){1'b1}}, iter_ep };

                        // Fill zero packet when configuration is done
                        if (iter_ep == 31 || ctrlstat == CTRLSTAT_CLEAR_EP_HALT) begin
                            ctrlstat       <= CTRLSTAT_LOAD_LEN;
                        end
                    end
                end else if (ctrlstat == CTRLSTAT_LOAD_DATA && (setup_remaining[RAM_WIDTH-2] == 0)) begin
                    mmfifo_addr        <= { 1'b0, setup_offptr_cur, setup_offptr_cur_low };

                    mmfifo_data        <= (setup_data_rom) ? usbrom_out_data : enum_in_data;
                    mmfifo_valid       <= 1'b1;
                    mmfifo_wr          <= 1'b1;
                    mmfifo_tag         <= 1'b0;

                    enum_in_data       <= 0;

                    setup_offptr_cur_low <= setup_offptr_cur_low + 1'b1;
                    if (setup_offptr_cur_low == 2'b11) begin
                        setup_offptr_cur <= setup_offptr_nxt;
                        setup_remaining  <= setup_remaining_nxt;
                    end

                    setup_req_len   <= setup_req_len_nxt;
                    setup_curpkt_sz <= setup_curpkt_sz_nxt;
                    if (setup_curpkt_sz_nxt[6] || setup_req_len_nxt[16] || (setup_data_rom && usbrom_out_last)) begin
                        ctrlstat             <= CTRLSTAT_LOAD_LEN;
                        setup_offptr_cur_low <= 2'b10;
                        setup_extra_zerodp   <= setup_curpkt_sz_nxt[6] && !setup_req_len_nxt[16];
                        setup_extra_data     <= !setup_req_len_nxt[16] && (!usbrom_out_last || !setup_data_rom);
                    end else begin
                        //setup_curpkt_sz      <= setup_curpkt_sz_nxt;
                    end

                end else if (ctrlstat == CTRLSTAT_LOAD_LEN) begin
                    mmfifo_addr        <= { 1'b0, setup_offptr, setup_offptr_cur_low };
                    mmfifo_data        <= (setup_offptr_cur_low[0] == 1'b0) ? { 5'h0, {(MAX_EPSZ_WIDTH-8){setup_curpkt_sz[6]}} } : { setup_curpkt_sz[6], setup_curpkt_sz };

                    mmfifo_valid       <= 1'b1;
                    mmfifo_wr          <= 1'b1;
                    mmfifo_tag         <= 1'b0;

                    setup_offptr_cur_low <= setup_offptr_cur_low + 1'b1;

                    if (setup_offptr_cur_low == 2'b11) begin
                        setup_offptr         <= setup_offptr_cur;
                        ctrlstat             <= CTRLSTAT_LOAD_COMMIT;
                        setup_curpkt_sz      <= setup_curpkt_sz + 2'b11;
                        setup_ep0upd_valid   <= 1'b1;
                    end
                end else if (ctrlstat == CTRLSTAT_LOAD_COMMIT) begin
                    if (setup_ep0upd_ready) begin
                        setup_extra_zerodp   <= 1'b0;
                        setup_curpkt_sz      <= ~0;
                        setup_offptr_cur_low <= setup_extra_zerodp ? 2'b10 : 2'b00;

                        if (setup_extra_data) begin
                            ctrlstat           <= CTRLSTAT_LOAD_DATA;
                        end else if (setup_extra_zerodp) begin
                            ctrlstat           <= CTRLSTAT_LOAD_LEN;
                        end else begin
                            ctrlstat           <= CTRLSTAT_NONE;
                        end
                    end
                end
            end
        end

        ACT_RESET_EP0_SETUP_IN, ACT_RESET_EP0_SETUP_OUT: begin
            if (HWENUMERATOR) begin
                //Make EP0 available for HW enumertion (reset it)
                setup_curpkt_sz  <= ~0;
                setup_offptr     <= 0;
                setup_offptr_cur <= 1; //1DW off to data pointer
                setup_remaining  <= { cfg_ep_fifozsz, {(RAM_PARAGRAPH_WIDTH - RAM_FIFO_ALIGH){1'b1}}};

                setup_ep0upd_valid <= 0;
            end

            ep_cur_rst_fifo_len <= 1'b1;
            ep_cur_offptr       <= 0;
            ep_cur_nyetflag     <= 1'b0;
            ep_cur_stall_clear  <= 1'b1;
            ep_cur_strobe       <= 1'b1;
            ep_cur_strobe_we    <= 1'b1;
            ep_cur_strobe_rdy   <= 1'b0;
            cur_ep              <= { (act_state == ACT_RESET_EP0_SETUP_IN) ? 1'b1 : 1'b0, active_ep[3:0] };

            //Reset frontend FIFO (don't wait for ready FE might be in reset)
            updfifo_valid      <= 1'b1;
            updfifo_data       <= { 2'b10, {(MAX_ENDPOINT_SZ_WIDTH-3){1'b1}}, (act_state == ACT_RESET_EP0_SETUP_IN) ? 1'b1 : 1'b0, active_ep[3:0] };

            if (act_state == ACT_RESET_EP0_SETUP_IN) begin
                ep_cur_toggle  <= 1'b1; //We expect always DATA1 as 1st IN in Data stage or DATA1 IN in status
                act_state      <= ACT_RESET_EP0_SETUP_OUT;
            end else begin
                ep_cur_toggle  <= 1'b0; //We expect always DATA0 after SETUP
                act_state      <= ACT_NONE;
            end
        end

        ACT_UPD_LEN_ACK: begin
            if ((~mmfifo_valid || mmfifo_ready) && !tx_valid) begin
                // Send only ACK for SETUP trans
                // Send ACK, NAK, STALL, NYET or nothing
                length_remaining <= length_remaining - 1'b1;  //length_remaining_nxtzero
                ep_cur_offptr    <= ep_nxt_offptr;

                //////////////////////////////////////////////////////////// NYET won't be sent!!!!!!!!!!!!!!!!!!!!
                tx_data  <= {4'h0, (dataout_state == SENDOUT_STALL)                                              ? R_PID_STALL :
                                   (dataout_state == SENDOUT_OK       || dataout_state == SENDOUT_REPEAT)        ? R_PID_ACK   :
                                   (dataout_state == SENDOUT_OVERFLOW || dataout_state == SENDOUT_OVERFLOW_NYET) ? R_PID_NAK   : R_PID_NYET};
                tx_valid <= (dataout_state == SENDOUT_CORRUPTED || dataout_state == SENDOUT_OK)  ? 1'b0 : 1'b1;
                tx_last  <= 1'b1;
                tx_error <= 1'b0;

                if (dataout_state == SENDOUT_OK) begin
                    mmfifo_addr        <= { ep_rampos,     2'b10 };
                    mmfifo_data        <= { (tran_state == FSM_TRAN_SETUP) ?  1'b1 : 1'b0, {(14-MAX_ENDPOINT_SZ_WIDTH){1'b0}}, bcnt[MAX_ENDPOINT_SZ_WIDTH:8]};
                    mmfifo_valid       <= 1'b1;
                    mmfifo_wr          <= 1'b1;
                    mmfifo_tag         <= 1'b0;

                    act_state          <= ACT_UPD_LEN_CMT;
                end else begin
                    act_state          <= ACT_NONE;
                    tran_state         <= FSM_TRAN_NONE;
                end
            end
        end

        ACT_UPD_LEN_CMT: begin
            if ((~mmfifo_valid || mmfifo_ready) && !tx_valid && !updfifo_valid) begin
                // Advance FIFO packet informtion
                updfifo_valid      <= 1'b1;
                updfifo_data       <= { bcnt[MAX_ENDPOINT_SZ_WIDTH:2], active_ep };

                // Commit counters
                ep_cur_rst_fifo_len<= 1'b0;
                ep_cur_stall_clear <= 1'b0;
                ep_cur_nyetflag    <= (dataout_state == SENDOUT_OVERFLOW_NYET) ? 1'b1 : 1'b0;
                ep_cur_strobe      <= 1'b1;
                ep_cur_strobe_we   <= 1'b1;
                ep_cur_strobe_rdy  <= 1'b1;
                ep_cur_toggle      <= (dataout_state == SENDOUT_OK) ? ~ep_toggle : ep_toggle;
                cur_ep             <= active_ep;

                // Send ACK
                tx_valid           <= 1'b1;

                mmfifo_addr        <= { ep_rampos,  2'b11 };
                mmfifo_data        <= bcnt[7:0];
                mmfifo_valid       <= 1'b1;
                mmfifo_wr          <= 1'b1;
                mmfifo_tag         <= 1'b0;

                act_state          <= ACT_NONE;
                tran_state         <= FSM_TRAN_NONE;
                if (tran_state == FSM_TRAN_SETUP) begin
                    ctrlstat       <= CTRLSTAT_CHECK_REQUEST;
                end
            end
        end

        ACT_CHECK_PING: begin
            if (!tx_valid || tx_ready) begin
                tx_data  <= {4'h0, (ep_stall)    ? R_PID_STALL :
                                   (ep_no_avail/*ep_nyetflag*/) ? R_PID_NAK   : R_PID_ACK};
                tx_valid <= 1'b1;
                tx_last  <= 1'b1;
                tx_error <= 1'b0;

                act_state         <= ACT_NONE;
                tran_state        <= FSM_TRAN_NONE;
                ep_cur_strobe_rdy <= 1'b1;
            end
        end

        ACT_CHECK_IN: begin
            if (ep_stall || ep_no_avail) begin
                if (!tx_valid || tx_ready) begin
                    tx_data  <= {4'h0, (ep_stall) ? R_PID_STALL : R_PID_NAK};
                    tx_valid <= 1'b1;
                    tx_last  <= 1'b1;
                    tx_error <= 1'b0;

                    act_state         <= ACT_NONE;
                    tran_state        <= FSM_TRAN_NONE;
                    ep_cur_strobe_rdy <= 1'b1;
                end
            end else begin
                //Request packet size
                mmfifo_addr        <= { ep_rampos, 2'b10};
                mmfifo_valid       <= 1'b1;
                mmfifo_wr          <= 1'b0;
                mmfifo_tag         <= 1'b0;

                length_remaining   <= ep_sta_length;
                ep_cur_offptr      <= ep_sta_offptr;
                ep_cur_offptr_low  <= 2'b10;

                in_addr_pre        <= 0;
                act_state          <= ACT_LEN_BUFFER0;
                ep_cur_strobe_rdy  <= 1'b1;
            end
        end

        // TODO reduce cycle delay
        // Now it's ok: When a device with a detachable cable responds to a packet from a host, it will provide an inter-packet delay of
        // at most 192 bit times measured at the B receptacle
        ACT_LEN_BUFFER0, ACT_LEN_BUFFER1, ACT_IN_WAIT_DATA, ACT_FILL_BUFFER: begin
            if ((!mmfifo_valid || mmfifo_ready) && (in_addr_pre == 0 || in_addr_pre == 3)) begin
                if (ep_cur_offptr_low == 2'b11  /*|| (bcnt_sent_next[MAX_ENDPOINT_SZ_WIDTH] && in_addr_pre == 3)*/) begin
                    length_remaining <= length_remaining - 1'b1;
                    ep_cur_offptr    <= ep_nxt_offptr;
                    mmfifo_addr      <= { ep_nxtaddr, 2'b00 };
                end else begin
                    mmfifo_addr[1:0] <= ep_cur_offptr_low + 1'b1;
                end
                ep_cur_offptr_low <= ep_cur_offptr_low + 1'b1;

                mmfifo_valid     <= 1'b1;
                mmfifo_tag       <= 1'b0;

                in_addr_pre      <= in_addr_pre + 1'b1;
                if (in_addr_pre == 3) begin
                    mmfifo_tag   <= (bcnt_sent_next[MAX_ENDPOINT_SZ_WIDTH]) ? 1'b1 : 1'b0;
                    in_addr_pre  <= (bcnt_sent_next[MAX_ENDPOINT_SZ_WIDTH]) ? 2 : 3;
                    bcnt_sent    <= bcnt_sent_next;
                end
            end

            if (mmfifo_rb_valid) begin
                if (act_state == ACT_LEN_BUFFER0) begin
                    bcnt[MAX_ENDPOINT_SZ_WIDTH:8]      <= mmfifo_rb_data[MAX_ENDPOINT_SZ_WIDTH-8:0];
                    bcnt_sent[MAX_ENDPOINT_SZ_WIDTH:8] <= mmfifo_rb_data[MAX_ENDPOINT_SZ_WIDTH-8:0];
                    bcnt_sent[7:0]                     <= 0;
                    act_state                          <= ACT_LEN_BUFFER1;
                end else if (act_state == ACT_LEN_BUFFER1) begin
                    bcnt[7:0]                          <= mmfifo_rb_data;
                    bcnt_sent[7:0]                     <= mmfifo_rb_data;

                    tx_data  <= {4'h0, (ep_toggle) ? R_PID_DATA1 : R_PID_DATA0};
                    tx_valid <= bcnt[MAX_ENDPOINT_SZ_WIDTH];
                    tx_last  <= bcnt[MAX_ENDPOINT_SZ_WIDTH];
                    tx_error <= 1'b0;

                    if (bcnt[MAX_ENDPOINT_SZ_WIDTH]) begin
                        act_state   <= ACT_NONE;
                        in_addr_pre <= 2;

                        length_remaining <= length_remaining - 1'b1;
                        ep_cur_offptr    <= ep_nxt_offptr;
                    end else begin
                        act_state   <= ACT_IN_WAIT_DATA;
                        in_addr_pre <= 3;
                    end
                end else if (act_state == ACT_IN_WAIT_DATA) begin
                    // Got 1st byte from new request
                    tx_valid    <= 1'b1;
                    act_state   <= ACT_FILL_BUFFER;
                    //

                end else if (act_state == ACT_FILL_BUFFER) begin
                    if (tx_ready) begin
                        tx_data   <= mmfifo_rb_data;
                        tx_valid  <= 1'b1;
                        tx_error  <= 1'b0;

                        if (mmfifo_rb_tag) begin
                            tx_last      <= 1'b1;
                            act_state    <= ACT_NONE;

                            length_remaining <= length_remaining - 1'b1;
                            ep_cur_offptr    <= ep_nxt_offptr;
                        end else begin
                            tx_last      <= 1'b0;
                            act_state    <= ACT_FILL_BUFFER;
                        end
                    end
                end
            end else if (/*!tx_last &&*/ !tx_valid && tx_ready && (act_state == ACT_FILL_BUFFER)) begin
                //Buffer underrun, terminate transaction with an error
                tx_last  <= 1'b1;
                tx_error <= 1'b1;
                tx_valid <= 1'b1;
                tx_data  <= mmfifo_rb_data;

                mmfifo_valid <= 1'b0;
                act_state    <= ACT_NONE;
            end
        end

        endcase


    end
end

//2, 4, 3
assign debug_state = { seen_reg[10:0], tran_state, act_state, ctrlstat };
assign debug2      = { in_cnt, data_cnt, setup_cnt };

endmodule
