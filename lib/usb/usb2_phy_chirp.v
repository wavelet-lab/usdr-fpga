// USB2 ULPI PHY with embedded Chirp & wakeup fsm
// crc5 and crc16 is checked here


// OUT, IN, SETUP, and PING token packets
// PID 	ADDR 	ENDP 	CRC5
//   8     7       4       5 	
//
// SOF
// PID 	Frame number 	CRC5
//   8            11       5
//
// DATA
// PID 	DATA 	CRC16 
//   8  0–8192     16
//
// ACK, NACK, STALL, NYET
// PID  
//   8

//
//
//  first pcket
//  [7:4]  active endpoint
//  [3:0]  PID
//
// 

module usb2_phy_chirp #(
  parameter NO_ADDR_CHECK = 1'b0
)(
    // PHY
    input            phy_clk,

    output reg [7:0] phy_do,
    input      [7:0] phy_di,
    output           phy_doe,

    input            phy_dir,
    input            phy_nxt,
    output reg       phy_stp,

    // Control iface
    input            reset,

    // Streaming IN (AXI stream)
    output reg [7:0] axis_usb_rx_data,
    output reg       axis_usb_rx_valid,
    output reg       axis_usb_rx_last,
    output reg       axis_usb_rx_error,

    // Streaming OUT (AXI stream)
    input [7:0]      usb_out_data,
    input            usb_out_valid,
    input            usb_out_txerror, //TX error sampled on the LAST byte
    input            usb_out_last,
    output           usb_out_ready,

    // Configuration
    input            cfg_usb2_en,
    input [6:0]      cfg_usb_addr,

    // Status
    output           stat_usb_hs,        // Entered HS mode succesfully
    output           stat_phy_ready,     // All initializations are done, phy is ready
    output           stat_usb_reset,

    output           host_disconnected,

    output [9:0] debug_state
);

// phy_dir: 1 - PHY sends data, 0 - PHY recives data
// rxactive == phy_dir && phy_nxt
reg phy_dir_prev = 1'b1;
always @(posedge phy_clk) begin
    phy_dir_prev <= phy_dir;
end

assign phy_doe = ~phy_dir;


///////////////////////////////////////////////////
// Generic UTMI registers
reg [1:0] utmi_linestate;


reg phy_ctrl_valid;
reg phy_ctrl_ready;
wire phy_ctrl_wr = 1'b1;

reg        usb_out_chirp_valid;

localparam LS_SE0 = 2'b00;
localparam LS_J   = 2'b01;
localparam LS_K   = 2'b10;
localparam LS_SE1 = 2'b11;

localparam LS_SQUELCH  = 2'b00;
localparam LS_NSQUELCH = 2'b01;


localparam TIME_1US = 60;
localparam TIME_1MS = 1000*TIME_1US;

localparam TIME_RST_SE0 = 150;
localparam TIME_CHIRP_K_NIN = 1*TIME_1MS;
localparam TIME_CHIRP_K_MAX = 6*TIME_1MS;

localparam TIME_RES_KJKJKJ_MAX = 6000;

localparam TIME_NO_ACTIVITY = 3*TIME_1MS;

localparam TIME_SYM = 150;


localparam TIME_CHRIP_JK_MIN = 40*TIME_1US;
localparam TIME_CHRIP_JK_MAX = 60*TIME_1US;


reg [3:0]  phyctrl_state;
localparam [3:0] 
    STCHIRP_RESET         = 0,
    STCHIRP_RESET_WAIT    = 1,
    STCHIRP_OTG_CLEAR     = 2,
    STCHIRP_FS_ENABLE     = 3,
    STCHIRP_FS_IDLE       = 4,
    STCHIRP_USB_RESET     = 5,
    STCHIRP_FS_CHIRP_K    = 6,
    STCHIRP_FS_WAIT_K     = 7,
    STCHIRP_FS_WAIT_J     = 8,
    STCHIRP_HS_RESET      = 9,
    STCHIRP_HS_IDLE       = 10,
    STCHIRP_HS_CHECK_SUSP = 11,
    STCHIRP_ROLLBACK_FS   = 12,
    STCHIRP_SUSPENDED     = 13,
    STCHIRP_RESUME        = 14,
    STCHIRP_RBRESET_FS    = 15
;



// ULPI Register
localparam REG_ULPI_FUNCTION    = 6'h04;
// XcvrSelect [1:0]
localparam ULPI_FUNC_XCVR_HS    = 8'b0000_0000;
localparam ULPI_FUNC_XCVR_FS    = 8'b0000_0001;
localparam ULPI_FUNC_XCVR_LS    = 8'b0000_0010;
localparam ULPI_FUNC_XCVR_FS_LS = 8'b0000_0011;
// TermSelect[2]
localparam ULPI_FUNC_NO_TERM    = 8'b0000_0000;
localparam ULPI_FUNC_EN_TERM    = 8'b0000_0100;
// OpMode[4:3]
localparam ULPI_FUNC_OP_NORM    = 8'b0000_0000;
localparam ULPI_FUNC_OP_NON_DR  = 8'b0000_1000;
localparam ULPI_FUNC_OP_DBSNZRI = 8'b0001_0000;
localparam ULPI_FUNC_OP_NOSYNC  = 8'b0001_1000;
// Reset[5]
localparam ULPI_FUNC_RESET      = 8'b0010_0000;
// SuspendM[6]
localparam ULPI_FUNC_LOW_POWER  = 8'b0000_0000;
localparam ULPI_FUNC_POWERED    = 8'b0100_0000;

localparam REG_ULPI_OTG = 6'h0a;
localparam ULPI_OTG_DEF = 8'b0000_0000;


// RPU_PD_EN (1.5 kOhm on DP, HS Trx)
localparam ULPI_PERPH_CHIRP     = ULPI_FUNC_XCVR_HS | ULPI_FUNC_EN_TERM | ULPI_FUNC_OP_DBSNZRI | ULPI_FUNC_POWERED;
// HSTERM_EN (45 Ohm on DP/DM) but able to distinguish SE0/J/K
localparam ULPI_PERPH_TEST_JK   = ULPI_FUNC_XCVR_HS | ULPI_FUNC_NO_TERM | ULPI_FUNC_OP_DBSNZRI | ULPI_FUNC_POWERED;
// HSTERM_EN (45 Ohm on DP/DM)
localparam ULPI_PERPH_HS        = ULPI_FUNC_XCVR_HS | ULPI_FUNC_NO_TERM | ULPI_FUNC_OP_NORM    | ULPI_FUNC_POWERED;
// RPU_PD_EN (1.5 kOhm on DP)
localparam ULPI_PERPH_FS        = ULPI_FUNC_XCVR_FS | ULPI_FUNC_EN_TERM | ULPI_FUNC_OP_NORM    | ULPI_FUNC_POWERED;

localparam ULPI_PERPH_FS_RESUME = ULPI_FUNC_XCVR_FS | ULPI_FUNC_EN_TERM | ULPI_FUNC_OP_DBSNZRI | ULPI_FUNC_POWERED;



// 20 bit timer wraps 17.48mS, enough to hold all required timings
localparam T0_WIDTH = 20;
localparam T1_WIDTH = 9;


reg [T0_WIDTH - 1:0] T0_value;
reg                  T0_reset;
reg                  T0_active;
reg                  T0_state;

reg [T1_WIDTH - 1:0] T1_value;
reg                  T1_reset;
reg                  T1_active;
reg                  T1_state;

wire T0_to_wtrstfs = !T0_reset && (T0_value[8]);                                   // 4.2us
wire T0_to_idle3ms = !T0_reset && (T0_value[17] && T0_value[15] && T0_value[14]);  // 3.0ms
wire T0_to_uch     = !T0_reset && (T0_value[16]);                                  // 1.1ms
wire T0_to_wtfs    = !T0_reset && (T0_value[16]);                                  // 1.1ms
wire T0_to_wtrev   = !T0_reset && (T0_value[17] && T0_value[15] && T0_value[14]);  // 3.0ms
wire T0_to_wtrsths = !T0_reset && (T0_value[14]);                                  // 273us

wire T1_to_filt    = !T1_reset && (T1_value[8]);                   // 4.2us

always @(posedge phy_clk) begin
  if (T0_reset) begin
    T0_value <= 0;
  end else begin
    if (T0_active) begin
        T0_value <= T0_value + 1'b1;
    end
  end
end

always @(posedge phy_clk) begin
  if (T1_reset) begin
    T1_value <= 0;
  end else begin
    if (T1_active) begin
        T1_value <= T1_value + 1'b1;
    end
  end
end


reg       usb2_reset;
reg       usb2_highspeed;
reg [7:0] phy_ctrl_reg_data;
reg [5:0] phy_ctrl_reg_addr;

localparam DEF_LATENCY = 7; // 128   >2us


reg [1:0]  chirp_kj_cnt;

// Link state FSM
//  must recoginse
// 1. Reset => Idle(J)->SE0 (after 2.5us detected as reset) and hold for 10ms -> Idle
// 2. Suspend after 3ms not sending anything
// 3. Resume =>  J->K (for 20ms) -> EOP
// TODO USB2.0 wakeup / suspend
// >3ms idle => suspended state

// T WTCLK should have a value somewhere between 0 and 5.0 ms ( is used to have the device wait for a stable clock )
//            |   min   |   max    |
// T FILTSE0  | 2.5uS   |          | Time for which a suspended high-speed capable device must see a continuous SE0 before beginning the high-speed detection handshake
// T WTRSTFS  | 2.5uS   | 3000uS   | Time a high-speed capable device operating in non-suspended full-speed must wait after start of SE0 before beginning the high-speed detection handshake
// T WTREV    | 3.0ms   | 3.125ms  | Time a high-speed capable device operating in high-speed must wait after start of SE0 before reverting to full-speed
// T WTRSTHS  | 100uS   | 875uS    | Time a device must wait after reverting to full-speed before sampling the bus state for SE0 and beginning the high-speed detection handshake
// T UCH      | 1.0ms   |          | Minimum duration of a Chirp K from a high-speed capable device within the reset protocol
// T UCHEND   |         | 7.0ms    | Time after start of SE0 by which a high-speed capable device is required to have completed its Chirp K within the reset protocol
// T WTHS     |         | 500uS    | Time between detection of downstream chirp and entering high-speed state
// T WTFS     | 1.0ms   | 2.5ms    | Time after end of upstream chirp at which device reverts to full-speed default state if no downstream chirp is detected
// T FILT     | 2.5uS

// FULL SPEED
// J   -- IDLE STATE
// K   -- Resume after suspend 
// SE0 -- Reset / EOP

/*
ila_1 ila_1(
    .clk(phy_clk),
    .probe0(phyctrl_state),
    .probe1(phyctrl_state),
    
    .probe2(utmi_linestate),
    .probe3(phy_dir),
    .probe4(phy_stp),
    .probe5(phy_ctrl_valid)
);
*/

always @(posedge phy_clk) begin
  if (reset) begin
    phyctrl_state       <= STCHIRP_RESET;
    phy_ctrl_valid      <= 1'b0;
    usb_out_chirp_valid <= 1'b0;
    usb2_highspeed      <= 1'b0;
    T0_reset            <= 1'b1;
    T1_reset            <= 1'b1;
    T0_active           <= 1'b0;
    T1_active           <= 1'b1;
    usb2_reset          <= 1'b1;
  end else begin
    if (phy_ctrl_valid && phy_ctrl_ready) begin
      phy_ctrl_valid <= 1'b0;
    end

    T0_reset      <= 1'b0;
    T1_reset      <= 1'b0;

    case (phyctrl_state)
    STCHIRP_RESET: begin
      if (phy_dir == 0 && phy_stp == 0 && !T1_reset && T1_value[DEF_LATENCY]) begin
        phy_ctrl_reg_data <= ULPI_FUNC_RESET | ULPI_FUNC_POWERED;
        phy_ctrl_reg_addr <= REG_ULPI_FUNCTION;
        phy_ctrl_valid    <= 1'b1;
        phyctrl_state     <= STCHIRP_RESET_WAIT;
      end
    end

    STCHIRP_RESET_WAIT: begin
      if (phy_dir_prev == 1'b1 && phy_dir == 1'b0) begin
        phyctrl_state     <= STCHIRP_OTG_CLEAR;
        T1_reset          <= 1'b1;
        T1_active         <= 1'b1;
      end
    end

    STCHIRP_OTG_CLEAR: begin
      if (phy_ctrl_valid == 0 && !T1_reset && T1_value[DEF_LATENCY]) begin
        phy_ctrl_reg_data <= ULPI_OTG_DEF;
        phy_ctrl_reg_addr <= REG_ULPI_OTG;
        phy_ctrl_valid    <= 1'b1;
        phyctrl_state     <= STCHIRP_FS_ENABLE;
        T1_reset          <= 1'b1;
        T1_active         <= 1'b0;
      end
    end

    STCHIRP_FS_ENABLE: begin
      if (phy_ctrl_valid == 0) begin
        phy_ctrl_reg_data <= ULPI_PERPH_FS;
        phy_ctrl_reg_addr <= REG_ULPI_FUNCTION;
        phy_ctrl_valid    <= 1'b1;
        phyctrl_state     <= STCHIRP_FS_IDLE;
      end
    end

    STCHIRP_FS_IDLE: begin
      // Powered, FS default, FS address, FS configured
      if (!T0_active && (utmi_linestate == LS_SE0 || utmi_linestate == LS_J)) begin
        T0_reset          <= 1'b1;
        T0_active         <= 1'b1;
        T0_state          <= (utmi_linestate == LS_SE0);
      end else if (T0_active) begin
        if ((utmi_linestate == LS_SE0) != T0_state) begin
          // Line state change
          T0_reset          <= 1'b1;
          T0_active         <= 1'b0;
        end else if (T0_state && T0_to_wtrstfs) begin
          //Reset sequence detected
          T0_reset          <= 1'b1;
          T0_active         <= 1'b0;

          phyctrl_state     <= STCHIRP_USB_RESET;
        end else if (!T0_state && T0_to_idle3ms) begin
          //Suspend sequence detected
          T0_reset          <= 1'b1;
          T0_active         <= 1'b0;

          phyctrl_state     <= STCHIRP_SUSPENDED;
          phy_ctrl_reg_data <= ULPI_PERPH_FS_RESUME;
          phy_ctrl_reg_addr <= REG_ULPI_FUNCTION;
          phy_ctrl_valid    <= 1'b1;
        end
      end

    end

    STCHIRP_USB_RESET: begin
      usb2_highspeed  <= 1'b0;
      usb2_reset      <= 1'b1;

      if (utmi_linestate != LS_SE0) begin
        phyctrl_state     <= STCHIRP_FS_IDLE;
        usb2_reset        <= 1'b0;
      end else if (cfg_usb2_en && (phy_ctrl_valid == 0)) begin
        phy_ctrl_reg_data <= ULPI_PERPH_CHIRP;
        phy_ctrl_reg_addr <= REG_ULPI_FUNCTION;
        phy_ctrl_valid    <= 1'b1;
        phyctrl_state     <= STCHIRP_FS_CHIRP_K;

        T0_reset          <= 1'b1;
        T0_active         <= 1'b1;
      end
    end

    STCHIRP_FS_CHIRP_K: begin
      usb_out_chirp_valid <= 1'b1;
      if (T0_to_uch) begin
        usb_out_chirp_valid <= 1'b0;
        phyctrl_state       <= STCHIRP_FS_WAIT_K;
        T0_reset            <= 1'b1;
        T0_active           <= 1'b1;
        chirp_kj_cnt        <= 0;
      end
    end

    STCHIRP_FS_WAIT_K: begin
      if (T0_to_wtfs) begin
        phy_ctrl_reg_data <= ULPI_PERPH_FS;
        phy_ctrl_reg_addr <= REG_ULPI_FUNCTION;
        phy_ctrl_valid    <= 1'b1;
        phyctrl_state     <= STCHIRP_ROLLBACK_FS;
        T0_reset          <= 1'b1;
        T0_active         <= 1'b1;
      end

      if (utmi_linestate == LS_K) begin
        if (!T1_active) begin
          T1_reset          <= 1'b1;
          T1_active         <= 1'b1;
        end else if (T1_to_filt) begin
          T1_reset          <= 1'b1;
          T1_active         <= 1'b0;

          phyctrl_state     <= STCHIRP_FS_WAIT_J;
        end
      end else begin 
        T1_reset          <= 1'b1;
        T1_active         <= 1'b0;
      end
    end

    STCHIRP_FS_WAIT_J: begin
      if (T0_to_wtfs) begin
        phy_ctrl_reg_data <= ULPI_PERPH_FS;
        phy_ctrl_reg_addr <= REG_ULPI_FUNCTION;
        phy_ctrl_valid    <= 1'b1;
        phyctrl_state     <= STCHIRP_ROLLBACK_FS;
        T0_reset          <= 1'b1;
        T0_active         <= 1'b1;
      end

      if (utmi_linestate == LS_J) begin
        if (!T1_active) begin
          T1_reset          <= 1'b1;
          T1_active         <= 1'b1;
        end else if (T1_to_filt) begin
          T1_reset          <= 1'b1;
          T1_active         <= 1'b0;

          phyctrl_state     <= STCHIRP_FS_WAIT_K;
          chirp_kj_cnt      <= chirp_kj_cnt + 1'b1;

          if (chirp_kj_cnt == 2'd2) begin
            phy_ctrl_reg_data <= ULPI_PERPH_HS;
            phy_ctrl_reg_addr <= REG_ULPI_FUNCTION;
            phy_ctrl_valid    <= 1'b1;
            phyctrl_state     <= STCHIRP_HS_RESET;

            T0_reset      <= 1'b1;
            T0_active     <= 1'b0;
          end
        end
      end else begin 
        T1_reset          <= 1'b1;
        T1_active         <= 1'b0;
      end
    end

    STCHIRP_HS_RESET: begin
      usb2_highspeed  <= 1'b1;
      usb2_reset      <= 1'b1;

      if (utmi_linestate == LS_SQUELCH) begin
        usb2_reset        <= 1'b0;
        phyctrl_state     <= STCHIRP_HS_IDLE;
      end
    end


    STCHIRP_HS_IDLE: begin
      if (utmi_linestate == LS_SQUELCH) begin
        if (!T0_active) begin
          T0_reset          <= 1'b1;
          T0_active         <= 1'b1;
        end else if (
`ifdef SYM
               0 &&
`endif
                    T0_to_wtrev) begin
          T0_reset          <= 1'b1;
          T0_active         <= 1'b1;

          phy_ctrl_reg_data <= ULPI_PERPH_FS;
          phy_ctrl_reg_addr <= REG_ULPI_FUNCTION;
          phy_ctrl_valid    <= 1'b1;

          phyctrl_state     <= STCHIRP_HS_CHECK_SUSP;
        end
      end else begin
        T0_reset          <= 1'b1;
        T0_active         <= 1'b0;
      end
    end

    STCHIRP_HS_CHECK_SUSP: begin
      if (T0_to_wtrsths) begin
        T0_reset          <= 1'b1;
        T0_active         <= 1'b0;
      
        if (utmi_linestate == LS_SE0) begin
          phyctrl_state <= STCHIRP_USB_RESET;
        end else begin
          phyctrl_state <= STCHIRP_SUSPENDED;
          
          phy_ctrl_reg_data <= ULPI_PERPH_FS_RESUME;
          phy_ctrl_reg_addr <= REG_ULPI_FUNCTION;
          phy_ctrl_valid    <= 1'b1;
        end
      end
    end

    STCHIRP_ROLLBACK_FS: begin
      if (T0_to_wtrstfs) begin
        T1_reset          <= 1'b1;
        T1_active         <= 1'b0;
        T0_reset          <= 1'b1;
        T0_active         <= 1'b0;

        phyctrl_state     <= STCHIRP_RBRESET_FS;
      end
    end

    STCHIRP_SUSPENDED: begin

      if (utmi_linestate == LS_SE0) begin
        if (!T0_active) begin
          T0_reset          <= 1'b1;
          T0_active         <= 1'b1;
        end else if (T0_to_wtrstfs) begin
          T0_reset          <= 1'b1;
          T0_active         <= 1'b0;

          phyctrl_state     <= STCHIRP_USB_RESET;
        end
      end else begin
        T0_reset          <= 1'b1;
        T0_active         <= 1'b0;

        if (utmi_linestate == LS_K) begin
          phyctrl_state     <= STCHIRP_RESUME;
        end
      end
    end

    STCHIRP_RESUME: begin
      if (utmi_linestate == LS_SE0) begin
          phyctrl_state     <= (usb2_highspeed) ? STCHIRP_HS_IDLE : STCHIRP_FS_IDLE;

          phy_ctrl_reg_data <= (usb2_highspeed) ? ULPI_PERPH_HS : ULPI_PERPH_FS;
          phy_ctrl_reg_addr <= REG_ULPI_FUNCTION;
          phy_ctrl_valid    <= 1'b1;
      end
    end

    STCHIRP_RBRESET_FS: begin
      if (utmi_linestate == LS_J) begin
        usb2_reset        <= 1'b0;
        usb2_highspeed    <= 1'b0;
        phyctrl_state     <= STCHIRP_FS_IDLE;
      end
    end

    endcase
  end
end


assign stat_usb_hs         = (phyctrl_state == STCHIRP_HS_IDLE);
assign stat_phy_ready      = (phyctrl_state == STCHIRP_HS_IDLE || phyctrl_state == STCHIRP_FS_IDLE);
assign stat_usb_reset      = usb2_reset;

// The PHY asserts NXT to throttle the data. When the
// Link is sending data to the PHY, NXT indicates when
// the current byte has been accepted by the PHY

// Controls the direction of the data bus. When the PHY
// has data to transfer to the Link, it drives DIR
// high to take ownership of the  bus. When the PHY has no
// data to transfer it drives DIR low and monitors the
// bus for commands from the Link.

// The Link asserts STP for one clock cycle to stop the
// data  stream  currently on the bus. If the Link is
// sending data to the PHY, STP indicates the last byte
// of data was on the bus in the previous cycle


localparam CMD_H_IDLE     = 2'b00;
localparam CMD_H_TRANSMIT = 2'b01;
localparam CMD_H_REG_WR   = 2'b10;
localparam CMD_H_REG_RD   = 2'b11;

localparam CMD_L_NXT_ADDR = 6'b101_111;

reg [2:0] state;
localparam [2:0] 
    ST_RESET         = 0,
    ST_IDLE          = 1,
    ST_RX_CMD        = 2,
    ST_TX_DATA_CRC   = 3,
    ST_TX_DATA       = 4,
    ST_TX_CMD_REG_WR = 5,
    ST_TX_DATA_CHIRP = 6,
    ST_TX_STOP       = 7;

reg [1:0] dt_state;
localparam [1:0] 
    DT_NONDATA = 0,
    DT_DATA    = 1,
    DT_CRC16_0 = 2,
    DT_CRC16_1 = 3;

reg    usb_out_flush_active;
assign usb_out_ready = usb_out_flush_active || ((stat_phy_ready) && (state == ST_IDLE || (state == ST_TX_DATA) && (phy_nxt)) && ~phy_dir);

reg       utmi_rxactive;
reg       utmi_rxerror;
reg       utmi_hostdisconnect;
reg       utmi_changed;
reg       alt_int;

assign host_disconnected = utmi_hostdisconnect;

reg       reg_ctrl_wr_active;
reg       phy_usb_out_txerror;

// To force a Hi-Speed transmit error, the Link must transmit an inverted CRC at the end of the packet.
//  The Link considers a packet to be completed when the RX CMD byte shows RxActive is set to 0b, or dir is deasserted, whichever occurs first. 
// Register read neeeds proper abortion handling, removing it for now

reg [7:0]  usb_in_data_p;
reg        usb_in_valid_p;
wire [7:0] usb_in_data_np  = phy_di;
wire       usb_in_valid_np = phy_dir && phy_nxt && (state == ST_RX_CMD);

localparam IN_DATA_PIPELINED = 1;
wire [7:0] usb_in_data   = (IN_DATA_PIPELINED) ? usb_in_data_p  : usb_in_data_np;
wire       usb_in_valid  = (IN_DATA_PIPELINED) ? usb_in_valid_p : usb_in_valid_np;

`include "usb_enums.vh"

// CRC16 checker for DATA
reg  [15:0] out_crc16_data;
wire [15:0] out_crc16_next;
usb_crc16_8_invr usb_crc16_8_invr_out(
  .d( usb_out_data ),
  .crci(out_crc16_data),
  .crco(out_crc16_next),
  .ok());

always @(posedge phy_clk) begin
  if (reset) begin
    phy_do               <= { CMD_H_IDLE, 6'b000_000 };
    phy_stp              <= 1'b1;
    state                <= ST_RESET;

    // State registers
    utmi_rxactive        <= 0;
    utmi_rxerror         <= 0;
    utmi_hostdisconnect  <= 1;
    alt_int              <= 0;
    utmi_linestate       <= 2'b11;
    utmi_changed         <= 0;

    // Output data
    usb_in_valid_p        <= 0;
    phy_ctrl_ready       <= 0;
    usb_out_flush_active <= 0;

  end else begin
    utmi_changed   <= 0;
    usb_in_valid_p <= 1'b0;
    phy_stp        <= 1'b0;

    if (phy_ctrl_ready && phy_ctrl_valid) begin
      phy_ctrl_ready <= 1'b0;
    end

    if (usb_out_valid && usb_out_last) begin
      usb_out_flush_active <= 0;
    end

    case (state)
    ST_RESET: begin
      // Wait for PHY to be ready
      phy_stp     <= 1'b1;
      if (~phy_dir) begin
        state     <= ST_IDLE;
        phy_stp   <= 1'b0;
      end
    end

    ST_IDLE: begin
      phy_do         <= { CMD_H_IDLE, 6'b000_000 };

      phy_usb_out_txerror <= 0;
      out_crc16_data      <= 16'h0000;
      dt_state            <= DT_NONDATA;

      // Idle state we can start transmition or rx can idicate input packet
      if (phy_dir) begin
        // DIR asserted -- turnaround cycle
        // Indication of active RX event (data transfer)
        utmi_rxactive <= phy_nxt;
        state         <= ST_RX_CMD;
      end else if (~phy_stp) begin
        // Data access
        // Register access
        if (phy_ctrl_valid && ~phy_ctrl_ready) begin
          phy_do[7:6] <= CMD_H_REG_WR;
          phy_do[5:0] <= phy_ctrl_reg_addr;
          state       <= ST_TX_CMD_REG_WR;
        end else if (usb_out_valid && !usb_out_flush_active || usb_out_chirp_valid) begin
          // LPM TOKEN TRANSMIT in usb_out_chirp_valid mode
          phy_do[7:6] <= CMD_H_TRANSMIT;
          phy_do[5:0] <= (usb_out_chirp_valid) ? 6'h00 : usb_out_data[5:0];

          if (usb_out_chirp_valid) begin
            state <= ST_TX_DATA_CHIRP;
          end else if (usb_out_data[3:0] == R_PID_DATA0 ||
                       usb_out_data[3:0] == R_PID_DATA1 ||
                       usb_out_data[3:0] == R_PID_DATA2 ||
                       usb_out_data[3:0] == R_PID_MDATA) begin
            state    <= (usb_out_last) ? ST_TX_DATA_CRC : ST_TX_DATA;
            dt_state <= DT_DATA;
          end else begin
            state    <= (usb_out_last) ? ST_TX_STOP : ST_TX_DATA;
          end

        end
      end
    end

    ST_TX_CMD_REG_WR, ST_TX_STOP, ST_TX_DATA_CHIRP, ST_RX_CMD, ST_TX_DATA, ST_TX_DATA_CRC: begin
      if (phy_dir) begin
        if (state == ST_TX_DATA) begin
          // Bus turnover happend in active trnsmition, drop the transaction
          usb_out_flush_active <= 1'b1;
        end
        if (~phy_nxt) begin
          // NXT == 0 means RXCMD on the bus

          alt_int <= phy_di[7];
          // phy_di[6] -- 0 - A DEVICE; 1 - B DEVICE
          case (phy_di[5:4])
          2'b00: begin
            utmi_rxactive       <= 1'b0;
            utmi_rxerror        <= 1'b0;
            utmi_hostdisconnect <= 1'b0;
          end
          2'b01: begin
            utmi_rxactive       <= 1'b1;
            utmi_rxerror        <= 1'b0;
            utmi_hostdisconnect <= 1'b0;
          end
          2'b11: begin
            utmi_rxactive       <= 1'b1;
            utmi_rxerror        <= 1'b1;
            utmi_hostdisconnect <= 1'b0;
          end
          2'b10: begin
            utmi_rxactive       <= 1'b0; // X by spec, set 0 to ease
            utmi_rxerror        <= 1'b0; // X by spec, set 0 to ease
            utmi_hostdisconnect <= 1'b1;
          end
          endcase
          // phy_di[3:2] VBUS encode
          utmi_linestate <=  phy_di[1:0];
          utmi_changed   <=  1'b1;
        end else begin
          // DATA PHASE
          if (state == ST_RX_CMD) begin
            usb_in_valid_p   <= 1'b1;
            usb_in_data_p    <= phy_di;
          end else begin
            // Data abortion, turnaround cycle
            // Register write will restart right after the RX
            state         <= ST_RX_CMD;
            utmi_rxactive <= 1'b1;
          end
        end
      end else begin
        if (state == ST_RX_CMD) begin
          // DIR deasserted -- turnaround cycle
          state         <= ST_IDLE;
          utmi_rxactive <= 1'b0;
          // mark packet LAST, turnaround kills the stream
        end else begin
          // register access accepted
          if (phy_nxt) begin
            if (state == ST_TX_CMD_REG_WR) begin
              phy_do         <= phy_ctrl_reg_data;
              phy_ctrl_ready <= 1'b1;
              state          <= ST_TX_STOP;
            end /*else if (state == ST_TX_CMD_REG_RD) begin
                phy_do <= { CMD_H_IDLE, 6'b000_000 };
                state  <= ST_REG_RD_T;
            end*/ else if (state == ST_TX_DATA_CHIRP) begin
              phy_do  <= 8'h00;
              if (!usb_out_chirp_valid) begin
                  phy_stp <= 1'b1;
                  state   <= ST_IDLE;
              end
            end else if (state == ST_TX_STOP) begin
              //Bit stuffing only if Low/Full speed
              if (phy_usb_out_txerror && !stat_usb_hs) begin
                phy_do  <= 8'hff;
              end else begin
                phy_do  <= 8'h00;
              end
              //phy_ctrl_ready <= 1'b1;
              phy_stp        <= 1'b1;
              state          <= ST_IDLE;
            end else if (state == ST_TX_DATA_CRC) begin
              dt_state <= dt_state + 1'b1;

              if (dt_state == DT_DATA) begin
                phy_do <= (phy_usb_out_txerror) ? ~out_crc16_data[7:0 ] : out_crc16_data[7:0];
                state  <= ST_TX_DATA_CRC;
              end else begin
                phy_do <= (phy_usb_out_txerror) ? ~out_crc16_data[15:8] : out_crc16_data[15:8];
                state  <= ST_TX_STOP;
              end
            end else begin
              // Data transfer
              if (usb_out_valid) begin 
                out_crc16_data      <= out_crc16_next;
                phy_do              <= usb_out_data;
                phy_usb_out_txerror <= usb_out_txerror;
                if (usb_out_last) begin
                  if ((dt_state == DT_DATA) && (stat_usb_hs || ~usb_out_txerror)) begin
                    state               <= ST_TX_DATA_CRC;
                  end else begin
                    state               <= ST_TX_STOP;
                  end
                end
              end else begin
                dt_state             <= dt_state + 1'b1;
                phy_usb_out_txerror  <= 1'b1;
                usb_out_flush_active <= 1'b1; 

                //Data underrun, end packet with error
                if ((dt_state == DT_DATA) && (stat_usb_hs)) begin
                  phy_do  <= ~out_crc16_data[7:0];
                  state   <= ST_TX_DATA_CRC;
                end else begin
                  phy_do  <= (stat_usb_hs) ? 8'h00 : 8'hff;  //Bit stuffing only if Low/Full speed
                  phy_stp <= 1'b1;
                  state   <= ST_IDLE;
                end
              end
            end
          end
        end
      end
    end

    endcase

  end
end


wire [7:0] usb_in_c0; // Curr
wire [7:0] usb_in_c1; // z^-1
wire [7:0] usb_in_c2; // z^-2
wire [7:0] usb_in_c3; // z^-3

// CRC5 checker for tokens IN, OUT, SETUP
wire [10:0] crc5_din = {usb_in_c0[2:0], usb_in_c1};
wire [4:0]  crc5;
wire        crc5_ok = (crc5 == usb_in_c0[7:3]);
usb_crc5_11_inv usb_crc5_11_inv(.d(crc5_din), .crco(crc5));

// CRC16 checker for DATA
reg  [15:0] crc16_data;
wire        crc16_ok_o;
reg         crc16_nok;
wire [15:0] crc16_next;
usb_crc16_8_invr usb_crc16_8_invr(
  .d( usb_in_c0 ),
  .crci(crc16_data),
  .crco(crc16_next),
  .ok(crc16_ok_o));

reg [7:0] usb_in_data_1;
reg [7:0] usb_in_data_2;
reg [7:0] usb_in_data_3;

assign usb_in_c0 = usb_in_data;
assign usb_in_c1 = usb_in_data_1;
assign usb_in_c2 = usb_in_data_2;
assign usb_in_c3 = usb_in_data_3;



reg [1:0] bytepos;

reg       utmi_rxactive_prev;
reg       utmi_rxerror_prev;

// utmi_rxactive, utmi_rxerror to AXI-S
// usb_in_valid , usb_in_data

// Packet with DATA
//                      CRC16    rxactive -> 0
// PID  B0  B1  B2  B3  B4  B5   \
//  x    0   1   2   3   3   3   3
//              opd  b0  b1  b2  b3

// Zero packet
//      CRC16   rxactive -> 0
// PID  B0  B1  \
//  x    0   1   2   
//              opd  


reg [2:0]  rx_ps;
localparam [2:0]
    PS_PID = 0,
    PS_DATA = 1,
    PS_TOKEN_B0 = 2,
    PS_TOKEN_B1 = 3,
    PS_SOF_B0 = 4,
    PS_SOF_B1 = 5,
    PS_ERROR = 6,
    PS_NONE = 7;

reg valid_transaction;

always @(posedge phy_clk) begin
  if (reset) begin
    axis_usb_rx_valid  <= 1'b0;
    utmi_rxactive_prev <= 1'b0;
    utmi_rxerror_prev  <= 1'b0;
    rx_ps              <= PS_NONE;
    bytepos            <= 2'b00;
    valid_transaction  <= 1'b0;

  end else begin
    axis_usb_rx_valid  <= 1'b0;

    utmi_rxactive_prev <= utmi_rxactive;
    utmi_rxerror_prev  <= utmi_rxerror;

    if (utmi_rxactive_prev == 1'b0 && utmi_rxactive == 1'b1) begin
      //Start of pcket
      rx_ps <= PS_PID;
    end

    // Changing utmi_rxactive and usb_in_valid == 1'b1 can't be in the same cycle
    if (usb_in_valid) begin
      usb_in_data_1 <= usb_in_data;
      usb_in_data_2 <= usb_in_data_1;
      usb_in_data_3 <= usb_in_data_2;

      if (bytepos != 2'b11)
        bytepos <= bytepos + 1'b1;

      // Packet processor
      case (rx_ps)
      PS_PID: begin
        bytepos    <= 0;
        crc16_data <= 16'h0000;
        crc16_nok  <= 1'b1;

        if (~usb_in_c0[7:4] != usb_in_c0[3:0]) begin
          rx_ps             <= PS_ERROR;
          valid_transaction <= 1'b0; //Ignore DATA and handshake tokens not for us
        end else begin
          case (usb_in_c0[3:0])
          R_PID_ACK, R_PID_NAK, R_PID_STALL, R_PID_NYET: begin
            rx_ps             <= PS_NONE;

            axis_usb_rx_data  <= {4'h0, usb_in_c0[3:0]};
            axis_usb_rx_valid <= valid_transaction;
            axis_usb_rx_last  <= 1'b1;
            axis_usb_rx_error <= 1'b0;
          end

          R_PID_DATA0, R_PID_DATA1, R_PID_DATA2, R_PID_MDATA: begin
            if (valid_transaction) begin
              rx_ps <= PS_DATA;
            end else begin
              rx_ps <= PS_NONE;
            end
          end

          R_PID_OUT, R_PID_IN, R_PID_SETUP, R_PID_PING: begin
            rx_ps <= PS_TOKEN_B0;
          end

          R_PID_SOF: begin
            rx_ps <= PS_SOF_B0;
          end

          // ERR, SPLIT
          default: begin
            rx_ps             <= PS_ERROR;
            valid_transaction <= 1'b0;
            //Wait till the end of packet
          end
          endcase
        end
      end


      PS_DATA: begin
        crc16_data <= crc16_next;
        crc16_nok  <= ~crc16_ok_o;

        if (bytepos >= 2'b10) begin
          axis_usb_rx_data  <= {(bytepos == 2'b10) ? 4'h0 : usb_in_c3[7:4], usb_in_c3[3:0]};
          axis_usb_rx_valid <= 1'b1;
          axis_usb_rx_last  <= 1'b0;
          axis_usb_rx_error <= 1'b0;
        end
      end

      PS_TOKEN_B0: begin
        rx_ps <= PS_TOKEN_B1;
      end

      PS_TOKEN_B1: begin
        rx_ps <= PS_NONE;

        if (crc5_ok) begin
          axis_usb_rx_data  <= {usb_in_c0[2:0], usb_in_c1[7], usb_in_c2[3:0]}; // USB_EP | PID
          axis_usb_rx_last  <= 1'b1;
          axis_usb_rx_error <= 1'b0;
          axis_usb_rx_valid <= NO_ADDR_CHECK || (usb_in_c1[6:0] == cfg_usb_addr);
          valid_transaction <= NO_ADDR_CHECK || (usb_in_c1[6:0] == cfg_usb_addr);
        end else begin
          valid_transaction <= 1'b0;
        end
      end

      PS_SOF_B0: begin
        rx_ps <= PS_SOF_B1;
      end

      PS_SOF_B1: begin
        rx_ps <= PS_NONE;
      end

      PS_ERROR: begin
        // Wait till the end of packet
      end

      PS_NONE: begin
        //Does not expect data, just wait for reset
      end

      endcase
    end

    if (utmi_rxactive_prev == 1'b1 && utmi_rxactive == 1'b0) begin
      //Packet terminted

      if (rx_ps == PS_DATA) begin
        if (bytepos >= 2'b10) begin
          axis_usb_rx_data  <= {(bytepos == 2'b10) ? 4'h0 : usb_in_c3[7:4], usb_in_c3[3:0]};
          axis_usb_rx_valid <= 1'b1; 
          axis_usb_rx_last  <= 1'b1;
          axis_usb_rx_error <= utmi_rxerror_prev | crc16_nok;
        end
      end

      rx_ps <= PS_NONE;
    end
  end
end

// 3, 3, 4
assign debug_state = { rx_ps, state, phyctrl_state };
endmodule
