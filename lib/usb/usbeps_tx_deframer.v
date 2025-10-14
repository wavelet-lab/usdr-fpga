module usbeps_tx_deframer #(
    parameter DATA_BITS           = 3,
    parameter _DATA_BYTES         = 1 << DATA_BITS,
    parameter TX_TIMESTAMP_BITS   = 49,
    parameter TX_RAM_ADDR_WIDTH   = 17,
    parameter TX_SAMPLES_WIDTH    = TX_RAM_ADDR_WIDTH - 1,
    parameter TX_FE_DESCR_WIDTH   = TX_TIMESTAMP_BITS + TX_SAMPLES_WIDTH + (TX_RAM_ADDR_WIDTH - DATA_BITS),
    parameter RAM_CHECK_BIT       = 8
) (
    input clk,
    input rst,

    output reg                        s_axis_endpoint_tx_busy, //(busy flag -- do not try to push data)
    input                             s_axis_endpoint_tx_valid,
    output                            s_axis_endpoint_tx_ready,
    input                             s_axis_endpoint_tx_last,
    input [32-1:0]                    s_axis_endpoint_tx_data,
    input [4-1:0]                     s_axis_endpoint_tx_keep,

    output reg                             mem_tvalid,
    input                                  mem_tready,
    output     [TX_RAM_ADDR_WIDTH-1:DATA_BITS] mem_taddr,
    output reg [_DATA_BYTES*8-1:0]         mem_tdata,
    output     [_DATA_BYTES-1:0]           mem_tkeep,

    output [TX_FE_DESCR_WIDTH-1:0]         usbs_burst_data,
    output reg                             usbs_burst_valid,
    input                                  usbs_burst_ready,
    input                                  usbs_burst_busy,

    input [TX_RAM_ADDR_WIDTH:RAM_CHECK_BIT] out_rd_addr,
    input [TX_TIMESTAMP_BITS-1:0]           out_rd_time,

    output [31:0]                          stat_drop_tsd_last,
    output [63:0]                          stat,
    output                                 fifo_full,
    input                                  fetx_mode_format
);

/*
16 bytes packet header (OLD format)

DW0: [31:0]  timestamp[31:0]
DW1: [31]    discad_timestamp_flag
     [30:16] samples_in_packet       -- up to 32k samples
     [15:0]  timestamp[47:32]
DW2: 00000000
DW3: 00000000

DW4-N IQ 16 bit


DW2:
 [0]     = 0 -> 16 bit IQ; 1 -> 12 bit IQ
 [1]     = 0 -> SISO;      1 -> MIMO
 [15:2]  = bytes


128k addres by 64
[17:3]

- 63                            0 -
-----------------------------------
-  bq   -   bi   -  aq   -   ai   -
-----------------------------------s
*/

assign mem_tkeep = {_DATA_BYTES{1'b1}};

reg [TX_RAM_ADDR_WIDTH:DATA_BITS]  bytes;

reg [14:0]                         samples;      // individual samples
reg [14:0]                         wrpktsamples; //

reg [47:0]                         timestamp;
reg                                nots;


reg [TX_RAM_ADDR_WIDTH:DATA_BITS]  ramadd;
assign                             mem_taddr = ramadd[TX_RAM_ADDR_WIDTH-1:DATA_BITS];

localparam [2:0]
    ST_IDLE       = 0,
    ST_HEADER_DW1 = 1,
    ST_HEADER_DW2 = 2,
    ST_HEADER_DW3 = 3,
    ST_PACKET_L   = 4,
    ST_PACKET_H   = 5,
    ST_PACKET_QL  = 6,
    ST_PACKET_QH  = 7;

reg [2:0] state;

localparam FE_BYTES_OFF   = 0;
localparam FE_SAMPLES_OFF = FE_BYTES_OFF    + (TX_RAM_ADDR_WIDTH - DATA_BITS);
localparam FE_TS_OFF      = FE_SAMPLES_OFF  + TX_SAMPLES_WIDTH;

assign usbs_burst_data[FE_SAMPLES_OFF - 1:FE_BYTES_OFF]             = bytes;
assign usbs_burst_data[FE_TS_OFF-1:FE_SAMPLES_OFF]                  = samples;
assign usbs_burst_data[FE_TS_OFF + TX_TIMESTAMP_BITS - 2:FE_TS_OFF] = timestamp[TX_TIMESTAMP_BITS - 2:0];
assign usbs_burst_data[FE_TS_OFF + TX_TIMESTAMP_BITS - 1]           = nots;

assign s_axis_endpoint_tx_ready =
    (state == ST_IDLE)     ? (!usbs_burst_valid || usbs_burst_ready) :
    (state == ST_PACKET_L) ? (!mem_tvalid || mem_tready) : 1'b1;

//512b is a minimum FIFO space to clear busy flag
localparam EP_SIZE = 1 + (512 / (1 << RAM_CHECK_BIT)); // 128 * 4
wire [TX_RAM_ADDR_WIDTH:RAM_CHECK_BIT] fifo_remaining =
    {(TX_RAM_ADDR_WIDTH - RAM_CHECK_BIT){1'b1}} - EP_SIZE + out_rd_addr - ramadd[TX_RAM_ADDR_WIDTH:RAM_CHECK_BIT] - 1'b1;
assign fifo_full = fifo_remaining[TX_RAM_ADDR_WIDTH];

wire [TX_TIMESTAMP_BITS:0] packet_late_bits = timestamp - out_rd_time;
wire                       packet_late      = packet_late_bits[TX_TIMESTAMP_BITS];
reg              drop_packet;
reg [15:0]       dropped;
reg [31:0]       drop_late;
reg [15:0]       pkt_pshd;

assign stat_drop_tsd_last = drop_late;
assign stat[15:0]   = { fifo_remaining[TX_RAM_ADDR_WIDTH:8], fetx_mode_format, usbs_burst_busy,     drop_packet, state };
assign stat[31:16]  = dropped;
assign stat[47:32]  = pkt_pshd;
assign stat[62:48]  = 0;
assign stat[63]     = fifo_full;

// s_axis_endpoint_tx_last indicates Endpoint transfers. Actual data ranges can be anything, especially when mutilple
// blocks combined together

wire samples_strobe = !fetx_mode_format || state == ST_PACKET_H || state == ST_PACKET_QH;

always @(posedge clk) begin
    if (rst) begin
        state                   <= ST_IDLE;
        mem_tvalid              <= 1'b0;
        ramadd                  <= 0;
        usbs_burst_valid        <= 1'b0;
        s_axis_endpoint_tx_busy <= 1'b0; // Clear FIFO in reset
        dropped                 <= 0;
        drop_packet             <= 1'b0;
        drop_late               <= 0;
        bytes                   <= 0;
        pkt_pshd                <= 0;
    end else begin

        if (mem_tvalid && mem_tready) begin
            mem_tvalid <= 1'b0;
            ramadd     <= ramadd + 1'b1;
        end

        if (usbs_burst_valid && usbs_burst_ready) begin
            usbs_burst_valid <= 1'b0;
        end

        if (mem_tvalid) begin
            s_axis_endpoint_tx_busy <= 1'b1;
        end else begin
            s_axis_endpoint_tx_busy <= /* (mode_repeat) ? 1'b0 : */ (usbs_burst_busy || fifo_full);
        end

        if (s_axis_endpoint_tx_valid && s_axis_endpoint_tx_ready) begin
            case (state)

            ST_IDLE: begin
                wrpktsamples    <= 0;
                bytes           <= ~0;

                timestamp[31:0] <= s_axis_endpoint_tx_data;

                state           <= state + 1'b1;
            end

            ST_HEADER_DW1: begin
                nots             <= s_axis_endpoint_tx_data[31];
                samples[14:0]    <= s_axis_endpoint_tx_data[30:16];
                timestamp[47:32] <= s_axis_endpoint_tx_data[15:0];

                state            <= state + 1'b1;
            end

            ST_HEADER_DW2: begin
                //samples[16:15]   <= s_axis_endpoint_tx_data[1:0];
                state            <= state + 1'b1;
            end

            ST_HEADER_DW3: begin
                state            <= state + 1'b1;

                if ((packet_late /* || sig_realign*/) && !nots) begin
                    dropped          <= dropped + 1'b1;
                    drop_packet      <= 1'b1;
                    drop_late        <= packet_late_bits;
                end else begin
                    drop_packet      <= 1'b0;
                end
            end

            ST_PACKET_L: mem_tdata[31:0]  <= s_axis_endpoint_tx_data;
            ST_PACKET_H: mem_tdata[63:32] <= s_axis_endpoint_tx_data;
            ST_PACKET_QL: if (DATA_BITS > 3) mem_tdata[95:64]  <= s_axis_endpoint_tx_data;
            ST_PACKET_QH: if (DATA_BITS > 3) mem_tdata[127:96] <= s_axis_endpoint_tx_data;

            endcase

            if (state == ST_PACKET_L || state == ST_PACKET_H ||
                state == ST_PACKET_QL || state == ST_PACKET_QH) begin

                mem_tvalid       <= 1'b0;
                if (samples_strobe) begin
                    wrpktsamples <= wrpktsamples + 1'b1;
                end
                state            <= state + 1'b1;

                s_axis_endpoint_tx_busy <= 1'b1;

                if (state == ST_PACKET_L) begin
                    bytes      <= bytes + 1'b1;
                end

                if (state == (DATA_BITS == 3 ? ST_PACKET_H : ST_PACKET_QH)) begin
                    state            <= ST_PACKET_L;
                    mem_tvalid       <= !drop_packet;
                end

                if (samples == wrpktsamples && samples_strobe) begin
                    mem_tvalid       <= !drop_packet;

                    state            <= ST_IDLE;
                    usbs_burst_valid <= !drop_packet;
                    if (!drop_packet) begin
                        pkt_pshd <= pkt_pshd + 1'b1;
                    end
                end
            end
        end

    end
end


endmodule

