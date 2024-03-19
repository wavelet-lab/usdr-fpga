// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
// Based on 3 ASYNC signals we reconstruct burst cnfiguration
//
module bsig_to_burstbuffer #(
  parameter MAX_BUSRTS_BITS    = 5,
  parameter MAX_BUSRTS         = 1 << MAX_BUSRTS_BITS, // Maximum number of bursts in a buffer
  parameter EN_LOWWMRK         = 1,
  parameter BUFFER_SIZE_BITS   = 16,
  parameter MAX_BUFF_SKIP_BITS = 24,
  parameter DATA_BITS          = 4       // Minimum transfer burst 3 - 64bits; 4 - 128bits; 5 - 256bits
)(
    input                                clk,
    input                                dma_en,

    // cfg
    input [1:0]                          cfg_max_payload_sz, // 128b / 256b / 512b / 1024b
    input                                cfg_dis_lowwmrk,
    input [MAX_BUSRTS_BITS-1:0]          cfg_buff_brst_z,
    input [BUFFER_SIZE_BITS-1:DATA_BITS] cfg_brst_words_z,

    // BUSRT pulses
    input                                fifo_burst_skip,
    input                                fifo_burst_fill,
    input                                fifo_burst_mlowmrk, //Signaled when burst filled with cfg_max_payload_sz

    output                               tbuffer_valid,     // Buffer update stats
    output                               tbuffer_last,      // Last update for this buffer
    output [BUFFER_SIZE_BITS:DATA_BITS]  tbuffer_data,      // Number of bytes filled in current buffer
    
    output                                     tburst_valid,     // 
    output                                     tburst_last,      // Last burst in complete buffer
    output [MAX_BUFF_SKIP_BITS+MAX_BUSRTS-1:0] tburst_data       // fillstatus & skipped count
);

// Signal generation
reg fifo_burst_skip_p;
reg fifo_burst_fill_p;
reg signal_fifo_burst_mlowmrk_p;

wire signal_fifo_burst_mlowmrk = EN_LOWWMRK && ~cfg_dis_lowwmrk && (signal_fifo_burst_mlowmrk_p != fifo_burst_mlowmrk);
wire signal_skip               = fifo_burst_skip_p != fifo_burst_skip;
wire signal_fill               = fifo_burst_fill_p != fifo_burst_fill;
wire signal_nxt_brst           = signal_skip | signal_fill;

always @(posedge clk) begin
  begin
    fifo_burst_skip_p           <= fifo_burst_skip;
    fifo_burst_fill_p           <= fifo_burst_fill;
    signal_fifo_burst_mlowmrk_p <= fifo_burst_mlowmrk;
  end
end

// Remaining number of bursts to finish the buffer
reg [MAX_BUSRTS_BITS-1:0]         rem_buffers_z;
wire [MAX_BUSRTS_BITS:0]          nxt_rem_buffers_z    = { 1'b0, rem_buffers_z } - 1'b1;
wire                              last_burst_in_buff   = nxt_rem_buffers_z[MAX_BUSRTS_BITS];

reg [BUFFER_SIZE_BITS:DATA_BITS]  cur_buffer_filled_z;  // Number of bytes filled in current buffer
reg [BUFFER_SIZE_BITS:DATA_BITS]  cur_burst_rem_z;      // Remaining bytes to fill in current BURST

reg                               zbufn;
wire                              non_zero_buffer      = zbufn | signal_fill;
reg                               completebfn;
wire                              full_complete_buffer = completebfn && ~signal_skip;
wire                              whole_buffer_filled  = signal_nxt_brst && last_burst_in_buff && non_zero_buffer;

wire [9:7]                        mlowmrk_size_ext_z   = (cfg_max_payload_sz == 2'b00) ? 3'b000 :
                                                         (cfg_max_payload_sz == 2'b01) ? 3'b001 : 
                                                         (cfg_max_payload_sz == 2'b10) ? 3'b011 : 3'b111;
wire [9:DATA_BITS]                mlowmrk_size_z       = { mlowmrk_size_ext_z, {(7 - DATA_BITS){1'b1}} };

wire [BUFFER_SIZE_BITS:DATA_BITS] nxt_cur_buffer_filled_z  = cur_buffer_filled_z + cur_burst_rem_z + 1'b1;
wire [BUFFER_SIZE_BITS:DATA_BITS] nxt_cur_buffer_mlowmrk_z = cur_buffer_filled_z + mlowmrk_size_z + 1'b1;

reg [MAX_BUSRTS-1:0]              buff_status;
reg [MAX_BUFF_SKIP_BITS-1:0]      buff_skiped;

wire [MAX_BUSRTS-1:0]             nxt_buff_status = { signal_fill, buff_status[MAX_BUSRTS-1:1] };

always @(posedge clk) begin
  if (~dma_en) begin
    cur_buffer_filled_z <= ~0;
    cur_burst_rem_z     <= {1'b0, cfg_brst_words_z};
    zbufn               <= 0;
    completebfn         <= 1'b1;
    buff_skiped         <= 0;
    buff_status         <= 0;
    rem_buffers_z       <= cfg_buff_brst_z;

  end else begin
    if (signal_fifo_burst_mlowmrk) begin
      cur_burst_rem_z     <= cur_burst_rem_z - mlowmrk_size_z - 1'b1;
      cur_buffer_filled_z <= nxt_cur_buffer_mlowmrk_z;
    end

    if (signal_nxt_brst) begin
      if (signal_fill) begin
        cur_buffer_filled_z <= nxt_cur_buffer_filled_z;
        cur_burst_rem_z     <= {1'b0, cfg_brst_words_z};
      end

      if (last_burst_in_buff) begin
        buff_status       <= 0;
        zbufn             <= 0;
        completebfn       <= 1'b1;
        rem_buffers_z     <= cfg_buff_brst_z;
        
        cur_buffer_filled_z <= ~0;

        if (non_zero_buffer) begin
          buff_skiped <= 0;
        end else begin
          buff_skiped <= buff_skiped + 1'b1;
        end
      end else begin
        zbufn             <= non_zero_buffer;
        completebfn       <= full_complete_buffer;
        buff_status       <= nxt_buff_status;
        rem_buffers_z     <= nxt_rem_buffers_z;
      end
    end
  end
end

assign tbuffer_valid = whole_buffer_filled || signal_fill || signal_fifo_burst_mlowmrk;
assign tbuffer_last  = whole_buffer_filled;
assign tbuffer_data  = (signal_fill)               ? nxt_cur_buffer_filled_z : 
                       (signal_fifo_burst_mlowmrk) ? nxt_cur_buffer_mlowmrk_z : cur_buffer_filled_z;

assign tburst_valid  = last_burst_in_buff || signal_nxt_brst;
assign tburst_last   = full_complete_buffer;
assign tburst_data   = { buff_skiped, nxt_buff_status };


endmodule
