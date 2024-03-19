// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module event_router #(
    parameter COUNT_BITS = 4,
    parameter OCOUNT_BITS = COUNT_BITS,
    parameter USER_BITS = 4,
    parameter DISP_COUNT = 2,
    parameter DISP_COUNT_BITS = 1,
    parameter PIPELINE = 1,
    parameter ULTRA_SCALE = 0
)(
    input clk,
    input rst,

    input [31:0]                          s_cer_data,
    input                                 s_cer_valid,
    output                                s_cer_ready,

    // Event source
    input [COUNT_BITS - 1:0]              s_evno_data,
    input                                 s_evno_valid,
    output                                s_evno_ready,

    // Disptch 
    output [DISP_COUNT * COUNT_BITS - 1:0]                mn_evno_data,
    output [DISP_COUNT * (OCOUNT_BITS + USER_BITS) - 1:0] mn_evno_user,
    output [DISP_COUNT - 1:0]                             mn_evno_valid,
    input [DISP_COUNT - 1:0]                              mn_evno_ready
);

// event => user:{orig_event, user_data} data:{mmaped_event}
//
// Confguration word 
// [7:0]    event_number
// [31:28]  0 - Update routing informaton 
//          1 - Clear status register
//
// 0: [15:8]  dispatch number   (up to 256 events)
//    [19:16] dispatch bus      (up to 15 buses), 4'b1111 means write to status register insted of sending out
//    [25:20] dispatch user     => mmaped to user

wire [4:0] wr_evno_addr = s_cer_data;
wire [7:0] dispatch_event;
wire [3:0] dispatch_bus;
wire [5:0] dispatch_user;
wire [4:0] rd_evno_addr = s_evno_data;

assign s_cer_ready = 1'b1;
ram_sxp #(.DATA_WIDTH(18), .ADDR_WIDTH(5), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b1)) dmacfg_w (
    .wclk(clk),
    .we(s_cer_valid && s_cer_ready),
    .waddr(wr_evno_addr),
    .wdata(s_cer_data[25:8]),
    .raddr(rd_evno_addr),
    .rdata({dispatch_user, dispatch_bus, dispatch_event})
);

wire [DISP_COUNT - 1:0] disp_ready;
assign s_evno_ready = (dispatch_bus == 4'b1111) || disp_ready[dispatch_bus];

genvar i;
generate
for (i = 0; i < DISP_COUNT; i=i+1) begin
	axis_opt_pipeline #(.WIDTH(OCOUNT_BITS + USER_BITS + COUNT_BITS), .PIPELINE(PIPELINE)) event_op (
	  .clk(clk),
	  .rst(rst),

	  .s_rx_tdata({ rd_evno_addr[OCOUNT_BITS-1:0], dispatch_user[USER_BITS-1:0], dispatch_event[COUNT_BITS-1:0] }),
	  .s_rx_tvalid(s_evno_valid && (i == dispatch_bus)),
	  .s_rx_tready(disp_ready[i]),

	  .m_tx_tdata({mn_evno_user[(OCOUNT_BITS + USER_BITS) * (i + 1) - 1:(OCOUNT_BITS + USER_BITS) * i], mn_evno_data[COUNT_BITS * (i + 1) - 1:COUNT_BITS * i]}),
	  .m_tx_tvalid(mn_evno_valid[i]),
	  .m_tx_tready(mn_evno_ready[i])
	);
end
endgenerate



endmodule

