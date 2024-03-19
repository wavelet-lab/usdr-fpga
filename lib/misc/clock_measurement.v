// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module clock_measurement #(
	parameter GEN_WIDTH = 4,
	parameter CNT_WIDTH = 28,
	parameter POSEDGE_ONLY = 0,
	parameter GATED_CLOCK = 0
) (
	input meas_clk_i,
	input meas_clk_ce_i,
	
	input ref_pulse_i,
	input ref_rst_i,
	
	output [GEN_WIDTH + CNT_WIDTH - 1:0] meas_data_o
);

wire ref_event;

synchronizer #(.INIT(0), .ASYNC_RESET(1)) clk_cntr_evnt (
    .clk(meas_clk_i),
    .rst(ref_rst_i),
    .a_in(ref_pulse_i),
    .s_out(ref_event)
);

reg [CNT_WIDTH - 1:0] ref_counter;
reg [GEN_WIDTH - 1:0] ref_ecounter_l = 0;
reg [CNT_WIDTH - 1:0] ref_counter_l  = 0;
reg                   ref_event_prev = 0;

always @(posedge meas_clk_i) begin
    ref_event_prev <= ref_event;

    if (meas_clk_ce_i || !GATED_CLOCK) begin
        ref_counter    <= ref_counter + 1'b1;
    end

    if ((ref_event_prev != ref_event) && (!POSEDGE_ONLY || ref_event_prev == 1'b0)) begin
        ref_ecounter_l <= ref_ecounter_l + 1'b1;
        ref_counter_l  <= ref_counter;
        ref_counter    <= 0;
    end
end

assign meas_data_o = { ref_ecounter_l, ref_counter_l };

endmodule
