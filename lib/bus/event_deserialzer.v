// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module event_deserialzer #(
    parameter COUNT = 16,
    parameter COUNT_BITS = 4
) (
    input clk,
    input rst,

    // Event source
    output reg [COUNT - 1:0]   mn_event_valid,
    input [COUNT - 1:0]        mn_event_ready,

    // Serialized event number    
    input [COUNT_BITS - 1:0]   s_evno_data,
    input                      s_evno_valid,
    output                     s_evno_ready
);

assign s_evno_ready = mn_event_ready[s_evno_data] || ~mn_event_valid[s_evno_data];

genvar i;
generate
for (i = 0; i < COUNT; i=i+1) begin: gen_vec
	always @(posedge clk) begin
		if (rst) begin
			mn_event_valid[i] <= 0;
		end else begin
			if (mn_event_valid[i] && mn_event_ready[i]) begin
				mn_event_valid[i] <= 0;
			end
			
			if (s_evno_valid && s_evno_ready && (s_evno_data == i)) begin
				mn_event_valid[i] <= 1'b1;
			end
		end
	end
end
endgenerate


endmodule

