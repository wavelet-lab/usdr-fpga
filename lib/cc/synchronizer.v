// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
// a_in  -- async input
// s_out -- sync output
module synchronizer #(
   parameter INIT         = 0,
   parameter ASYNC_RESET  = 0,
   parameter SYNC_STAGES  = 2
) (
   input  clk,
   input  rst,
   input  a_in,
   output s_out
);

(* ASYNC_REG = "TRUE" *)
reg [SYNC_STAGES-1:0] sreg;
assign                s_out = sreg[SYNC_STAGES-1];

generate
if (ASYNC_RESET) begin
  always @(posedge clk or posedge rst) sreg <= rst ? {SYNC_STAGES{INIT}} : { sreg[SYNC_STAGES-2:0], a_in };
end else begin
  always @(posedge clk)                sreg <= rst ? {SYNC_STAGES{INIT}} : { sreg[SYNC_STAGES-2:0], a_in };
end
endgenerate

endmodule
