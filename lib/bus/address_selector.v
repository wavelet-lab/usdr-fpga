// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module address_selector #(
    parameter ADDR_WIDTH = 32,
    parameter ADDR_ALIGN = 32,
    parameter [32*32 - 1 :0] ADDR_MASK = 0,
    parameter [32*32 - 1 :0] ADDR_COMP = 0,
    parameter COUNT = 1,
    parameter COUNT_BITS = $clog2(COUNT)
) (
    input  [ADDR_WIDTH-1:0] addr,
    output [COUNT_BITS-1:0] selector,
    output                  nmatch
);

wire [COUNT - 1:0] mask_comb;
genvar i;
for (i = 0; i < COUNT; i=i+1) begin
    assign mask_comb[i] = ((addr & ADDR_MASK[ADDR_ALIGN * (i + 1) - 1:ADDR_ALIGN * i]) == ADDR_COMP[ADDR_ALIGN * (i + 1) - 1:ADDR_ALIGN * i]);
end

ctz_clz #(.BITS(COUNT)) ctz_arddr_i(.data_i(mask_comb), .num_o(selector), .zero_o(nmatch));

endmodule

