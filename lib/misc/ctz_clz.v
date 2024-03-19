// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
// TODO: Add optimization for high order numbers
//
// If all is none return bit count
//
module ctz_clz #(
    parameter BITS = 32,
    parameter CLZ  =  0,
    parameter LOG_BITS = $clog2(BITS),
    parameter ZERO_IF_NONE = 0
) (
    input  [BITS-1:0]     data_i,
    output [LOG_BITS-1:0] num_o,
    output                zero_o
);

localparam F_BITS  = 1 << LOG_BITS;
localparam F_EXTRA = F_BITS - BITS;

wire  [F_BITS-1:0]     full_data    =  ((F_EXTRA > 0) && CLZ) ? { data_i, {F_EXTRA{1'b0}} } : data_i;
wire  [F_BITS-1:0]     full_data_r;

genvar i;
generate

if (BITS > 64) begin
    assert_maximum_bits_is_64 assert_failed(.data_i(data_i));
end

for (i = 0; i < F_BITS; i=i+1) begin
    assign full_data_r[i] = CLZ ? full_data[F_BITS - 1 - i] : full_data[i];
end

assign num_o = 
    full_data_r[0] ? 1'b0 :
    (LOG_BITS >= 1) && full_data_r[1]  ? 1'b1 :
    (LOG_BITS >= 2) && full_data_r[2]  ? 2'b10 :
    (LOG_BITS >= 2) && full_data_r[3]  ? 2'b11 :
    (LOG_BITS >= 3) && full_data_r[4]  ? 3'b100 :
    (LOG_BITS >= 3) && full_data_r[5]  ? 3'b101 :
    (LOG_BITS >= 3) && full_data_r[6]  ? 3'b110 :
    (LOG_BITS >= 3) && full_data_r[7]  ? 3'b111 :
    (LOG_BITS >= 4) && full_data_r[8]  ? 4'b1000 :
    (LOG_BITS >= 4) && full_data_r[9]  ? 4'b1001 :
    (LOG_BITS >= 4) && full_data_r[10] ? 4'b1010 :
    (LOG_BITS >= 4) && full_data_r[11] ? 4'b1011 :
    (LOG_BITS >= 4) && full_data_r[12] ? 4'b1100 :
    (LOG_BITS >= 4) && full_data_r[13] ? 4'b1101 :
    (LOG_BITS >= 4) && full_data_r[14] ? 4'b1110 :
    (LOG_BITS >= 4) && full_data_r[15] ? 4'b1111 :
    (LOG_BITS >= 5) && full_data_r[16] ? 5'b10000 :
    (LOG_BITS >= 5) && full_data_r[17] ? 5'b10001 :
    (LOG_BITS >= 5) && full_data_r[18] ? 5'b10010 :
    (LOG_BITS >= 5) && full_data_r[19] ? 5'b10011 :
    (LOG_BITS >= 5) && full_data_r[20] ? 5'b10100 :
    (LOG_BITS >= 5) && full_data_r[21] ? 5'b10101 :
    (LOG_BITS >= 5) && full_data_r[22] ? 5'b10110 :
    (LOG_BITS >= 5) && full_data_r[23] ? 5'b10111 :
    (LOG_BITS >= 5) && full_data_r[24] ? 5'b11000 :
    (LOG_BITS >= 5) && full_data_r[25] ? 5'b11001 :
    (LOG_BITS >= 5) && full_data_r[26] ? 5'b11010 :
    (LOG_BITS >= 5) && full_data_r[27] ? 5'b11011 :
    (LOG_BITS >= 5) && full_data_r[28] ? 5'b11100 :
    (LOG_BITS >= 5) && full_data_r[29] ? 5'b11101 :
    (LOG_BITS >= 5) && full_data_r[30] ? 5'b11110 :
    (LOG_BITS >= 5) && full_data_r[31] ? 5'b11111 :
    (LOG_BITS >= 6) && full_data_r[32] ? 6'b100000 :
    (LOG_BITS >= 6) && full_data_r[33] ? 6'b100001 :
    (LOG_BITS >= 6) && full_data_r[34] ? 6'b100010 :
    (LOG_BITS >= 6) && full_data_r[35] ? 6'b100011 :
    (LOG_BITS >= 6) && full_data_r[36] ? 6'b100100 :
    (LOG_BITS >= 6) && full_data_r[37] ? 6'b100101 :
    (LOG_BITS >= 6) && full_data_r[38] ? 6'b100110 :
    (LOG_BITS >= 6) && full_data_r[39] ? 6'b100111 :
    (LOG_BITS >= 6) && full_data_r[40] ? 6'b101000 :
    (LOG_BITS >= 6) && full_data_r[41] ? 6'b101001 :
    (LOG_BITS >= 6) && full_data_r[42] ? 6'b101010 :
    (LOG_BITS >= 6) && full_data_r[43] ? 6'b101011 :
    (LOG_BITS >= 6) && full_data_r[44] ? 6'b101100 :
    (LOG_BITS >= 6) && full_data_r[45] ? 6'b101101 :
    (LOG_BITS >= 6) && full_data_r[46] ? 6'b101110 :
    (LOG_BITS >= 6) && full_data_r[47] ? 6'b101111 :
    (LOG_BITS >= 6) && full_data_r[48] ? 6'b110000 :
    (LOG_BITS >= 6) && full_data_r[49] ? 6'b110001 :
    (LOG_BITS >= 6) && full_data_r[50] ? 6'b110010 :
    (LOG_BITS >= 6) && full_data_r[51] ? 6'b110011 :
    (LOG_BITS >= 6) && full_data_r[52] ? 6'b110100 :
    (LOG_BITS >= 6) && full_data_r[53] ? 6'b110101 :
    (LOG_BITS >= 6) && full_data_r[54] ? 6'b110110 :
    (LOG_BITS >= 6) && full_data_r[55] ? 6'b110111 :
    (LOG_BITS >= 6) && full_data_r[56] ? 6'b111000 :
    (LOG_BITS >= 6) && full_data_r[57] ? 6'b111001 :
    (LOG_BITS >= 6) && full_data_r[58] ? 6'b111010 :
    (LOG_BITS >= 6) && full_data_r[59] ? 6'b111011 :
    (LOG_BITS >= 6) && full_data_r[60] ? 6'b111100 :
    (LOG_BITS >= 6) && full_data_r[61] ? 6'b111101 :
    (LOG_BITS >= 6) && full_data_r[62] ? 6'b111110 :
    (LOG_BITS >= 6) && full_data_r[63] ? 6'b111111 : (ZERO_IF_NONE ? 0 :(BITS - 1));

assign zero_o = &(~full_data_r);
endgenerate


endmodule
