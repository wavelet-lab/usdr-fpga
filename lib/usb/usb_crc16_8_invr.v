// USB CRC16:
//  poly: x^16+x^15+x^2+1
//  init: 0xffff
//  d in: 15-msb, 0-lsb
//  crc:  reflected and inverted

module usb_crc16_8_invr(
    input [7:0]   d,
    input [15:0]  crci,
    output [15:0] crco,
    output        ok
);
wire [15:0] c = ~crci;
wire [15:0] crc;

assign crc[0] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ c[8] ^ c[7] ^ c[6] ^ c[5] ^ c[4] ^ c[3] ^ c[2] ^ c[1] ^ c[0];
assign crc[1] = c[9];
assign crc[2] = c[10];
assign crc[3] = c[11];
assign crc[4] = c[12];
assign crc[5] = c[13];
assign crc[6] = d[0] ^ c[14] ^ c[0];
assign crc[7] = d[0] ^ d[1] ^ c[15] ^ c[1] ^ c[0];
assign crc[8] = d[1] ^ d[2] ^ c[2] ^ c[1];
assign crc[9] = d[2] ^ d[3] ^ c[3] ^ c[2];
assign crc[10] = d[3] ^ d[4] ^ c[4] ^ c[3];
assign crc[11] = d[4] ^ d[5] ^ c[5] ^ c[4];
assign crc[12] = d[5] ^ d[6] ^ c[6] ^ c[5];
assign crc[13] = d[6] ^ d[7] ^ c[7] ^ c[6];
assign crc[14] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ c[6] ^ c[5] ^ c[4] ^ c[3] ^ c[2] ^ c[1] ^ c[0];
assign crc[15] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ c[7] ^ c[6] ^ c[5] ^ c[4] ^ c[3] ^ c[2] ^ c[1] ^ c[0];

assign crco = ~crc;

assign ok = (crc == 16'hb001);

endmodule
