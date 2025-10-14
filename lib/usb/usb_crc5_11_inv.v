// USB CRC5:
//  poly: x^5+x^2+1
//  init: 0x1f
//  in:   10-msb, 0-lsb
//  out:  reflected and inverted
module usb_crc5_11_inv(
    input [10:0] d,
    output [4:0] crco
);

assign crco[0] = d[0] ^ d[1] ^ d[2] ^ d[5] ^ d[6] ^ d[8];
assign crco[1] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[6] ^ d[7] ^ d[9] ^ 1'b1;
assign crco[2] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[7] ^ d[8] ^ d[10];
assign crco[3] = d[0] ^ d[3] ^ d[4] ^ d[6] ^ d[9];
assign crco[4] = d[0] ^ d[1] ^ d[4] ^ d[5] ^ d[7] ^ d[10];

endmodule
