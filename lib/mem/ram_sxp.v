// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
// Simple Single and Dual port memory primitive optimized for xilinx 7s and us 
// TODO: add initializaton parameters!!!
//
module ram_sxp #(
    parameter DATA_WIDTH  = 6,
    parameter ADDR_WIDTH  = 5,
    parameter ULTRA_SCALE = 0,
    parameter MODE_SDP    = 1
)(
   input                   wclk,
   input                   we,
   input  [ADDR_WIDTH-1:0] waddr,
   input  [DATA_WIDTH-1:0] wdata,

   input  [ADDR_WIDTH-1:0] raddr,
   output [DATA_WIDTH-1:0] rdata
);

`ifdef SYM
ram_sxp_sym #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) ram_sxp_sym (
    .wclk(wclk),
    .we(we),
    .waddr(waddr),
    .wdata(wdata),
    .raddr(MODE_SDP ? raddr : waddr),
    .rdata(rdata)
);
`else
genvar i;
generate

localparam SLICE_BITS_US = (MODE_SDP) ? (ADDR_WIDTH <= 5 ? 14 : 7) : (ADDR_WIDTH <= 5 ? 16 : 8);
localparam SLICE_BITS_7S = (MODE_SDP) ? (ADDR_WIDTH <= 5 ? 6 : 3)  : (ADDR_WIDTH <= 5 ? 8 : 4);

localparam SLICE_BITS = (ULTRA_SCALE) ? SLICE_BITS_US : SLICE_BITS_7S;
localparam COUNT      = (DATA_WIDTH + SLICE_BITS - 1) / SLICE_BITS;
wire [SLICE_BITS * COUNT-1:0] do;
wire [SLICE_BITS * COUNT-1:0] di = wdata;

for (i = 0; i < COUNT; i=i+1) begin
  if ((ULTRA_SCALE == 0 || DATA_WIDTH <= SLICE_BITS_7S) && ADDR_WIDTH <= 5) begin: part_7s_32
    wire [1:0] dextra;
    if (!MODE_SDP) begin
        assign do[SLICE_BITS*i+7:SLICE_BITS*i+6] = dextra;
    end
    RAM32M #(
        .INIT_A(64'h0000000000000000), // Initial contents of A Port
        .INIT_B(64'h0000000000000000), // Initial contents of B Port
        .INIT_C(64'h0000000000000000), // Initial contents of C Port
        .INIT_D(64'h0000000000000000)  // Initial contents of D Port
    ) RAM32X6SDP (
        .DOA(do[SLICE_BITS*i+1:SLICE_BITS*i+0]),
        .DOB(do[SLICE_BITS*i+3:SLICE_BITS*i+2]),
        .DOC(do[SLICE_BITS*i+5:SLICE_BITS*i+4]),
        .DOD(dextra),

        .ADDRA(MODE_SDP ? raddr : waddr),
        .ADDRB(MODE_SDP ? raddr : waddr),
        .ADDRC(MODE_SDP ? raddr : waddr),
        .ADDRD(waddr),

        .DIA(di[SLICE_BITS*i+1:SLICE_BITS*i+0]),
        .DIB(di[SLICE_BITS*i+3:SLICE_BITS*i+2]),
        .DIC(di[SLICE_BITS*i+5:SLICE_BITS*i+4]),
        .DID(MODE_SDP ? 2'b0 : di[SLICE_BITS*i+7:SLICE_BITS*i+6]),

        .WCLK(wclk),
        .WE(we)
    );
  end else if (ULTRA_SCALE == 1 && ADDR_WIDTH <= 5) begin: part_us_32
    wire [1:0] dextra;
    if (!MODE_SDP) begin
        assign do[SLICE_BITS*i+15:SLICE_BITS*i+14] = dextra;
    end
    RAM32M16 #(
        .INIT_A(64'h0000000000000000), // Initial contents of A Port
        .INIT_B(64'h0000000000000000), // Initial contents of B Port
        .INIT_C(64'h0000000000000000), // Initial contents of C Port
        .INIT_D(64'h0000000000000000), // Initial contents of D Port
        .INIT_E(64'h0000000000000000), // Initial contents of E Port
        .INIT_F(64'h0000000000000000), // Initial contents of F Port
        .INIT_G(64'h0000000000000000), // Initial contents of G Port
        .INIT_H(64'h0000000000000000)  // Initial contents of H Port
    ) RAM32X14SDP (
        .DOA(do[SLICE_BITS*i+1:SLICE_BITS*i+0]),
        .DOB(do[SLICE_BITS*i+3:SLICE_BITS*i+2]),
        .DOC(do[SLICE_BITS*i+5:SLICE_BITS*i+4]),
        .DOD(do[SLICE_BITS*i+7:SLICE_BITS*i+6]),
        .DOE(do[SLICE_BITS*i+9:SLICE_BITS*i+8]),
        .DOF(do[SLICE_BITS*i+11:SLICE_BITS*i+10]),
        .DOG(do[SLICE_BITS*i+13:SLICE_BITS*i+12]),
        .DOH(dextra),

        .ADDRA(MODE_SDP ? raddr : waddr),
        .ADDRB(MODE_SDP ? raddr : waddr),
        .ADDRC(MODE_SDP ? raddr : waddr),
        .ADDRD(MODE_SDP ? raddr : waddr),
        .ADDRE(MODE_SDP ? raddr : waddr),
        .ADDRF(MODE_SDP ? raddr : waddr),
        .ADDRG(MODE_SDP ? raddr : waddr),
        .ADDRH(waddr),

        .DIA(di[SLICE_BITS*i+1:SLICE_BITS*i+0]),
        .DIB(di[SLICE_BITS*i+3:SLICE_BITS*i+2]),
        .DIC(di[SLICE_BITS*i+5:SLICE_BITS*i+4]),
        .DID(di[SLICE_BITS*i+7:SLICE_BITS*i+6]),
        .DIE(di[SLICE_BITS*i+9:SLICE_BITS*i+8]),
        .DIF(di[SLICE_BITS*i+11:SLICE_BITS*i+10]),
        .DIG(di[SLICE_BITS*i+13:SLICE_BITS*i+12]),
        .DIH(MODE_SDP ? 2'b0 : di[SLICE_BITS*i+15:SLICE_BITS*i+14]),

        .WCLK(wclk),
        .WE(we)
    );
  end else if ((ULTRA_SCALE == 0 || DATA_WIDTH <= 3) && ADDR_WIDTH == 6) begin: part_7s_64
    wire [0:0] dextra;
    if (!MODE_SDP) begin
        assign do[SLICE_BITS*i+3] = dextra;
    end
    RAM64M #(
        .INIT_A(64'h0000000000000000), // Initial contents of A Port
        .INIT_B(64'h0000000000000000), // Initial contents of B Port
        .INIT_C(64'h0000000000000000), // Initial contents of C Port
        .INIT_D(64'h0000000000000000)  // Initial contents of D Port
    ) RAM64X3SDP (
        .DOA(do[SLICE_BITS*i+0]),
        .DOB(do[SLICE_BITS*i+1]),
        .DOC(do[SLICE_BITS*i+2]),
        .DOD(dextra),

        .ADDRA(MODE_SDP ? raddr : waddr),
        .ADDRB(MODE_SDP ? raddr : waddr),
        .ADDRC(MODE_SDP ? raddr : waddr),
        .ADDRD(waddr),

        .DIA(di[SLICE_BITS*i+0]),
        .DIB(di[SLICE_BITS*i+1]),
        .DIC(di[SLICE_BITS*i+2]),
        .DID(MODE_SDP ? 1'b0 : di[SLICE_BITS*i+3]),

        .WCLK(wclk),
        .WE(we)
    );
  end else if (ULTRA_SCALE == 1 && ADDR_WIDTH == 6) begin: part_us_64
    wire [0:0] dextra;
    if (!MODE_SDP) begin
        assign do[SLICE_BITS*i+7] = dextra;
    end
    RAM64M8 #(
        .INIT_A(64'h0000000000000000), // Initial contents of A Port
        .INIT_B(64'h0000000000000000), // Initial contents of B Port
        .INIT_C(64'h0000000000000000), // Initial contents of C Port
        .INIT_D(64'h0000000000000000), // Initial contents of D Port
        .INIT_E(64'h0000000000000000), // Initial contents of E Port
        .INIT_F(64'h0000000000000000), // Initial contents of F Port
        .INIT_G(64'h0000000000000000), // Initial contents of G Port
        .INIT_H(64'h0000000000000000)  // Initial contents of H Port
    ) RAM64X7SDP (
        .DOA(do[SLICE_BITS*i+0]),
        .DOB(do[SLICE_BITS*i+1]),
        .DOC(do[SLICE_BITS*i+2]),
        .DOD(do[SLICE_BITS*i+3]),
        .DOE(do[SLICE_BITS*i+4]),
        .DOF(do[SLICE_BITS*i+5]),
        .DOG(do[SLICE_BITS*i+6]),
        .DOH(dextra),

        .ADDRA(MODE_SDP ? raddr : waddr),
        .ADDRB(MODE_SDP ? raddr : waddr),
        .ADDRC(MODE_SDP ? raddr : waddr),
        .ADDRD(MODE_SDP ? raddr : waddr),
        .ADDRE(MODE_SDP ? raddr : waddr),
        .ADDRF(MODE_SDP ? raddr : waddr),
        .ADDRG(MODE_SDP ? raddr : waddr),
        .ADDRH(waddr),

        .DIA(di[SLICE_BITS*i+0]),
        .DIB(di[SLICE_BITS*i+1]),
        .DIC(di[SLICE_BITS*i+2]),
        .DID(di[SLICE_BITS*i+3]),
        .DIE(di[SLICE_BITS*i+4]),
        .DIF(di[SLICE_BITS*i+5]),
        .DIG(di[SLICE_BITS*i+6]),
        .DIH(MODE_SDP ? 1'b0 : di[SLICE_BITS*i+7]),

        .WCLK(wclk),
        .WE(we)
    );
  end else begin
    unsupported_dpram_configuration assert_dpram(.DI(di), .DO(do));
  end
end

assign rdata = do[DATA_WIDTH-1:0];


endgenerate
`endif


endmodule
