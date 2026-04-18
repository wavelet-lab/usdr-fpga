// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module ram_sxp_sym #(
    parameter DATA_WIDTH = 6,
    parameter ADDR_WIDTH = 5
)(
   input                   wclk,
   input                   we,
   input  [ADDR_WIDTH-1:0] waddr,
   input  [DATA_WIDTH-1:0] wdata,

   input  [ADDR_WIDTH-1:0] raddr,
   output [DATA_WIDTH-1:0] rdata
);

// Easy debugging
generate
if (ADDR_WIDTH == 1) begin: addr_2
reg [DATA_WIDTH-1:0] r0;
reg [DATA_WIDTH-1:0] r1;

always @(posedge wclk) begin
  if (we) begin
    case (waddr)
      0:  r0 <= wdata;
      1:  r1 <= wdata;
    endcase
  end
end

assign rdata = 
    (raddr == 0) ? r0 : r1;

end else if (ADDR_WIDTH == 2) begin: addr_4
reg [DATA_WIDTH-1:0] r0;
reg [DATA_WIDTH-1:0] r1;
reg [DATA_WIDTH-1:0] r2;
reg [DATA_WIDTH-1:0] r3;

always @(posedge wclk) begin
  if (we) begin
    case (waddr)
      0:  r0 <= wdata;
      1:  r1 <= wdata;
      2:  r2 <= wdata;
      3:  r3 <= wdata;
    endcase
  end
end

assign rdata = 
    (raddr == 0) ? r0 :
    (raddr == 1) ? r1 :
    (raddr == 2) ? r2 : r3;
    
end else if (ADDR_WIDTH == 3) begin: addr_8
reg [DATA_WIDTH-1:0] r0;
reg [DATA_WIDTH-1:0] r1;
reg [DATA_WIDTH-1:0] r2;
reg [DATA_WIDTH-1:0] r3;
reg [DATA_WIDTH-1:0] r4;
reg [DATA_WIDTH-1:0] r5;
reg [DATA_WIDTH-1:0] r6;
reg [DATA_WIDTH-1:0] r7;

always @(posedge wclk) begin
  if (we) begin
    case (waddr)
      0:  r0 <= wdata;
      1:  r1 <= wdata;
      2:  r2 <= wdata;
      3:  r3 <= wdata;
      4:  r4 <= wdata;
      5:  r5 <= wdata;
      6:  r6 <= wdata;
      7:  r7 <= wdata;
    endcase
  end
end

assign rdata = 
    (raddr == 0) ? r0 :
    (raddr == 1) ? r1 :
    (raddr == 2) ? r2 :
    (raddr == 3) ? r3 :
    (raddr == 4) ? r4 :
    (raddr == 5) ? r5 :
    (raddr == 6) ? r6 : r7;
    
end else if (ADDR_WIDTH == 4) begin: addr_16
reg [DATA_WIDTH-1:0] r0;
reg [DATA_WIDTH-1:0] r1;
reg [DATA_WIDTH-1:0] r2;
reg [DATA_WIDTH-1:0] r3;
reg [DATA_WIDTH-1:0] r4;
reg [DATA_WIDTH-1:0] r5;
reg [DATA_WIDTH-1:0] r6;
reg [DATA_WIDTH-1:0] r7;
reg [DATA_WIDTH-1:0] r8;
reg [DATA_WIDTH-1:0] r9;
reg [DATA_WIDTH-1:0] r10;
reg [DATA_WIDTH-1:0] r11;
reg [DATA_WIDTH-1:0] r12;
reg [DATA_WIDTH-1:0] r13;
reg [DATA_WIDTH-1:0] r14;
reg [DATA_WIDTH-1:0] r15;

always @(posedge wclk) begin
  if (we) begin
    case (waddr)
      0:  r0 <= wdata;
      1:  r1 <= wdata;
      2:  r2 <= wdata;
      3:  r3 <= wdata;
      4:  r4 <= wdata;
      5:  r5 <= wdata;
      6:  r6 <= wdata;
      7:  r7 <= wdata;
      8:  r8 <= wdata;
      9:  r9 <= wdata;
      10: r10<= wdata;
      11: r11<= wdata;
      12: r12<= wdata;
      13: r13<= wdata;
      14: r14<= wdata;
      15: r15<= wdata;
    endcase
  end
end

assign rdata = 
    (raddr == 0) ? r0 :
    (raddr == 1) ? r1 :
    (raddr == 2) ? r2 :
    (raddr == 3) ? r3 :
    (raddr == 4) ? r4 :
    (raddr == 5) ? r5 :
    (raddr == 6) ? r6 :
    (raddr == 7) ? r7 :
    (raddr == 8) ? r8 :
    (raddr == 9) ? r9 :
    (raddr == 10) ? r10 :
    (raddr == 11) ? r11 :
    (raddr == 12) ? r12 :
    (raddr == 13) ? r13 :
    (raddr == 14) ? r14 : r15;

end else if (ADDR_WIDTH == 5) begin: addr_32
reg [DATA_WIDTH-1:0] r0;
reg [DATA_WIDTH-1:0] r1;
reg [DATA_WIDTH-1:0] r2;
reg [DATA_WIDTH-1:0] r3;
reg [DATA_WIDTH-1:0] r4;
reg [DATA_WIDTH-1:0] r5;
reg [DATA_WIDTH-1:0] r6;
reg [DATA_WIDTH-1:0] r7;
reg [DATA_WIDTH-1:0] r8;
reg [DATA_WIDTH-1:0] r9;
reg [DATA_WIDTH-1:0] r10;
reg [DATA_WIDTH-1:0] r11;
reg [DATA_WIDTH-1:0] r12;
reg [DATA_WIDTH-1:0] r13;
reg [DATA_WIDTH-1:0] r14;
reg [DATA_WIDTH-1:0] r15;
reg [DATA_WIDTH-1:0] r16;
reg [DATA_WIDTH-1:0] r17;
reg [DATA_WIDTH-1:0] r18;
reg [DATA_WIDTH-1:0] r19;
reg [DATA_WIDTH-1:0] r20;
reg [DATA_WIDTH-1:0] r21;
reg [DATA_WIDTH-1:0] r22;
reg [DATA_WIDTH-1:0] r23;
reg [DATA_WIDTH-1:0] r24;
reg [DATA_WIDTH-1:0] r25;
reg [DATA_WIDTH-1:0] r26;
reg [DATA_WIDTH-1:0] r27;
reg [DATA_WIDTH-1:0] r28;
reg [DATA_WIDTH-1:0] r29;
reg [DATA_WIDTH-1:0] r30;
reg [DATA_WIDTH-1:0] r31;

always @(posedge wclk) begin
  if (we) begin
    case (waddr)
      0:  r0 <= wdata;
      1:  r1 <= wdata;
      2:  r2 <= wdata;
      3:  r3 <= wdata;
      4:  r4 <= wdata;
      5:  r5 <= wdata;
      6:  r6 <= wdata;
      7:  r7 <= wdata;
      8:  r8 <= wdata;
      9:  r9 <= wdata;
      10: r10<= wdata;
      11: r11<= wdata;
      12: r12<= wdata;
      13: r13<= wdata;
      14: r14<= wdata;
      15: r15<= wdata;
      16: r16<= wdata;
      17: r17<= wdata;
      18: r18<= wdata;
      19: r19<= wdata;
      20: r20<= wdata;
      21: r21<= wdata;
      22: r22<= wdata;
      23: r23<= wdata;
      24: r24<= wdata;
      25: r25<= wdata;
      26: r26<= wdata;
      27: r27<= wdata;
      28: r28<= wdata;
      29: r29<= wdata;
      30: r30<= wdata;
      31: r31<= wdata;
    endcase
  end
end

assign rdata = 
    (raddr == 0) ? r0 :
    (raddr == 1) ? r1 :
    (raddr == 2) ? r2 :
    (raddr == 3) ? r3 :
    (raddr == 4) ? r4 :
    (raddr == 5) ? r5 :
    (raddr == 6) ? r6 :
    (raddr == 7) ? r7 :
    (raddr == 8) ? r8 :
    (raddr == 9) ? r9 :
    (raddr == 10) ? r10 :
    (raddr == 11) ? r11 :
    (raddr == 12) ? r12 :
    (raddr == 13) ? r13 :
    (raddr == 14) ? r14 :
    (raddr == 15) ? r15 :
    (raddr == 16) ? r16 :
    (raddr == 17) ? r17 :
    (raddr == 18) ? r18 :
    (raddr == 19) ? r19 :
    (raddr == 20) ? r20 :
    (raddr == 21) ? r21 :
    (raddr == 22) ? r22 :
    (raddr == 23) ? r23 :
    (raddr == 24) ? r24 :
    (raddr == 25) ? r25 :
    (raddr == 26) ? r26 :
    (raddr == 27) ? r27 :
    (raddr == 28) ? r28 :
    (raddr == 29) ? r29 :
    (raddr == 30) ? r30 : r31;
    
end else if (ADDR_WIDTH == 6) begin: addr_64
reg [DATA_WIDTH-1:0] r0;
reg [DATA_WIDTH-1:0] r1;
reg [DATA_WIDTH-1:0] r2;
reg [DATA_WIDTH-1:0] r3;
reg [DATA_WIDTH-1:0] r4;
reg [DATA_WIDTH-1:0] r5;
reg [DATA_WIDTH-1:0] r6;
reg [DATA_WIDTH-1:0] r7;
reg [DATA_WIDTH-1:0] r8;
reg [DATA_WIDTH-1:0] r9;
reg [DATA_WIDTH-1:0] r10;
reg [DATA_WIDTH-1:0] r11;
reg [DATA_WIDTH-1:0] r12;
reg [DATA_WIDTH-1:0] r13;
reg [DATA_WIDTH-1:0] r14;
reg [DATA_WIDTH-1:0] r15;
reg [DATA_WIDTH-1:0] r16;
reg [DATA_WIDTH-1:0] r17;
reg [DATA_WIDTH-1:0] r18;
reg [DATA_WIDTH-1:0] r19;
reg [DATA_WIDTH-1:0] r20;
reg [DATA_WIDTH-1:0] r21;
reg [DATA_WIDTH-1:0] r22;
reg [DATA_WIDTH-1:0] r23;
reg [DATA_WIDTH-1:0] r24;
reg [DATA_WIDTH-1:0] r25;
reg [DATA_WIDTH-1:0] r26;
reg [DATA_WIDTH-1:0] r27;
reg [DATA_WIDTH-1:0] r28;
reg [DATA_WIDTH-1:0] r29;
reg [DATA_WIDTH-1:0] r30;
reg [DATA_WIDTH-1:0] r31;
reg [DATA_WIDTH-1:0] r32;
reg [DATA_WIDTH-1:0] r33;
reg [DATA_WIDTH-1:0] r34;
reg [DATA_WIDTH-1:0] r35;
reg [DATA_WIDTH-1:0] r36;
reg [DATA_WIDTH-1:0] r37;
reg [DATA_WIDTH-1:0] r38;
reg [DATA_WIDTH-1:0] r39;
reg [DATA_WIDTH-1:0] r40;
reg [DATA_WIDTH-1:0] r41;
reg [DATA_WIDTH-1:0] r42;
reg [DATA_WIDTH-1:0] r43;
reg [DATA_WIDTH-1:0] r44;
reg [DATA_WIDTH-1:0] r45;
reg [DATA_WIDTH-1:0] r46;
reg [DATA_WIDTH-1:0] r47;
reg [DATA_WIDTH-1:0] r48;
reg [DATA_WIDTH-1:0] r49;
reg [DATA_WIDTH-1:0] r50;
reg [DATA_WIDTH-1:0] r51;
reg [DATA_WIDTH-1:0] r52;
reg [DATA_WIDTH-1:0] r53;
reg [DATA_WIDTH-1:0] r54;
reg [DATA_WIDTH-1:0] r55;
reg [DATA_WIDTH-1:0] r56;
reg [DATA_WIDTH-1:0] r57;
reg [DATA_WIDTH-1:0] r58;
reg [DATA_WIDTH-1:0] r59;
reg [DATA_WIDTH-1:0] r60;
reg [DATA_WIDTH-1:0] r61;
reg [DATA_WIDTH-1:0] r62;
reg [DATA_WIDTH-1:0] r63;

always @(posedge wclk) begin
  if (we) begin
    case (waddr)
      0:  r0 <= wdata;
      1:  r1 <= wdata;
      2:  r2 <= wdata;
      3:  r3 <= wdata;
      4:  r4 <= wdata;
      5:  r5 <= wdata;
      6:  r6 <= wdata;
      7:  r7 <= wdata;
      8:  r8 <= wdata;
      9:  r9 <= wdata;
      10: r10<= wdata;
      11: r11<= wdata;
      12: r12<= wdata;
      13: r13<= wdata;
      14: r14<= wdata;
      15: r15<= wdata;
      16: r16<= wdata;
      17: r17<= wdata;
      18: r18<= wdata;
      19: r19<= wdata;
      20: r20<= wdata;
      21: r21<= wdata;
      22: r22<= wdata;
      23: r23<= wdata;
      24: r24<= wdata;
      25: r25<= wdata;
      26: r26<= wdata;
      27: r27<= wdata;
      28: r28<= wdata;
      29: r29<= wdata;
      30: r30<= wdata;
      31: r31<= wdata;
      32: r32<= wdata;
      33: r33<= wdata;
      34: r34<= wdata;
      35: r35<= wdata;
      36: r36<= wdata;
      37: r37<= wdata;
      38: r38<= wdata;
      39: r39<= wdata;
      40: r40<= wdata;
      41: r41<= wdata;
      42: r42<= wdata;
      43: r43<= wdata;
      44: r44<= wdata;
      45: r45<= wdata;
      46: r46<= wdata;
      47: r47<= wdata;
      48: r48<= wdata;
      49: r49<= wdata;
      50: r50<= wdata;
      51: r51<= wdata;
      52: r52<= wdata;
      53: r53<= wdata;
      54: r54<= wdata;
      55: r55<= wdata;
      56: r56<= wdata;
      57: r57<= wdata;
      58: r58<= wdata;
      59: r59<= wdata;
      60: r60<= wdata;
      61: r61<= wdata;
      62: r62<= wdata;
      63: r63<= wdata;
    endcase
  end
end

assign rdata = 
    (raddr == 0) ? r0 :
    (raddr == 1) ? r1 :
    (raddr == 2) ? r2 :
    (raddr == 3) ? r3 :
    (raddr == 4) ? r4 :
    (raddr == 5) ? r5 :
    (raddr == 6) ? r6 :
    (raddr == 7) ? r7 :
    (raddr == 8) ? r8 :
    (raddr == 9) ? r9 :
    (raddr == 10) ? r10 :
    (raddr == 11) ? r11 :
    (raddr == 12) ? r12 :
    (raddr == 13) ? r13 :
    (raddr == 14) ? r14 :
    (raddr == 15) ? r15 :
    (raddr == 16) ? r16 :
    (raddr == 17) ? r17 :
    (raddr == 18) ? r18 :
    (raddr == 19) ? r19 :
    (raddr == 20) ? r20 :
    (raddr == 21) ? r21 :
    (raddr == 22) ? r22 :
    (raddr == 23) ? r23 :
    (raddr == 24) ? r24 :
    (raddr == 25) ? r25 :
    (raddr == 26) ? r26 :
    (raddr == 27) ? r27 :
    (raddr == 28) ? r28 :
    (raddr == 29) ? r29 :
    (raddr == 30) ? r30 :
    (raddr == 31) ? r31 :
    (raddr == 32) ? r32 :
    (raddr == 33) ? r33 :
    (raddr == 34) ? r34 :
    (raddr == 35) ? r35 :
    (raddr == 36) ? r36 :
    (raddr == 37) ? r37 :
    (raddr == 38) ? r38 :
    (raddr == 39) ? r39 :
    (raddr == 40) ? r40 :
    (raddr == 41) ? r41 :
    (raddr == 42) ? r42 :
    (raddr == 43) ? r43 :
    (raddr == 44) ? r44 :
    (raddr == 45) ? r45 :
    (raddr == 46) ? r46 :
    (raddr == 47) ? r47 :
    (raddr == 48) ? r48 :
    (raddr == 49) ? r49 :
    (raddr == 50) ? r50 :
    (raddr == 51) ? r51 :
    (raddr == 52) ? r52 :
    (raddr == 53) ? r53 :
    (raddr == 54) ? r54 :
    (raddr == 55) ? r55 :
    (raddr == 56) ? r56 :
    (raddr == 57) ? r57 :
    (raddr == 58) ? r58 :
    (raddr == 59) ? r59 :
    (raddr == 60) ? r60 :
    (raddr == 61) ? r61 :
    (raddr == 62) ? r62 :
                    r63;

end else begin
    reg [DATA_WIDTH-1:0] bulk_storage[0:(1<<ADDR_WIDTH) - 1];
    always @(posedge wclk) begin
        if (we) begin
            bulk_storage[waddr] <= wdata;
        end
    end
    assign rdata = bulk_storage[raddr];
end
endgenerate

endmodule
