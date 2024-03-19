// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module cc_counter #(
   parameter          WIDTH = 8,
   parameter          GRAY_BITS = WIDTH,
   parameter          CC_LOW_SKIP = 0,
   parameter          ASYNC_RESET = 0,
   parameter [1023:0] INIT = 0
)(
   input                           in_clk,
   input                           in_rst,
   input                           in_increment,
   output  [WIDTH-1:0]             in_counter,

   input                           out_clk,
   input                           out_rst,
   output  [WIDTH-1:CC_LOW_SKIP]   out_counter
);

localparam GRAY_WIDTH      = (CC_LOW_SKIP + GRAY_BITS > WIDTH) ? WIDTH : CC_LOW_SKIP + GRAY_BITS;
localparam GRAY_ITERATIONS = $clog2(GRAY_WIDTH);
localparam GRAY_NORM_WIDTH = 1 << GRAY_ITERATIONS;

reg  [WIDTH - 1:0]                 iclk_counter;
reg  [GRAY_WIDTH - 1:CC_LOW_SKIP]  iclk_counter_gray;
assign                             in_counter = iclk_counter;

always @(posedge in_clk) begin
  if (in_rst) begin
    iclk_counter      <= INIT;
    iclk_counter_gray <= INIT[GRAY_WIDTH - 1:CC_LOW_SKIP] ^ INIT[GRAY_WIDTH - 1:CC_LOW_SKIP + 1];
  end else begin
    if (in_increment) begin
        iclk_counter  <= iclk_counter + 1'b1;
    end

    iclk_counter_gray <= iclk_counter[GRAY_WIDTH - 1:CC_LOW_SKIP] ^ iclk_counter[GRAY_WIDTH - 1:CC_LOW_SKIP + 1];
  end
end

// Cross clock barrier
wire [GRAY_WIDTH - 1:CC_LOW_SKIP]                    oclk_counter_gray;
wire [GRAY_WIDTH - 1:CC_LOW_SKIP]                    oclk_counter;
reg  [GRAY_WIDTH - 1:CC_LOW_SKIP]                    oclk_counter_reg;

wire [GRAY_NORM_WIDTH * (GRAY_ITERATIONS + 1) - 1:0] gray_reverse_tmp;
assign                                               gray_reverse_tmp[GRAY_NORM_WIDTH - 1:0] = oclk_counter_gray;
assign                                               oclk_counter[GRAY_WIDTH - 1:CC_LOW_SKIP] = gray_reverse_tmp[GRAY_NORM_WIDTH * (GRAY_ITERATIONS + 1) - 1 : GRAY_NORM_WIDTH * GRAY_ITERATIONS];

/* verilator lint_off UNOPTFLAT */
genvar i;
generate
  for (i = CC_LOW_SKIP; i < GRAY_WIDTH; i = i + 1) begin: gray_bit_sync
    synchronizer #(.ASYNC_RESET(ASYNC_RESET)) sync_g_reg (
      .clk(out_clk),
      .rst(out_rst),
      .a_in(iclk_counter_gray[i]),
      .s_out(oclk_counter_gray[i])
    );
  end

  for (i = 0; i < GRAY_ITERATIONS; i = i + 1) begin: gray_reverse_iter
    wire [GRAY_NORM_WIDTH - 1:0] prev = gray_reverse_tmp[GRAY_NORM_WIDTH * (i + 1) - 1 : GRAY_NORM_WIDTH * (i + 0)];
    assign gray_reverse_tmp[GRAY_NORM_WIDTH * (i + 2) - 1 : GRAY_NORM_WIDTH * (i + 1)] = prev ^ (prev >> (1 << i));
  end

  if (ASYNC_RESET) begin
    always @(posedge out_clk or posedge out_rst) oclk_counter_reg <= out_rst ? INIT[GRAY_WIDTH - 1:CC_LOW_SKIP] : oclk_counter;
  end else begin
    always @(posedge out_clk)                    oclk_counter_reg <= out_rst ? INIT[GRAY_WIDTH - 1:CC_LOW_SKIP] : oclk_counter;
  end

  assign out_counter[GRAY_WIDTH - 1:CC_LOW_SKIP] = oclk_counter_reg;

  if (GRAY_WIDTH != WIDTH) begin: bit_extender
    reg  [WIDTH-1:GRAY_WIDTH] extra_part;
    wire                      extra_part_inc = (oclk_counter_reg[GRAY_WIDTH-1] == 1'b1 && oclk_counter[GRAY_WIDTH-1] == 1'b0);
    assign                    out_counter[WIDTH-1:GRAY_WIDTH] = extra_part;

    if (ASYNC_RESET) begin
        always @(posedge out_clk or posedge out_rst) begin
            if (out_rst) begin
                extra_part <= INIT[WIDTH-1:GRAY_WIDTH];
            end else if (extra_part_inc) begin
                extra_part <= extra_part + 1'b1;
            end
        end
    end else begin
        always @(posedge out_clk) begin
            if (out_rst) begin
                extra_part <= INIT[WIDTH-1:GRAY_WIDTH];
            end else if (extra_part_inc) begin
                extra_part <= extra_part + 1'b1;
            end
        end
    end
  end

endgenerate
/* verilator lint_on UNOPTFLAT */

endmodule
