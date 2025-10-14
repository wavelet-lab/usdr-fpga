// USDR CLEAN
//
// Calculate re*re + im*im
// Latency:    3 cycles
// Throughput: 1 cycle

module calc_pwr #(
    parameter WIDTH      = 16,
    parameter PWR_WIDTH  = 2 * WIDTH,
    parameter USER_WIDTH = 1
)(
  input clk,
  input rst,

  input [WIDTH-1:0]       i_re,
  input [WIDTH-1:0]       i_im,
  input                   i_valid,
  input [USER_WIDTH-1:0]  i_user,

  output [PWR_WIDTH-1:0]  o_power,
  output                  o_valid,
  output [USER_WIDTH-1:0] o_user
);

localparam A_WIDTH   = (WIDTH > 18) ? 18 : WIDTH;
localparam B_WIDTH   = (WIDTH > 25) ? 25 : WIDTH;
localparam LSB_A     = (WIDTH < 18) ? 0  : WIDTH-A_WIDTH;
localparam LSB_B     = (WIDTH < 25) ? 0  : WIDTH-B_WIDTH;

localparam MUL_WIDTH  = A_WIDTH + B_WIDTH - 1;
localparam PMUL_WIDTH = MUL_WIDTH + 1;

reg [MUL_WIDTH-1:0]     s1_re_square;
reg [A_WIDTH-1:0]       s1_im_a;
reg [B_WIDTH-1:0]       s1_im_b;
reg                     s1_valid;
reg                     s1_last;
reg [USER_WIDTH-1:0]    s1_user;

reg [MUL_WIDTH-1:0]     s2_re_square;
reg [MUL_WIDTH-1:0]     s2_im_square;
reg                     s2_valid;
reg                     s2_last;
reg [USER_WIDTH-1:0]    s2_user;

reg [MUL_WIDTH:0]       s3_sum_square; //Bit extension after sum
reg                     s3_valid;
reg                     s3_last;
reg [USER_WIDTH-1:0]    s3_user;

wire [A_WIDTH-1:0]    s0_re_a     = i_re[WIDTH-1:WIDTH-A_WIDTH];
wire [B_WIDTH-1:0]    s0_re_b     = i_re[WIDTH-1:WIDTH-B_WIDTH];
wire [A_WIDTH-1:0]    s0_im_a     = i_im[WIDTH-1:WIDTH-A_WIDTH];
wire [B_WIDTH-1:0]    s0_im_b     = i_im[WIDTH-1:WIDTH-B_WIDTH];
wire                  s0_valid    = i_valid;
wire [USER_WIDTH-1:0] s0_user     = i_user;

always @(posedge clk) begin
  if (rst) begin
    s1_valid           <= 0;
    s2_valid           <= 0;
    s3_valid           <= 0;

  end else begin
    s1_re_square  <= $signed(s0_re_a) * $signed(s0_re_b);
    s1_im_a       <= s0_im_a;
    s1_im_b       <= s0_im_b;
    s1_valid      <= s0_valid;
    s1_user       <= s0_user;

    s2_re_square  <= s1_re_square;
    s2_im_square  <= $signed(s1_im_a) * $signed(s1_im_b);
    s2_valid      <= s1_valid;
    s2_user       <= s1_user;

    s3_sum_square <= s2_re_square + s2_im_square;
    s3_valid      <= s2_valid;
    s3_user       <= s2_user;
  end
end

assign o_user   = s3_user;
assign o_valid  = s3_valid;
assign o_power  = (PWR_WIDTH > PMUL_WIDTH) ? { {(PWR_WIDTH - PMUL_WIDTH){s3_sum_square[PMUL_WIDTH-1]}}, s3_sum_square[PMUL_WIDTH-1:0] } :
                         s3_sum_square[PMUL_WIDTH-1:PMUL_WIDTH-PWR_WIDTH];


endmodule
