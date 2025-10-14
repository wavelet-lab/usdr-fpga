module fft_log_power_calc #(
  parameter IN_WIDTH = 22,
  parameter IN_USER_WIDTH = 1
)(
  input clk,
  input rst,

  input [IN_WIDTH-1:0]       i_re,
  input [IN_WIDTH-1:0]       i_im,
  input                      i_valid,
  input                      i_last,
  input [IN_USER_WIDTH-1:0]  i_user,

  output [15:0]              o_power,
  output                     o_valid,
  output                     o_last,
  output [IN_USER_WIDTH-1:0] o_user
);

localparam A_WIDTH = (IN_WIDTH > 18) ? 18 : IN_WIDTH;
localparam B_WIDTH = (IN_WIDTH > 25) ? 25 : IN_WIDTH;

localparam MUL_WIDTH = A_WIDTH + B_WIDTH;


localparam LOG_PREC_I = 8;
localparam LOG_ORD    = 6;

reg [A_WIDTH-1:0]       s0_re_a;
reg [B_WIDTH-1:0]       s0_re_b;
reg [A_WIDTH-1:0]       s0_im_a;
reg [B_WIDTH-1:0]       s0_im_b;
reg                     s0_valid;
reg                     s0_last;
reg [IN_USER_WIDTH-1:0] s0_user;

reg [MUL_WIDTH-1:0]     s1_re_square;
reg [A_WIDTH-1:0]       s1_im_a;
reg [B_WIDTH-1:0]       s1_im_b;
reg                     s1_valid;
reg                     s1_last;
reg [IN_USER_WIDTH-1:0] s1_user;

reg [MUL_WIDTH-1:0]     s2_re_square;
reg [MUL_WIDTH-1:0]     s2_im_square;
reg                     s2_valid;
reg                     s2_last;
reg [IN_USER_WIDTH-1:0] s2_user;

reg [MUL_WIDTH:0]       s3_sum_square; //Bit extension after sum
reg                     s3_valid;
reg                     s3_last;
reg [IN_USER_WIDTH-1:0] s3_user;

wire [MUL_WIDTH-1:0]    pwr_sum = s3_sum_square[MUL_WIDTH-1:0];  //without sign
reg [5:0]               pwr_exp;
reg [MUL_WIDTH-1:0]     pwr_rsum;
reg                     pwr_rsum_exp_valid;
reg                     s4_last;
reg [IN_USER_WIDTH-1:0] s4_user;

wire [MUL_WIDTH-1:0]    pwr_norm     = pwr_rsum << pwr_exp;
wire [LOG_PREC_I-1:0]   pwr_normcut  = pwr_norm[MUL_WIDTH - 2:MUL_WIDTH - 2 - LOG_PREC_I + 1];

reg [7:0]               pwr_mnt;
reg [5:0]               pwr_exp1;
reg                     logmnt_exp1_valid;
wire [7:0]              ilogval = pwr_mnt;
reg                     s5_last;
reg [IN_USER_WIDTH-1:0] s5_user;


reg [5:0]               pwr_corr_exp;
reg [9:0]               pwr_fractional;
reg                     pwr_corr_frac_valid;
reg                     s6_last;
reg [IN_USER_WIDTH-1:0] s6_user;

(* rom_style = "distributed" *)
reg [6:0] log2drom [2**8-1:0];
initial $readmemh("log2rom_d256.data", log2drom);
wire [6:0] ilogout = log2drom[ilogval];

assign o_power = { pwr_corr_exp, pwr_fractional };
assign o_valid = pwr_corr_frac_valid;
assign o_last  = s6_last;
assign o_user  = s6_user;

localparam MAX_PWR = 42;

`define PWR_CHK(x) pwr_sum[(x)] == 1'b1 ? MAX_PWR - ((x)+1) :

wire [5:0] exp_sum =                                                                                        `PWR_CHK(41) `PWR_CHK(40)
    `PWR_CHK(39) `PWR_CHK(38) `PWR_CHK(37) `PWR_CHK(36) `PWR_CHK(35) `PWR_CHK(34) `PWR_CHK(33) `PWR_CHK(32) `PWR_CHK(31) `PWR_CHK(30)
    `PWR_CHK(29) `PWR_CHK(28) `PWR_CHK(27) `PWR_CHK(26) `PWR_CHK(25) `PWR_CHK(24) `PWR_CHK(23) `PWR_CHK(22) `PWR_CHK(21) `PWR_CHK(20)
    `PWR_CHK(19) `PWR_CHK(18) `PWR_CHK(17) `PWR_CHK(16) `PWR_CHK(15) `PWR_CHK(14) `PWR_CHK(13) `PWR_CHK(12) `PWR_CHK(11) `PWR_CHK(10)
    `PWR_CHK(09) `PWR_CHK(08) `PWR_CHK(07) `PWR_CHK(06) `PWR_CHK(05) `PWR_CHK(04) `PWR_CHK(03) `PWR_CHK(02) `PWR_CHK(01) `PWR_CHK(00)
MAX_PWR;


always @(posedge clk) begin
  if (rst) begin
    s0_valid           <= 0;
    s1_valid           <= 0;
    s2_valid           <= 0;
    s3_valid           <= 0;

    pwr_rsum_exp_valid <= 0;
    logmnt_exp1_valid  <= 0;
    pwr_corr_frac_valid<= 0;
  end else begin
    //Stage 0..3. Mul & Sum
    s0_re_a       <= i_re[IN_WIDTH-1:IN_WIDTH-A_WIDTH];
    s0_re_b       <= i_re[IN_WIDTH-1:IN_WIDTH-B_WIDTH];
    s0_im_a       <= i_im[IN_WIDTH-1:IN_WIDTH-A_WIDTH];
    s0_im_b       <= i_im[IN_WIDTH-1:IN_WIDTH-B_WIDTH];
    s0_valid      <= i_valid;
    s0_last       <= i_last;
    s0_user       <= i_user;

    s1_re_square  <= $signed(s0_re_a) * $signed(s0_re_b);
    s1_im_a       <= s0_im_a;
    s1_im_b       <= s0_im_b;
    s1_valid      <= s0_valid;
    s1_last       <= s0_last;
    s1_user       <= s0_user;

    s2_re_square  <= s1_re_square;
    s2_im_square  <= $signed(s1_im_a) * $signed(s1_im_b);
    s2_valid      <= s1_valid;
    s2_last       <= s1_last;
    s2_user       <= s1_user;

    s3_sum_square <= s2_re_square + s2_im_square;
    s3_valid      <= s2_valid;
    s3_last       <= s2_last;
    s3_user       <= s2_user;

    //Stage 4. Sum + Norm
    pwr_exp            <= exp_sum;
    pwr_rsum           <= pwr_sum;
    pwr_rsum_exp_valid <= s3_valid;
    s4_last            <= s3_last;
    s4_user            <= s3_user;

    // Stage 5 shift
    pwr_mnt            <= pwr_normcut;
    pwr_exp1           <= pwr_exp;
    logmnt_exp1_valid  <= pwr_rsum_exp_valid;
    s5_last            <= s4_last;
    s5_user            <= s4_user;

    // Stage 6 correction
    pwr_fractional     <= {pwr_mnt, 2'b00} + ilogout;
    pwr_corr_exp       <= ~pwr_exp1;
    pwr_corr_frac_valid<= logmnt_exp1_valid;
    s6_last            <= s5_last;
    s6_user            <= s5_user;
  end
end

endmodule
