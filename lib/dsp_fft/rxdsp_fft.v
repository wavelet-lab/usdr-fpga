module rxdsp_fft #(
  parameter BUFFER_SIZE_ADDR = 16,
  parameter TIME_BITS = 32,
  parameter DATA_WIDTH = 12,
  parameter PIPELINED = 0
)(
  input                         clk,
  input                         reset,

  input [BUFFER_SIZE_ADDR-1:0]  cfg_bsz,
  input                         dspcmd_valid,
  output                        dspcmd_ready,
  input [31:0]                  dspcmd_data,

  input [DATA_WIDTH-1:0]        in_ai,
  input [DATA_WIDTH-1:0]        in_aq,
  input [DATA_WIDTH-1:0]        in_bi,
  input [DATA_WIDTH-1:0]        in_bq,
  input                         in_valid,
  output                        in_ready,
  input                         in_last,
  input [TIME_BITS-1:0]         in_time,

  output [63:0]                 dsp_data,
  output                        dsp_valid,
  output                        dsp_last,
  output [7:0]                  dsp_keep,
  output [BUFFER_SIZE_ADDR-1:3] dsp_waddr
);


wire [15:0] dsp0_ai    = { in_ai };
wire [15:0] dsp0_aq    = { in_aq };
wire [15:0] dsp0_bi    = { in_bi };
wire [15:0] dsp0_bq    = { in_bq };
wire        dsp0_valid = in_valid;
wire        dsp0_last  = in_last;

wire [15:0] dsp1_ai;
wire [15:0] dsp1_aq;
wire [15:0] dsp1_bi;
wire [15:0] dsp1_bq;
wire        dsp1_valid;
wire        dsp1_last;


wire        in_sdr_ready;
assign      in_ready = in_sdr_ready;



(* rom_style = "distributed" *)
reg [14:0] windowrom [2**9-1:0];
initial $readmemh("windowrom_d512.data", windowrom);

reg fft_wnd_bypass = 0;
always @(posedge clk) begin
  if (reset) begin
    fft_wnd_bypass <= 0;
  end else if (dspcmd_valid) begin
    fft_wnd_bypass <= dspcmd_data[27];
  end
end


reg               s0_valid;
reg [8:0]         s0_romidx;
reg [14:0]        s0_window_data;
reg [15:0]        s0_data_aq;
reg [15:0]        s0_data_ai;
reg               s0_last;

localparam WND_PREC = 15+15+1;
localparam FFT_PREC = 14;

reg                  s1_valid;
reg [WND_PREC-1:0]   s1_data_aq;
reg [WND_PREC-1:0]   s1_data_ai;
reg                  s1_last;

reg                  s2_valid;
reg [WND_PREC-1:0]   s2_data_aq;
reg [WND_PREC-1:0]   s2_data_ai;
reg                  s2_last;

always @(posedge clk) begin
  if (reset) begin
    s0_valid  <= 0;
    s1_valid  <= 0;
    s2_valid  <= 0;

    s0_romidx <= 0;
  end else begin
    if (s0_valid && in_sdr_ready) begin
        s0_valid <= 1'b0;
    end

    // Stage 0
    if (dsp0_valid && (!s0_valid || in_ready)) begin
        s0_romidx <= s0_romidx + 1'b1;
        if (dsp0_last) begin
            s0_romidx <= 0;
        end

        s0_window_data <= (fft_wnd_bypass) ? 15'h7fff : windowrom[s0_romidx];
        s0_data_aq     <= dsp0_aq[15:0];
        s0_data_ai     <= dsp0_ai[15:0];
        s0_valid       <= dsp0_valid;
        s0_last        <= dsp0_last || (s0_romidx == 511);
    end

    if (s1_valid && in_sdr_ready) begin
        s1_valid <= 1'b0;
    end

    // Stage 1
    if (s0_valid && (!s1_valid || in_ready)) begin
        s1_valid       <= s0_valid;
        s1_last        <= s0_last;
        s1_data_aq     <= $signed(s0_data_aq) * $signed({1'b0, s0_window_data});
        s1_data_ai     <= $signed(s0_data_ai) * $signed({1'b0, s0_window_data}); // (1+15) * (1+15)  = 1+31
    end

    if (s2_valid && in_sdr_ready) begin
        s2_valid <= 1'b0;
        s2_last  <= 1'b0;
    end

    if (s1_valid && (!s2_valid || in_ready)) begin
        s2_valid       <= s1_valid;
        s2_last        <= s1_last;
        s2_data_aq     <= s1_data_aq;
        s2_data_ai     <= s1_data_ai;
    end

  end
end

wire [23:0] fft_data_re;
wire [23:0] fft_data_im;
wire [8:0]  fft_xk_index;

`define WITH_FFT
`ifdef WITH_FFT
xfft_0 xfft_0(
    .aclk(clk),
    .aresetn(~reset),
    .s_axis_config_tdata(24'b0),
    .s_axis_config_tvalid(1'b0),
    .s_axis_config_tready(),

    .s_axis_data_tdata({2'h0, s2_data_aq[WND_PREC-1:WND_PREC-FFT_PREC], 2'h0, s2_data_ai[WND_PREC-1:WND_PREC-FFT_PREC]}),
    .s_axis_data_tvalid(s2_valid),
    .s_axis_data_tlast(s2_last),
    .s_axis_data_tready(in_sdr_ready),

    .m_axis_data_tdata({fft_data_im, fft_data_re}),
    .m_axis_data_tvalid(dsp1_valid),
    .m_axis_data_tready(1'b1),
    .m_axis_data_tlast(dsp1_last),
    .m_axis_data_tuser(fft_xk_index)
);

`else
assign in_sdr_ready = ~reset;
assign fft_xk_index = 0;
assign dsp1_valid  = s2_valid;
assign dsp1_last   = s2_last;
assign fft_data_im = s2_data_aq[WND_PREC-1:WND_PREC-FFT_PREC];
assign fft_data_re = s2_data_ai[WND_PREC-1:WND_PREC-FFT_PREC];
`endif

wire [15:0] log_power;
wire        dsp2_valid;
wire        dsp2_last;
wire [8:0]  dsp2_xk_index;


fft_log_power_calc #(.IN_WIDTH(24), .IN_USER_WIDTH(9)) fft_log_power_calc(
  .clk(clk),
  .rst(reset),
  .i_re(fft_data_re[23:0]),
  .i_im(fft_data_im[23:0]),
  .i_valid(dsp1_valid),
  .i_last(dsp1_last),
  .i_user(fft_xk_index),
  .o_power(log_power),
  .o_valid(dsp2_valid),
  .o_last(dsp2_last),
  .o_user(dsp2_xk_index)
);


reg [7:0]  dsp_s3_pktoff;
reg [8:0]  dsp_s3_xk_index;
reg        dsp_s3_valid;
reg        dsp_s3_last;
reg [15:0] dsp_s3_data;

always @(posedge clk) begin
    if (reset) begin
        dsp_s3_valid  <= 1'b0;
        dsp_s3_pktoff <= 0;
        dsp_s3_last   <= 1'b0;
    end else begin
        dsp_s3_last   <= 1'b0;

        if (dsp2_valid) begin
            if (dsp2_last) begin
                dsp_s3_pktoff <= dsp_s3_pktoff + 1'b1;

                if (dsp_s3_pktoff == cfg_bsz[BUFFER_SIZE_ADDR-1:9]) begin
                    dsp_s3_pktoff <= 0;
                    dsp_s3_last   <= 1'b1;
                end
            end
        end


        dsp_s3_valid    <= dsp2_valid;
        dsp_s3_data     <= log_power;
        dsp_s3_xk_index <= dsp2_xk_index ^ 9'h100;

    end
end

assign dsp_data = { dsp_s3_data, dsp_s3_data, dsp_s3_data, dsp_s3_data };
assign dsp_keep =
    (dsp_s3_xk_index[1:0] == 2'b00) ? 8'b00_00_00_11 :
    (dsp_s3_xk_index[1:0] == 2'b01) ? 8'b00_00_11_00 :
    (dsp_s3_xk_index[1:0] == 2'b10) ? 8'b00_11_00_00 : 8'b11_00_00_00;

assign dsp_waddr = { dsp_s3_pktoff, dsp_s3_xk_index[8:2] };
assign dsp_last  = dsp_s3_last;
assign dsp_valid = dsp_s3_valid;


assign dspcmd_ready = 1'b1;

endmodule

