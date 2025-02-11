// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2025 Wavelet Lab
//
//
module reconf_dsp_elem #(
	parameter CHANS = 1,
	parameter WIDTH = 16,
	parameter OWIDTH = 16,
	parameter WIDTH_B = 18,
	parameter WIDTH_C = 48,
	parameter WIDTH_P = 48,
	parameter FIFO_PA_BITS = 5,
	parameter FIFO_PD_BITS = 5,
	parameter FIFO_PF_BITS = 5,
	parameter DSP_C_P = 1,
	parameter CMD_WIDTH = 3,
	parameter FIFO_PIPELINE = 1,
	parameter C_PIPELINE = 1,
	parameter DSP_ID = 1,
	parameter EXT_STALL = 0,
	parameter BACKPRESSURE = 0,
	parameter LAST = 0
)(
	input rst,
	input clk,

	// Data stream input
	input [WIDTH * CHANS - 1:0]        din_data,
	input                              din_valid,
	output                             din_ready,

	// Data stream output
	output  [OWIDTH * CHANS - 1:0]     dout_data,
	output                             dout_valid,
	input                              dout_ready,

	// Const / Data stream
	input [WIDTH_B * CHANS - 1:0]      dbin_data,
	//input                            dbin_valid

	// PCIN / PCOUT cascase
	input [WIDTH_P * CHANS - 1:0]  pcin_data,
	output [WIDTH_P * CHANS - 1:0] pcout_data,

	// Execution / control / configuration
	input [FIFO_PA_BITS-1:0]       exe_faa,  // Address from PA stack register file
	input [FIFO_PD_BITS-1:0]       exe_fad,  // Address from PD stack register file
	input [FIFO_PF_BITS-1:0]       exe_fac,  // Address from PC stack register file
	input                          exe_pa_l, // Load din_data into PA stack register
	input                          exe_pd_l, // Load din_data into PD stack register
	input                          exe_pc_l, // Load din_data into PC stack register
	input                          exe_pi_r, // Signal to consume ( put ready )
	input                          exe_pp_l, // Push data signal to the next stage
	input [CMD_WIDTH-1:0]          exe_cmd,
	input                          exe_cfg_omux,

	input                          exe_cin_ready,
	output                         exe_cout_ready,

    output                         exe_ready, // 1 -- means command accepted and PC need to be updated

	output                         alarm_ovf
);

// Overall pipeline stages
localparam C_PIPELINE_NOE2 = 0;
localparam STAGES = 5;

//////////////////////////////
// P = 0    + (A + D) * B
// P = C    + (A + D) * B
// P = P    + (A + D) * B
// P = Pcin + (A + D) * B
// P = P + PCIN
// P = 0 + A*B
// P = C + PCIN
// P = P + C
//////////////////////////////

localparam [14:0]
	DSP_OP_ADpBx   = 15'b0000101_0000_010,
	DSP_OP_CADpBxp = 15'b0110101_0000_010,
	DSP_OP_PADpBxp = 15'b0100101_0000_010,
	DSP_OP_IADpBxp = 15'b0010101_0000_010,
	DSP_OP_PIp     = 15'b0010010_0000_001,
	DSP_OP_ABx     = 15'b0000101_0000_000,
	DSP_OP_CIp     = 15'b0011100_0000_001,
	DSP_OP_PCp     = 15'b0001110_0000_001;

localparam CMD_WIDTH_EX = 3;

localparam [15 * (1 << CMD_WIDTH_EX) - 1 : 0] DSP_ALL_CMDS = { DSP_OP_PCp, DSP_OP_CIp, DSP_OP_ABx, DSP_OP_PIp, DSP_OP_IADpBxp, DSP_OP_PADpBxp, DSP_OP_CADpBxp, DSP_OP_ADpBx };

localparam [(1 << CMD_WIDTH_EX) - 1 : 0]      DSP_LOAD_C   = {       1'b1,       1'b1,       1'b0,       1'b0,           1'b0,           1'b0,           1'b1,        1'b0  };
localparam [(1 << CMD_WIDTH_EX) - 1 : 0]      DSP_LOAD_AD  = {       1'b0,       1'b0,       1'b1,       1'b0,           1'b1,           1'b1,           1'b1,        1'b1  };


// Wait for data to be availble
wire wait_data_n  = exe_ready;
wire pa_we        = exe_pa_l;
wire pd_we        = exe_pd_l;
wire pf_we        = exe_pc_l;

//
// pa
// pd
// dsp_cmd
//

// Port A, C, D (FIFO)
wire [WIDTH * CHANS - 1:0]   pa_data_q;
wire [WIDTH * CHANS - 1:0]   pd_data_q;
wire [WIDTH * CHANS - 1:0]   pf_data_q;
wire [WIDTH * CHANS - 1:0]   pa_data_f;
wire [WIDTH * CHANS - 1:0]   pd_data_f;
wire [WIDTH * CHANS - 1:0]   pf_data_f;
wire [WIDTH * CHANS - 1:0]   pa_data = (FIFO_PIPELINE) ? pa_data_q : pa_data_f;
wire [WIDTH * CHANS - 1:0]   pd_data = (FIFO_PIPELINE) ? pd_data_q : pd_data_f;
wire [WIDTH * CHANS - 1:0]   pf_data = (FIFO_PIPELINE) ? pf_data_q : pf_data_f;

// DSP result out (conct.)
wire [WIDTH * CHANS - 1:0]   o_data;

localparam FIFO_PA = 1 << FIFO_PA_BITS;
localparam FIFO_PD = 1 << FIFO_PD_BITS;
localparam FIFO_PF = 1 << FIFO_PF_BITS;

wire [FIFO_PA_BITS-1:0] pa_addr = exe_faa;
wire [FIFO_PD_BITS-1:0] pd_addr = exe_fad;
wire [FIFO_PF_BITS-1:0] pf_addr = exe_fac;


wire [CMD_WIDTH_EX - 1:0] exe_cmd_ex = (CMD_WIDTH >= CMD_WIDTH_EX) ? exe_cmd : (exe_cmd != 3 || ~pf_addr[0] ? {1'b0, exe_cmd } : 4);
reg [CMD_WIDTH_EX - 1:0]  dsp_cmd_s0;
always @(posedge clk) begin
  if (wait_data_n) begin
  	dsp_cmd_s0 <= exe_cmd_ex;
  end
end
wire [CMD_WIDTH_EX - 1:0]   dsp_cmd_raw = (FIFO_PIPELINE) ? dsp_cmd_s0 : exe_cmd_ex;
wire [14:0]                 dsp_cmd;

genvar i;
generate
for (i = 0; i < 15; i = i + 1) begin
	assign dsp_cmd[i] = DSP_ALL_CMDS[15 * dsp_cmd_raw + i];
end
endgenerate

localparam LOAD_NCOND = 1'b0;
wire ce_pad = wait_data_n && (LOAD_NCOND || DSP_LOAD_AD[exe_cmd_ex]);
wire ca_pac = wait_data_n && (LOAD_NCOND || DSP_LOAD_C[exe_cmd_ex]);

srl_ra #(.WIDTH(CHANS*WIDTH), .DEEP(FIFO_PA)) in_pa (
	.clk(clk), .we(wait_data_n && pa_we), .data_i(din_data),
	.addr_i(pa_addr), .data_o(pa_data_f), .ce(ce_pad), .rstq(1'b0), .dataq_o(pa_data_q), .datasrl_o()
);

srl_ra #(.WIDTH(CHANS*WIDTH), .DEEP(FIFO_PD)) in_pd (
	.clk(clk), .we(wait_data_n && pd_we), .data_i(din_data),
	.addr_i(pd_addr), .data_o(pd_data_f), .ce(ce_pad), .rstq(1'b0), .dataq_o(pd_data_q), .datasrl_o()
);


wire [WIDTH * CHANS - 1:0]   pc_data;
wire [4 * CHANS - 1:0]       dsp_p_svalid;
generate
if (DSP_C_P) begin
srl_ra #(.WIDTH(CHANS*WIDTH), .DEEP(FIFO_PF)) in_pf (
	.clk(clk), .we(wait_data_n && pf_we), .data_i(din_data),
	.addr_i(pf_addr), .data_o(pf_data_f), .ce(ca_pac), .rstq(1'b0), .dataq_o(pf_data_q), .datasrl_o()
);

reg [WIDTH * CHANS - 1:0]   pc_data_s0;
always @(posedge clk) begin
  if (wait_data_n) begin
  	pc_data_s0 <= pf_data;
  end
end
wire [WIDTH * CHANS - 1:0]  pc_data_l = (C_PIPELINE) ? pc_data_s0 : pf_data;

reg [WIDTH * CHANS - 1:0]   pc_data_d0;
reg [WIDTH * CHANS - 1:0]   pc_data_d1;
always @(posedge clk) begin
  if (wait_data_n) begin
	if (dsp_p_svalid[0])
  		pc_data_d0 <= pf_data;
  	if (dsp_p_svalid[1])
  		pc_data_d1 <= pc_data_d0;
  end
end
assign pc_data = (C_PIPELINE_NOE2) ? pc_data_l : pc_data_d1;
end else begin
    assign pc_data      = {(WIDTH * CHANS){1'b1}};
end
endgenerate

reg [WIDTH_B * CHANS - 1:0]   pb_data_s0;
always @(posedge clk) begin
  if (wait_data_n) begin
  	pb_data_s0 <= dbin_data;
  end
end
wire [WIDTH_B * CHANS - 1:0]   pb_data = (FIFO_PIPELINE) ? pb_data_s0 : dbin_data;


// Propagation of data output valid signal over the pipeline
reg  data_valid_strobe;
always @(posedge clk) begin
  if (rst) begin
  	data_valid_strobe <= 0;
  end else if (wait_data_n) begin
  	data_valid_strobe <= exe_pp_l;
  end
end

wire dsp_tag =  (FIFO_PIPELINE) ? data_valid_strobe : exe_pp_l;

localparam DSP_WIDTH_PA = 30;
localparam DSP_WIDTH_PD = 25;
localparam DSP_WIDTH_PB = 18;
localparam DSP_WIDTH_PC = 48;
localparam DSP_WIDTH_PP = 48;

wire [CHANS-1:0] dsp_ovalid_data;
wire [CHANS-1:0] dsp_out_tag;
wire [CHANS-1:0] dsp_ovt;

genvar ch;
generate
for (ch = 0; ch < CHANS; ch = ch + 1) begin: gen_dsp

wire [DSP_WIDTH_PA-1:0] dsp_a = { {(DSP_WIDTH_PA - WIDTH){pa_data[WIDTH * (ch + 1) - 1]}}, pa_data[WIDTH * (ch + 1) - 1:WIDTH * ch] };
wire [DSP_WIDTH_PD-1:0] dsp_d = { {(DSP_WIDTH_PD - WIDTH){pd_data[WIDTH * (ch + 1) - 1]}}, pd_data[WIDTH * (ch + 1) - 1:WIDTH * ch] };
wire [DSP_WIDTH_PC-1:0] dsp_c = (DSP_C_P) ? { {(DSP_WIDTH_PC - WIDTH - WIDTH_B + 2){pc_data[WIDTH * (ch + 1) - 1]}}, pc_data[WIDTH * (ch + 1) - 1:WIDTH * ch], {(WIDTH_B-2){1'b0}} } : 48'hxxxx_xxxx_xxxx;

wire [DSP_WIDTH_PB-1:0] dsp_b = { {(DSP_WIDTH_PB - WIDTH_B + 1){pb_data[WIDTH_B * (ch + 1) - 1]}}, pb_data[WIDTH_B * (ch + 1) - 2:WIDTH_B * ch] };
wire [DSP_WIDTH_PP-1:0] dsp_out_data;

// DSP function pipeline:
// 0:
//   tmp_a  <= pa
//   tmp_d  <= pd
//   tmp_b0 <= pb
// 1:
//   tmp_ad <= pa + pd
//   tmp_b1 <= tmp_b0
// 2:
//   tmp_c  <= pc
//   tmp_m  <= tmp_ad * tmp_b1
// 3:
//   tmp_p  <= pcin/c/m


dsp48e1_pipeline #(.ID(DSP_ID), .NO_STALL(!BACKPRESSURE)) dsp(
	.clk(clk),
	.rst(rst),

	.s_stall_n(wait_data_n),

	.s_adb_data({dsp_a, dsp_d, dsp_b}),
	.s_adb_valid(wait_data_n),
	.s_adb_cmd(dsp_cmd[14:0]),
	.s_adb_tag(dsp_tag),

	// Async C update
	.s_c_data(dsp_c),
	.s_c_valid((DSP_C_P) ? dsp_p_svalid[2] : 1'b0),

	.m_p_data(dsp_out_data),
	.m_p_valid(dsp_ovalid_data[ch]),
	.m_p_tag(dsp_out_tag[ch]),
	.m_p_tv(dsp_ovt[ch]),

	.m_p_svalid(dsp_p_svalid[4 * (ch + 1) - 1 : 4 * ch]),

	// Cascase
	.dspcasc_p_o( pcout_data[WIDTH_P * (ch + 1) - 1 : WIDTH_P * ch] ),
	.dspcasc_p_i( pcin_data[WIDTH_P * (ch + 1) - 1 : WIDTH_P * ch] )
);

// signed data, MSB
// FIXME correct OWIDTH ------------------------------------------------@
assign o_data[ OWIDTH * (ch + 1) - 1 : OWIDTH * ch ] = dsp_out_data[ OWIDTH - 1 + WIDTH_B - 1 : WIDTH_B - 1];

end
endgenerate

wire data_oready;
wire data_ovalid   = (exe_cfg_omux) ? din_valid && exe_pi_r : dsp_ovt[0];
wire cmd_in_ready  = !exe_pi_r || din_valid;           // CMD isn't blocked by lacking of incoming data
assign exe_ready   = cmd_in_ready && exe_cout_ready;
assign din_ready   = exe_pi_r && exe_cout_ready;

generate
if (BACKPRESSURE) begin
    wire cmd_out_ready    = (!data_ovalid || data_oready);   // CMD isn't blocked by output backpressure
    assign exe_cout_ready = (exe_cfg_omux && !LAST) ? exe_cin_ready : cmd_out_ready;
    assign alarm_ovf      = 1'b0;

    `ifdef SYM
    always @(posedge clk) begin
        if (!rst) begin
            if (exe_cout_ready && data_ovalid && !data_oready) begin
                $display("STAGE %d BUFFER VIOLATION!!!!", DSP_ID);
                $finish;
            end
        end
    end
    `endif
end else begin
    reg alarm_ovf_r;
    assign exe_cout_ready = 1'b1;
    assign alarm_ovf = alarm_ovf_r;
    always @(posedge clk) begin
        if (rst) begin
            alarm_ovf_r  <= 1'b0;
        end else begin
            if (data_ovalid && dout_valid && ~dout_ready) begin
                alarm_ovf_r  <= 1'b1;
            end
        end
    end
end
endgenerate


axis_opt_pipeline #(.WIDTH(OWIDTH * CHANS), .PIPELINE(1'b1), .REG_READY(BACKPRESSURE)) out_reg (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata((exe_cfg_omux) ? din_data : o_data),
  .s_rx_tvalid(data_ovalid && exe_cout_ready),
  .s_rx_tready(data_oready),

  .m_tx_tdata(dout_data),
  .m_tx_tvalid(dout_valid),
  .m_tx_tready(dout_ready)
);

endmodule

