// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module reconf_dsp_fir #(
	parameter CHANS = 2,
	parameter IN_WIDTH = 16,
	parameter OUT_WIDTH = 16,
	parameter CFG_WIDTH = 32,
	parameter STAGES    = 8,
	parameter SEQUENCER_DEEPBITS = 5
)(
	input rst,
	input clk,

	input                                             in_valid,
	input [IN_WIDTH * CHANS - 1:0]                    in_data,
	output                                            in_ready,

	output                                            out_valid,
	output [OUT_WIDTH * CHANS - 1:0]                  out_data,
	input                                             out_ready,

	// input                                             cfg_rst,
	input                                             cfg_valid,
	input [CFG_WIDTH-1:0]                             cfg_data,

	output [STAGES-1:0]                               alarm_ovf
);

localparam CMD_B_W = 18;
localparam CMD_PAD_W = 5;
localparam CMD_PC_W = 4;
localparam CMD_FIFO_CMDS_W = 4;
localparam CMD_OMUX_W = 1;
localparam CMD_DSPC_W = 2;
localparam CMD_RST_W = 1;

localparam CMD_B_OFF         = 0;
localparam CMD_PA_OFF        = CMD_B_OFF    + CMD_B_W;
localparam CMD_PD_OFF        = CMD_PA_OFF   + CMD_PAD_W;
localparam CMD_PC_OFF        = CMD_PD_OFF   + CMD_PAD_W;
localparam CMD_FIFO_CMDS_OFF = CMD_PC_OFF   + CMD_PC_W;
localparam CMD_OMUX_OFF      = CMD_FIFO_CMDS_OFF + CMD_FIFO_CMDS_W;
localparam CMD_DSPC_OFF      = CMD_OMUX_OFF + CMD_OMUX_W;
localparam CMD_RST_OFF       = CMD_DSPC_OFF + CMD_DSPC_W;

localparam CMD_SLICE_WIDTH   = CMD_RST_OFF  + CMD_RST_W;

localparam [127:0] CMD_DUMMY_NO_RST  =                      (1 << CMD_OMUX_OFF) | (1 << (CMD_FIFO_CMDS_OFF + 3));
localparam [127:0] CMD_DUMMY         = (1 << CMD_RST_OFF) | (1 << CMD_OMUX_OFF) | (1 << (CMD_FIFO_CMDS_OFF + 3));

localparam WIDTH_P = 48;

wire  [WIDTH_P * CHANS * (STAGES + 1) - 1:0]  pc_cascade;

wire  [IN_WIDTH * CHANS * (STAGES + 1) - 1:0] cascade_data;
wire  [STAGES:0]                              csscade_data_v;
wire  [STAGES:0]                              csscade_data_r;

assign csscade_data_v[0] = in_valid;
assign in_ready = csscade_data_r[0];
assign cascade_data[IN_WIDTH * CHANS - 1:0] = in_data;

assign out_valid = csscade_data_v[STAGES];
assign csscade_data_r[STAGES] = out_ready;
assign out_data = cascade_data[IN_WIDTH * CHANS * (STAGES + 1) - 1:IN_WIDTH * CHANS * STAGES];


localparam SLICE_CFG_W = CFG_WIDTH / STAGES;

genvar i;
generate
for (i = 0; i < STAGES; i = i + 1) begin: dsp_stage
	reg [SEQUENCER_DEEPBITS-1:0]    seq_pc_reg;
	wire                            pipe_stall_n;

	wire [CMD_SLICE_WIDTH-1:0]      seq_ucode_s;
	wire [CMD_SLICE_WIDTH-1:0]      seq_ucode;

	wire [CMD_B_W - 1:0]            seq_cmd_b   = seq_ucode[CMD_B_W - 1 + CMD_B_OFF:CMD_B_OFF];
	wire [CMD_PAD_W - 1:0]          seq_cmd_pa  = seq_ucode[CMD_PAD_W - 1 + CMD_PA_OFF:CMD_PA_OFF];
	wire [CMD_PAD_W - 1:0]          seq_cmd_pd  = seq_ucode[CMD_PAD_W - 1 + CMD_PD_OFF:CMD_PD_OFF];
	wire [CMD_PC_W - 1:0]           seq_cmd_pc  = seq_ucode[CMD_PC_W - 1 + CMD_PC_OFF:CMD_PC_OFF];

	wire [CMD_DSPC_W - 1:0]         seq_cmd_dsp   = seq_ucode[CMD_DSPC_W - 1 + CMD_DSPC_OFF:CMD_DSPC_OFF];
	wire                            seq_cmd_dspex = (seq_cmd_dsp == 2'b00) && (seq_cmd_pc[1:0] == 2'b11); // ISA compression to fit PIp instruction
	wire                            seq_omux      = seq_ucode[CMD_OMUX_OFF];

	wire                            seq_fifo_padl = seq_ucode[CMD_FIFO_CMDS_OFF + 0];
	wire                            seq_fifo_pcl  = seq_ucode[CMD_FIFO_CMDS_OFF + 1];
	wire                            seq_fifo_ppl  = seq_ucode[CMD_FIFO_CMDS_OFF + 2];
	wire                            seq_fifo_pir  = seq_ucode[CMD_FIFO_CMDS_OFF + 3];

	wire [SLICE_CFG_W-1:0]          cfg_s_data = cfg_data[SLICE_CFG_W * (i + 1) - 1:SLICE_CFG_W * i];

	wire [IN_WIDTH * CHANS - 1:0]   pdata_in   = cascade_data[IN_WIDTH * CHANS * (i + 1) - 1:IN_WIDTH * CHANS * i];
	wire [IN_WIDTH * CHANS - 1:0]   pdata_out;

	assign cascade_data[IN_WIDTH * CHANS * (i + 2) - 1:IN_WIDTH * CHANS * (i + 1)] = pdata_out;

	srl_ra #(
		.WIDTH(CMD_SLICE_WIDTH), .IN_WIDTH(SLICE_CFG_W), .DEEP(1 << SEQUENCER_DEEPBITS),
		.INIT_POS1(CMD_DUMMY),
		.INIT_POS0(CMD_DUMMY_NO_RST),
		.INIT_REG(CMD_DUMMY),
		.RST_REG(CMD_DUMMY)
	) sequencer_fifo(
	 	.clk(clk), .we(cfg_valid), .data_i(cfg_s_data), .addr_i(seq_pc_reg), .data_o(seq_ucode_s), .ce(pipe_stall_n), .rstq(rst /*cfg_rst*/), .dataq_o(seq_ucode)
	);

	assign pipe_stall_n = !seq_fifo_pir || csscade_data_v[i];

	always @(posedge clk) begin
		if (/*cfg_rst ||*/ rst) begin
			seq_pc_reg <= 0;
		end else begin
			if (pipe_stall_n) begin
				seq_pc_reg <= seq_pc_reg + 1'b1;
				if (seq_ucode_s[CMD_RST_OFF])
					seq_pc_reg <= 1'b0;
			end
		end
	end

	reconf_dsp_elem #(
		.CHANS(CHANS),
		.WIDTH(IN_WIDTH),
		.OWIDTH(IN_WIDTH),
		.FIFO_PA_BITS(CMD_PAD_W),
		.FIFO_PD_BITS(CMD_PAD_W),
		.FIFO_PF_BITS(CMD_PC_W),
		.CMD_WIDTH(3),
		.C_PIPELINE(1), // FIXME: !!!!
		.DSP_ID(i)
	) element (
		.rst(rst /* cfg_rst */),
		.clk(clk),

		// Data stream input
		.din_data(pdata_in),
		.din_valid(csscade_data_v[i]),
		.din_ready(csscade_data_r[i]),

		// Data stream output
		.dout_data(pdata_out),
		.dout_valid(csscade_data_v[i + 1]),
		.dout_ready(csscade_data_r[i + 1]),

		// Const / Data stream
		.dbin_data({CHANS{seq_cmd_b}}),

		// PCIN / PCOUT cascase
		.pcin_data(pc_cascade[WIDTH_P * CHANS * (i + 1) - 1:WIDTH_P * CHANS * i]),
		.pcout_data(pc_cascade[WIDTH_P * CHANS * (i + 2) - 1:WIDTH_P * CHANS * (i + 1)]),

		// Execution / control / configuration
		.exe_faa(seq_cmd_pa),
		.exe_fad(seq_cmd_pd),
		.exe_fac(seq_cmd_pc),
		.exe_pa_l(seq_fifo_padl),
		.exe_pd_l(seq_fifo_padl),
		.exe_pc_l(seq_fifo_pcl),
		.exe_pi_r(seq_fifo_pir),
		.exe_pp_l(seq_fifo_ppl), // Push data signal to the next stage
		.exe_cmd({ seq_cmd_dspex, seq_cmd_dsp }),
		.exe_cfg_omux(seq_omux),

		.alarm_ovf(alarm_ovf[i])
	);

end

endgenerate




endmodule

