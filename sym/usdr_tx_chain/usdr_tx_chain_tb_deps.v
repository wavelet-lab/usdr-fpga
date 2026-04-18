`timescale 1ns / 1ps
`default_nettype none

module cordic_0(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire        s_axis_phase_tvalid,
    input  wire [15:0] s_axis_phase_tdata,
    input  wire        s_axis_cartesian_tvalid,
    input  wire [31:0] s_axis_cartesian_tdata,
    output wire        m_axis_dout_tvalid,
    output wire [31:0] m_axis_dout_tdata
);

// Behavioral model for hw/usdr/ip/cordic_0/cordic_0.xci:
// - Rotate
// - Parallel architecture, maximum pipelining
// - SignedFraction X/Y format, 16-bit in/out packed as {Q, I}
// - Scaled_Radians phase format, 16-bit
// - Coarse rotation enabled
// - Compensation scaling enabled
// - Truncate output mode
//
// Per AMD/Xilinx PG105, a parallel CORDIC has a basic latency of N cycles for N-bit output
// width. This instance uses 16-bit outputs, so model 16 cycles of latency.
localparam integer CORDIC_WIDTH       = 16;
localparam integer CORDIC_LATENCY     = 16;
localparam integer XY_FRAC_BITS       = CORDIC_WIDTH - 2;
localparam integer PHASE_FRAC_BITS    = CORDIC_WIDTH - 3;
localparam real    CORDIC_PI          = 3.14159265358979323846;
localparam real    CORDIC_TWO_PI      = 6.28318530717958647692;
localparam real    XY_SCALE_REAL      = 16384.0; // 1QN signed fraction for 16-bit X/Y
localparam real    PHASE_SCALE_REAL   = 8192.0;  // 2QN scaled-radians for 16-bit phase

reg  [31:0] data_pipe  [0:CORDIC_LATENCY-1];
reg  [CORDIC_LATENCY-1:0] valid_pipe = {CORDIC_LATENCY{1'b0}};
integer pipe_idx;

function automatic real wrap_scaled_phase;
    input real raw_phase;
    real wrapped;
    begin
        wrapped = raw_phase;
        while (wrapped > CORDIC_PI)
            wrapped = wrapped - CORDIC_TWO_PI;
        while (wrapped <= -CORDIC_PI)
            wrapped = wrapped + CORDIC_TWO_PI;
        wrap_scaled_phase = wrapped;
    end
endfunction

function automatic real phase_word_to_radians;
    input [15:0] phase_word;
    integer phase_signed;
    begin
        phase_signed = $signed(phase_word);
        phase_word_to_radians = wrap_scaled_phase((phase_signed * CORDIC_PI) / PHASE_SCALE_REAL);
    end
endfunction

function automatic real signed_fraction_to_real;
    input [15:0] sample_word;
    integer sample_signed;
    begin
        sample_signed = $signed(sample_word);
        signed_fraction_to_real = sample_signed / XY_SCALE_REAL;
    end
endfunction

function automatic [15:0] real_to_signed_fraction_trunc;
    input real sample_real;
    real clipped;
    integer quantized;
    begin
        if (sample_real > ((32767.0) / XY_SCALE_REAL))
            clipped = (32767.0) / XY_SCALE_REAL;
        else if (sample_real < ((-32768.0) / XY_SCALE_REAL))
            clipped = (-32768.0) / XY_SCALE_REAL;
        else
            clipped = sample_real;

        quantized = $rtoi(clipped * XY_SCALE_REAL);
        real_to_signed_fraction_trunc = quantized[15:0];
    end
endfunction

function automatic [31:0] rotate_cartesian_sample;
    input [15:0] phase_word;
    input [31:0] cartesian_word;
    real phase_radians;
    real x_real;
    real y_real;
    real x_rot;
    real y_rot;
    reg [15:0] x_word;
    reg [15:0] y_word;
    begin
        phase_radians = phase_word_to_radians(phase_word);
        x_real = signed_fraction_to_real(cartesian_word[15:0]);
        y_real = signed_fraction_to_real(cartesian_word[31:16]);

        x_rot = (x_real * $cos(phase_radians)) - (y_real * $sin(phase_radians));
        y_rot = (y_real * $cos(phase_radians)) + (x_real * $sin(phase_radians));

        x_word = real_to_signed_fraction_trunc(x_rot);
        y_word = real_to_signed_fraction_trunc(y_rot);
        rotate_cartesian_sample = {y_word, x_word};
    end
endfunction

always @(posedge aclk) begin
    if (!aresetn) begin
        valid_pipe <= {CORDIC_LATENCY{1'b0}};
        for (pipe_idx = 0; pipe_idx < CORDIC_LATENCY; pipe_idx = pipe_idx + 1) begin
            data_pipe[pipe_idx] <= 32'd0;
        end
    end else begin
        for (pipe_idx = CORDIC_LATENCY - 1; pipe_idx > 0; pipe_idx = pipe_idx - 1) begin
            data_pipe[pipe_idx] <= data_pipe[pipe_idx - 1];
        end

        valid_pipe[CORDIC_LATENCY-1:1] <= valid_pipe[CORDIC_LATENCY-2:0];
        valid_pipe[0] <= s_axis_phase_tvalid && s_axis_cartesian_tvalid;

        if (s_axis_phase_tvalid && s_axis_cartesian_tvalid)
            data_pipe[0] <= rotate_cartesian_sample(s_axis_phase_tdata, s_axis_cartesian_tdata);
        else
            data_pipe[0] <= 32'd0;
    end
end

assign m_axis_dout_tvalid = valid_pipe[CORDIC_LATENCY-1];
assign m_axis_dout_tdata  = data_pipe[CORDIC_LATENCY-1];

endmodule

module cordic_1(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire        s_axis_phase_tvalid,
    input  wire [16:0] s_axis_phase_tdata,
    input  wire        s_axis_cartesian_tvalid,
    input  wire [40:0] s_axis_cartesian_tdata,
    output wire        m_axis_dout_tvalid,
    output wire [40:0] m_axis_dout_tdata
);

// Behavioral model for the wider TX-path CORDIC used by usdr_tx_chain.
// For this TX-chain unit test, a pipelined passthrough is sufficient:
// the testbench separately validates the standalone cordic_0 math model, while
// the DUT-driven CORDIC path only needs stable non-X data propagation.
localparam integer CORDIC_WIDTH     = 17;
localparam integer CORDIC_LATENCY   = 17;

reg  [40:0] data_pipe  [0:CORDIC_LATENCY-1];
reg  [CORDIC_LATENCY-1:0] valid_pipe = {CORDIC_LATENCY{1'b0}};
reg                       valid_out_r = 1'b0;
integer pipe_idx;

always @(posedge aclk) begin
    if (!aresetn) begin
        valid_pipe <= {CORDIC_LATENCY{1'b0}};
        valid_out_r <= 1'b0;
        for (pipe_idx = 0; pipe_idx < CORDIC_LATENCY; pipe_idx = pipe_idx + 1) begin
            data_pipe[pipe_idx] <= 41'd0;
        end
    end else begin
        for (pipe_idx = CORDIC_LATENCY - 1; pipe_idx > 0; pipe_idx = pipe_idx - 1) begin
            data_pipe[pipe_idx] <= data_pipe[pipe_idx - 1];
        end

        valid_pipe[CORDIC_LATENCY-1:1] <= valid_pipe[CORDIC_LATENCY-2:0];
        valid_pipe[0] <= s_axis_phase_tvalid && s_axis_cartesian_tvalid;
        valid_out_r <= valid_pipe[CORDIC_LATENCY-1];

        if (s_axis_phase_tvalid && s_axis_cartesian_tvalid)
            data_pipe[0] <= s_axis_cartesian_tdata;
        else
            data_pipe[0] <= 41'd0;
    end
end

assign m_axis_dout_tvalid = valid_out_r;
assign m_axis_dout_tdata  = data_pipe[CORDIC_LATENCY-1];

endmodule

module ila_1(
    input wire clk,
    input wire        probe0,
    input wire [15:0] probe1,
    input wire [15:0] probe2,
    input wire        probe3,
    input wire        probe4
);

// Debug core stub for simulation: intentionally no behavior.

endmodule

module iq_corr #(
    parameter WIDTH     = 16,
    parameter CFG_WIDTH = 16,
    parameter OUT_WIDTH = 16
) (
    input wire                        clk,
    input wire                        rst,
    input wire [CFG_WIDTH * 2 - 1:0]  cfg_amp,
    input wire [CFG_WIDTH * 2 - 1:0]  cfg_tan,
    input wire [WIDTH * 2 - 1:0]      in_data,
    input wire                        in_valid,
    output wire [OUT_WIDTH * 2 - 1:0] out_data,
    output reg                        out_valid
);

// Lightweight simulation model: combinational identity IQ path.
always @(*) begin
    out_valid = !rst && in_valid;
end

assign out_data = {
    in_data[(2 * WIDTH) - 2 -: OUT_WIDTH],
    in_data[OUT_WIDTH-1:0]
};

endmodule

`default_nettype wire
