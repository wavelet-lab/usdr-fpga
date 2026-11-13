module ddr_realign #(
    parameter WIDTH = 12
)(
    input         clk,
    input         rst,

    input [3:0]              s_demux_frame,
    input [4 * WIDTH - 1:0]  s_demux_data,
    input                    s_demux_valid,

    output [4 * WIDTH - 1:0] m_align_data,
    output                   m_align_valid,

    input                    cfg_sisoddr,
    input                    cfg_sisoddr_skip,
    output [31:0]            stat
);


reg  [4 * WIDTH - 1:0] p_ddr_in_4_data;
reg  [3:0]             p_ddr_in_4_frame;
reg                    p_ddr_in_4_valid;
reg  [1:0]             p_sel;
reg  [1:0]             p_sel_prev;

wire [7:0]             combo_frame      = {s_demux_frame, p_ddr_in_4_frame};
wire [8 * WIDTH - 1:0] combo_data       = {s_demux_data,  p_ddr_in_4_data};
wire [3:0]             combo_frame_siso = p_ddr_in_4_frame;


reg [15:0] frame_errs;
reg [15:0] frame_realigns;

assign stat = { frame_errs, frame_realigns };

// 1100
always @(posedge clk) begin
  if (rst) begin
    p_sel            <= 0;
    p_sel_prev       <= 0;
    frame_errs       <= 0;
    frame_realigns   <= 0;
    p_ddr_in_4_valid <= 0;
  end else begin
    p_ddr_in_4_valid <= s_demux_valid;
    if (s_demux_valid) begin
        p_ddr_in_4_data  <= s_demux_data;
        p_ddr_in_4_frame <= s_demux_frame;

        // Standard SISO_DDR
        if (cfg_sisoddr && !cfg_sisoddr_skip && combo_frame_siso == 4'b1010) begin
            p_sel <= 0;
        end else if (cfg_sisoddr && !cfg_sisoddr_skip && combo_frame_siso == 4'b0101) begin
            p_sel <= 1;
        // Skip SISO_DDR on x2 recv clock
        end else if (!cfg_sisoddr && cfg_sisoddr_skip && ((combo_frame_siso & 8'b0101_0101) == 8'b0100_0100)) begin
            p_sel <= 0;
        end else if (!cfg_sisoddr && cfg_sisoddr_skip && ((combo_frame_siso & 8'b0101_0101) == 8'b0001_0001)) begin
            p_sel <= 2;
        // Classical MIMO
        end else if (!cfg_sisoddr && !cfg_sisoddr_skip && combo_frame == 8'b1100_1100) begin //MIMO
            p_sel <= 0;
        end else if (!cfg_sisoddr && !cfg_sisoddr_skip && combo_frame == 8'b0110_0110) begin //MIMO
            p_sel <= 3;
        end else if (!cfg_sisoddr && !cfg_sisoddr_skip && combo_frame == 8'b0011_0011) begin //MIMO
            p_sel <= 2;
        end else if (!cfg_sisoddr && !cfg_sisoddr_skip && combo_frame == 8'b1001_1001) begin //MIMO
            p_sel <= 1;
        end else if (p_ddr_in_4_valid) begin
            frame_errs <= frame_errs + 1'b1;
        end
    end

    p_sel_prev <= p_sel;
    if (p_sel_prev != p_sel) begin
        frame_realigns <= frame_realigns + 1'b1;
    end
  end
end

assign m_align_data  = combo_data >> (WIDTH * p_sel);
assign m_align_valid = s_demux_valid;//p_ddr_in_4_valid;


endmodule

