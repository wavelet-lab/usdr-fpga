module ddr_bitslip_fsm #(
    parameter [0:0] VIO_CONTROL = 1'b1
)(
    input         cfg_sisoddr,
    input         cfg_sisoddr_skp,

    input         lvds_data_clk_div_buf,
    input         lvds_data_clk_reset,
    
    input         reset_bitslip_fsm,
    input         bitslip_man,
    input         tunephase_man,
    
    input [3:0]   lvds_frame_div,

    output reg    lvds_bitslip,
    
    output reg    lvds_tunephase,
    input         lvds_tunephase_ready,
    
    output [31:0] lvds_stat
);

localparam SKIP_CYCLES = 4'b1111;


wire frame_aligned_mimo    = (lvds_frame_div == 4'b1100);
wire frame_misaligned_mimo = (lvds_frame_div == 4'b0110) ||
                             (lvds_frame_div == 4'b0011) ||
                             (lvds_frame_div == 4'b1001);
                             
wire frame_aligned_sskp    = (lvds_frame_div[0] == 1'b0 && lvds_frame_div[2] == 1'b1);
wire frame_misaligned_sskp = !frame_aligned_sskp && (
                             (lvds_frame_div[1] == 1'b0 && lvds_frame_div[3] == 1'b1) || 
                             (lvds_frame_div[2] == 1'b0 && lvds_frame_div[0] == 1'b1) ||
                             (lvds_frame_div[3] == 1'b0 && lvds_frame_div[1] == 1'b1));
                             
wire frame_aligned_siso    = (lvds_frame_div == 4'b1010);
wire frame_misaligned_siso = (lvds_frame_div == 4'b0101);

wire frame_aligned         = cfg_sisoddr_skp ? frame_aligned_sskp    : cfg_sisoddr ? frame_aligned_siso    : frame_aligned_mimo;
wire frame_misaligned      = cfg_sisoddr_skp ? frame_misaligned_sskp : cfg_sisoddr ? frame_misaligned_siso : frame_misaligned_mimo;


reg [3:0]                lvds_check_delay;
reg [3:0]                lvds_upd_delay;


reg [31:0]                lvds_frame_corr;
reg [31:0]                lvds_phase_corr;
reg [31:0]               lvds_phase_err;

wire [3:0]               lvds_frame_dive;

reg lvds_do_bitslip;
reg lvds_do_tunephase;

reg bitslip_man_p = 0;
reg tunephase_man_p = 0;

assign lvds_stat = { lvds_phase_err[15:0], lvds_frame_corr[7:0], lvds_phase_corr[7:0] };

wire ctrl_override = VIO_CONTROL;

wire bitslip_man_vio;
wire tunephase_man_vio;

wire bitslip_man_ctrl  = (ctrl_override) ?  bitslip_man_vio   : bitslip_man;
wire tunephase_man_ctrl= (ctrl_override) ?  tunephase_man_vio : tunephase_man;

generate
if (VIO_CONTROL) begin
vio_0 vio_0(
    .clk(lvds_data_clk_div_buf),
    .probe_in0(lvds_phase_corr),
    .probe_in1(lvds_frame_corr),
    .probe_in2(lvds_phase_err),
    .probe_in3(frame_aligned_mimo),
    .probe_in4(lvds_frame_div),
    .probe_in5(lvds_tunephase_ready),
    .probe_out0(bitslip_man_vio),
    .probe_out1(tunephase_man_vio)
);
end
endgenerate

always @(posedge lvds_data_clk_div_buf) begin
    if (reset_bitslip_fsm) begin
        lvds_check_delay  <= 0;
        lvds_upd_delay    <= 0;
        lvds_frame_corr   <= 0;
        lvds_phase_err    <= 0;
        lvds_phase_corr   <= 0;
        lvds_do_bitslip   <= 0; // (bitslip_man_p == 0 && bitslip_man_ctrl == 1'b1);
        lvds_do_tunephase <= 0; // (tunephase_man_p == 0 && tunephase_man_ctrl == 1'b1);
    end else begin
        lvds_do_bitslip   <= 0;
        lvds_do_tunephase <= 1'b0;
                
        bitslip_man_p     <= bitslip_man_ctrl;
        tunephase_man_p   <= tunephase_man_ctrl;
        
        
        if (lvds_check_delay != SKIP_CYCLES) begin
            lvds_check_delay <= lvds_check_delay + 1'b1;
        end else begin
            lvds_do_bitslip   <= (bitslip_man_p != bitslip_man_ctrl);
            lvds_do_tunephase <= ( 1 || lvds_tunephase_ready ) ? (tunephase_man_p != tunephase_man_ctrl) : 0;
                
            if (frame_misaligned) begin
                lvds_upd_delay   <= lvds_upd_delay + 1'b1;
                if (lvds_upd_delay ==  SKIP_CYCLES) begin
                    lvds_check_delay   <= 0;
                    lvds_do_bitslip    <= 1'b1;
                end
            end else if (~frame_aligned) begin
                lvds_phase_err <= lvds_phase_err + 1'b1;
                lvds_upd_delay <= 0;
                
                if (lvds_tunephase_ready) begin
                    lvds_check_delay  <= 0;
                    lvds_do_tunephase <= 1'b1;
                end
            end
        end
        
        if (lvds_do_tunephase) begin
            lvds_phase_corr   <= lvds_phase_corr + 1'b1;
        end
        if (lvds_do_bitslip) begin
            lvds_frame_corr    <= lvds_frame_corr + 1'b1;
        end
    end
end

always @(posedge lvds_data_clk_div_buf or posedge lvds_data_clk_reset) begin
    if (lvds_data_clk_reset) begin
        lvds_bitslip   <= 0;
        lvds_tunephase <= 0;
    end else begin
        lvds_bitslip   <= lvds_do_bitslip;
        lvds_tunephase <= lvds_do_tunephase;
    end
end


endmodule
