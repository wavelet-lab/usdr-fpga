`timescale 1ns / 1ps
`default_nettype none

module tb_usdr_tx_chain;

localparam integer OUT_WIDTH   = 12;
localparam integer IN_WIDTH    = 16;
localparam integer L_TX_CHANS  = 1;

localparam [6:0]
    CFG_REG_RESET  = 7'd0,
    CFG_REG_DCCTRL = 7'd1,
    CFG_REG_IQIMB_0 = 7'd2,
    CFG_REG_IQIMB_1 = 7'd3,
    CFG_REG_IQIMB_2 = 7'd4,
    CFG_REG_IQIMB_3 = 7'd5,
    CFG_REG_DC_GEN = 7'd7,
    CFG_REG_NCO0_L = 7'd8,
    CFG_REG_NCO0_H = 7'd9,
    CFG_REG_NCO1_L = 7'd10,
    CFG_REG_NCO1_H = 7'd11;

reg                          o_clk      = 1'b0;
wire [OUT_WIDTH-1:0]         o_data;
wire                         o_sel;
wire                         o_clk_fwd;

wire                         i_clk;
reg  [2*IN_WIDTH-1:0]        i_data     = 32'd0;
reg                          i_valid    = 1'b0;
wire                         i_ready;
wire                         i_rst;

reg                          cfg_clk    = 1'b0;
reg                          cfg_rst    = 1'b1;
reg                          cfg_wvalid = 1'b0;
wire                         cfg_wready;
reg  [31:0]                  cfg_wdata  = 32'd0;
wire                         cfg_rvalid;
reg                          cfg_rready = 1'b1;
wire [31:0]                  cfg_rdata;
wire                         cfg_tx_rst;

reg                          cordic_rstn       = 1'b0;
reg                          cordic_phase_vld  = 1'b0;
reg  [15:0]                  cordic_phase_data = 16'd0;
reg                          cordic_cart_vld   = 1'b0;
reg  [31:0]                  cordic_cart_data  = 32'd0;
wire                         cordic_out_vld;
wire [31:0]                  cordic_out_data;

integer failures = 0;
integer saw_data_i = 0;
integer saw_data_q = 0;
integer saw_sel_hi = 0;
integer saw_sel_lo = 0;
integer saw_fwd_hi = 0;
integer saw_fwd_lo = 0;
integer k;

usdr_tx_chain #(
    .OUT_WIDTH(OUT_WIDTH),
    .IN_WIDTH(IN_WIDTH),
    .L_TX_CHANS(L_TX_CHANS),
    .ULTRA_SCALE(1'b0),
    .NEG_POLARITY(1'b0)
) dut (
    .o_clk(o_clk),
    .o_data(o_data),
    .o_sel(o_sel),
    .o_clk_fwd(o_clk_fwd),
    .i_clk(i_clk),
    .i_data(i_data),
    .i_valid(i_valid),
    .i_ready(i_ready),
    .i_rst(i_rst),
    .cfg_clk(cfg_clk),
    .cfg_rst(cfg_rst),
    .cfg_wvalid(cfg_wvalid),
    .cfg_wready(cfg_wready),
    .cfg_wdata(cfg_wdata),
    .cfg_rvalid(cfg_rvalid),
    .cfg_rready(cfg_rready),
    .cfg_rdata(cfg_rdata),
    .cfg_tx_rst(cfg_tx_rst)
);

cordic_0 cordic_ref (
    .aclk(o_clk),
    .aresetn(cordic_rstn),
    .s_axis_phase_tvalid(cordic_phase_vld),
    .s_axis_phase_tdata(cordic_phase_data),
    .s_axis_cartesian_tvalid(cordic_cart_vld),
    .s_axis_cartesian_tdata(cordic_cart_data),
    .m_axis_dout_tvalid(cordic_out_vld),
    .m_axis_dout_tdata(cordic_out_data)
);

always #2 o_clk = ~o_clk;
always #5 cfg_clk = ~cfg_clk;

task automatic cfg_write;
    input [6:0]  addr;
    input [23:0] data24;
    begin
        @(posedge cfg_clk);
        cfg_wdata  <= {1'b0, addr, data24};
        cfg_wvalid <= 1'b1;
        @(posedge cfg_clk);
        cfg_wvalid <= 1'b0;
        cfg_wdata  <= 32'd0;
    end
endtask

task automatic set_reset_bits;
    input ddr_rst;
    input dsp_rst;
    input ext_rst;
    begin
        cfg_write(CFG_REG_RESET, {20'd0, ext_rst, 1'b0, dsp_rst, ddr_rst});
    end
endtask

task automatic set_dc_corr;
    input [11:0] corr_i;
    input [11:0] corr_q;
    begin
        cfg_write(CFG_REG_DCCTRL, {corr_q, corr_i});
    end
endtask

task automatic set_iq_imbalance_zero;
    begin
        cfg_write(CFG_REG_IQIMB_0, 24'd0);
        cfg_write(CFG_REG_IQIMB_1, 24'd0);
        cfg_write(CFG_REG_IQIMB_2, 24'd0);
        cfg_write(CFG_REG_IQIMB_3, 24'd0);
    end
endtask

task automatic set_dc_generator_disabled;
    begin
        @(posedge cfg_clk);
        cfg_wdata  <= {1'b0, CFG_REG_DC_GEN, 8'd0, 16'd0};
        cfg_wvalid <= 1'b1;
        @(posedge cfg_clk);
        cfg_wvalid <= 1'b0;
        cfg_wdata  <= 32'd0;
    end
endtask

task automatic set_nco;
    input [31:0] phase0;
    input [31:0] phase1;
    begin
        cfg_write(CFG_REG_NCO0_L, {8'd0, phase0[15:0]});
        cfg_write(CFG_REG_NCO0_H, {8'd0, phase0[31:16]});
        cfg_write(CFG_REG_NCO1_L, {8'd0, phase1[15:0]});
        cfg_write(CFG_REG_NCO1_H, {8'd0, phase1[31:16]});
    end
endtask

task automatic check_bit;
    input actual;
    input expected;
    input [8*48-1:0] label;
    begin
        if (actual !== expected) begin
            failures = failures + 1;
            $display("FAIL %0s actual=%0b expected=%0b @%0t", label, actual, expected, $time);
        end else begin
            $display("PASS %0s = %0b @%0t", label, actual, $time);
        end
    end
endtask

task automatic check_u12;
    input [11:0] actual;
    input [11:0] expected;
    input [8*48-1:0] label;
    begin
        if (actual !== expected) begin
            failures = failures + 1;
            $display("FAIL %0s actual=0x%03h expected=0x%03h @%0t", label, actual, expected, $time);
        end else begin
            $display("PASS %0s = 0x%03h @%0t", label, actual, $time);
        end
    end
endtask

task automatic check_u16;
    input [15:0] actual;
    input [15:0] expected;
    input [8*48-1:0] label;
    begin
        if (actual !== expected) begin
            failures = failures + 1;
            $display("FAIL %0s actual=0x%04h expected=0x%04h @%0t", label, actual, expected, $time);
        end else begin
            $display("PASS %0s = 0x%04h @%0t", label, actual, $time);
        end
    end
endtask

task automatic check_s16_near;
    input signed [15:0] actual;
    input signed [15:0] expected;
    input integer       tolerance;
    input [8*48-1:0]    label;
    integer diff;
    begin
        diff = $signed(actual) - $signed(expected);
        if (diff < 0) diff = -diff;

        if (diff > tolerance) begin
            failures = failures + 1;
            $display("FAIL %0s actual=%0d expected=%0d tol=%0d @%0t", label, $signed(actual), $signed(expected), tolerance, $time);
        end else begin
            $display("PASS %0s actual=%0d expected=%0d tol=%0d @%0t", label, $signed(actual), $signed(expected), tolerance, $time);
        end
    end
endtask

task automatic cordic_push_sample;
    input [15:0] phase_word;
    input [31:0] cart_word;
    begin
        @(posedge o_clk);
        cordic_phase_data <= phase_word;
        cordic_cart_data  <= cart_word;
        cordic_phase_vld  <= 1'b1;
        cordic_cart_vld   <= 1'b1;

        @(posedge o_clk);
        cordic_phase_data <= 16'd0;
        cordic_cart_data  <= 32'd0;
        cordic_phase_vld  <= 1'b0;
        cordic_cart_vld   <= 1'b0;
    end
endtask

task automatic wait_for_cordic_output;
    integer timeout;
    begin
        timeout = 0;
        while ((timeout < 32) && (cordic_out_vld !== 1'b1)) begin
            @(posedge o_clk);
            #1;
            timeout = timeout + 1;
        end

        if (timeout == 32) begin
            failures = failures + 1;
            $fatal(1, "timeout waiting for standalone cordic_0 output");
        end
    end
endtask

task automatic wait_for_tx_data;
    integer timeout;
    begin
        timeout = 0;
        while ((timeout < 64) &&
               ((dut.ncoo_dacclk_valid[0] !== 1'b1) ||
                (^dut.ncoo_dacclk_data[0] === 1'bx) ||
                (^dut.corrected_i === 1'bx) ||
                (^dut.corrected_q === 1'bx) ||
                (^dut.serder_i === 1'bx) ||
                (^dut.serder_q === 1'bx))) begin
            @(posedge dut.dac_clk);
            #1;
            timeout = timeout + 1;
        end

        if (timeout == 64) begin
            failures = failures + 1;
            $fatal(1, "timeout waiting for tx datapath to produce a valid sample");
        end

        @(posedge dut.dac_clk);
        #1;
    end
endtask

initial begin
    #5000;
    $fatal(1, "timeout waiting for tb_usdr_tx_chain to finish");
end

initial begin
    $display("[%0t] tb_usdr_tx_chain start", $time);

    repeat (2) @(posedge o_clk);
    cordic_rstn <= 1'b1;

    cordic_push_sample(16'h0000, {16'h0000, 16'h2000});
    wait_for_cordic_output;
    check_s16_near(cordic_out_data[15:0], 16'sh2000, 1, "cordic_i_phase0");
    check_s16_near(cordic_out_data[31:16], 16'sh0000, 1, "cordic_q_phase0");

    cordic_push_sample(16'h1000, {16'h0000, 16'h2000});
    wait_for_cordic_output;
    check_s16_near(cordic_out_data[15:0], 16'sh0000, 4, "cordic_i_phase_pi_over_2");
    check_s16_near(cordic_out_data[31:16], 16'sh2000, 4, "cordic_q_phase_pi_over_2");

    repeat (4) @(posedge cfg_clk);
    cfg_rst <= 1'b0;

    check_bit(cfg_wready, 1'b1, "cfg_wready_static_high");
    check_bit(cfg_rvalid, 1'b1, "cfg_rvalid_static_high");
    check_u16(cfg_rdata[15:0], 16'h0000, "cfg_rdata_default");

    set_reset_bits(1'b1, 1'b1, 1'b1);
    repeat (2) @(posedge cfg_clk);
    check_bit(cfg_tx_rst, 1'b1, "cfg_tx_rst_asserted");
    check_bit(i_rst, 1'b1, "i_rst_asserted");

    set_dc_corr(12'h010, 12'h020);
    set_iq_imbalance_zero;
    set_dc_generator_disabled;
    set_nco(32'h0123_4567, 32'h89ab_cdef);

    repeat (2) @(posedge cfg_clk);
    check_u16(dut.reg_nco0_l, 16'h4567, "reg_nco0_l");
    check_u16(dut.reg_nco0_h, 16'h0123, "reg_nco0_h");
    check_u16(dut.reg_nco1_l, 16'hcdef, "reg_nco1_l");
    check_u16(dut.reg_nco1_h, 16'h89ab, "reg_nco1_h");

    set_nco(32'd0, 32'd0);
    repeat (2) @(posedge cfg_clk);

    set_reset_bits(1'b0, 1'b0, 1'b0);
    repeat (2) @(posedge cfg_clk);
    @(posedge i_clk);
    repeat (2) @(posedge i_clk);

    check_bit(cfg_tx_rst, 1'b0, "cfg_tx_rst_deasserted");
    check_bit(i_rst, 1'b0, "i_rst_deasserted");
    check_bit(i_ready, 1'b1, "i_ready_from_dut");

    i_data  <= {16'h3400, 16'h1200};
    i_valid <= 1'b1;
    wait_for_tx_data;

    check_u16(dut.corrected_i, 16'h1210, "corrected_i");
    check_u16(dut.corrected_q, 16'h3420, "corrected_q");
    check_u12(dut.serder_i, 12'h121, "serder_i");
    check_u12(dut.serder_q, 12'h342, "serder_q");

    for (k = 0; k < 24; k = k + 1) begin
        @(posedge o_clk or negedge o_clk);
        #1;
        if (o_data == dut.serder_i) saw_data_i = 1;
        if (o_data == dut.serder_q) saw_data_q = 1;
        if (o_sel) saw_sel_hi = 1;
        if (!o_sel) saw_sel_lo = 1;
        if (o_clk_fwd) saw_fwd_hi = 1;
        if (!o_clk_fwd) saw_fwd_lo = 1;
    end

    check_bit(saw_data_i[0], 1'b1, "serialized_i_seen");
    check_bit(saw_data_q[0], 1'b1, "serialized_q_seen");
    check_bit(saw_sel_hi[0], 1'b1, "sel_high_seen");
    check_bit(saw_sel_lo[0], 1'b1, "sel_low_seen");
    check_bit(saw_fwd_hi[0], 1'b1, "fwdclk_high_seen");
    check_bit(saw_fwd_lo[0], 1'b1, "fwdclk_low_seen");

    set_reset_bits(1'b0, 1'b0, 1'b1);
    repeat (2) @(posedge cfg_clk);
    check_bit(cfg_tx_rst, 1'b1, "cfg_tx_rst_reasserted");
    check_bit(i_rst, 1'b1, "i_rst_reasserted");

    if (failures != 0) begin
        $fatal(1, "tb_usdr_tx_chain failed with %0d errors", failures);
    end

    $display("[%0t] tb_usdr_tx_chain PASS", $time);
    $finish;
end

endmodule

`default_nettype wire
