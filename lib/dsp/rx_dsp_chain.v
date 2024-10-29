// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module rx_dsp_chain #(
    parameter ADC_WIDTH      = 14,
    parameter DSP_WIDTH      = 16,
    parameter CFG_WIDTH      = 32,
    parameter DSPPHASE_WIDTH = 32,
    parameter CORDIC         = 0,
    parameter DC_CORR        = 0,
    parameter STAGES         = 8
)(
    input                        adc_clk,
    input                        adc_rst,
    input [ADC_WIDTH * 2 - 1:0]  adc_data,
    input                        adc_valid,
    input [DSPPHASE_WIDTH-1:0]   adc_dsp_cordic_phase,

    input                        adc_dc_corr_en,
    output [DSP_WIDTH * 2 - 1:0] adc_dc_corr_vals,


    input                        dsp_clk,
    input                        dsp_rst,
    output [DSP_WIDTH * 2 - 1:0] dsp_data,
    output                       dsp_valid,
    input                        dsp_ready,
    // input                        dsp_cfg_rst,
    input                        dsp_cfg_valid,
    input [CFG_WIDTH-1:0]        dsp_cfg_data
);

// Order of DSP chain
// 0. AGC estimation coarse (anti saturation)
// 1. DC_CORR
// 2. IQ_CORR
// 3. CORDIC
// 4. FIR ARRAY


localparam CHANS   = 2;

wire                       ncoo_adcclk_valid;
wire [DSP_WIDTH * 2 - 1:0] ncoo_adcclk_data;

wire [ADC_WIDTH - 1:0] adc_data_i     = adc_data[1 * ADC_WIDTH - 1:0 * ADC_WIDTH];
wire [ADC_WIDTH - 1:0] adc_data_q     = adc_data[2 * ADC_WIDTH - 1:1 * ADC_WIDTH];

wire [DSP_WIDTH - 1:0] adc_data_ext_i = adc_data_i << (DSP_WIDTH - ADC_WIDTH);
wire [DSP_WIDTH - 1:0] adc_data_ext_q = adc_data_q << (DSP_WIDTH - ADC_WIDTH);

wire [DSP_WIDTH - 1:0] adc_dccorr_i;
wire [DSP_WIDTH - 1:0] adc_dccorr_q;
wire [DSP_WIDTH - 1:0] adc_dccorr_valid;

// DC offset digital correction
generate
    if (DC_CORR) begin
        dsp_dc_corr #(.WIDTH(DSP_WIDTH)) dc_corr (
            .clk(adc_clk),
            .rst(adc_rst),
            .in_data({ adc_data_ext_q, adc_data_ext_i }),
            .in_valid(adc_valid),
            .out_data({ adc_dccorr_q, adc_dccorr_i }),
            .out_valid(adc_dccorr_valid),
            .cfg_dc_corr_en(adc_dc_corr_en),
            .corr_data(adc_dc_corr_vals)
        );
    end else begin
        assign adc_dccorr_i     = adc_data_ext_i;
        assign adc_dccorr_q     = adc_data_ext_q;
        assign adc_dccorr_valid = adc_valid;
        
        assign adc_dc_corr_vals = {DSP_WIDTH{2'b00}};
    end
endgenerate


// IQ_CORR


generate
    if (CORDIC) begin
        reg [DSPPHASE_WIDTH-1:0] nco_value;
        always @(posedge adc_clk) begin
            if (adc_rst) begin
                nco_value <= 0;
            end else begin
                nco_value <= nco_value + adc_dsp_cordic_phase;
            end
        end

        wire [DSP_WIDTH:0] ncoi_adcclk_data_i = { adc_dccorr_i[DSP_WIDTH - 1], adc_dccorr_i };
        wire [DSP_WIDTH:0] ncoi_adcclk_data_q = { adc_dccorr_q[DSP_WIDTH - 1], adc_dccorr_q };

        //wire [15:0] ncoi_adcclk_data_i = (ADC_WIDTH < 15) ?
            //{ adc_data[ADC_WIDTH - 1], adc_data[ADC_WIDTH - 1:0], {(15 - ADC_WIDTH){1'b0}}} :
            //{ adc_data[ADC_WIDTH - 1], adc_data[ADC_WIDTH - 1:ADC_WIDTH - 15]};
        //wire [15:0] ncoi_adcclk_data_q = (ADC_WIDTH < 15) ?
            //{ adc_data[2*ADC_WIDTH - 1], adc_data[2*ADC_WIDTH - 1:ADC_WIDTH], {(15 - ADC_WIDTH){1'b0}}} :
            //{ adc_data[2*ADC_WIDTH - 1], adc_data[2*ADC_WIDTH - 1:2*ADC_WIDTH - 15]};

        //wire [31:0] ncoi_adcclk_data_native = { ncoi_adcclk_data_q, ncoi_adcclk_data_i };

        wire [31:0] ncoi_adcclk_data_native = { ncoi_adcclk_data_q[DSP_WIDTH:DSP_WIDTH - 15], ncoi_adcclk_data_i[DSP_WIDTH:DSP_WIDTH - 15] };
        wire [31:0] ncoo_adcclk_data_native;

        cordic_0 c0(
            .aclk(adc_clk),
            .aresetn(!adc_rst),
            .s_axis_phase_tdata( { nco_value[31], nco_value[31], nco_value[31:18]} ),
            .s_axis_phase_tvalid(adc_dccorr_valid),
            .s_axis_cartesian_tdata(ncoi_adcclk_data_native),
            .s_axis_cartesian_tvalid(adc_dccorr_valid),
            .m_axis_dout_tdata(ncoo_adcclk_data_native),
            .m_axis_dout_tvalid(ncoo_adcclk_valid)
        );

        assign ncoo_adcclk_data = { ncoo_adcclk_data_native[30:16], 1'b0, ncoo_adcclk_data_native[14:0], 1'b0 };
    end else begin
        assign ncoo_adcclk_valid = adc_valid;
        //assign ncoo_adcclk_data[DSP_WIDTH - 1:0]           = {adc_data[ADC_WIDTH - 1: 0],           {(DSP_WIDTH - ADC_WIDTH){1'b0}} };
        //assign ncoo_adcclk_data[2*DSP_WIDTH - 1:DSP_WIDTH] = {adc_data[ADC_WIDTH*2 - 1: ADC_WIDTH], {(DSP_WIDTH - ADC_WIDTH){1'b0}} };

        assign ncoo_adcclk_data = { adc_dccorr_q, adc_dccorr_i };
    end
endgenerate

wire [2 * DSP_WIDTH - 1:0] ncoo_dspclk_data;
wire                       ncoo_dspclk_valid;
wire                       ncoo_dspclk_ready;

axis_cc_fifo #(
    .WIDTH(DSP_WIDTH * 2),
    .DEEP_BITS(5)
) adc_fifo_i (
    .rx_clk(adc_clk),
    .rx_rst(adc_rst),

    .s_rx_tdata(ncoo_adcclk_data),
    .s_rx_tvalid(ncoo_adcclk_valid),
    .s_rx_tready(),

    .tx_clk(dsp_clk),
    .tx_rst(dsp_rst),

    .m_tx_tdata(ncoo_dspclk_data),
    .m_tx_tvalid(ncoo_dspclk_valid),
    .m_tx_tready(ncoo_dspclk_ready)
);

reconf_dsp_fir #(
    .IN_WIDTH(DSP_WIDTH),
    .CHANS(CHANS),
    .OUT_WIDTH(DSP_WIDTH),
    .CFG_WIDTH(CFG_WIDTH),
    .STAGES(STAGES)
) reconf (
    .rst(dsp_rst),
    .clk(dsp_clk),

    .in_valid(ncoo_dspclk_valid),
    .in_data(ncoo_dspclk_data),
    .in_ready(ncoo_dspclk_ready),

    .out_valid(dsp_valid),
    .out_data(dsp_data),
    .out_ready(dsp_ready),

   // .cfg_rst(dsp_cfg_rst),
    .cfg_valid(dsp_cfg_valid),
    .cfg_data(dsp_cfg_data)
);


endmodule
