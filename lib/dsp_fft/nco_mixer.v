module nco_mixer #(
    parameter IQCHANS   = 1,
    parameter ADC_WIDTH = 12
)(
    input                            adc_clk,
    input                            adc_rst,

    input [2*ADC_WIDTH - 1:0]        adc_data,
    input                            adc_valid,

    input [32*1-1:0]                 cfg_dsp_cordic_phase,

    output [IQCHANS*32-1:0]          nco_data,
    output [IQCHANS-1:0]             nco_valid
);

genvar i;
generate
for (i = 0; i < IQCHANS; i = i + 1) begin: genb
    reg [31:0] nco_value;
    always @(posedge adc_clk) begin
        if (adc_rst) begin
            nco_value <= 0;
        end else begin
            if (adc_valid)
                nco_value <= nco_value + cfg_dsp_cordic_phase[32 * i + 31 : 32 * i];
        end
    end

    wire [15:0] ncoi_adcclk_data_i = (ADC_WIDTH < 15) ?
        { adc_data[ADC_WIDTH - 1], adc_data[ADC_WIDTH - 1:0], {(15 - ADC_WIDTH){1'b0}}} :
        { adc_data[ADC_WIDTH - 1], adc_data[ADC_WIDTH - 1:ADC_WIDTH - 15]};
    wire [15:0] ncoi_adcclk_data_q = (ADC_WIDTH < 15) ?
        { adc_data[2*ADC_WIDTH - 1], adc_data[2*ADC_WIDTH - 1:ADC_WIDTH], {(15 - ADC_WIDTH){1'b0}}} :
        { adc_data[2*ADC_WIDTH - 1], adc_data[2*ADC_WIDTH - 1:2*ADC_WIDTH - 15]};

    wire [31:0] ncoi_adcclk_data_native = { ncoi_adcclk_data_q, ncoi_adcclk_data_i };
    wire [31:0] ncoo_adcclk_data_native;

    wire ncoo_adcclk_valid;
    // 1+15-bit phase accuracy 
    cordic_0 c0(
        .aclk(adc_clk),
        .aresetn(!adc_rst),
        .s_axis_phase_tdata( { nco_value[31], nco_value[31], nco_value[31:18]} ),
        .s_axis_phase_tvalid(adc_valid),
        .s_axis_cartesian_tdata(ncoi_adcclk_data_native),
        .s_axis_cartesian_tvalid(adc_valid),
        .m_axis_dout_tdata(ncoo_adcclk_data_native),
        .m_axis_dout_tvalid(ncoo_adcclk_valid)
    );

    wire [31:0] ncoo_adcclk_data = { ncoo_adcclk_data_native[30:16], 1'b0, ncoo_adcclk_data_native[14:0], 1'b0 };

    assign nco_valid[i]                    = ncoo_adcclk_valid;
    assign nco_data[32 * i + 31 : 32 * i]  = ncoo_adcclk_data;
end
endgenerate


endmodule
