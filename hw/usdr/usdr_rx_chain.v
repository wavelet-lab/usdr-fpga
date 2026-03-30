`timescale 1ns / 1ps

module usdr_rx_chain #(
    parameter IN_WIDTH = 12,
    parameter OUT_WIDTH = 16,
    parameter L_RX_CHANS = 1,
    parameter ULTRA_SCALE = 1'b0,
    parameter NEG_POLARITY = 1'b1
) (
    input                                   i_clk,
    input [IN_WIDTH-1:0]                    i_data,
    input                                   i_sel,
    
    output                                  o_clk,
    output [2 * OUT_WIDTH * L_RX_CHANS-1:0] o_data,
    output                                  o_valid,
    input                                   o_ready,
    output                                  o_rst,
    
    // Configuration DDC
    input                      cfg_clk,
    input                      cfg_rst,
    
    input                      cfg_wvalid,
    output                     cfg_wready,
    input [31:0]               cfg_wdata,
    
    output                     cfg_rvalid,
    input                      cfg_rready,
    output [31:0]              cfg_rdata,
    
    output                     cfg_rx_rst
);

assign cfg_wready = 1'b1;
assign cfg_rvalid = 1'b1;

localparam DSPPHASE_WIDTH = 32;


// Chain
// 1 -> 1:2 demux, incl. clock -> out_width extention
// 2  DC corrector
// 3  IQ corrector
// 4  Cordic
// 5  DDC

genvar i;

// Divided clock
wire clk_div_o;
wire clk_io;

wire clk_div_ndiv;
BUFR #(.BUFR_DIVIDE("1")) mrcc_div1(
    .I(i_clk),
    .O(clk_div_ndiv),
    .CE(1'b1),
    .CLR(1'b0)
);


BUFR #(.BUFR_DIVIDE("2")) mrcc_div2(
    .I(i_clk),
    .O(clk_div_o),
    .CE(1'b1),
    .CLR(1'b0)
);
BUFIO in_bufio(.I(i_clk), .O(clk_io));

// Debux
wire [2*IN_WIDTH-1:0] data_deser;
wire [7:0]            data_iqsel;
wire                  ddr_reset;
wire                  ddr_reset_adc;


generate
for (i = 0; i < IN_WIDTH; i=i+1) begin: dlvds
    ISERDESE2 #(
//        .DATA_RATE("SDR"),
//        .DATA_WIDTH(2),
        .DATA_RATE("DDR"),
        .DATA_WIDTH(4),
        .INTERFACE_TYPE("NETWORKING"),
        .NUM_CE(1)
    ) iserdese2_lvds_data (
        .RST(ddr_reset),
        .D(i_data[i]),
        .DDLY(),
//        .Q1(data_deser[i + 1*IN_WIDTH]),
//        .Q2(data_deser[i + 0*IN_WIDTH]),
//        .Q3(),
//        .Q4(),
        .Q1(data_deser[i + 1*IN_WIDTH]),
        .Q2(),
        .Q3(data_deser[i + 0*IN_WIDTH]),
        .Q4(),
        .Q5(),
        .Q6(),
        .Q7(),
        .Q8(),
        .CLK(clk_io),
        .CLKB(~clk_io),
        .OCLK(1'b0),
        .OCLKB(1'b0),
        .CLKDIV(clk_div_o),
        .CLKDIVP(1'b0),
        .DYNCLKSEL(1'b0),
        .DYNCLKDIVSEL(1'b0),
        .CE1(1'b1),
        .CE2(1'b1),
        .BITSLIP(1'b0),
        .OFB(1'b0),
        .SHIFTIN1(1'b0),
        .SHIFTIN2(1'b0),
        .SHIFTOUT1(),
        .SHIFTOUT2(),
        .O()
    );
end
endgenerate

ISERDESE2 #(
//    .DATA_RATE("SDR"),
//    .DATA_WIDTH(2),
    .DATA_RATE("DDR"),
    .DATA_WIDTH(4),
    .INTERFACE_TYPE("NETWORKING"),
    .NUM_CE(1)
) iserdese2_lvds_iqsel (
    .RST(ddr_reset),
    .D(i_sel),
    .DDLY(),
//    .Q1(data_iqsel[1]),
//    .Q2(data_iqsel[0]),
//    .Q3(data_iqsel[2]),
//    .Q4(data_iqsel[3]),
    .Q1(data_iqsel[1]),
    .Q2(),
    .Q3(data_iqsel[0]),
    .Q4(),
//    .Q5(data_iqsel[4]),
//    .Q6(data_iqsel[5]),
//    .Q7(data_iqsel[6]),
//    .Q8(data_iqsel[7]),
    .CLK(clk_io),
    .CLKB(~clk_io),
    .OCLK(1'b0),
    .OCLKB(1'b0),
    .CLKDIV(clk_div_o),
    .CLKDIVP(1'b0),
    .DYNCLKSEL(1'b0),
    .DYNCLKDIVSEL(1'b0),
    .CE1(1'b1),
    .CE2(1'b1),
    .BITSLIP(1'b0),
    .OFB(1'b0),
    .SHIFTIN1(1'b0),
    .SHIFTIN2(1'b0),
    .SHIFTOUT1(),
    .SHIFTOUT2(),
    .O()
);
//        __    __    __    _
// CLK   /  \__/  \__/  \__/
//        _____       _____
// IQSEL /     \_____/     \_
//
// ISE2  .Q2_0  .Q1_0 .Q2_1 .Q1_1 ...          
//         I0     Q0    I1    Q1
// bit      0      1     0     1
//                                                                                     Combined: { N , N-1 }
// Data aligned:      data_iqsel == 2'b01  data == { .Q1_N (Q0)    , .Q2_N   (I0) }   ||  { Q0, I0, Q-1, I-1 }
// Data misaligned:   data_iqsel == 2'b10  data == { .Q2_N (I1)    , .Q1_N-1 (Q0) }   ||  { I1, Q0, I0,  Q-1 }
//

reg  igp_dsp_cfg_rst;

wire adc_clk;
BUFG clk_div_bufg(.I(clk_div_o), .O(adc_clk));
//assign adc_clk = clk_div_o;

reg [2*IN_WIDTH-1:0] deser_aligned_data;
reg [2*IN_WIDTH-1:0] data_deser_p;
reg                  deser_aligned_valid;

wire [4*IN_WIDTH-1:0] data_deser_w = { data_deser, data_deser_p };

localparam [1:0] IQ_SEL_ALIGNED = NEG_POLARITY ? 2'b10 : 2'b01;
 
always @(posedge adc_clk) begin
    deser_aligned_valid  <= !ddr_reset_adc;
    data_deser_p         <= data_deser;
    
    deser_aligned_data   <= data_deser_w >> ((data_iqsel == IQ_SEL_ALIGNED) ? (2*IN_WIDTH) : (IN_WIDTH));
end

// IQ corr + dc_offset, extend to 16 bits
wire [OUT_WIDTH - 1:0] deser_data_ext_q = deser_aligned_data[2 * IN_WIDTH - 1:IN_WIDTH] <<< (OUT_WIDTH - IN_WIDTH);
wire [OUT_WIDTH - 1:0] deser_data_ext_i = deser_aligned_data[IN_WIDTH - 1:0]            <<< (OUT_WIDTH - IN_WIDTH);

`define NULLLL
`ifdef NULLLL
wire                     adc_dc_corr_en;
wire [2*OUT_WIDTH - 1:0] adc_dc_corr_vals;
wire                     adc_dccorr_valid;
wire [OUT_WIDTH - 1:0]   adc_dccorr_q;
wire [OUT_WIDTH - 1:0]   adc_dccorr_i;

dsp_dc_corr #(.WIDTH(OUT_WIDTH)) dc_corr (
    .clk(adc_clk),
    .rst(ddr_reset_adc),
    .in_data({ deser_data_ext_q, deser_data_ext_i }),
    .in_valid(deser_aligned_valid),
    
    .out_data({ adc_dccorr_q, adc_dccorr_i }),
    .out_valid(adc_dccorr_valid),
    .cfg_dc_corr_en(adc_dc_corr_en),
    .corr_data(adc_dc_corr_vals)
);


// Cordic
localparam DSP_WIDTH = OUT_WIDTH;

wire [L_RX_CHANS * DSPPHASE_WIDTH-1:0] adc_dsp_cordic_phase;
wire [L_RX_CHANS * 32:0]               ncoo_adcclk_data;
wire [L_RX_CHANS - 1:0]                ncoo_adcclk_valid;

generate
for (i = 0; i < L_RX_CHANS; i=i+1) begin: ncos
    reg  [DSPPHASE_WIDTH-1:0] nco_value;
    
    always @(posedge adc_clk) begin
        if (ddr_reset_adc) begin
            nco_value <= 0;
        end else begin
            nco_value <= nco_value + adc_dsp_cordic_phase[DSPPHASE_WIDTH * (i + 1) - 1:DSPPHASE_WIDTH * i];
        end
    end    

    wire [DSP_WIDTH:0] ncoi_adcclk_data_i = { adc_dccorr_i[DSP_WIDTH - 1], adc_dccorr_i };
    wire [DSP_WIDTH:0] ncoi_adcclk_data_q = { adc_dccorr_q[DSP_WIDTH - 1], adc_dccorr_q };
    
    wire [31:0] ncoi_adcclk_data_native = { ncoi_adcclk_data_q[DSP_WIDTH:DSP_WIDTH - 15], ncoi_adcclk_data_i[DSP_WIDTH:DSP_WIDTH - 15] };
    wire [31:0] ncoo_adcclk_data_native;
    
    cordic_0 c0(
        .aclk(adc_clk),
        .aresetn(!ddr_reset_adc),
        .s_axis_phase_tdata( { nco_value[31], nco_value[31], nco_value[31:18]} ),
        .s_axis_phase_tvalid(adc_dccorr_valid),
        .s_axis_cartesian_tdata(ncoi_adcclk_data_native),
        .s_axis_cartesian_tvalid(adc_dccorr_valid),
        .m_axis_dout_tdata(ncoo_adcclk_data_native),
        .m_axis_dout_tvalid(ncoo_adcclk_valid[i])
    );
    
    assign ncoo_adcclk_data[32 * (i + 1) - 1 : 32 * i] = { ncoo_adcclk_data_native[30:16], 1'b0, ncoo_adcclk_data_native[14:0], 1'b0 };
end
endgenerate
// FIR filter


localparam STAGES    = 8;
localparam CFG_WIDTH = 8;

// FIR gearbox
wire [STAGES-1:0]   dsp_dspchain_prg_data;
wire                dsp_dspchain_prg_valid;
wire                fir_adc_clk_dsp_uload_rst;

synchronizer  #(.INIT(1)) dsp_uload_rst_sync (.clk(adc_clk),   .rst(1'b0), .a_in(igp_dsp_cfg_rst), .s_out(fir_adc_clk_dsp_uload_rst));

axis_cc_fifo #(
    .WIDTH(STAGES)
) igpdsprxcfg_fifo (
    .rx_clk(cfg_clk),
    .rx_rst(igp_dsp_cfg_rst),

    .s_rx_tdata(igp_prg_data),
    .s_rx_tvalid(igp_prg_strobe),
    .s_rx_tready(),

    .tx_clk(adc_clk),
    .tx_rst(fir_adc_clk_dsp_uload_rst),

    .m_tx_tdata(dsp_dspchain_prg_data),
    .m_tx_tvalid(dsp_dspchain_prg_valid),
    .m_tx_tready(1'b1)
);

reconf_dsp_fir #(
    .IN_WIDTH(DSP_WIDTH),
    .CHANS(2 * L_RX_CHANS),
    .OUT_WIDTH(DSP_WIDTH),
    .CFG_WIDTH(CFG_WIDTH),
    .STAGES(STAGES),
    .ULTRA_SCALE(ULTRA_SCALE)
) rx_reconf (
    .rst(ddr_reset_adc),
    .clk(adc_clk),

    .in_valid(ncoo_adcclk_valid),
    .in_data(ncoo_adcclk_data),
    .in_ready(),

    .out_valid(o_valid),
    .out_data(o_data),
    .out_ready(o_ready),

    .cfg_valid(dsp_dspchain_prg_valid),
    .cfg_data(dsp_dspchain_prg_data)
);

// Configurator
`else

wire [31:0] adc_dc_corr_vals = data_iqsel;
assign o_valid = deser_aligned_valid;
assign o_data  = { deser_data_ext_i, deser_data_ext_q, deser_data_ext_q, deser_data_ext_i };

`endif

reg [15:0] reg_nco0_l;
reg [15:0] reg_nco0_h;
reg [15:0] reg_nco1_l;
reg [15:0] reg_nco1_h;

assign adc_dsp_cordic_phase = { reg_nco1_h, reg_nco1_l, reg_nco0_h, reg_nco0_l };

localparam [7:0]
    CFG_REG_RESET = 0,
    CFG_REG_DCCTRL = 1,
    CFG_REG_IQIMB_0 = 2,
    CFG_REG_IQIMB_1 = 3,
    CFG_REG_IQIMB_2 = 4,
    CFG_REG_IQIMB_3 = 5,
    CFG_REG_NCO0_L = 8,
    CFG_REG_NCO0_H = 9,
    CFG_REG_NCO1_L = 10,
    CFG_REG_NCO1_H = 11,
    CFG_REG_DSP_LD = 64,
    CFG_REG_RD_ADDR_MSK = 128;


wire       rd_reg_update = cfg_wdata[31];
wire [6:0] wr_addr = cfg_wdata[30:24];
reg [6:0]  rd_addr;


// DC_EN
reg adc_dc_corr_en_cfg;
synchronizer  #(.INIT(1)) dc_en_reg_sync (.clk(adc_clk),   .rst(1'b0), .a_in(adc_dc_corr_en_cfg), .s_out(adc_dc_corr_en));

reg ddr_reset_cfg;
reg ddr_resete_cfg;
synchronizer  #(.INIT(1)) ddr_reset_sync     (.clk(clk_div_o),   .rst(1'b0), .a_in(ddr_reset_cfg),  .s_out(ddr_reset));
synchronizer  #(.INIT(1)) ddr_reset_adc_sync (.clk(adc_clk),     .rst(1'b0), .a_in(ddr_reset_cfg),  .s_out(ddr_reset_adc));

synchronizer  #(.INIT(1)) ddr_reset_ext_sync (.clk(adc_clk),     .rst(1'b0), .a_in(ddr_resete_cfg), .s_out(o_rst));

always @(posedge cfg_clk) begin
    if (cfg_rst) begin
        ddr_reset_cfg   <= 1'b1;
        igp_dsp_cfg_rst <= 1'b1;
    end else begin
        if (cfg_wvalid) begin
            if (rd_reg_update) begin
                rd_addr <= wr_addr;
            end else begin
                case (wr_addr)
                    CFG_REG_RESET: begin
                        ddr_reset_cfg   <= cfg_wdata[0];
                        igp_dsp_cfg_rst <= cfg_wdata[1];
                        ddr_resete_cfg  <= cfg_wdata[3];
                    end
                    CFG_REG_DCCTRL: adc_dc_corr_en_cfg <= cfg_wdata[0];
                    CFG_REG_NCO0_L: reg_nco0_l <= cfg_wdata;
                    CFG_REG_NCO0_H: reg_nco0_h <= cfg_wdata;
                    CFG_REG_NCO1_L: reg_nco1_l <= cfg_wdata;
                    CFG_REG_NCO1_H: reg_nco1_h <= cfg_wdata;
                endcase
            end
        end
    end
end

assign igp_prg_data   = cfg_wdata[7:0];
assign igp_prg_strobe = cfg_wvalid && !rd_reg_update && (wr_addr == CFG_REG_DSP_LD);


assign cfg_rdata = adc_dc_corr_vals;

assign o_clk = adc_clk;

assign cfg_rx_rst = ddr_resete_cfg;


ila_0 ila_0(
    .clk(cfg_clk),
    .probe0(clk_div_ndiv),
    .probe1(data_iqsel[0]),
    .probe2(data_iqsel[1]),
    .probe3(ddr_reset),
    .probe4(i_sel),
    .probe5(adc_clk)
);


endmodule
