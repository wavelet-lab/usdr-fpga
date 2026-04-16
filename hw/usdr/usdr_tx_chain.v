`timescale 1ns / 1ps
`default_nettype none

module usdr_tx_chain #(
    parameter OUT_WIDTH = 12,
    parameter IN_WIDTH = 16,
    parameter L_TX_CHANS = 1,
    parameter ULTRA_SCALE = 1'b0,
    parameter NEG_POLARITY = 1'b1
) (
    input wire                                  o_clk,
    output wire [OUT_WIDTH-1:0]                 o_data,
    output wire                                 o_sel,
    output wire                                 o_clk_fwd,

    output wire                                  i_clk,
    input wire [2 * IN_WIDTH * L_TX_CHANS-1:0]   i_data,
    input wire                                   i_valid,
    output wire                                  i_ready,
    output wire                                  i_rst,
    input wire                                   i_enable,

    // Configuration DDC
    input wire                     cfg_clk,
    input wire                     cfg_rst,

    input wire                     cfg_wvalid,
    output wire                    cfg_wready,
    input wire [31:0]              cfg_wdata,

    output wire                    cfg_rvalid,
    input wire                     cfg_rready,
    output wire [31:0]             cfg_rdata,

    output wire                    cfg_tx_rst
);

assign cfg_wready = 1'b1;
assign cfg_rvalid = 1'b1;

localparam DSPPHASE_WIDTH = 32;
localparam DSP_WIDTH = IN_WIDTH;

// Chain
// 1  DUC
// 2  Cordic
// 3  weighted mux
// 4  IQ corrector
// 5  DC corrector
// 6 -> 2:1 demux, incl. clock -> out_width shrink

genvar i;

wire ddr_reset;
wire ddr_reset_dac;
wire dac_clk;

// FIR filter
localparam STAGES    = 8;
localparam CFG_WIDTH = 8;

// FIR gearbox
wire [STAGES-1:0]   dsp_dspchain_prg_data;
wire                dsp_dspchain_prg_valid;
wire [STAGES-1:0]   igp_prg_data;
wire                igp_prg_strobe;

reg                 igp_dsp_cfg_rst;
wire                fir_dac_clk_dsp_uload_rst;

`define VFDVVDVDVDFDV
`ifdef VFDVVDVDVDFDV
synchronizer  #(.INIT(1)) dsp_uload_rst_sync (.clk(dac_clk),   .rst(1'b0), .a_in(igp_dsp_cfg_rst), .s_out(fir_dac_clk_dsp_uload_rst));

axis_cc_fifo #(
    .WIDTH(STAGES)
) igpdsprxcfg_fifo (
    .rx_clk(cfg_clk),
    .rx_rst(igp_dsp_cfg_rst),

    .s_rx_tdata(igp_prg_data),
    .s_rx_tvalid(igp_prg_strobe),
    .s_rx_tready(),

    .tx_clk(dac_clk),
    .tx_rst(fir_dac_clk_dsp_uload_rst),

    .m_tx_tdata(dsp_dspchain_prg_data),
    .m_tx_tvalid(dsp_dspchain_prg_valid),
    .m_tx_tready(1'b1)
);

wire phy_rdy;

wire [2 * IN_WIDTH * L_TX_CHANS-1:0]   duc_data;
wire                                   duc_valid;
wire                                   duc_ready = phy_rdy;

reconf_dsp_fir #(
    .IN_WIDTH(DSP_WIDTH),
    .CHANS(2 * L_TX_CHANS),
    .OUT_WIDTH(DSP_WIDTH),
    .CFG_WIDTH(CFG_WIDTH),
    .STAGES(STAGES),
    .ULTRA_SCALE(ULTRA_SCALE),
    .BACKPRESSURE(1'b1),
    .HAS_C_PORT(1'b0),
    .EXTERNAL_OMUX(1'b0)
) tx_reconf (
    .rst(ddr_reset_dac),
    .clk(dac_clk),

    .in_valid(i_valid),
    .in_data(i_data),
    .in_ready(i_ready),

    .out_valid(duc_valid),
    .out_data(duc_data),
    .out_ready(duc_ready),

    .cfg_valid(dsp_dspchain_prg_valid),
    .cfg_data(dsp_dspchain_prg_data)
);

localparam CORDIC_WIDTH = 17;

wire [L_TX_CHANS * DSPPHASE_WIDTH-1:0] dac_dsp_cordic_phase;
wire [L_TX_CHANS - 1:0]                ncoo_dacclk_valid;

// Use extra bit since rotattion can increase amplitutde up to sqrt(2)
wire [DSP_WIDTH:0]                ncoo_dacclk_data_i[0:L_TX_CHANS-1];
wire [DSP_WIDTH:0]                ncoo_dacclk_data_q[0:L_TX_CHANS-1];

wire nco_reset_dac;

generate
for (i = 0; i < L_TX_CHANS; i=i+1) begin: ncos
    reg  [DSPPHASE_WIDTH-1:0] nco_value;

    always @(posedge dac_clk) begin
        if (nco_reset_dac) begin
            nco_value <= 0;
        end else begin
            if (phy_rdy) begin
                nco_value <= nco_value + dac_dsp_cordic_phase[DSPPHASE_WIDTH * (i + 1) - 1:DSPPHASE_WIDTH * i];
            end
        end
    end

    wire [2 * IN_WIDTH - 1:0] locdata = duc_data[ 2 * IN_WIDTH * (i + 1) - 1 : 2 * IN_WIDTH * i];
    wire [DSP_WIDTH:0] ncoi_dacclk_data_i = { locdata[IN_WIDTH - 1],     locdata[IN_WIDTH - 1:0] };
    wire [DSP_WIDTH:0] ncoi_dacclk_data_q = { locdata[2 * IN_WIDTH - 1], locdata[2* IN_WIDTH - 1:IN_WIDTH] };
    wire [CORDIC_WIDTH - 1:0] nco_phase = { nco_value[31], nco_value[31], nco_value[31:34 - CORDIC_WIDTH] };

    wire [15:0] clamp_q;
    wire [15:0] clamp_i;

    if (CORDIC_WIDTH == 16) begin
        wire [31:0] ncoi_dacclk_data_native = { ncoi_dacclk_data_q[DSP_WIDTH:DSP_WIDTH - 15], ncoi_dacclk_data_i[DSP_WIDTH:DSP_WIDTH - 15] };
        wire [31:0] ncoo_dacclk_data_native;
    
        cordic_0 c0(
            .aclk(dac_clk),
            .aresetn(!ddr_reset_dac),
            .s_axis_phase_tdata( nco_phase ),
            .s_axis_phase_tvalid(duc_valid),
            .s_axis_cartesian_tdata(ncoi_dacclk_data_native),
            .s_axis_cartesian_tvalid(duc_valid),
            .m_axis_dout_tdata(ncoo_dacclk_data_native),
            .m_axis_dout_tvalid(ncoo_dacclk_valid[i])
        );
    
        // assign clamp_q = ( ncoo_dacclk_data_native[31:30] == 2'b10 ? 16'h8000 : ncoo_dacclk_data_native[31:30] == 2'b01 ? 16'h7fff : { ncoo_dacclk_data_native[30:16], 1'b0 } );
        // assign clamp_i = ( ncoo_dacclk_data_native[15:14] == 2'b10 ? 16'h8000 : ncoo_dacclk_data_native[15:14] == 2'b01 ? 16'h7fff : { ncoo_dacclk_data_native[14:0], 1'b0 } );
        assign ncoo_dacclk_data_i[i] = { ncoo_dacclk_data_native[15:0], 1'b0 };
        assign ncoo_dacclk_data_q[i] = { ncoo_dacclk_data_native[31:16], 1'b0 }; 
    end else begin
        wire [40:0] ncoi_dacclk_data_native = { ncoi_dacclk_data_q[DSP_WIDTH:DSP_WIDTH - 16], 7'h0, ncoi_dacclk_data_i[DSP_WIDTH:DSP_WIDTH - 16] };
        wire [40:0] ncoo_dacclk_data_native;
    
        cordic_1 c1(
            .aclk(dac_clk),
            .aresetn(!ddr_reset_dac),
            .s_axis_phase_tdata( nco_phase ),
            .s_axis_phase_tvalid(duc_valid),
            .s_axis_cartesian_tdata(ncoi_dacclk_data_native),
            .s_axis_cartesian_tvalid(duc_valid),
            .m_axis_dout_tdata(ncoo_dacclk_data_native),
            .m_axis_dout_tvalid(ncoo_dacclk_valid[i])
        );
    
        //assign clamp_q = ( ncoo_dacclk_data_native[40:39] == 2'b10 ? 16'h8000 : ncoo_dacclk_data_native[40:39] == 2'b01 ? 16'h7fff : ncoo_dacclk_data_native[39:24] );
        //assign clamp_i = ( ncoo_dacclk_data_native[16:15] == 2'b10 ? 16'h8000 : ncoo_dacclk_data_native[16:15] == 2'b01 ? 16'h7fff : ncoo_dacclk_data_native[15:0] );  
        
        assign ncoo_dacclk_data_i[i] = ncoo_dacclk_data_native[16:0];
        assign ncoo_dacclk_data_q[i] = ncoo_dacclk_data_native[40:24]; 
    end

    //assign ncoo_dacclk_data[i] = { clamp_q, clamp_i };
end
endgenerate


// Accumulate
localparam EX_BITS = $clog2(L_TX_CHANS);
localparam ACC_WIDTH = EX_BITS + DSP_WIDTH + 1; //18 bits for 2-NCO 16bit data
reg [ACC_WIDTH-1:0] acc_i;
reg [ACC_WIDTH-1:0] acc_q;

integer j;
always @* begin
    acc_i = 0;
    acc_q = 0;
    for (j = 0; j < L_TX_CHANS; j = j + 1) begin
        acc_i = $signed(acc_i) + $signed(ncoo_dacclk_data_i[j]);
        acc_q = $signed(acc_q) + $signed(ncoo_dacclk_data_q[j]);
    end
end

`else

wire [DSP_WIDTH - 1:0] acc_i = i_data[IN_WIDTH - 1:0];
wire [DSP_WIDTH - 1:0] acc_q = i_data[2 * IN_WIDTH - 1:IN_WIDTH];
assign i_ready = 1'b1;
    
`endif

// Divided clock
wire clk_div_o;
wire clk_io;

// IQ-corrector
localparam IQ_CFG_WIDTH = 24;
localparam TX_OUT_WIDTH = 16;

reg [IQ_CFG_WIDTH-1:0] cfg_amp_i;
reg [IQ_CFG_WIDTH-1:0] cfg_amp_q;
reg [IQ_CFG_WIDTH-1:0] cfg_tan_i;
reg [IQ_CFG_WIDTH-1:0] cfg_tan_q;

wire [TX_OUT_WIDTH-1:0] acc_corr_q;
wire [TX_OUT_WIDTH-1:0] acc_corr_i;
iq_corr #(.WIDTH(ACC_WIDTH), .CFG_WIDTH(IQ_CFG_WIDTH), .OUT_WIDTH(TX_OUT_WIDTH)) iq_corr (
    .clk(dac_clk),
    .rst(ddr_reset_dac),

    .cfg_amp({cfg_amp_q, cfg_amp_i}),
    .cfg_tan({cfg_tan_q, cfg_tan_i}),

    .in_data({acc_q, acc_i}),
    .in_valid(1'b1),

    .out_data({acc_corr_q, acc_corr_i}),
    .out_valid()
);

// DC-corrector and clamp
reg [11:0]                corr_i;
reg [11:0]                corr_q;
reg [TX_OUT_WIDTH - 1:0]  corrected_i;
reg [TX_OUT_WIDTH - 1:0]  corrected_q;
reg [TX_OUT_WIDTH - 2:0]  corrected_clp_i;
reg [TX_OUT_WIDTH - 2:0]  corrected_clp_q;

localparam CLAMP_MAX = { 1'b0, {(TX_OUT_WIDTH - 2){1'b1}}};
localparam CLAMP_MIN = { 1'b1, {(TX_OUT_WIDTH - 2){1'b0}}};

always @(posedge dac_clk) begin
    if (!i_enable) begin
        corrected_clp_i <= 0;
        corrected_clp_q <= 0;
    end else begin
        if (phy_rdy) begin
            corrected_i <= $signed(acc_corr_i) + $signed(corr_i);
            corrected_q <= $signed(acc_corr_q) + $signed(corr_q);
            
            corrected_clp_i <= ( corrected_i[TX_OUT_WIDTH-1-:2] == 2'b10 ? CLAMP_MAX : corrected_i[TX_OUT_WIDTH-1-:2] == 2'b01 ? CLAMP_MIN : corrected_i[TX_OUT_WIDTH - 2:0] );
            corrected_clp_q <= ( corrected_q[TX_OUT_WIDTH-1-:2] == 2'b10 ? CLAMP_MAX : corrected_q[TX_OUT_WIDTH-1-:2] == 2'b01 ? CLAMP_MIN : corrected_q[TX_OUT_WIDTH - 2:0] );  
      
        end
    end
end

wire [OUT_WIDTH-1:0] serder_i = corrected_clp_i[TX_OUT_WIDTH - 2 -:OUT_WIDTH];
wire [OUT_WIDTH-1:0] serder_q = corrected_clp_q[TX_OUT_WIDTH - 2 -:OUT_WIDTH];

// SERDES interface
assign phy_rdy = 1'b1;

BUFR #(.BUFR_DIVIDE("2")) mrcc_div2(
    .I(o_clk),
    .O(clk_div_o),
    .CE(1'b1),
    .CLR(1'b0)
);
BUFG clk_div_bufg(.I(clk_div_o), .O(dac_clk));

reg clk_phase = 1'b0;
wire ddr_reset_cfg_s;

`define CLK_D2
`ifdef CLK_D2
    wire full_clk;
    wire full_rst;
    wire [OUT_WIDTH-1:0] serder_i_f;
    wire [OUT_WIDTH-1:0] serder_q_f;
    wire                 serder_rdy;
    wire                 serder_valid;
  
    ila_1 ila_1(
        .clk(dac_clk),
        .probe0(i_enable),
        //.probe1(serder_i),
        //.probe2(serder_q),
        .probe1(acc_corr_i),
        .probe2(acc_corr_q),
//        .probe1(acc_i),
//        .probe2(acc_q),
        
        .probe3(ddr_reset_dac),
        .probe4(ddr_reset_cfg_s)
    );
  
    BUFG in_bufg(.I(o_clk), .O(full_clk));
    
    synchronizer  #(.INIT(1)) ddr_reset_full_clk(.clk(full_clk), .rst(1'b0), .a_in(ddr_reset_dac),  .s_out(full_rst));
    
    axis_cc_fifo #(
        .WIDTH(OUT_WIDTH * 2),
        .DEEP_BITS(3)
    ) dsc_to_usrclk_fifo (
        .rx_clk(dac_clk),
        .rx_rst(ddr_reset_dac),
    
        .s_rx_tdata( { serder_q, serder_i }  ),
        .s_rx_tvalid( i_enable ),
        .s_rx_tready(  ),
    
        .tx_clk(full_clk),
        .tx_rst(full_rst),
    
        .m_tx_tdata( {serder_q_f, serder_i_f }),
        .m_tx_tvalid(serder_valid),
        .m_tx_tready(serder_rdy)
    );

    reg [11:0]  txd_iob_data_s;
    reg         txd_iob_iqsel_s;
    
    (* IOB = "true" *) reg [11:0] lms_txd_r;
    (* IOB = "true" *) reg        lms_txiqsel_r;

    always @(posedge full_clk) begin
        lms_txd_r     <= txd_iob_data_s;
        lms_txiqsel_r <= txd_iob_iqsel_s;
        
        if (~serder_valid) begin
            txd_iob_data_s  <= 0;
            txd_iob_iqsel_s <= 0;
        end else begin
            txd_iob_iqsel_s <= ~txd_iob_iqsel_s;
            txd_iob_data_s  <= (txd_iob_iqsel_s) ? serder_q_f : serder_i_f;
        end
    end
    
    assign serder_rdy = txd_iob_iqsel_s; 

    assign o_data     = lms_txd_r;
    assign o_sel      = lms_txiqsel_r;
    ODDR txclk_f_oddr( .Q(o_clk_fwd), .D1(1'b0), .D2(1'b1), .CE(1'b1), .C(full_clk), .R(1'b0));
    

`else
BUFIO in_bufio(.I(o_clk), .O(clk_io));

wire phy_tx_ce = 1'b1;


generate
for (i = 0; i < OUT_WIDTH; i=i+1) begin: dlvds
    OSERDESE2 #(
        .DATA_RATE_OQ("DDR"),
        .DATA_RATE_TQ("DDR"),
        .DATA_WIDTH(4)
    ) oserdese2_lms_diq_tx(
        .RST(ddr_reset),
        .D1(serder_i[i]),
        .D2(serder_i[i]),
        .D3(serder_q[i]), //serder_q[i]),
        .D4(serder_q[i]),
        .D5(),
        .D6(),
        .D7(),
        .D8(),
        .CLK(clk_io),
        .CLKDIV(clk_div_o),
        .OCE(phy_tx_ce),
        .T1(),
        .T2(),
        .T3(),
        .T4(),
        .TCE(1'b1),
        .TBYTEIN(),
        .TBYTEOUT(),
        .OQ(o_data[i]),
        .TQ(),
        .OFB(),
        .TFB(),
        .SHIFTOUT1(),
        .SHIFTOUT2(),
        .SHIFTIN1(),
        .SHIFTIN2()
    );
end
endgenerate

wire [1:0] cfg_port_iqsel = NEG_POLARITY ? 2'b10 : 2'b01;

OSERDESE2 #(
    .DATA_RATE_OQ("DDR"),
    .DATA_RATE_TQ("DDR"),
    .DATA_WIDTH(4)
) oserdese2_lms_iqsel_tx(
    .RST(ddr_reset),
    .D1(cfg_port_iqsel[0]),
    .D2(cfg_port_iqsel[0]),
    .D3(cfg_port_iqsel[1]), //cfg_port_iqsel[1]),
    .D4(cfg_port_iqsel[1]),
    .D5(),
    .D6(),
    .D7(),
    .D8(),
    .CLK(clk_io),
    .CLKDIV(clk_div_o),
    .OCE(phy_tx_ce),
    .T1(),
    .T2(),
    .T3(),
    .T4(),
    .TCE(1'b1),
    .TBYTEIN(),
    .TBYTEOUT(),
    .OQ(o_sel),
    .TQ(),
    .OFB(),
    .TFB(),
    .SHIFTOUT1(),
    .SHIFTOUT2(),
    .SHIFTIN1(),
    .SHIFTIN2()
);

OSERDESE2 #(
    .DATA_RATE_OQ("DDR"),
    .DATA_RATE_TQ("DDR"),
    .DATA_WIDTH(4)
) oserdese2_lms_fwdclk_tx(
    .RST(ddr_reset),
    .D1(!clk_phase),
    .D2(clk_phase),
    .D3(!clk_phase),
    .D4(clk_phase),
    .D5(),
    .D6(),
    .D7(),
    .D8(),
    .CLK(clk_io),
    .CLKDIV(clk_div_o),
    .OCE(phy_tx_ce),
    .T1(),
    .T2(),
    .T3(),
    .T4(),
    .TCE(1'b1),
    .TBYTEIN(),
    .TBYTEOUT(),
    .OQ(o_clk_fwd),
    .TQ(),
    .OFB(),
    .TFB(),
    .SHIFTOUT1(),
    .SHIFTOUT2(),
    .SHIFTIN1(),
    .SHIFTIN2()
);

`endif

reg [15:0] reg_nco0_l;
reg [15:0] reg_nco0_h;
reg [15:0] reg_nco1_l;
reg [15:0] reg_nco1_h;

assign dac_dsp_cordic_phase = { reg_nco1_h, reg_nco1_l, reg_nco0_h, reg_nco0_l };

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
reg ddr_reset_cfg;
reg ddr_resete_cfg;
reg ddr_reset_nco_cfg;

synchronizer  #(.INIT(1)) ddr_reset_sync     (.clk(clk_div_o),   .rst(1'b0), .a_in(ddr_reset_cfg),  .s_out(ddr_reset));
synchronizer  #(.INIT(1)) ddr_reset_adc_sync (.clk(dac_clk),     .rst(1'b0), .a_in(ddr_reset_cfg),  .s_out(ddr_reset_dac));

synchronizer  #(.INIT(1)) ddr_reset_ext_sync (.clk(dac_clk),     .rst(1'b0), .a_in(ddr_resete_cfg), .s_out(i_rst));


synchronizer  #(.INIT(1)) ddr_reset_nco_sync (.clk(dac_clk),     .rst(ddr_reset_cfg), .a_in(ddr_reset_nco_cfg),  .s_out(nco_reset_dac));

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
                        clk_phase       <= cfg_wdata[7];
                        ddr_reset_nco_cfg <= cfg_wdata[8]; // NCO phase reset
                    end
                    CFG_REG_DCCTRL: begin
                        corr_i  <= cfg_wdata[11:0];
                        corr_q  <= cfg_wdata[23:12];
                    end
                    
                    CFG_REG_IQIMB_0: cfg_amp_i <= cfg_wdata;
                    CFG_REG_IQIMB_1: cfg_amp_q <= cfg_wdata;
                    CFG_REG_IQIMB_2: cfg_tan_i <= cfg_wdata;
                    CFG_REG_IQIMB_3: cfg_tan_q <= cfg_wdata;

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


assign cfg_rdata  = 0;
assign i_clk      = dac_clk;
assign cfg_tx_rst = ddr_resete_cfg;

assign ddr_reset_cfg_s = ddr_reset_cfg;
endmodule

`default_nettype wire