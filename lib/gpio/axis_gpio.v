// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
// USDR PROJECT
// CLEAN MODULE
//
module axis_gpio #(
    parameter WIDTH           = 15,
    parameter _INT_WIDTH      = (WIDTH > 15) ? 15 : WIDTH,
    parameter DEF_CFG_GPIO    = 30'h0,
    parameter DEF_GPIO_OUT    = 15'h0,
    parameter ALT_CFG0_OE     = 15'h0
)(
    input                     clk,
    input                     rst,

    // UL Write channel
    input [31:0]              axis_wdata,
    input                     axis_wvalid,
    output                    axis_wready,

    // Output data readback stream
    output [31:0]             axis_rdata,
    output                    axis_rvalid,
    input                     axis_rready,

    input  [_INT_WIDTH-1:0]   gpi_data,
    output [_INT_WIDTH-1:0]   gpo_data,
    output [_INT_WIDTH-1:0]   gpo_data_oe,
    output [_INT_WIDTH-1:0]   gpio_altf0_valid,

    // Alternative functions
    output  [_INT_WIDTH-1:0]  alt0_in,
    input   [_INT_WIDTH-1:0]  alt0_out,
    input   [_INT_WIDTH-1:0]  alt0_out_oe
);

reg [2 * _INT_WIDTH - 1:0]    cfg_gpioconf;
reg [_INT_WIDTH-1:0]          gpio_odata;

wire [1:0] gpio_reg = axis_wdata[31:30];
localparam [1:0]
    GPIO_OUT_REG = 2'b00,
    GPIO_BIT_SET = 2'b01,
    GPIO_BIT_CLR = 2'b10,
    GPIO_CFG_REG = 2'b11;

wire [14:0] mask = axis_wdata[29:15];
wire [14:0] data = axis_wdata[14:0];

wire [3:0] cidx  = axis_wdata[29:26];
wire [1:0] cfgd  = axis_wdata[1:0];


always @(posedge clk) begin
    if (rst) begin
        cfg_gpioconf  <= DEF_CFG_GPIO;
        gpio_odata    <= DEF_GPIO_OUT;
    end else begin
        if (axis_wready && axis_wvalid) begin
            case (gpio_reg)
            GPIO_OUT_REG: begin
                gpio_odata    <= (~mask & gpio_odata) | (mask & data);
            end
            GPIO_BIT_SET: begin
                gpio_odata    <= gpio_odata | (mask & data);
            end
            GPIO_BIT_CLR: begin
                gpio_odata    <= gpio_odata & (~(mask & data));
            end
            GPIO_CFG_REG: begin
                cfg_gpioconf[2 * cidx + 0] <= cfgd[0];
                cfg_gpioconf[2 * cidx + 1] <= cfgd[1];
            end
            endcase
        end

    end
end

// config
// 0   -- input
// 1   -- output
// 2   -- altconfig
// 3   -- sampled input
localparam [1:0]
    GPIOCFG_INPUT  = 2'b00,
    GPIOCFG_OUTPUT = 2'b01,
    GPIOCFG_ALT0   = 2'b10;

genvar i;
generate
for (i = 0; i < _INT_WIDTH; i=i+1) begin: route
    wire [1:0] cfg_pin = { cfg_gpioconf[2 * i + 1:2 * i] };

    assign gpo_data[i]         = (cfg_pin == GPIOCFG_ALT0) ? alt0_out[i]    : gpio_odata[i];
    assign gpo_data_oe[i]      = (cfg_pin == GPIOCFG_ALT0) ? alt0_out_oe[i] : (cfg_pin == GPIOCFG_OUTPUT) ? 1'b1 : 1'b0;
    assign gpio_altf0_valid[i] = (cfg_pin == GPIOCFG_ALT0);
end
endgenerate

assign axis_rvalid = 1'b1;
assign axis_rdata  = { {(16-_INT_WIDTH){1'b0}}, gpio_odata, {(16-_INT_WIDTH){1'b0}}, gpi_data};
assign alt0_in     = gpi_data;

assign axis_wready = 1'b1;

endmodule
