module dsp_dc_corr #(
    parameter AVG_BITS = 21,
    parameter CHANS    = 2,
    parameter WIDTH    = 12
)(
    input clk,
    input rst,

    input                            cfg_dc_corr_en,

    input [WIDTH * CHANS - 1:0]      in_data,
    input                            in_valid,

    output reg [WIDTH * CHANS - 1:0] out_data,
    output reg                       out_valid,

    output [WIDTH * CHANS - 1:0]     corr_data
);

reg [AVG_BITS:0]              wnd_counter;
wire                          wnd_update = wnd_counter[AVG_BITS];

always @(posedge clk) begin
    if (rst || !cfg_dc_corr_en) begin
        wnd_counter <= 0;
    end else if (in_valid) begin
        wnd_counter <= wnd_counter + 1'b1;
        if (wnd_counter[AVG_BITS]) begin
            wnd_counter[AVG_BITS] <= 1'b0;
        end
    end
end

always @(posedge clk) begin
    out_valid <= in_valid;
end

genvar i;
generate
for (i = 0; i < CHANS; i = i + 1) begin
    reg [WIDTH + AVG_BITS - 1:0] in_acc;
    reg [WIDTH - 1:0]            corr;
    wire [WIDTH - 1:0]           in_data_c    = in_data[WIDTH * (i + 1) - 1:WIDTH * i];
    wire                         in_sign      = in_data_c[WIDTH - 1];
    wire [WIDTH - 1:0]           in_acc_trunc = in_acc[WIDTH + AVG_BITS - 1:AVG_BITS];
    
    always @(posedge clk) begin
        if (rst || !cfg_dc_corr_en) begin
            in_acc <= 0;
            corr   <= 0;
        end else if (in_valid) begin
            in_acc <= in_acc + {{AVG_BITS{in_sign}}, in_data_c};
            if (wnd_update) begin
                in_acc <= 0;
                corr   <= in_acc_trunc;
            end
        end
    end
    
    always @(posedge clk) begin
        if (in_valid) begin 
            out_data[WIDTH * (i + 1) - 1:WIDTH * i]     <= in_data_c - corr;
        end
    end
    
    assign corr_data[WIDTH * (i + 1) - 1:WIDTH * i] = corr;
end
endgenerate


endmodule
