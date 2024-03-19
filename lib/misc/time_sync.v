// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module time_sync #(
    parameter TIMER_WIDTH = 48 
)(
    input      rx_streamingready,
    input      tx_streamingready,
    output reg tx_timer_en,
    output reg rx_timer_en,

    // sync events
    input            onepps_raw,

    // control
    input            igp_clk,
    input            igp_reset,
    input            igp_timecmd_valid,
    input [15:0]     igp_timecmd_data,
    output           igp_timecmd_ready,

    output           sysref_gen
);

assign igp_timecmd_ready = 1'b1;


wire opepps_igpclk;

synchronizer #(.INIT(1), .ASYNC_RESET(1)) flashonepps_raw_sync_reset_reg(
    .clk(igp_clk),
    .rst(igp_reset),
    .a_in(onepps_raw),
    .s_out(opepps_igpclk)
);


localparam CNTWIDTH = 20;

reg [CNTWIDTH:0] delay_cnt;
reg [2:0]        st_type;
reg              st_en_gen;

reg [11:0] genclk;
always @(posedge igp_clk) begin
    if (igp_reset || !st_en_gen) begin
        genclk <= 0;
    end else begin
        genclk <= genclk + 1'b1;
    end
end

assign sysref_gen = genclk[11];

// Sync types
localparam [2:0]
    ST_STOP         = 0, 
    ST_ONEPPS       = 1,
    ST_ADCACT       = 2,
    ST_DACACT       = 3,
    ST_ADCANDDACACT = 4,
    ST_ADCORDACACT  = 5,
    ST_DELAYADCDAC  = 6,
    ST_FREERUN      = 7;

always @(posedge igp_clk) begin
    if (igp_reset) begin
        st_type     <= ST_STOP;
        tx_timer_en <= 1'b0;
        rx_timer_en <= 1'b0;
        delay_cnt   <= 0;
        st_en_gen   <= 0;
    end else begin
        case (st_type)
        ST_STOP: begin
            tx_timer_en <= 1'b0;
            rx_timer_en <= 1'b0;
        end

        ST_ONEPPS: begin
            if (opepps_igpclk) begin
                tx_timer_en <= 1'b1;
                rx_timer_en <= 1'b1;

                st_en_gen   <= 1'b0;
            end
        end

        ST_ADCACT: begin
            if (rx_streamingready) begin
                tx_timer_en <= 1'b1;
                rx_timer_en <= 1'b1;
            end
        end

        ST_DACACT: begin
            if (tx_streamingready) begin
                tx_timer_en <= 1'b1;
                rx_timer_en <= 1'b1;
            end
        end

        ST_ADCANDDACACT: begin
            if (rx_streamingready && tx_streamingready) begin
                tx_timer_en <= 1'b1;
                rx_timer_en <= 1'b1;
            end
        end

        ST_ADCORDACACT: begin
            if (rx_streamingready || tx_streamingready) begin
                tx_timer_en <= 1'b1;
                rx_timer_en <= 1'b1;
            end
        end

        ST_DELAYADCDAC: begin
            tx_timer_en <= 1'b0;
            rx_timer_en <= 1'b0;
            if (rx_streamingready && tx_streamingready) begin
                delay_cnt <= delay_cnt + 1'b1;
                if (delay_cnt[CNTWIDTH]) begin
                    st_type <= ST_ADCANDDACACT;
                end
            end
        end

        ST_FREERUN: begin
            tx_timer_en <= 1'b1;
            rx_timer_en <= 1'b1;
        end
        endcase

        if (igp_timecmd_valid && igp_timecmd_ready) begin
            st_en_gen   <= igp_timecmd_data[3];
            st_type     <= igp_timecmd_data[2:0];
            delay_cnt   <= 0;
        end
    end
end




endmodule


