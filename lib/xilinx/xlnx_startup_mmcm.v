// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module xlnx_startup_mmcm #(
    parameter PCIE_BITS = 2
)(
    input  cfg_mclk,
    output startup_mode_pcie,

    output pipe_mmcm_rst_n,

    input  usb2_clk_ready,
    input  pipe_mmcm_lock_out,

    output brng_usb_logic_reset,
    output brng_phy_nrst,

    output brng_usb_user_reset,
    output usb2_fsm_done,

    output [4:0] debug_state
);

// Bring up PCIe/USB selection FSM
////////////////////////////////////////////////////////////////////
localparam [0:0]  SMODE_PCIE = 1'b0,
                  SMODE_USB2 = 1'b1;

localparam NOLOCK_BITS = 28;

reg                  startup_mode = 1'b0; // 0 - PCIE; 1 - USB2
reg                  pipe_mmcm_rst_n_r = 1'b1;

reg [NOLOCK_BITS-1:0] pcie_no_lock_cntr = 0;
reg                  brng_usb_clk_en_r = 1'b0;
reg                  brng_usb_nrst_r   = 1'b0;
reg                  brng_usb_user_reset_r = 1'b1;
reg                  brng_usb_logic_reset_r = 1'b1;

localparam           IUSB2_DLY = 8;
wire [4:0]           fsm_usb2_reinit = pcie_no_lock_cntr[IUSB2_DLY + 4:IUSB2_DLY];
localparam [4:0]     FUR_USBCLK_EN          = 5'd0,
                     FUR_PHYUSB_RESETB      = 5'd1+3,
                     FUR_MCMS_RESET_ACT     = 5'd2+3,
                     FUR_MCMS_RESET_RELEASE = 5'd3+3,
                     FUR_MCMS_WAIT_LOCK_0   = 5'd4+3,
                     FUR_MCMS_WAIT_LOCK_1   = 5'd5+3,
                     FUR_MCMS_WAIT_LOCK_2   = 5'd6+3,
                     FUR_MCMS_WAIT_LOCK_3   = 5'd7+3,
                     FUR_PHY_LOGIC_RST_H    = 5'd8+3,
                     FUR_PHY_LOGIC_RST_L    = 5'd9+3,
                     FUR_END                = 5'd10+3;

assign debug_state = fsm_usb2_reinit;

always @(posedge cfg_mclk) begin
    pcie_no_lock_cntr <= pcie_no_lock_cntr + 1'b1;

    if (pcie_no_lock_cntr[NOLOCK_BITS-1] == 1'b1 && startup_mode == SMODE_PCIE && ~pipe_mmcm_lock_out) begin
        startup_mode <= SMODE_USB2;
    end

    if (pcie_no_lock_cntr[NOLOCK_BITS-1] == 1'b1 && startup_mode == SMODE_USB2) begin
        case (fsm_usb2_reinit)
        FUR_USBCLK_EN: begin
            brng_usb_clk_en_r   <= 1'b1;
            if (!usb2_clk_ready) begin
                pcie_no_lock_cntr[IUSB2_DLY + 4:IUSB2_DLY]    <= FUR_USBCLK_EN;
            end
        end
        FUR_PHYUSB_RESETB:       brng_usb_nrst_r     <= 1'b1;
        FUR_MCMS_RESET_ACT:      pipe_mmcm_rst_n_r   <= 1'b0;
        FUR_MCMS_RESET_RELEASE:  pipe_mmcm_rst_n_r   <= 1'b1;
        FUR_MCMS_WAIT_LOCK_0, FUR_MCMS_WAIT_LOCK_1, FUR_MCMS_WAIT_LOCK_2, FUR_MCMS_WAIT_LOCK_3: begin
            if (pipe_mmcm_lock_out && fsm_usb2_reinit == FUR_MCMS_WAIT_LOCK_3) begin
                brng_usb_user_reset_r <= 1'b0;
            end
            if (fsm_usb2_reinit == FUR_MCMS_WAIT_LOCK_2 && ~pipe_mmcm_lock_out) begin
                pcie_no_lock_cntr[IUSB2_DLY + 4:IUSB2_DLY]    <= FUR_MCMS_WAIT_LOCK_0;
            end
        end
        FUR_PHY_LOGIC_RST_H: brng_usb_logic_reset_r <= 1'b1;
        FUR_PHY_LOGIC_RST_L: brng_usb_logic_reset_r <= 1'b0;
        FUR_END:             pcie_no_lock_cntr[IUSB2_DLY + 4:IUSB2_DLY]    <= pcie_no_lock_cntr[IUSB2_DLY + 4:IUSB2_DLY];
        endcase
    end
end

assign startup_mode_pcie    = (startup_mode == SMODE_PCIE);
assign brng_usb_logic_reset = brng_usb_logic_reset_r;
assign brng_phy_nrst        = brng_usb_nrst_r;
assign pipe_mmcm_rst_n      = pipe_mmcm_rst_n_r;
assign brng_usb_user_reset  = brng_usb_user_reset_r;

assign usb2_fsm_done        = fsm_usb2_reinit == FUR_END;

endmodule
