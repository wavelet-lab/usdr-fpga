// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module pci_msi_intrrupt #(
    parameter COUNT = 16,
    parameter COUNT_BITS = 4
) (
    input clk,
    input rst,

    // Xilinx PCI-e interface
    input            interrupt_msi_enabled,
    input            interrupt_rdy,

    output reg       interrupt,
    output reg [7:0] interrupt_num,
    input      [2:0] interrupt_mmenable,

    output     [4:0] cap_interrupt_msgnum,
   
    // User Interrupr interfcae
    input [COUNT_BITS - 1:0] s_int_data,
    input                    s_int_valid,
    output                   s_int_ready
);
assign cap_interrupt_msgnum =
  (interrupt_mmenable == 3'b000 && (COUNT > 1))  ? { 5'b00001 } :
  (interrupt_mmenable == 3'b001 && (COUNT > 2))  ? { 5'b00010 } :
  (interrupt_mmenable == 3'b010 && (COUNT > 4))  ? { 5'b00100 } :
  (interrupt_mmenable == 3'b011 && (COUNT > 8))  ? { 5'b01000 } :
  (interrupt_mmenable == 3'b100 && (COUNT > 16)) ? { 5'b10000 } :
                                                   { COUNT[4:0] };
wire [4:0] msi_num = s_int_data;
wire [5:0] msi_num_fit =
  (interrupt_mmenable == 3'b000 && (COUNT > 1))                    ? 6'b1_00000 :
  (interrupt_mmenable == 3'b001 && (COUNT > 2)  && (msi_num > 0))  ? 6'b1_00001 :
  (interrupt_mmenable == 3'b010 && (COUNT > 4)  && (msi_num > 2))  ? 6'b1_00011 :
  (interrupt_mmenable == 3'b011 && (COUNT > 8)  && (msi_num > 6))  ? 6'b1_00111 :
  (interrupt_mmenable == 3'b100 && (COUNT > 16) && (msi_num > 14)) ? 6'b1_01111 :
                                                                     { 1'b0, msi_num };
                                                                     
wire [4:0] msi_num_gen = msi_num_fit[4:0];

reg [0:0]              pcie_int_state;

localparam PCIE_INT_IDLE          = 0;
localparam PCIE_INT_WAIT_REL_H    = 1; // for MSI interrupt

assign s_int_ready = (pcie_int_state == PCIE_INT_IDLE);

always @(posedge clk) begin
  if (rst) begin
    pcie_int_state   <= PCIE_INT_IDLE;
    interrupt        <= 0;
    interrupt_num    <= 0;
  end else begin

    case (pcie_int_state)
      PCIE_INT_IDLE: begin
        if (s_int_valid && s_int_ready) begin
          if (interrupt_msi_enabled) begin
            interrupt        <= 1'b1;
            interrupt_num    <= msi_num_gen;
            pcie_int_state   <= PCIE_INT_WAIT_REL_H;
          end else begin
            // Ignore for now
          end
        end
      end

      PCIE_INT_WAIT_REL_H: begin
        if (interrupt_rdy) begin
          interrupt       <= 1'b0;
        end

        if (~interrupt) begin
          pcie_int_state  <= PCIE_INT_IDLE;
        end
      end
    endcase
    
  end
end



endmodule
