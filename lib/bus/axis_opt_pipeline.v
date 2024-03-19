// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module axis_opt_pipeline #(
  parameter         WIDTH     = 32,
  parameter         PIPELINE  = 1,
  parameter         REG_READY = 0
) (
  input                            clk,
  input                            rst,

  input [WIDTH-1:0]                s_rx_tdata,
  input                            s_rx_tvalid,
  output                           s_rx_tready,

  output [WIDTH-1:0]               m_tx_tdata,
  output                           m_tx_tvalid,
  input                            m_tx_tready
);

generate
if (PIPELINE && REG_READY) begin
	reg [WIDTH - 1:0] data;
	reg               valid;

	reg [WIDTH - 1:0] ext_data;
	reg               ext_valid;

	always @(posedge clk) begin
		if (rst) begin
			valid     <= 0;
			ext_valid <= 0;
		end else begin
			if (s_rx_tready) begin
				if (s_rx_tvalid) begin
					data  <= s_rx_tdata;
				end
				valid <= s_rx_tvalid;
				if (~m_tx_tready) begin
					ext_data  <= data;
					ext_valid <= valid;
				end
			end
		end
	end

	assign s_rx_tready = ~ext_valid;
	assign m_tx_tdata  = (ext_valid) ? ext_data : data;
	assign m_tx_tvalid = valid || ext_valid;
end else if (PIPELINE) begin
	reg [WIDTH - 1:0] data;
	reg               valid;
	wire              ready = m_tx_tready;

	always @(posedge clk) begin
		if (rst) begin
			valid <= 0;
		end else begin
			if (ready && valid) begin
				valid <= 0;
			end
			
			if (s_rx_tvalid && s_rx_tready) begin
				data  <= s_rx_tdata;
				valid <= 1'b1;
			end
		end
	end

	assign s_rx_tready = (~valid || ready);
	assign m_tx_tdata  = data;
	assign m_tx_tvalid = valid;
end else begin
	assign s_rx_tready = m_tx_tready;
	assign m_tx_tdata  = s_rx_tdata;
	assign m_tx_tvalid = s_rx_tvalid;
end
endgenerate


endmodule

