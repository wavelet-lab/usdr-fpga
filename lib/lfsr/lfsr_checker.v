module lfsr_checker #(
    parameter [4:0] MAX_ERROR = 16, // Max error until full resync
	parameter       DATA_WIDTH = 14,
	parameter       LFSR_POLY = 23'b100_0010_0000_0000_0000_0000,
	parameter       LFSR_LEN = 23,
	parameter       STAT_WIDTH = 32,
	parameter [0:0] XOR_CONST = 1'b0
)(
	input clk,
	input rst,

	input en,

	input [DATA_WIDTH - 1: 0] data_i,

	output reg                    alarm_o,
    output reg [STAT_WIDTH - 1:0] sync_error,
    output reg [STAT_WIDTH - 1:0] lost_error,
    output reg [STAT_WIDTH - 1:0] ber_error
);

localparam [3:0]
	ST_NONE = 0,
	ST_SYNC_START = 1,
	ST_SYNC_DONE = 2 + LFSR_LEN - DATA_WIDTH;

reg [3:0]          state;
reg [LFSR_LEN-1:0] lfsr;
reg [4:0]          data_error;

wire lfsr_bit_next = ^(lfsr & LFSR_POLY) ^ XOR_CONST;

wire [DATA_WIDTH - 1:0] check_lfsr = { lfsr[DATA_WIDTH - 2:0], lfsr_bit_next };

reg sig_dataerror;
reg sig_datarst;
reg sig_syncerror;

always @(posedge clk) begin
	if (rst) begin
		state         <= ST_NONE;
		alarm_o     <= 0;

		sig_dataerror <= 0;
		sig_datarst   <= 1'b1;
		sig_syncerror <= 0;
	end else begin

		sig_dataerror <= 0;
		sig_datarst   <= 0;
		sig_syncerror <= 0;

		if (sig_datarst) begin
		  data_error <= 0;
		end else if (sig_dataerror) begin
		  alarm_o    <= 1'b1;
		  data_error <= data_error + 1'b1;
		  ber_error  <= ber_error + 1'b1;

          if (data_error[4]) begin
            lost_error    <= lost_error + 1'b1;
            state         <= ST_SYNC_START;
		  end
		end

		if (sig_syncerror) begin
	      sync_error    <= sync_error + 1'b1;

	      state         <= ST_SYNC_START;
	      alarm_o       <= 1'b1;
		end

		if (!en) begin
			state <= ST_NONE;
		end else begin
			if (state == ST_NONE) begin
				alarm_o    <= 0;
				sync_error <= 0;
				ber_error  <= 0;
				lost_error <= 0;
				state      <= ST_SYNC_START;

			end else if (state == ST_SYNC_START) begin
				state                  <= state + 1'b1;
				lfsr                   <= data_i;
				//lfsr[DATA_WIDTH - 1:0] <= data_i;
				sig_datarst            <= 1'b1;

			end else if (state < ST_SYNC_DONE) begin
				state                  <= state + 1'b1;
				lfsr                   <= { lfsr[LFSR_LEN - 2:0], data_i[0] };

				if (lfsr[DATA_WIDTH - 2:0] != data_i[DATA_WIDTH - 1:1]) begin
					sig_syncerror <= 1'b1;
				end
			end else begin
				lfsr                   <= { lfsr[LFSR_LEN - 2:0], lfsr_bit_next };

				if (check_lfsr != data_i[DATA_WIDTH - 1:0]) begin
					sig_dataerror <= 1'b1;
				end else begin
					sig_datarst  <= 1'b1;
				end
			end
		end
	end
end


endmodule

