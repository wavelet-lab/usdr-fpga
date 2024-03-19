// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module event_fifo_gen #(
	parameter EVENT_COUNT_BITS = 4,
	parameter ADDR_WIDTH = 6,
	parameter DATA_WIDTH = 32,
	parameter AUX_WIDTH = 2,
	parameter [1023:0] A_EVENT_ADDRS = 0,
	parameter [1023:0] A_EVENT_LEN   = 0
)(
    input clk,
    input rst,

    // Event notification
    input      [AUX_WIDTH - 1:0]          s_evno_user,
    input      [EVENT_COUNT_BITS - 1:0]   s_evno_data, 
    input                                 s_evno_valid,
    output                                s_evno_ready,

    // Readback data
    output reg [ADDR_WIDTH - 1:2]         m_al_araddr,
    output reg                            m_al_arvalid,
    input                                 m_al_arready,
    input [DATA_WIDTH - 1:0]              m_al_rdata,
    input                                 m_al_rvalid,
    output                                m_al_rready,

    // FIFO out
    output reg                            m_evd_last,
    output reg [DATA_WIDTH - 1:0]         m_evd_data,
    output reg                            m_evd_valid,
    input                                 m_evd_ready,

    // Event read confirmation
    output reg [EVENT_COUNT_BITS - 1:0]   m_cnfno_data,
    output reg                            m_cnfno_valid,
    input                                 m_cnfno_ready
);

// First packet in event
// b31 .. b16  seqno
// b15 .. b12  eventsz
// b11 .. b6   aux
// b5  .. b0   eventno


// user bit format
//   => aux | event_no

localparam REQ_LEN_BITS  = 2;

localparam [0:0] 
	ST_INIT = 0,
	ST_ADDR_REQ = 1;
reg [0:0]                   state;

reg [REQ_LEN_BITS - 1:0]    addr_req_cnt;
reg [REQ_LEN_BITS - 1:0]    data_req_cnt;

wire [31:0] cfg_addr;
wire [7:0]  cfg_len;
genvar i;
generate
for (i = 0; i < 32; i=i+1) begin
	assign cfg_addr[i] = A_EVENT_ADDRS[32 * s_evno_data + i];
end
for (i = 0; i < 8; i=i+1) begin
	assign cfg_len[i] = A_EVENT_LEN[32 * s_evno_data + i];
end
endgenerate

assign s_evno_ready = (state == ST_INIT) && ~m_evd_valid && (m_cnfno_ready || ~m_cnfno_valid);

reg [EVENT_COUNT_BITS - 1:0] event_no_r;
assign m_al_rready = (m_evd_ready || ~m_evd_valid);

reg [15:0] seqnum;

always @(posedge clk) begin
	if (rst) begin
		state         <= ST_INIT;
		m_al_arvalid  <= 0;
		m_evd_valid   <= 0;
		m_cnfno_valid <= 0;

		seqnum        <= 0;
	end else begin
	
		if (m_al_arvalid && m_al_arready) begin
			if (addr_req_cnt == 0) begin
				m_al_arvalid <= 0;
			end else begin
			    addr_req_cnt <= addr_req_cnt - 1'b1;
				m_al_araddr  <= m_al_araddr + 1'b1;
			end
		end
		
		if (m_evd_valid && m_evd_ready) begin
			m_evd_valid <= 0;
		end
		
		if (m_cnfno_valid && m_cnfno_ready) begin
			m_cnfno_valid <= 0;
		end
		
		case (state)
		ST_INIT: begin
		  if (s_evno_valid && s_evno_ready) begin
			m_al_araddr      <= cfg_addr[(ADDR_WIDTH - 2) - 1:0];
			m_al_arvalid     <= 1'b1;
			addr_req_cnt     <= cfg_len[REQ_LEN_BITS - 1:0];
			data_req_cnt     <= cfg_len[REQ_LEN_BITS - 1:0];
			event_no_r       <= s_evno_data;
			state            <= ST_ADDR_REQ;
			
			m_evd_data[31:16]<= seqnum;
			m_evd_data[15:12]<= cfg_len[REQ_LEN_BITS - 1:0];
			m_evd_data[11:6] <= s_evno_user;
			m_evd_data[5:0]  <= s_evno_data;
			m_evd_valid      <= 1'b1;
			m_evd_last       <= 1'b0;
			seqnum           <= seqnum + 1'b1;
          end
		end
		
		ST_ADDR_REQ: begin
			if (m_al_rvalid && m_al_rready) begin
				m_evd_data   <= m_al_rdata;
				m_evd_valid  <= 1'b1;
				m_evd_last   <= (data_req_cnt == 0);

				data_req_cnt <= data_req_cnt - 1'b1;
				if (data_req_cnt == 0) begin
					state         <= ST_INIT;
					
					m_cnfno_data  <= event_no_r;
					m_cnfno_valid <= 1'b1;
				end
			end
		end
		endcase
	
	end
end



endmodule

