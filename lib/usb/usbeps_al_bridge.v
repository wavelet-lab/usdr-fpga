module usbeps_al_bridge #(
    parameter AL_BUS_WIDTH = 12
)(
    input clk,
    input rst,

    // AXIS_M
    output                     m_axis_endpoint_rx_valid,
    input                      m_axis_endpoint_rx_ready,
    output                     m_axis_endpoint_rx_last,
    output  [32-1:0]           m_axis_endpoint_rx_data,
    output  [4-1:0]            m_axis_endpoint_rx_keep,

    // AXIS_S
    output                     s_axis_endpoint_tx_busy, //(busy flag -- do not try to push data)
    input                      s_axis_endpoint_tx_valid,
    output                     s_axis_endpoint_tx_ready,
    input                      s_axis_endpoint_tx_last,
    input [32-1:0]             s_axis_endpoint_tx_data,
    input [4-1:0]              s_axis_endpoint_tx_keep,


    output reg [AL_BUS_WIDTH-1:2]  m_al_waddr,
    output [31:0]                  m_al_wdata,
    output                         m_al_wvalid,
    input                          m_al_wready,

    output reg [AL_BUS_WIDTH-1:2]  m_al_araddr,
    output reg                     m_al_arvalid,
    input                          m_al_arready,

    input[31:0]                    m_al_rdata,
    input                          m_al_rvalid,
    output                         m_al_rready
);

// dw0 [R/W ... L5:L0|A15...A0]  up to 64DW -- 256bytes
localparam MAX_LEN_DW_BITS = 5;
wire [AL_BUS_WIDTH-1:2]    req_addr = s_axis_endpoint_tx_data[AL_BUS_WIDTH-2-1:0];
wire [MAX_LEN_DW_BITS-1:0] req_len  = s_axis_endpoint_tx_data[MAX_LEN_DW_BITS+16-1:16];
wire                       req_rd   = s_axis_endpoint_tx_data[31];


reg  [1:0] state;
reg  [MAX_LEN_DW_BITS:0] req_dws_zsz;
wire [MAX_LEN_DW_BITS:0] req_dws_zsz_nxt = req_dws_zsz - 1'b1;
reg  [MAX_LEN_DW_BITS:0] dat_dws_zsz;
wire [MAX_LEN_DW_BITS:0] dat_dws_zsz_nxt = dat_dws_zsz - 1'b1;

localparam [1:0]
    ST_IDLE    = 0,
    ST_TX      = 1,
    ST_RX      = 2,
    ST_INDALID = 3;

assign s_axis_endpoint_tx_busy  = (state != ST_IDLE);
assign s_axis_endpoint_tx_ready = (state == ST_IDLE) || (state == ST_TX && m_al_wready) || (state == ST_INDALID);

assign m_al_wdata  = s_axis_endpoint_tx_data;
assign m_al_wvalid = (state == ST_TX && s_axis_endpoint_tx_valid);

assign m_axis_endpoint_rx_last  = dat_dws_zsz_nxt[MAX_LEN_DW_BITS];
assign m_axis_endpoint_rx_keep  = 4'b1111;
assign m_axis_endpoint_rx_data  = m_al_rdata;
assign m_axis_endpoint_rx_valid = m_al_rvalid;
assign m_al_rready              = m_axis_endpoint_rx_ready;

always @(posedge clk) begin
    if (rst) begin
        state   <= ST_IDLE;
        m_al_arvalid <= 1'b0;
    end else begin

        case (state)
        ST_IDLE: begin
            if (s_axis_endpoint_tx_valid) begin
                req_dws_zsz <= {1'b0, req_len};
                dat_dws_zsz <= {1'b0, req_len};

                m_al_waddr  <= req_addr;
                m_al_araddr <= req_addr;

                if (req_rd) begin
                    state   <= ST_RX;
                    m_al_arvalid <= 1'b1;
                end else if (!req_rd && !s_axis_endpoint_tx_last) begin
                    state   <= ST_TX;
                end else begin
                    // Write 0 bytes, just ignore
                    state   <= ST_IDLE;
                end
            end
        end

        ST_RX: begin
            if (m_al_arvalid && m_al_arready) begin
                req_dws_zsz <= req_dws_zsz_nxt;
                m_al_araddr      <= m_al_araddr + 1'b1;
                m_al_arvalid     <= ~(req_dws_zsz_nxt[MAX_LEN_DW_BITS]);
            end

            if (m_al_rvalid && m_al_rready) begin
                dat_dws_zsz <= dat_dws_zsz_nxt;
            end

            if (m_axis_endpoint_rx_ready && m_axis_endpoint_rx_valid && m_axis_endpoint_rx_last) begin
                state <= ST_IDLE;
            end
        end

        ST_TX, ST_INDALID: begin
            if (s_axis_endpoint_tx_ready && s_axis_endpoint_tx_valid) begin
                req_dws_zsz  <= req_dws_zsz_nxt;
                m_al_waddr   <= m_al_waddr + 1'b1;

                if (s_axis_endpoint_tx_last || req_dws_zsz_nxt[MAX_LEN_DW_BITS]) begin
                    state <= ST_IDLE;
                end
            end
        end

        endcase
    end
end



endmodule
