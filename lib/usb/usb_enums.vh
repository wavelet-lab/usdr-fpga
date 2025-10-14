
// Tokens
localparam R_PID_OUT   = 4'b0001;
localparam R_PID_IN    = 4'b1001;
localparam R_PID_SOF   = 4'b0101;
localparam R_PID_SETUP = 4'b1101;

// Data
localparam R_PID_DATA0 = 4'b0011;
localparam R_PID_DATA1 = 4'b1011;
localparam R_PID_DATA2 = 4'b0111;
localparam R_PID_MDATA = 4'b1111;

// Handshake
localparam R_PID_ACK   = 4'b0010;
localparam R_PID_NAK   = 4'b1010;
localparam R_PID_STALL = 4'b1110;
localparam R_PID_NYET  = 4'b0110;

// Special
localparam R_PID_PRE_ERR = 4'b1100;
localparam R_PID_SPLIT = 4'b1000;
localparam R_PID_PING  = 4'b0100;

// Mandatory SETUP transactions
localparam [7:0] 
    CLEAR_FEATURE     = 8'd1,
    GET_CONFIGURATION = 8'd8,
    GET_DESCRIPTOR    = 8'd6,
    GET_STATUS        = 8'd0,
    SET_ADDRESS       = 8'd5,
    SET_CONFIGURATION = 8'd9,
    SET_FEATURE       = 8'd3;
