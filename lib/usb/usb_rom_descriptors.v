module usb_rom_descriptors #(
    parameter         REQ_WIDTH = 8,
    parameter [15:0]  USDR_PID = 0,
    parameter [7:0]   REQ_TERMINATOR = 0,
    parameter         DPARAGRAPS = 1,
    parameter [127:0] ROM_USB_00 = 128'h317003303b34203b34703109601f1110, //64
    parameter [127:0] ROM_USB_01 = 128'h1200e909d2cb1dc79238f0840d32740f, //ab
    parameter [127:0] ROM_USB_02 = { 40'h0002010001, USDR_PID, 72'h372740000000021001 },
    parameter [127:0] ROM_USB_03 = 128'h01003502090001400000000200060a01, //71
    parameter [127:0] ROM_USB_04 = 128'h81050702ffffff0500000409f0800001, //07
    parameter [127:0] ROM_USB_05 = 128'h40038205070002000201050700020002, //c0
    parameter [127:0] ROM_USB_06 = 128'h00020002030507000200028305070100, //81
    parameter [127:0] ROM_USB_07 = 128'h006c0065007600610057031004090304, //50
    parameter [127:0] ROM_USB_08 = 128'h00440053006200650057030e00740065, //5b
    parameter [127:0] ROM_USB_09 = 128'ha93408b638000510180200390f050052, //75
    parameter [127:0] ROM_USB_0a = 128'h1c01fc010065b6158876a0fd8b47a009, //e0
    parameter [127:0] ROM_USB_0b = 128'h9e9d65d29c4cc74589d8dd60df000510, //c0
    parameter [127:0] ROM_USB_0c = 128'h000000000a00fe001e060300009f8a64, //9e
    parameter [127:0] ROM_USB_0d = 128'h004253554e495700030014001e060300, //18
    parameter [127:0] ROM_USB_0e = 128'h7264737701030a000000000000000000, //1a
    parameter [127:0] ROM_USB_0f = 128'hcccccccccccccccccccccccccc6f692e, //e4
    parameter [127:0] ROM_USB_10 = 128'h32710f316d03303834202e09601c1110, //55
    parameter [127:0] ROM_USB_11 = 128'h0210011200e609d2c81dc78f38f0810d, //e5
    parameter [127:0] ROM_USB_12 = { 64'h060a010002010001, USDR_PID, 48'h372740000000 },
    parameter [127:0] ROM_USB_13 = 128'h80000101003502090001400000000200, //fd
    parameter [127:0] ROM_USB_14 = 128'h00400281050702ffffff0500000409f0, //c4
    parameter [127:0] ROM_USB_15 = 128'h07010040038205070000400201050700, //84
    parameter [127:0] ROM_USB_16 = 128'h09030400004002030507000040028305, //89
    parameter [127:0] ROM_USB_17 = 128'h740065006c0065007600610057031004, //4f
    parameter [127:0] ROM_USB_18 = 128'h05005200440053006200650057030e00, //1d
    parameter [127:0] ROM_USB_19 = 128'h47a009a93408b638000510180200390f, //cc
    parameter [127:0] ROM_USB_1a = 128'h0005101c01fc010065b6158876a0fd8b, //1b
    parameter [127:0] ROM_USB_1b = 128'h9f8a649e9d65d29c4cc74589d8dd60df, //a4
    parameter [127:0] ROM_USB_1c = 128'h060300000000000a00fe001e06030000, //ea
    parameter [127:0] ROM_USB_1d = 128'h000000004253554e495700030014001e, //1d
    parameter [127:0] ROM_USB_1e = 128'h6f692e7264737701030a000000000000, //32
    parameter [127:0] ROM_USB_1f = 128'hcccccccccccccccccccccccccccccccc, //00
    parameter         LAST = 0
)(
    input clk,
    input reset,

    input                 req_valid,
    input [REQ_WIDTH-1:0] req_data,
    input                 req_par,

    output                out_valid,
    output [7:0]          out_data,
    output                out_last,
    output reg            out_nomatch,
    input                 out_ready
);

// Descriptots ROM
(* rom_style = "distributed" *)
wire [7:0] descr_rom[0:511];

// for j in range(16): print("assign {%s} = ROM_USB_0%x;" % (reduce(lambda x, y: "%s, %s" % (x, y), ["descr_rom[%d]" % i for i in range(16*j+15,16*j-1,-1)]), j))
assign {descr_rom[15], descr_rom[14], descr_rom[13], descr_rom[12], descr_rom[11], descr_rom[10], descr_rom[9], descr_rom[8], descr_rom[7], descr_rom[6], descr_rom[5], descr_rom[4], descr_rom[3], descr_rom[2], descr_rom[1], descr_rom[0]} = ROM_USB_00;
assign {descr_rom[31], descr_rom[30], descr_rom[29], descr_rom[28], descr_rom[27], descr_rom[26], descr_rom[25], descr_rom[24], descr_rom[23], descr_rom[22], descr_rom[21], descr_rom[20], descr_rom[19], descr_rom[18], descr_rom[17], descr_rom[16]} = ROM_USB_01;
assign {descr_rom[47], descr_rom[46], descr_rom[45], descr_rom[44], descr_rom[43], descr_rom[42], descr_rom[41], descr_rom[40], descr_rom[39], descr_rom[38], descr_rom[37], descr_rom[36], descr_rom[35], descr_rom[34], descr_rom[33], descr_rom[32]} = ROM_USB_02;
assign {descr_rom[63], descr_rom[62], descr_rom[61], descr_rom[60], descr_rom[59], descr_rom[58], descr_rom[57], descr_rom[56], descr_rom[55], descr_rom[54], descr_rom[53], descr_rom[52], descr_rom[51], descr_rom[50], descr_rom[49], descr_rom[48]} = ROM_USB_03;
assign {descr_rom[79], descr_rom[78], descr_rom[77], descr_rom[76], descr_rom[75], descr_rom[74], descr_rom[73], descr_rom[72], descr_rom[71], descr_rom[70], descr_rom[69], descr_rom[68], descr_rom[67], descr_rom[66], descr_rom[65], descr_rom[64]} = ROM_USB_04;
assign {descr_rom[95], descr_rom[94], descr_rom[93], descr_rom[92], descr_rom[91], descr_rom[90], descr_rom[89], descr_rom[88], descr_rom[87], descr_rom[86], descr_rom[85], descr_rom[84], descr_rom[83], descr_rom[82], descr_rom[81], descr_rom[80]} = ROM_USB_05;
assign {descr_rom[111], descr_rom[110], descr_rom[109], descr_rom[108], descr_rom[107], descr_rom[106], descr_rom[105], descr_rom[104], descr_rom[103], descr_rom[102], descr_rom[101], descr_rom[100], descr_rom[99], descr_rom[98], descr_rom[97], descr_rom[96]} = ROM_USB_06;
assign {descr_rom[127], descr_rom[126], descr_rom[125], descr_rom[124], descr_rom[123], descr_rom[122], descr_rom[121], descr_rom[120], descr_rom[119], descr_rom[118], descr_rom[117], descr_rom[116], descr_rom[115], descr_rom[114], descr_rom[113], descr_rom[112]} = ROM_USB_07;
assign {descr_rom[143], descr_rom[142], descr_rom[141], descr_rom[140], descr_rom[139], descr_rom[138], descr_rom[137], descr_rom[136], descr_rom[135], descr_rom[134], descr_rom[133], descr_rom[132], descr_rom[131], descr_rom[130], descr_rom[129], descr_rom[128]} = ROM_USB_08;
assign {descr_rom[159], descr_rom[158], descr_rom[157], descr_rom[156], descr_rom[155], descr_rom[154], descr_rom[153], descr_rom[152], descr_rom[151], descr_rom[150], descr_rom[149], descr_rom[148], descr_rom[147], descr_rom[146], descr_rom[145], descr_rom[144]} = ROM_USB_09;
assign {descr_rom[175], descr_rom[174], descr_rom[173], descr_rom[172], descr_rom[171], descr_rom[170], descr_rom[169], descr_rom[168], descr_rom[167], descr_rom[166], descr_rom[165], descr_rom[164], descr_rom[163], descr_rom[162], descr_rom[161], descr_rom[160]} = ROM_USB_0a;
assign {descr_rom[191], descr_rom[190], descr_rom[189], descr_rom[188], descr_rom[187], descr_rom[186], descr_rom[185], descr_rom[184], descr_rom[183], descr_rom[182], descr_rom[181], descr_rom[180], descr_rom[179], descr_rom[178], descr_rom[177], descr_rom[176]} = ROM_USB_0b;
assign {descr_rom[207], descr_rom[206], descr_rom[205], descr_rom[204], descr_rom[203], descr_rom[202], descr_rom[201], descr_rom[200], descr_rom[199], descr_rom[198], descr_rom[197], descr_rom[196], descr_rom[195], descr_rom[194], descr_rom[193], descr_rom[192]} = ROM_USB_0c;
assign {descr_rom[223], descr_rom[222], descr_rom[221], descr_rom[220], descr_rom[219], descr_rom[218], descr_rom[217], descr_rom[216], descr_rom[215], descr_rom[214], descr_rom[213], descr_rom[212], descr_rom[211], descr_rom[210], descr_rom[209], descr_rom[208]} = ROM_USB_0d;
assign {descr_rom[239], descr_rom[238], descr_rom[237], descr_rom[236], descr_rom[235], descr_rom[234], descr_rom[233], descr_rom[232], descr_rom[231], descr_rom[230], descr_rom[229], descr_rom[228], descr_rom[227], descr_rom[226], descr_rom[225], descr_rom[224]} = ROM_USB_0e;
assign {descr_rom[255], descr_rom[254], descr_rom[253], descr_rom[252], descr_rom[251], descr_rom[250], descr_rom[249], descr_rom[248], descr_rom[247], descr_rom[246], descr_rom[245], descr_rom[244], descr_rom[243], descr_rom[242], descr_rom[241], descr_rom[240]} = ROM_USB_0f;

assign {descr_rom[271], descr_rom[270], descr_rom[269], descr_rom[268], descr_rom[267], descr_rom[266], descr_rom[265], descr_rom[264], descr_rom[263], descr_rom[262], descr_rom[261], descr_rom[260], descr_rom[259], descr_rom[258], descr_rom[257], descr_rom[256]} = ROM_USB_10;
assign {descr_rom[287], descr_rom[286], descr_rom[285], descr_rom[284], descr_rom[283], descr_rom[282], descr_rom[281], descr_rom[280], descr_rom[279], descr_rom[278], descr_rom[277], descr_rom[276], descr_rom[275], descr_rom[274], descr_rom[273], descr_rom[272]} = ROM_USB_11;
assign {descr_rom[303], descr_rom[302], descr_rom[301], descr_rom[300], descr_rom[299], descr_rom[298], descr_rom[297], descr_rom[296], descr_rom[295], descr_rom[294], descr_rom[293], descr_rom[292], descr_rom[291], descr_rom[290], descr_rom[289], descr_rom[288]} = ROM_USB_12;
assign {descr_rom[319], descr_rom[318], descr_rom[317], descr_rom[316], descr_rom[315], descr_rom[314], descr_rom[313], descr_rom[312], descr_rom[311], descr_rom[310], descr_rom[309], descr_rom[308], descr_rom[307], descr_rom[306], descr_rom[305], descr_rom[304]} = ROM_USB_13;
assign {descr_rom[335], descr_rom[334], descr_rom[333], descr_rom[332], descr_rom[331], descr_rom[330], descr_rom[329], descr_rom[328], descr_rom[327], descr_rom[326], descr_rom[325], descr_rom[324], descr_rom[323], descr_rom[322], descr_rom[321], descr_rom[320]} = ROM_USB_14;
assign {descr_rom[351], descr_rom[350], descr_rom[349], descr_rom[348], descr_rom[347], descr_rom[346], descr_rom[345], descr_rom[344], descr_rom[343], descr_rom[342], descr_rom[341], descr_rom[340], descr_rom[339], descr_rom[338], descr_rom[337], descr_rom[336]} = ROM_USB_15;
assign {descr_rom[367], descr_rom[366], descr_rom[365], descr_rom[364], descr_rom[363], descr_rom[362], descr_rom[361], descr_rom[360], descr_rom[359], descr_rom[358], descr_rom[357], descr_rom[356], descr_rom[355], descr_rom[354], descr_rom[353], descr_rom[352]} = ROM_USB_16;
assign {descr_rom[383], descr_rom[382], descr_rom[381], descr_rom[380], descr_rom[379], descr_rom[378], descr_rom[377], descr_rom[376], descr_rom[375], descr_rom[374], descr_rom[373], descr_rom[372], descr_rom[371], descr_rom[370], descr_rom[369], descr_rom[368]} = ROM_USB_17;
assign {descr_rom[399], descr_rom[398], descr_rom[397], descr_rom[396], descr_rom[395], descr_rom[394], descr_rom[393], descr_rom[392], descr_rom[391], descr_rom[390], descr_rom[389], descr_rom[388], descr_rom[387], descr_rom[386], descr_rom[385], descr_rom[384]} = ROM_USB_18;
assign {descr_rom[415], descr_rom[414], descr_rom[413], descr_rom[412], descr_rom[411], descr_rom[410], descr_rom[409], descr_rom[408], descr_rom[407], descr_rom[406], descr_rom[405], descr_rom[404], descr_rom[403], descr_rom[402], descr_rom[401], descr_rom[400]} = ROM_USB_19;
assign {descr_rom[431], descr_rom[430], descr_rom[429], descr_rom[428], descr_rom[427], descr_rom[426], descr_rom[425], descr_rom[424], descr_rom[423], descr_rom[422], descr_rom[421], descr_rom[420], descr_rom[419], descr_rom[418], descr_rom[417], descr_rom[416]} = ROM_USB_1a;
assign {descr_rom[447], descr_rom[446], descr_rom[445], descr_rom[444], descr_rom[443], descr_rom[442], descr_rom[441], descr_rom[440], descr_rom[439], descr_rom[438], descr_rom[437], descr_rom[436], descr_rom[435], descr_rom[434], descr_rom[433], descr_rom[432]} = ROM_USB_1b;
assign {descr_rom[463], descr_rom[462], descr_rom[461], descr_rom[460], descr_rom[459], descr_rom[458], descr_rom[457], descr_rom[456], descr_rom[455], descr_rom[454], descr_rom[453], descr_rom[452], descr_rom[451], descr_rom[450], descr_rom[449], descr_rom[448]} = ROM_USB_1c;
assign {descr_rom[479], descr_rom[478], descr_rom[477], descr_rom[476], descr_rom[475], descr_rom[474], descr_rom[473], descr_rom[472], descr_rom[471], descr_rom[470], descr_rom[469], descr_rom[468], descr_rom[467], descr_rom[466], descr_rom[465], descr_rom[464]} = ROM_USB_1d;
assign {descr_rom[495], descr_rom[494], descr_rom[493], descr_rom[492], descr_rom[491], descr_rom[490], descr_rom[489], descr_rom[488], descr_rom[487], descr_rom[486], descr_rom[485], descr_rom[484], descr_rom[483], descr_rom[482], descr_rom[481], descr_rom[480]} = ROM_USB_1e;
assign {descr_rom[511], descr_rom[510], descr_rom[509], descr_rom[508], descr_rom[507], descr_rom[506], descr_rom[505], descr_rom[504], descr_rom[503], descr_rom[502], descr_rom[501], descr_rom[500], descr_rom[499], descr_rom[498], descr_rom[497], descr_rom[496]} = ROM_USB_1f;

// [descr|idx] [length] [off]
//  00 terminator
// Dictionary length 3 * N + 1

localparam [2:0]
    ST_IDLE = 0,
    ST_LOOKUP = 1,
    ST_LENLOAD = 2,
    ST_OFFLOAD = 3,
    ST_TRANSFER = 4;
reg [2:0] state;

reg [7:0]    rom_addr;
reg [7+1:0]  req_len;
wire [7:0]   rom_data = descr_rom[{ req_par, rom_addr }];

assign out_last  = (req_len[7+1]);
assign out_valid = (state[2]);
assign out_data  = rom_data;

always @(posedge clk) begin
    if (reset) begin
        state       <= ST_IDLE;
        out_nomatch <= 0;
    end else begin

        if (req_valid) begin
            state         <= ST_LOOKUP;
            rom_addr      <= 0;
            out_nomatch   <= 0;
        end

        if (state == ST_LOOKUP) begin
            if (rom_data == REQ_TERMINATOR) begin
                out_nomatch <= 1'b1;
                state       <= ST_IDLE;
            end else if (rom_data == req_data) begin
                state       <= ST_LENLOAD;
                rom_addr    <= rom_addr + 1'b1;
            end else begin
                rom_addr    <= rom_addr + 8'b11;
            end
        end else if (state == ST_LENLOAD) begin
            req_len       <= {1'b0, rom_data};
            rom_addr      <= rom_addr + 1'b1;
            state         <= ST_OFFLOAD;
        end else if (state == ST_OFFLOAD) begin
            rom_addr      <= rom_data;
            state         <= ST_TRANSFER;
            req_len       <= req_len - 1'b1;
        end else if (state == ST_TRANSFER && out_ready) begin
            if (out_last) begin
                state       <= ST_IDLE;
            end

            rom_addr      <= rom_addr + 1'b1;
            req_len       <= req_len - 1'b1;
        end

    end
end


endmodule
