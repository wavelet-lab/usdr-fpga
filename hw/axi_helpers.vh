// AXI helpers


`define STATIC_ASSERT(expr, message) \
    generate if (!(expr)) assertation_failed assertation_failed``message``(.value(expr)); endgenerate
    
`define SUBVECTOR(name, width, offset) name[width * (offset + 1) - 1:width * offset]

`define DEFINE_AXIS_PORT_NAME(name, sig) \
    wire ``name````sig``
    
`define DEFINE_AXIS_VPORT_NAME(name, sig, bcnt) \
    wire [bcnt - 1:0] ``name````sig``
    
 
`define DEFINE_AXIS_R_PORT(name)        `DEFINE_AXIS_PORT_NAME(name, ready)
`define DEFINE_AXIS_V_PORT(name)        `DEFINE_AXIS_PORT_NAME(name, valid)
`define DEFINE_AXIS_D_PORT(name, bits)  `DEFINE_AXIS_VPORT_NAME(name, data, bits)
`define DEFINE_AXIS_L_PORT(name)        `DEFINE_AXIS_PORT_NAME(name, last)
`define DEFINE_AXIS_K_PORT(name, bits)  `DEFINE_AXIS_VPORT_NAME(name, keep, bits)
`define DEFINE_AXIS_A_PORT(name, bits)  `DEFINE_AXIS_VPORT_NAME(name, addr, bits)
`define DEFINE_AXIS_I_PORT(name, bits)  `DEFINE_AXIS_VPORT_NAME(name, id, bits)
`define DEFINE_AXIS_U_PORT(name, bits)  `DEFINE_AXIS_VPORT_NAME(name, user, bits)

`define DEFINE_AXIS_RV_PORT(name) \
    `DEFINE_AXIS_R_PORT(name); \
    `DEFINE_AXIS_V_PORT(name)

`define DEFINE_AXIS_RVD_PORT(name, bits) \
    `DEFINE_AXIS_RV_PORT(name); \
    `DEFINE_AXIS_D_PORT(name, bits)
    
`define DEFINE_AXIS_RVDL_PORT(name, bits) \
    `DEFINE_AXIS_RVD_PORT(name, bits); \
    `DEFINE_AXIS_L_PORT(name)

`define DEFINE_AXIS_RVDLK_PORT(name, bits, kbits) \
    `DEFINE_AXIS_RVDL_PORT(name, bits); \
    `DEFINE_AXIS_K_PORT(name, kbits)
    
`define DEFINE_AXIS_RVDLKU_PORT(name, bits, kbits, ubits) \
    `DEFINE_AXIS_RVDLK_PORT(name, bits, kbits); \
    `DEFINE_AXIS_U_PORT(name, ubits)


`define DEFINE_ALWR_AXIS(busname, abits, dbits) \
    `DEFINE_AXIS_RVD_PORT(``busname``_w, dbits); \
    `DEFINE_AXIS_A_PORT(``busname``_w, abits)
    
`define DEFINE_ALRD_AXIS(busname, abits, dbits) \
    `DEFINE_AXIS_RV_PORT(``busname``_ar); \
    `DEFINE_AXIS_A_PORT(``busname``_ar, abits); \
    `DEFINE_AXIS_RVD_PORT(``busname``_r, dbits)

`define DEFINE_ALRDI_AXIS(busname, abits, dbits, ibits) \
    `DEFINE_ALRD_AXIS(busname, abits, dbits); \
    `DEFINE_AXIS_I_PORT(``busname``_ar, ibits); \
    `DEFINE_AXIS_I_PORT(``busname``_r, ibits)
    
`define DEFINE_ALRDWR_AXIS(busname, abits, dbits) \
    `DEFINE_ALWR_AXIS(busname, abits, dbits); \
    `DEFINE_ALRD_AXIS(busname, abits, dbits)
    
////////////////////////////////////////////////////////////////////////////

`define AXIS_PORT_CONNECT(mname, wname, port) \
    .``mname````port``(``wname````port``) \

`define AXIS_RV_PORT_CONN(mname, wname) \
    `AXIS_PORT_CONNECT(mname, wname, ready), \
    `AXIS_PORT_CONNECT(mname, wname, valid)

`define AXIS_RVD_PORT_CONN(mname, wname) \
    `AXIS_RV_PORT_CONN(mname, wname), \
    `AXIS_PORT_CONNECT(mname, wname, data)

`define AXIS_RVDL_PORT_CONN(mname, wname) \
    `AXIS_RVD_PORT_CONN(mname, wname), \
    `AXIS_PORT_CONNECT(mname, wname, last)  

`define AXIS_VDL_PORT_CONN(mname, wname) \
    `AXIS_PORT_CONNECT(mname, wname, valid), \
    `AXIS_PORT_CONNECT(mname, wname, data), \
    `AXIS_PORT_CONNECT(mname, wname, last)

`define AXIS_RVDLK_PORT_CONN(mname, wname) \
    `AXIS_RVDL_PORT_CONN(mname, wname), \
    `AXIS_PORT_CONNECT(mname, wname, keep)

`define AXIS_RVDLKU_PORT_CONN(mname, wname) \
    `AXIS_RVDLK_PORT_CONN(mname, wname), \
    `AXIS_PORT_CONNECT(mname, wname, user)
    
`define AXIS_ALWR_CONNECT(mname, wname) \
    `AXIS_RVD_PORT_CONN(``mname``_w, ``wname``_w), \
    `AXIS_PORT_CONNECT(``mname``_w, ``wname``_w, addr)

`define AXIS_ALRD_CONNECT(mname, wname) \
    `AXIS_RV_PORT_CONN(``mname``_ar, ``wname``_ar), \
    `AXIS_PORT_CONNECT(``mname``_ar, ``wname``_ar, addr), \
    `AXIS_RVD_PORT_CONN(``mname``_r, ``wname``_r)
    
`define AXIS_ALRDI_CONNECT(mname, wname) \
    `AXIS_ALRD_CONNECT(mname, wname), \
    `AXIS_PORT_CONNECT(``mname``_ar, ``wname``_ar, id), \
    `AXIS_PORT_CONNECT(``mname``_r, ``wname``_r, id)

`define AXIS_ALRDWR_CONNECT(mname, wname) \
    `AXIS_ALWR_CONNECT(mname, wname), \
    `AXIS_ALRD_CONNECT(mname, wname)

    
////////////////////////////////////////////////////////////////////////
// AL VECTOR helpers

`define AXIS_PORT_VCONNECT_2(mname, wname1, wname0, port) \
    .``mname````port``({ ``wname1````port``, ``wname0````port`` }) \

`define AXIS_RV_PORT_VCONN_2(mname, wname1, wname0) \
    `AXIS_PORT_VCONNECT_2(mname, wname1, wname0, ready), \
    `AXIS_PORT_VCONNECT_2(mname, wname1, wname0, valid)

`define AXIS_RVD_PORT_VCONN_2(mname, wname1, wname0) \
    `AXIS_RV_PORT_VCONN_2(mname, wname1, wname0), \
    `AXIS_PORT_VCONNECT_2(mname, wname1, wname0, data)

    
`define AXIS_ALWR_VCONNECT_2(mname, wname1, wname0) \
    `AXIS_RVD_PORT_VCONN_2(``mname``_w, ``wname1``_w, ``wname0``_w), \
    `AXIS_PORT_VCONNECT_2(``mname``_w, ``wname1``_w, ``wname0``_w, addr)

`define AXIS_ALRD_VCONNECT_2(mname, wname1, wname0) \
    `AXIS_RV_PORT_VCONN_2(``mname``_ar, ``wname1``_ar, ``wname0``_ar), \
    `AXIS_PORT_VCONNECT_2(``mname``_ar, ``wname1``_ar, ``wname0``_ar, addr), \
    `AXIS_RVD_PORT_VCONN_2(``mname``_r, ``wname1``_r, ``wname0``_r)
    
`define AXIS_ALRDI_VCONNECT_2(mname, wname1, wname0) \
    `AXIS_ALRD_VCONNECT_2(mname, wname1, wname0), \
    `AXIS_PORT_VCONNECT_2(``mname``_ar, ``wname1``_ar, ``wname0``_ar, id), \
    `AXIS_PORT_VCONNECT_2(``mname``_r, ``wname1``_r, ``wname0``_r, id)

`define AXIS_ALRDWR_VCONNECT_2(mname, wname1, wname0) \
    `AXIS_ALWR_VCONNECT_2(mname, wname1, wname0), \
    `AXIS_ALRD_VCONNECT_2(mname, wname1, wname0)
    
    

