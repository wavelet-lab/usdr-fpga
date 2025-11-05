
`define DEFINE_CSR32_PORT(name) `DEFINE_AXIS_RVD_PORT(axis_``name``_, 32)

`define DEFINE_CSR32_WR_PORT(name) `DEFINE_CSR32_PORT(wr_``name)
`define ASSIGN_CSR32_WR(addr, name) \
    assign axis_wr_``name``_data = axis_csrwr_data; \
    assign axis_wr_``name``_valid = axis_csrwr_valid[addr]; \
    assign axis_csrwr_ready[addr] = axis_wr_``name``_ready

`define CSR32_WR(addr, name) \
    `DEFINE_CSR32_WR_PORT(name); \
    `ASSIGN_CSR32_WR(addr, name)

`define DEFINE_CSR32_RD_PORT(name) `DEFINE_CSR32_PORT(rd_``name)
`define ASSIGN_CSR32_RD(addr, name) \
    assign axis_rd_``name``_ready = axis_csrrd_ready[addr]; \
    assign axis_csrrd_data[32*(addr + 1)-1:32*(addr)] = axis_rd_``name``_data; \
    assign axis_csrrd_valid[addr] = axis_rd_``name``_valid

`define CSR32_RD(addr, name) \
    `DEFINE_CSR32_RD_PORT(name); \
    `ASSIGN_CSR32_RD(addr, name)

`define CSR32_RDWR(addr, name) \
    `CSR32_RD(addr, name); \
    `CSR32_WR(addr, name)

`define CSR32_RD_CONST(addr, value) \
    `CSR32_RD(addr, addr); \
    assign axis_rd_``addr``_data = value; \
    assign axis_rd_``addr``_valid = 1'b1

`define CSR32_RD_NULL(addr) `CSR32_RD_CONST(addr, 32'h0000_0000)

`define CSR32_WR_NULL(addr) \
    `CSR32_WR(addr, addr); \
    assign axis_wr_``addr``_ready = 1'b1

`define CSR32_RDWR_NULL(addr) \
    `CSR32_RD_NULL(addr); \
    `CSR32_WR_NULL(addr)

`define DEFINE_CSR32_RDWR_PORT(name) \
    `DEFINE_CSR32_RD_PORT(name); \
    `DEFINE_CSR32_WR_PORT(name)

`define ASSIGN_CSR32_RDWR(addr, name) \
    `ASSIGN_CSR32_RD(addr, name); \
    `ASSIGN_CSR32_WR(addr, name)

