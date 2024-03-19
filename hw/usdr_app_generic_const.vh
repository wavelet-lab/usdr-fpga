// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//

// Generic global constants


localparam REG_WR_CTRL     = 0;
localparam REG_I2C         = 1;
localparam REG_SPI0        = 2;
localparam REG_SPI1        = 3;

localparam REG_WR_RXDMA_CNF0  = 4;
localparam REG_WR_RXDMA_CTRL0 = 5;
localparam REG_WR_MBUS2_ADDR  = 6;
localparam REG_WR_MBUS2_DATA  = 7;

localparam REG_WR_PNTFY_CFG     = 8;
localparam REG_WR_PNTFY_ACK     = 9;
localparam REG_WR_FLASHSPI_CMD  = 10;
localparam REG_WR_FLASHSPI_ADDR = 11;

localparam REG_WR_UNUSED_2    = 12;
localparam REG_SPI2        = 13;
localparam REG_SPI3        = 14;
localparam REG_INTS        = 15;

localparam REG_WR_LBDSP       = 16;


// TODO replace
localparam REG_GPIO_CTRL       = 3;


// for TX, unused anyway
localparam REG_WR_TXDMA_CNF_L  = 12;
localparam REG_WR_TXDMA_CNF_T  = 13;
localparam REG_WR_TXDMA_COMB   = 14;  // TxDMA low 16 bits, TIMER high 16 bits

////////////////////////////////////////////////////

localparam REG_RD_IGPI_0      = 0;
//localparam REG_RD_I2C         = 1;
//localparam REG_RD_SPI0        = 2;
//localparam REG_RD_SPI1        = 3;

localparam REG_RD_RXDMA_BST0  = 4; // rxdma_buffs
localparam REG_RD_RXDMA_BST1  = 5; // rxdma_bursts
localparam REG_RD_RXDMA_STAT  = 6;
localparam REG_RD_UNUSED_0    = 7;

localparam REG_RD_UNUSED_1       = 8;
localparam REG_RD_TIMEEVENT      = 9;
localparam REG_RD_FLASHSPI_STAT  = 10;
localparam REG_RD_FLASHSPI_DATA  = 11;


localparam REG_RD_LBDSP          = 12;


localparam REG_RD_GPIO_N         = 26;
localparam REG_RD_GPIO_0         = 27;

localparam REG_RD_TXDMA_STAT     = 28;
localparam REG_RD_TXDMA_STATM    = 29;
localparam REG_RD_TXDMA_STATTS   = 30;
localparam REG_RD_TXDMA_STAT_CPL = 31;


// 16 to 22  -- GPI0 throuh GPI27 directly mmaped

localparam REG_IGP_RDWD_CNT   = 23 - 16;

// General RD/WR
localparam REG_UART_TRX       = 23;

localparam REG_CFG_PHY_0      = 24;
localparam REG_CFG_PHY_1      = 25;

