// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//

// Generic global constants


localparam REG_WR_CTRL          = 0;
localparam REG_I2C              = 1;
localparam REG_SPI0             = 2;
localparam REG_GPIO_CTRL        = 3;

localparam REG_WR_RXDMA_CNF0    = 4;
localparam REG_WR_RXDMA_CTRL0   = 5;
localparam REG_WR_MBUS2_ADDR    = 6;
localparam REG_WR_MBUS2_DATA    = 7;

localparam REG_WR_PNTFY_CFG     = 8;
localparam REG_WR_PNTFY_ACK     = 9;
localparam REG_WR_FLASHSPI_CMD  = 10;
localparam REG_WR_FLASHSPI_ADDR = 11;

// Old TX core
localparam REG_WR_TXDMA_CNF_L   = 12;  // TODO replace
localparam REG_WR_TXDMA_CNF_T   = 13;  // TODO replace

localparam REG_WR_TXDMA_COMB    = 14;  // TODO replace:  TxDMA low 16 bits, TIMER high 16 bits
localparam REG_INTS             = 15;
////////////////////////////////////////////////////

localparam REG_RD_IGPI_0      = 0;
//localparam REG_RD_I2C         = 1;
//localparam REG_RD_SPI0        = 2;

localparam REG_RD_RXDMA_BST0  = 4; // rxdma_buffs
localparam REG_RD_RXDMA_BST1  = 5; // rxdma_bursts
localparam REG_RD_RXDMA_STAT  = 6;
localparam REG_RD_RXFE_PWR0   = 7;

localparam REG_RD_RXFE_PWR1      = 8;
localparam REG_RD_TIMEEVENT      = 9;
localparam REG_RD_FLASHSPI_STAT  = 10;
localparam REG_RD_FLASHSPI_DATA  = 11;

/////////////////////////////////////////////

// 16 to 22  -- GPI0 throuh GPI27 directly mmaped
localparam REG_IGP_RDWD_CNT      = 28 - 16;

// New TX core
localparam REG_WR_TXDMA_CFG0     = 28;
localparam REG_WR_TXDMA_CFG1     = 29;
localparam REG_WR_TXDMA_TS_HI    = 30;
localparam REG_WR_TXDMA_TS_LO    = 31;

localparam REG_RD_TXDMA_STAT     = 28;
localparam REG_RD_TXDMA_STATM    = 29;
localparam REG_RD_TXDMA_STATTS   = 30;
localparam REG_RD_TXDMA_STAT_CPL = 31;


////////////////////////////////////////////////////////


localparam REG_SPI1              = 48;
localparam REG_SPI2              = 49;
localparam REG_SPI3              = 50;

 
localparam REG_LBDSP             = 52;
localparam REG_UART_TRX          = 54;

localparam REG_CFG_PHY_0         = 56;
localparam REG_CFG_PHY_1         = 57;

localparam REG_SPI_EXT_CFG       = 58;
localparam REG_SPI_EXT_DATA      = 59;

localparam REG_I2C2_CFG          = 60;
localparam REG_I2C2              = 61;
