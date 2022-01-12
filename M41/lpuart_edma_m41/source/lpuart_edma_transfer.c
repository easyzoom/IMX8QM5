/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_lpuart_edma.h"
#include "fsl_debug_console.h"
#if defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT
#include "fsl_dmamux.h"
#endif
#include "fsl_irqsteer.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_LPUART                 DMA__LPUART3
#define DEMO_LPUART_CLKSRC          kCLOCK_DMA_Lpuart3
#define DEMO_LPUART_CLK_FREQ        CLOCK_GetIpFreq(kCLOCK_DMA_Lpuart3)
#define DEMO_LPUART_IRQn            DMA_UART3_INT_IRQn
#define DEMO_LPUART_IRQ_HANDLER     DMA_UART3_INT_IRQHandler
#define DEMO_LPUART_POWER           SC_R_UART_2
#define DEMO_RX_CHANNEL_POWER       SC_R_DMA_0_CH18
#define DEMO_TX_CHANNEL_POWER       SC_R_DMA_0_CH19
#define DEMO_LPUART_IRQ             DMA_UART3_INT_IRQn
#define DEMO_LPUART_RX_IRQ_HANDLER  DMA_UART3_DMA_RX_INT_IRQn
#define DEMO_LPUART_TX_IRQ_HANDLER  DMA_UART3_DMA_TX_INT_IRQn
#define LPUART_TX_DMA_CHANNEL       19U
#define LPUART_RX_DMA_CHANNEL       18U
#define EXAMPLE_LPUART_DMA_BASEADDR DMA__EDMA0
#define ECHO_BUFFER_LENGTH          100


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* LPUART user callback */
void LPUART_UserCallback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/

lpuart_edma_handle_t g_lpuartEdmaHandle;
edma_handle_t g_lpuartTxEdmaHandle;
edma_handle_t g_lpuartRxEdmaHandle;
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_txBuffer[ECHO_BUFFER_LENGTH]) = {0};
AT_NONCACHEABLE_SECTION_INIT(uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH]) = {0};
lpuart_transfer_t receiveXfer;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    lpuart_config_t lpuartConfig;
    edma_config_t config;

    sc_ipc_t ipc;
    uint32_t freq;

    ipc = BOARD_InitRpc();

    BOARD_InitPins(ipc);
    BOARD_BootClockRUN();
    BOARD_InitMemory();
    BOARD_InitDebugConsole();

    /* Power on LPUART for M4. */
    if (sc_pm_set_resource_power_mode(ipc, DEMO_LPUART_POWER, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        assert(false);
    }
    if (sc_pm_set_resource_power_mode(ipc, DEMO_RX_CHANNEL_POWER, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        assert(false);
    }
    if (sc_pm_set_resource_power_mode(ipc, DEMO_TX_CHANNEL_POWER, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        assert(false);
    }

    /* Set UART Clock frequency */
    freq = CLOCK_SetIpFreq(DEMO_LPUART_CLKSRC, SC_133MHZ);
    if (freq == 0)
    {
        assert(freq);
    }

    /* Enable interrupt in irqsteer */
    if (sc_pm_set_resource_power_mode(ipc, SC_R_IRQSTR_M4_1, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        assert(false);
    }
    IRQSTEER_Init(IRQSTEER);
    IRQSTEER_EnableInterrupt(IRQSTEER, DEMO_LPUART_IRQ);
    IRQSTEER_EnableInterrupt(IRQSTEER, DEMO_LPUART_RX_IRQ_HANDLER);
    IRQSTEER_EnableInterrupt(IRQSTEER, DEMO_LPUART_TX_IRQ_HANDLER);

    /* Initialize the LPUART. */
    /*
     * lpuartConfig.baudRate_Bps = 115200U;
     * lpuartConfig.parityMode = kLPUART_ParityDisabled;
     * lpuartConfig.stopBitCount = kLPUART_OneStopBit;
     * lpuartConfig.txFifoWatermark = 0;
     * lpuartConfig.rxFifoWatermark = 0;
     * lpuartConfig.enableTx = false;
     * lpuartConfig.enableRx = false;
     */
    LPUART_GetDefaultConfig(&lpuartConfig);
    lpuartConfig.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    lpuartConfig.enableTx     = true;
    lpuartConfig.enableRx     = true;

    LPUART_Init(DEMO_LPUART, &lpuartConfig, DEMO_LPUART_CLK_FREQ);
    /* Init the EDMA module */
    EDMA_GetDefaultConfig(&config);
    EDMA_Init(EXAMPLE_LPUART_DMA_BASEADDR, &config);
    EDMA_CreateHandle(&g_lpuartTxEdmaHandle, EXAMPLE_LPUART_DMA_BASEADDR, LPUART_TX_DMA_CHANNEL);
    EDMA_CreateHandle(&g_lpuartRxEdmaHandle, EXAMPLE_LPUART_DMA_BASEADDR, LPUART_RX_DMA_CHANNEL);
    /* Create LPUART DMA handle. */
    LPUART_TransferCreateHandleEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, NULL, NULL, &g_lpuartTxEdmaHandle,
                                    &g_lpuartRxEdmaHandle);
    /* Enable RX interrupt. */
    LPUART_EnableInterrupts(DEMO_LPUART, kLPUART_IdleLineInterruptEnable);
    EnableIRQ(DEMO_LPUART_IRQn);
    receiveXfer.data     = g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;
    LPUART_ReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &receiveXfer);

    while (1)
    {
    }
}

void DEMO_LPUART_IRQ_HANDLER(void)
{
    uint32_t data;
    if ((kLPUART_IdleLineFlag) & LPUART_GetStatusFlags(DEMO_LPUART))
    {
        //DEMO_LPUART->STAT |= LPUART_STAT_IDLE_MASK;
        LPUART_TransferGetReceiveCountEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &data);
        PRINTF("\r\n_______________%d______________\r\n",data);
        LPUART_WriteBlocking(DEMO_LPUART, g_rxBuffer,data);
        LPUART_ClearStatusFlags(DEMO_LPUART,kLPUART_IdleLineInterruptEnable);
        LPUART_TransferAbortReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle);
        receiveXfer.data     = g_rxBuffer;
        receiveXfer.dataSize = ECHO_BUFFER_LENGTH;
        LPUART_ReceiveEDMA(DEMO_LPUART, &g_lpuartEdmaHandle, &receiveXfer);
    }
    SDK_ISR_EXIT_BARRIER;
}