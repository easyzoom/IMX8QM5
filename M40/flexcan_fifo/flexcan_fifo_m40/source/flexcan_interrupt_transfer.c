/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "fsl_irqsteer.h"
#include "fsl_lpi2c.h"
#include "fsl_rgpio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Considering that the first valid MB must be used as Reserved TX MB for ERR005641,
 * if RX FIFO enables (RFEN bit in MCE set as 1) and RFFN in CTRL2 is set default as zero,
 * the first valid TX MB Number shall be 8;
 * if RX FIFO enables (RFEN bit in MCE set as 1) and RFFN in CTRL2 is set by other values (0x1~0xF),
 * the user should consider to detail the first valid MB number;
 * if RX FIFO disables (RFEN bit in MCE set as 0) , the first valid MB number would be zero.
 */
#define TX_MESSAGE_BUFFER_NUM   (9)
#define EXAMPLE_CAN             DMA__CAN0
#define EXAMPLE_CAN_CLK_FREQ    CLOCK_GetIpFreq(kCLOCK_DMA_Can0)
#define DLC                     (1)
#define CAN_POWER               SC_R_CAN_0
#define CAN_CLOCK               kCLOCK_DMA_Can0
#define CAN_INTERRUPT           DMA_FLEXCAN0_INT_IRQn
/* The CAN clock prescaler = CAN source clock/(baud rate * quantum), and the prescaler must be an integer.
   The quantum default value is set to 10=(3+2+1)+4, because for most platforms the CAN clock frequency is
   a multiple of 10. e.g. 120M CAN source clock/(1M baud rate * 10) is an integer. If the CAN clock frequency
   is not a multiple of 10, users need to set SET_CAN_QUANTUM and define the PSEG1/PSEG2/PROPSEG (classical CAN)
   and FPSEG1/FPSEG2/FPROPSEG (CANFD) vaule. Or can set USE_IMPROVED_TIMING_CONFIG macro to use driver api to
   calculates the improved timing values. */
#define SET_CAN_QUANTUM 1
#define PSEG1           6
#define PSEG2           4
#define PROPSEG         6
#define FPSEG1          6
#define FPSEG2          4
#define FPROPSEG        7
/* Fix MISRA_C-2012 Rule 17.7. */
#define LOG_INFO (void)PRINTF
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
flexcan_handle_t flexcanHandle;
volatile bool txComplete = false;
volatile bool rxComplete = false;
volatile bool wakenUp    = false;
flexcan_mb_transfer_t txXfer, rxXfer;
flexcan_fifo_transfer_t rxFifoXfer;
flexcan_frame_t txFrame;
AT_NONCACHEABLE_SECTION(flexcan_frame_t rxFrame);
uint32_t txIdentifier;
uint32_t rxIdentifier;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief FlexCAN Call Back function
 */
static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    if ((kStatus_FLEXCAN_TxIdle == status) && (TX_MESSAGE_BUFFER_NUM == result))
    {
        txComplete = true;
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    flexcan_config_t flexcanConfig;
    flexcan_rx_fifo_config_t rxFifoConfig;
    uint32_t rxFifoFilter[] = {
        FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0, 0, 0), FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0, 1, 0),
        FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0, 0, 0), FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0, 1, 0)};
    uint8_t node_type;

    /* Initialize board hardware. */
    sc_ipc_t ipc;

    ipc = BOARD_InitRpc();

    BOARD_InitPins(ipc);
    BOARD_BootClockRUN();
    BOARD_InitMemory();
    BOARD_InitDebugConsole();

    /* Power on Peripherals. */
    if (sc_pm_set_resource_power_mode(ipc, SC_R_IRQSTR_M4_0, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        PRINTF("Error: Failed to power on IRQSTEER!\r\n");
    }
    if (sc_pm_set_resource_power_mode(ipc, CAN_POWER, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        PRINTF("Error: Failed to power on FLEXCAN\r\n");
    }

    /* Set Peripheral clock frequency. */
    if (CLOCK_SetIpFreq(CAN_CLOCK, SC_80MHZ) == 0)
    {
        PRINTF("Error: Failed to set FLEXCAN frequency\r\n");
    }

    /* Enable interrupt in irqsteer */
    IRQSTEER_Init(IRQSTEER);
    IRQSTEER_EnableInterrupt(IRQSTEER, CAN_INTERRUPT);

    LOG_INFO("********* FLEXCAN Interrupt EXAMPLE *********\r\n");
    LOG_INFO("    Message format: Standard (11 bit id)\r\n");
    LOG_INFO("    Message buffer %d used for Tx.\r\n", TX_MESSAGE_BUFFER_NUM);
    LOG_INFO("    Interrupt Mode: Enabled\r\n");
    LOG_INFO("    Operation Mode: TX and RX --> Normal\r\n");
    LOG_INFO("*********************************************\r\n\r\n");

    do
    {
        LOG_INFO("Please select local node as A or B:\r\n");
        LOG_INFO("Note: Node B should start first.\r\n");
        LOG_INFO("Node:");
        node_type = GETCHAR();
        LOG_INFO("%c", node_type);
        LOG_INFO("\r\n");
    } while ((node_type != 'A') && (node_type != 'B') && (node_type != 'a') && (node_type != 'b'));

    /* Select mailbox ID. */
    if ((node_type == 'A') || (node_type == 'a'))
    {
        txIdentifier = 0x321;
        rxIdentifier = 0x123;
    }
    else
    {
        txIdentifier = 0x123;
        rxIdentifier = 0x321;
    }

    /* Get FlexCAN module default Configuration. */
    /*
     * flexcanConfig.clkSrc                 = kFLEXCAN_ClkSrc0;
     * flexcanConfig.baudRate               = 1000000U;
     * flexcanConfig.baudRateFD             = 2000000U;
     * flexcanConfig.maxMbNum               = 16;
     * flexcanConfig.enableLoopBack         = false;
     * flexcanConfig.enableSelfWakeup       = false;
     * flexcanConfig.enableIndividMask      = false;
     * flexcanConfig.disableSelfReception   = false;
     * flexcanConfig.enableListenOnlyMode   = false;
     * flexcanConfig.enableDoze             = false;
     */
    FLEXCAN_GetDefaultConfig(&flexcanConfig);

#if defined(EXAMPLE_CAN_CLK_SOURCE)
    flexcanConfig.clkSrc = EXAMPLE_CAN_CLK_SOURCE;
#endif

/* If special quantum setting is needed, set the timing parameters. */
    flexcanConfig.timingConfig.phaseSeg1 = PSEG1;
    flexcanConfig.timingConfig.phaseSeg2 = PSEG2;
    flexcanConfig.timingConfig.propSeg   = PROPSEG;

    FLEXCAN_Init(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);

    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

    /* Set Rx Masking mechanism. */
    FLEXCAN_SetRxMbGlobalMask(EXAMPLE_CAN, FLEXCAN_RX_MB_STD_MASK(rxIdentifier, 0, 0));

    // /* Setup Rx Message Buffer. */
    // mbConfig.format = kFLEXCAN_FrameFormatStandard;
    // mbConfig.type   = kFLEXCAN_FrameTypeData;
    // mbConfig.id     = FLEXCAN_ID_STD(rxIdentifier);
    // FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);

    /* Setup Tx Message Buffer. */
    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);

    /* Setup Rx FIFO. */
    rxFifoConfig.idFilterTable = rxFifoFilter;
    rxFifoConfig.idFilterType  = kFLEXCAN_RxFifoFilterTypeA;
    rxFifoConfig.idFilterNum   = sizeof(rxFifoFilter) / sizeof(rxFifoFilter[0]);
    rxFifoConfig.priority      = kFLEXCAN_RxFifoPrioHigh;
    FLEXCAN_SetRxFifoConfig(EXAMPLE_CAN, &rxFifoConfig, true);

    while (true)
    {
        if ((node_type == 'A') || (node_type == 'a'))
        {
            GETCHAR();

            txFrame.id     = FLEXCAN_ID_STD(txIdentifier);
            txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
            txFrame.type   = (uint8_t)kFLEXCAN_FrameTypeData;
            txFrame.length = (uint8_t)DLC;
            txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
            txXfer.frame = &txFrame;
            (void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);

            while (!txComplete)
            {
            };
            txComplete = false;

            /* Receive data through Rx FIFO. */
            rxFifoXfer.frame = &rxFrame;
            for (size_t i = 0; i < 4; i++)
            {
                (void)FLEXCAN_TransferReceiveFifoNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxFifoXfer);
                while (!rxComplete)
                {
                }
                rxComplete = false;

                LOG_INFO("Receive Msg%d from FIFO: word0 = 0x%x, word1 = 0x%x, ID Filter Hit%d.\r\n", i + 1, rxFrame.dataWord0,
                        rxFrame.dataWord1, rxFrame.idhit);
            }
        }
        else
        {
            /* Before this , should first make node B enter STOP mode after FlexCAN
             * initialized with enableSelfWakeup=true and Rx MB configured, then A
             * sends frame N which wakes up node B. A will continue to send frame N
             * since no acknowledgement, then B received the second frame N(In the
             * application it seems that B received the frame that woke it up which
             * is not expected as stated in the reference manual, but actually the
             * output in the terminal B received is the same second frame N). */
            if (wakenUp)
            {
                LOG_INFO("B has been waken up!\r\n\r\n");
            }

            rxFifoXfer.frame = &rxFrame;
            for (size_t i = 0; i < 4; i++)
            {
                (void)FLEXCAN_TransferReceiveFifoNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxFifoXfer);
                while (!rxComplete)
                {
                }
                rxComplete = false;

                LOG_INFO("Receive Msg%d from FIFO: word0 = 0x%x, word1 = 0x%x, ID Filter Hit%d.\r\n", i + 1, rxFrame.dataWord0,
                        rxFrame.dataWord1, rxFrame.idhit);
            }

            txFrame.id     = FLEXCAN_ID_STD(txIdentifier);
            txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
            txXfer.frame = &txFrame;
            (void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);

            while (!txComplete)
            {
            };
            txComplete = false;
            LOG_INFO("Wait Node A to trigger the next transmission!\r\n\r\n");
        }
    }
}
