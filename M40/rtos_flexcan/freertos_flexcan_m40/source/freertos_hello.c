/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "fsl_lpuart.h"
#include "fsl_irqsteer.h"
#include "fsl_flexcan.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)
#define EXAMPLE_CAN          DMA__CAN0
#define EXAMPLE_CAN_CLK_FREQ CLOCK_GetIpFreq(kCLOCK_DMA_Can0)
#define EXAMPLE_CAN_POWER    SC_R_CAN_0
#define EXAMPLE_CAN_IRQn     DMA_FLEXCAN0_INT_IRQn
#define EXAMPLE_CAN_CLOCK    kCLOCK_DMA_Can0
#define DLC                  (0)
#define PSEG1                6
#define PSEG2                4
#define PROPSEG              6
#define LOG_INFO             (void)PRINTF
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void hello_task(void *pvParameters);
flexcan_handle_t flexcanHandle;
volatile bool txComplete = false;
volatile bool rxComplete = false;
volatile bool wakenUp    = false;
flexcan_mb_transfer_t txXfer, rxXfer;
flexcan_frame_t frame;
uint32_t txIdentifier;
uint32_t rxIdentifier;
/*******************************************************************************
 * Code
 ******************************************************************************/
static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    switch (status)
    {

        case kStatus_FLEXCAN_RxIdle:
        case kStatus_FLEXCAN_RxRemote:
            if ((RX_MESSAGE_BUFFER_NUM == result) || (TX_MESSAGE_BUFFER_NUM == result))
            {
                rxComplete = true;
            }
            break;

        case kStatus_FLEXCAN_TxIdle :
        case kStatus_FLEXCAN_TxSwitchToRx :
            if (TX_MESSAGE_BUFFER_NUM == result)
            {
                txComplete = true;
            }
            break;

        case kStatus_FLEXCAN_WakeUp:
            wakenUp = true;
            break;

        default:
            break;
    }
}
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    sc_ipc_t ipc = BOARD_InitRpc();

    BOARD_InitPins(ipc);
    BOARD_BootClockRUN();
    BOARD_InitMemory();
    BOARD_InitDebugConsole();
    /* Power on Peripherals. */
    if (sc_pm_set_resource_power_mode(ipc, SC_R_IRQSTR_M4_0, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        PRINTF("Error: Failed to power on IRQSTEER!\r\n");
    }
    if (sc_pm_set_resource_power_mode(ipc, EXAMPLE_CAN_POWER, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
    {
        PRINTF("Error: Failed to power on FLEXCAN\r\n");
    }

    /* Set Peripheral clock frequency. */
    if (CLOCK_SetIpFreq(EXAMPLE_CAN_CLOCK, SC_80MHZ) == 0)
    {
        PRINTF("Error: Failed to set FLEXCAN frequency\r\n");
    }

    /* Enable interrupt in irqsteer */
    IRQSTEER_Init(IRQSTEER);
    IRQSTEER_EnableInterrupt(IRQSTEER, EXAMPLE_CAN_IRQn);
    
    if (xTaskCreate(hello_task, "Hello_task", configMINIMAL_STACK_SIZE + 100, NULL, hello_task_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void hello_task(void *pvParameters)
{
    flexcan_config_t flexcanConfig;
    flexcan_rx_mb_config_t mbConfig;
    uint8_t node_type;
    LOG_INFO("********* FLEXCAN Interrupt EXAMPLE *********\r\n");
    LOG_INFO("    Message format: Standard (11 bit id)\r\n");
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
        txIdentifier = 0x701;
        rxIdentifier = 0x123;
    }
    else
    {
        txIdentifier = 0x123;
        rxIdentifier = 0x321;
    }
    FLEXCAN_GetDefaultConfig(&flexcanConfig);
    //flexcanConfig.enableLoopBack         = true;
    flexcanConfig.disableSelfReception   = true;
    flexcanConfig.timingConfig.phaseSeg1 = PSEG1;
    flexcanConfig.timingConfig.phaseSeg2 = PSEG2;
    flexcanConfig.timingConfig.propSeg   = PROPSEG;
    FLEXCAN_Init(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);

    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

    /* Set Rx Masking mechanism. */
    FLEXCAN_SetRxMbGlobalMask(EXAMPLE_CAN, FLEXCAN_RX_MB_STD_MASK(0, 0, 0));

    /* Setup Rx Message Buffer. */
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;
    mbConfig.id     = FLEXCAN_ID_STD(0);
    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);

    if ((node_type == 'A') || (node_type == 'a'))
    {
        LOG_INFO("Press any key to trigger one-shot transmission\r\n\r\n");
        frame.dataByte0 = 0;
    }
    else
    {
        LOG_INFO("Start to Wait data from Node A\r\n\r\n");
    }

    for (;;)
    {
        if ((node_type == 'A') || (node_type == 'a'))
        {
            // GETCHAR();
#if 1
            frame.id     = FLEXCAN_ID_STD(txIdentifier);
            frame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
            frame.type   = (uint8_t)kFLEXCAN_FrameTypeRemote;
            frame.length = (uint8_t)DLC;
            // frame.dataByte0 = 0x40;
            // frame.dataByte1 = 0x41;
            // frame.dataByte3 = 0x60;
            // frame.dataByte4 = 0x00;
            // frame.dataByte5 = 0x00;
            // frame.dataByte6 = 0x00;
            // frame.dataByte7 = 0x00;
#if (defined(USE_CANFD) && USE_CANFD)
            frame.brs = (uint8_t)1U;
#endif
            txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
#if (defined(USE_CANFD) && USE_CANFD)
            txXfer.framefd = &frame;
            (void)FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
#else
            txXfer.frame = &frame;
            (void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
#endif

            while (!txComplete)
            {
            };
            txComplete = false;
#endif
            /* Start receive data through Rx Message Buffer. */
            rxXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
#if (defined(USE_CANFD) && USE_CANFD)
            rxXfer.framefd = &frame;
            (void)FLEXCAN_TransferFDReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
#else
            rxXfer.frame = &frame;
            (void)FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
#endif

            /* Wait until Rx MB full. */
            while (!rxComplete)
            {
            };
            rxComplete = false;
            LOG_INFO("Rx MB ID: 0x%3x, Rx MB data: 0x%x, Time stamp: %d\r\n", frame.id >> CAN_ID_STD_SHIFT,
                     frame.dataByte0, frame.timestamp);
            if(((frame.id >> CAN_ID_STD_SHIFT) & 0x0780) == 0x0080)
            {
                rxXfer.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
                rxXfer.frame = &frame;
                (void)FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
                while (!rxComplete)
                {
                };
                rxComplete = false;
                LOG_INFO("Rx MB ID: 0x%3x, Rx MB data: 0x%x, Time stamp: %d\r\n", frame.id >> CAN_ID_STD_SHIFT,
                     frame.dataByte0, frame.timestamp);
            }

            // LOG_INFO("Rx MB ID: 0x%3x, Rx MB data: 0x%x, Time stamp: %d\r\n", frame.id >> CAN_ID_STD_SHIFT,
            //          frame.dataByte0, frame.timestamp);
            // LOG_INFO("Press any key to trigger the next transmission!\r\n\r\n");
            // frame.dataByte0++;
            // frame.dataByte1 = 0x55;
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

            /* Start receive data through Rx Message Buffer. */
            rxXfer.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
#if (defined(USE_CANFD) && USE_CANFD)
            rxXfer.framefd = &frame;
            (void)FLEXCAN_TransferFDReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
#else
            frame.length = 1;
            rxXfer.frame = &frame;
            (void)FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
#endif

            /* Wait until Rx receive full. */
            while (!rxComplete)
            {
            };
            rxComplete = false;

            LOG_INFO("Rx MB ID: 0x%3x, Rx MB data: 0x%x, Time stamp: %d\r\n", frame.id >> CAN_ID_STD_SHIFT,
                     frame.dataByte0, frame.timestamp);

            frame.id     = FLEXCAN_ID_STD(txIdentifier);
            txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
#if (defined(USE_CANFD) && USE_CANFD)
            frame.brs      = 1;
            txXfer.framefd = &frame;
            (void)FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
#else
            txXfer.frame = &frame;
            (void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
#endif

            while (!txComplete)
            {
            };
            txComplete = false;
            LOG_INFO("Wait Node A to trigger the next transmission!\r\n\r\n");
        }
    }
}
