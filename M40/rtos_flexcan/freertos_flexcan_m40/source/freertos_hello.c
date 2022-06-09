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
#define DLC                  (8)
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
int remote_frame = 0;
enum
{
  PHASE_START                   = 0,
  PHASE_FLUSH_RX_MSG               ,
  PHASE_CHECK_TX_BOX               ,
  PHASE_SEND_REQ_MSG               ,
  PHASE_CHECK_RX_BOX               ,
  PHASE_RECV_RSP_MSG               ,
  PHASE_BUILDING_RET               ,
  PHASE_END                        ,
  PHASE_ERROR                      ,
};
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
        txIdentifier = 0x601;
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
    int phase_step = PHASE_START;
    uint32_t phase_tick = xTaskGetTickCount();
    status_t  status = kStatus_Success;
    txIdentifier = 0x601;

    for (;;)
    {
        if ((node_type == 'A') || (node_type == 'a'))
        {
            switch (phase_step)
            {
                case PHASE_START:
                    phase_step = PHASE_FLUSH_RX_MSG;
                    break;
                case PHASE_FLUSH_RX_MSG:
                    {
                        status = FLEXCAN_ReadRxMb(EXAMPLE_CAN, (uint8_t)RX_MESSAGE_BUFFER_NUM, &frame);
                        if (status != kStatus_Success)
                        {
                            PRINTF("%02x %02x %02x %02x %02x %02x %02x %d %x\r\n", \
                                            txXfer.frame->dataByte0, txXfer.frame->dataByte1, txXfer.frame->dataByte2,\
                                            txXfer.frame->dataByte3, txXfer.frame->dataByte4, txXfer.frame->dataByte5,\
                                            txXfer.frame->dataByte6, txXfer.frame->dataByte7, remote_frame, txIdentifier);
                            PRINTF("ERROR: RxMessage Error. %s:%d status:%d,mb:%x,inter:%x\r\n", __FUNCTION__, __LINE__,status,FLEXCAN_GetMbStatusFlags(EXAMPLE_CAN, 1 << rxXfer.mbIdx),FLEXCAN_GetStatusFlags(EXAMPLE_CAN));
                        }
                        FLEXCAN_TransferAbortReceive(EXAMPLE_CAN, &flexcanHandle, (uint8_t)RX_MESSAGE_BUFFER_NUM);
                        FLEXCAN_TransferAbortSend(EXAMPLE_CAN, &flexcanHandle, (uint8_t)TX_MESSAGE_BUFFER_NUM);
                        FLEXCAN_ClearMbStatusFlags(EXAMPLE_CAN, 1 << (uint8_t)TX_MESSAGE_BUFFER_NUM);
                        FLEXCAN_ClearMbStatusFlags(EXAMPLE_CAN, 1 << (uint8_t)RX_MESSAGE_BUFFER_NUM);
                        // PRINTF("start:%x %02x %02x %02x %02x %02x %02x %02x\r\n", (rxXfer.frame->id >> CAN_ID_STD_SHIFT),\
                        //                 rxXfer.frame->dataByte0, rxXfer.frame->dataByte1, rxXfer.frame->dataByte2,\
                        //                 rxXfer.frame->dataByte3, rxXfer.frame->dataByte4, rxXfer.frame->dataByte5,\
                        //                 rxXfer.frame->dataByte6, rxXfer.frame->dataByte7);
                        phase_step = PHASE_CHECK_TX_BOX;
                        phase_tick = xTaskGetTickCount();
                    }
                    break;
                case PHASE_CHECK_TX_BOX:
                    {
                        frame.id     = FLEXCAN_ID_STD(txIdentifier);
                        frame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
                        frame.type   = (uint8_t)(remote_frame ? kFLEXCAN_FrameTypeRemote : kFLEXCAN_FrameTypeData);
                        frame.length = (uint8_t)DLC;
                        frame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x40) | CAN_WORD0_DATA_BYTE_1(0x41) | 
                        CAN_WORD0_DATA_BYTE_2(0x60) | CAN_WORD0_DATA_BYTE_3(0x00);
                        frame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x00) | CAN_WORD1_DATA_BYTE_5(0x00) | 
                        CAN_WORD1_DATA_BYTE_6(0x00) | CAN_WORD1_DATA_BYTE_7(0x00);
                        txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
                        txXfer.frame = &frame;
                        status = FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
                        if (status != kStatus_Success)
                        {
                            PRINTF("remote_frame:%d tx_buf:%x ", remote_frame, txIdentifier);
                            PRINTF("ERROR: TxMessage Error. %s:%d status:%d\r\n", __FUNCTION__, __LINE__,status);
                        }
                            phase_step = PHASE_SEND_REQ_MSG;
                            phase_tick = xTaskGetTickCount();
                    }
                    break;
                case PHASE_SEND_REQ_MSG:
                    if(txComplete)
                    {
                        txComplete = false;
                        phase_step = PHASE_CHECK_RX_BOX;
                        phase_tick = xTaskGetTickCount();
                    }
                    else
                    if (xTaskGetTickCount() > phase_tick + 10)
                    {
                        PRINTF("ERROR: TxMessage Timeout. %s:%d\r\n", __FUNCTION__, __LINE__);
                    }
                    break;
                case PHASE_CHECK_RX_BOX:
                    {
                        /* receive response */
                        rxXfer.mbIdx = (uint8_t)(remote_frame ? (uint8_t)TX_MESSAGE_BUFFER_NUM : (uint8_t)RX_MESSAGE_BUFFER_NUM);
                        rxXfer.frame = &frame;
                        if((uint8_t)0 == flexcanHandle.mbState[rxXfer.mbIdx])
                        {
                            status = FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
                            if (status != kStatus_Success)
                            {
                                PRINTF("%02x %02x %02x %02x %02x %02x %02x %d %x\r\n", \
                                                txXfer.frame->dataByte0, txXfer.frame->dataByte1, txXfer.frame->dataByte2,\
                                                txXfer.frame->dataByte3, txXfer.frame->dataByte4, txXfer.frame->dataByte5,\
                                                txXfer.frame->dataByte6, txXfer.frame->dataByte7, remote_frame, txIdentifier);
                                PRINTF("ERROR: RxMessage Error. %s:%d status:%d,mb:%x,inter:%x\r\n", __FUNCTION__, __LINE__,status,FLEXCAN_GetMbStatusFlags(EXAMPLE_CAN, 1 << rxXfer.mbIdx),FLEXCAN_GetStatusFlags(EXAMPLE_CAN));
                            }
                            phase_step = PHASE_RECV_RSP_MSG;
                            phase_tick = xTaskGetTickCount();
                        }
                        if (xTaskGetTickCount() > phase_tick +  10)
                        {
                            PRINTF("ERROR: RxMB Error. %s:%d status:%d,mb:%x,inter:%x\r\n", __FUNCTION__, __LINE__,status,FLEXCAN_GetMbStatusFlags(EXAMPLE_CAN, 1 << rxXfer.mbIdx),FLEXCAN_GetStatusFlags(EXAMPLE_CAN));
                        }
                    }
                    break;
                case PHASE_RECV_RSP_MSG:
                    {
                        if(!rxComplete)
                        {
                            if (xTaskGetTickCount() > phase_tick + 10)
                            {
                                PRINTF("mb:%x,inter:%x\r\n",FLEXCAN_GetMbStatusFlags(EXAMPLE_CAN, 1 << rxXfer.mbIdx),FLEXCAN_GetStatusFlags(EXAMPLE_CAN));
                                PRINTF("%02x %02x %02x %02x %02x %02x %02x %d %x\r\n", \
                                            txXfer.frame->dataByte0, txXfer.frame->dataByte1, txXfer.frame->dataByte2,\
                                            txXfer.frame->dataByte3, txXfer.frame->dataByte4, txXfer.frame->dataByte5,\
                                            txXfer.frame->dataByte6, txXfer.frame->dataByte7, remote_frame, txIdentifier);
                                PRINTF("remote_frame:%d rx_buf:%x ", remote_frame, (rxXfer.frame->id >> CAN_ID_STD_SHIFT));
                                PRINTF("%02x %02x %02x %02x %02x %02x %02x\r\n", \
                                        rxXfer.frame->dataByte0, rxXfer.frame->dataByte1, rxXfer.frame->dataByte2,\
                                        rxXfer.frame->dataByte3, rxXfer.frame->dataByte4, rxXfer.frame->dataByte5,\
                                        rxXfer.frame->dataByte6, rxXfer.frame->dataByte7);
                                PRINTF("ERROR: RxMessage Timeout. %s:%d\r\n", __FUNCTION__, __LINE__);
                            }
                        }
                        if(rxComplete)
                        {
                            rxComplete = false;
                            phase_step = PHASE_BUILDING_RET;
                            phase_tick = xTaskGetTickCount();
                        }
                    }
                    break;
                case PHASE_BUILDING_RET:
                    {
                        LOG_INFO("Rx MB ID: 0x%3x, Rx MB data: 0x%x, Time stamp: %d\r\n", frame.id >> CAN_ID_STD_SHIFT, frame.dataByte0, frame.timestamp);
                        if(remote_frame == 0)
                        {
                            remote_frame = 1;
                            txIdentifier = 0x601;
                        }
                        else
                        {
                            remote_frame = 0;
                            txIdentifier = 0x701;
                        }
                    }
                    break;
                
                default:
                    break;
            }
        }
        else
        {
            PRINTF("node_type ERROR: \r\n");
        }
    }
}
