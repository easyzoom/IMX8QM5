#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

#include "board.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/* M40_I2C0_SCL (number AM44), FTDI_M40_UART0_RX */
#define BOARD_INITPINS_FTDI_M40_UART0_RX_PIN_FUNCTION_ID       SC_P_M40_I2C0_SCL   /*!< Pin function id */

/* M40_I2C0_SDA (number AU51), FTDI_M40_UART0_TX */
#define BOARD_INITPINS_FTDI_M40_UART0_TX_PIN_FUNCTION_ID       SC_P_M40_I2C0_SDA   /*!< Pin function id */

/* FLEXCAN0_TX (coord H6), BB_CAN0_TX/J20C[26] */
#define BOARD_INITPINS_BB_CAN0_TX_PIN_FUNCTION_ID               SC_P_FLEXCAN0_TX   /*!< Pin function id */

/* FLEXCAN0_RX (coord C5), BB_CAN0_RX/J20C[25] */
#define BOARD_INITPINS_BB_CAN0_RX_PIN_FUNCTION_ID               SC_P_FLEXCAN0_RX   /*!< Pin function id */

/* FLEXCAN1_TX (coord G7), BB_CAN1_TX/J20C[10] */
#define BOARD_INITPINS_BB_CAN1_TX_PIN_FUNCTION_ID               SC_P_FLEXCAN1_TX   /*!< Pin function id */

/* FLEXCAN1_RX (coord E5), BB_CAN1_RX/J20C[11] */
#define BOARD_INITPINS_BB_CAN1_RX_PIN_FUNCTION_ID               SC_P_FLEXCAN1_RX   /*!< Pin function id */

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 * @param ipc scfw ipchandle.
 *
 */
void BOARD_InitPins(sc_ipc_t ipc);                         /*!< Function assigned for the core: Cortex-M4F[cm4_core0] */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
