#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

#include "board.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/* UART0_RTS_B (number AU45), BB_UART2_RX/J20A[28] */
#define BOARD_INITPINS_BB_UART2_RX_PIN_FUNCTION_ID              SC_P_UART0_RTS_B   /*!< Pin function id */

/* UART0_CTS_B (number AW49), BB_UART2_TX/J20A[29] */
#define BOARD_INITPINS_BB_UART2_TX_PIN_FUNCTION_ID              SC_P_UART0_CTS_B   /*!< Pin function id */

/* M40_GPIO0_00 (number AR47), BB_ARD_MIK_UART3_RX/J20A[34] */
#define BOARD_INITPINS_BB_ARD_MIK_UART3_RX_PIN_FUNCTION_ID     SC_P_M40_GPIO0_00   /*!< Pin function id */

/* M40_GPIO0_01 (number AU53), BB_ARD_MIK_UART3_TX/J20A[35] */
#define BOARD_INITPINS_BB_ARD_MIK_UART3_TX_PIN_FUNCTION_ID     SC_P_M40_GPIO0_01   /*!< Pin function id */

/* UART1_TX (number AY48), BT_UART1_TX */
#define BOARD_INITPINS_BT_UART1_TX_PIN_FUNCTION_ID                 SC_P_UART1_TX   /*!< Pin function id */

/* UART1_RX (number AT44), BT_UART1_RX */
#define BOARD_INITPINS_BT_UART1_RX_PIN_FUNCTION_ID                 SC_P_UART1_RX   /*!< Pin function id */

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
