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

/* M41_GPIO0_00 (number AP44), BB_UART4_RX_AUDIN/J20C[34] */
#define BOARD_INITPINS_BB_UART4_RX_AUDIN_PIN_FUNCTION_ID       SC_P_M41_GPIO0_00   /*!< Pin function id */

/* M41_GPIO0_01 (number AU47), BB_UART4_TX_AUDIN/J20C[35] */
#define BOARD_INITPINS_BB_UART4_TX_AUDIN_PIN_FUNCTION_ID       SC_P_M41_GPIO0_01   /*!< Pin function id */

/* M41_I2C0_SCL (number AR45), BB_M41_I2C0_1V8_SCL/J20A[22] */
#define BOARD_INITPINS_BB_M41_I2C0_1V8_SCL_PIN_FUNCTION_ID     SC_P_M41_I2C0_SCL   /*!< Pin function id */

/* M41_I2C0_SDA (number AU49), BB_M41_I2C0_1V8_SDA/J20A[23] */
#define BOARD_INITPINS_BB_M41_I2C0_1V8_SDA_PIN_FUNCTION_ID     SC_P_M41_I2C0_SDA   /*!< Pin function id */

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
void BOARD_InitPins(sc_ipc_t ipc);                         /*!< Function assigned for the core: Cortex-M4F[cm4_core1] */

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
