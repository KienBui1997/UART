#ifndef _UART0_H_
#define _UART0_H_

#include "MKL46Z4.h"

/*DEFINE************************************************************************
 *
 * Create define
 *
*END***************************************************************************/
#define UART0_RX_PIN            (1u)
#define UART0_TX_PIN            (2u)

#define UART0_BDH_MASK          (0x1F00u)
#define UART0_BDH_SHIFT         (8u)
#define UART0_BDL_MASK          (0xFFu)

#define UART_ASYNCH_CLK         (20971520u)
#define MIN_OSR                 (4u)
#define MAX_OSR                 (32u)
#define MIN_BR                  (1u)
#define MAX_BR                  (8191u)

typedef enum BRn
{
    Baud_rate_600       =   600u,
    Baud_rate_2400      =   2400u,
    Baud_rate_4800      =   4800u,
    Baud_rate_9600      =   9600u,
    Baud_rate_38400     =   38400u,
    Baud_rate_56000     =   56000u,
    Baud_rate_115200    =   115200u,
    Baud_rate_256000    =   256000u

} BAUD_RATEn_Type;

/*FUNCTION_PROTOTYPES***********************************************************
 *
 * Declare functions to serve program
 *
*END***************************************************************************/
typedef void (*UART0_callback_Func)(uint8_t character);

void UART0_init(BAUD_RATEn_Type BRn,UART0_callback_Func callback);

void UART0_puts(uint8_t *str);

#endif /* _UART0_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/