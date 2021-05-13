#include "uart0.h"

/*GLOBAL_VARIABLE***************************************************************
 *
 * Declare static global variables
 *
*END***************************************************************************/
static UART0_callback_Func s_callback;
static uint8_t s_charReceived;

/*FUNCTION_PROTOTYPES***********************************************************
 *
 * Declare prototypes static function
 *
*END***************************************************************************/
static void UART_config_Baudrate(uint32_t BR);

/*FUNCTION**********************************************************************
 *
 * Name        : UART_config_Baudrate
 * Description : config baudrate register automatically
 *
*END***************************************************************************/
static void UART_config_Baudrate(uint32_t BR)
{
    uint32_t targetBR;
    uint8_t targetOSR;

    uint32_t br;
    uint8_t osr;

    uint32_t minError = 0xFFFFFFFF;

    uint32_t error1;
    uint32_t error2;

    uint32_t tempCLK_BR;
    tempCLK_BR = (UART_ASYNCH_CLK) / (BR);

    for( osr = MIN_OSR; osr <= MAX_OSR; osr++)
    {
        br = (uint32_t)(tempCLK_BR)/(osr);
        
        if(br == 0)
        {
            br = 1;
        }

        if( (br >= MIN_BR) && (br <= MAX_BR) )
        {
            error1 = tempCLK_BR - br*osr;
            error2 = (br+1)*osr - tempCLK_BR;

            if(error1 < minError)
            {
                minError = error1;

                targetBR = br;
                targetOSR = osr;
            }
            else if(error2 < minError)
            {
                minError = error2;

                targetBR = br+1;
                targetOSR = osr;
            }

            if(minError == 0)
            {
                break;
            }
        }
    }

    /* clear and write value to register BDH,BDL,C4 to config baudrate */
    UART0->BDH &= ~UART0_BDH_SBR_MASK;
    UART0->BDH |= UART0_BDH_SBR( (targetBR  & UART0_BDH_MASK) >> UART0_BDH_SHIFT );

    UART0->BDL &= ~UART0_BDL_SBR_MASK;
    UART0->BDL |= UART0_BDL_SBR( targetBR & UART0_BDL_MASK );

    UART0->C4 = UART0_C4_OSR(targetOSR-1);

}
/*FUNCTION**********************************************************************
*
* Name        : UART_init
* Description : initialize and config clock, port, register for UART0
*
*END***************************************************************************/
void UART0_init(BAUD_RATEn_Type BRn,UART0_callback_Func callback)
{
    /* config PORTA_RX_TX */
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    /* clear and config PTA1(RX0), PTA2(TX0) */
    PORTA->PCR[UART0_RX_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[UART0_TX_PIN] &= ~PORT_PCR_MUX_MASK;

    PORTA->PCR[UART0_RX_PIN] |= PORT_PCR_MUX(2);
    PORTA->PCR[UART0_TX_PIN] |= PORT_PCR_MUX(2);

    /* config clock for UART0 */
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

    /* clear and select MCGFLLCLK for UART0 */
    SIM->SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK;
    SIM->SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;

    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);

    /* disable Transmitter, Receiver UART0 */
    UART0->C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK);

    /* Data bit: 8 bit */
    UART0->C1 &= ~UART0_C1_M_MASK;

    /* Parity: None */
    UART0->C1 &= ~UART0_C1_PE_MASK;

    /* Stop bit: 1 bit */
    UART0->BDH &= ~UART0_BDH_SBNS_MASK;

    /* config BR, OSR for UART0 */
    UART_config_Baudrate(BRn);

    /* config data transmit not inverted */
    UART0->C3 &= ~UART0_C3_TXINV_MASK;

    /* config LSB first */
    UART0->S2 &= ~UART0_S2_MSBF_MASK;

    /* enable NVIC for UART0_IRQ */
    NVIC_EnableIRQ(UART0_IRQn);

    /* enable UART0_Receive_ISR */
    UART0->C2 |= UART0_C2_RIE_MASK;

    /* Enable: Transmitter, Receiver UART0 */
    UART0->C2 |= UART0_C2_TE_MASK | UART0_C2_RE_MASK;

    /* callback function */
    s_callback = callback;

}
/*FUNCTION**********************************************************************
*
* Name        : UART0_puts
* Description : put string on UART0
*
*END***************************************************************************/
void UART0_puts(uint8_t *str)
{
    uint8_t i = 0;

    while ( (str[i] != '\0') )
    {
        /* wait until transmit data buffer empty */
        while ( !(UART0->S1 & UART0_S1_TDRE_MASK) );

        UART0->D = str[i];
        i++;
    }
}
/*FUNCTION**********************************************************************
*
* Name        : UART0_IRQHandler
* Description : receive each character from PC through terminal
*
*END***************************************************************************/
void UART0_IRQHandler(void)
{
    s_charReceived = UART0->D;

    s_callback(s_charReceived);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/