#ifndef _KL46_H_
#define _KL46_H_
/*TYPEDEF***********************************************************************
 *
 * Create typedef and define data structure
 *
*END**************************************************************************/
typedef unsigned char   uint8_t;
typedef unsigned int    uint32_t;

typedef char            int8_t;
typedef int             int32_t;

#ifndef __cplusplus

#define bool            _Bool
#define true            1
#define false           0

#endif /* __cplusplus */

#ifdef __cplusplus
    #define __I         volatile        /* 'read only' permissions */
#else
    #define __I         volatile const  /* 'read only' permissions */
#endif

#define __O             volatile        /* 'write only' permissions */
#define __IO            volatile        /* 'read/write' permissions */

typedef struct
{
    __IO    uint32_t PCR[32];           /* Pin Control Register n, array offset 0x0, array step: 0x4 */
    __O     uint32_t GPCLR;             /* Global Pin Control Low Register, offset 0x80 */
    __O     uint32_t GPCHR;             /* Global Pin Control High Register, offset 0x84 */
            uint8_t Reverse[24];
    __IO    uint32_t ISFR;              /* Interrupt Status Flag Register, offset: 0xA0 */

} PORT_Type;

typedef struct
{
    __IO    uint32_t PDOR;              /* Port Data Output Register, offset: 0x0 */
    __O     uint32_t PSOR;              /* Port Set Output Register, offset: 0x4 */
    __O     uint32_t PCOR;              /* Port Clear Output Register, offset: 0x8 */
    __O     uint32_t PTOR;              /* Port Toggle Output Register, offset: 0xC */
    __I     uint32_t PDIR;              /* Port Data Input Register, offset: 0x10 */
    __IO    uint32_t PDDR;              /* Port Data Direction Register, offset: 0x14 */

} GPIO_Type;

/*DEFINE***********************************************************************
 *
 * Create define
 *
*END**************************************************************************/
#define PORTC_BASE      (0x4004B000u)                   /* Peripheral PORTC base address */
#define PORTC           ( (PORT_Type*) PORTC_BASE )     /* Peripheral PORTC base pointer */

#define PORTE_BASE      (0x4004D000u)                   /* Peripheral PORTE base address */
#define PORTE           ( (PORT_Type*) PORTE_BASE )     /* Peripheral PORTE base pointer */

#define GPIOC_BASE      (0x400FF080u)                   /* Peripheral GPIOC base address */
#define GPIOC           ( (GPIO_Type*) GPIOC_BASE )     /* Peripheral GPIOC base pointer */

#define GPIOE_BASE      (0x400FF100u)                   /* Peripheral GPIOE base address */
#define GPIOE           ( (GPIO_Type*) GPIOE_BASE )     /* Peripheral GPIOE base pointer */

#define SIM_SCGC5_BASE  (0x40048038u)                   /* Peripheral SIM SCGC5 base address */
#define SIM_SCGC5       ( (uint32_t*) SIM_SCGC5_BASE )  /* Peripheral SIM SCGC5 base pointer */

#define PORTC_SIM_SCGC5     11u
#define PORTE_SIM_SCGC5     13u

#define MUX_SHIFT           8u
#define MUX_MASK            0x700u

#define PE_SHIFT            1u                          /* Pull Enable shift */
#define PS_SHIFT            0                           /* Pull Select shift */

#endif /*_KL46_H_*/
/*****************************************************************************
 * EOF
 *****************************************************************************/