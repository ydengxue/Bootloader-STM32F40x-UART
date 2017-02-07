/*****************************************************************************************************
* Description:                 STM32F40x Registers definiton
*
* Author:                      Dengxue Yan, Washington University in St. Louis
*
* Email:                       Dengxue.Yan@wustl.edu
*
* Rev History:
*       <Author>        <Date>        <Hardware>     <Version>        <Description>
*     Dengxue Yan   09/15/2016 15:30       --           1.00             Create
*****************************************************************************************************/
#ifndef _STM32F40x_REGISTER_H_
#define _STM32F40x_REGISTER_H_

//====================================================================================================
// External header files reference
//====================================================================================================
#include "UserTypesDef.h"
#include "Debug.h"

#ifdef __cplusplus
extern "C" {
#endif

//====================================================================================================
// Macros
//====================================================================================================
#ifndef IO_ADDRESS
#define IO_ADDRESS(x) (x)
#endif

//====================================================================================================
// Types define
//====================================================================================================
typedef struct
{
    VUint32 CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
    VUint32 PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
    VUint32 CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
    VUint32 CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
    VUint32 AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
    VUint32 AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
    VUint32 AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
    VUint32  RESERVED0;     /*!< Reserved, 0x1C                                                                    */
    VUint32 APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
    VUint32 APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
    VUint32      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
    VUint32 AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
    VUint32 AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
    VUint32 AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
    VUint32      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
    VUint32 APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
    VUint32 APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
    VUint32      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
    VUint32 AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    VUint32 AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
    VUint32 AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
    VUint32      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
    VUint32 APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    VUint32 APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
    VUint32      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
    VUint32 BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
    VUint32 CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
    VUint32      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
    VUint32 SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
    VUint32 PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_REGS;
#define pRCC   ((RCC_REGS *)0x40023800u)

typedef struct
{
    VUint32 ISER[8];                 /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register           */
    VUint32      RESERVED0[24];
    VUint32 ICER[8];                 /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register         */
    VUint32      RESERVED1[24];
    VUint32 ISPR[8];                 /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register          */
    VUint32      RESERVED2[24];
    VUint32 ICPR[8];                 /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register        */
    VUint32      RESERVED3[24];
    VUint32 IABR[8];                 /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register           */
    VUint32      RESERVED4[56];
    VUint8  IP[240];                 /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
    VUint32      RESERVED5[644];
    VUint32 STIR;                    /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register     */
} NVIC_REGS;
#define pNVIC  ((NVIC_REGS *)0xE000E100u)

typedef struct
{
    const Uint32 CPUID;                   /*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
    VUint32 ICSR;                    /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
    VUint32 VTOR;                    /*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */
    VUint32 AIRCR;                   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
    VUint32 SCR;                     /*!< Offset: 0x010 (R/W)  System Control Register                               */
    VUint32 CCR;                     /*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
    VUint8  SHP[12];                 /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
    VUint32 SHCSR;                   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
    VUint32 CFSR;                    /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register                    */
    VUint32 HFSR;                    /*!< Offset: 0x02C (R/W)  HardFault Status Register                             */
    VUint32 DFSR;                    /*!< Offset: 0x030 (R/W)  Debug Fault Status Register                           */
    VUint32 MMFAR;                   /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register                      */
    VUint32 BFAR;                    /*!< Offset: 0x038 (R/W)  BusFault Address Register                             */
    VUint32 AFSR;                    /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register                       */
    const VUint8 PFR[2];                  /*!< Offset: 0x040 (R/ )  Processor Feature Register                            */
    const VUint8 DFR;                     /*!< Offset: 0x048 (R/ )  Debug Feature Register                                */
    const VUint8 ADR;                     /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register                            */
    const VUint8 MMFR[4];                 /*!< Offset: 0x050 (R/ )  Memory Model Feature Register                         */
    const VUint8 ISAR[5];                 /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register                   */
    VUint32      RESERVED0[5];
    VUint32 CPACR;                   /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register                   */
} SCB_REGS;
#define pSCB  ((SCB_REGS *)0xE000ED00u)

typedef struct
{
    VUint32 MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
    VUint32 PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
    VUint32 EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
    VUint32      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
    VUint32 CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_REGS;
#define pSYSCFG  ((SYSCFG_REGS *)0x40013800u)

typedef struct
{
    VUint32 BCR1;// 0x0000
    VUint32 BTR1;// 0x0000
    VUint32 BCR2;// 0x0000
    VUint32 BTR2;// 0x0000
    VUint32 BCR3;// 0x0000
    VUint32 BTR3;// 0x0000
    VUint32 BCR4;// 0x0000
    VUint32 BTR4;// 0x0000
    VUint32      RESERVED0[56];// 0x20
    VUint32      RESERVED1;// 0x0100
    VUint32 BWTR1;// 0x0104
    VUint32      RESERVED2;// 0x0108
    VUint32 BWTR2;// 0x010C
    VUint32      RESERVED3;// 0x0110
    VUint32 BWTR3;// 0x0114
    VUint32      RESERVED4;// 0x0118
    VUint32 BWTR4;// 0x011C
} FSMC_REGS;
#define pFSMC  ((FSMC_REGS *)0xA0000000u)

typedef struct
{
    VUint16 CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
    VUint16      RESERVED0;   /*!< Reserved, 0x02                                            */
    VUint16 CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
    VUint16      RESERVED1;   /*!< Reserved, 0x06                                            */
    VUint16 SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
    VUint16      RESERVED2;   /*!< Reserved, 0x0A                                            */
    VUint16 DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    VUint16      RESERVED3;   /*!< Reserved, 0x0E                                            */
    VUint16 SR;          /*!< TIM status register,                 Address offset: 0x10 */
    VUint16      RESERVED4;   /*!< Reserved, 0x12                                            */
    VUint16 EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
    VUint16      RESERVED5;   /*!< Reserved, 0x16                                            */
    VUint16 CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
    VUint16      RESERVED6;   /*!< Reserved, 0x1A                                            */
    VUint16 CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
    VUint16      RESERVED7;   /*!< Reserved, 0x1E                                            */
    VUint16 CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
    VUint16      RESERVED8;   /*!< Reserved, 0x22                                            */
    VUint32 CNT;         /*!< TIM counter register,                Address offset: 0x24 */
    VUint16 PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
    VUint16      RESERVED9;   /*!< Reserved, 0x2A                                            */
    VUint32 ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
    VUint16 RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
    VUint16      RESERVED10;  /*!< Reserved, 0x32                                            */
    VUint32 CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
    VUint32 CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
    VUint32 CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
    VUint32 CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
    VUint16 BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
    VUint16      RESERVED11;  /*!< Reserved, 0x46                                            */
    VUint16 DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
    VUint16      RESERVED12;  /*!< Reserved, 0x4A                                            */
    VUint16 DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
    VUint16      RESERVED13;  /*!< Reserved, 0x4E                                            */
    VUint16 OR;          /*!< TIM option register,                 Address offset: 0x50 */
    VUint16      RESERVED14;  /*!< Reserved, 0x52                                            */
} TIMER_REGS;
#define pTIMER2    ((TIMER_REGS *)0x40000000u)
#define pTIMER14   ((TIMER_REGS *)0x40002000u)

typedef struct
{
    VUint16 SR;         /*!< USART Status register,                   Address offset: 0x00 */
    VUint16      RESERVED0;  /*!< Reserved, 0x02                                                */
    VUint16 DR;         /*!< USART Data register,                     Address offset: 0x04 */
    VUint16      RESERVED1;  /*!< Reserved, 0x06                                                */
    VUint16 BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
    VUint16      RESERVED2;  /*!< Reserved, 0x0A                                                */
    VUint16 CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
    VUint16      RESERVED3;  /*!< Reserved, 0x0E                                                */
    VUint16 CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
    VUint16      RESERVED4;  /*!< Reserved, 0x12                                                */
    VUint16 CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
    VUint16      RESERVED5;  /*!< Reserved, 0x16                                                */
    VUint16 GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
    VUint16      RESERVED6;  /*!< Reserved, 0x1A                                                */
} UART_REGS;
#define pUART1   ((UART_REGS *)0x40011000u)
#define pUART2   ((UART_REGS *)0x40004400u)
#define pUART3   ((UART_REGS *)0x40004800u)
#define pUART4   ((UART_REGS *)0x40004C00u)
#define pUART5   ((UART_REGS *)0x40005000u)
#define pUART6   ((UART_REGS *)0x40011400u)

typedef struct
{
    VUint16 CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
    VUint16      RESERVED0;  /*!< Reserved, 0x02                                                           */
    VUint16 CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
    VUint16      RESERVED1;  /*!< Reserved, 0x06                                                           */
    VUint16 SR;         /*!< SPI status register,                                Address offset: 0x08 */
    VUint16      RESERVED2;  /*!< Reserved, 0x0A                                                           */
    VUint16 DR;         /*!< SPI data register,                                  Address offset: 0x0C */
    VUint16      RESERVED3;  /*!< Reserved, 0x0E                                                           */
    VUint16 CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
    VUint16      RESERVED4;  /*!< Reserved, 0x12                                                           */
    VUint16 RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
    VUint16      RESERVED5;  /*!< Reserved, 0x16                                                           */
    VUint16 TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
    VUint16      RESERVED6;  /*!< Reserved, 0x1A                                                           */
    VUint16 I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
    VUint16      RESERVED7;  /*!< Reserved, 0x1E                                                           */
    VUint16 I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
    VUint16      RESERVED8;  /*!< Reserved, 0x22                                                           */
} SPI_REGS;
#define pSPI1   ((SPI_REGS *)0x40013000u)
#define pSPI2   ((SPI_REGS *)0x40003800u)
#define pSPI3   ((SPI_REGS *)0x40003C00u)

#define DMA_STREAM_POS         25u
#define DMA_STREAM_CHSEL_MASK  (Uint32)(0x7u << DMA_STREAM_POS)
typedef struct
{
    VUint32 CR;     /*!< DMA stream x configuration register      */
    VUint32 NDTR;   /*!< DMA stream x number of data register     */
    VUint32 PAR;    /*!< DMA stream x peripheral address register */
    VUint32 M0AR;   /*!< DMA stream x memory 0 address register   */
    VUint32 M1AR;   /*!< DMA stream x memory 1 address register   */
    VUint32 FCR;    /*!< DMA stream x FIFO control register       */
} DMA_STREAM_REGS;

typedef struct
{
    VUint32 LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
    VUint32 HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
    VUint32 LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
    VUint32 HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
    DMA_STREAM_REGS STREAM[8];
} DMA_REGS;
#define pDMA1   ((DMA_REGS *)0x40026000u)
#define pDMA2   ((DMA_REGS *)0x40026400u)

typedef struct
{
    VUint32 MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
    VUint32 OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
    VUint32 OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
    VUint32 PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    VUint32 IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
    VUint32 ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
    VUint16 BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
    VUint16 BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
    VUint32 LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
    VUint32 AFRL;     /*!< GPIO alternate function registers,     Address offset: 0x20 */
    VUint32 AFRH;     /*!< GPIO alternate function registers,     Address offset: 0x24 */
} GPIO_REGS;
#define pGPIOA ((GPIO_REGS *)0x40020000u)
#define pGPIOB ((GPIO_REGS *)0x40020400u)
#define pGPIOC ((GPIO_REGS *)0x40020800u)
#define pGPIOD ((GPIO_REGS *)0x40020C00u)
#define pGPIOE ((GPIO_REGS *)0x40021000u)
#define pGPIOF ((GPIO_REGS *)0x40021400u)
#define pGPIOG ((GPIO_REGS *)0x40021800u)
#define pGPIOH ((GPIO_REGS *)0x40021C00u)
#define pGPIOI ((GPIO_REGS *)0x40022000u)

typedef struct
{
    VUint32 IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
    VUint32 EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
    VUint32 RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
    VUint32 FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
    VUint32 SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
    VUint32 PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_REGS;
#define pEXTI ((EXTI_REGS *)0x40013C00)

typedef struct
{
    VUint32 KR;   /*!< IWDG Key register,       Address offset: 0x00 */
    VUint32 PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
    VUint32 RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
    VUint32 SR;   /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_REGS;
#define pIWDG   ((IWDG_REGS *)0x40003000u)

typedef struct
{
    VUint32 CR;   /*!< WWDG Control register,       Address offset: 0x00 */
    VUint32 CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
    VUint32 SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_REGS;
#define pWWDG   ((WWDG_REGS *)0x40002C00u)

#ifdef __cplusplus
}
#endif

#endif

