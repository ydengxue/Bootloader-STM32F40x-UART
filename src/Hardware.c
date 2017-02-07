/*****************************************************************************************************
 * Description:                 Hardware related functions
 *
 * Author:                      Dengxue Yan, Washington University in St. Louis
 *
 * Email:                       Dengxue.Yan@wustl.edu
 *
 * Rev History:
 *       <Author>        <Date>        <Hardware>     <Version>        <Description>
 *     Dengxue Yan   09/15/2016 15:30       --           1.00             Create
 *****************************************************************************************************/
//====================================================================================================
// Declaration of header files 
//----------------------------------------------------------------------------------------------------
// Lib headers
#include <string.h>
#include <stdlib.h>

// Self-defined headers
#include "STM32F40xRegister.h"
#include "Debug.h"
#include "stm32f4xx.h"

//====================================================================================================
// Macros
//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Self-defined TPYEs
//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Declaration of local functions, these functions can not be accessed from outside
//----------------------------------------------------------------------------------------------------
static int32 DebugUARTInitial(void);
static int32 TimerInitial(void);

//====================================================================================================
// Declaration of external functions
//----------------------------------------------------------------------------------------------------
//extern void rt_hw_interrupt_enable(int32);

//====================================================================================================
// Declaration of external variables
//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Definition of global variables
//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Definition of local variables, these variables can not be accessed from outside
//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Functions implementation
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
// Interface functions
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//   Function: HardwareInitial
//      Input: void 
//     Output: void
//     Return: int32: function status
//Description: Hardware initial function
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
int32 HardwareInitial(void)
{
    Uint32 lv_ulong;
    if (    (NORMAL_SUCCESS != DebugUARTInitial())
         || (NORMAL_SUCCESS != TimerInitial())
       )
    {
        DebugPrintf("Hardware initial failed!\r\n");
        return NORMAL_ERROR;
    }

    pRCC->AHB1ENR |= 0x00000180u;

    lv_ulong = pGPIOH->MODER;
    lv_ulong &= 0x03FFFC3Fu;
    lv_ulong |= 0x54000140u;
    pGPIOH->MODER = lv_ulong;

    lv_ulong = pGPIOH->OTYPER;
    lv_ulong &= 0xFFFF1FE7u;
    lv_ulong |= 0x00000000u;
    pGPIOH->OTYPER = lv_ulong;

    pGPIOH->BSRRL = 0xE008u;
    pGPIOH->BSRRL = 0x0010u;

    // enable global interrupt after hardware initial
    __enable_irq();

    return NORMAL_SUCCESS;
}


//----------------------------------------------------------------------------------------------------
// Local Functions
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//   Function: DebugUARInitial
//      Input: void
//     Output: void
//     Return: int32 : Function Status
//Description: Debug uart port initial
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
static int8 const uart_initial_string[] = "    \r\n\r\n";
static int32 DebugUARTInitial(void)
{
    Uint32 lv_ulong;
    Uint32 i;

    pRCC->AHB1ENR |= 0x00000001u;
    pRCC->APB2ENR |= 0x00000010u;

    lv_ulong = pGPIOA->MODER;
    lv_ulong &= 0xFFC3FFFFu;
    lv_ulong |= 0x00280000u;
    pGPIOA->MODER = lv_ulong;

    lv_ulong = pGPIOA->OTYPER;
    lv_ulong &= 0xFFFFF9FFu;
    lv_ulong |= 0x00000000u;
    pGPIOA->OTYPER = lv_ulong;

    lv_ulong = pGPIOA->OSPEEDR;
    lv_ulong &= 0xFFC3FFFFu;
    lv_ulong |= 0x00000000u;
    pGPIOA->OSPEEDR = lv_ulong;

    lv_ulong = pGPIOA->AFRH;
    lv_ulong &= 0xFFFFF00Fu;
    lv_ulong |= 0x00000770u;
    pGPIOA->AFRH = lv_ulong;

    pDEBUG->BRR = 0x02D9u;// 115200 bps
    pDEBUG->SR  = 0;
    pDEBUG->CR1 = 0x200Cu;

    pDEBUG->DR = '\r';
    for (i = 0; i < sizeof(uart_initial_string); i++)
    {
        DebugSendByte(uart_initial_string[i]);
    }

    return NORMAL_SUCCESS;
}

//----------------------------------------------------------------------------------------------------
//   Function: TimerInitial
//      Input: void
//     Output: void
//     Return: int32: Function Status
//Description: Timer initial
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
static int32 TimerInitial(void)
{
    // Timer14
    pRCC->APB1ENR |= 0x00000100u;

    pTIMER14->PSC = 83;
    pTIMER14->ARR = 19;
    pTIMER14->CCR1 = 0xFFFFu;
    pTIMER14->SR &= (~0x0003u);
    pTIMER14->DIER|= 0x0001u;// interrupt enable
    pTIMER14->CR1 = 0x0005u;

    pNVIC->IP[45]  = 0x40u;
    pNVIC->ISER[1] |= 0x00002000u;

    return NORMAL_SUCCESS;
}

