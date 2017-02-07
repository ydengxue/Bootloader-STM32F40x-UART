/*****************************************************************************************************
 * Description:                 Debug functions declaration
 *
 * Author:                      Dengxue Yan, Washington University in St. Louis
 *
 * Email:                       Dengxue.Yan@wustl.edu
 *
 * Rev History:
 *       <Author>        <Date>        <Hardware>     <Version>        <Description>
 *     Dengxue Yan   09/15/2016 15:30       --           1.00             Create
 *****************************************************************************************************/
#ifndef _Debug_H
#define _Debug_H

#ifdef __cplusplus
extern "C" {
#endif

//====================================================================================================
// Outside headers reference
//====================================================================================================
#include "UserTypesDef.h"
#include "stdio.h"

//====================================================================================================
// Types define
//====================================================================================================

//====================================================================================================
// Macros
//====================================================================================================
#define pDEBUG pUART1

#define XMODEM_SOH 0x01u
#define XMODEM_EOT 0x04u
#define XMODEM_ACK 0x06u
#define XMODEM_NAK 0x15u
#define XMODEM_CAN 0x18u

#define DebugPrintf printf
#define TRACE DebugPrintf

#define DEBUG_UART_DESIRED_BAUD   (115200u)
#define DEBUG_UART_OSM_SEL   0
#if (0 == DEBUG_UART_OSM_SEL)
#define DEBUG_UART_OVERSAMPLE_CNT (16u)
#elif (1 == DEBUG_UART_OSM_SEL)
#define DEBUG_UART_OVERSAMPLE_CNT (13u)
#else
#error "DEBUG_UART_OSM_SEL difine error!"
#endif


//====================================================================================================
// Debug buffers
//====================================================================================================
#define FRAME_BUFFER_SIZE       512u
#define FRAME_BUFFER_MASK         (FRAME_BUFFER_SIZE - 1u)
#define FRAME_BUFFER_ADDR(addr)   ((Uint16)((addr) & (FRAME_BUFFER_MASK)))

#define UART_RX_BUF_SIZE         512u
#define UART_RX_BUF_MASK         (UART_RX_BUF_SIZE - 1u)
#define UART_RX_BUF_ADDR(addr)   ((Uint16)((addr) & (UART_RX_BUF_MASK)))

#define UART_TX_BUF_SIZE         512u
#define UART_TX_BUF_MASK         (UART_TX_BUF_SIZE - 1u)
#define UART_TX_BUF_ADDR(addr)   ((Uint16)((addr) & (UART_TX_BUF_MASK)))


//====================================================================================================
// External functions declaration
//====================================================================================================
extern int32 rt_send_byte(int8 byte);
extern int32 DebugSendByte(int8 byte);

#ifdef __cplusplus
}
#endif

#endif

