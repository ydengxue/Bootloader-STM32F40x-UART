/*****************************************************************************************************
 * Description:                 System Entrance 
 *
 * Author:                      Dengxue Yan, Washington University in St. Louis
 *
 * Email:                       Dengxue.Yan@wustl.edu
 *
 * Rev History:
 *       <Author>        <Date>        <Hardware>     <Version>        <Description>
 *     Dengxue Yan   03/15/2015 15:30       --           1.00             Create
 *****************************************************************************************************/
//====================================================================================================
// Declaration of header files 
//----------------------------------------------------------------------------------------------------
// Lib headers
#include <stdio.h>

// Self-defined headers
#include "STM32F40xRegister.h"
#include "stm32f4xx.h"
#include "UserTypesDef.h"
#include "SystemBase.h"
#include "FileTypesDef.h"
#include "Debug.h"
#include "stm32f4xx_flash.h"

//====================================================================================================
// Macros
//----------------------------------------------------------------------------------------------------
#define FLASH_Sector_0     ((uint16_t)0x0000) /*!< Sector Number 0 */
#define FLASH_Sector_1     ((uint16_t)0x0008) /*!< Sector Number 1 */
#define FLASH_Sector_2     ((uint16_t)0x0010) /*!< Sector Number 2 */
#define FLASH_Sector_3     ((uint16_t)0x0018) /*!< Sector Number 3 */
#define FLASH_Sector_4     ((uint16_t)0x0020) /*!< Sector Number 4 */
#define FLASH_Sector_5     ((uint16_t)0x0028) /*!< Sector Number 5 */
#define FLASH_Sector_6     ((uint16_t)0x0030) /*!< Sector Number 6 */
#define FLASH_Sector_7     ((uint16_t)0x0038) /*!< Sector Number 7 */
#define FLASH_Sector_8     ((uint16_t)0x0040) /*!< Sector Number 8 */
#define FLASH_Sector_9     ((uint16_t)0x0048) /*!< Sector Number 9 */
#define FLASH_Sector_10    ((uint16_t)0x0050) /*!< Sector Number 10 */
#define FLASH_Sector_11    ((uint16_t)0x0058) /*!< Sector Number 11 */

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_00     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_01     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_02     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_03     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_04     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_05     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_06     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_07     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_08     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_09     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10     ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11     ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */
#define ADDR_FLASH_SECTOR_END    ((uint32_t)0x08100000) /* Base @ of Sector 11, 128 Kbytes */

//====================================================================================================
// Self-defined TPYEs
//----------------------------------------------------------------------------------------------------
typedef struct 
{
    Uint16 sector_no;
    int32 sector_length;
} FLASH_INTERNAL_SECTOR_INFO;

//====================================================================================================
// Declaration of local functions, these functions can not be accessed from outside
//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Declaration of external functions
//----------------------------------------------------------------------------------------------------
extern int32 HardwareInitial(void);
extern int rt_application_init(void);
extern int rt_application_start(unsigned int, unsigned int);

//====================================================================================================
// Definition of global variables
//----------------------------------------------------------------------------------------------------

//====================================================================================================
// Definition of local variables, these variables can not be accessed from outside
//----------------------------------------------------------------------------------------------------
static Uint32 timer_tick_count;
static int32 FlashInternalDownload(void);
static int32 FlashInternalErase(Uint32 addr);
static int32 FlashInternalWrite(Uint32 addr, Uint8 const *src, int32 length);
static Uint16 DefaultCrcCal(Uint8 const *src, int32 length, Uint16 crc_initial);

static VUint16 debug_rx_buffer_rd_ptr;
static VUint16 debug_rx_buffer_wr_ptr;
static Uint8  debug_rx_buffer[UART_RX_BUF_SIZE];
static Vint8  enter_debug_mode;
static VUint32 system_ms_count;

static const FLASH_INTERNAL_SECTOR_INFO flash_internal_sector_info[12] =
{
    {0x0000u, 0x00004000u},
    {0x0008u, 0x00004000u},
    {0x0010u, 0x00004000u},
    {0x0018u, 0x00004000u},
    {0x0020u, 0x00010000u},
    {0x0028u, 0x00020000u},
    {0x0030u, 0x00020000u},
    {0x0038u, 0x00020000u},
    {0x0040u, 0x00020000u},
    {0x0048u, 0x00020000u},
    {0x0050u, 0x00020000u},
    {0x0058u, 0x00020000u}
};

//====================================================================================================
// Functions implementation
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
// Interface functions
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//   Function: main
//      Input: 
//     Output:
//     Return: int: The status of function
//Description: System entrance
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
int main(void)
{
    Uint32 i;
    int32  lv_wait_count;
    Uint32 lv_system_ms_count_last;
    Uint32 lv_file_length;
    Uint32 lv_temp;
    Uint32 lv_kernel_addr;
    Uint8  lv_file_type;
    Uint8 const *lv_p_app_addr;
    Uint8 const *lv_p_extra;
    Uint8 const *lv_p_file_info;
    Uint16 lv_crc;
    Uint16 lv_version_int;
    Uint16 lv_version_frac;
    Uint16 lv_year;

    if (NORMAL_SUCCESS != HardwareInitial())
    {
        DebugPrintf("System hardware initial failed!");
        return NORMAL_ERROR;
    }

    DebugPrintf("Hardware ready!\r\n");
    DebugPrintf("#---------------------------------------------------------------------------\r\n");
    DebugPrintf("# Boot Compile Time : %s %s\r\n", __DATE__, __TIME__);
    DebugPrintf("# Boot Author       : Dengxue Yan\r\n");
    DebugPrintf("#---------------------------------------------------------------------------\r\n");
    
    enter_debug_mode = 0x00u;
    lv_p_app_addr = (Uint8 const *)FLASH_CPU_APP_START_ADDR;
    lv_p_file_info = (Uint8 *)CPU_APP_FILE_INFO_ADDR;
    lv_file_type = lv_p_file_info[3];
    lv_file_length = (*(Uint32 *)(lv_p_file_info)) & 0x00FFFFFFu;
    if (FILE_TYPE_CPU != lv_file_type)
    {
        DebugPrintf("Can not support application file type %d!\r\n", lv_file_type);
    }
    else
    {
        if ((lv_file_length > FLASH_CPU_APP_MAX_LENGTH) || (lv_file_length < FLASH_CPU_APP_MIN_LENGTH))
        {
            DebugPrintf("Application file length=%d is error! It must be in scope [%d, %d].\r\n", lv_file_length, FLASH_CPU_APP_MIN_LENGTH, FLASH_CPU_APP_MAX_LENGTH);
        }
        else
        {
            if (0u != DefaultCrcCal(lv_p_app_addr, lv_file_length + FILE_EXTRA_INFO_LENGTH, 0x5A5Au))
            {
                DebugPrintf("Application file CRC check failed!\r\n");
            }
            else
            {
                DebugPrintf("\r\n");
                lv_wait_count = 3;
                DebugPrintf("Application will run after %d second. Press 'Z' to enter boot mode\r\n", lv_wait_count);
                
                lv_system_ms_count_last = system_ms_count;
                while (0 != lv_wait_count)
                {
                    while (debug_rx_buffer_rd_ptr != debug_rx_buffer_wr_ptr)
                    {
                        if ('Z' == debug_rx_buffer[UART_RX_BUF_ADDR(debug_rx_buffer_rd_ptr++)])
                        {
                            enter_debug_mode = 0xFFu;
                            break;
                        }
                    }

                    if (0x00u != enter_debug_mode)
                    {
                        break;
                    }
               
                    if ((Uint32)(system_ms_count - lv_system_ms_count_last) >= 1000u)
                    {
                        lv_wait_count--;
                        DebugPrintf("%c[A", 0x1B);
                        DebugPrintf("Application will run after %d second. Press 'Z' to enter boot mode\r\n", lv_wait_count);
                        lv_system_ms_count_last += 1000;
                    }
                }
                
                if (0x00u == enter_debug_mode)
                {
                    DebugPrintf("\r\n");
                    DebugPrintf("Load application!\r\n\r\n");
                    lv_p_extra = lv_p_app_addr + lv_file_length;
                    lv_crc = (lv_p_extra[13] << 8) | lv_p_extra[12];
                    lv_version_int = (lv_p_extra[1] << 8) | lv_p_extra[0];
                    lv_version_frac = (lv_p_extra[3] << 8) | lv_p_extra[2];
                    lv_year = (lv_p_extra[5] << 8) | lv_p_extra[4];
                    DebugPrintf("#---------------------------------------------------------------------------\r\n");
                    DebugPrintf("# Application Compile Time : %04u-%02u-%02u %02u:%02u:%02u\r\n", lv_year, lv_p_extra[6], lv_p_extra[7], lv_p_extra[8], lv_p_extra[9], lv_p_extra[10]);
                    DebugPrintf("# Application File Length  : %d bytes\r\n", lv_file_length);
                    DebugPrintf("# Application Version      : %u.%04u\r\n", lv_version_int, lv_version_frac);
                    DebugPrintf("# Application CRC          : %04X\r\n", lv_crc);
                    DebugPrintf("#---------------------------------------------------------------------------\r\n");
                        
                    lv_temp = *(Uint32 *)(lv_p_app_addr);
                    lv_temp = (lv_temp - 4) & (~0x3);
                    lv_kernel_addr = *(Uint32 *)(lv_p_app_addr + 4);

                    pTIMER14->DIER = 0;
                    pTIMER14->CR1 = 0;

                    pNVIC->ICER[1] |= 0x00002000u;
                    pNVIC->ICPR[1] |= 0x00002000u;

                    __disable_irq();
                    rt_application_start(lv_kernel_addr, lv_temp);
                    enter_debug_mode = 0xFFu;
                }
            }
        }
    }

    DebugPrintf("\r\n");
    enter_debug_mode = 0xFFu;
    DebugPrintf("Enter Boot download mode!\r\n\r\n");
 
    debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;

    i = 1;
    while (0 != i)
    {
        FlashInternalDownload();
    }

    return NORMAL_SUCCESS;
}

//----------------------------------------------------------------------------------------------------
//   Function: TIM8_TRG_COM_TIM14_IRQHandler
//      Input: void
//     Output: void
//     Return: void
//Description: Timer interrupt
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    timer_tick_count++;
    if (timer_tick_count >= 50)
    {
        timer_tick_count = 0;
        system_ms_count++;
    }

    pTIMER14->SR &= (~0x0003u);
    if (0 != ((pUART1->SR) & 0x0020u))
    {
        debug_rx_buffer[UART_RX_BUF_ADDR(debug_rx_buffer_wr_ptr++)] = pUART1->DR;
        pUART1->SR |= 0x0020u;
    }
}

//----------------------------------------------------------------------------------------------------
// Local Functions
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//   Function: XModemFrameRecieve
//      Input: Uint8 frame_no, Uin8 *buffer, Uint8 buffer_length
//     Output: void
//     Return: int32 : Function Status
//Description: XModem Frame Recieve Function
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
static int32 XModemFrameRecieve(Uint8 frame_no, Uint8 *buffer, Uint8 buffer_length, Uint8 no_ack_flag)
{
    int32  lv_error_retry_times;
    int32  lv_overtime_retry_times;
    int32  lv_index;
    int32  lv_read_length;
    Uint32 lv_debug_rx_buffer_wr_ptr_temp;
    Uint32 lv_debug_rx_system_ms_count_last_value;
    Uint8  lv_check_sum;
    Uint32 lv_data_addr;

    if (NULL == buffer)
    {
        DebugPrintf("XModem receive buffer can not be NULL!\r\n");
        return NORMAL_ERROR;        
    }

    lv_error_retry_times = 10;
    while (0 != lv_error_retry_times)
    {
        lv_overtime_retry_times = 10;
        while (0 != lv_overtime_retry_times)
        {
            lv_debug_rx_buffer_wr_ptr_temp = debug_rx_buffer_wr_ptr;
            lv_debug_rx_system_ms_count_last_value = system_ms_count;
            while (UART_RX_BUF_ADDR(debug_rx_buffer_wr_ptr - debug_rx_buffer_rd_ptr) < 132u)
            {
                if (lv_debug_rx_buffer_wr_ptr_temp != debug_rx_buffer_wr_ptr)
                {
                    lv_debug_rx_buffer_wr_ptr_temp = debug_rx_buffer_wr_ptr;
                    lv_debug_rx_system_ms_count_last_value = system_ms_count;
                }
                else
                {
                    if ((system_ms_count - lv_debug_rx_system_ms_count_last_value) >= 1000u)
                    {
                        debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;
                        DebugSendByte(NAK);
                        lv_overtime_retry_times--;
                        break;
                   }
                }
            }

            if (UART_RX_BUF_ADDR(debug_rx_buffer_wr_ptr - debug_rx_buffer_rd_ptr) >= 132u)
            {
                break;
            }
        }

        if (0 == lv_overtime_retry_times)
        {
            debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;
            DebugPrintf("XModem receives data frame overtime!\r\n");
            return NORMAL_ERROR;
        }
        
        lv_check_sum = 0;
        lv_data_addr = debug_rx_buffer_rd_ptr + 3;
        for (lv_index = 0; lv_index < 128u; lv_index++)
        {
            lv_check_sum += debug_rx_buffer[UART_RX_BUF_ADDR(lv_data_addr + lv_index)];
        }
        
        if (    (SOH == debug_rx_buffer[UART_RX_BUF_ADDR(debug_rx_buffer_rd_ptr)])
             && (frame_no == debug_rx_buffer[UART_RX_BUF_ADDR(debug_rx_buffer_rd_ptr + 1)])
             && ((Uint8)(~frame_no) == debug_rx_buffer[UART_RX_BUF_ADDR(debug_rx_buffer_rd_ptr + 2)])
             && (lv_check_sum == debug_rx_buffer[UART_RX_BUF_ADDR(lv_data_addr + 128u)]))
        {
            break;
        }
        else
        {
            debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;
            DebugSendByte(NAK);
            lv_error_retry_times--;
            continue;
        }
    }
    
    if (0 == lv_error_retry_times)
    {
        debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;
        DebugPrintf("XModem receives error data frame!\r\n");
        return NORMAL_ERROR;
    }
    else
    {
        if (buffer_length >= 128u)
        {
            lv_read_length = 128u;
        }
        else
        {
            lv_read_length = buffer_length; 
        }
        for (lv_index = 0; lv_index < lv_read_length; lv_index++)
        {
            buffer[lv_index] = debug_rx_buffer[UART_RX_BUF_ADDR(lv_data_addr + lv_index)];
        }
        
        debug_rx_buffer_rd_ptr += 132u;
        if (0 == no_ack_flag)
        {
            DebugSendByte(ACK);
        }

        return lv_read_length;
    }
}

//----------------------------------------------------------------------------------------------------
//   Function: XModemEndConfirm
//      Input: void
//     Output: void
//     Return: int32 : Function Status
//Description: XModem end confirm fram handle
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
static int32 XModemEndConfirm(void)
{
    int32 lv_error_retry_times;
    int32 lv_overtime_retry_times;
    Uint32 lv_debug_rx_system_ms_count_last_value;
    
    lv_error_retry_times = 10;
    while (0 != lv_error_retry_times)
    {
        lv_overtime_retry_times = 10;
        while (0 != lv_overtime_retry_times)
        {
            lv_debug_rx_system_ms_count_last_value = system_ms_count;
            while (0 == UART_RX_BUF_ADDR(debug_rx_buffer_wr_ptr - debug_rx_buffer_rd_ptr))
            {
                if ((system_ms_count - lv_debug_rx_system_ms_count_last_value) >= 1000u)
                {
                    DebugSendByte(NAK);
                    debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;
                    lv_overtime_retry_times--;
                    break;
                }
            }
                
            if (0 != UART_RX_BUF_ADDR(debug_rx_buffer_wr_ptr - debug_rx_buffer_rd_ptr))
            {
                break;
            }
        }

        if (0 == lv_overtime_retry_times)
        {
            debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;
            DebugPrintf("XModem receives end confirm frame overtime\r\n");
            return NORMAL_ERROR;
        }

        if (EOT == debug_rx_buffer[UART_RX_BUF_ADDR(debug_rx_buffer_rd_ptr)])
        {
            break;
        }
        else
        {
            debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;
            DebugSendByte(NAK);
            lv_error_retry_times--;
            continue;
        }
    }

    if (0 == lv_error_retry_times)
    {
        DebugPrintf("XModem receives error end confirm frame\r\n");
        return NORMAL_ERROR;
    }
    else
    {
        debug_rx_buffer_rd_ptr++;
        DebugSendByte(ACK);
        return NORMAL_SUCCESS;
    }
}

//----------------------------------------------------------------------------------------------------
//   Function: NandDownload
//      Input: char const *args
//     Output: void
//     Return: int32 : Function status
//Description: Nandflash download
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
static int32 FlashInternalDownload(void)
{
    Uint8   lv_xmodem_frame_no;
    int32   lv_flash_start_addr;
    int32   lv_flash_end_addr;
    Uint32  lv_debug_rx_system_ms_count_last_value;
    Uint8   lv_buffer[128u];
    int32   lv_flash_write_length;
    int32   lv_flash_erase_length;
    Uint16  lv_file_type;
    Uint32  lv_file_length;
    int32   lv_file_min_length;
    int32   lv_file_max_length;
    Uint32  lv_flash_erase_addr;
    Uint32  lv_flash_write_addr;
    Uint32  lv_current_frame_valid_data_length;

    while (0 == UART_RX_BUF_ADDR(debug_rx_buffer_wr_ptr - debug_rx_buffer_rd_ptr))
    {
        DebugPrintf("Enter internal flash download mode!\r\n");

        DebugSendByte(NAK);
        lv_debug_rx_system_ms_count_last_value = system_ms_count;
        while (0 == UART_RX_BUF_ADDR(debug_rx_buffer_wr_ptr - debug_rx_buffer_rd_ptr))
        {
            if ((system_ms_count - lv_debug_rx_system_ms_count_last_value) >= 1000u)
            {
                lv_debug_rx_system_ms_count_last_value += 1000u;
                DebugSendByte(NAK);
            }
        }

        lv_xmodem_frame_no = 1;
        if (SOH != debug_rx_buffer[UART_RX_BUF_ADDR(debug_rx_buffer_rd_ptr)])
        {
            DebugPrintf("The first download data debug uart received is not XMODEM SOH \"0x%02X\", but \"0x%02X\", please check!\r\n", (Uint8)SOH, (Uint8)debug_rx_buffer[UART_RX_BUF_ADDR(debug_rx_buffer_rd_ptr)]);
            debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;
            return NORMAL_SUCCESS;
        }
        
        if (FILE_INFO_LENGTH != XModemFrameRecieve(lv_xmodem_frame_no, lv_buffer, FILE_INFO_LENGTH, 1))
        {
            DebugPrintf("Uart receive first download data failed!\r\n");
            return NORMAL_ERROR;
        }

        lv_file_type = lv_buffer[3];
        lv_file_length = (lv_buffer[0]) | (lv_buffer[1] << 8) | (lv_buffer[2] << 16);
        lv_flash_start_addr = 0;
        switch (lv_file_type)
        {
            case FILE_TYPE_CPU:
            {
                lv_flash_start_addr = FLASH_CPU_APP_START_ADDR;
                lv_file_min_length  = FLASH_CPU_APP_MIN_LENGTH;
                lv_file_max_length  = FLASH_CPU_APP_MAX_LENGTH;
                break;
            }
            default:
            {
                DebugPrintf("Unknow file type %u!\r\n", lv_file_type);
                debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;
                return NORMAL_ERROR;
            }
        }

        if ((lv_file_length > lv_file_max_length) || (lv_file_length < lv_file_min_length))
        {
            DebugPrintf("File type %u's length is %d, exceed the scope [0, %d]!\r\n", lv_file_type, lv_file_length, FLASH_CPU_APP_MAX_LENGTH);
            return NORMAL_ERROR;
        }

        if (FILE_TYPE_CPU == lv_file_type)
        {
            lv_flash_erase_length = FlashInternalErase(CPU_APP_FILE_INFO_ADDR);
            if (lv_flash_erase_length < 0)
            {
                DebugPrintf("Internal flash erase at addr:%08X failed!, return value is %08X\r\n", CPU_APP_FILE_INFO_ADDR, lv_flash_erase_length);
                return NORMAL_ERROR;
            }
        }

        lv_flash_end_addr = lv_flash_start_addr + lv_file_length + FILE_EXTRA_INFO_LENGTH;
        lv_flash_erase_addr = lv_flash_start_addr;
        while (lv_flash_erase_addr < lv_flash_end_addr)
        {
            lv_flash_erase_length = FlashInternalErase(lv_flash_erase_addr);
            if (lv_flash_erase_length < 0)
            {
                DebugPrintf("Internal flash erase at addr:%08X failed!, return value is %08X\r\n", lv_flash_erase_addr, lv_flash_erase_length);
                return NORMAL_ERROR;
            }
            lv_flash_erase_addr += lv_flash_erase_length;
        }

        if (FILE_TYPE_CPU == lv_file_type)
        {
            lv_flash_write_length = FlashInternalWrite(CPU_APP_FILE_INFO_ADDR, lv_buffer, FILE_INFO_LENGTH);
            if (lv_flash_write_length != FILE_INFO_LENGTH)
            {
                DebugPrintf("flash.in.write at addr:%08X failed!, return value is %08X\r\n", CPU_APP_FILE_INFO_ADDR, lv_flash_write_length);
                return NORMAL_ERROR;
            }
            lv_flash_write_addr = lv_flash_start_addr;
        }
        else
        {
            lv_flash_write_addr = lv_flash_start_addr;
            lv_flash_write_length = FlashInternalWrite(lv_flash_write_addr, lv_buffer, FILE_INFO_LENGTH);
            if (lv_flash_write_length != FILE_INFO_LENGTH)
            {
                DebugPrintf("flash.in.write at addr:%08X failed!, return value is %08X\r\n", lv_flash_write_addr, lv_flash_write_length);
                return NORMAL_ERROR;
            }
            lv_flash_write_addr += lv_flash_write_length;
        }

        // frame 1 must be confirmed here, because the flash erase time is too long, uart may lost datas if confirm too early!
        DebugSendByte(ACK);
        
        while (lv_flash_write_addr < lv_flash_end_addr)
        {
            lv_xmodem_frame_no++;
            if ((lv_flash_end_addr - lv_flash_write_addr) < 128u)
            {
                lv_current_frame_valid_data_length = lv_flash_end_addr - lv_flash_write_addr;
            }
            else
            {
                lv_current_frame_valid_data_length = 128u;
            }

            if (lv_current_frame_valid_data_length != XModemFrameRecieve(lv_xmodem_frame_no, lv_buffer, lv_current_frame_valid_data_length, 0))
            {
                DebugPrintf("Uart receive download data frame no %d failed!\r\n", lv_xmodem_frame_no);
                return NORMAL_ERROR;
            }

            lv_flash_write_length = FlashInternalWrite(lv_flash_write_addr, lv_buffer, lv_current_frame_valid_data_length);
            if (lv_flash_write_length != lv_current_frame_valid_data_length)
            {
                DebugPrintf("flash.in.write at addr:%08X failed!, return value is %08X\r\n", lv_flash_write_addr, lv_flash_write_length);
                return NORMAL_ERROR;
            }
            lv_flash_write_addr += lv_flash_write_length;
        }

        if (NORMAL_SUCCESS != XModemEndConfirm())
        {
            DebugPrintf("XModem receive end confirm frame failed\r\n");
            return NORMAL_ERROR;
        }

        // Waiting for 100ms
        lv_debug_rx_system_ms_count_last_value = system_ms_count;
        while ((system_ms_count - lv_debug_rx_system_ms_count_last_value) < 50u);

        if (0 != DefaultCrcCal((Uint8 *)lv_flash_start_addr, lv_file_length + FILE_EXTRA_INFO_LENGTH, 0x5A5Au))
        {
            DebugPrintf("File type %u crc check failed! Please not power off and download again\r\n", lv_file_type);
            debug_rx_buffer_rd_ptr = debug_rx_buffer_wr_ptr;
            return NORMAL_ERROR;
        }
        DebugPrintf("File type 0x%04X download success, please reboot the device!\r\n", lv_file_type);
    }
    return NORMAL_SUCCESS;
}



//----------------------------------------------------------------------------------------------------
//   Function: FlashInternalGetSectorInfo
//      Input: Uint32 addr
//     Output: void
//     Return: int32 : FLASH_INTERNAL_SECTOR_INFO const *
//Description: Return flash sector info according to the input addr
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
static FLASH_INTERNAL_SECTOR_INFO const *FlashInternalGetSectorInfo(Uint32 addr)
{ 
    if ((addr < ADDR_FLASH_SECTOR_00) || (addr >= ADDR_FLASH_SECTOR_END))
    {
        return NULL;
    }
    else if (addr < ADDR_FLASH_SECTOR_04)
    {
        return &flash_internal_sector_info[((addr >> 14) & 0x3)];
    }
    else if (addr < ADDR_FLASH_SECTOR_05)
    {
        return &flash_internal_sector_info[4];  
    }
    else
    {
        return &flash_internal_sector_info[4 + ((addr >> 17) & 0xF)];  
    }
}

//----------------------------------------------------------------------------------------------------
//   Function: FlashInternalWrite
//      Input: Uint32 addr
//             Uint8 const *src
//             int32 length
//     Output: void
//     Return: int32 : Function status
//Description: Nand flash write
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
static int32 FlashInternalWrite(Uint32 addr, Uint8 const *src, int32 length)
{
    Uint32 lv_addr_end;
    Uint32 lv_addr;
    Uint8 const *lv_p_src;
    int32  lv_addr_word_end;
    int32  lv_addr_halfword_end;
    
    if (NULL == src)
    {
        return -2;
    }
        
    lv_p_src = src;
    lv_addr = addr; 
    lv_addr_end = lv_addr + length;
    if ((lv_addr < ADDR_FLASH_SECTOR_00) || (lv_addr_end >= ADDR_FLASH_SECTOR_END))
    {
        return -3;
    }

    if (0 == length)
    {
        return 0;
    }

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
    
    if (0 == (lv_addr & 3))
    {
        lv_addr_word_end = (lv_addr_end & (~3));
        while (lv_addr < lv_addr_word_end)
        {
            if (FLASH_COMPLETE != FLASH_ProgramWord(lv_addr, *(Uint32 *)lv_p_src))
            {
                FLASH_Lock();
                DebugPrintf("Internal flash write word at addr %08X failed!\r\n", lv_addr);
                return NORMAL_ERROR;
            }
            lv_addr += 4;
            lv_p_src += 4;
        }
    }

    if (0 == (lv_addr & 1))
    {
        lv_addr_halfword_end = (lv_addr_end & (~1));
        while (lv_addr < lv_addr_halfword_end)
        {
            if (FLASH_COMPLETE != FLASH_ProgramHalfWord(lv_addr, *(Uint16 *)lv_p_src))
            {
                FLASH_Lock();
                DebugPrintf("Internal flash write half word at addr %08X failed!\r\n", lv_addr);
                return NORMAL_ERROR;
            }
            lv_addr += 2;
            lv_p_src += 2;
        }
   
    }

    while (lv_addr < lv_addr_end)
    {
        if (FLASH_COMPLETE != FLASH_ProgramByte(lv_addr, *lv_p_src))
        {
            FLASH_Lock();
            DebugPrintf("Internal flash write byte at addr %08X failed!\r\n", lv_addr);
            return NORMAL_ERROR;
        }
        lv_addr += 1;
        lv_p_src += 1;
    }

    FLASH_Lock();

    return length;

}

//----------------------------------------------------------------------------------------------------
//   Function: FlashInternalErase
//      Input: Uint32 addr:
//     Output: void
//     Return: int32: Function status
//Description: Erase a sector according to the input addr
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
static int32 FlashInternalErase(Uint32 addr)
{
    FLASH_INTERNAL_SECTOR_INFO const *lv_p_flash_sector_info;
    
    lv_p_flash_sector_info = FlashInternalGetSectorInfo(addr);
    if (NULL == lv_p_flash_sector_info)
    {
        DebugPrintf("Get flash sector number failed!\r\n");
        return NORMAL_ERROR;
    }

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
              FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 

    if (FLASH_COMPLETE != FLASH_EraseSector(lv_p_flash_sector_info->sector_no, VoltageRange_3))
    {
        FLASH_Lock();
        DebugPrintf("Internal flash sector=%d erase failed!\r\n", lv_p_flash_sector_info->sector_no);
        return NORMAL_ERROR;
    }

    FLASH_Lock();
    
    return lv_p_flash_sector_info->sector_length;   
}

//----------------------------------------------------------------------------------------------------
//   Function: DefaultCrcCal
//      Input: Uint8 const *ptr:
//             Uint16 length:
//             Uint16 crc_initial:
//     Output: void
//     Return: Uint16:
//Description: Default CRC calculation
//    <AUTHOR>        <MODIFYTIME>            <REASON>
//   Dengxue Yan     03/21/2011 16:30          Create
//----------------------------------------------------------------------------------------------------
static Uint16 const const_default_crc_tab[256] =
{
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

static Uint16 DefaultCrcCal(Uint8 const *src, int32 length, Uint16 crc_initial)
{
    int32  lv_index;
    Uint16 lv_crc;
    Uint8  lv_crc_index;

    lv_crc = crc_initial;
    for (lv_index = 0; lv_index < length; lv_index++)
    {
        lv_crc_index = (lv_crc & 0xff) ^ src[lv_index];
        lv_crc = const_default_crc_tab[lv_crc_index] ^ ((lv_crc >> 8) & 0xff);
    }

    return lv_crc;
}

