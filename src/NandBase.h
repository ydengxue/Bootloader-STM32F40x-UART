/*****************************************************************************************************
* FileName     :    NandBase.h
*
* Reference    :    NULL
*
* Description  :    Nand操作相关头文件
*
* History      :
*       <Author>        <Data>        <Hardware>     <Version>        <Description>
*     YanDengxue   2011-03-29 15:30       --           1.00             Create
*****************************************************************************************************/
#ifndef _Nand_Base_H_
#define _Nand_Base_H_

#ifdef __cplusplus
extern "C" {
#endif

//====================================================================================================
// 宏定义
//====================================================================================================
//----------------------------------------------------------------------------------------------------
// NAND地址
//----------------------------------------------------------------------------------------------------
#define NAND_CMD_ADDR    0x62000010u
#define pNAND_CMD_ADDR   ((VUint8 *)0x62000010u)
#define NAND_ADDR_ADDR   0x62000008u 
#define pNAND_ADDR_ADDR  ((VUint8 *)0x62000008u)
#define NAND_DATA_ADDR   0x62000000u
#define pNAND_DATA_ADDR  ((VUint8 *)0x62000000u)

//----------------------------------------------------------------------------------------------------
// NAND命令
//----------------------------------------------------------------------------------------------------
#define NAND_CMD_READ1            0x00u
#define NAND_CMD_READ2            0x30u
#define NAND_CMD_RANDOM_READ1     0x05u
#define NAND_CMD_RANDOM_READ2     0xE0u
#define NAND_CMD_READ_ID          0x90u
#define NAND_CMD_RESET            0xFFu
#define NAND_CMD_PAGE_PROGRAM1    0x80u
#define NAND_CMD_PAGE_PROGRAM2    0x10u
#define NAND_CMD_BLOCK_ERASE1     0x60u
#define NAND_CMD_BLOCK_ERASE2     0xD0u
#define NAND_CMD_READ_STATUS      0x70u

//----------------------------------------------------------------------------------------------------
//NAND定义
//----------------------------------------------------------------------------------------------------
#define NAND_PAGE_SIZE         2048u// page大小
#define NAND_PAGE_SPARE_SIZE   64u// 额外空间大小
#define NAND_PAGES_PER_BLOCK   64u// 每个块的page数
#define NAND_TOTAL_BLOCKS      8192u// nand总共的block数

#define NAND_SECTOR_SIZE       512u// page大小
#define NAND_SECTOR_SPARE_ECC_SIZE 10u// 额外空间大小
#define NAND_ID_BYTES_NUM      5u// NAND ID的字节数
#define NAND_BAD_BLOCK_MARKER_SIZE   2u

#define NAND_SECTORS_PER_PAGE  (NAND_PAGE_SIZE / NAND_SECTOR_SIZE)
#define NAND_SECTOR_SPARE_SIZE (NAND_PAGE_SPARE_SIZE / NAND_SECTORS_PER_PAGE)// sector额外空间大小
#define NAND_SECTOR_SPARE_NONECC_SIZE (NAND_SECTOR_SPARE_SIZE - NAND_SECTOR_SPARE_ECC_SIZE)
#define NAND_PAGE_TOTAL_SIZE   (NAND_PAGE_SIZE + NAND_PAGE_SPARE_SIZE)
#define NAND_BLOCK_SIZE        (NAND_PAGE_SIZE * NAND_PAGES_PER_BLOCK)// block大小
#define NAND_SPARE_ECC_OFFSET  (NAND_PAGE_SPARE_SIZE - NAND_SECTORS_PER_PAGE * NAND_SECTOR_SPARE_ECC_SIZE)
#define NAND_TOTAL_PAGES       (NAND_TOTAL_BLOCKS * NAND_PAGES_PER_BLOCK)// nand总共的page数

//====================================================================================================
// 外部函数声明
//====================================================================================================
extern int32 NandIDRead(Uint8 *id, Uint8 id_length);
extern int32 NandSectorRead(int32 page_addr, int32 sector_addr, Uint8 *dest, int32 data_length, Uint8 *p_spare, Uint8 mode);
extern int32 NandPageRead(int32 page_addr, Uint8 *dest, int32 data_length, Uint8 *p_spare, Uint8 mode);
extern int32 NandPageReadRandom(int32 page_addr, Uint8 *dest, int32 start_addr, int32 data_length, Uint8 *p_spare, Uint8 mode);
extern int32 NandRead(Uint8 *dest, int32 data_length, Uint64 nand_addr, Uint8 mode);
extern int32 NandReadUseBlockPageAddr(Uint8 *dest, int32 data_length, int32 block_addr, int32 page_addr, int32 byte_addr, Uint8 mode);
extern int32 NandSectorReadNoEccCheck(int32 page_addr, int32 sector_addr, Uint8 *dest, int32 data_length, Uint8 *p_spare);
extern int32 NandPageReadNoEccCheck(int32 page_addr, Uint8 *dest, int32 data_length, Uint8 *p_spare);
extern int32 NandPageWrite(int32 row_addr, Uint8 const *p_data, int32 data_length, Uint8 mode);
extern int32 NandReadPageSpareBytes(int32 row_addr, Uint8 *dest);
extern int32 NandBlockErase(int32 fv_block_number);
extern int32 NandSectorReadSpareBytes(int32 page_addr, int32 sector_addr, Uint8 *dest);
extern int32 NandPageReadSpareBytes(int32 page_addr, Uint8 *dest);
extern int32 NandBlockCheck(int32 block_addr);

#ifdef __cplusplus
}
#endif

#endif

