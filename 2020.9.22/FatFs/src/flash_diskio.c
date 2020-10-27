
/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "flash_diskio.h"
#include "FlashIO.h"
#include <string.h>
#include "Basic.h"
#include "stm32h743xx.h"

#define Flash_TIMEOUT 1.0
static uint16_t BLOCKSIZE = 0;
#ifdef DCACHE_SIZE
 Static_AXIDMABuf uint8_t scratch[FF_MAX_SS];
#endif

static DSTATUS Flash_initialize(BYTE lun);
static DSTATUS Flash_status(BYTE lun);
static DRESULT Flash_read(BYTE lun, BYTE *buff, DWORD sector, UINT count);
static DRESULT Flash_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count);
static DRESULT Flash_ioctl(BYTE lun, BYTE cmd, void *buff);

const Diskio_drvTypeDef  Flash_Driver =
{
  Flash_initialize,
  Flash_status,
  Flash_read,
#if  _USE_WRITE == 1
  Flash_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  Flash_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
static DSTATUS Flash_initialize(BYTE lun)
{
  DSTATUS Stat = STA_NOINIT;
	BLOCKSIZE = getFlashSectorSize();
	if( getFlashSectorCount() != 0 )
		Stat &= ~STA_NOINIT;
  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
static DSTATUS Flash_status(BYTE lun)
{
  DSTATUS Stat = STA_NOINIT;
	if( getFlashSectorCount() != 0 )
		Stat &= ~STA_NOINIT;
  return Stat;
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
static DRESULT Flash_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  osEvent event;
  uint32_t timer;
#ifdef DCACHE_SIZE
	if( isDMABuf( (uint32_t)buff ) )
	{	//������Ϊ��cache����
#endif
		//ֱ��dma�ͽ�������
		if( FlashReadSectors( (uint8_t*)buff, (uint32_t)(sector), count, Flash_TIMEOUT ) == true )
		{
			res = RES_OK;
		}	
#ifdef DCACHE_SIZE
	}
	else if( ((uint32_t)buff & 0x1f) == 0 )
	{	//������32�ֽڶ���
		//ֱ��dma�ͽ�������
		SCB_InvalidateDCache_by_Addr((uint32_t*)buff, count*BLOCKSIZE);
		if( FlashReadSectors( (uint8_t*)buff, (uint32_t)(sector), count, Flash_TIMEOUT ) == true )
		{
			res = RES_OK;
		}
	}
	else
	{
		//��������32�ֽڶ���
		//��������dma���������û������ڸ��ƹ�ȥ
		int i;
		bool ret = false;
		for (i = 0; i < count; i++) 
		{
      ret = FlashReadSectors( (uint8_t*)scratch, (uint32_t)(sector), 1, Flash_TIMEOUT );
      if (ret == true) 
			{
				//�����ݴ����û��������Ƶ�buf
				memcpy(buff, scratch, BLOCKSIZE);
				buff += BLOCKSIZE;
      }
      else
        break;
    }

    if ((i == count) && (ret == true))
			res = RES_OK;
	}
#endif
  return res;
}

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
static DRESULT Flash_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  osEvent event;
  uint32_t timer;

#ifdef DCACHE_SIZE
	if( isDMABuf( (uint32_t)buff ) )
	{	//������Ϊ��cache����
		//ֱ����dma����
#endif
		if( FlashWriteSectors( (uint8_t*)buff, (uint32_t)(sector), count , Flash_TIMEOUT ) == true )
		{
			res = RES_OK;
		}
	
#ifdef DCACHE_SIZE
	}
	else if( ((uint32_t)buff & 0x1f) == 0 )
	{	//������32�ֽڶ���
		//ֱ����dma����
		
		//��Cache����д���ڴ�
		SCB_CleanDCache_by_Addr((uint32_t*)buff, count*BLOCKSIZE);
		if( FlashWriteSectors( (uint8_t*)buff, (uint32_t)(sector), count , Flash_TIMEOUT ) == true )
		{
			res = RES_OK;
		}
	}
	else
	{	//��������32�ֽڶ���
		//�������ȸ��Ƶ����û������ٷ���	
		int i;
    uint8_t ret;
		
    for (i = 0; i < count; i++) {
			//���ݴ�buf������scratch
			memcpy((void *)scratch, (void *)buff, BLOCKSIZE);
      ret = FlashWriteSectors( (uint8_t*)scratch, (uint32_t)(sector), 1 , Flash_TIMEOUT );
      if (ret == true) 
			{
				buff += BLOCKSIZE;
      }
      else
        break;
    }
		
    if ((i == count) && (ret == true))
		{
			res = RES_OK;
		}
	}
#endif
  return res;
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
static DRESULT Flash_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  if (BLOCKSIZE == 0 || getFlashSectorCount()==0) 
		return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    *(DWORD*)buff = getFlashSectorCount();
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    *(WORD*)buff = getFlashSectorSize();
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    *(DWORD*)buff = 1;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
#endif /* _USE_IOCTL == 1 */
