#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_sdram.h"

#ifdef RT_USING_EXSDRAM
#include "stm32h7xx.h"


/* #define SDRAM_MEMORY_WIDTH            FMC_SDRAM_MEM_BUS_WIDTH_8  */
#define SDRAM_MEMORY_WIDTH            FMC_SDRAM_MEM_BUS_WIDTH_16 
/* #define SDRAM_MEMORY_WIDTH               FMC_SDRAM_MEM_BUS_WIDTH_32*/

#define SDCLOCK_PERIOD                   FMC_SDRAM_CLOCK_PERIOD_2 
/* #define SDCLOCK_PERIOD                FMC_SDRAM_CLOCK_PERIOD_3*/


#define SDRAM_CLK_ENABLE()            do{__HAL_RCC_SYSCFG_CLK_ENABLE();__HAL_RCC_FMC_CLK_ENABLE();}while(0)
#define SDRAM_GPIO_CLK_ENABLE()      	do{		\
																					__GPIOC_CLK_ENABLE();\
																					__GPIOD_CLK_ENABLE();\
																					__GPIOE_CLK_ENABLE();\
																					__GPIOF_CLK_ENABLE();\
																					__GPIOG_CLK_ENABLE();\
																			}while(0)

#define SDRAN_AF                     GPIO_AF12_FMC
																			
#define REFRESH_COUNT                    ((uint32_t)677)   /* SDRAM refresh counter (100Mhz FMC clock) */

#define SDRAM_TIMEOUT                    ((uint32_t)0xFFFF)
																			
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)
/*																			
#define DMA_BUFFER_SZ (4096)																			
ALIGN(32)
static rt_uint8_t dma_buffer[DMA_BUFFER_SZ];		*/																	
																			
struct stm32_sdram{
	//struct rt_device parent;
	//struct rt_mutex  lock;
	SDRAM_HandleTypeDef sdramHandle;
	FMC_SDRAM_TimingTypeDef Timing;
	FMC_SDRAM_CommandTypeDef Command;	
};
									
static struct stm32_sdram _stm32_sdram;

																			
void HAL_SDRAM_MspInit(SDRAM_HandleTypeDef *hsdram)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
	SDRAM_CLK_ENABLE();
	SDRAM_GPIO_CLK_ENABLE();
	
	GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;        
	GPIO_InitStruct.Pin 			= GPIO_PIN_5;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;        
	GPIO_InitStruct.Alternate = SDRAN_AF;
	
	GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1/* | GPIO_PIN_5*/
													| GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
													| GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7
													| GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
													| GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
													| GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11
													| GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);	
													
	GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
													| GPIO_PIN_4 | GPIO_PIN_5	| GPIO_PIN_8 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);												
}
/*
static rt_size_t stm32_sdram_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size){
	struct stm32_sdram* sdram = (struct stm32_sdram*)dev;
	rt_mutex_take(&_stm32_sdram.lock, RT_WAITING_FOREVER);
	if (dev->open_flag & RT_DEVICE_FLAG_DMA_RX){
		rt_uint32_t addr = (rt_uint32_t)buffer;
		if (addr& 0x03){
			rt_size_t len = 0;
			while (len < size){
				int rbytes = size - len;
				if (rbytes > DMA_BUFFER_SZ)
					rbytes = DMA_BUFFER_SZ;
				else if (size & 0x03){
					rbytes = (rbytes + 3) &(~0x03);
				}					
				HAL_SDRAM_Read_DMA(&sdram->sdramHandle, (rt_uint32_t*)(pos+len), (uint32_t*)dma_buffer, rbytes);
				rt_memcpy((rt_uint8_t*)buffer+len, dma_buffer, rbytes);
				len+=rbytes;				
			}
		}
		else
			HAL_SDRAM_Read_DMA(&sdram->sdramHandle, (rt_uint32_t*)pos, buffer, size);
	}
	else{
		HAL_SDRAM_Read_8b(&sdram->sdramHandle, (rt_uint32_t*)pos, buffer, size);
	}
	rt_mutex_release(&_stm32_sdram.lock);
	return size;
}
static rt_size_t stm32_sdram_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size){
	struct stm32_sdram* sdram = (struct stm32_sdram*)dev;
	rt_mutex_take(&_stm32_sdram.lock, RT_WAITING_FOREVER);
	if (dev->open_flag & RT_DEVICE_FLAG_DMA_TX){
		rt_uint32_t addr = (rt_uint32_t)buffer;
		if (addr& 0x03){
			rt_size_t len = 0;
			while (len < size){
				int wbytes = size - len;
				if (wbytes > DMA_BUFFER_SZ)
					wbytes = DMA_BUFFER_SZ;
				rt_memcpy(dma_buffer, (rt_uint8_t*)buffer+len, wbytes);
				HAL_SDRAM_Write_DMA(&sdram->sdramHandle, (rt_uint32_t*)(pos+len), (uint32_t*)dma_buffer, wbytes);
				len+=wbytes;				
			}
		}
		else
			HAL_SDRAM_Write_DMA(&sdram->sdramHandle, (rt_uint32_t*)pos, (rt_uint32_t*) buffer, size);
	}
	else{
		if (HAL_OK != HAL_SDRAM_Write_8b(&sdram->sdramHandle, (rt_uint32_t*)pos,(rt_uint8_t*) buffer, size))
			return 0;
	}
	rt_mutex_release(&_stm32_sdram.lock);
	return size;
}
*/
int stm32_hw_sdram_init(void){
	rt_memset(&_stm32_sdram, 0, sizeof(struct stm32_sdram));
  /* SDRAM device configuration */
  _stm32_sdram.sdramHandle.Instance = FMC_SDRAM_DEVICE;

  /* Timing configuration for 100Mhz as SDRAM clock frequency (System clock is up to 200Mhz) */
  _stm32_sdram.Timing.LoadToActiveDelay    = 2;
  _stm32_sdram.Timing.ExitSelfRefreshDelay = 8;
  _stm32_sdram.Timing.SelfRefreshTime      = 6;
  _stm32_sdram.Timing.RowCycleDelay        = 6;
  _stm32_sdram.Timing.WriteRecoveryTime    = 2;
  _stm32_sdram.Timing.RPDelay              = 2;
  _stm32_sdram.Timing.RCDDelay             = 2;

  _stm32_sdram.sdramHandle.Init.SDBank             = FMC_SDRAM_BANK1;
  _stm32_sdram.sdramHandle.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_9;
  _stm32_sdram.sdramHandle.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_13;
  _stm32_sdram.sdramHandle.Init.MemoryDataWidth    = SDRAM_MEMORY_WIDTH;
  _stm32_sdram.sdramHandle.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  _stm32_sdram.sdramHandle.Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_2;
  _stm32_sdram.sdramHandle.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  _stm32_sdram.sdramHandle.Init.SDClockPeriod      = SDCLOCK_PERIOD;
  _stm32_sdram.sdramHandle.Init.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
  _stm32_sdram.sdramHandle.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_0;//FMC_SDRAM_RPIPE_DELAY_0

/*	_stm32_sdram.parent.type = RT_Device_Class_MTD;
	_stm32_sdram.parent.read = stm32_sdram_read;
	_stm32_sdram.parent.write = stm32_sdram_write;
	rt_mutex_init(&_stm32_sdram.lock, "sdram0", RT_IPC_FLAG_FIFO);*/
	if (HAL_OK == HAL_SDRAM_Init(&_stm32_sdram.sdramHandle, &_stm32_sdram.Timing)){
		uint32_t tmpmrd;
		_stm32_sdram.Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
		_stm32_sdram.Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
		_stm32_sdram.Command.AutoRefreshNumber      = 1;
		_stm32_sdram.Command.ModeRegisterDefinition = 0;

		/* Send the command */
		HAL_SDRAM_SendCommand(&_stm32_sdram.sdramHandle, &_stm32_sdram.Command, SDRAM_TIMEOUT);

		/* Step 2: Insert 100 us minimum delay */
		/* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
		{
				volatile uint32_t delay = 0xFFFFFF;
				while (delay--);
			//HAL_Delay((1);
		}
		/* Step 3: Configure a PALL (precharge all) command */
		_stm32_sdram.Command.CommandMode            = FMC_SDRAM_CMD_PALL;
		_stm32_sdram.Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
		_stm32_sdram.Command.AutoRefreshNumber      = 1;
		_stm32_sdram.Command.ModeRegisterDefinition = 0;

		/* Send the command */
		if (HAL_OK != HAL_SDRAM_SendCommand(&_stm32_sdram.sdramHandle, &_stm32_sdram.Command, SDRAM_TIMEOUT))
			rt_kprintf("SDRAM: err 1\n");

		/* Step 4: Configure an Auto Refresh command */
		_stm32_sdram.Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
		_stm32_sdram.Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
		_stm32_sdram.Command.AutoRefreshNumber      = 8;
		_stm32_sdram.Command.ModeRegisterDefinition = 0;

		/* Send the command */
		if (HAL_OK != HAL_SDRAM_SendCommand(&_stm32_sdram.sdramHandle, &_stm32_sdram.Command, SDRAM_TIMEOUT))
			rt_kprintf("SDRAM: err 2\n");

		/* Step 5: Program the external memory mode register */
		tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |\
											 SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
											 SDRAM_MODEREG_CAS_LATENCY_2           |\
											 SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
											 SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

		_stm32_sdram.Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
		_stm32_sdram.Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
		_stm32_sdram.Command.AutoRefreshNumber      = 1;
		_stm32_sdram.Command.ModeRegisterDefinition = tmpmrd;

		/* Send the command */
		if (HAL_OK != HAL_SDRAM_SendCommand(&_stm32_sdram.sdramHandle, &_stm32_sdram.Command, SDRAM_TIMEOUT))
						rt_kprintf("SDRAM: err 3\n");


		/* Step 6: Set the refresh rate counter */
		/* Set the device refresh rate */
		if (HAL_OK != HAL_SDRAM_ProgramRefreshRate(&_stm32_sdram.sdramHandle, REFRESH_COUNT))
						rt_kprintf("SDRAM: err 4\n");


		
		
		//rt_device_register(&_stm32_sdram.parent, "sdram0", RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_DMA_RX|RT_DEVICE_FLAG_DMA_TX);
		return 0;
	}
	else
		return -1;
}
INIT_BOARD_EXPORT(stm32_hw_sdram_init);





#endif
