#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_adc.h"

#ifdef RT_USING_ADCONVERT

#ifndef RT_USING_ADC
#define MAX_DMA_BLOCK 3
#define TIMER_FREQUENCY_RANGE_MIN      ((uint32_t)    1)    /* Timer minimum frequency (unit: Hz), used to calculate frequency range. With a timer 16 bits, maximum frequency will be 32000 times this value. */
#define TIMER_PRESCALER_MAX_VALUE      (0xFFFF-1)           /* Timer prescaler maximum value (0xFFFF for a timer 16 bits) */
#endif

typedef enum{
	INDEX_CNLA = 0,
	INDEX_CNLB,
	INDEX_CNLC,
	INDEX_CNL_CNT,
}cnl_index_t;

typedef struct {
	ADC_HandleTypeDef		hadc;
#ifndef RT_USING_ADC
	DMA_HandleTypeDef		dma;
	rt_uint32_t 				XferAddrBase;
	rt_uint32_t         XferTransferNumber;
  rt_uint32_t    			XferSingleBytes;
  rt_uint32_t    			XferHalfBlockBytes;
  __IO rt_uint32_t    XferCount;
  __IO rt_uint32_t    XferSize;
	
	__IO rt_bool_t			bFirstBlock;	
	struct rt_completion rx_comp;	
#endif
}adc_dma_cnl_t;

struct adc_gpio_def{
#ifdef RT_USING_ADC
	struct rt_adc_device	parent;
#else
	struct rt_device		parent;
	RCC_ClkInitTypeDef	clk_init_struct;
	TIM_MasterConfigTypeDef master_timer_config;
	TIM_HandleTypeDef   Tim3Handle;
#endif
	adc_dma_cnl_t cnl[INDEX_CNL_CNT];
};
#ifdef RT_USING_ADC
static rt_err_t stm_enabled(struct rt_adc_device *device
	, rt_uint32_t channel, rt_bool_t enabled);
static rt_err_t stm_convert(struct rt_adc_device *device
	, rt_uint32_t channel, rt_uint32_t *value);
static struct rt_adc_ops __adc_ops = {stm_enabled,stm_convert};
#else
static void rt_dma_hw_init(void);
static rt_err_t adc_open(rt_device_t dev, rt_uint16_t oflag);
static rt_err_t adc_close(rt_device_t dev);
static rt_err_t adc_control(rt_device_t dev, int cmd, void *args);


//static rt_err_t adc_rx_indicate(rt_device_t dev, rt_size_t size);

//ALIGN(32)
 
#endif

//#pragma location=0x30044000
struct adc_gpio_def __adc_dev;
//__attribute__((at(0x30044000))) struct adc_gpio_def __adc_dev;

/*
	VSGNLA PF10 ADC3_INP6
	VSGNLB PB1  ADC12_INP5 ADC2
	VSGNLC PA6  ADC12_INP3 ADC1
	VSGNLV PA5  ADC12_INP19 ADC1
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
		__HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP); //ADC外设时钟选择
	
		if (ADC1 == hadc->Instance){
			__HAL_RCC_ADC12_CLK_ENABLE();           //使能ADC1/2时钟
			__HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
			GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_5;  //PA5 PA5
			GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //模拟
			GPIO_Initure.Pull=GPIO_NOPULL;          //不带上下拉
			HAL_GPIO_Init(GPIOA,&GPIO_Initure);	
		}
		else if (ADC2 == hadc->Instance){
			__HAL_RCC_ADC12_CLK_ENABLE();           //使能ADC1/2时钟
			__HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOA时钟
			GPIO_Initure.Pin=GPIO_PIN_1;            //PB1
			GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //模拟
			GPIO_Initure.Pull=GPIO_NOPULL;          //不带上下拉
			HAL_GPIO_Init(GPIOB,&GPIO_Initure);
		}
		else if (ADC3 == hadc->Instance){
			__HAL_RCC_ADC3_CLK_ENABLE();
			__HAL_RCC_GPIOF_CLK_ENABLE();			//开启GPIOA时钟
			GPIO_Initure.Pin=GPIO_PIN_10;            //PF10
			GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //模拟
			GPIO_Initure.Pull=GPIO_NOPULL;          //不带上下拉
			HAL_GPIO_Init(GPIOF,&GPIO_Initure);
		}
}
static void __adc_init(cnl_index_t idx, ADC_TypeDef* adc){
	ADC_ChannelConfTypeDef   sConfig;	
	ADC_HandleTypeDef* hadc = &__adc_dev.cnl[idx].hadc;
	
	RT_ASSERT(idx < INDEX_CNL_CNT);
	hadc->Instance=adc;
	hadc->Init.ClockPrescaler=ADC_CLOCK_ASYNC_DIV2; 	//4分频，ADCCLK=PER_CK/4=64/4=16MHZ
	hadc->Init.Resolution=ADC_RESOLUTION_16B;           	//16位模式
	hadc->Init.ScanConvMode=DISABLE;                    	//非扫描模式	
	hadc->Init.ContinuousConvMode=DISABLE;               //关闭连续转换

	hadc->Init.DiscontinuousConvMode=DISABLE;            //禁止不连续采样模式
	hadc->Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_RISING;//使用软件触发
	hadc->Init.NbrOfConversion=1;                        //1个转换在规则序列中 也就是只转换规则序列1 
	hadc->Init.NbrOfDiscConversion=0;                    //不连续采样通道数为0
	hadc->Init.LowPowerAutoWait=DISABLE;					//自动低功耗关闭				
	hadc->Init.BoostMode=ENABLE;							//BOOT模式关闭
	hadc->Init.EOCSelection=ADC_EOC_SINGLE_CONV;       	//关闭EOC中断
	hadc->Init.ExternalTrigConv=ADC_EXTERNALTRIG_T3_TRGO;      //软件触发
	hadc->Init.ConversionDataManagement=ADC_CONVERSIONDATA_DMA_CIRCULAR;  //规则通道的数据仅仅保存在DR寄存器里面

	
	hadc->Init.Overrun=ADC_OVR_DATA_OVERWRITTEN;			//有新的数据的死后直接覆盖掉旧数据
	hadc->Init.OversamplingMode=DISABLE;					//过采样关闭
	HAL_ADC_DeInit(hadc);                                 //初始化 
	HAL_ADC_Init(hadc);                                 //初始化 
	
	
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
	
	switch(idx){
		case INDEX_CNLA:
			sConfig.Channel      = ADC_CHANNEL_6;               /* Sampled channel number */
			break;		
		case INDEX_CNLB:
			sConfig.Channel      = ADC_CHANNEL_5;               /* Sampled channel number */
			break;		
		case INDEX_CNLC:
			sConfig.Channel      = ADC_CHANNEL_3;               /* Sampled channel number */
			//HAL_ADC_ConfigChannel(hadc, &sConfig);
			//sConfig.Channel      = ADC_CHANNEL_19;               /* Sampled channel number */
		break;
		default:
			break;
	}	
  HAL_ADC_ConfigChannel(hadc, &sConfig);
	HAL_ADCEx_Calibration_Start(hadc,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED); //ADC校准
}
#ifndef RT_USING_ADC
static void rt_triggle_source(rt_uint32_t hz){	
	static uint32_t latency;
  static uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
  static uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */

	__HAL_RCC_TIM3_CLK_ENABLE();

 	HAL_RCC_GetClockConfig(&__adc_dev.clk_init_struct, &latency);

	if (__adc_dev.clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1)
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
  else
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;

	
	timer_prescaler = timer_clock_frequency/ (TIMER_PRESCALER_MAX_VALUE * (hz /TIMER_FREQUENCY_RANGE_MIN)) +1;//(timer_clock_frequency / (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) +1;	
	
	__adc_dev.Tim3Handle.Instance = TIM3;  
  /* Configure timer parameters */
  __adc_dev.Tim3Handle.Init.Period            = timer_clock_frequency / (timer_prescaler * (hz)) ;
  __adc_dev.Tim3Handle.Init.Prescaler         = (timer_prescaler - 1);
  __adc_dev.Tim3Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  __adc_dev.Tim3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  __adc_dev.Tim3Handle.Init.RepetitionCounter = 0x0;	
	HAL_TIM_Base_Init(&__adc_dev.Tim3Handle);	
	
	__adc_dev.master_timer_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
  __adc_dev.master_timer_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  __adc_dev.master_timer_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;//TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&__adc_dev.Tim3Handle, &__adc_dev.master_timer_config);
	
}
#endif

static void rt_adc_hw_init(void){
	//rt_memset(&__adc_dev, 0, sizeof(struct adc_gpio_def));
	__adc_init(INDEX_CNLA, ADC3);
	__adc_init(INDEX_CNLB, ADC2);
	__adc_init(INDEX_CNLC, ADC1);
#ifndef RT_USING_ADC
	rt_triggle_source(ADC_FREQ);
	rt_dma_hw_init();
#endif
}

int rt_hw_adc_init(void){

	rt_memset(&__adc_dev, 0, sizeof(struct adc_gpio_def));
	for (rt_uint8_t n = 0; n < INDEX_CNL_CNT; n++){
		rt_completion_init(&(__adc_dev.cnl[n].rx_comp));
	}	
	rt_adc_hw_init();
	
#ifdef RT_USING_ADC
	return rt_hw_adc_register(&__adc_dev.parent, "adc", &__adc_ops, &__adc_dev);
#else
	__adc_dev.parent.type = RT_Device_Class_Char;	
	__adc_dev.parent.open = adc_open;
	__adc_dev.parent.close = adc_close;
	__adc_dev.parent.control = adc_control;
	
	return rt_device_register(&__adc_dev.parent, "adc", RT_DEVICE_FLAG_RDWR);
#endif
}
INIT_BOARD_EXPORT(rt_hw_adc_init);


#ifdef RT_USING_ADC
static rt_err_t stm_enabled(struct rt_adc_device *device
	, rt_uint32_t channel, rt_bool_t enabled){
		struct adc_gpio_def* padc = (struct adc_gpio_def*)device->parent.user_data;
    RT_ASSERT(device != RT_NULL);

    if (enabled){
        ADC_Enable(&padc->hadcA);
        ADC_Enable(&padc->hadcB);
        ADC_Enable(&padc->hadcC);
    }
    else{
        ADC_Disable(&padc->hadcA);
        ADC_Disable(&padc->hadcB);
        ADC_Disable(&padc->hadcC);
    }
    return RT_EOK;		
}
static rt_err_t stm_convert(struct rt_adc_device *device
	, rt_uint32_t channel, rt_uint32_t *value){
		struct adc_gpio_def* padc = (struct adc_gpio_def*)device->parent.user_data;
    RT_ASSERT(device != RT_NULL);

		if (channel < 4){
	    ADC_ChannelConfTypeDef ADC1_ChanConf;
			ADC_HandleTypeDef* hadc = 0;
			uint16_t cnl = 0;
			switch (channel){
				case 0://A
					hadc = &padc->hadcA;
					cnl = ADC_CHANNEL_6;
					break;
				case 1://B
					hadc = &padc->hadcB;
					cnl = ADC_CHANNEL_5;
					break;
				case 2://C
					hadc = &padc->hadcC;
					cnl = ADC_CHANNEL_3;
					break;
				case 3://V
					hadc = &padc->hadcC;
					cnl = ADC_CHANNEL_19;
					break;
				default:
					return -RT_ENOSYS;					
			}
			ADC1_ChanConf.Channel = cnl; //通道
			ADC1_ChanConf.Rank=ADC_REGULAR_RANK_1;                  	//1个序列
			ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_64CYCLES_5;      	//采样时间       
			ADC1_ChanConf.SingleDiff=ADC_SINGLE_ENDED;  				//单边采集          		
			ADC1_ChanConf.OffsetNumber=ADC_OFFSET_NONE;             	
			ADC1_ChanConf.Offset=0;
			HAL_ADC_ConfigChannel(hadc,&ADC1_ChanConf);        //通道配置

			HAL_ADC_Start(hadc);                               //开启ADC
	
			if (HAL_OK != HAL_ADC_PollForConversion(hadc,10))
				rt_kprintf("adc converted timeout\n");                //轮询转换
			*value = (rt_uint16_t)HAL_ADC_GetValue(hadc);	            //返回最近一次ADC1规则组的转换结果
			
			return RT_EOK;	
		}
		else
			return -RT_ENOSYS;
}
#else
static void __dma_init(cnl_index_t idx, DMA_Stream_TypeDef * stream){
	rt_uint32_t mode = DMA_CIRCULAR;
	adc_dma_cnl_t* cnl = &__adc_dev.cnl[idx];
	ADC_HandleTypeDef* hadc = &cnl->hadc;	
	DMA_HandleTypeDef* hdma = &cnl->dma;
	hdma->Instance 								= stream;
  hdma->Init.Direction 					= DMA_PERIPH_TO_MEMORY;
  hdma->Init.PeriphInc 					= DMA_PINC_DISABLE;
  hdma->Init.MemInc    					= DMA_MINC_ENABLE;
	hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma->Init.Mode                = mode;
  hdma->Init.Priority            = DMA_PRIORITY_MEDIUM;	
	switch ((rt_uint32_t)hadc->Instance){
	case (rt_uint32_t)ADC1:
		hdma->Init.Request  						= DMA_REQUEST_ADC1;
		break;
	case (rt_uint32_t)ADC2:
		hdma->Init.Request  						= DMA_REQUEST_ADC2;
		break;
	case (rt_uint32_t)ADC3:
		hdma->Init.Request  						= DMA_REQUEST_ADC3;
		break;
	default:
		break;		
	}
	HAL_DMA_DeInit(hdma);
  HAL_DMA_Init(hdma);	
	__HAL_LINKDMA(hadc, DMA_Handle, cnl->dma);

}
static void ADC_DMAError(DMA_HandleTypeDef *hdma)
{
  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Set ADC state */
  SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);

  /* Set ADC error code to DMA error */
  SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_DMA);

  /* Error callback */
  //HAL_ADC_ErrorCallback(hadc);
}
static char tmp_char[1024];
static void __config_multi_dma(adc_dma_cnl_t* cnl, uint32_t* pData, uint32_t Length){
		rt_uint32_t	SecondMemAddress = 0;
		ADC_HandleTypeDef* hadc = &cnl->hadc;	
		//DMA_HandleTypeDef* hdma = &cnl->dma;
		RT_ASSERT(0 == (Length % ADC_BUFF_SLICE_SZ));//必须整倍数，不考虑多余情况	
		RT_ASSERT(2 <= (Length / ADC_BUFF_SLICE_SZ));//至少2个buffer

	  cnl->XferCount = 0;
		cnl->XferTransferNumber = 0;
	
		cnl->XferSingleBytes			= 2;
		cnl->XferCount = 1;
		cnl->XferSize = Length;
		cnl->XferAddrBase = (rt_uint32_t)pData;
      
    /* Get the number of buffer */
    while(cnl->XferSize > 0xFFFF)
    {
      cnl->XferSize = (cnl->XferSize/2);
      cnl->XferCount = cnl->XferCount*2;
    }

    /* Update DCMI counter  and transfer number*/
    cnl->XferCount = (cnl->XferCount - 2);
    cnl->XferTransferNumber = cnl->XferCount;
		cnl->XferHalfBlockBytes = cnl->XferSingleBytes * Length / 2;
    /* Update second memory address */
    SecondMemAddress = cnl->XferAddrBase + (cnl->XferSingleBytes*cnl->XferSize);
	
		rt_snprintf(tmp_char, 1024, "CNL[%d]%08X %d %d %d %d %d %08X\n"
		, cnl - &__adc_dev.cnl[0]
		, cnl->XferAddrBase
		, cnl->XferTransferNumber
		, cnl->XferSingleBytes
		, cnl->XferHalfBlockBytes
		, cnl->XferCount
		, cnl->XferSize
		, SecondMemAddress
		);
		rt_kprintf(tmp_char);
    /* Start DMA multi buffer transfer */
    HAL_DMAEx_MultiBufferStart_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, SecondMemAddress, cnl->XferSize);
}
static HAL_StatusTypeDef HAL_ADC_Start_MultiDMA(adc_dma_cnl_t* cnl /*ADC_HandleTypeDef* hadc*/, uint32_t* pData, uint32_t Length)
{
	ADC_HandleTypeDef* hadc = &cnl->hadc;
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Perform ADC enable and conversion start if no conversion is on going */
  if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET)
  {
    /* Process locked */
    __HAL_LOCK(hadc);

    /* Ensure that dual regular conversions are not enabled or unavailable.   */
    /* Otherwise, dedicated API HAL_ADCEx_MultiModeStart_DMA() must be used.  */
    if (ADC_IS_DUAL_REGULAR_CONVERSION_ENABLE(hadc) == RESET)
    {
      /* Enable the ADC peripheral */
      tmp_hal_status = ADC_Enable(hadc);

      /* Start conversion if ADC is effectively enabled */
      if (tmp_hal_status == HAL_OK)
      {
        /* State machine update: Check if an injected conversion is ongoing */
        if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
        {
          /* Reset ADC error code fields related to regular conversions only */
          CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
        }
        else
        {
          /* Set ADC error code to none */
          ADC_CLEAR_ERRORCODE(hadc);
        }
        /* Clear HAL_ADC_STATE_READY and regular conversion results bits, set HAL_ADC_STATE_REG_BUSY bit */
        ADC_STATE_CLR_SET(hadc->State,
                          (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP),
                          HAL_ADC_STATE_REG_BUSY);

        /* Reset HAL_ADC_STATE_MULTIMODE_SLAVE bit
           - by default if ADC is Master or Independent or if multimode feature is not available
           - if multimode setting is set to independent mode (no dual regular or injected conversions are configured) */
        if (ADC12_NONMULTIMODE_OR_MULTIMODEMASTER(hadc))
        {
          CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
        }

        /* Set the DMA transfer complete callback */
        hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;
        hadc->DMA_Handle->XferM1CpltCallback = ADC_DMAConvCplt;

        /* Set the DMA half transfer complete callback */
        hadc->DMA_Handle->XferHalfCpltCallback = 0;//ADC_DMAHalfConvCplt;

        /* Set the DMA error callback */
        hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;


        /* Manage ADC and DMA start: ADC overrun interruption, DMA start,     */
        /* ADC start (in case of SW start):                                   */

        /* Clear regular group conversion flag and overrun flag               */
        /* (To ensure of no unknown state from potential previous ADC         */
        /* operations)                                                        */
        __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

        /* With DMA, overrun event is always considered as an error even if
           hadc->Init.Overrun is set to ADC_OVR_DATA_OVERWRITTEN. Therefore,
           ADC_IT_OVR is enabled.  */
        __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

        /* Start the DMA channel */
        //HAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, Length);
				__config_multi_dma(cnl, pData, Length);
        /* Enable conversion of regular group.                                  */
        /* Process unlocked */
        __HAL_UNLOCK(hadc);
        /* If software start has been selected, conversion starts immediately.  */
        /* If external trigger has been selected, conversion will start at next */
        /* trigger event.                                                       */
        SET_BIT(hadc->Instance->CR, ADC_CR_ADSTART);
      }
      else
      {
        /* Process unlocked */
        __HAL_UNLOCK(hadc);
      }  /* if (tmp_hal_status == HAL_OK) */
    }
    else
    {
      tmp_hal_status = HAL_ERROR;
      /* Process unlocked */
      __HAL_UNLOCK(hadc);
    } /* if (ADC_IS_DUAL_REGULAR_CONVERSION_ENABLE(hadc) == RESET) */

  }
  else
  {
    tmp_hal_status = HAL_BUSY;
  }

  /* Return function status */
  return tmp_hal_status;
}

static void __adc_dma_start(cnl_index_t idx, rt_uint32_t addr, rt_uint32_t length){
	adc_dma_cnl_t* cnl = &__adc_dev.cnl[idx];
	//ADC_HandleTypeDef* hadc = &cnl->hadc;	
	//DMA_HandleTypeDef* hdma = &cnl->dma;
	HAL_ADC_Start_MultiDMA(cnl, (rt_uint32_t*)addr, length);
}
static void __adc_dma_stop(cnl_index_t idx){
	adc_dma_cnl_t* cnl = &__adc_dev.cnl[idx];
	ADC_HandleTypeDef* hadc = &cnl->hadc;	
	//DMA_HandleTypeDef* hdma = &cnl->dma;
	//HAL_DMA_Abort(hdma);
	HAL_ADC_Stop_DMA(hadc);
}

static void rt_dma_hw_init(void){
  __HAL_RCC_DMA1_CLK_ENABLE();
	//__HAL_LINKDMA(&__adc_dev.cnl[ADC_CNLA].hadc, DMA_Handle, __adc_dev.cnl[ADC_CNLA].dma);
	//__HAL_LINKDMA(&__adc_dev.cnl[ADC_CNLB].hadc, DMA_Handle, __adc_dev.cnl[ADC_CNLB].dma);
	//__HAL_LINKDMA(&__adc_dev.cnl[ADC_CNLC].hadc, DMA_Handle, __adc_dev.cnl[ADC_CNLC].dma);

	__dma_init(INDEX_CNLA, DMA1_Stream3);
	__dma_init(INDEX_CNLB, DMA1_Stream2);
	__dma_init(INDEX_CNLC, DMA1_Stream1);	

  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);  
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);  	
	
	//HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(TIM3_IRQn);

}
static rt_err_t adc_open(rt_device_t dev, rt_uint16_t oflag){
	struct adc_gpio_def* padc = (struct adc_gpio_def*) dev;
	rt_memset((void*)ADC_CNLA, 0xCC, ADC_BUFF_PHYS_SZ);
	rt_memset((void*)ADC_CNLB, 0xCC, ADC_BUFF_PHYS_SZ);
	rt_memset((void*)ADC_CNLC, 0xCC, ADC_BUFF_PHYS_SZ);

	__HAL_TIM_CLEAR_IT(&padc->Tim3Handle, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(&padc->Tim3Handle, TIM_FLAG_UPDATE);

	__adc_dma_start(INDEX_CNLA, ADC_CNLA, ADC_BUFF_SZ);
	__adc_dma_start(INDEX_CNLB, ADC_CNLB, ADC_BUFF_SZ);
	__adc_dma_start(INDEX_CNLC, ADC_CNLC, ADC_BUFF_SZ);
	
	/*
	
	
	__adc_dev.m_bFirstA = RT_TRUE;
	__adc_dev.m_bFirstB = RT_TRUE;
	__adc_dev.m_bFirstC = RT_TRUE;
	
	__adc_dev.m_bFirstHalfA = RT_FALSE;
	__adc_dev.m_bFirstHalfB = RT_FALSE;
	__adc_dev.m_bFirstHalfC = RT_FALSE;
	
	HAL_ADC_Start_DMA(&padc->hadcA,
                        (uint32_t *)ADC_CNLA,
                        ADC_BUFF_SLICE_SZ
                       ) ;
	HAL_ADC_Start_DMA(&padc->hadcB,
                        (uint32_t *)ADC_CNLB,
                        ADC_BUFF_SLICE_SZ
                       ) ;
	HAL_ADC_Start_DMA(&padc->hadcC,
                        (uint32_t *)ADC_CNLC,
                        ADC_BUFF_SLICE_SZ*2
                       ) ;*/
					 
	HAL_TIM_Base_Start(&padc->Tim3Handle);
	return RT_EOK;
}
static rt_err_t adc_close(rt_device_t dev){
	struct adc_gpio_def* padc = (struct adc_gpio_def*) dev;
	HAL_TIM_Base_Stop(&padc->Tim3Handle);
	__adc_dma_stop(INDEX_CNLA);
	__adc_dma_stop(INDEX_CNLB);
	__adc_dma_stop(INDEX_CNLC);

	
	__HAL_TIM_DISABLE_IT(&padc->Tim3Handle, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&padc->Tim3Handle, 0);
	for (rt_uint8_t n = 0; n < INDEX_CNL_CNT; n++){
		rt_completion_wait(&(__adc_dev.cnl[n].rx_comp), 0);
	}
	return RT_EOK;
}
static rt_err_t adc_control(rt_device_t dev, int cmd, void *args){
	struct adc_gpio_def* padc = (struct adc_gpio_def*) dev;
	switch (cmd){
		case RT_DEVICE_ADC_SET_FREQ :		{	
			  uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
				uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
				uint32_t hz = *(uint32_t*)args;
				if (padc->clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1)
				{
					timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
				}
				else
				{
					timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;
				}
				timer_prescaler = timer_clock_frequency/ (TIMER_PRESCALER_MAX_VALUE * (hz /TIMER_FREQUENCY_RANGE_MIN)) +1;//(timer_clock_frequency / (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) +1;	
				padc->Tim3Handle.Init.Period            = ((timer_clock_frequency / (timer_prescaler * hz)) - 1);
				padc->Tim3Handle.Init.Prescaler         = (timer_prescaler - 1);
		}break;
		case RT_DEVICE_ADC_WAIT_OFFSET:
		 RT_ASSERT(args != RT_NULL);
		 rt_completion_wait(&__adc_dev.cnl[0].rx_comp, RT_WAITING_FOREVER);
		 rt_completion_wait(&__adc_dev.cnl[1].rx_comp, RT_WAITING_FOREVER);
		 rt_completion_wait(&__adc_dev.cnl[2].rx_comp, RT_WAITING_FOREVER);
		 if ((__adc_dev.cnl[0].bFirstBlock !=__adc_dev.cnl[1].bFirstBlock) || (__adc_dev.cnl[0].bFirstBlock != __adc_dev.cnl[2].bFirstBlock))
			 rt_kprintf("not synchrous\n");
			if(__adc_dev.cnl[1].bFirstBlock)
				*(rt_uint32_t*)args = 0;
			else
				*(rt_uint32_t*)args = __adc_dev.cnl[0].XferHalfBlockBytes;
			break;
		default:
			return -RT_ENOSYS;
	}
	return RT_EOK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	adc_dma_cnl_t* cnl = (adc_dma_cnl_t*)hadc;
	static uint32_t tmp = 0;
	tmp  = ((((DMA_Stream_TypeDef *)(cnl->dma.Instance))->CR) & DMA_SxCR_CT);

  if(cnl->XferCount != 0)
  {
		uint32_t addr = 0;
    /* Update memory 0 address location */
    if (((cnl->XferCount & 1) == 0) && (tmp != 0))//(((cnl->XferCount % 2) == 0) && (tmp != 0))
    {
      addr = ((DMA_Stream_TypeDef *)(cnl->dma.Instance))->M0AR;
      HAL_DMAEx_ChangeMemory(&cnl->dma, (addr + (cnl->XferSingleBytes*cnl->XferSize * 2)), MEMORY0);
      cnl->XferCount--;	 
		}
    
    else if (((cnl->XferCount & 1) == 1) && (tmp == 0))/* Update memory 1 address location */
    {
      addr = ((DMA_Stream_TypeDef *)(cnl->dma.Instance))->M1AR;
      HAL_DMAEx_ChangeMemory(&cnl->dma, (addr + (cnl->XferSingleBytes*cnl->XferSize * 2)), MEMORY1);
      cnl->XferCount--;
    }
		else{
			__nop();
		}
		if (addr == (cnl->XferAddrBase + cnl->XferHalfBlockBytes - cnl->XferSize * cnl->XferSingleBytes)){
				cnl->bFirstBlock = RT_TRUE;
				rt_completion_done(&(cnl->rx_comp));
			}	
  }
  else if (tmp != 0)/* Update memory 0 address location */
  {
		HAL_DMAEx_ChangeMemory(&cnl->dma, cnl->XferAddrBase, MEMORY0);
  }
  else if (tmp == 0) /* Update memory 1 address location */
  {   
		HAL_DMAEx_ChangeMemory(&cnl->dma, cnl->XferAddrBase + (cnl->XferSingleBytes*cnl->XferSize), MEMORY1);
    cnl->XferCount = cnl->XferTransferNumber;
		cnl->bFirstBlock = RT_FALSE;
		rt_completion_done(&(cnl->rx_comp));
  }
}

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static rt_tick_t t1ast = 0, tdet = 0, count = 0;
	count++;
	if (count == ADC_FREQ){
		count = 0;
		if (t1ast == 0) t1ast = rt_tick_get();
		tdet = rt_tick_get() - t1ast;
		t1ast += tdet;
		__nop();
	}
}*/
void TIM3_IRQHandler(void){
	HAL_TIM_IRQHandler(&__adc_dev.Tim3Handle);
}
void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(__adc_dev.cnl[INDEX_CNLA].hadc.DMA_Handle);
}
void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(__adc_dev.cnl[INDEX_CNLB].hadc.DMA_Handle);
}
void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(__adc_dev.cnl[INDEX_CNLC].hadc.DMA_Handle);
}
/*
void ADC_IRQHandler(void){
	while (1);
}
void ADC3_IRQHandler(void){
	while (1);
	HAL_ADC_IRQHandler(&__adc_dev.hadcA);
}
*/

#endif

#endif

