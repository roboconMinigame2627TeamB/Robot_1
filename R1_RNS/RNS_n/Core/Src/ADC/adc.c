/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adc.h"
/************************************************/
/*		 	 	Variables	      	 		  	*/
/************************************************/

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/
/*
 * Function Name		: ADC_DMA_Init
 * Function Description : Configure adc with dma. you have to call adc_channel config to start converting.
 * Function Remarks		: None
 * Function Arguments	: ADC_t						pointer to adc structure
 * 						  hadcx		       			pointer to adc handle
 * 						  hdma_adcx		 			pointer to dma handle
 * 						  DMA_CHANNEL				Dma channel used for conversion. can be DMA_CHANNEL_0 to DMA_CHANNEL_15
 * 						  ChannelNo					Specifies the Number of channels that will be converted
 * 						  receive_buffer			buffer that will contain the data received for example receive_buffer[0]
 * 						  							will have the first channel data,receive_buffer[1] will have the 2nd
 * 						  							channel and so on.
 * 						  dmapreemptionpriority		preemption priority for DMA interrupt
 * 						  dmasubpriority			subpriority for DMA interrupt
 * Function Return		: None
 * Function Example		: ADC_DMA_Init(&adc1,&hadc2,&hdma_adc1,DMA_CHANNEL_0,2,adc1_buf,0,0);
 */
void ADC_DMAxInit(ADC_t* ADC_t,ADC_HandleTypeDef* hadcx,DMA_HandleTypeDef* hdma_adcx,
		uint32_t ChannelNo, uint16_t dmapreemptionpriority,uint16_t dmasubpriority)
{


	IRQn_Type nvic;

	__HAL_RCC_DMA2_CLK_ENABLE();
	ADC_t->ADC_number = ChannelNo;
	ADC_t->ADC_rank = 1;
	ADC_t->ADC_valuep = ADC_t->ADC_value;
	ADC_t->hadcx = hadcx;
	uint32_t DMA_CHANNEL;



	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	if(hadcx == &hadc1){
		hadcx->Instance = ADC1;
		hdma_adcx->Instance = DMA2_Stream0;
		nvic = DMA2_Stream0_IRQn;
		DMA_CHANNEL = DMA_CHANNEL_0;
	}else if(hadcx == &hadc2){
		hadcx->Instance = ADC2;
		hdma_adcx->Instance = DMA2_Stream2;
		nvic = DMA2_Stream2_IRQn;
		DMA_CHANNEL = DMA_CHANNEL_1;
	}else if (hadcx == &hadc3){
		hadcx->Instance = ADC3;
		hdma_adcx->Instance = DMA2_Stream1;
		nvic = DMA2_Stream1_IRQn;
		DMA_CHANNEL = DMA_CHANNEL_2;
	}


	hadcx->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadcx->Init.Resolution = ADC_RESOLUTION_12B;
	hadcx->Init.ScanConvMode = ENABLE;
	hadcx->Init.ContinuousConvMode = ENABLE;
	hadcx->Init.DiscontinuousConvMode = DISABLE;
	hadcx->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadcx->Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadcx->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadcx->Init.NbrOfConversion = ChannelNo;
	hadcx->Init.DMAContinuousRequests = ENABLE;
	hadcx->Init.EOCSelection = DISABLE;
	hadcx->Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	if (HAL_ADC_Init(hadcx) != HAL_OK)
	{
		Error_Handler();
	}

	hdma_adcx->Init.Channel = DMA_CHANNEL;
	hdma_adcx->Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adcx->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adcx->Init.MemInc = DMA_MINC_ENABLE;
	hdma_adcx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adcx->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adcx->Init.Mode = DMA_CIRCULAR;
	hdma_adcx->Init.Priority = DMA_PRIORITY_HIGH;
	hdma_adcx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_adcx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_adcx->Init.MemBurst = DMA_MBURST_INC4;
	hdma_adcx->Init.PeriphBurst = DMA_PBURST_INC4;
	if (HAL_DMA_Init(hdma_adcx) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_LINKDMA(hadcx,DMA_Handle,*hdma_adcx);

	HAL_NVIC_SetPriority(nvic, dmapreemptionpriority ,dmasubpriority);
	HAL_NVIC_EnableIRQ(nvic);

}


/*
 * Function Name		: ADC_Channel_Config
 * Function Description : Configure for the selected ADC regular channel
 * Function Remarks		: None
 * Function Arguments	: ADC_t						pointer to adc structure
 * 						  ADC_Channel		       	adc channel can be ADC_CHANNEL_0 to ADC_CHANNEL_18
 * 						  DMA_GPIOx		 			DMA GPIO port
 * 						  DMA_GPIO_Pin				DMA pin
 * Function Return		: None
 * Function Example		: ADC_Channel_Config(&adc1,ADC_CHANNEL_10,IP16_Analog1_PIN);
 */
void ADC_Channel_Config(ADC_t* ADC_t,uint32_t ADC_Channel, GPIO_TypeDef* DMA_GPIOx, uint16_t DMA_GPIO_Pin)
{

	GPIOPinsInit (DMA_GPIOx, DMA_GPIO_Pin, GPIO_MODE_ANALOG, GPIO_SPEED_FREQ_VERY_HIGH,  GPIO_NOPULL);

	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_Channel;
	sConfig.Rank = (ADC_t->ADC_rank)++;
	sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES ;
	if (HAL_ADC_ConfigChannel(ADC_t->hadcx, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start_DMA(ADC_t->hadcx, (uint32_t *)ADC_t->ADC_valuep, ADC_t->ADC_number);

}

/*
 * Function Name		: DMA2_Stream0_IRQHandler
 * Function Description : DMA2_Stream0 event interrupt handler.
 * Function Remarks		: None
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: None
 */
void DMA2_Stream0_IRQHandler(void)
{

	HAL_DMA_IRQHandler(&hdma_adc1);
	HAL_DMA_IRQHandler(&hdma_adc2);
	HAL_DMA_IRQHandler(&hdma_adc3);

}

/*
 * Function Name		: DMA2_Stream1_IRQHandler
 * Function Description : DMA2_Stream1 event interrupt handler.
 * Function Remarks		: None
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: None
 */
void DMA2_Stream1_IRQHandler(void)
{

	HAL_DMA_IRQHandler(&hdma_adc1);
	HAL_DMA_IRQHandler(&hdma_adc2);
	HAL_DMA_IRQHandler(&hdma_adc3);

}



