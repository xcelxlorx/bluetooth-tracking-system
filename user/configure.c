#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "misc.h"
#include "configure.h"

volatile uint32_t distance[1];
volatile uint32_t acc[1];
void RCC_Configure(void)
{
  // TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'
  /* UART TX/RX port clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
 /* UART1 pin setting */
  //TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* UART2 pin setting */
  //TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* TIM2 setting */
  // CH2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* TIM3 setting */
  // CH3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // CH4
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* relay setting */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* ADC1 setting */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void USART1_Init(void)
{
  USART_InitTypeDef USART1_InitStructure;

  // Enable the USART1 peripheral
  USART_Cmd(USART1, ENABLE);
  
  // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
  USART1_InitStructure.USART_BaudRate = 9600;
  USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART1_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART1_InitStructure.USART_Parity = USART_Parity_No;
  USART1_InitStructure.USART_StopBits = USART_StopBits_1;
  USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
  
  USART_Init(USART1, &USART1_InitStructure);
  
  // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	
}

void USART2_Init(void)
{
  USART_InitTypeDef USART2_InitStructure;

  // Enable the USART2 peripheral
  USART_Cmd(USART2, ENABLE);
  
  // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
  USART2_InitStructure.USART_BaudRate = 9600;
  USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART2_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART2_InitStructure.USART_Parity = USART_Parity_No;
  USART2_InitStructure.USART_StopBits = USART_StopBits_1;
  USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
  
  USART_Init(USART2, &USART2_InitStructure);
  
  // TODO: Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}
// microwave sensor config
void ADC1_Configure(void){
  ADC_InitTypeDef ADC;
  ADC.ADC_Mode = ADC_Mode_Independent;
  ADC.ADC_ScanConvMode = DISABLE;
  ADC.ADC_ContinuousConvMode = ENABLE;
  ADC.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC.ADC_DataAlign = ADC_DataAlign_Right;
  ADC.ADC_NbrOfChannel = 1;
  
  ADC_Init(ADC1, &ADC);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12,1, ADC_SampleTime_239Cycles5);
  //ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  
  ADC_DMACmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  
  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
  ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}
// microwave sensor config
void DMA1_Configure(void){
  DMA_InitTypeDef DMA_Instructure;
  DMA_DeInit(DMA1_Channel1);
  DMA_Instructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)distance;
  DMA_Instructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Instructure.DMA_Priority = DMA_Priority_High;
  DMA_Instructure.DMA_BufferSize = 1;
  DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_Instructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_Init(DMA1_Channel1, &DMA_Instructure);
  DMA_Cmd(DMA1_Channel1, ENABLE); 
}

// acc sensor config
void ADC3_Configure(void){
  ADC_InitTypeDef ADC;
  ADC.ADC_Mode = ADC_Mode_Independent;
  ADC.ADC_ScanConvMode = DISABLE;
  ADC.ADC_ContinuousConvMode = ENABLE;
  ADC.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC.ADC_DataAlign = ADC_DataAlign_Right;
  ADC.ADC_NbrOfChannel = 1;
  ADC_Init(ADC3, &ADC);
  
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12,1, ADC_SampleTime_239Cycles5);
  ADC_DMACmd(ADC3, ENABLE);
  ADC_Cmd(ADC3, ENABLE);
  
  ADC_ResetCalibration(ADC3);
  while (ADC_GetResetCalibrationStatus(ADC3));
  ADC_StartCalibration(ADC3);
  while(ADC_GetCalibrationStatus(ADC3));
  ADC_SoftwareStartConvCmd(ADC3,ENABLE);
}

// acc sensor config
void DMA2_Configure(void){
  DMA_InitTypeDef DMA_Instructure;
  DMA_DeInit(DMA2_Channel5);
  DMA_Instructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC3->DR;
  DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)acc;
  DMA_Instructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Instructure.DMA_Priority = DMA_Priority_High;
  DMA_Instructure.DMA_BufferSize = 1;
  DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_Instructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_Init(DMA2_Channel5, &DMA_Instructure);
  DMA_Cmd(DMA2_Channel5, ENABLE); 
}
// servo motor config
void TIM2_Configure(void)
{
  printf("Setup\n");
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  TIM_TimeBaseStructure.TIM_Period = 10000; 
  TIM_TimeBaseStructure.TIM_Prescaler = 7200 / 50; // 10ms == 100Hz = 10000us
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 600; // us
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
  
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
}

// motor config
void TIM3_Configure(void) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  TIM_TimeBaseStructure.TIM_Period = 10000; 
  TIM_TimeBaseStructure.TIM_Prescaler = 7200 / 100; // 10ms
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; // us
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; // us
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);
  
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}

void NVIC_Configure(void) {
  NVIC_InitTypeDef NVIC_InitStructure_UART1;
  NVIC_InitTypeDef NVIC_InitStructure_UART2;
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  // UART1
  // 'NVIC_EnableIRQ' is only required for USART setting
  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_InitStructure_UART1.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure_UART1.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure_UART1.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure_UART1.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_UART1);
 
  // UART2
  // 'NVIC_EnableIRQ' is only required for USART setting
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_InitStructure_UART2.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure_UART2.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure_UART2.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure_UART2.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_UART2);
}