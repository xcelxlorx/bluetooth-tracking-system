void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void ADC1_Configure(void);
void ADC3_Configure(void);
void DMA1_Configure(void);
void DMA2_Configure(void);
void TIM2_Configure(void);
void TIM3_Configure(void);
void NVIC_Configure(void);

extern volatile uint32_t distance[1];
extern volatile uint32_t acc[1];