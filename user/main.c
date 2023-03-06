#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "misc.h"
#include "configure.h"
#include "math.h"
#include "string.h"

int predictedRSSI;
int RSSI;
int d;

int initialized = 0;
float processNoise= 0.005;
int measurementNoise= 20;
int predictedRSSI = 0;
float errorCovariance = 0;

int x;

int filtering(int rssi){
  int priorRSSI;
  float priorErrorCovariance;
  
  if(initialized == 0) {
    initialized = 1;
    priorRSSI = rssi;
    priorErrorCovariance = 1;
  }
  else {
    priorRSSI = predictedRSSI;
    priorErrorCovariance = errorCovariance + processNoise;
  }
  
  float kalmanGain = priorErrorCovariance/ (priorErrorCovariance + measurementNoise);
  predictedRSSI = priorRSSI + (kalmanGain * (rssi - priorRSSI));
  errorCovariance = (1 - kalmanGain) * priorErrorCovariance;
  
  return predictedRSSI;
}

void USART1_IRQHandler() {
    uint16_t word;

    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
        word = USART_ReceiveData(USART1);
        USART_SendData(USART2, word);
    }
    
    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}

int temp = 0;
int RSSI_Index= 0;

enum DIRECTION {
  LEFT,
  FORWARD,
  RIGHT
};

uint32_t servo_index = FORWARD;
int rssi_array[5] = {44, 54, 42, 56, 42}; 
unsigned array_index;


int minmax() {
  int i;
  int min = 100000 , max = 0;
  int total = 0;
  for(i = 0; i < 5; ++i)
  {
    if( rssi_array[i] < min) min = rssi_array[i];
    if( rssi_array [i] > max) max = rssi_array[i];
    total  += rssi_array[i];
  }
  
  
  
  return total / 5;
}

void USART2_IRQHandler() { 
  uint16_t word;
  
  if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
    
    word = USART_ReceiveData(USART2);
    USART_SendData(USART1, word);

    
    if (word >= 0x30 &&  word <= 0x39) {
      temp = temp * 10 + (word - '0');
    }
    
    if (word ==  0x22) {
      
      int t = temp;
      int count = 0;
      while(t) {
          t /= 10;
          count++;
      }

      if (count == 2){ 
        //predictedRSSI = filtering(temp);
        RSSI = temp;
        rssi_array[array_index++ % 5]  = temp;
        d = minmax(); 
      }
      temp = 0;
      
    }
    
    if (word == 0x4C){ //L
      servo_index = LEFT;
    }
     if (word == 0x52){//R
      servo_index = RIGHT;
    }
     if (word == 0x46){ //F
      servo_index = FORWARD;
    }
    
  }

  USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}

void Delay(int time)
{
    int i;
    for (i = 0; i < time; i++) {}        
}

int32_t temp_CCR;
const int speed_threshold[] = {0, 4300, 5700, 8000};
const int speed_interval[] =  {100, 400, 600, 1000};
enum SPEED {STOP, SLOW, MEDIUM, FAST};

uint32_t limit_index = STOP;
uint32_t interval_index = MEDIUM;


void change_speed_high()
{
  temp_CCR += speed_interval[interval_index];
  
  if (temp_CCR < speed_threshold[limit_index]){
    TIM3->CCR3 = temp_CCR - 1;
    TIM3->CCR4 = temp_CCR - 1;
  }
  else
    temp_CCR = speed_threshold[limit_index];
}

void change_speed_low()
{
  temp_CCR -= speed_interval[interval_index];
  if (temp_CCR > speed_threshold[limit_index]) {
    TIM3->CCR3 = temp_CCR - 1;
    TIM3->CCR4 = temp_CCR - 1;
  }
  else
    temp_CCR = speed_threshold[limit_index];
}

void change_speed()
{
  if (temp_CCR < speed_threshold[limit_index])
    change_speed_high();
  else
    change_speed_low();
}

void stop_motor()
{
  int temp_interval = speed_interval[interval_index];
  while (temp_CCR > 0)
  {
    change_speed_low();
    Delay(10);
  }
}

int servo_duty[] = {500, 600, 700};

void change_angle()
{
  TIM2->CCR2 = servo_duty[servo_index];
}

int main(void)
{
  SystemInit();
  RCC_Configure();
  GPIO_Configure();
  USART1_Init();
  USART2_Init();
  ADC1_Configure();
  DMA1_Configure();
  TIM2_Configure();
  TIM3_Configure();
  NVIC_Configure();
  
  interval_index = FAST;
  limit_index = SLOW;
  servo_index = FORWARD;
  distance[0] = 1000;
    
  while(1)
  {
    // ACTIVE PHASE
    // 장애물이 있을 때 최대속력을 0으로
    
    if(distance[0] < 1000) {
      limit_index = STOP;
    }
    else
    {
      if (d < 53.0)limit_index = SLOW;
      else if (d < 60.0) limit_index = MEDIUM;
      else limit_index = FAST;
      
    }
    change_angle();
    change_speed();
  }
  
  return 0;
}
