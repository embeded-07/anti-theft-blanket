#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "core_cm3.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void NVIC_Configure(void);
void EXTI15_10_IRQHandler(void);
void Delay(void);

//---------------------------------------------------------------------------------------------------
int flag = 1;

void RCC_Configure(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void EXTI_Configure(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  // PC10 - 기울기 센서
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void NVIC_Configure(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  // 기울기 센서
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void EXTI15_10_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10) == Bit_RESET)
    {
      flag = 0;
    }
    else
    {
      flag = 1;
    }
    EXTI_ClearITPendingBit(EXTI_Line10);
  }
}

void Delay(void)
{
  int i;
  for (i = 0; i < 2000000; i++)
  {
  }
}

int getSlopeSensorValue()
{
  return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10);
}

int main(void)
{

  SystemInit();

  RCC_Configure();

  GPIO_Configure();

  EXTI_Configure();

  NVIC_Configure();

  LCD_Init();
  Touch_Configuration();
  Touch_Adjust();
  LCD_Clear(WHITE);

  while (1)
  {
    int slopeValue = getSlopeSensorValue();
    LCD_ShowNum(60, 160, slopeValue, 10, WHITE, GRAY);
  }
  return 0;
}