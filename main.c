#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "core_cm3.h"
#include "misc.h"
#include "touch.h"

/* function prototype */
void RCC_Configure(void);
void Slope_GPIO_Configure(void);
void Light_GPIO_Configure(void);
void EXTI_Configure(void);
void ADC_Configure(void);
void ADC1_2_IRQHandler(void);
void Slope_NVIC_Configure(void);
void Light_NVIC_Configure(void);
void EXTI15_10_IRQHandler(void);
void Delay(void);

//---------------------------------------------------------------------------------------------------
uint8_t initStatus = 0;
uint8_t slopeFlag = 0;
uint8_t lightFlag = 0;

uint8_t startSignal = 0;
uint16_t brightValue = 0;

void RCC_Configure(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  // PC10 - 기울기 센서
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  // 조도 센서
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  // LED, 부저
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
}

// 기울기 센서 초기화 - PC10
void Slope_GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// 조도 센서 초기화 - PC2
void Light_GPIO_Configure()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Output_GPIO_Configure()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
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

// 조도 센서
void ADC_Configure(void)
{
  ADC_InitTypeDef ADC_12;
  ADC_12.ADC_ContinuousConvMode = ENABLE;
  ADC_12.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_12.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_12.ADC_Mode = ADC_Mode_Independent;
  ADC_12.ADC_NbrOfChannel = 1;
  ADC_12.ADC_ScanConvMode = DISABLE;

  ADC_Init(ADC1, &ADC_12);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5);
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1))
    ;
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1))
    ;
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// 기울기 센서
void Slope_NVIC_Configure(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

// 조도 센서
void Light_NVIC_Configure()
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_EnableIRQ(ADC1_2_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
}

// 조도 센서 값 핸들링
void ADC1_2_IRQHandler()
{
  uint16_t threshold = 3950;

  if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
  {
    brightValue = ADC_GetConversionValue(ADC1);
    if (startSignal && slopeFlag && (brightValue < threshold))
    {
      lightFlag = 1;
    }
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  }
}

// 기울기 센서 값 핸들링
void EXTI15_10_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    if (startSignal && (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10) != initStatus))
    {
      slopeFlag = 1;
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

void clickStartButton()
{
  startSignal = 1;
  initStatus = getSlopeSensorValue();
  GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
  return;
}

int main(void)
{
  SystemInit();
  RCC_Configure();
  Slope_GPIO_Configure();
  Light_GPIO_Configure();
  ADC_Configure();
  EXTI_Configure();
  Slope_NVIC_Configure();
  Light_NVIC_Configure();
  Output_GPIO_Configure();

  // 시작버튼이라 가정
  clickStartButton();
  while (1)
  {

    // 조도 센서 밝기 감지
    if (lightFlag == 1)
    {
      GPIO_SetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
      Delay();
      GPIO_ResetBits(GPIOD, GPIO_Pin_3);
    }
    // 기울기 센서 변화 감지
    else if (slopeFlag == 1)
    {
      // 조도 센서 값 읽기
      ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    }
    // 평상시
    else
    {
    }
  }
  return 0;
}
