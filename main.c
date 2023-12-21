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
#include "lcd.h"

/* 사용 센서 및 장비 */
/*
  - 조도 센서 (PC2)
  - 입력 버튼 (PC4)
  - 기울기 센서 (PC10)
  - LED (PD2)
  - 부저 (PD3)
  - 블루투스 모듈
*/

/* 작동 흐름 */
/*
  - 시작 버튼 입력
  -> 기울기 센서 감지
  -> 조도 센서 감지
  -> LED 및 부저 작동
  -> 블루투스 모듈 작동
  -> 종료 버튼 입력
*/

/* 프로토타입 선언 */
// 필요 포트 전압 인가 및 센서 핀 할당
void RCC_Configure(void);
void Slope_GPIO_Configure(void);
void Light_GPIO_Configure(void);
void Input_GPIO_Configure(void);
void USART_GPIO_Configure(void);

// 센서 상세 설정
void EXTI_Configure(void);
void ADC_Configure(void);

// 블루투스 통신 설정
void USART1_Init(void);
void USART2_Init(void);

// 인터럽트 및 이벤트 의존성 설정
void ADC1_2_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);

// 우선순위 설정
void Slope_NVIC_Configure(void);
void Light_NVIC_Configure(void);
void USART_NVIC_Configure(void);

// 커스텀 구현 함수
void Delay(void);
void StatusInit(void);
uint8_t getSlopeSensorValue(void);

// 시스템 부팅시 모든 세팅 값 초기화
void StartSystem(void);

/* 전역 변수 */
// IRQHandler를 통해 값 변경 후 main 함수에서 검증으로 사용
uint8_t startFlag = 0; // 시작 버튼 신호 저장: 0 || 1
uint8_t slopeFlag = 0; // 기울기 센서 임계값 초과 여부 저장: 0 || 1
uint8_t lightFlag = 0; // 조도 센서 임계값 초과 여부 저장: 0 || 1

// IRQHandler에서 실시간으로 입력받고 임계값과 비교하는 각 센서 값
uint8_t initSlopeValue = 0; // 초기 기울기 값 저장: 0 || 1
uint16_t brightValue = 0;   // 조도 센서 입력 값 저장

/* 필요 포트 전압 인가 */
void RCC_Configure(void)
{
  // Alternate Function IO clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  // USART1, USART2 TX/RX port clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  // 기울기 센서, 버튼
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  // LED, 부저
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  // 조도 센서
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  // USART1, USART2 TX/RX port clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

/* 기울기 센서 핀 할당 */
void Slope_GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* 조도 센서 핀 할당 */
void Light_GPIO_Configure()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* 입력 버튼 핀 할당 */
void Input_GPIO_Configure()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD | GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* LED, 부저 출력 핀 할당 */
void Output_GPIO_Configure()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/* USART 설정 */
void USART_GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // USART1 TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // USART1 RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD | GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // USART2 TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // USART2 RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD | GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* EXTI 설정 */
void EXTI_Configure(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  // 기울기 센서
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // 입력 버튼
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

/* 조도 센서 */
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

/* USART1 초기화 */
void USART1_Init(void)
{
  USART_InitTypeDef USART1_InitStructure;

  // Enable the USART1 peripheral
  USART_Cmd(USART1, ENABLE);

  USART1_InitStructure.USART_BaudRate = 9600;
  USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART1_InitStructure.USART_Parity = USART_Parity_No;
  USART1_InitStructure.USART_StopBits = USART_StopBits_1;
  USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1, &USART1_InitStructure);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/* USART2 초기화 */
void USART2_Init(void)
{
  USART_InitTypeDef USART2_InitStructure;

  // Enable the USART2 peripheral
  USART_Cmd(USART2, ENABLE);

  USART2_InitStructure.USART_BaudRate = 9600;
  USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART2_InitStructure.USART_Parity = USART_Parity_No;
  USART2_InitStructure.USART_StopBits = USART_StopBits_1;
  USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART2, &USART2_InitStructure);

  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

/* 기울기 센서 우선순위 할당 */
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

/* 조도 센서 우선순위 할당 */
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

/* USART 우선순위 할당 */
void USART_NVIC_Configure(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  // USART1
  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // USART2
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/* 담요 -> 스마트폰 통신 핸들링 */
void USART1_IRQHandler(void)
{
  uint16_t word;
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    word = USART_ReceiveData(USART1);
    USART_SendData(USART2, word); // 단순 전송
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}

/* 스마트폰 -> 담요 통신 핸들링 */
void USART2_IRQHandler(void)
{
  uint16_t word;
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
    word = USART_ReceiveData(USART2);
    USART_SendData(USART1, word); // 단순 전송
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}

/* 조도 센서 값 핸들링 */
void ADC1_2_IRQHandler()
{
  uint16_t threshold = 3950; // 담요 속과 밖의 밝기 차이를 테스트를 통해 설정한 임계값

  if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
  {
    brightValue = ADC_GetConversionValue(ADC1);
    // 시작 버튼 입력 & 기울기 센서 감지 & 임계값 초과인 경우 발동
    bool isLightSensorOn = startFlag && slopeFlag && (brightValue < threshold);
    if (isLightSensorOn)
    {
      lightFlag = 1; // 조도 센서도 감지 되었으므로 도난 확정 > main()
    }
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  }
}

/* 기울기 센서 값 핸들링 */
void EXTI15_10_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    // 시작 버튼 입력 & 임계값 초과인 경우 발동
    bool isSlopeSensorOn = startFlag && (getSlopeSensorValue() != initSlopeValue);
    if (isSlopeSensorOn)
    {
      slopeFlag = 1; // 기울기 센서가 감지 되었으므로 조도 센서 감지 시작 > main()
    }
    EXTI_ClearITPendingBit(EXTI_Line10);
  }
}

/* 부저 간격 딜레이 */
void Delay(void)
{
  int i;
  for (i = 0; i < 2000000; i++)
  {
  }
}

/* 기울기 센서 값 입력 */
uint8_t getSlopeSensorValue(void)
{
  return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10);
}

/* 시작 버튼 입력과 동시에 실행되어 전역 변수 초기화 */
void StatusInit(void)
{
  startFlag = 1;
  initSlopeValue = getSlopeSensorValue();
  GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
  return;
}

/* 시스템 부팅시 모든 세팅 값 초기화 */
void StartSystem(void)
{
  SystemInit();
  RCC_Configure();

  Slope_GPIO_Configure();
  Light_GPIO_Configure();
  Input_GPIO_Configure();
  Output_GPIO_Configure();
  USART_GPIO_Configure();

  ADC_Configure();
  EXTI_Configure();

  USART1_Init();
  USART2_Init();

  Slope_NVIC_Configure();
  Light_NVIC_Configure();
  USART_NVIC_Configure();

  LCD_Init();
  Touch_Configuration();
  Touch_Adjust();
  LCD_Clear(WHITE);
}

/* 센싱 및 시스템 로직 구현 */
int main(void)
{
  StartSystem();

  /* 로직 개요 */
  /*
    시작 버튼 입력까지 대기
    -> 시작 버튼 입력시 IRQHandler를 통해 기울기 센서 작동 시작
    -> 도둑이 담요를 들추는 경우
    -> 기울기 센서가 움직임 감지시 IRQHandler를 통해 조도 센서 작동 시작
    -> 조도 센서가 빛 감지시 도난 확정으로 인식
    -> LED, 부저 작동
    -> 블루투스를 통해 사용자 휴대폰에 알림 전송
    -> 사용자가 돌아와서 종료 버튼 입력시 모두 종료
  */
  while (1)
  {
    // 시작 버튼 입력 전까지 대기
    while (startFlag != 1)
    {
      startFlag = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4);
      LCD_ShowString(0, 0, "...", BLACK, WHITE);
      // 시작 버튼 입력시 전역 변수 초기화
      if (startFlag == 1)
      {
        StatusInit();
        LCD_Clear(WHITE);
        break;
      }
    }

    /* 센싱 및 시스템 로직 */
    // IRQHandler에서 기울기 센서 움직임 감지
    // -> IRQHandler에서 전역 변수 변경으로 조도 센서 센싱 시작
    if (slopeFlag == 1)
    {
      //? 이거 없어도 될 것 같은데
      ADC_SoftwareStartConvCmd(ADC1, ENABLE);
      LCD_Clear(WHITE);
      LCD_ShowString(0, 0, "DANGER???", RED, WHITE);
    }
    // IRQHandler에서 조도 센서 변화 감지
    // -> 부저 작동 및 블루투스 전송
    else if (lightFlag == 1)
    {
      // LED 및 부저 작동
      GPIO_SetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
      Delay();
      GPIO_ResetBits(GPIOD, GPIO_Pin_3);
      LCD_Clear(WHITE);
      LCD_ShowString(0, 0, "DANGER!!!", RED, WHITE);

      // 블루투스 통신을 통해 사용자 휴대폰으로 경고 메시지 전송
      char str[] = "STOLEN!";
      for (int i = 0; str[i] != '\0'; i++)
      {
        USART_SendData(USART2, str[i]);
      }

      // 도난 감지 후 사용자가 돌아와서 버튼 누를 때까지 대기
      while (startFlag != 0)
      {
        // 사용자가 다시 버튼 누르면 종료 후 초기화
        startFlag = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4);
        if (startFlag == 0)
        {
          // 전역 변수 초기화
          GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3);
          slopeFlag = 0;
          lightFlag = 0;
          break;
        }
      }
    }
    // 평상시
    else
    {
      LCD_Clear(WHITE);
      LCD_ShowString(0, 0, "START!!", BLACK, WHITE);
    }
  }
  return 0;
}