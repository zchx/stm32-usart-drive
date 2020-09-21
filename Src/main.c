#include "main.h"
#include "usart.h"


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemInitial(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  
  /* Set Interrupt Group Priority */
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  
  /* NOJTAG: JTAG-DP Disabled and SW-DP Enabled */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();
  
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    while(1);  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  
  /* Update SystemCoreClock variable */
  SystemCoreClockUpdate();
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint8_t buf[64];
  uint32_t readLen;
  
  SystemInitial();
    
  uartOpen(0, 115200);
  uartOpen(1, 115200);
  uartOpen(2, 115200);

  uartWriteStr(0, "uart1 test\r\n");
  uartWriteStr(1, "uart2 test\r\n");
  uartWriteStr(2, "uart3 test\r\n");
  
  while (1)
  {
    readLen = uartRead(0, buf, sizeof(buf));
    if(readLen > 0)
    {
      uartWrite(0, buf, readLen);
    }
    
    readLen = uartRead(1, buf, sizeof(buf));
    if(readLen > 0)
    {
      uartWrite(1, buf, readLen);
    }
    
    readLen = uartRead(2, buf, sizeof(buf));
    if(readLen > 0)
    { 
      uartWrite(2, buf, readLen);
    }
  }
}

