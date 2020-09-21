#include "stm32f103xe.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_usart.h"
#include <string.h>

#define USART_NUM    3

#define USART1_TX_BUFFER_SIZE     64
#define USART1_RX_BUFFER_SIZE     64

#define USART2_TX_BUFFER_SIZE     100
#define USART2_RX_BUFFER_SIZE     100

#define USART3_TX_BUFFER_SIZE     256
#define USART3_RX_BUFFER_SIZE     256

#define TxRxBUF_SET(ctl, txbuf, rxbuf)    do{\
                                            (ctl)->pTxBuf = txbuf;\
                                            (ctl)->TxBufSize = sizeof(txbuf);\
                                            (ctl)->pRxBuf = rxbuf;\
                                            (ctl)->RxBufSize = sizeof(rxbuf);\
                                          }while(0)

/* Private Structure Definitions */                                          
typedef const struct
{
  USART_TypeDef *USARTx;
  DMA_TypeDef *DMAx;
  uint32_t TxDMAChannel;
  uint32_t RxDMAChannel;
  IRQn_Type IRQn;
}MCU_USART_RESOURCES;


typedef struct
{
  uint32_t TxBufSize;
  uint32_t RxBufSize;
  volatile uint32_t  TxLock;
  volatile uint32_t  RxLen;
  uint8_t *pTxBuf;
  uint8_t *pRxBuf;
}USART_CTRL;


static MCU_USART_RESOURCES USART_Resources[USART_NUM] = 
{
  {USART1, DMA1, LL_DMA_CHANNEL_4, LL_DMA_CHANNEL_5, USART1_IRQn},
  {USART2, DMA1, LL_DMA_CHANNEL_7, LL_DMA_CHANNEL_6, USART2_IRQn},
  {USART3, DMA1, LL_DMA_CHANNEL_2, LL_DMA_CHANNEL_3, USART3_IRQn},
};

static USART_CTRL UartCtrl[USART_NUM];

/*Define Tx Rx Buffer */
static uint8_t Tx1Buffer[USART1_TX_BUFFER_SIZE];
static uint8_t Rx1Buffer[USART1_RX_BUFFER_SIZE];

static uint8_t Tx2Buffer[USART2_TX_BUFFER_SIZE];
static uint8_t Rx2Buffer[USART2_RX_BUFFER_SIZE];

static uint8_t Tx3Buffer[USART3_TX_BUFFER_SIZE];
static uint8_t Rx3Buffer[USART3_RX_BUFFER_SIZE];

/* Prototypes */
static void uartStartRx(uint32_t uartChan);
static void uartISR(uint32_t uartChan);


void uartOpen(uint32_t uartChan, uint32_t baudrate)
{
  LL_USART_InitTypeDef USART_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  USART_TypeDef *pUSART;
  DMA_TypeDef *uartDMA;
  IRQn_Type uartIRQn;
  uint32_t priority;
  uint32_t txDmaChannel, rxDmaChannel;

  if(uartChan >= USART_NUM)
    return;

  pUSART = USART_Resources[uartChan].USARTx;
  uartDMA = USART_Resources[uartChan].DMAx;
  txDmaChannel = USART_Resources[uartChan].TxDMAChannel;
  rxDmaChannel = USART_Resources[uartChan].RxDMAChannel;
  uartIRQn = USART_Resources[uartChan].IRQn;

  USART_InitStruct.BaudRate = baudrate;  
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

  GPIO_InitStruct.Speed =  LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;

  switch(uartChan)
  {
    case 0:      /* USART1: Use PA9 as Tx Pin, PA10 as Rx Pin */
      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

      GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
      GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
      LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
      GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
      LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
      TxRxBUF_SET(&UartCtrl[uartChan], Tx1Buffer, Rx1Buffer);
      priority = NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0);
    break;
    
    case 1:      /* USART2: Use PA2 as Tx Pin, PA3 as Rx Pin */
      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

      GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
      GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
      LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
      GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
      LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      TxRxBUF_SET(&UartCtrl[uartChan], Tx2Buffer, Rx2Buffer);
      priority = NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0);
    break;

    case 2:      /* USART3: Use PB10 as Tx Pin, PB11 as Rx Pin */
      LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

      GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
      GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
      LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
      GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
      LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      TxRxBUF_SET(&UartCtrl[uartChan], Tx3Buffer, Rx3Buffer);
      priority = NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0);
    break;
  }
  
  LL_USART_Disable(pUSART);
  LL_USART_Init(pUSART, &USART_InitStruct);
  LL_USART_Enable(pUSART);

  /* Configure the DMA functional parameters for transmission */
  LL_DMA_ConfigTransfer(uartDMA, txDmaChannel,
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
                        LL_DMA_PRIORITY_HIGH              |
                        LL_DMA_MODE_NORMAL                |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_BYTE            |
                        LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_ConfigTransfer(uartDMA, rxDmaChannel,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                        LL_DMA_PRIORITY_HIGH              |
                        LL_DMA_MODE_NORMAL                |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_BYTE            |
                        LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_SetPeriphAddress(uartDMA, txDmaChannel, LL_USART_DMA_GetRegAddr(pUSART));
  LL_DMA_SetPeriphAddress(uartDMA, rxDmaChannel, LL_USART_DMA_GetRegAddr(pUSART));
  
  NVIC_SetPriority(uartIRQn, priority);
  NVIC_EnableIRQ(uartIRQn);
  LL_USART_EnableIT_TC(pUSART);
  LL_DMA_DisableChannel(uartDMA, txDmaChannel);
  uartStartRx(uartChan);
}


void uartClose(uint32_t uartChan)
{
  USART_TypeDef *pUSART;
  DMA_TypeDef *uartDMA;
  IRQn_Type uartIRQn;
  uint32_t txDmaChannel, rxDmaChannel;

  if(uartChan >= USART_NUM)
    return;

  pUSART = USART_Resources[uartChan].USARTx;
  uartDMA = USART_Resources[uartChan].DMAx;
  txDmaChannel = USART_Resources[uartChan].TxDMAChannel;
  rxDmaChannel = USART_Resources[uartChan].RxDMAChannel;
  uartIRQn = USART_Resources[uartChan].IRQn;

  LL_DMA_DisableChannel(uartDMA, txDmaChannel);
  LL_DMA_DisableChannel(uartDMA, rxDmaChannel);

  NVIC_DisableIRQ(uartIRQn);
  LL_USART_DeInit(pUSART);
}


uint32_t uartWrite(uint32_t uartChan, uint8_t *pData, uint32_t dataLen)
{
  USART_TypeDef *pUSART;
  DMA_TypeDef *txDMA;
  uint32_t txDmaChannel;
  uint32_t writeLen;

  if(uartChan >= USART_NUM)
    return 0;

  while(UartCtrl[uartChan].TxLock != 0){};
  UartCtrl[uartChan].TxLock = 1;

  writeLen = (dataLen <= UartCtrl[uartChan].TxBufSize) ? dataLen : UartCtrl[uartChan].TxBufSize;
  memcpy(UartCtrl[uartChan].pTxBuf, pData, writeLen);

  pUSART = USART_Resources[uartChan].USARTx;
  txDMA = USART_Resources[uartChan].DMAx;
  txDmaChannel = USART_Resources[uartChan].TxDMAChannel;

  LL_DMA_DisableChannel(txDMA, txDmaChannel);
  LL_DMA_SetMemoryAddress(txDMA, txDmaChannel, (uint32_t)UartCtrl[uartChan].pTxBuf);
  LL_DMA_SetDataLength(txDMA, txDmaChannel, writeLen);

//  LL_USART_EnableIT_TC(pUSART);
  LL_USART_EnableDMAReq_TX(pUSART);
  LL_DMA_EnableChannel(txDMA, txDmaChannel);

  return writeLen;
}


uint32_t uartWriteStr(uint32_t uartChan, char *str)
{
  return uartWrite(uartChan, (uint8_t *)str, strlen(str));
}


uint32_t uartRead(uint32_t uartChan, uint8_t *pBuf, uint32_t bufSize)
{
  uint32_t readLen = 0;

  if(uartChan >= USART_NUM || pBuf == NULL || bufSize == 0)
    return 0;
 
  if(UartCtrl[uartChan].RxLen > 0)
  {
    readLen = UartCtrl[uartChan].RxLen < bufSize ? UartCtrl[uartChan].RxLen : bufSize;
    memcpy(pBuf, UartCtrl[uartChan].pRxBuf, readLen);
    UartCtrl[uartChan].RxLen = 0;
    uartStartRx(uartChan);
  }

  return readLen;
}


static void uartStartRx(uint32_t uartChan)
{
  USART_TypeDef  *pUSART;
  DMA_TypeDef *uartDMA;
  uint32_t rxDmaChannel;

  if(uartChan >= USART_NUM)
     return;

  pUSART = USART_Resources[uartChan].USARTx;
  uartDMA = USART_Resources[uartChan].DMAx;
  rxDmaChannel = USART_Resources[uartChan].RxDMAChannel;
  
  LL_DMA_DisableChannel(uartDMA, rxDmaChannel);
  LL_DMA_SetDataLength(uartDMA, rxDmaChannel, 1);
  LL_DMA_EnableChannel(uartDMA, rxDmaChannel);
  
  LL_DMA_DisableChannel(uartDMA, rxDmaChannel);
  LL_DMA_SetMemoryAddress(uartDMA, rxDmaChannel, (uint32_t)UartCtrl[uartChan].pRxBuf);  
  LL_DMA_SetDataLength(uartDMA, rxDmaChannel, UartCtrl[uartChan].RxBufSize);

  LL_USART_EnableDMAReq_RX(pUSART);
  LL_DMA_EnableChannel(uartDMA, rxDmaChannel);

  LL_USART_DisableIT_RXNE(pUSART);
  LL_USART_EnableIT_IDLE(pUSART);
}


static void uartISR(uint32_t uartChan)
{
  USART_TypeDef *pUSART = USART_Resources[uartChan].USARTx ;
  DMA_TypeDef *uartDMA = USART_Resources[uartChan].DMAx;
  uint32_t txDmaChannel, rxDmaChannel;

  if(LL_USART_IsActiveFlag_TC(pUSART))
  {
    txDmaChannel = USART_Resources[uartChan].TxDMAChannel;
    LL_USART_ClearFlag_TC(pUSART);
//  LL_USART_DisableIT_TC(pUSART);
    LL_DMA_DisableChannel(uartDMA, txDmaChannel);
    UartCtrl[uartChan].TxLock = 0;
  }
  
  if(LL_USART_IsActiveFlag_IDLE(pUSART))
  {
    rxDmaChannel = USART_Resources[uartChan].RxDMAChannel;
    LL_USART_ClearFlag_IDLE(pUSART);
    LL_USART_DisableIT_IDLE(pUSART);
//  LL_DMA_DisableChannel(uartDMA, rxDmaChannel); 
    UartCtrl[uartChan].RxLen = UartCtrl[uartChan].RxBufSize - LL_DMA_GetDataLength(uartDMA, rxDmaChannel);
  }
}


void USART1_IRQHandler(void)
{
  uartISR(0);
}


void USART2_IRQHandler(void)
{
  uartISR(1);
}


void USART3_IRQHandler(void)
{
  uartISR(2);
}
