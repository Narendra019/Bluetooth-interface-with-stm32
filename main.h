#ifndef __main_H
#define __main_H
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */
#include <stdio.h>

typedef struct
{
	__IO uint32_t ISER0;  /*ISER0 , Address offset 0x00 */
	__IO uint32_t ISER1;  /*ISER1 , Address offset 0x04 */
}NVIC_Typedef;

typedef struct
{
	__IO uint32_t SR;  /*Status register , Address offset 0x00 */
	__IO uint32_t DR;  /*Data Register , Address offset  0x04*/
	__IO uint32_t BRR; /*Baud rate register ,Address offset 0x08 */
	__IO uint32_t CR1; /*Control register 1,Address offset 0x0C */
	__IO uint32_t CR2; /*Control register 2,Address offset 0x10 */
	__IO uint32_t CR3; /*Control register 3,Address offset 0x14 */
	__IO uint32_t GTPR; /*Guard time and prescaler register,Address offset 0x18 */
}USART_Typedef;

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

typedef struct
{
	__IO uint32_t CR1;/*!<SPI control register 1,       Address offset: 0x00     */
	__IO uint32_t CR2;/*!<SPI control register 2,       Address offset: 0x04    */
	__IO uint32_t SR1;/*!<SPI status  register 1,       Address offset: 0x08    */
	__IO uint32_t DR1;/*!<SPI Data  register ,          Address offset: 0x0c    */
	__IO uint32_t CRC;/*!<SPI CRC polynomial register,  Address offset: 0x10   */
	__IO uint32_t RX_CRC;/*!<SPI RX CRC polynomial register,  Address offset: 0x14   */
	__IO uint32_t TX_CRC;/*!<SPI TX CRC polynomial register,  Address offset: 0x18   */
	__IO uint32_t I2SCFGR;/*!<SPI I2S Configuration register, Address offset: 0x1C   */
	__IO uint32_t I2SPR;/*!SPI_I2S prescaler register,        Address offset:0x20 */
}SPI_TypeDef;

typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  __IO uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  uint32_t      RESERVED7[1];  /*!< Reserved, 0x88                                                                    */
  __IO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
} RCC_TypeDef;

typedef struct
{
  __IO uint32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
  __IO uint32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;

typedef struct
{
   __IO uint32_t CR1;
   __IO uint32_t CR2;
   __IO uint32_t OAR1;
   __IO uint32_t OAR2;
   __IO uint32_t DR;
   __IO uint32_t SR1;
   __IO uint32_t SR2;
   __IO uint32_t CCR;
   __IO uint32_t TRISE;
   __IO uint32_t FLTR;
}I2C_Typedef;
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;

#define PERIPH_BASE           0x40000000UL
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)

#define SPI1_BASE          (APB2PERIPH_BASE + 0x3000UL)
#define SPI2_BASE          (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE          (APB1PERIPH_BASE + 0x3C00UL)
#define SPI4_BASE          (APB2PERIPH_BASE + 0x3400UL)

#define TIM1_BASE           (APB2PERIPH_BASE + 0X0000UL)
#define TIM2_BASE           (PERIPH_BASE +  0X0000UL)
#define TIM3_BASE           (PERIPH_BASE +  0X0400UL)
#define TIM4_BASE           (PERIPH_BASE +  0X0800UL)
#define TIM5_BASE           (PERIPH_BASE +  0X0C00UL)

#define I2C1_BASE           (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE           (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE           (APB1PERIPH_BASE + 0x5C00UL)

#define SPI1                 ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                 ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                 ((SPI_TypeDef *) SPI3_BASE)
#define SPI4                 ((SPI_TypeDef *) SPI4_BASE)

#define I2C1                ((I2C_Typedef *) I2C1_BASE)
#define I2C2                ((I2C_Typedef *) I2C2_BASE)
#define I3C2                ((I2C_Typedef *) I2C3_BASE)


#define USART1_BASE         (APB2PERIPH_BASE + 0x1000UL)

#define USART1           ((USART_Typedef*) USART1_BASE)
#define TIM1             ((TIM_TypeDef*) TIM1_BASE)
#define TIM2             ((TIM_TypeDef*) TIM2_BASE)
#define TIM3             ((TIM_TypeDef*) TIM3_BASE)
#define TIM4             ((TIM_TypeDef*) TIM4_BASE)
#define TIM5             ((TIM_TypeDef*) TIM5_BASE)
#define RCC              ((RCC_TypeDef *) RCC_BASE)

#define GPIO_PIN_MASK              0x0000FFFFU /* PIN mask for assert test */
#define IS_GPIO_PIN(PIN)        (((((uint32_t)PIN) & GPIO_PIN_MASK ) != 0x00U) && ((((uint32_t)PIN) & ~GPIO_PIN_MASK) == 0x00U))
#define IS_GPIO_PIN_ACTION(ACTION) (((ACTION) == GPIO_PIN_RESET) || ((ACTION) == GPIO_PIN_SET))

#define NVIC_BASE_ADDRESS       0xE000E100
#define NVIC                    ((NVIC_Typedef *) NVIC_BASE_ADDRESS)

#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define RCC_HSEON      1U<<16
#define RCC_HSION      1U<<0
#define RCC_PWREN      1U<<28
#define RCC_HPRE       8U<<4
#define RCC_PPRE1      4U<<10
#define RCC_PPRE2      0U
#define RCC_PLLSRC     1U<<22
#define RCC_PLLON      1U<<24
#define RCC_SW         0U
#define PWR_VOS        2U<<14
#define TIM2_CC1IE     1U<<1
#define TIM2_CCIS      1U
#define TIM2_IC1PSC    0U
#define TIM2_CC1P      0U<<1/* Output polarity in Capture Compare*/
#define TIM2_CC1NP     0U<<2
#define TIM2_CC1E      1

#define USART1_CLK     1<<4  /* USART1 CLK ENABLE */
#define USART1_EN      1<<13 /* USART1 Enable */
#define USART1_RE      1<<2  /* Recieve Enable */
#define USART1_TE      1<<3  /*Transmit Enable */
#define USART1_PE      1<<10
#define USART1_ODD     1<<9
#endif
