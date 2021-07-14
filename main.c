#include<main.h>
#include<stdarg.h>
void SystemInit();
void Uart1_config();
void uart_printf(const char *fmt, ...);
void delay(int val);
void Gpio_Enable();
void USART1_IRQHandler()
{
        /* ISR */
        uint8_t rx;
	rx=USART1->DR;
	if(rx==0x78)
	{
		GPIOB->ODR ^=1<<3;
	}
}
void SystemInit()
{
    RCC->CR|=RCC_HSION;
    RCC->APB1ENR|=RCC_PWREN;
    PWR->CR|=PWR_VOS;
    RCC->CFGR|=RCC_SW;
}
char buf[512];
void uart_printf(const char *fmt, ...)
{
	int ret;
	va_list args;
	va_start(args, fmt);
	ret = vsnprintf(buf, 512, fmt, args);
	va_end(args);
	if (ret > 0)
	{
		for (int i =0; i < ret; i++)
		{
			USART1->SR = 0;
			USART1->DR = buf[i];
			while(!(USART1->SR & (1<<6)) );
		}
	}
}
void Uart1_config()
{
	RCC->APB2ENR |=USART1_CLK;
	RCC->AHB1ENR |=1;
	GPIOA->MODER |=2<<18;/* Enabling Mode of PA9 */
	GPIOA->MODER |=2<<20;/*Enabling Mode of PA9 */
	GPIOA->OSPEEDR |=3<<18;/* Enabling High speed  of PA9*/
	GPIOA->OSPEEDR |=3<<20;/* Enabling High speed of PA10*/
	GPIOA->AFR[1]  |=7<<4;/*Enabling AFR7 of PA9 */
        GPIOA->AFR[1]  |=7<<8;/*Enabling AFR8 of PA10 */
	USART1->CR1 |=USART1_EN;
        USART1->BRR =((3<<0) | (0x68<<4));  /*9600 Baud Rate*/
	USART1->CR1 |=USART1_RE;
	USART1->CR1 |=USART1_TE;
	USART1->CR1 |=1<<5;/*RXNEIE: RXNE interrupt enable*/
        NVIC->ISER1|=1<<5;/*ENABLE USART1 IRQ  IS 37NUM*/
}
int main()
{
   SystemInit();
   Uart1_config();
   Gpio_Enable();
   GPIOB->ODR |=(1<<3);/*Enable PB3 */
   while(1)
   {
      /*wait for Interrupt*/
   }
}
void delay(int val)
{
	RCC->APB1ENR |= 2;      //ENABLE TIMER 3
	TIM3->PSC=160;
	TIM3->EGR=1;/*Update generation in TIM3_EGR */
	TIM3->CR1=1;/*Enable Timer 3*/
	TIM3->CNT=0;
	while(TIM3->CNT < val);
	TIM3->SR=0;
}
