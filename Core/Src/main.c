
#include <stdint.h>
#include "main.h"
#include <string.h>
unsigned int tick = 0;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
int cnt =0;
char rx_buf[32];
float vin =0;
float temp =0;
uint32_t* SYST_CR = (uint32_t*)(0xe000e10);// 0xE000E010 SYST_CSR RW trang 279/276 reference manual arm m4
uint32_t* SYST_RV = (uint32_t*)(0xe000e14);
void delay_init()
{
	/* int systick */
	*SYST_RV = 16000/8 -1; // set reload value qua bộ chia 8 ; (2000-1), time là 2000
	*SYST_CR |= 1| (1<<1); // cho bit so 0 =1, bit số 1 bằng 1, khi sư kiện ngắt xảy ra thì nhảy lên bảng vector table
}
void delay(unsigned int time)
{
	while (tick < time);
//	*SYST_CR &= ~1;
	tick = 0; // ban đầu biến tick bằng 0 ; thường delay thì sử dụng sýstem tick, HOAC TIMER
}

/*void SysTick_Handler()
{
	tick++; // mỗi 1 milli giây thì từ động nhảy lên một
	*SYST_CR &= ~(1<<16);
}*/

#define RCC_ADDR_BASE 0x40023800
#define GPIOD_ADDR_BASE 0x40020c00
void LED_Init()
{
	uint32_t* RCC_AHB1ENR  = (uint32_t*)(RCC_ADDR_BASE + 0x30);
	*RCC_AHB1ENR |= (1<<3);
	uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_ADDR_BASE + 0x00);
	*GPIOD_MODER |= (0b01 << 24) | (0b01 << 26) | (0b01 << 28) | (0b01 << 30);
}
typedef enum
{
	OFF,
	ON
}Led_state_t;
void LED_ctrl(int led_num, Led_state_t state)
{
	uint32_t* GPIOD_ODR  = (uint32_t*)(GPIOD_ADDR_BASE + 0x14);
	if(state == OFF)
		*GPIOD_ODR &= ~(1<<(12 + led_num));
	else
		*GPIOD_ODR |= (1<<(12 + led_num));
}
#define GPIOD_BASE_ADDRESS 0x40020C00
#define GPIOA_BASE_ADDRESS 0x40020000
void button_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER  = (uint32_t*)(GPIOA_BASE_ADDRESS + 0x00);
	uint32_t* GPIOA_PUPDR  = (uint32_t*)(GPIOA_BASE_ADDRESS + 0x0C);
	*GPIOA_MODER &= ~(0b11);
	*GPIOA_PUPDR &= ~(0b11);
}
char get_button()
{
	uint32_t* GPIOA_IDR  = (uint32_t*)(GPIOA_BASE_ADDRESS + 0x10);

	return *GPIOA_IDR & 0b000001;
}
void EXTIO_Custom_Handler()
{

}
void EXTI0_init()
{
	uint32_t* EXTICR1 = (uint32_t*)0x40013808;
	*EXTICR1 &= ~(0b1111 << 0);
	uint32_t* FTSR = (uint32_t*)0x40013c0c;
	*FTSR |= (1<<0);
	uint32_t* IMR = (uint32_t*)0x40013c00;
	*IMR |= (1<<0);
	uint32_t* NVIC_ISER0 =(uint32_t*)0xe000e100;
	*NVIC_ISER0 |= (1<<6);

}
void EXTI0_IRQHandler()
{
	__asm("NOP");
	uint32_t* PR = (uint32_t*)0x40013c14;
	*PR |= (1<<0);
}
void EXTIO_custom_Handler()
{
	uint32_t* PR = (uint32_t*)0x40013c14;
	*PR |= (1<<0);
}
void UART_Init()
{
	/* set alternate function for PA2 & PA3 in alternate function 07 */
	 __HAL_RCC_GPIOA_CLK_ENABLE();
	 uint32_t* GPIOA_MODER = (uint32_t*)0x40020000;
	 *GPIOA_MODER &= ~(0b1111 <<2);
	 *GPIOA_MODER |= (0b10 <<4) | (0b10<<6);
	 uint32_t* GPIOA_AFRL = (uint32_t*)0x40020020;
	 *GPIOA_AFRL &= ~(0xff<<8);
	 *GPIOA_AFRL |= (7<<8) | (7<<12);

	 __HAL_RCC_USART2_CLK_ENABLE();
	 /* set baund_rate 9600 */
	 uint32_t* BRR = (uint32_t*)0x40004408;
	 //104.1667
	 *BRR = (104<<4)|(3<<0);

	 uint32_t* CR1 = (uint32_t*)0x4000440c;
	 *CR1 |= (1<<13) | (1<<2) | (1<<13); //
	 *CR1 |= (1<<5);
	 uint32_t* ISER1 = (uint32_t*)0xe000e104;
	 *ISER1 |= (1<<(38-32));// bit 38-32 của position 38-32

}
void UART_Send_1Byte(char data)
{
	uint32_t* DR = (uint32_t*)0x40004404;
	uint32_t* SR = (uint32_t*)0x40004400;
	while(((*SR >> 7)&1) !=1);
	*DR =data;
	while(((*SR >> 6)&1) !=0); /* !0 khác 0*/

}
void UART_Send_Multi_Byte(char*arr,int size)
{
	for(int i=0; i< size; i++)
		UART_Send_1Byte(arr[i]);
}
/*
char UART_Recv_Byte()
{
	uint32_t* DR = (uint32_t*)0x40004404;
	uint32_t* SR = (uint32_t*)0x40004400;
}
*/

int rx_index = 0;
void UART2_IRQHandler()
{
	 uint32_t* DR = (uint32_t*)0x40004404;
	 rx_buf[rx_index]= *DR;
	 if (rx_index>=31)
		 rx_index = 0;

}


void ADC_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* MODER = (uint32_t*)0x40020000;
	*MODER |= (0b11<<0);

	__HAL_RCC_ADC1_CLK_ENABLE();
	uint32_t* SMPR1 = (uint32_t*)0x4001200c;
	*SMPR1 |= (0b111<<18);
	uint32_t* SMPR2 = (uint32_t*)0x40012010;
	*SMPR2 |= (0b111<<0);

	uint32_t* JSQR = (uint32_t*)0x40012038;
	*JSQR |= (16<<15); //set channel 16 (temp sensor)

//	uint32_t* CR1 = (uint32_t*)0x40012004;
	uint32_t* CR2 = (uint32_t*)0x40012008;
	*CR2 |= (0b01 << 20) |(1<<0);

	uint32_t* CCR = (uint32_t*)0x40012304;
	*CCR |= (1<<23);
}
uint16_t Read_ADC ()
{
	uint32_t* CR2 = (uint32_t*)0x40012008;
	*CR2 |= (1<<22);
	uint32_t* SR = (uint32_t*)0x40012000;
	while (((*SR >> 2)&1) == 0);
	*SR &=~(1<<2);

	uint32_t* JDR1 = (uint32_t*)0x4001203c;
	return *JDR1;
}

int main(void)
{
  HAL_Init();
  MX_GPIO_Init();
  UART_Init();

  ADC_init();
  while (1)
  {
 	  cnt = Read_ADC();
 	  vin = (cnt*3)/4095.0;
 	  temp = ((vin - 0.76) / 0.0025) + 25;
	  HAL_Delay(1000);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
