#include "stm32f446xx.h"
#include <stdio.h>

#define RS (1<<10)   // PA10 = RS
#define EN (1<<11)   // PA11 = EN

void delay_us(int t) {
    for (volatile int i=0; i<t*10; i++);
}

void lcd_pulse(void) {
    GPIOA->ODR |= EN;
    delay_us(100);
    GPIOA->ODR &= ~EN;
    delay_us(100);
}

void commands(uint8_t cmd) {
    GPIOA->ODR = (GPIOA->ODR & ~(0xFF<<2)) | (cmd<<2); // PA2–PA9 = D0–D7
    GPIOA->ODR &= ~RS;    // RS=0
    lcd_pulse();
}

void datas(uint8_t data) {
    GPIOA->ODR = (GPIOA->ODR & ~(0xFF<<2)) | (data<<2); // PA2–PA9 = D0–D7
    GPIOA->ODR |= RS;     // RS=1
    lcd_pulse();
}

void lcd_init(void) {
    commands(0x38); delay_us(2000);  // 8-bit, 2-line, 5x7
    commands(0x0C); delay_us(2000);  // Display ON, Cursor OFF
    commands(0x01); delay_us(2000);  // Clear display
    commands(0x06); delay_us(2000);  // Entry mode
    commands(0x80); delay_us(2000);  // Force cursor to beginning
		commands(0x90); delay_us(2000);
		commands(0xD0); delay_us(2000);
	
}

int main(void) {
    uint16_t adc_value;
    float voltage;
    char buffer[16];
	char o[]={"Motor : ON"};
	char c[]={"Motor : OFF"};
	char m[]={"Motor Control"};
	int i;

    // Enable GPIOA and GPIOB clocks
    RCC->AHB1ENR |= (1<<0) | (1<<1);

    // Set PA2–PA11 as output (LCD)
    GPIOA->MODER &= ~(0xFFFFF << 4);
    GPIOA->MODER |=  (0x55555 << 4);

    // Set PA1 as analog (ADC input)
    GPIOA->MODER &= ~(3<<(2*1));
    GPIOA->MODER |=  (3<<(2*1));

    // Set PB1 as output (LED)
    GPIOB->MODER &= ~(3 << (2*1));
    GPIOB->MODER |=  (1 << (2*1));

    // Enable ADC1 clock
    RCC->APB2ENR |= (1<<8);

    // ADC Common config
    ADC123_COMMON->CCR &= ~(3<<16);
    ADC123_COMMON->CCR |=  (1<<16);  // prescaler /4

    // ADC1 config
    ADC1->SMPR2 |= (7<<0);    // channel 1, 480 cycles
    ADC1->SQR3 = 1;           // channel 1 in SQ1
    ADC1->SQR1 = 0;           // sequence length =1
    ADC1->CR2 |= 1;           // ADC ON

    for (volatile int i=0; i<100000; i++); // LCD power-up delay
    lcd_init();
		for(i=0;i<=12;i++)
		{
			commands(0x80+i);
			datas(m[i]);
		}
		for(i=0;i<=10;i++)
		{
			commands(0x90+i);
			datas(c[i]);
		}

    while (1)
			{
        // Start conversion
        ADC1->CR2 |= (1<<30);
        while (!(ADC1->SR & (1<<1)));  // wait for EOC
        adc_value = ADC1->DR;

        // Convert ADC value to voltage (0–3.3V)
        voltage = ((float)adc_value / 4095.0f) * 3.3f;

        // Display voltage on LCD
        commands(0xc0);
        sprintf(buffer, "V=%.2f V   ", voltage);
        for (int i=0; buffer[i]!='\0'; i++)
						//commands(0xc0+i);
            datas(buffer[i]);

        // LED ON when voltage = 2.0V
        if (voltage >= 2.0f)
				{
					for(i=0;i<=9;i++)
			{
				commands(0xD0+i);
				datas(o[i]);
		}
					
            GPIOB->ODR |=  (1 << 1);   // LED ON
				}	
        else
            GPIOB->ODR &= ~(1 << 1);   // LED OFF
    }
}
