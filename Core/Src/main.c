/* ========================================================================
 * STM32F411CEU6 - Line Follower with Obstacle Detection
 * Ultrasonic on PA4 (TRIG) and PA5 (ECHO)
 * ======================================================================== */

#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>

// ========== Configuration ==========
#define LCD_ADDR 0x27
#define LCD_BACKLIGHT 0x08

#define MOTOR_A_IN1_PIN   6
#define MOTOR_A_IN2_PIN   7
#define MOTOR_B_IN3_PIN   0
#define MOTOR_B_IN4_PIN   1


// Motor speeds
#define SPEED_BASE     300
#define SPEED_TURN     250
#define SPEED_FAST     350

// Obstacle detection threshold (in cm)
#define OBSTACLE_DISTANCE  20

// ========== Function Prototypes ==========
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM2_PWM_Init(void);
void TIM1_Init(void);
void I2C1_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

void I2C_Start(void);
void I2C_Stop(void);
void I2C_WriteAddr(uint8_t addr);
void I2C_WriteData(uint8_t data);

void LCD_Send4Bits(uint8_t data, uint8_t lcd_addr);
void LCD_SendCmd(uint8_t cmd, uint8_t lcd_addr);
void LCD_SendData(uint8_t data, uint8_t lcd_addr);
void LCD_Init(uint8_t lcd_addr);
void LCD_SetCursor(uint8_t row, uint8_t col, uint8_t lcd_addr);
void LCD_SendString(char *str, uint8_t lcd_addr);
void LCD_Print(char *str, uint8_t row, uint8_t col, uint8_t lcd_addr);

uint8_t Read_Left_IR(void);
uint8_t Read_Center_IR(void);
uint8_t Read_Right_IR(void);
uint16_t Read_Ultrasonic_Distance(void);

void Motor_Forward(uint16_t left_speed, uint16_t right_speed);
void Motor_Stop(void);

void Follow_Line(void);
void Display_Status(uint8_t left, uint8_t center, uint8_t right, uint16_t distance);

// ========== System Clock Configuration ==========
void SystemClock_Config(void) {
    FLASH->ACR |= FLASH_ACR_LATENCY_0WS;
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN;
}

// ========== Delay Functions ==========
void delay_us(uint32_t us) {
    volatile uint32_t count = us * 16;
    while(count--);
}

void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

// ========== GPIO Initialization ==========
void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    volatile uint32_t delay = 1000;
    while(delay--);

    // IR Sensors as inputs
    GPIOA->MODER &= ~GPIO_MODER_MODER0;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_1;

    GPIOB->MODER &= ~GPIO_MODER_MODER14;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD14;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD14_1;

    GPIOB->MODER &= ~GPIO_MODER_MODER15;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD15;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD15_1;

    // Ultrasonic TRIG (PA4) as output
    GPIOA->MODER &= ~GPIO_MODER_MODER4;
    GPIOA->MODER |= GPIO_MODER_MODER4_0;
    GPIOA->BSRR = (1 << (TRIG_PIN + 16));  // Set LOW initially

    // Ultrasonic ECHO (PA5) as input
    GPIOA->MODER &= ~GPIO_MODER_MODER5;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5;

    // Motor PWM pins (PA1, PA2)
    GPIOA->MODER &= ~(GPIO_MODER_MODER1 | GPIO_MODER_MODER2);
    GPIOA->MODER |= (GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL1 | GPIO_AFRL_AFSEL2);
    GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL1_Pos) | (1 << GPIO_AFRL_AFSEL2_Pos);

    // Motor direction pins
    GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);

    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0);

    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1);

    GPIOA->BSRR = (1 << (MOTOR_A_IN1_PIN + 16)) | (1 << (MOTOR_A_IN2_PIN + 16));
    GPIOB->BSRR = (1 << (MOTOR_B_IN3_PIN + 16)) | (1 << (MOTOR_B_IN4_PIN + 16));
}

// ========== TIM1 for Microsecond Timing ==========
void TIM1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 16 - 1;      // 16MHz / 16 = 1MHz (1μs per tick)
    TIM1->ARR = 0xFFFF;      // Maximum count
    TIM1->CR1 |= TIM_CR1_CEN; // Enable counter
}

// ========== TIM2 PWM Initialization ==========
void TIM2_PWM_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    volatile uint32_t delay = 1000;
    while(delay--);

    TIM2->PSC = 16 - 1;
    TIM2->ARR = 1000 - 1;

    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM2->CCER |= TIM_CCER_CC2E;

    TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM2->CCER |= TIM_CCER_CC3E;

    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;

    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
}

// ========== I2C1 Initialization ==========
void I2C1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
    GPIOB->AFR[0] |= (4 << GPIO_AFRL_AFSEL6_Pos) | (4 << GPIO_AFRL_AFSEL7_Pos);

    GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);

    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_Start(void) {
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
}

void I2C_Stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_WriteAddr(uint8_t addr) {
    I2C1->DR = addr;
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
}

void I2C_WriteData(uint8_t data) {
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
}

void LCD_Send4Bits(uint8_t data, uint8_t lcd_addr) {
    uint8_t byte;
    byte = data | 0x04 | LCD_BACKLIGHT;
    I2C_Start();
    I2C_WriteAddr((lcd_addr << 1) | 0);
    I2C_WriteData(byte);
    I2C_Stop();
    delay_us(1);

    byte = data | LCD_BACKLIGHT;
    I2C_Start();
    I2C_WriteAddr((lcd_addr << 1) | 0);
    I2C_WriteData(byte);
    I2C_Stop();
    delay_us(50);
}

void LCD_SendCmd(uint8_t cmd, uint8_t lcd_addr) {
    uint8_t data_u = (cmd & 0xF0);
    uint8_t data_l = ((cmd << 4) & 0xF0);
    LCD_Send4Bits(data_u, lcd_addr);
    LCD_Send4Bits(data_l, lcd_addr);
    if(cmd == 0x01 || cmd == 0x02) delay_ms(2);
    else delay_us(50);
}

void LCD_SendData(uint8_t data, uint8_t lcd_addr) {
    uint8_t data_u = (data & 0xF0);
    uint8_t data_l = ((data << 4) & 0xF0);
    uint8_t byte;

    byte = data_u | 0x05 | LCD_BACKLIGHT;
    I2C_Start();
    I2C_WriteAddr((lcd_addr << 1) | 0);
    I2C_WriteData(byte);
    I2C_Stop();
    delay_us(1);

    byte = data_u | 0x01 | LCD_BACKLIGHT;
    I2C_Start();
    I2C_WriteAddr((lcd_addr << 1) | 0);
    I2C_WriteData(byte);
    I2C_Stop();
    delay_us(50);

    byte = data_l | 0x05 | LCD_BACKLIGHT;
    I2C_Start();
    I2C_WriteAddr((lcd_addr << 1) | 0);
    I2C_WriteData(byte);
    I2C_Stop();
    delay_us(1);

    byte = data_l | 0x01 | LCD_BACKLIGHT;
    I2C_Start();
    I2C_WriteAddr((lcd_addr << 1) | 0);
    I2C_WriteData(byte);
    I2C_Stop();
    delay_us(50);
}

void LCD_Init(uint8_t lcd_addr) {
    delay_ms(100);
    LCD_Send4Bits(0x30, lcd_addr);
    delay_ms(5);
    LCD_Send4Bits(0x30, lcd_addr);
    delay_ms(1);
    LCD_Send4Bits(0x30, lcd_addr);
    delay_ms(1);
    LCD_Send4Bits(0x20, lcd_addr);
    delay_ms(1);
    LCD_SendCmd(0x28, lcd_addr);
    delay_ms(1);
    LCD_SendCmd(0x08, lcd_addr);
    delay_ms(1);
    LCD_SendCmd(0x01, lcd_addr);
    delay_ms(3);
    LCD_SendCmd(0x06, lcd_addr);
    delay_ms(1);
    LCD_SendCmd(0x0C, lcd_addr);
    delay_ms(1);
}

void LCD_SetCursor(uint8_t row, uint8_t col, uint8_t lcd_addr) {
    uint8_t pos = (row == 0) ? (0x80 | col) : (0xC0 | col);
    LCD_SendCmd(pos, lcd_addr);
}

void LCD_SendString(char *str, uint8_t lcd_addr) {
    while(*str) {
        LCD_SendData(*str++, lcd_addr);
    }
}

void LCD_Print(char *str, uint8_t row, uint8_t col, uint8_t lcd_addr) {
    LCD_SetCursor(row, col, lcd_addr);
    LCD_SendString(str, lcd_addr);
}


uint8_t Read_Left_IR(void) {
    return (GPIOB->IDR & GPIO_IDR_ID14) ? 1 : 0;
}

uint8_t Read_Center_IR(void) {
    return (GPIOB->IDR & GPIO_IDR_ID15) ? 1 : 0;
}

uint8_t Read_Right_IR(void) {
    return (GPIOA->IDR & GPIO_IDR_ID0) ? 1 : 0;
}

void Motor_Forward(uint16_t left_speed, uint16_t right_speed) {
    GPIOA->BSRR = (1 << MOTOR_A_IN1_PIN);
    GPIOA->BSRR = (1 << (MOTOR_A_IN2_PIN + 16));

    GPIOB->BSRR = (1 << MOTOR_B_IN3_PIN);
    GPIOB->BSRR = (1 << (MOTOR_B_IN4_PIN + 16));

    if(left_speed > 999) left_speed = 999;
    if(right_speed > 999) right_speed = 999;

    TIM2->CCR2 = left_speed;
    TIM2->CCR3 = right_speed;
}

void Motor_Stop(void) {
    GPIOA->BSRR = (1 << (MOTOR_A_IN1_PIN + 16)) | (1 << (MOTOR_A_IN2_PIN + 16));
    GPIOB->BSRR = (1 << (MOTOR_B_IN3_PIN + 16)) | (1 << (MOTOR_B_IN4_PIN + 16));
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
}

void Display_Status(uint8_t left, uint8_t center, uint8_t right, uint16_t distance) {
    char buffer[17];

    sprintf(buffer, "L:%c C:%c R:%c    ",
            left ? 'B' : 'W',
            center ? 'B' : 'W',
            right ? 'B' : 'W');
    LCD_Print(buffer, 0, 0, LCD_ADDR);

    if(distance < OBSTACLE_DISTANCE) {
        sprintf(buffer, "STOP! %dcm      ", distance);
    } else {
        sprintf(buffer, "OK %dcm         ", distance);
    }
    LCD_Print(buffer, 1, 0, LCD_ADDR);
}

void Follow_Line(void) {
    uint8_t left, center, right;
    uint16_t distance;

    while(1) {
        left = Read_Left_IR();
        center = Read_Center_IR();
        right = Read_Right_IR();
        distance = Read_Ultrasonic_Distance();

        Display_Status(left, center, right, distance);

        // Check obstacle
        if(distance < OBSTACLE_DISTANCE) {
            Motor_Stop();
            delay_ms(100);
            continue;
        }

        // Line following
        if(center && !left && !right) {
            Motor_Forward(SPEED_FAST, SPEED_FAST);
        }
        else if(center && left && !right) {
            Motor_Forward(SPEED_BASE, SPEED_TURN);
        }
        else if(center && !left && right) {
            Motor_Forward(SPEED_TURN, SPEED_BASE);
        }
        else if(left && !center && !right) {
            Motor_Forward(SPEED_TURN, SPEED_BASE);
        }
        else if(right && !center && !left) {
            Motor_Forward(SPEED_BASE, SPEED_TURN);
        }
        else if(left && center && right) {
            Motor_Stop();
            delay_ms(500);
        }
        else if(!left && !center && !right) {
            Motor_Stop();
        }
        else {
            Motor_Forward(SPEED_BASE, SPEED_BASE);
        }

        delay_ms(50);
    }
}

int main(void) {
    SystemClock_Config();
    GPIO_Init();
    TIM1_Init();
    TIM2_PWM_Init();
    I2C1_Init();

    Motor_Stop();
    delay_ms(500);

    LCD_Init(LCD_ADDR);
    LCD_SendCmd(0x01, LCD_ADDR);
    delay_ms(2);

    LCD_Print("Line+Obstacle", 0, 0, LCD_ADDR);
    LCD_Print("Ready!", 1, 0, LCD_ADDR);

    delay_ms(2000);

    Follow_Line();

    return 0;
}
