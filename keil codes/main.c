/*

    Clock: 84 MHz
    I2C1 (PB8/PB9): MPU-6050
    ADC1 (PA1, PA5): Flex sensors
    USART2 (PA2/PA3): Bluetooth
    LED (PD12)
*/

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define ENABLE_HIGH_SPEED_BT  0 

volatile uint32_t msTicks = 0;
volatile float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
volatile uint32_t flexSensorValue1 = 0; // PA1
volatile uint32_t flexSensorValue2 = 0; // PA5

#define MPU_ADDR         0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_WHO_AM_I     0x75

#define GYRO_SCALE       16.4f
#define ACCEL_SCALE      4096.0f

float AccErrorX = 0.0f, AccErrorY = 0.0f;
float GyroErrorX = 0.0f, GyroErrorY = 0.0f, GyroErrorZ = 0.0f;

void SystemClock_Config(void);
void SysTick_Init_ms(void);
uint32_t millis(void);
void delay_ms_block(uint32_t ms);
static void delay_short(void);

void I2C1_Init(void);
int I2C1_ReadBytes(uint8_t dev, uint8_t reg, uint8_t *buf, int n);
uint8_t I2C1_ReadReg(uint8_t dev, uint8_t reg);
int MPU_Init(void);
int ReadAccelGyroBurst(int16_t *accel, int16_t *gyro);
void I2C1_Bus_Recovery(void);
void calculate_IMU_error(void);

void BT_Init(uint32_t baudrate);
void BT_SendString(const char *s);
void BT_SendAnglesAndFlex(float r, float p, float y, uint32_t flex1, uint32_t flex2);

void ADC1_Init_PA1_PA5(void);
uint16_t ADC1_Read_Channel(uint8_t ch);
void LED_Init_PD12(void);

void SysTick_Handler(void) { msTicks++; }

void SysTick_Init_ms(void)
{
    SysTick->LOAD = 84000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

uint32_t millis(void) { return msTicks; }

static void delay_short(void) { for (volatile int i = 0; i < 2000; ++i) __NOP(); }

void delay_ms_block(uint32_t ms)
{
    uint32_t t = millis();
    while ((millis() - t) < ms) { __NOP(); }
}

void SystemClock_Config(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;
    
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) | (336 << RCC_PLLCFGR_PLLN_Pos) |
                   (1 << RCC_PLLCFGR_PLLP_Pos) | (7 << RCC_PLLCFGR_PLLQ_Pos) | RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
}

/* I2C1  */
void I2C1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB8, PB9 AF4 Open Drain
    GPIOB->MODER &= ~((3U << 16) | (3U << 18));
    GPIOB->MODER |=  ((2U << 16) | (2U << 18)); 
    GPIOB->OTYPER |= (1U<<8) | (1U<<9);
    GPIOB->OSPEEDR |= ((3U << 16) | (3U << 18));
    GPIOB->PUPDR |=  ((1U << 16) | (1U << 18));
    GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOB->AFR[1] |= ((4U << 0) | (4U << 4));

    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    I2C1->CR1 = 0;
    I2C1->CR2 = 42; 
    I2C1->CCR = 0x8000 | 35; 
    I2C1->TRISE = 13;
    I2C1->CR1 |= I2C_CR1_PE;
}

int I2C1_ReadBytes(uint8_t dev, uint8_t reg, uint8_t *buf, int n)
{
    if (n <= 0) return -1;
    uint32_t t = 0;
    while (I2C1->SR2 & I2C_SR2_BUSY) if (++t > 1000000) return -2;

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (dev << 1);
    t = 0;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (++t > 1000000) { I2C1_Bus_Recovery(); return -3; }
    }
    (void)I2C1->SR2;

    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (dev << 1) | 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));

    I2C1->CR1 |= I2C_CR1_ACK;
    (void)I2C1->SR2;

    for (int i = 0; i < n; ++i) {
        if (i == n - 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        uint32_t rtime = 0;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
            if (++rtime > 1000000) { I2C1_Bus_Recovery(); return -4; }
        }
        buf[i] = (uint8_t)I2C1->DR;
    }
    I2C1->CR1 |= I2C_CR1_ACK;
    return 0;
}

uint8_t I2C1_ReadReg(uint8_t dev, uint8_t reg)
{
    uint8_t b = 0;
    I2C1_ReadBytes(dev, reg, &b, 1);
    return b;
}

void I2C1_Bus_Recovery(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER = (GPIOB->MODER & ~(3U << 16)) | (1U << 16); // PB8 Out
    GPIOB->MODER = (GPIOB->MODER & ~(3U << 18)); // PB9 In
    GPIOB->PUPDR |= (1U << 18);
    for (int i = 0; i < 9; ++i) {
        GPIOB->BSRR = (1U << 8); delay_short();
        GPIOB->BSRR = (1U << 24); delay_short();
    }
    GPIOB->BSRR = (1U << 8); delay_short();
    I2C1_Init();
}

int MPU_Init(void)
{
    I2C1->CR1 |= I2C_CR1_START; while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = (MPU_ADDR << 1); while (!(I2C1->SR1 & I2C_SR1_ADDR)); (void)I2C1->SR2;
    I2C1->DR = REG_PWR_MGMT_1; while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = 0x00; while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
    delay_ms_block(10);

    uint8_t configData[] = {
        REG_CONFIG, 0x03,
        REG_GYRO_CONFIG, 0x18,
        REG_ACCEL_CONFIG, 0x10
    };
    
    for(int i=0; i<6; i+=2) {
        I2C1->CR1 |= I2C_CR1_START; while (!(I2C1->SR1 & I2C_SR1_SB));
        I2C1->DR = (MPU_ADDR << 1); while (!(I2C1->SR1 & I2C_SR1_ADDR)); (void)I2C1->SR2;
        I2C1->DR = configData[i]; while (!(I2C1->SR1 & I2C_SR1_TXE));
        I2C1->DR = configData[i+1]; while (!(I2C1->SR1 & I2C_SR1_BTF));
        I2C1->CR1 |= I2C_CR1_STOP;
        delay_ms_block(5);
    }

    return 0;
}

int ReadAccelGyroBurst(int16_t *accel, int16_t *gyro)
{
    uint8_t buf[14];
    if (I2C1_ReadBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14) != 0) return -1;
    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    gyro[0]  = (int16_t)((buf[8] << 8) | buf[9]);
    gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);
    return 0;
}
void BT_Init(uint32_t baudrate)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2, PA3 AF7
    GPIOA->MODER   = (GPIOA->MODER & ~((3U<<4)|(3U<<6))) | ((2U<<4)|(2U<<6));
    GPIOA->AFR[0]  = (GPIOA->AFR[0] & ~((0xFU<<8)|(0xFU<<12))) | ((7U<<8)|(7U<<12));

    USART2->CR1 &= ~USART_CR1_UE;

    uint32_t apb1_freq = 42000000;
    uint32_t div = (apb1_freq + (baudrate/2)) / baudrate;
    USART2->BRR = div;

    USART2->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART2->CR1 |= USART_CR1_UE;
}

void BT_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (c & 0xFF);
}

void BT_SendString(const char *s)
{
    while (*s) BT_SendChar(*s++);
}

void BT_SendAnglesAndFlex(float r, float p, float y, uint32_t flex1, uint32_t flex2)
{
    char buf[80];
    snprintf(buf, sizeof(buf), "%.1f,%.1f,%lu,%lu\n", r, p, (unsigned long)flex1, (unsigned long)flex2);
    BT_SendString(buf);
}

void calculate_IMU_error(void)
{
    float sumAccX = 0, sumAccY = 0;
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    int16_t a[3], g[3];
    int count = 0;

    for(int i=0; i<20; i++) { ReadAccelGyroBurst(a, g); delay_ms_block(2); }

    while (count < 200) {
        if (ReadAccelGyroBurst(a, g) == 0) {
            float AccX = a[0] / ACCEL_SCALE;
            float AccY = a[1] / ACCEL_SCALE;
            float AccZ = a[2] / ACCEL_SCALE;
            
            sumAccX += atanf(AccY / sqrtf(AccX*AccX + AccZ*AccZ)) * 180/M_PI;
            sumAccY += atanf(-AccX / sqrtf(AccY*AccY + AccZ*AccZ)) * 180/M_PI;
            
            sumGyroX += g[0] / GYRO_SCALE;
            sumGyroY += g[1] / GYRO_SCALE;
            sumGyroZ += g[2] / GYRO_SCALE;
            count++;
            delay_ms_block(2);
        }
    }
    AccErrorX = sumAccX / 200.0f;
    AccErrorY = sumAccY / 200.0f;
    GyroErrorX = sumGyroX / 200.0f;
    GyroErrorY = sumGyroY / 200.0f;
    GyroErrorZ = sumGyroZ / 200.0f;
}

/*ADC & LED: PA1 and PA5 */
void ADC1_Init_PA1_PA5(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // 
    GPIOA->MODER |= (3U << 2);   // PA1 analog
    GPIOA->MODER |= (3U << 10);  // PA5 analog

   
    ADC1->SMPR2 |= (7U << 3);   // channel 1 
    ADC1->SMPR2 |= (7U << 15);  // channel 5 
    ADC1->SQR1 = 0; // R
    ADC1->SQR3 = 1; //
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC1_Read_Channel(uint8_t ch) {
   
    ADC1->SQR3 = (uint32_t)ch & 0x1F;
    
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    uint16_t val = (uint16_t)ADC1->DR;
    return val;
}

void LED_Init_PD12(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER = (GPIOD->MODER & ~(3U << 24)) | (1U << 24);
}

void LED2_Init_PD13(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(3U << 26);
    GPIOD->MODER |=  (1U << 26);  
}

int main(void)
{
    SystemClock_Config();
    SysTick_Init_ms();
    
    BT_Init(9600);
    delay_ms_block(100);

#if ENABLE_HIGH_SPEED_BT
    BT_SendString("AT+BAUD8");
    delay_ms_block(1100);
    BT_Init(115200);
    delay_ms_block(100);
    BT_SendString("\r\nUART UPGRADED TO 115200\r\n");
#else
    BT_SendString("\r\nUART AT 9600 (SLOW)\r\n");
#endif

    I2C1_Init();
    MPU_Init();
    
    BT_SendString("Calibrating...\n");
    calculate_IMU_error();
    BT_SendString("Ready.\n");

    ADC1_Init_PA1_PA5();
    LED_Init_PD12();
		LED2_Init_PD13();  

    uint32_t prevTime = millis();
    float gyroAngleX = 0.0f, gyroAngleY = 0.0f, gyroAngleZ = 0.0f;
    static int left_clicked = 0;
    static int right_clicked = 0;

    while (1) {
        int16_t a[3], g[3];
        
        if (ReadAccelGyroBurst(a, g) == 0) {
            
            float AccX = a[0] / ACCEL_SCALE;
            float AccY = a[1] / ACCEL_SCALE;
            float AccZ = a[2] / ACCEL_SCALE;

            float accAngleX = atanf(AccY / sqrtf(AccX*AccX + AccZ*AccZ)) * 180/M_PI - AccErrorX;
            float accAngleY = atanf(-AccX / sqrtf(AccY*AccY + AccZ*AccZ)) * 180/M_PI - AccErrorY;

            float GyroX = g[0] / GYRO_SCALE - GyroErrorX;
            float GyroY = g[1] / GYRO_SCALE - GyroErrorY;
            float GyroZ = g[2] / GYRO_SCALE - GyroErrorZ;

            uint32_t now = millis();
            float dt = (now - prevTime) / 1000.0f;
            if (dt < 0.001f) dt = 0.001f;
            prevTime = now;

            gyroAngleX += GyroX * dt;
            gyroAngleY += GyroY * dt;
            gyroAngleZ += GyroZ * dt;

            roll  = 0.96f * gyroAngleX + 0.04f * accAngleX;
            pitch = 0.96f * gyroAngleY + 0.04f * accAngleY;
            yaw   = gyroAngleZ;
            
            // Read both flex sensors
            flexSensorValue1 = ADC1_Read_Channel(1); // PA1 -> channel 1
            flexSensorValue2 = ADC1_Read_Channel(5); // PA5 -> channel 5

            // Left click logic (PA1)
            if (flexSensorValue1 < 1500) { 
                GPIOD->BSRR = (1U << 12); // LED4 ON
                if (!left_clicked) {
                    BT_SendString("LCLICK\n");
                    left_clicked = 1;
                }
            } else {
                GPIOD->BSRR = (1U << 28); // LED4 OFF
                left_clicked = 0;
            }
						// right click logic(PA5)
          if (flexSensorValue2 < 1500) { 
                GPIOD->BSRR = (1U << 13);   // LED3 ON
                if (!right_clicked) {
                    BT_SendString("RCLICK\n");
                    right_clicked = 1;
                }
            } else {
                GPIOD->BSRR = (1U << (13 + 16)); // LED3 OFF
                right_clicked = 0;
            }
						
            BT_SendAnglesAndFlex(roll, pitch, yaw, flexSensorValue1, flexSensorValue2);

        } else {
            I2C1_Bus_Recovery();
            MPU_Init();
        }

        delay_ms_block(10); 
    }
}