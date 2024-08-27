#include "stm32f10x.h"                  // Device header
#include "Delay.h"

#define RCC_APB2ENR *(uint32_t*)(0x40021000 + 0x18)    
#define GPIOB_CRH   *(uint32_t*)(0X40010C00 + 0x04)
#define GPIOB_BSRR  *(uint32_t*)(0X40010C00 + 0x10)
#define GPIOB_BRR   *(uint32_t*)(0X40010C00 + 0x14)
#define GPIOB_IDR   *(uint32_t*)(0X40010C00 + 0x08)

void MyI2C_W_SCL(uint8_t BitValue)
{
    if(BitValue == 0)
    {
        GPIOB_BRR |= (0x01<<10);
    }
    else
    {
        GPIOB_BSRR |= (0x01<<10);
    }
    Delay_us(10);
}

void MyI2C_W_SDA(uint8_t BitValue)
{
    if(BitValue == 0)
    {
        GPIOB_BRR |= (0x01<<11);
    }
    else
    {
        GPIOB_BSRR |= (0x01<<11);
    }
    Delay_us(10);
}

uint8_t MyI2C_R_SDA()
{
    uint8_t BitValue = (uint8_t)((GPIOB_IDR & (0x01<<11))>>11);
    Delay_us(10);
    return BitValue;
}

void MyI2C_Init()
{
    RCC_APB2ENR = 0x01<<3;//打开GPIOB的外设时钟
    
    GPIOB_CRH &= ~((uint32_t)0xFF<<8);
    GPIOB_CRH |= ((uint32_t)0x77<<8);//选择PB10和PB11配置为50MHz通用开漏输出模式
    
    GPIOB_BSRR |= ((uint16_t)0x03<<10);//默认为高电平
}

void MyI2C_Start()
{
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(0);
}

void MyI2C_Stop()
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for(i=0;i<8;i++)
    {
        MyI2C_W_SDA(Byte & (0x80>>i));
        MyI2C_W_SCL(1);
        MyI2C_W_SCL(0);
    }
}

uint8_t MyI2C_ReceiveByte()
{
    uint8_t i,Byte = 0x00;
    MyI2C_W_SDA(1);
    for(i=0;i<8;i++)
    {
        MyI2C_W_SCL(1);
        if(MyI2C_R_SDA() == 1) {Byte |= (0x80>>i);}
        MyI2C_W_SCL(0);
    }
    return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    MyI2C_W_SCL(1);
    MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck()
{
    uint8_t AckBit;
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    AckBit = MyI2C_R_SDA();
    MyI2C_W_SCL(0);
    return AckBit;
}
