#include "stm32f10x.h"
#include "led.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "debug.h"
#include "ledseq.h"

#include "i2cdev.h"
#include "i2c_gpio.h"

#define BOARD_2

#ifdef BOARD_2
#define SCL_PIN     GPIO_Pin_6
#define SDA_PIN     GPIO_Pin_7
#define I2C_TYPE    GPIOB
#define RCC_I2C     RCC_APB2Periph_GPIOB
#else
#define SCL_PIN     GPIO_Pin_11
#define SDA_PIN     GPIO_Pin_8
#define I2C_TYPE    GPIOA
#define RCC_I2C     RCC_APB2Periph_GPIOA
#endif

#define SCL_H         I2C_TYPE->BSRR = SCL_PIN
#define SCL_L         I2C_TYPE->BRR  = SCL_PIN

#define SDA_H         I2C_TYPE->BSRR = SDA_PIN
#define SDA_L         I2C_TYPE->BRR  = SDA_PIN

#define SCL_read      I2C_TYPE->IDR  & SCL_PIN
#define SDA_read      I2C_TYPE->IDR  & SDA_PIN


static void I2C_delay(void)
{
    volatile int i = 7;

    while (i)
        i--;
}

static bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if (! (SDA_read)) {
        return false;
    }

    SDA_L;
    I2C_delay();
    if (SDA_read)

        return false;
    SDA_L;
    I2C_delay();

    return true;
}


static void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}

static void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}


static void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}


static bool I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if (SDA_read)
    {
        SCL_L;

        return false;
    }
    SCL_L;

    return true;
}

static void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--)
    {
        SCL_L;
        I2C_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

static uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--)
    {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if (SDA_read)
        {
            byte |= 0x01;
        }
    }

    SCL_L;
    return byte;
}

void i2cInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_I2C, ENABLE);

    GPIO_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(I2C_TYPE, &GPIO_InitStructure);
}


bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;

    if (!I2C_Start())
        return false;

    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck())
    {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++)
    {
        I2C_SendByte(data[i]);
        if (!I2C_WaitAck())
        {
            I2C_Stop();
            return false;
        }
    }

    I2C_Stop();
    return true;
}

bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck())
    {
        I2C_Stop();
        return false;
    }

    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();
    return true;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start())
        return false;

    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck())
    {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck();
    while (len)
    {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }

    I2C_Stop();
    return true;
}

uint16_t i2cGetErrorCounter(void)
{
    return 0;
}

bool i2cdevRead(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
               uint16_t len, uint8_t *data) {
    I2Cx;
    return i2cRead(devAddress, memAddress, len, data);
}

bool i2cdevReadByte(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data)
{
  return i2cdevRead(I2Cx, devAddress, memAddress, 1, data);
}

bool i2cdevReadBit(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitNum, uint8_t *data)
{
  uint8_t byte;
  bool status;

  status = i2cdevRead(I2Cx, devAddress, memAddress, 1, &byte);
  *data = byte & (1 << bitNum);

  return status;
}

bool i2cdevReadBits(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data)
{
  bool status;
  uint8_t byte;

  if ((status = i2cdevReadByte(I2Cx, devAddress, memAddress, &byte)) == TRUE)
  {
      uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
      byte &= mask;
      byte >>= (bitStart - length + 1);
      *data = byte;
  }
  return status;
}

bool i2cdevWrite(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                uint16_t len, uint8_t *data)
{
    I2Cx;
    return i2cWriteBuffer(devAddress, memAddress, len, data);
}

bool i2cdevWriteByte(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t data)
{
  return i2cdevWrite(I2Cx, devAddress, memAddress, 1, &data);
}

bool i2cdevWriteBit(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data)
{
    uint8_t byte;
    i2cdevReadByte(I2Cx, devAddress, memAddress, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    return i2cdevWriteByte(I2Cx, devAddress, memAddress, byte);
}

bool i2cdevWriteBits(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data)
{
  bool status;
  uint8_t byte;

  if ((status = i2cdevReadByte(I2Cx, devAddress, memAddress, &byte)) == TRUE)
  {
      uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
      data <<= (bitStart - length + 1); // shift data into correct position
      data &= mask; // zero all non-important bits in data
      byte &= ~(mask); // zero all important bits in existing byte
      byte |= data; // combine data with existing byte
      status = i2cdevWriteByte(I2Cx, devAddress, memAddress, byte);
  }

  return status;
}

int i2cdevInit(I2C_TypeDef *I2Cx) {
    I2Cx;
    i2cInit();
    return 1;
}

