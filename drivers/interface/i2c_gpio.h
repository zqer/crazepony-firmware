/* Use gpio to emulate i2c clock, and will be removed next version */

#ifndef __I2C_GPIO_H_
#define __I2C_GPIO_H_
#include "stm32f10x.h"

bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data);
bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf);
void i2cInit(void);
uint16_t i2cGetErrorCounter(void);
/* static void i2cUnstick(void); */

#if 0
#define SCL_PIN GPIO_Pin_6
#define SDA_PIN GPIO_Pin_7
#endif

#endif

