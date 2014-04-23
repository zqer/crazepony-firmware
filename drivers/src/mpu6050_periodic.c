#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "led.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "debug.h"
#include "ledseq.h"

#include "i2c_gpio.h"
#include "mpu6050.h"

#define devAddr MPU6050_ADDRESS_AD0_LOW
static uint8_t buffer[14];

static void MPU6050_Delay(unsigned long time)
{
   long i = 0;

   while(i < time) {
        i++;
   }
}


void MPU6050_READ(void)
{
    i2cRead(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
}


void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;

    i2cRead(dev, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    i2cWrite(dev, reg, b);
}


void IICwriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data)
{
    u8 mask;
    u8 b;

    i2cRead(dev, reg, 1, &b);
    mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
    data <<= (8 - length);
    data >>= (7 - bitStart);
    b &= mask;
    b |= data;
    i2cWrite(dev, reg, b);
}

uint8_t mpu6050GetDeviceID()
{
  i2cRead(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_LENGTH, buffer);
  return buffer[0];
}

char MPU6050_Check(void)
{
    uint8_t buf1[5]={0xaa,0xaa,0xaa,0xaa,0xaa};
    uint8_t buf2[5];
    char i;

    for(i=0;i<5;i++){
        IICwriteBits(devAddr, 0x00+i, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, buf1[i]);
    }
    for(i=0;i<5;i++) {
        i2cRead(devAddr,0x00+i,MPU6050_CFG_DLPF_CFG_LENGTH,buf2);
    }

    for (i = 0; i < 5; i++)
    {
        if (buf1[i] != 0xaa)
            break;
    }

    if (i == 5) {
        return 1 ;
    } else {
        return 0;
    }
}

void MPU6050_INIT(void)
{
    MPU6050_Delay(2000);
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    MPU6050_Delay(2000);
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    MPU6050_Delay(2000);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    MPU6050_Delay(2000);
    MPU6050_setDLPF(MPU6050_DLPF_BW_42);
    MPU6050_Delay(2000);
    MPU6050_setSleepEnabled(0);
    MPU6050_Delay(2000);
    MPU6050_setI2CMasterModeEnabled(0);
    MPU6050_Delay(2000);
    MPU6050_setI2CBypassEnabled(1);
    MPU6050_Delay(2000);
}
