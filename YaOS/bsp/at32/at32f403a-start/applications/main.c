/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-08     shelton      first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_gpio.h"
#include "app_uart.h"

/* defined the LED2 pin: PD13 */
#define LED2_PIN    GET_PIN(D, 13)
/* defined the LED3 pin: PD14 */
#define LED3_PIN    GET_PIN(D, 14)
/* defined the LED4 pin: PD15 */
#define LED4_PIN    GET_PIN(D, 15)
#define TEST_KEY    GET_PIN(A,0)


extern void Spk_SendCMD(unsigned char CMD, unsigned char feedback, unsigned short dat);

int main(void)
{
//    uint32_t Speed = 50;
    int count = 1;    /* set LED0 pin mode to output */

    /* set LED2 pin mode to output */
    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
    /* set LED3 pin mode to output */
    rt_pin_mode(LED3_PIN, PIN_MODE_OUTPUT);
    /* set LED4 pin mode to output */
    rt_pin_mode(LED4_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(TEST_KEY,PIN_MODE_INPUT);
    if (uart_open("uart2") != RT_EOK)
    {
        rt_kprintf("uart open error.\n");
        while (1)
        {
            rt_thread_delay(10);
        }
    }
    rt_thread_delay(500);
    Spk_SendCMD(0x0F, 0, 0x0101);

    while (1)
    {
//        rt_pin_write(LED2_PIN, PIN_LOW);
//        rt_thread_mdelay(Speed);
//        rt_pin_write(LED3_PIN, PIN_LOW);
//        rt_thread_mdelay(Speed);
//        rt_pin_write(LED4_PIN, PIN_LOW);
//        rt_thread_mdelay(Speed);
//        rt_pin_write(LED2_PIN, PIN_HIGH);
//        rt_thread_mdelay(Speed);
//        rt_pin_write(LED3_PIN, PIN_HIGH);
//        rt_thread_mdelay(Speed);
//        rt_pin_write(LED4_PIN, PIN_HIGH);
//        rt_thread_mdelay(Speed);
        /* 读取按键 KEY0 的引脚状态 */
        if (rt_pin_read(TEST_KEY) == PIN_HIGH)
        {
            rt_thread_mdelay(100);
            if (rt_pin_read(TEST_KEY) == PIN_HIGH)
            {
                /* 按键已被按下，输出 log，点亮 LED 灯 */
                rt_pin_write(LED2_PIN, PIN_LOW);
                Spk_SendCMD(0x01, 0, 0x0000);
            }
        }
        else
        {
            /* 按键没被按下，熄灭 LED 灯 */
            rt_pin_write(LED2_PIN, PIN_HIGH);
        }
        rt_thread_mdelay(10);
        count++;
    }
}

