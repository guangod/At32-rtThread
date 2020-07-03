/*
 * File      : app_uart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 *2017-12-15      DQL          the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include "app_uart.h"
#include "drv_usart.h"

unsigned char SpkSendCmd[10] = {0};
unsigned char SpkRsvBuf[10] = {0};

/* 串口接收事件标志 */
#define UART_RX_EVENT (1 << 0)

/* 事件控制块 */
static struct rt_event event;
/* 串口设备句柄 */
static rt_device_t uart_device = RT_NULL;
static struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

/* 回调函数 */
static rt_err_t uart_intput(rt_device_t dev, rt_size_t size)
{
    /* 发送事件 */
    rt_event_send(&event, UART_RX_EVENT);

    return RT_EOK;
}

rt_uint8_t uart_getchar(void)
{
    rt_uint32_t e;
    rt_uint8_t ch;

    /* 读取1字节数据 */
    while (rt_device_read(uart_device, 0, &ch, 1) != 1)
    {
        /* 接收事件 */
        rt_event_recv(&event, UART_RX_EVENT, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER, &e);
    }

    return ch;
}
void uart_putchar(const rt_uint8_t c)
{
    rt_size_t len = 0;
    rt_uint32_t timeout = 0;
    do
    {
        len = rt_device_write(uart_device, 0, &c, 1);
        timeout++;
    }
    while (len != 1 && timeout < 500);
}
void uart_putstring(const rt_uint8_t *s)
{
    while (*s)
    {
        uart_putchar(*s++);
    }
}

/********************************************************************************************
 - 功能描述：求和校验
 - 隶属模块：
 - 参数说明：
 - 返回说明：
 - 注：      和校验的思路如下
             发送的指令，去掉起始和结束。将中间的6个字节进行累加，最后取反码
             接收端就将接收到的一帧数据，去掉起始和结束。将中间的数据累加，再加上接收到的校验
             字节。刚好为0.这样就代表接收到的数据完全正确。
********************************************************************************************/
void DoSum(unsigned char *Str, unsigned char len)
{
    unsigned short xorsum = 0;
    unsigned char i;
    Str += 1;
    for (i = 0; i < (len - 2); i++)
    {
        xorsum  = xorsum + Str[i];
    }
    xorsum     = 0 - xorsum;
    *(Str + i)   = (unsigned char)(xorsum >> 8);
    *(Str + i + 1) = (unsigned char)(xorsum & 0x00ff);
}
/********************************************************************************************
 - 功能描述： 串口向外发送命令[包括控制和查询]
 - 隶属模块： 外部
 - 参数说明： CMD:表示控制指令，请查阅指令表，还包括查询的相关指令
              feedback:是否需要应答[0:不需要应答，1:需要应答]
              data:传送的参数
 - 返回说明：
 - 注：
********************************************************************************************/
void Spk_SendCMD(unsigned char CMD, unsigned char feedback, unsigned short dat)
{
    SpkSendCmd[0] = 0x7E;
    SpkSendCmd[1] = 0xff;    //保留字节
    SpkSendCmd[2] = 0x06;    //长度
    SpkSendCmd[3] = CMD;     //控制指令
    SpkSendCmd[4] = feedback;//是否需要反馈
    SpkSendCmd[5] = (unsigned char)(dat >> 8);//datah
    SpkSendCmd[6] = (unsigned char)(dat);     //datal
    SpkSendCmd[9] = 0xEF;
    DoSum(&SpkSendCmd[0], 8);       //校验
    rt_device_write(uart_device, 0, &SpkSendCmd, sizeof(SpkSendCmd));
}

rt_err_t uart_open(const char *name)
{
    rt_err_t res;

    /* 查找系统中的串口设备 */
    uart_device = rt_device_find(name);
    /* 查找到设备后将其打开 */
    if (uart_device != RT_NULL)
    {
        config.baud_rate = BAUD_RATE_9600;
        config.data_bits = DATA_BITS_8;
        config.stop_bits = STOP_BITS_1;
        config.bufsz = 128;
        config.parity = PARITY_NONE;
        config.bit_order = BIT_ORDER_LSB;
        rt_device_control(uart_device, RT_DEVICE_CTRL_CONFIG, &config);

        res = rt_device_set_rx_indicate(uart_device, uart_intput);
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("set %s rx indicate error.%d\n", name, res);
            return -RT_ERROR;
        }

        /* 打开设备，以可读写、中断方式 */
        res = rt_device_open(uart_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
        /* 检查返回值 */
        if (res != RT_EOK)
        {
            rt_kprintf("open %s device error.%d\n", name, res);
            return -RT_ERROR;
        }
//        rt_device_write(uart_device, 0, &FirstSong, sizeof(FirstSong));

    }
    else
    {
        rt_kprintf("can't find %s device.\n", name);
        return -RT_ERROR;
    }

    /* 初始化事件对象 */
    rt_event_init(&event, "event", RT_IPC_FLAG_FIFO);

    return RT_EOK;
}






