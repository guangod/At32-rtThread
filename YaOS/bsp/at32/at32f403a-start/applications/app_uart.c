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

/* ���ڽ����¼���־ */
#define UART_RX_EVENT (1 << 0)

/* �¼����ƿ� */
static struct rt_event event;
/* �����豸��� */
static rt_device_t uart_device = RT_NULL;
static struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

/* �ص����� */
static rt_err_t uart_intput(rt_device_t dev, rt_size_t size)
{
    /* �����¼� */
    rt_event_send(&event, UART_RX_EVENT);

    return RT_EOK;
}

rt_uint8_t uart_getchar(void)
{
    rt_uint32_t e;
    rt_uint8_t ch;

    /* ��ȡ1�ֽ����� */
    while (rt_device_read(uart_device, 0, &ch, 1) != 1)
    {
        /* �����¼� */
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
 - �������������У��
 - ����ģ�飺
 - ����˵����
 - ����˵����
 - ע��      ��У���˼·����
             ���͵�ָ�ȥ����ʼ�ͽ��������м��6���ֽڽ����ۼӣ����ȡ����
             ���ն˾ͽ����յ���һ֡���ݣ�ȥ����ʼ�ͽ��������м�������ۼӣ��ټ��Ͻ��յ���У��
             �ֽڡ��պ�Ϊ0.�����ʹ�����յ���������ȫ��ȷ��
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
 - ���������� �������ⷢ������[�������ƺͲ�ѯ]
 - ����ģ�飺 �ⲿ
 - ����˵���� CMD:��ʾ����ָ������ָ�����������ѯ�����ָ��
              feedback:�Ƿ���ҪӦ��[0:����ҪӦ��1:��ҪӦ��]
              data:���͵Ĳ���
 - ����˵����
 - ע��
********************************************************************************************/
void Spk_SendCMD(unsigned char CMD, unsigned char feedback, unsigned short dat)
{
    SpkSendCmd[0] = 0x7E;
    SpkSendCmd[1] = 0xff;    //�����ֽ�
    SpkSendCmd[2] = 0x06;    //����
    SpkSendCmd[3] = CMD;     //����ָ��
    SpkSendCmd[4] = feedback;//�Ƿ���Ҫ����
    SpkSendCmd[5] = (unsigned char)(dat >> 8);//datah
    SpkSendCmd[6] = (unsigned char)(dat);     //datal
    SpkSendCmd[9] = 0xEF;
    DoSum(&SpkSendCmd[0], 8);       //У��
    rt_device_write(uart_device, 0, &SpkSendCmd, sizeof(SpkSendCmd));
}

rt_err_t uart_open(const char *name)
{
    rt_err_t res;

    /* ����ϵͳ�еĴ����豸 */
    uart_device = rt_device_find(name);
    /* ���ҵ��豸����� */
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
        /* ��鷵��ֵ */
        if (res != RT_EOK)
        {
            rt_kprintf("set %s rx indicate error.%d\n", name, res);
            return -RT_ERROR;
        }

        /* ���豸���Կɶ�д���жϷ�ʽ */
        res = rt_device_open(uart_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
        /* ��鷵��ֵ */
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

    /* ��ʼ���¼����� */
    rt_event_init(&event, "event", RT_IPC_FLAG_FIFO);

    return RT_EOK;
}






