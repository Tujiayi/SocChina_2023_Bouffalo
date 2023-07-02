/**
 ****************************************************************************************************
 * @file        lwip_demo.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-04
 * @brief       lwIP OneONET HTTPS 实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */
 
#ifndef _LWIP_HTTP_H
#define _LWIP_HTTP_H

/* 接收数据缓冲区 */
#define LWIP_DEMO_RX_BUFSIZE         1024                   /* 最大接收数据长度 */
extern uint8_t g_lwip_demo_recvbuf[LWIP_DEMO_RX_BUFSIZE] __attribute__((section(".psram_noinit"))); 
/* 发送数据内容 */
#define LWIP_DEMO_TX_BUFSIZE         1024
extern uint8_t g_lwip_demo_sendbuf[LWIP_DEMO_TX_BUFSIZE] __attribute__((section(".psram_noinit")));

/* 用户需要根据设备信息完善以下宏定义中的二元组内容 */
#define onenet_id  "1087860550"                         /* onenet产品的device ID */
#define apikey     "vZ1QC1r6OIWzAmuTAEM86jOa4rw="     /* onenet设备的device api */

#define onenet_id_device3  "1088217464"                         /* onenet产品的device ID */
#define apikey_device3     "vAvEA7PNvypB700jPQb1IKwMuds="     /* onenet设备的device api */

extern char led_flag,send_pic_flag;

void lwip_onenet(void);
void lwip_picture(void);

void bflb_mjpeg_dump_hex(uint8_t *data, uint32_t len);

#endif































