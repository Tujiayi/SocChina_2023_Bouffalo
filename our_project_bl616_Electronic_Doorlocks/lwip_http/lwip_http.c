#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>
#include <stdio.h>
#include <lwip/sockets.h>
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip_http.h"

/* 需要自己设置远程IP地址 */
#define IP_ADDR_ONENET              "183.230.40.33"
#define LWIP_ONENET_PORT            80                        /* 连接的本地端口号 */

#define IP_ADDR_APP                 "192.168.43.210"
#define LWIP_APP_PORT               8088                      /* 连接的本地端口号 */

/* 接收数据缓冲区 */
#define LWIP_DEMO_RX_BUFSIZE         1024                   /* 最大接收数据长度 */
uint8_t g_lwip_demo_recvbuf[LWIP_DEMO_RX_BUFSIZE] __attribute__((section(".psram_noinit"))); 
/* 发送数据内容 */
#define LWIP_DEMO_TX_BUFSIZE         1024
uint8_t g_lwip_demo_sendbuf[LWIP_DEMO_TX_BUFSIZE] __attribute__((section(".psram_noinit")));

/* 数据发送标志位 */
uint8_t g_lwip_send_flag;
int g_sock_onenet = -1,g_sock_picture = -1;
int g_lwip_connect_state_onenet = 0,g_lwip_connect_state_picture = 0;
uint8_t dma_finish=0;

uint8_t g_temp_rh[2];                                 /* g_temp_rh[0]是温度 g_temp_rh[1]是湿度 */
char g_tempStr[5];                                    /* 上传温度数值 */
char g_humiStr[5];                                    /* 上传湿度数值 */
uint16_t g_len;
char led_flag = 0;
extern uint8_t *pic_mjepg;
extern uint32_t jpeg_len;

extern uint8_t PWM_Out_Flag;

char voice[10]; 
extern uint32_t Audio_flag;
/*-------------------------------------------------*/
/*函数名：OneNet服务器 HTTP GET报文                */
/*参  数：device_id： 设备ID                       */
/*返回值：0：正确  其他：错误                      */
/*-------------------------------------------------*/
void lwip_onehttp_getpkt(char *pkt,char *key,char *devid)
{
	char temp[128];
	*pkt=0;

	memset(pkt,0,1024);   //清空缓冲区
    memset(temp,0,128);      //清空缓冲区                                             
	sprintf(pkt,"GET /devices/%s/datapoints HTTP/1.1\r\n",devid);//构建报文
	sprintf(temp,"api-key:%s\r\n",key);                             //构建报文
	strcat(pkt,temp);                                                //追加报文
	strcat(pkt,"Host:api.heclouds.com\r\n\r\n");                     //追加报文

}

/**
* @brief  OneNET HTTP数据包
* @param  pkt:保存数据包
* @param  key:密钥
* @param  devid:设备id
* @param  dsid:数据流名称
* @retval val:数据流数值
*/
uint32_t lwip_onehttp_postpkt(char *pkt, char *key, char *devid, char *dsid, char *val)
{
    char dataBuf[100] = {0};
    char lenBuf[10] = {0};
    *pkt = 0;

    sprintf(dataBuf, ",;%s,%s", dsid, val);     /* 采用分割字符串格式:type = 5  */
    sprintf(lenBuf, "%d", strlen(dataBuf));

    strcat(pkt, "POST /devices/");
    strcat(pkt, devid);
    strcat(pkt, "/datapoints?type=5 HTTP/1.1\r\n");

    strcat(pkt, "api-key:");
    strcat(pkt, key);
    strcat(pkt, "\r\n");

    strcat(pkt, "Host:api.heclouds.com\r\n");

    strcat(pkt, "Content-Length:");
    strcat(pkt, lenBuf);
    strcat(pkt, "\r\n\r\n");

    strcat(pkt, dataBuf);

    return strlen(pkt);
}

int i,index_write;
/**
 * @brief       lwip_onenet实验入口
 * @param       无
 * @retval      无
 */
void lwip_onenet(void)
{

    struct sockaddr_in atk_client_addr;
    err_t err;
    int recv_data_len;

    while (1)
    {
sock_start_onenet:
        g_lwip_connect_state_onenet = 0;
        g_sock_onenet = socket(AF_INET,SOCK_STREAM, 0);                 /* 可靠数据流交付服务既是TCP协议 */
        if(g_sock_onenet<0)
        {
            printf("Socket error\r\n");
            vTaskDelay(10);
            goto sock_start_onenet;
        }

        atk_client_addr.sin_family = AF_INET;                   /* 表示IPv4网络协议 */
        atk_client_addr.sin_port = htons(LWIP_ONENET_PORT);       /* 端口号 */
        atk_client_addr.sin_addr.s_addr = inet_addr(IP_ADDR_ONENET);   /* 远程IP地址 */
        memset(&(atk_client_addr.sin_zero), 0, sizeof(atk_client_addr.sin_zero));
        /* 连接远程IP地址 */
        err = connect(g_sock_onenet, (struct sockaddr *)&atk_client_addr, sizeof(struct sockaddr));
      
        if (err == -1)
        {
            printf("connect server failed\r\n");
            g_sock_onenet = -1;
            closesocket(g_sock_onenet);
            vTaskDelay(10);
            goto sock_start_onenet;
        }
        printf("connect server success\r\n");

        g_lwip_connect_state_onenet = 1;
        
        while (1)
        {
            // g_temp_rh[0] =i++;                                  /* 温度的数据 */
            // g_temp_rh[1] =i+10;                                /* 湿度的数据 */
            // if(i>80)
            //     i=1;
            // g_tempStr[0] = g_temp_rh[0] / 10 + 0x30;   /* 上传温度 */
            // g_tempStr[1] = g_temp_rh[0] % 10 + 0x30;

            // g_humiStr[0] = g_temp_rh[1] / 10 + 0x30;   /* 上传湿度 */
            // g_humiStr[1] = g_temp_rh[1] % 10 + 0x30;
            
            // if(index_write==0)
            // {
            //     g_len = lwip_onehttp_postpkt(g_lwip_demo_sendbuf, apikey, onenet_id, "temprature",g_tempStr);
            //     // printf("%s",g_lwip_demo_sendbuf);
            //     // printf("\r\n");
            //     printf("send temprature\r\n");
            //     write(g_sock_onenet, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));              /* 发送tcp_server_sentbuf中的数据 */
            //     index_write++;
            //     vTaskDelay(1000);
            // }
            // else if(index_write==1)
            // {
            //     g_len = lwip_onehttp_postpkt(g_lwip_demo_sendbuf, apikey, onenet_id, "humidirtty",g_humiStr);
            //     // printf("%s",g_lwip_demo_sendbuf);
            //     // printf("\r\n");
            //     printf("send humidirty\r\n");
            //     write(g_sock_onenet, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));              /* 发送tcp_server_sentbuf中的数据 */
            //     index_write++;
            //     vTaskDelay(5000);
            // }
            // else if(index_write==2)
            // {
            //     lwip_onehttp_getpkt(g_lwip_demo_sendbuf,apikey, onenet_id);
            //     write(g_sock_onenet, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf)); 
            //     printf("get switch statues\r\n");
            //     index_write=0;
            //     vTaskDelay(1000);
            // }

            if(Audio_flag==0)
            {
                if(dma_finish){
                    dma_finish=0;
                    sprintf(voice,"dma_finish");
                    g_len = lwip_onehttp_postpkt(g_lwip_demo_sendbuf, apikey, onenet_id, "device2_stream1",voice);
                    // printf("%s",g_lwip_demo_sendbuf);
                    // printf("\r\n");
                    // printf("Unlock successful\r\n");
                    vTaskDelay(1000);
                    write(g_sock_onenet, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));   
                }
                else{
                lwip_onehttp_getpkt(g_lwip_demo_sendbuf,apikey_device3, onenet_id_device3);
                write(g_sock_onenet, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf)); 
                printf("get door statues\r\n");
                vTaskDelay(40);
                }
            }

            if(Audio_flag==1)
            {
                sprintf(voice,"Audio_success");
                g_len = lwip_onehttp_postpkt(g_lwip_demo_sendbuf, apikey, onenet_id, "device2_stream1",voice);
                // printf("%s",g_lwip_demo_sendbuf);
                // printf("\r\n");
                printf("Unlock successful\r\n");
                write(g_sock_onenet, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));              /* 发送tcp_server_sentbuf中的数据 */
                Audio_flag=0;
                dma_finish=1;
                vTaskDelay(1000);
            }
            else if(Audio_flag==2)
            {
                sprintf(voice,"Audio_fail");
                g_len = lwip_onehttp_postpkt(g_lwip_demo_sendbuf, apikey, onenet_id, "device2_stream1",voice);
                // printf("%s",g_lwip_demo_sendbuf);
                // printf("\r\n");
                printf("Unlock failed\r\n");
                write(g_sock_onenet, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));              /* 发送tcp_server_sentbuf中的数据 */
                Audio_flag=0;
                dma_finish=1;
                vTaskDelay(1000);
            }
            else if(Audio_flag==3)
            {
                sprintf(voice,"Audio_please");
                g_len = lwip_onehttp_postpkt(g_lwip_demo_sendbuf, apikey, onenet_id, "device2_stream1",voice);
                // printf("%s",g_lwip_demo_sendbuf);
                // printf("\r\n");
                printf("Please enter the password\r\n");
                write(g_sock_onenet, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));              /* 发送tcp_server_sentbuf中的数据 */
                Audio_flag=0;
                dma_finish=1;
                vTaskDelay(1000);
            }
            else if(Audio_flag==4)
            {
                sprintf(voice,"Audio_error");
                g_len = lwip_onehttp_postpkt(g_lwip_demo_sendbuf, apikey, onenet_id, "device2_stream1",voice);
                // printf("%s",g_lwip_demo_sendbuf);
                // printf("\r\n");
                printf("Password error\r\n");
                write(g_sock_onenet, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));              /* 发送tcp_server_sentbuf中的数据 */
                Audio_flag=0;
                dma_finish=1;
                vTaskDelay(1000);
            }
            else if(Audio_flag==5)
            {
                sprintf(voice,"Audio_correct");
                g_len = lwip_onehttp_postpkt(g_lwip_demo_sendbuf, apikey, onenet_id, "device2_stream1",voice);
                // printf("%s",g_lwip_demo_sendbuf);
                // printf("\r\n");
                printf("password correct\r\n");
                write(g_sock_onenet, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));              /* 发送tcp_server_sentbuf中的数据 */
                Audio_flag=0;
                dma_finish=1;
                vTaskDelay(1000);
            }

            recv_data_len = recv(g_sock_onenet,g_lwip_demo_recvbuf,
                                 LWIP_DEMO_RX_BUFSIZE,0);
            if (recv_data_len <= 0 )
            {
                closesocket(g_sock_onenet);
                g_sock_onenet = -1;
                goto sock_start_onenet;
            }
            else
            {
                // printf("*************\r\n%s\r\n****************\r\n",g_lwip_demo_recvbuf);
                if(strstr((char *)g_lwip_demo_recvbuf,"door_close"))
                {
                    PWM_Out_Flag=1;
                    printf("close door\r\n");
                }
                else if(strstr((char *)g_lwip_demo_recvbuf,"door_open"))
                {   
                    PWM_Out_Flag=0;
                    printf("open door\r\n");
                }
            }

            closesocket(g_sock_onenet);
            g_sock_onenet = -1;
            goto sock_start_onenet;
        }
    }
}


// /**
//  * @brief       lwip_picture实验入口
//  * @param       无
//  * @retval      无
//  */
// void lwip_picture(void)
// {
//     struct sockaddr_in atk_client_addr;
//     err_t err;
//     int recv_data_len;

//     while (1)
//     {
// sock_start_picture:
//         g_lwip_connect_state_picture = 0;
//         g_sock_picture = socket(AF_INET,SOCK_DGRAM, 0);                 /* 可靠数据流交付服务既是TCP协议 */
//         if(g_sock_picture<0)
//         {
//             printf("Socket error\r\n");
//             vTaskDelay(10);
//             goto sock_start_picture;
//         }

//         atk_client_addr.sin_family = AF_INET;                   /* 表示IPv4网络协议 */
//         atk_client_addr.sin_port = htons(LWIP_APP_PORT);       /* 端口号 */
//         atk_client_addr.sin_addr.s_addr = inet_addr(IP_ADDR_APP);   /* 远程IP地址 */
//         memset(&(atk_client_addr.sin_zero), 0, sizeof(atk_client_addr.sin_zero));
//         /* 连接远程IP地址 */
//         err = connect(g_sock_picture, (struct sockaddr *)&atk_client_addr, sizeof(struct sockaddr));
      
//         if (err == -1)
//         {
//             printf("connect server failed\r\n");
//             g_sock_picture = -1;
//             closesocket(g_sock_picture);
//             vTaskDelay(10);
//             goto sock_start_picture;
//         }
//         printf("connect server success\r\n");

//         g_lwip_connect_state_picture = 1;
        
//         while (1)
//         {
       
//             //     send_pic_flag=1;
//             //     // for(pic_index=0;pic_index<jpeg_len;pic_index++)
//             //     // {
//             //     //     sprintf(pic_buff+pic_index ,"%c", pic_mjepg+pic_index);
//             //     // }
//                 // bflb_mjpeg_dump_hex(pic_mjepg,jpeg_len);
//             //    g_len = lwip_pic_postpkt(g_lwip_demo_sendbuf,pic_apikey, pic_dev_id,"image",jpeg_len);
//             //     // printf("%s",g_lwip_demo_sendbuf);
//                 printf("send picture\r\n");
//                 // write(g_sock_picture, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));              /* 发送tcp_server_sentbuf中的数据 */
//                 write(g_sock_picture,pic_mjepg,jpeg_len);
//                 printf("send picture success\r\n");
//                 // write(g_sock_picture,pic_buff, jpeg_len); 
//             //     //bflb_mjpeg_dump_hex(pic_mjepg,jpeg_len);
//             //     // index_write++;
//                 vTaskDelay(1000);
   

//             // recv_data_len = recv(g_sock_picture,g_lwip_demo_recvbuf,
//             //                      LWIP_DEMO_RX_BUFSIZE,0);
//             // if (recv_data_len <= 0 )
//             // {
//             //     closesocket(g_sock_picture);
//             //     g_sock_picture = -1;
//             //     goto sock_start_picture;
//             // }
           
//             // closesocket(g_sock_picture);
//             // g_sock_picture = -1;
//             // goto sock_start_picture;
//         }
//     }
// }





// void bflb_mjpeg_dump_hex(uint8_t *data, uint32_t len)
// {
//     uint32_t i = 0;

//     for (i = 0; i < len; i++) {
//         if (i % 16 == 0) {
//             printf("\r\n");
//         }

//         printf("%02X ", data[i]);
//     }

//     printf("\r\n");
// }
