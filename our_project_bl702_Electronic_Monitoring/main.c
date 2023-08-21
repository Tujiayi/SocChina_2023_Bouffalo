/**
 * @file main.c
 * @brief
 *
 * Copyright (c) 2021 Bouffalolab team
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 */
#include "bflb_platform.h"
#include "at.h"
#include "hal_pwm.h"
#include "hal_gpio.h"
#include "hal_clock.h"
#include "hal_dma.h"
#include "hal_mjpeg.h"
#include "bsp_sf_psram.h"
#include "bsp_image_sensor.h"
#include "image_proc.h"
#include "tensorflow/lite/micro/examples/person_detection/main_functions.h"

#define CAMERA_RESOLUTION_X (640)
#define CAMERA_RESOLUTION_Y (480)
#define CAMERA_FRAME_SIZE   (CAMERA_RESOLUTION_X * CAMERA_RESOLUTION_Y)
#define CAMERA_WRITE_ADDR   (0x26000000)
#define CAMERA_BUFFER_SIZE  (0x96000)

#define PERSON_THRESHOLD (-20)

static mjpeg_device_t mjpeg_cfg;
static cam_device_t camera_cfg = {
    .software_mode = CAM_MANUAL_MODE,
    .frame_mode = CAM_FRAME_INTERLEAVE_MODE,
    .yuv_format = CAM_YUV_FORMAT_YUV400_ODD,
    .cam_write_ram_addr = CAMERA_WRITE_ADDR,
    .cam_write_ram_size = CAMERA_BUFFER_SIZE,
    .cam_frame_size = CAMERA_FRAME_SIZE,

    .cam_write_ram_addr1 = 0,
    .cam_write_ram_size1 = 0,
    .cam_frame_size1 = 0,
};

void lwip_onehttp_getpkt(char *pkt,char *key,char *devid);
uint32_t lwip_onehttp_postpkt(char *pkt, char *key, char *devid, char *dsid, char *val);

#define ONENET_IP   "183.230.40.33"  //onenent的ip
#define ONENET_PORT   80     //onenet的端口号
#define DEVICEID    "1072729475"      //onenet的设备号
#define API_KEY     "4mz5btAO4gacvED8XTJACtVz=Y0="  //自己设备的api-key
#define SSID        "nova75G"
#define PWD         "20020526l"

#define UART_RECV_BUF_SZIE 64    //接收BUFF的大小
uint8_t recv_buf[UART_RECV_BUF_SZIE];
#define UART_SEND_BUF_SIZE 1024  //发送BUFF的大小
uint8_t tcp_buffer[UART_SEND_BUF_SIZE];

uint32_t recv_len = 0;
#define UART_RECV_BUFFER_SIZE 1024  //发送BUFF的大小
uint8_t recv_buffer[UART_RECV_BUFFER_SIZE] = { 0 };

struct at_client static_client;
int ret = 0;

char http_buf[10];

uint8_t person_flag = 0;

uint8_t temp1 = 2;
uint8_t temp2 = 2;
uint8_t temp3 = 2;
uint8_t temp4 = 2;
uint8_t Judge_Value = 0;

void Pid_init(void)
{
    gpio_set_mode(GPIO_PIN_20, GPIO_INPUT_MODE);
    gpio_set_mode(GPIO_PIN_22, GPIO_INPUT_MODE);
    gpio_set_mode(GPIO_PIN_7, GPIO_INPUT_MODE);
    gpio_set_mode(GPIO_PIN_8, GPIO_INPUT_MODE);
    MSG("gpio test !\r\n");
}

uint8_t Pir_Judge(uint8_t temp1, uint8_t temp2, uint8_t temp3, uint8_t temp4)
{
    // if(temp1 == 1 && temp2 == 0 && temp3 == 0 && temp4 == 0)
    // {
    //     printf("angle0\r\n");
    //     return 1;
    // }
    // else if(temp1 == 1 && temp2 == 1 && temp3 == 0 && temp4 == 0)
    // {
    //     printf("angle45\r\n");
    //     return 2;
    // }
    // else if(temp1 == 0 && temp2 == 1 && temp3 == 1 && temp4 == 0)
    // {
    //     printf("angle90\r\n");
    //     return 3;
    // }
    // else if(temp1 == 0 && temp2 == 0 && temp3 == 1 && temp4 == 1)
    // {
    //     printf("angle135\r\n");
    //     return 4;
    // }
    // else if(temp1 == 0 && temp2 == 0 && temp3 == 0 && temp4 == 1)
    // {
    //     printf("angle180\r\n");
    //     return 5;
    // }
    // else
    // {
    //     return 0;
    // }
    if(temp1 == 1 && temp2 == 0)
    {
        printf("angle0\r\n");
        return 1;
    }
    else if(temp1 == 1 && temp2 == 1)
    {
        printf("angle45\r\n");
        return 2;
    }
    else if(temp2 == 1 && temp3 == 1)
    {
        printf("angle90\r\n");
        return 3;
    }
    else if(temp3 == 1 && temp4 == 1)
    {
        printf("angle135\r\n");
        return 4;
    }
    else if(temp3 == 0 && temp4 == 1)
    {
        printf("angle180\r\n");
        return 5;
    }
    else
    {
        return 0;
    }
}

int main(void)
{
    int score = 0;
    uint8_t *picture;
    uint8_t *picture_lcd;
    uint32_t pic_offset;
    uint32_t length;

    uint16_t temp1, temp2, temp3, temp4; 
    bflb_platform_init(0);
    Pid_init();
    MSG("gpio test !\r\n");

    // PWM Initial
    pwm_dutycycle_config_t pwm_cfg = {
        3,
        4
    };
    pwm_register(PWM_CH1_INDEX, "Control");
    struct device *Control = device_find("Control");

    if (Control) {
        PWM_DEV(Control)->period = 40; //frequence = 32K/(31+1)/10 = 100HZ
        PWM_DEV(Control)->threshold_low = 3;
        PWM_DEV(Control)->threshold_high = 4;
        device_open(Control, DEVICE_OFLAG_STREAM_TX);
        pwm_channel_start(Control);
    }

    device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);

    bsp_sf_psram_init(1);

    at_client_init();

    memset(&static_client, 0, sizeof(struct at_client));

    static_client.recv_line_buf = recv_buf;
    static_client.recv_bufsz = UART_RECV_BUF_SZIE;

    static_client.resp_succ = "OK";    
    static_client.resp_err = NULL;
    static_client.timeout = 2000;

    ret = at_exe_cmd(&static_client, "AT");

    ret = at_exe_cmd(&static_client, "AT+RST");
    bflb_platform_delay_ms(500);
    do
    {
        at_exe_cmd(&static_client, "AT+CWMODE=1");
    } while (static_client.resp_status<0);
    // ret = at_exe_cmd(&static_client, "AT+CWAUTOCONN=1");
    // ret = at_exe_cmd(&static_client, "AT+CIPRECVCFG=1");   //这种方式会照成配置不成功
    do
    {
        at_exe_cmd(&static_client, "AT+CIPRECVCFG=1");
    } while (static_client.resp_status<0);

    static_client.resp_succ = "wifi connected";
    static_client.resp_err = NULL;
    static_client.timeout = 2000;

    while (at_exe_cmd(&static_client, "AT+CWJAP=\"%s\",\"%s\"", SSID, PWD) < 0) {
    }

    MSG("wifi connected success\r\n");

    bflb_platform_delay_ms(5000);  //足够的延时保证获得IP联网

    
    static_client.resp_succ = "CONNECTED";
    static_client.resp_err = "ERROR: Connect fail";
    static_client.timeout = 5000;

    while (at_exe_cmd(&static_client, "AT+CIPSTART=0,\"TCP\",\"%s\",%d",ONENET_IP,ONENET_PORT) < 0) {
    }

    MSG("tcp connected\r\n");
    bflb_platform_delay_ms(1000);

    // TFLite setup
    MSG("Setup TFLite\r\n");

    setup();

    if (SUCCESS != image_sensor_init(DISABLE, &camera_cfg, &mjpeg_cfg)) {
        MSG("Init error!\n");
        BL_CASE_FAIL;
        while (1) {
        }
    }
    struct device *cam0 = device_find("camera0");
    device_control(cam0, DEVICE_CTRL_RESUME, NULL);

    while (1) {

        MSG("*******************************************\r\n");
        temp1 = gpio_read(GPIO_PIN_20); 
        temp2 = gpio_read(GPIO_PIN_8);
        temp3 = gpio_read(GPIO_PIN_7);
        temp4 = gpio_read(GPIO_PIN_22);
        MSG("TEMP:%d %d %d %d\r\n", temp1, temp2, temp3, temp4);
        MSG("*******************************************\r\n");

        Judge_Value = Pir_Judge(temp1, temp2, temp3, temp4);
        if(Judge_Value == 1)
        {
            printf("angle0\r\n");
            pwm_dutycycle_config_t pwm_cfg = {
                3,
                4
            };
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        else if(Judge_Value == 2)
        {
            printf("angle45\r\n");
            pwm_dutycycle_config_t pwm_cfg = {
                3,
                5
            };
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        else if(Judge_Value == 3)
        {
            printf("angle90\r\n");
            pwm_dutycycle_config_t pwm_cfg = {
                3,
                6
            };
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        else if(Judge_Value == 4)
        {
            printf("angle135\r\n");
            pwm_dutycycle_config_t pwm_cfg = {
                3,
                7
            };
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        else if(Judge_Value == 5)
        {
            printf("angle180\r\n");
            pwm_dutycycle_config_t pwm_cfg = {
                3,
                8
            };
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        // 延时，为了适应PIR传感器的锁存时间。
        bflb_platform_delay_ms(500);

        while (SUCCESS != cam_get_one_frame_interleave(&picture, &length)) {
        }
        device_control(cam0, DEVICE_CTRL_SUSPEND, NULL);
        // UART_SendData(0, (uint8_t *)(picture), length);

        //AI data preprocess
        int8_t *scaled_img = image_proc(picture, length, CAMERA_RESOLUTION_X, CAMERA_RESOLUTION_Y, (int *)&pic_offset);

        loop(scaled_img, &score);

        printf("person score %d\r\n", score);
        
        if(score > -20){
            person_flag = 1;
        }
        else{
            person_flag = 0;
        }

        device_control(cam0, DEVICE_CTRL_CAM_FRAME_DROP, NULL);
        device_control(cam0, DEVICE_CTRL_RESUME, NULL);

        if(person_flag)
        {
            sprintf((char *)http_buf, "person");
            //准备POST发送数据
            memset(tcp_buffer, 0, sizeof(tcp_buffer));
            lwip_onehttp_postpkt(tcp_buffer, API_KEY,DEVICEID, "device1_stream1", http_buf);
        }
        else
        {
            sprintf((char *)http_buf,"noperson");
            //准备POST发送数据
            memset(tcp_buffer,0,sizeof(tcp_buffer));
            lwip_onehttp_postpkt(tcp_buffer, API_KEY,DEVICEID, "device1_stream1", http_buf);
        }

        static_client.resp_succ = "+CIPSTA";
        static_client.resp_err = "";
        static_client.timeout = 5000;
        ret=at_exe_cmd(&static_client,"AT+CIPSTART?");

        printf("resp_status:%d\r\n",static_client.resp_status);

        if (static_client.resp_status < 0)
        {
            static_client.resp_succ = "CONNECTED";
            static_client.resp_err = "ERROR: Connect fail";
            static_client.timeout = 5000;

            while (at_exe_cmd(&static_client, "AT+CIPSTART=0,\"TCP\",\"%s\",%d",ONENET_IP,ONENET_PORT) < 0) {
            }

            MSG("tcp connected\r\n");
            bflb_platform_delay_ms(200);
        }    
        ret = at_write_recv(tcp_buffer, strlen(tcp_buffer), recv_buffer, &recv_len, 5000);
        // at_dump_hex(recv_buffer, recv_len);
        memset(tcp_buffer, 0, sizeof(tcp_buffer));
        lwip_onehttp_getpkt(tcp_buffer,API_KEY, DEVICEID);
        ret = at_write_recv(tcp_buffer, strlen(tcp_buffer), recv_buffer, &recv_len, 5000);

        if(strstr(recv_buffer, "angle0"))
        {
            printf("angle0\r\n");
            pwm_dutycycle_config_t pwm_cfg = {
                3,
                4
            };
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        else if(strstr(recv_buffer, "angle45"))
        {
            printf("angle45\r\n");
            pwm_dutycycle_config_t pwm_cfg = {
                3,
                5
            };
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        else if(strstr(recv_buffer, "angle90"))
        {
            printf("angle90\r\n");
            pwm_dutycycle_config_t pwm_cfg = {
                3,
                6
            };
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        else if(strstr(recv_buffer, "angle135"))
        {
            printf("angle135\r\n");
            pwm_dutycycle_config_t pwm_cfg = {
                3,
                7
            };
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        else if(strstr(recv_buffer, "angle180"))
        {
            printf("angle180\r\n");
            pwm_dutycycle_config_t pwm_cfg = {
                3,
                8
            };
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }


    }

    BL_CASE_SUCCESS;
    while (1) {
        bflb_platform_delay_ms(100);
    }
}

/*-------------------------------------------------*/
/*函数名：OneNet服务器 HTTP GET报文                */
/*参  数：device_id： 设备ID                       */
/*返回值：0：正确  其他：错误                      */
/*-------------------------------------------------*/
void lwip_onehttp_getpkt(char *pkt,char *key,char *devid)
{
	char temp[128];
	*pkt=0;

	memset(pkt, 0, 1024);   //清空缓冲区
    memset(temp, 0, 128);      //清空缓冲区                                             
	sprintf(pkt, "GET /devices/%s/datapoints HTTP/1.1\r\n", devid);//构建报文
	sprintf(temp, "api-key:%s\r\n", key);                             //构建报文
	strcat(pkt, temp);                                                //追加报文
	strcat(pkt, "Host:api.heclouds.com\r\n\r\n");                     //追加报文

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
