/* Inclued Header File s*/
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

/* Camera All Define */
#define CAMERA_RESOLUTION_X (640)
#define CAMERA_RESOLUTION_Y (480)
#define CAMERA_FRAME_SIZE   (CAMERA_RESOLUTION_X * CAMERA_RESOLUTION_Y)
#define CAMERA_WRITE_ADDR   (0x26000000)
#define CAMERA_BUFFER_SIZE  (0x96000)

/* AI All Define */
#define PERSON_THRESHOLD (-20)

/* Onenet All Define */
#define ONENET_IP      "183.230.40.33"                   // the ip of onenent
#define ONENET_PORT    80                                // the Port number of onenet
#define DEVICEID      "1072729475"                       // the Device number of onenet
#define API_KEY       "4mz5btAO4gacvED8XTJACtVz=Y0="     // the own device of api-key
#define SSID          "nova75G"
#define PWD           "20020526l"

/* Cam and Mjpeg Config */
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

/* Lwip function  */
void lwip_onehttp_getpkt(char *pkt,char *key,char *devid);
uint32_t lwip_onehttp_postpkt(char *pkt, char *key, char *devid, char *dsid, char *val);

/* At All Defines */
#define UART_RECV_BUF_SZIE 64    //接收BUFF的大小
uint8_t recv_buf[UART_RECV_BUF_SZIE];
#define UART_SEND_BUF_SIZE 1024  //发送BUFF的大小
uint8_t tcp_buffer[UART_SEND_BUF_SIZE];
#define UART_RECV_BUFFER_SIZE 1024  //发送BUFF的大小
uint8_t recv_buffer[UART_RECV_BUFFER_SIZE] = { 0 };
uint32_t recv_len = 0;
struct at_client static_client;
int ret = 0;
char http_buf[10];

/* Onenet Flag bits */
uint8_t person_flag = 0;

int main(void)
{
    /* Variable declaration */
    int score = 0;
    uint8_t *picture;
    uint32_t pic_offset;
    uint32_t length;
    /* bsp platform init */
    bflb_platform_init(0);
    /* PWM Initial */ 
    pwm_dutycycle_config_t pwm_cfg = {3, 4};
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
    /* Psram Initial for the Camera and AI */ 
    bsp_sf_psram_init(1);
    /* AT Initial */
    at_client_init();
    /* AT using to connect the wifi */
    memset(&static_client, 0, sizeof(struct at_client));
    static_client.recv_line_buf = recv_buf;
    static_client.recv_bufsz = UART_RECV_BUF_SZIE;
    static_client.resp_succ = "OK";    
    static_client.resp_err = NULL;
    static_client.timeout = 2000;
    ret = at_exe_cmd(&static_client, "AT");
    ret = at_exe_cmd(&static_client, "AT+RST");
    bflb_platform_delay_ms(500);
    do{
        at_exe_cmd(&static_client, "AT+CWMODE=1");
    } while (static_client.resp_status<0);
    // ret = at_exe_cmd(&static_client, "AT+CWAUTOCONN=1");
    // ret = at_exe_cmd(&static_client, "AT+CIPRECVCFG=1");   //这种方式会照成配置不成功
    do{
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
    while(at_exe_cmd(&static_client, "AT+CIPSTART=0,\"TCP\",\"%s\",%d", ONENET_IP, ONENET_PORT)<0){
    }
    MSG("tcp connected\r\n");
    bflb_platform_delay_ms(1000);
    /* TFLite setup */ 
    MSG("Setup TFLite\r\n");
    setup();
    /* Image sensor Initial */
    if(SUCCESS != image_sensor_init(DISABLE, &camera_cfg, &mjpeg_cfg)){
        MSG("Init error!\n");
        BL_CASE_FAIL;
        while (1) {
        }
    }
    /* Cam Initial */
    struct device *cam0 = device_find("camera0");
    device_control(cam0, DEVICE_CTRL_RESUME, NULL);
    while (1) {
        /* Determine whether to acquire a frame */
        while(SUCCESS != cam_get_one_frame_interleave(&picture, &length)){
        }
        device_control(cam0, DEVICE_CTRL_SUSPEND, NULL);
        /* AI data preprocess */ 
        int8_t *scaled_img = image_proc(picture, length, CAMERA_RESOLUTION_X, CAMERA_RESOLUTION_Y, (int *)&pic_offset);
        loop(scaled_img, &score);
        printf("person score %d\r\n", score);
        /* Whether there is anyone's judgment */ 
        if(score > -20){
            person_flag = 1;
        }
        else{
            person_flag = 0;
        }
        device_control(cam0, DEVICE_CTRL_CAM_FRAME_DROP, NULL);
        device_control(cam0, DEVICE_CTRL_RESUME, NULL);
        /* AT post data */
        if(person_flag){
            sprintf((char *)http_buf, "person");
            /* Prepare AT post data */
            memset(tcp_buffer, 0, sizeof(tcp_buffer));
            lwip_onehttp_postpkt(tcp_buffer, API_KEY,DEVICEID, "device1_stream1", http_buf);
        }
        else{
            sprintf((char *)http_buf, "noperson");
            /* Prepare AT post data */
            memset(tcp_buffer, 0, sizeof(tcp_buffer));
            lwip_onehttp_postpkt(tcp_buffer, API_KEY,DEVICEID, "device1_stream1", http_buf);
        }
        static_client.resp_succ = "+CIPSTA";
        static_client.resp_err = "";
        static_client.timeout = 5000;
        ret=at_exe_cmd(&static_client, "AT+CIPSTART?");
        printf("resp_status:%d\r\n", static_client.resp_status);
        if(static_client.resp_status < 0){
            static_client.resp_succ = "CONNECTED";
            static_client.resp_err = "ERROR: Connect fail";
            static_client.timeout = 5000;
            while (at_exe_cmd(&static_client, "AT+CIPSTART=0,\"TCP\",\"%s\",%d",ONENET_IP,ONENET_PORT) < 0) {
            }
            MSG("tcp connected\r\n");
            bflb_platform_delay_ms(200);
        }    
        ret = at_write_recv(tcp_buffer, strlen(tcp_buffer), recv_buffer, &recv_len, 5000);
        memset(tcp_buffer, 0, sizeof(tcp_buffer));
        lwip_onehttp_getpkt(tcp_buffer, API_KEY, DEVICEID);
        ret = at_write_recv(tcp_buffer, strlen(tcp_buffer), recv_buffer, &recv_len, 5000);
        at_dump_hex(recv_buffer, recv_len);
        /* Recv onenet data to cotrol angle */
        if(strstr(recv_buffer, "angle0")){
            printf("0\r\n");
            pwm_dutycycle_config_t pwm_cfg = {3, 4};
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        else if(strstr(recv_buffer, "angle45")){
            printf("45\r\n");
            pwm_dutycycle_config_t pwm_cfg = {3, 5};
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        else if(strstr(recv_buffer, "angle90")){
            printf("90\r\n");
            pwm_dutycycle_config_t pwm_cfg = {3, 6};
            device_control(Control, DEVICE_CTRL_PWM_DUTYCYCLE_CONFIG, &pwm_cfg);
        }
        // MSG("*******************************************\r\n");
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
