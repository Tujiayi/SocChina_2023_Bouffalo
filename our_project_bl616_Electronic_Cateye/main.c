/****************************************************************************
 * Included Files
 ****************************************************************************/
#include "bflb_mtimer.h"
#include "bflb_i2c.h"
#include "bflb_cam.h"
#include "image_sensor.h"
#include "lcd.h"
#include "image_trans.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include <stdint.h>
#include <stdio.h>
#include <netdb.h>
#include <lwip/tcpip.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "bl_fw_api.h"
#include "wifi_mgmr_ext.h"
#include "wifi_mgmr.h"
#include "bflb_dma.h"
#include "bflb_irq.h"
#include "bflb_uart.h"
#include "bflb_gpio.h"
#include "bl616_glb.h"
#include "rfparam_adapter.h"
#include "board.h"
#include "bluetooth.h"
#include "conn.h"
#include "conn_internal.h"
#if defined(BL702) || defined(BL602)
#include "ble_lib_api.h"
#elif defined(BL616)
#include "btble_lib_api.h"
#endif
#include "hci_driver.h"
#include "rfparam_adapter.h"
#include "hci_core.h"
#include "btble.h"
#include "bflb_gpio.h"
#define DBG_TAG "MAIN"
#include "log.h"
#include "lwip_http.h" 
#include "bflb_mjpeg.h"
#include "jpeg_head.h"
#include "bsp_es8388.h"
#include "fhm_onechannel_16k_20.h"
#include "bflb_l1c.h"
#include "bflb_i2s.h"
#include "bflb_clock.h"
/****************************************************************************
 * All Define 
 ****************************************************************************/
/* Cam Define */
#define CAM_FRAME_COUNT_USE   1
#define CROP_WQVGA_X        (240)
#define CROP_WQVGA_Y        (320)
#define CAM_BUFF_NUM        (4)
#define PERSON_THRESHOLD    (-20)
#define BLOCK_NUM           2
#define ROW_NUM             CROP_WQVGA_Y
#define CAM_FRAME_COUNT_USE 50
#define SIZE_BUFFER (4 * 1024 * 1024)
volatile uint32_t pic_count = 0;
volatile uint32_t wifi_strat_flag = 1;

/* JPEG for Image transmission */
uint8_t jpg_head_buf[800] = { 0 };
uint32_t jpg_head_len;
uint8_t MJPEG_QUALITY = 50;

/* WIFI ID PASSID */ 
volatile uint8_t *PASSID_Wifi[30];
volatile uint8_t *SSID_Wifi[30];

/* Aduio define */
struct bflb_device_s *i2s0;
struct bflb_device_s *dma0_ch0;
volatile uint32_t dma_tc_flag0 = 0;
volatile uint8_t dma_finish = 0;

static struct bflb_dma_channel_lli_pool_s tx_llipool[100];
static struct bflb_dma_channel_lli_transfer_s tx_transfers[1];

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define WIFI_STACK_SIZE     (1536)
#define TASK_PRIORITY_FW    (16)

/* LWIP_DEMO 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define LWIP_DMEO_TASK_PRIO     11         /* 任务优先级 */
#define LWIP_DMEO_STK_SIZE      2 * 1024    /* 任务堆栈大小 */
TaskHandle_t LWIP_Task_Handler;             /* 任务句柄 */
void lwip_demo_task(void *pvParameters);    /* 任务函数 */

/* LWIP_PICTURE 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define LWIP_PICTURE_TASK_PRIO     11         /* 任务优先级 */
#define LWIP_PICTURE_STK_SIZE      7 * 1024    /* 任务堆栈大小 */
TaskHandle_t LWIP_PICTURE_Task_Handler;             /* 任务句柄 */
void lwip_PICTURE_task(void *pvParameters);    /* 任务函数 */

/* CAM_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define CAM_TASK_PRIO           12         /* 任务优先级 */
#define CAM_STK_SIZE            1024 * 70        /* 任务堆栈大小 */
TaskHandle_t CamTask_Handler;               /* 任务句柄 */
void cam_task(void *pvParameters);          /* 任务函数 */

/* Audio_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define Audio_TASK_PRIO           11         /* 任务优先级 */
#define Audio_SIZE            1024 * 1        /* 任务堆栈大小 */
TaskHandle_t AudioTask_Handler;               /* 任务句柄 */
void audio_task(void *pvParameters);          /* 任务函数 */

/* Audio Config */
static ES8388_Cfg_Type ES8388Cfg = {
    .work_mode = ES8388_CODEC_MDOE,          /*!< ES8388 work mode */
    .role = ES8388_SLAVE,                    /*!< ES8388 role */
    .mic_input_mode = ES8388_DIFF_ENDED_MIC, /*!< ES8388 mic input mode */
    .mic_pga = ES8388_MIC_PGA_3DB,           /*!< ES8388 mic PGA */
    .i2s_frame = ES8388_LEFT_JUSTIFY_FRAME,  /*!< ES8388 I2S frame */
    .data_width = ES8388_DATA_LEN_16,        /*!< ES8388 I2S dataWitdh */
};

void dma0_ch0_isr(void *arg)
{
    // dma_tc_flag0++;
    bflb_dma_channel_stop(dma0_ch0);
    printf("tc done\r\n");
    dma_tc_flag0=0;
    dma_finish=1;
}

void i2s_gpio_init()
{
    struct bflb_device_s *gpio;

    gpio = bflb_device_get_by_name("gpio");
    /* I2S_FS */
    bflb_gpio_init(gpio, GPIO_PIN_1, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2S_DI */
    bflb_gpio_init(gpio, GPIO_PIN_10, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2S_DO */
    bflb_gpio_init(gpio, GPIO_PIN_3, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2S_BCLK */
    bflb_gpio_init(gpio, GPIO_PIN_0, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* MCLK CLKOUT */
    bflb_gpio_init(gpio, GPIO_PIN_2, GPIO_FUNC_CLKOUT | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    // /* I2C0_SDA */
    // bflb_gpio_init(gpio, GPIO_PIN_15, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    // /* I2C0_SCL */
    // bflb_gpio_init(gpio, GPIO_PIN_14, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
}

void i2s_dma_init()
{
    struct bflb_i2s_config_s i2s_cfg = {
        .bclk_freq_hz = 16000 * 32 * 2, /* bclk = Sampling_rate * frame_width * channel_num */
        .role = I2S_ROLE_MASTER,
        .format_mode = I2S_MODE_LEFT_JUSTIFIED,
        .channel_mode = I2S_CHANNEL_MODE_NUM_1,
        .frame_width = I2S_SLOT_WIDTH_32,
        .data_width = I2S_SLOT_WIDTH_16,
        .fs_offset_cycle = 0,

        .tx_fifo_threshold = 0,
        .rx_fifo_threshold = 0,
    };

    struct bflb_dma_channel_config_s tx_config = {
        .direction = DMA_MEMORY_TO_PERIPH,
        .src_req = DMA_REQUEST_NONE,
        .dst_req = DMA_REQUEST_I2S_TX,
        .src_addr_inc = DMA_ADDR_INCREMENT_ENABLE,
        .dst_addr_inc = DMA_ADDR_INCREMENT_DISABLE,
        .src_burst_count = DMA_BURST_INCR1,
        .dst_burst_count = DMA_BURST_INCR1,
        .src_width = DMA_DATA_WIDTH_16BIT,
        .dst_width = DMA_DATA_WIDTH_16BIT,
    };

    printf("i2s init\r\n");
    i2s0 = bflb_device_get_by_name("i2s0");
    /* i2s init */
    bflb_i2s_init(i2s0, &i2s_cfg);
    /* enable dma */
    bflb_i2s_link_txdma(i2s0, true);
    bflb_i2s_link_rxdma(i2s0, true);

    printf("dma init\r\n");
    dma0_ch0 = bflb_device_get_by_name("dma0_ch0");
    bflb_dma_channel_init(dma0_ch0, &tx_config);
    bflb_dma_channel_irq_attach(dma0_ch0, dma0_ch0_isr, NULL);

    tx_transfers[0].src_addr = (uint32_t)Voice_data1;
    tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
    tx_transfers[0].nbytes = sizeof(Voice_data1);

    printf("dma lli init\r\n");
    uint32_t num = bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 100, tx_transfers, 1);
    bflb_dma_channel_lli_link_head(dma0_ch0, tx_llipool, num);
    printf("dma lli num: %d \r\n", num);
    // bflb_dma_channel_start(dma0_ch0);
}

void mclk_out_init()
{
#ifdef BL616
    /* output MCLK,
    Will change the clock source of i2s,
    It needs to be called before i2s is initialized
    clock source 25M
    */
    GLB_Set_I2S_CLK(ENABLE, 2, GLB_I2S_DI_SEL_I2S_DI_INPUT, GLB_I2S_DO_SEL_I2S_DO_OUTPT);
    GLB_Set_Chip_Clock_Out3_Sel(GLB_CHIP_CLK_OUT_3_I2S_REF_CLK);
#endif
}

extern void ble_tp_init();

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct bflb_device_s *gpio;

/****************************************************************************
 * Private Data
 ****************************************************************************/

int wifi_statues =  -1;
static TaskHandle_t wifi_fw_task;
static wifi_conf_t conf =
{
    .country_code = "CN",
};

// extern void shell_init_with_task(struct bflb_device_s *shell);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Functions
 ****************************************************************************/

int wifi_start_firmware_task(void)
{
    LOG_I("Starting wifi ...\r\n");
    /* enable wifi clock */
    GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_IP_WIFI_PHY | GLB_AHB_CLOCK_IP_WIFI_MAC_PHY | GLB_AHB_CLOCK_IP_WIFI_PLATFORM);
    GLB_AHB_MCU_Software_Reset(GLB_AHB_MCU_SW_WIFI);

    /* set ble controller EM Size */

    GLB_Set_EM_Sel(GLB_WRAM160KB_EM0KB);

    if (0 != rfparam_init(0, NULL, 0)) {
        LOG_I("PHY RF init failed!\r\n");
        return 0;
    }

    LOG_I("PHY RF init success!\r\n");

    /* Enable wifi irq */

    extern void interrupt0_handler(void);
    bflb_irq_attach(WIFI_IRQn, (irq_callback)interrupt0_handler, NULL);
    bflb_irq_enable(WIFI_IRQn);
    xTaskCreate(wifi_main, (char *)"fw", WIFI_STACK_SIZE, NULL, TASK_PRIORITY_FW, &wifi_fw_task);
    return 0;
}

void wifi_event_handler(uint32_t code)
{
    switch (code) {
        case CODE_WIFI_ON_INIT_DONE:
        {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_INIT_DONE\r\n", __func__);
            wifi_mgmr_init(&conf);
        }
        break;
        case CODE_WIFI_ON_MGMR_DONE:
        {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_MGMR_DONE\r\n", __func__);
            do{
                wifi_statues = wifi_mgmr_sta_quickconnect(PASSID_Wifi, SSID_Wifi, 2442, 5000);
            }while(wifi_statues!=0);
        }
        break;
        case CODE_WIFI_ON_SCAN_DONE:
        {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_SCAN_DONE\r\n", __func__);
            wifi_mgmr_sta_scanlist();
        }
        break;
        case CODE_WIFI_ON_CONNECTED:
        {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_CONNECTED\r\n", __func__);
            void mm_sec_keydump();
            mm_sec_keydump();
        }
        break;
        case CODE_WIFI_ON_GOT_IP:
        {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_GOT_IP\r\n", __func__);
        }
        break;
        case CODE_WIFI_ON_DISCONNECT:
        {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_DISCONNECT\r\n", __func__);
        }
        break;
        case CODE_WIFI_ON_AP_STARTED:
        {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_AP_STARTED\r\n", __func__);
        }
        break;
        case CODE_WIFI_ON_AP_STOPPED:
        {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_AP_STOPPED\r\n", __func__);
        }
        break;
        case CODE_WIFI_ON_AP_STA_ADD:
        {
            LOG_I("[APP] [EVT] [AP] [ADD] %lld\r\n", xTaskGetTickCount());
        }
        break;
        case CODE_WIFI_ON_AP_STA_DEL:
        {
            LOG_I("[APP] [EVT] [AP] [DEL] %lld\r\n", xTaskGetTickCount());
        }
        break;
        default:
        {
            LOG_I("[APP] [EVT] Unknown code %u \r\n", code);
        }
    }
}

/**
 * @brief       lwIP运行例程
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void lwip_demo_task(void *pvParameters)
{
    pvParameters = pvParameters;
    while (1)
    {
        if(wifi_strat_flag == 0)
        {
            bt_le_adv_stop();
            wifi_start_firmware_task();
            break;
        }
    }
    vTaskDelay(7000);
    lwip_onenet();
    while (1)
    {
        vTaskDelay(5);
    }
}

/**
 * @brief       lwIP_PICTURE运行例程
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void lwip_PICTURE_task(void *pvParameters)
{
    pvParameters = pvParameters;
    while (1)
    {
        if(wifi_strat_flag == 0)
        {
            // bt_le_adv_stop();
            // wifi_start_firmware_task();
            break;
        }
    }
    vTaskDelay(7000);
    lwip_picture();
    while (1)
    {
        vTaskDelay(5);
    }
}

static int btblecontroller_em_config(void)
{
    extern uint8_t __LD_CONFIG_EM_SEL;
    volatile uint32_t em_size;
    em_size = (uint32_t)&__LD_CONFIG_EM_SEL;
    if (em_size == 0) {
        GLB_Set_EM_Sel(GLB_WRAM160KB_EM0KB);
    } else if (em_size == 32 * 1024) {
        GLB_Set_EM_Sel(GLB_WRAM128KB_EM32KB);
    } else if (em_size == 64 * 1024) {
        GLB_Set_EM_Sel(GLB_WRAM96KB_EM64KB);
    } else {
        GLB_Set_EM_Sel(GLB_WRAM96KB_EM64KB);
    }
    return 0;
}

static void ble_connected(struct bt_conn *conn, u8_t err)
{
    if(err || conn->type != BT_CONN_TYPE_LE)
    {
        return;
    }
    printf("%s",__func__);
}

static void ble_disconnected(struct bt_conn *conn, u8_t reason)
{  
    int ret;

    if(conn->type != BT_CONN_TYPE_LE)
    {
        return;
    }

    printf("%s",__func__);

    // enable adv
    ret = set_adv_enable(true);
    if(ret) {
        printf("Restart adv fail. \r\n");
    }
}

static struct bt_conn_cb ble_conn_callbacks = {
	.connected	=   ble_connected,
	.disconnected	=   ble_disconnected,
};

static void ble_start_adv(void)
{
    struct bt_le_adv_param param;
    int err = -1;
    struct bt_data adv_data[1] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR | BT_LE_AD_GENERAL)
    };
    struct bt_data adv_rsp[1] = {
        BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "BL616")
    };

    memset(&param, 0, sizeof(param));
    // Set advertise interval
    param.interval_min = BT_GAP_ADV_FAST_INT_MIN_2;
    param.interval_max = BT_GAP_ADV_FAST_INT_MAX_2;
    /*Get adv type, 0:adv_ind,  1:adv_scan_ind, 2:adv_nonconn_ind 3: adv_direct_ind*/
    param.options = (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_ONE_TIME); 

    err = bt_le_adv_start(&param, adv_data, ARRAY_SIZE(adv_data), adv_rsp, ARRAY_SIZE(adv_rsp));
    if(err){
        printf("Failed to start advertising (err %d) \r\n", err);
    }
    printf("Start advertising success.\r\n");
}


void bt_enable_cb(int err)
{
    if (!err) {
        bt_addr_le_t bt_addr;
        bt_get_local_public_address(&bt_addr);
        printf("BD_ADDR:(MSB)%02x:%02x:%02x:%02x:%02x:%02x(LSB) \r\n",
            bt_addr.a.val[5], bt_addr.a.val[4], bt_addr.a.val[3], bt_addr.a.val[2], bt_addr.a.val[1], bt_addr.a.val[0]);

        bt_conn_cb_register(&ble_conn_callbacks);
        ble_tp_init();

        // start advertising
        ble_start_adv();
    }
}

static struct bflb_device_s *mjpeg;
uint32_t jpeg_len;
uint8_t *pic_mjepg=(uint8_t *)(BFLB_PSRAM_BASE + 4*1024*1024-150*1024);

void mjpeg_isr(int irq, void *arg)
{

    uint32_t intstatus = bflb_mjpeg_get_intstatus(mjpeg);
    if (intstatus & MJPEG_INTSTS_ONE_FRAME) {
        bflb_mjpeg_int_clear(mjpeg, MJPEG_INTCLR_ONE_FRAME);
        jpeg_len = bflb_mjpeg_get_frame_info(mjpeg, &pic_mjepg);
        pic_count++;
        // bflb_mjpeg_dump_hex(pic_mjepg,jpeg_len);
        bflb_mjpeg_pop_one_frame(mjpeg);

    }
}

// static uint8_t picture[CROP_WQVGA_X * CROP_WQVGA_Y * CAM_BUFF_NUM] ATTR_NOINIT_PSRAM_SECTION __attribute__((aligned(64)));
// static uint8_t tmp_pic[CROP_WQVGA_X * CROP_WQVGA_Y * (CAM_BUFF_NUM / 2)] ATTR_NOINIT_PSRAM_SECTION __attribute__((aligned(64)));
// static uint8_t tmp_pic1[CROP_WQVGA_X * CROP_WQVGA_Y * (CAM_BUFF_NUM / 2)] ATTR_NOINIT_PSRAM_SECTION __attribute__((aligned(64)));
    
/**
 * @brief       Cam
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */

void cam_task(void *pvParameters)
{
    pvParameters = pvParameters;

    uint32_t i, pic_size;
    uint32_t j = 0;

    uint8_t *pic;
    static uint8_t picture[CROP_WQVGA_X * CROP_WQVGA_Y * CAM_BUFF_NUM] ATTR_NOINIT_PSRAM_SECTION __attribute__((aligned(64)));
    static uint8_t tmp_pic[CROP_WQVGA_X * CROP_WQVGA_Y * (CAM_BUFF_NUM / 2)] ATTR_NOINIT_PSRAM_SECTION __attribute__((aligned(64)));
    static uint8_t tmp_pic1[CROP_WQVGA_X * CROP_WQVGA_Y * (CAM_BUFF_NUM / 2)] ATTR_NOINIT_PSRAM_SECTION __attribute__((aligned(64)));

    static struct bflb_device_s *i2c0;
    static struct bflb_device_s *cam0;

    struct bflb_cam_config_s cam_config;
    struct image_sensor_config_s *sensor_config;

    lcd_init();
    board_i2c0_gpio_init();
    board_dvp_gpio_init();

    lcd_clear(0x0000);
    i2c0 = bflb_device_get_by_name("i2c0");
    cam0 = bflb_device_get_by_name("cam0");

    if (image_sensor_scan(i2c0, &sensor_config)) {
        printf("\r\nSensor name: %s\r\n", sensor_config->name);
    } else {
        printf("\r\nError! Can't identify sensor!\r\n");
        while (1) {
        }
    }

    /* Crop resolution_x, should be set before init */
    bflb_cam_crop_hsync(cam0, 112, 112 + CROP_WQVGA_X);
    /* Crop resolution_y, should be set before init */
    bflb_cam_crop_vsync(cam0, 120, 120 + CROP_WQVGA_Y);

    memcpy(&cam_config, sensor_config, IMAGE_SENSOR_INFO_COPY_SIZE);
    cam_config.with_mjpeg = true;
    cam_config.input_source = CAM_INPUT_SOURCE_DVP;
    cam_config.output_format = CAM_OUTPUT_FORMAT_AUTO;
    cam_config.output_bufaddr = (uint32_t)picture;
    cam_config.output_bufsize = CROP_WQVGA_X * CROP_WQVGA_Y * (CAM_BUFF_NUM / 2);

    bflb_cam_init(cam0, &cam_config);
    bflb_cam_start(cam0);

    mjpeg = bflb_device_get_by_name("mjpeg");

    struct bflb_mjpeg_config_s config;

    config.format = MJPEG_FORMAT_YUV422_YUYV;
    config.quality = MJPEG_QUALITY;
    config.rows = ROW_NUM;
    config.resolution_x = CROP_WQVGA_X;
    config.resolution_y = CROP_WQVGA_Y;
    config.input_bufaddr0 = (uint32_t)picture;
    config.input_bufaddr1 = 0;
    config.output_bufaddr = (uint32_t)picture + CROP_WQVGA_X * 2 * ROW_NUM;
    config.output_bufsize = 150 * 1024;
    config.input_yy_table = NULL; /* use default table */
    config.input_uv_table = NULL; /* use default table */

    bflb_mjpeg_init(mjpeg, &config);

    jpg_head_len = JpegHeadCreate(YUV_MODE_422, MJPEG_QUALITY, CROP_WQVGA_X, CROP_WQVGA_Y, jpg_head_buf);
    bflb_mjpeg_fill_jpeg_header_tail(mjpeg, jpg_head_buf, jpg_head_len);

    bflb_mjpeg_tcint_mask(mjpeg, false);
    bflb_irq_attach(mjpeg->irq_num, mjpeg_isr, NULL);
    bflb_irq_enable(mjpeg->irq_num);

    bflb_mjpeg_start(mjpeg);

    // 显示旋转180度
    lcd_set_dir(2,0);

    while (1)
    {
        if(bflb_cam_get_frame_count(cam0) > 0){     
            bflb_cam_stop(cam0);
            pic_size = bflb_cam_get_frame_info(cam0, &pic);
            bflb_cam_pop_one_frame(cam0);
            // printf("pop picture %d: 0x%08x, len: %d\r\n", j, (uint32_t)pic, pic_size);
            j++;

            YUYV_to_RGB565(pic, tmp_pic1);

            //高低字节反序
            for (i = 0; i < pic_size; i += 2) {
                    tmp_pic[i] = tmp_pic1[i];
                    tmp_pic[i+1] = tmp_pic1[i + 1];
                    tmp_pic[i+1]=(tmp_pic[i+1]&0x07)|((tmp_pic1[i]&0x1f)<<3);
                    tmp_pic[i]=(tmp_pic[i]&0xe0)|((tmp_pic1[i+1]&0xf8)>>3);
            }
            while (lcd_draw_is_busy()) {};
            lcd_set_dir(2, 0);
            lcd_draw_picture_nonblocking(0, 0, CROP_WQVGA_X - 1, CROP_WQVGA_Y - 1, tmp_pic);
            bflb_cam_start(cam0);
        }
        vTaskDelay(10);
    }
}

void audio_task(void *pvParameters)
{
    pvParameters = pvParameters;
    printf("\n\ri2s dma test\n\r");
    /* gpio init */
    i2s_gpio_init();

    /* init ES8388 Codec */
    printf("es8388 init\n\r");
    ES8388_Init(&ES8388Cfg);
    ES8388_Set_Voice_Volume(70);

    /* i2s init */
    i2s_dma_init();

    /* enable i2s tx and rx */
    bflb_i2s_feature_control(i2s0, I2S_CMD_DATA_ENABLE, I2S_CMD_DATA_ENABLE_TX | I2S_CMD_DATA_ENABLE_RX);

    printf("test end\n\r");
    while (1)
    {
        // The unlock is successful.
        if(dma_tc_flag0 == 1)
        {
            tx_transfers[0].src_addr = (uint32_t)Voice_data1;
            tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
            tx_transfers[0].nbytes = sizeof(Voice_data1);

            printf("dma lli init\r\n");
            uint32_t num = bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 100, tx_transfers, 1);
            bflb_dma_channel_lli_link_head(dma0_ch0, tx_llipool, num);
            printf("88888888888888888888888888888\r\n");
            printf("dma lli num: %d \r\n", num);
            printf("88888888888888888888888888888\r\n");
            bflb_dma_channel_start(dma0_ch0);
        }
        // Unlock failed.
        else if(dma_tc_flag0 == 2)
        {
            tx_transfers[0].src_addr = (uint32_t)Voice_data2;
            tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
            tx_transfers[0].nbytes = sizeof(Voice_data2);

            printf("dma lli init\r\n");
            uint32_t num = bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 100, tx_transfers, 1);
            bflb_dma_channel_lli_link_head(dma0_ch0, tx_llipool, num);
            printf("dma lli num: %d \r\n", num);
            bflb_dma_channel_start(dma0_ch0);
        }
        // Please enter a password.
        else if(dma_tc_flag0 == 3)
        {
            tx_transfers[0].src_addr = (uint32_t)Voice_data3;
            tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
            tx_transfers[0].nbytes = sizeof(Voice_data3);

            printf("dma lli init\r\n");
            uint32_t num = bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 100, tx_transfers, 1);
            bflb_dma_channel_lli_link_head(dma0_ch0, tx_llipool, num);
            printf("dma lli num: %d \r\n", num);
            bflb_dma_channel_start(dma0_ch0);
        }
        // The password is correct.
        else if(dma_tc_flag0 == 4)
        {
            tx_transfers[0].src_addr = (uint32_t)Voice_data4;
            tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
            tx_transfers[0].nbytes = sizeof(Voice_data4);

            printf("dma lli init\r\n");
            uint32_t num = bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 100, tx_transfers, 1);
            bflb_dma_channel_lli_link_head(dma0_ch0, tx_llipool, num);
            printf("dma lli num: %d \r\n", num);
            bflb_dma_channel_start(dma0_ch0);
        }
        // The password is incorrect.
        else if(dma_tc_flag0 == 5)
        {
            tx_transfers[0].src_addr = (uint32_t)Voice_data5;
            tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
            tx_transfers[0].nbytes = sizeof(Voice_data5);

            printf("dma lli init\r\n");
            uint32_t num = bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 100, tx_transfers, 1);
            bflb_dma_channel_lli_link_head(dma0_ch0, tx_llipool, num);
            printf("dma lli num: %d \r\n", num);
            bflb_dma_channel_start(dma0_ch0);
        }
        while(dma_tc_flag0 != 0)
        {}
        vTaskDelay(100);
    }
}

int main(void)
{
    board_init();
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_PIN_32, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);

    tcpip_init(NULL, NULL);
    bflb_gpio_set(gpio, GPIO_PIN_32);
    // wifi_start_firmware_task();
    configASSERT((configMAX_PRIORITIES > 4));

    /* set ble controller EM Size */
    btblecontroller_em_config();

    /* Init rf */
    if (0 != rfparam_init(0, NULL, 0)) {
        printf("PHY RF init failed!\r\n");
        return 0;
    }

     // Initialize BLE controller
    #if defined(BL702) || defined(BL602)
    ble_controller_init(configMAX_PRIORITIES - 1);
    #else
    btble_controller_init(configMAX_PRIORITIES - 1);
    #endif
    // Initialize BLE Host stack
    hci_driver_init();
    bt_enable(bt_enable_cb);

    /* 创建lwIP_picture任务 */
    xTaskCreate((TaskFunction_t )lwip_PICTURE_task,
                (const char*    )"lwip_PICTURE_task",
                (uint16_t       )LWIP_PICTURE_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LWIP_PICTURE_TASK_PRIO,
                (TaskHandle_t*  )&LWIP_PICTURE_Task_Handler);

    /* 创建lwIP任务 */
    xTaskCreate((TaskFunction_t )lwip_demo_task,
                (const char*    )"lwip_demo_task",
                (uint16_t       )LWIP_DMEO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LWIP_DMEO_TASK_PRIO,
                (TaskHandle_t*  )&LWIP_Task_Handler);

    /* CAM测试任务 */
    xTaskCreate((TaskFunction_t )cam_task,
                (const char*    )"cam_task",
                (uint16_t       )CAM_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )CAM_TASK_PRIO,
                (TaskHandle_t*  )&CamTask_Handler);

    /* Audio测试任务 */
    xTaskCreate((TaskFunction_t )audio_task,
                (const char*    )"audio_task",
                (uint16_t       )Audio_SIZE,
                (void*          )NULL,
                (UBaseType_t    )Audio_TASK_PRIO,
                (TaskHandle_t*  )&AudioTask_Handler);

    vTaskStartScheduler();

    while (1) {
    }
}