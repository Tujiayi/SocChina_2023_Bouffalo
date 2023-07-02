/****************************************************************************
 * Included Files
 ****************************************************************************/
#include "bflb_mtimer.h"
#include "bflb_i2c.h"
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
#include "board.h"
#include "as608.h"
#include "finger_identify.h"
#include "keyboard.h"
#include "bflb_pwm_v2.h"
#include "bflb_clock.h"
#include "lcd_conf_user.h"
#include "lcd.h"
/****************************************************************************
 * All Define 
 ****************************************************************************/
volatile uint32_t wifi_strat_flag = 1;

/* LCD */
volatile uint8_t Lcd_Bufer[14];

/* PWM */
volatile uint8_t PWM_Out_Flag = 0;

/* WIFI ID PASSID */ 
//PASSID_Wifi:用户名；SSID_Wifi：密码
volatile uint8_t *PASSID_Wifi[30];
volatile uint8_t *SSID_Wifi[30];

/* Locker */
#define EEPROM_TRANSFER_LENGTH 8
#define EEPROM_SELECT_PAGE0    (0 << 5)

uint8_t Password[EEPROM_TRANSFER_LENGTH];
uint8_t Password_temp[EEPROM_TRANSFER_LENGTH];
uint8_t subaddr[2] = { 0x00, EEPROM_SELECT_PAGE0};

static struct bflb_device_s *i2c0;
struct bflb_i2c_msg_s msgs[2];
struct bflb_device_s *pwm;
struct bflb_device_s *gpio;
struct bflb_device_s *uartx;
volatile uint8_t uart1_rxbuf[40] = {0};
volatile uint8_t uart1_txbuf[40] = {0};
volatile uint32_t flag = 0;
uint32_t i = 0;
// 模块内有效指纹个数
uint16_t ValidN;
// 指纹模块AS608参数
SysPara AS608Para;
// Audio
volatile uint32_t Audio_flag = 0;
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define WIFI_STACK_SIZE     (1536)
#define TASK_PRIORITY_FW    (16)

/* LWIP_DEMO 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define LWIP_DMEO_TASK_PRIO     11         /* 任务优先级 */
#define LWIP_DMEO_STK_SIZE      15 * 1024    /* 任务堆栈大小 */
TaskHandle_t LWIP_Task_Handler;             /* 任务句柄 */
void lwip_demo_task(void *pvParameters);    /* 任务函数 */

/* LOCK_DEMO 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define LOCK_DMEO_TASK_PRIO     11         /* 任务优先级 */
#define LOCK_DMEO_STK_SIZE      4 * 1024    /* 任务堆栈大小 */
TaskHandle_t LOCK_Task_Handler;             /* 任务句柄 */
void Lock_demo_task(void *pvParameters);    /* 任务函数 */

/* PWM_DEMO 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define PWM_DMEO_TASK_PRIO     11         /* 任务优先级 */
#define PWM_DMEO_STK_SIZE      4 * 1024    /* 任务堆栈大小 */
TaskHandle_t PWM_Task_Handler;             /* 任务句柄 */
void PWM_demo_task(void *pvParameters);    /* 任务函数 */


extern void ble_tp_init();

wifi_mgmr_scan_params_t wifi_list;

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
 * @brief       PWM运行例程
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void PWM_demo_task(void *pvParameters)
{
    pvParameters = pvParameters;

    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_PIN_16, GPIO_FUNC_PWM0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);

    pwm = bflb_device_get_by_name("pwm_v2_0");

    /* period = .XCLK / .clk_div / .period = 40MHz / 40 / 20000 = 50Hz */
    struct bflb_pwm_v2_config_s cfg = {
        .clk_source = BFLB_SYSTEM_XCLK,
        .clk_div = 40,
        .period = 20000,
    };
    // Initial turn to the 0
    bflb_pwm_v2_init(pwm, &cfg);
    bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH0, 0, 500); /* duty = (1500-0)/20000 = 7.5% */
    bflb_pwm_v2_channel_positive_start(pwm, PWM_CH0);
    bflb_pwm_v2_start(pwm);


    while (1)
    {
        if(PWM_Out_Flag == 0){
            bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH0, 0, 500); /* duty = (1500-0)/20000 = 7.5% */
            bflb_pwm_v2_channel_positive_start(pwm, PWM_CH0);
            bflb_pwm_v2_start(pwm);
        }
        else if(PWM_Out_Flag == 1){
            bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH0, 0, 1000); /* duty = (1500-0)/20000 = 7.5% */
            bflb_pwm_v2_channel_positive_start(pwm, PWM_CH0);
            bflb_pwm_v2_start(pwm); 
        }
        else if(PWM_Out_Flag == 2){
            bflb_pwm_v2_channel_set_threshold(pwm, PWM_CH0, 0, 1500); /* duty = (1500-0)/20000 = 7.5% */
            bflb_pwm_v2_channel_positive_start(pwm, PWM_CH0);
            bflb_pwm_v2_start(pwm); 
        }   
        vTaskDelay(5);
    }
}


/**
 * @brief       LOCK运行例程
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */

void uart1_isr(int irq, void *arg)
{
    uint32_t intstatus = bflb_uart_get_intstatus(uartx);
    if (intstatus & UART_INTSTS_RTO) {
        while (bflb_uart_rxavailable(uartx)) {
            uart1_rxbuf[i++] = (uint8_t)(bflb_uart_getchar(uartx));
        }
        flag = 1;
        i = 0;
        bflb_uart_int_clear(uartx, UART_INTCLR_RTO);
    }
}

void Lock_demo_task(void *pvParameters)
{
    pvParameters = pvParameters;
    
    uint8_t ensure;
    uint8_t count_1 = 0;
    uint8_t count_2 = 0;
    uint16_t Key_value = 0;
    uint16_t num_1 = 0;
    uint8_t flag_unlock = 0;
    
    snprintf((char *)Lcd_Bufer, 14, "   BouFallo  ");
    lcd_draw_str_ascii16(11, 10, 0x0000, 0xffff, Lcd_Bufer, 14);

    // Initial GPIO for IIC
    board_i2c0_gpio_init();
    i2c0 = bflb_device_get_by_name("i2c0");
    bflb_i2c_init(i2c0, 400000);
    /* Write and read buffer init */
    for (size_t i = 0; i < EEPROM_TRANSFER_LENGTH; i++) {
        Password[i] = 0;
        Password_temp[i] = i + 1;
    }

    // Initial the iic   
    msgs[0].addr = 0x50;
    msgs[0].flags = I2C_M_NOSTOP;
    msgs[0].buffer = subaddr;
    msgs[0].length = 2;

    /* Write page 0 */
    // msgs[1].addr = 0x50;
    // msgs[1].flags = 0;
    // msgs[1].buffer = Password_temp;
    // msgs[1].length = EEPROM_TRANSFER_LENGTH;

    bflb_i2c_transfer(i2c0, msgs, 2);
    printf("write over\r\n");
    bflb_mtimer_delay_ms(100);

    /* Read page 0 */
    msgs[1].addr = 0x50;
    msgs[1].flags = I2C_M_READ;
    msgs[1].buffer = Password;
    msgs[1].length = EEPROM_TRANSFER_LENGTH;
    bflb_i2c_transfer(i2c0, msgs, 2);
    printf("read password over.\r\n");
    bflb_mtimer_delay_ms(100);

    for (uint8_t i = 0; i < EEPROM_TRANSFER_LENGTH; i++) {
        printf("check succeed, %d read: %02x\r\n", i, Password[i]);
    }
    printf("check over.\r\n");

    // Initial GPIO for LED and AS608
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_PIN_32, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(gpio, GPIO_PIN_32);
    bflb_gpio_init(gpio, GPIO_PIN_26, GPIO_INPUT | GPIO_PULLDOWN | GPIO_SMT_EN | GPIO_DRV_0);
    // Initial GPIO for KEY
    bflb_gpio_init(gpio, GPIO_PIN_33, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    // Initial GPIO for KEYBoard
    Initial_KeyBoard();

    board_uartx_gpio_init();
    uartx = bflb_device_get_by_name(DEFAULT_TEST_UART);
    struct bflb_uart_config_s cfg;
    cfg.baudrate = 115200;
    cfg.data_bits = UART_DATA_BITS_8;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.parity = UART_PARITY_NONE;
    cfg.flow_ctrl = 0;
    cfg.tx_fifo_threshold = 20;
    cfg.rx_fifo_threshold = 20;
    bflb_uart_init(uartx, &cfg);
    /* use tx end must stop tx freerun */
    // bflb_uart_feature_control(uartx, UART_CMD_SET_TX_FREERUN, false);
    
    bflb_uart_feature_control(uartx, UART_CMD_SET_RX_TRANSFER_LEN, 20);
    bflb_uart_feature_control(uartx, UART_CMD_SET_TX_END_INTERRUPT, false);
    bflb_uart_feature_control(uartx, UART_CMD_SET_RX_END_INTERRUPT, false);
    bflb_uart_rxint_mask(uartx, false);
    bflb_irq_attach(uartx->irq_num, uart1_isr, NULL);
    bflb_irq_enable(uartx->irq_num);

    // Shake Hand 
    while(PS_HandShake(&AS608Addr)){
        bflb_mtimer_delay_ms(400);
        printf("Connection is not Ok.\r\n");
        bflb_mtimer_delay_ms(800);
    }
    printf("Connection is Ok.\r\n");
    printf("New Address:%x\r\n", AS608Addr);
    // 读库指纹个数
    ensure = PS_ValidTempleteNum(&ValidN);
    if(ensure != 0x00){
        printf("%s\r\n", EnsureMessage(ensure));
    }
    // 读指纹参数
    ensure = PS_ReadSysPara(&AS608Para); 
    if(ensure == 0x00){
        printf("Capacity:%d Grade:%d\r\n", AS608Para.PS_max-ValidN, AS608Para.PS_level);
	}
    else{
        printf("%s\r\n", EnsureMessage(ensure));
    }

    while (1)
    {
        // LCD clear
        snprintf((char *)Lcd_Bufer, 14, "             ");
    	lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
		snprintf((char *)Lcd_Bufer, 14, "             ");
    	lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
        snprintf((char *)Lcd_Bufer, 14, "             ");
    	lcd_draw_str_ascii16(11, 100, 0x0000, 0xffff, Lcd_Bufer, 14);

        // Press figer to unlock.
        if(bflb_gpio_read(gpio, GPIO_PIN_26) == true){
            press_FR();
            vTaskDelay(1000);
            // Clear lcd, and go to normal.
            snprintf((char *)Lcd_Bufer, 14, "             ");
    		lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			snprintf((char *)Lcd_Bufer, 14, "             ");
    		lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
        }

        // Keyboard to control.
        Key_value = KEY_SCAN();

        // Review the input information.
        if(Key_value != 0X00){
            if(Key_value == 0x0E){
                Key_value = 0x00;
            }
            printf("Value:%x\r\n", Key_value);
        }
        // Pressing the * button will enter the password verification stage, so that the password entry operation is performed.
        if(Key_value == 0xF0){
            i = 0;
            printf("Please put in the password.\r\n");
            snprintf((char *)Lcd_Bufer, 14, "    Put In   ");
    		lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			snprintf((char *)Lcd_Bufer, 14, "   Password  ");
    		lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
            Audio_flag = 3;
            while(true){
		        num_1 = KEY_SCAN();
                if(num_1 != 0x00){
                    // Each time you press it, the LCD refreshes.
                    snprintf((char *)Lcd_Bufer, 14, "   ********  ");
                    lcd_draw_str_ascii16(10, 100, 0x0000, 0xffff, Lcd_Bufer, count_2 + 4);
                    if(num_1 == 0x0E){
                        num_1 = 0x00;
                    }
                    if(num_1 == Password[count_2]){  
                        count_1++;
                        printf("OK, %x.\r\n", num_1);
                    }
                    else if(num_1 == 0xF0){
                        // Press the * button again to determine whether the password is entered correctly.
                        vTaskDelay(1000);
                        if(count_1 == EEPROM_TRANSFER_LENGTH){
                            Audio_flag = 5;
                            snprintf((char *)Lcd_Bufer, 14, "             ");
                            lcd_draw_str_ascii16(11, 100, 0x0000, 0xffff, Lcd_Bufer, 14);
                            Add_FR();
                            snprintf((char *)Lcd_Bufer, 14, "             ");
    		                lcd_draw_str_ascii16(11, 100, 0x0000, 0xffff, Lcd_Bufer, 14);
                            snprintf((char *)Lcd_Bufer, 14, "     Add     ");
    		                lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			                snprintf((char *)Lcd_Bufer, 14, "  Successful ");
    		                lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
                            vTaskDelay(1000);
                        }
                        else{
                            printf("Password wrong.\r\n");
                            Audio_flag = 4;
                            snprintf((char *)Lcd_Bufer, 14, "   Password  ");
    		                lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			                snprintf((char *)Lcd_Bufer, 14, "    Wrong    ");
    		                lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
                            Audio_flag = 5;
                            vTaskDelay(1000);
                        }
                        break;
                    }
                    count_2++;
                }  
	        }
            count_1 = 0;
            count_2 = 0;
        }

        // Delete the FR
        else if(Key_value == 0x0F){
            i = 0;
            printf("Please put in the password.\r\n");
            snprintf((char *)Lcd_Bufer, 14, "    Put In   ");
    		lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			snprintf((char *)Lcd_Bufer, 14, "   Password  ");
    		lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
            Audio_flag = 3;
            while(true){
		        num_1 = KEY_SCAN();
                if(num_1 != 0x00){
                    // Each time you press it, the LCD refreshes.
                    snprintf((char *)Lcd_Bufer, 14, "   ********  ");
                    lcd_draw_str_ascii16(10, 100, 0x0000, 0xffff, Lcd_Bufer, count_2 + 4);
                    if(num_1 == 0x0E){
                        num_1 = 0x00;
                    }
                    if(num_1 == Password[count_2]){
                        count_1++;
                        printf("OK, %x.\r\n", num_1);
                    }
                    else if(num_1 == 0x0F){
                        vTaskDelay(1000);
                        if(count_1 == EEPROM_TRANSFER_LENGTH){
                            Audio_flag = 5;
                            snprintf((char *)Lcd_Bufer, 14, "             ");
    		                lcd_draw_str_ascii16(11, 100, 0x0000, 0xffff, Lcd_Bufer, 14);
                            Del_FR();
                            snprintf((char *)Lcd_Bufer, 14, "             ");
    		                lcd_draw_str_ascii16(11, 100, 0x0000, 0xffff, Lcd_Bufer, 14);
                            snprintf((char *)Lcd_Bufer, 14, "     Del     ");
    		                lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			                snprintf((char *)Lcd_Bufer, 14, "  Successful ");
    		                lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
                            vTaskDelay(1000);
                        }
                        else{
                            printf("Password wrong.\r\n");
                            Audio_flag = 4;
                            snprintf((char *)Lcd_Bufer, 14, "   Password  ");
    		                lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			                snprintf((char *)Lcd_Bufer, 14, "    Wrong    ");
    		                lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
                            vTaskDelay(1000);
                        }
                        break;
                    }
                    count_2++;
                }  
	        }
            count_1 = 0;
            count_2 = 0;
        }
        // put in the password for Lock
        else if(Key_value == 0x0A){
            i = 0;
            printf("Please put in the password.\r\n");
            snprintf((char *)Lcd_Bufer, 14, "    Put In   ");
    		lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			snprintf((char *)Lcd_Bufer, 14, "   Password  ");
    		lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
            Audio_flag = 3;
            while(true){
		        num_1 = KEY_SCAN();
                if(num_1 != 0x00){
                    // Each time you press it, the LCD refreshes.
                    snprintf((char *)Lcd_Bufer, 14, "   ********  ");
                    lcd_draw_str_ascii16(10, 100, 0x0000, 0xffff, Lcd_Bufer, count_2 + 4);
                    if(num_1 == 0x0E){
                        num_1 = 0x00;
                    }
                    if(num_1 == Password[count_2]){
                        count_1++;
                        printf("OK, %x.\r\n", num_1);
                    }
                    else if(num_1 == 0x0A){
                        vTaskDelay(1000);
                        if(count_1 == EEPROM_TRANSFER_LENGTH){
                            PWM_Out_Flag = 1;
                            printf("Unlock.\r\n");
                            snprintf((char *)Lcd_Bufer, 14, "   Unclock   ");
    		                lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
                            Audio_flag = 1;
                            vTaskDelay(1000);
                            PWM_Out_Flag = 0;
                        }
                        else{
                            PWM_Out_Flag = 0;
                            printf("Password wrong.\r\n");
                            snprintf((char *)Lcd_Bufer, 14, "   Password  ");
    		                lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			                snprintf((char *)Lcd_Bufer, 14, "    Wrong    ");
    		                lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
                            Audio_flag = 4;
                            vTaskDelay(1000);
                        }
                        break;
                    }
                    count_2++;
                }  
	        }
            count_1 = 0;
            count_2 = 0;
        }
        // Change the password
        else if (Key_value == 0x0B)
        {
            printf("First, please put in the passwprd.\r\n");
            snprintf((char *)Lcd_Bufer, 14, "    Put In   ");
    		lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			snprintf((char *)Lcd_Bufer, 14, "   Password  ");
    		lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
            Audio_flag = 3;
            i = 0;
            while(true){
		        num_1 = KEY_SCAN();
                if(num_1 != 0x00){
                    // Each time you press it, the LCD refreshes.
                    snprintf((char *)Lcd_Bufer, 14, "   ********  ");
                    lcd_draw_str_ascii16(10, 100, 0x0000, 0xffff, Lcd_Bufer, count_2 + 4);
                    if(num_1 == 0x0E){
                        num_1 = 0x00;
                    }
                    if(num_1 == Password[count_2]){
                        count_1++;
                        printf("OK, %x.\r\n", num_1);
                    }
                    else if(num_1 == 0x0B){
                        if(count_1 == EEPROM_TRANSFER_LENGTH){
                            Audio_flag = 5;
                            flag_unlock = 1;
                            printf("Password right.\r\n");
                            snprintf((char *)Lcd_Bufer, 14, "   Password  ");
    		                lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			                snprintf((char *)Lcd_Bufer, 14, "    Right    ");
    		                lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
                            Audio_flag = 5;
                            vTaskDelay(1000);
                        }
                        else{
                            flag_unlock = 0;
                            Audio_flag = 4;
                            printf("Password wrong.\r\n");
                            snprintf((char *)Lcd_Bufer, 14, "   Password  ");
    		                lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			                snprintf((char *)Lcd_Bufer, 14, "    Wrong    ");
    		                lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
                            Audio_flag = 4;
                            vTaskDelay(1000);
                        }
                        break;
                    }
                    count_2++;
                }
	        }
            count_1 = 0;
            count_2 = 0;
            if(flag_unlock == 1){
                printf("Please put in the new password.\r\n");
                snprintf((char *)Lcd_Bufer, 14, "   Put New   ");
    		    lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			    snprintf((char *)Lcd_Bufer, 14, "   Password  ");
    		    lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
                snprintf((char *)Lcd_Bufer, 14, "             ");
    		    lcd_draw_str_ascii16(11, 100, 0x0000, 0xffff, Lcd_Bufer, 14);
                Audio_flag = 3;
                while(true){
                    num_1 = KEY_SCAN();
                    if(num_1 != 0x00){
                        // Each time you press it, the LCD refreshes.
                        snprintf((char *)Lcd_Bufer, 14, "   ********  ");
                        lcd_draw_str_ascii16(10, 100, 0x0000, 0xffff, Lcd_Bufer, count_2 + 4);
                        if(num_1 == 0x0E){
                            num_1 = 0x00;
                        }   
                        if(num_1 == 0x0b){
                            if(count_1 == EEPROM_TRANSFER_LENGTH){
                                printf("Set ok.\r\n");
                                /* Write page 0 */
                                msgs[1].addr = 0x50;
                                msgs[1].flags = 0;
                                msgs[1].buffer = Password_temp;
                                msgs[1].length = EEPROM_TRANSFER_LENGTH;

                                bflb_i2c_transfer(i2c0, msgs, 2);
                                printf("write over\r\n");
                                bflb_mtimer_delay_ms(100);

                                /* Read page 0 */
                                msgs[1].addr = 0x50;
                                msgs[1].flags = I2C_M_READ;
                                msgs[1].buffer = Password;
                                msgs[1].length = EEPROM_TRANSFER_LENGTH;
                                bflb_i2c_transfer(i2c0, msgs, 2);
                                printf("read password over.\r\n");
                                bflb_mtimer_delay_ms(100);

                                snprintf((char *)Lcd_Bufer, 14, "    Change   ");
    		                    lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			                    snprintf((char *)Lcd_Bufer, 14, "  Successful ");
    		                    lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
                                vTaskDelay(1000);
                            }
                            else{
                                printf("Set fail.\r\n");
                                snprintf((char *)Lcd_Bufer, 14, "    Change   ");
    		                    lcd_draw_str_ascii16(11, 40, 0x0000, 0xffff, Lcd_Bufer, 14);
			                    snprintf((char *)Lcd_Bufer, 14, "     Fail    ");
    		                    lcd_draw_str_ascii16(11, 70, 0x0000, 0xffff, Lcd_Bufer, 14);
                                vTaskDelay(1000);
                            }
                            break;
                        }
                        else{
                            Password_temp[count_1] = num_1;
                            printf("OK, %x.\r\n", Password_temp[count_1]); 
                            snprintf((char *)Lcd_Bufer, 14, "   ********  ");
                        lcd_draw_str_ascii16(10, 100, 0x0000, 0xffff, Lcd_Bufer, count_1 + 4);
                            count_1++;
                        }
                    }
                }
                count_1 = 0;
            }
        }      
        Key_value = 0;
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

int main(void)
{
    board_init();
    lcd_init();
    lcd_clear(0xFFFF);

    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_PIN_32, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);

    tcpip_init(NULL, NULL);
    bflb_gpio_set(gpio, GPIO_PIN_32);

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

    /* 创建lwIP任务 */
    xTaskCreate((TaskFunction_t )lwip_demo_task,
                (const char*    )"lwip_demo_task",
                (uint16_t       )LWIP_DMEO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LWIP_DMEO_TASK_PRIO,
                (TaskHandle_t*  )&LWIP_Task_Handler);

    /* 创建lock任务 */
    xTaskCreate((TaskFunction_t )Lock_demo_task,
                (const char*    )"lock_demo_task",
                (uint16_t       )LOCK_DMEO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LOCK_DMEO_TASK_PRIO,
                (TaskHandle_t*  )&LOCK_Task_Handler);

    /* 创建pwm任务 */
    xTaskCreate((TaskFunction_t )PWM_demo_task,
                (const char*    )"PWM_demo_task",
                (uint16_t       )PWM_DMEO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )PWM_DMEO_TASK_PRIO,
                (TaskHandle_t*  )&PWM_Task_Handler);

    vTaskStartScheduler();

    while (1) {
    }
}