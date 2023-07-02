#include "keyboard.h"
#include "bflb_gpio.h"
#include "board.h"
#include "bflb_mtimer.h"

extern struct bflb_device_s *gpio;

uint8_t Key_row[1] = {0xff};   // 保存按键行扫描情况的状态数组

#define KEY_CLO0_OUT_LOW  bflb_gpio_reset(gpio, GPIO_PIN_27) 
#define KEY_CLO1_OUT_LOW  bflb_gpio_reset(gpio, GPIO_PIN_28) 
#define KEY_CLO2_OUT_LOW  bflb_gpio_reset(gpio, GPIO_PIN_29) 
#define KEY_CLO3_OUT_LOW  bflb_gpio_reset(gpio, GPIO_PIN_30) 

#define KEY_CLO0_OUT_HIGH  bflb_gpio_set(gpio, GPIO_PIN_27) 
#define KEY_CLO1_OUT_HIGH  bflb_gpio_set(gpio, GPIO_PIN_28) 
#define KEY_CLO2_OUT_HIGH  bflb_gpio_set(gpio, GPIO_PIN_29) 
#define KEY_CLO3_OUT_HIGH  bflb_gpio_set(gpio, GPIO_PIN_30) 

void Initial_KeyBoard(void)
{
    bflb_gpio_init(gpio, GPIO_PIN_27, GPIO_OUTPUT | GPIO_PULLDOWN | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_init(gpio, GPIO_PIN_28, GPIO_OUTPUT | GPIO_PULLDOWN | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_init(gpio, GPIO_PIN_29, GPIO_OUTPUT | GPIO_PULLDOWN | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_init(gpio, GPIO_PIN_30, GPIO_OUTPUT | GPIO_PULLDOWN | GPIO_SMT_EN | GPIO_DRV_0);
    
    bflb_gpio_init(gpio, GPIO_PIN_31, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_init(gpio, GPIO_PIN_32, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_init(gpio, GPIO_PIN_33, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_init(gpio, GPIO_PIN_25, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
}

/***
 *函数名：KEY_ROW_SCAN
 *功  能：按键行扫描
 *返回值：1~4，对应1~4行按键位置
 */
uint8_t KEY_ROW_SCAN(void)
{
    //读出行扫描状态
    Key_row[0] = bflb_gpio_read(gpio, GPIO_PIN_31) << 3;
    Key_row[0] = Key_row[0] | (bflb_gpio_read(gpio, GPIO_PIN_32) << 2);
    Key_row[0] = Key_row[0] | (bflb_gpio_read(gpio, GPIO_PIN_33) << 1);
    Key_row[0] = Key_row[0] | (bflb_gpio_read(gpio, GPIO_PIN_25));
    
    if(Key_row[0] != 0x0F)         //行扫描有变化，判断该列有按键按下
    {
      bflb_mtimer_delay_ms(10);                    //消抖
      if(Key_row[0] != 0x0F)
        {   
                //printf("Key_Row_DATA = 0x%x\r\n",Key_row[0]);
                switch(Key_row[0])
                {
                    case 0x07:         //0111 判断为该列第1行的按键按下
                        return 1;
                    case 0x0B:         //1011 判断为该列第2行的按键按下
                        return 2;
                    case 0x0D:         //1101 判断为该列第3行的按键按下
                        return 3;
                    case 0x0E:         //1110 判断为该列第4行的按键按下
                        return 4;
                    default :
                        return 0;
                }
        }
        else return 0;
    }
    else return 0;
}

/***
 *函数名：KEY_SCAN
 *功  能：4*4按键扫描
 *返回值：0~16，对应16个按键
 */
unsigned short int KEY_SCAN(void)
{    
    uint8_t Key_Num = 0;       //1-16对应的按键数
    uint8_t key_row_num = 0;        //行扫描结果记录
    
    KEY_CLO0_OUT_LOW;        
    if((key_row_num = KEY_ROW_SCAN()) != 0 )
    { 
        while(KEY_ROW_SCAN() != 0);  //消抖
        Key_Num = 0 + key_row_num;
    }
    KEY_CLO0_OUT_HIGH;
    
    KEY_CLO1_OUT_LOW;        
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    { 
        while(KEY_ROW_SCAN() != 0);
        Key_Num = 4 + key_row_num;
    }
    KEY_CLO1_OUT_HIGH;
    
    KEY_CLO2_OUT_LOW;    
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    { 
        while(KEY_ROW_SCAN() != 0);
        Key_Num = 8 + key_row_num;
    }
    KEY_CLO2_OUT_HIGH;
    
    KEY_CLO3_OUT_LOW;    
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    {
        while(KEY_ROW_SCAN() != 0);
        Key_Num = 12 + key_row_num;
    }
    KEY_CLO3_OUT_HIGH;

    /* Key_Value_Process */
    if(Key_Num == 0x01 | Key_Num == 0x02 | Key_Num == 0x03){
        Key_Num = Key_Num;
    }
    else if(Key_Num == 0x05 | Key_Num == 0x06 | Key_Num == 0x07){
        Key_Num = Key_Num - 1;
    }
    else if(Key_Num == 0x09 | Key_Num == 0x0a | Key_Num == 0x0b){
        Key_Num = Key_Num - 2;
    }
    else if(Key_Num == 0x0D){
        Key_Num = 0xF0;
    }
    else if(Key_Num == 0x04){
        Key_Num = 0X0A;
    }
    else if(Key_Num == 0x08){
        Key_Num = 0X0B;
    }
    else if(Key_Num == 0x0C){
        Key_Num = 0X0C;
    }
    else if(Key_Num == 0x10){
        Key_Num = 0X0D;
    }



    return Key_Num;
}

