# 简介
    本项目名称为“基于博流开发平台的智能安防系统设计——智能监控”。其基于博流开发平台，使用开发板BL706avb完成系统开发。该项目囊括了是否有人的判断，以及利用OneNet平台作为中介，完成摄像头角度旋转信息的下发以及安防信息的上传。

# 如何使用
    将本项目代码利用git工具git clone至本地后将其移动至博流官方git仓库中的SDK的example文件夹下后，即可使用。
    因本开发板基于release_v1.4.5分支开发，故在git clone时请注意分支。
**博流仓库地址：https://github.com/bouffalolab/bouffalo_sdk**
    
# 详细介绍
## 引脚使用情况
    在博流官方git仓库的BOUFFALO_SDK分支release_v1.4.5代码文件夹bsp\board\bl702\bl706_avb下打开pinmux_config.h文件进行引脚配置的修改。
```
#define CONFIG_GPIO0_FUNC GPIO_FUN_CAM
#define CONFIG_GPIO1_FUNC GPIO_FUN_CAM
#define CONFIG_GPIO2_FUNC GPIO_FUN_CAM
#define CONFIG_GPIO3_FUNC GPIO_FUN_CAM
#define CONFIG_GPIO4_FUNC GPIO_FUN_CAM
#define CONFIG_GPIO5_FUNC GPIO_FUN_CAM
#define CONFIG_GPIO6_FUNC GPIO_FUN_CAM
#define CONFIG_GPIO7_FUNC GPIO_FUN_USB
#define CONFIG_GPIO8_FUNC GPIO_FUN_USB
#define CONFIG_GPIO9_FUNC GPIO_FUN_CLK_OUT
#define CONFIG_GPIO10_FUNC GPIO_FUN_UART1_TX
#define CONFIG_GPIO11_FUNC GPIO_FUN_I2C
#define CONFIG_GPIO12_FUNC GPIO_FUN_CAM
#define CONFIG_GPIO14_FUNC GPIO_FUN_UART0_TX
#define CONFIG_GPIO15_FUNC GPIO_FUN_UART0_RX
#define CONFIG_GPIO16_FUNC GPIO_FUN_I2C
#define CONFIG_GPIO17_FUNC GPIO_FUN_UNUSED
#define CONFIG_GPIO18_FUNC GPIO_FUN_ADC
#define CONFIG_GPIO19_FUNC GPIO_FUN_UART1_RX
#define CONFIG_GPIO20_FUNC GPIO_FUN_PWM
#define CONFIG_GPIO21_FUNC GPIO_FUN_PWM
#define CONFIG_GPIO22_FUNC GPIO_FUN_UNUSED
#define CONFIG_GPIO23_FUNC GPIO_FUN_UNUSED
#define CONFIG_GPIO24_FUNC GPIO_FUN_UNUSED
#define CONFIG_GPIO25_FUNC GPIO_FUN_UNUSED
#define CONFIG_GPIO26_FUNC GPIO_FUN_UNUSED
#define CONFIG_GPIO27_FUNC GPIO_FUN_UNUSED
#define CONFIG_GPIO28_FUNC GPIO_FUN_UNUSED
#define CONFIG_GPIO29_FUNC GPIO_FUN_CAM
#define CONFIG_GPIO30_FUNC GPIO_FUN_CAM
#define CONFIG_GPIO31_FUNC GPIO_FUN_CAM
```
## 外设使用情况
### 摄像头外设
    摄像头外设采用GC0308传感器。博流公司支持的摄像头传感器如下：
| Name | interface |
|:----:|:---------:|
| bf2013 | DVP  |
| gc0308 | DVP  |
| gc0328 | DVP  |
| gc2053 | DVP/CSI  |
| ov2685 | CSI |
### 无线模块外设
    无线模块采用BL604作为无线模块，完成WIFI联网功能。其实现是利用外设UART1实现。接线顺序如下：
|  name    |  bl706  |  bl604 |
|:------:  |:-------:|:------:|
|   GND    |   GND   |   GND  |    
|RX to TX | GPIO_19 | GPIO_4 |
|TX to RX | GPIO_10 | GPIO_3 |

**AT固件bin文件在该文件下，文件名：bl602_AT_whole_v1.1.bin**

**具体使用见文档：BL602 AT TEST.docx**


### 舵机模块
    舵机采用SG90模拟舵机，利用PWM输出控制。对应IO口：GPIO21
## 时钟使用情况
    在博流官方git仓库的BOUFFALO_SDK分支release_v1.4.5代码文件夹bsp\board\bl702\bl706_avb下打开clock_config.h文件进行引脚配置的修改。
    在本项目中主要是PWM输出时钟的修改，其修改为如下：
```
#define BSP_PWM_CLOCK_SOURCE ROOT_CLOCK_SOURCE_32K_CLK
#define BSP_PWM_CLOCK_DIV    15
```

