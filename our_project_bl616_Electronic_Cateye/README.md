# 简介
    本项目名称为“基于博流开发平台的智能安防系统设计——智能猫眼”。其基于博流开发平台，使用开发板BL618G0开发板完成系统开发。该项目囊括了蓝牙配网，无线组网，摄像头图像数据采集并在LCD屏幕上显示，同时进行无线图传且在手机APP上完成显示以及利用OneNet平台作为中介，完成控制信息的获取并控制音频输出。

# 如何使用
    将本项目代码利用git工具git clone至本地后将其移动至博流官方git仓库中的SDK的example文件夹下后，即可使用。
    因本开发板基于master分支开发，故在git clone时请注意分支。
**博流仓库地址：https://github.com/bouffalolab/bouffalo_sdk**
    
# 详细介绍
## 引脚使用情况
```
所使用的引脚好以及其实现功能均在代码中相关初始化部分体现出，需自行进行学习。
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
    本子系统的无线功能的设计采用博流系列芯片BL618中集成的蓝牙和WIFI完成无线相关功能。

**蓝牙：蓝牙配网**

**WIFI:无线组网**

### LCD屏幕模块
    采用基于SPI协议、驱动芯片型号为为ST7789V的LCD屏幕。其像素为240*320，其数据输入格式为RGB565.
    除此屏幕外，博流公司支持的屏幕如下：

| Name | interface | remark |
|:---:|:------:|:------:|
| ili9341 | spi/dbi  | spi for allchips, dbi for bl616/bl808 |
| ili9488 | spi/dbi/dpi  | spi for allchips, dbi for bl616/bl808, dpi for bl808 |
| st7789v | spi  | spi for allchips |
| gc9503v | dpi  | dpi for bl808 |

### 音频输出模块
    基于ES8388音频芯片完成开发。


