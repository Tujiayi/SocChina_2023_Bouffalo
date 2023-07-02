#include "bflb_core.h"

void init_yuv422p_table(void);
void yuv422sp_to_rgb24(unsigned char *yuv422sp, unsigned char *rgb, int width, int height);
void rgb24_to_rgb565(uint8_t *rgb24, uint8_t *rgb16);
void rgb565_to_gray(uint8_t *rgb565, uint8_t *gray, uint32_t pic_size);
void Y_to_rgb565_gray(uint8_t *y_400, uint16_t *gray, uint32_t len);
void y400_to_rgb565_gray(uint8_t *y_400, uint16_t *gray, uint32_t y400_x, uint32_t y400_y, uint32_t ratio);
void UYVY_to_RGB565(const void *inBuff, void *outBuff);
void YUYV_to_RGB565(const void *inBuff, void *outBuff);


