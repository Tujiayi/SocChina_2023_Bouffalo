#include "image_trans.h"

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

static long U[256], V[256], Y1[256], Y2[256];

/**
 * @brief
 *
 */
void init_yuv422p_table(void)
{
    int i;

    // Initialize table
    for (i = 0; i < 256; i++) {
        V[i] = 15938 * i - 2221300;
        U[i] = 20238 * i - 2771300;
        Y1[i] = 11644 * i;
        Y2[i] = 19837 * i - 311710;
    }
}

/**
 * @brief
 *
 * @param yuv422sp
 * @param rgb
 * @param width
 * @param height
 */
void yuv422sp_to_rgb24(unsigned char *yuv422sp, unsigned char *rgb, int width, int height)
{
    int y, cb, cr;
    int r, g, b;
    int i = 0;
    unsigned char *p_y;
    unsigned char *p_uv;
    unsigned char *p_rgb;
    static int init_yuv422sp = 0; // just do it once

    p_y = yuv422sp;
    p_uv = p_y + width * height;
    p_rgb = rgb;

    if (init_yuv422sp == 0) {
        init_yuv422p_table();
        init_yuv422sp = 1;
    }

    for (i = 0; i < width * height / 2; i++) {
        y = p_y[0];
        cb = p_uv[0];
        cr = p_uv[1];

        r = MAX(0, MIN(255, (V[cr] + Y1[y]) / 10000));                 //R value
        b = MAX(0, MIN(255, (U[cb] + Y1[y]) / 10000));                 //B value
        g = MAX(0, MIN(255, (Y2[y] - 5094 * (r)-1942 * (b)) / 10000)); //G value

        // default rank：RGB
        p_rgb[0] = b;
        p_rgb[1] = g;
        p_rgb[2] = r;

        y = p_y[1];
        cb = p_uv[0];
        cr = p_uv[1];
        r = MAX(0, MIN(255, (V[cr] + Y1[y]) / 10000));                 //R value
        b = MAX(0, MIN(255, (U[cb] + Y1[y]) / 10000));                 //B value
        g = MAX(0, MIN(255, (Y2[y] - 5094 * (r)-1942 * (b)) / 10000)); //G value

        p_rgb[3] = b;
        p_rgb[4] = g;
        p_rgb[5] = r;

        p_y += 2;
        p_uv += 2;
        p_rgb += 6;
    }
}

/**
 * @brief
 *
 * @param rgb24
 * @param rgb16
 */
void rgb24_to_rgb565(uint8_t *rgb24, uint8_t *rgb16)
{
    int i = 0, j = 0;

    for (i = 0; i < 640 * 480 * 3; i += 3) {
        rgb16[j + 1] = rgb24[i] >> 3;                 // B
        rgb16[j + 1] |= ((rgb24[i + 1] & 0x1C) << 3); // G
        rgb16[j] = rgb24[i + 2] & 0xF8;               // R
        rgb16[j] |= (rgb24[i + 1] >> 5);              // G
        j += 2;
    }
}

/**
 * @brief
 *
 * @param rgb565
 * @param gray
 */
void rgb565_to_gray(uint8_t *rgb565, uint8_t *gray, uint32_t pic_size)
{
    int i,j;
    for(j = 0; j < pic_size/2; j += 2)
    {
        i=2*j;
        gray[j] = (((rgb565[i] >> 3) & 0x1F) * 77 +
                (((rgb565[i] << 3) & 0x38) + ((rgb565[i + 1] >> 5) & 0x07)) * 150 +
                (rgb565[i + 1] & 0x1F ) * 29 + 128 ) / 256;
    }
}



/**
 * @brief Convert YUV400 to grayscale image
 *
 * @param y_400 YUV400 image ptr
 * @param gray gray image ptr
 * @param len image size
 */
void Y_to_rgb565_gray(uint8_t *y_400, uint16_t *gray, uint32_t len)
{
    int8_t tmp = 0;

    for (uint32_t i = 0; i < len; i++) {
        if ((uint8_t)(y_400[i]) >= 0xaf) {
            tmp = (uint8_t)(y_400[i] >> 3);
        } else {
            tmp = (uint8_t)((y_400[i] + 0x30) >> 3);
        }
        gray[i] = tmp;
        gray[i] = gray[i] << 5;
        gray[i] += tmp;
        gray[i] = gray[i] << 6;
        gray[i] += tmp;
    }
}

void y400_to_rgb565_gray(uint8_t *y_400, uint16_t *gray, uint32_t y400_x, uint32_t y400_y, uint32_t ratio)
{
    uint8_t tmp = 0;
    uint32_t g = 0;
    for (uint32_t i = 0; i < y400_y; i += ratio) {
        for (uint32_t j = 0; j < y400_x; j += ratio) {
            tmp = (*y_400) >> 3;
            gray[g] = tmp;
            gray[g] = gray[g] << 5;
            gray[g] += tmp;
            gray[g] = gray[g] << 6;
            gray[g] += tmp;
            g++;
            y_400 += ratio;
        }
        y_400 += y400_x * (ratio - 1);
    }
}

void UYVY_to_RGB565(const void *inBuff, void *outBuff)
{
	int i=0;
	int rows = 0;
	int clos = 0;
	int u, v, y,r, g, b;
	uint8_t *yuv_buf;
	uint8_t *rbg_buf;
    int Ypos, Upos, Vpos;   /* Y U V在数据缓存中的偏移 */
    Ypos = 1;
    Upos = Ypos - 1;
    Vpos = Ypos + 1;

	yuv_buf = (uint8_t *)inBuff;
	rbg_buf = (uint8_t *)outBuff;

	u = 0;
	v = 0;
	y = 0;

	for (rows = 0; rows < 240   ; rows++)
	{
		for (clos = 0; clos < 320; clos++)
		{
			y = yuv_buf[Ypos];
            u = yuv_buf[Upos];
            v = yuv_buf[Vpos];						 

            r = (uint8_t)(y - 16 + 1.372 * (v - 128));
            g = (uint8_t)(y - 16 - 0.337 * (u - 128) - 0.699 * (v- 128));
            b = (uint8_t)(y - 16 + 1.734 * (u - 128));

			// r = y + v + ((v * 103) >> 8);
            // g = y - ((u * 88) >> 8) - ((v * 183) >> 8);
            // b = y + u + ((u * 198) >> 8);

            r = r > 255?255:(r < 0?0:r);
            g = g > 255?255:(g < 0?0:g);
            b = b > 255?255:(b < 0?0:b);

            /* 从低到高r g b */
            *(rbg_buf ++) = (((g & 0x1c) << 3) | (b >> 3)); /* g低5位，b高5位 */
            *(rbg_buf ++) = ((r & 0xf8) | (g >> 5));    /* r高5位，g高3位 */
            /* 两个字节数据中包含一个Y */
            Ypos += 2;
            //Ypos++;
            i++;
            /* 每两个Y更新一次UV */
            if(!(i & 0x01)) 
            {
                Upos = Ypos - 1;
                Vpos = Ypos + 1;
            }
		}
	}
}
uint8_t clamp(uint8_t v, uint8_t minValue, uint8_t maxValue) {
    if (v < minValue) return minValue;
    else if (v > maxValue) return maxValue;
    else return v;
}
void YUYV_to_RGB565(const void *inBuff, void *outBuff)
{
	int i=0;
	int rows = 0;
	int clos = 0;
	int u, v, y,r, g, b;
	uint8_t *yuv_buf;
	uint8_t *rbg_buf;
    int Ypos, Upos, Vpos;   /* Y U V在数据缓存中的偏移 */
    Ypos = 2;
    Upos = Ypos - 1;
    Vpos = Ypos + 1;

	yuv_buf = (uint8_t *)inBuff;
	rbg_buf = (uint8_t *)outBuff;

	u = 0;
	v = 0;
	y = 0;

	for (rows = 0; rows < 240 ; rows++)
	{
		for (clos = 0; clos < 320; clos++)
		{
			y = yuv_buf[Ypos];
            u = yuv_buf[Upos];
            v = yuv_buf[Vpos];						 

            r = (uint8_t)(y - 16 + 1.372 * (v - 128));
            g = (uint8_t)(y - 16 - 0.337 * (u - 128) - 0.699 * (v- 128));
            b = (uint8_t)(y - 16 + 1.734 * (u - 128));

			// r = y + v + ((v * 103) >> 8);
            // g = y - ((u * 88) >> 8) - ((v * 183) >> 8);
            // b = y + u + ((u * 198) >> 8);

            r = clamp(r, 0, 255), g = clamp(g, 0, 255), b = clamp(b, 0, 255);

            /* 从低到高r g b */
            *(rbg_buf ++) = (((g & 0x1c) << 3) | (b >> 3)); /* g低5位，b高5位 */
            *(rbg_buf ++) = ((r & 0xf8) | (g >> 5));    /* r高5位，g高3位 */
            /* 两个字节数据中包含一个Y */
            Ypos += 2;
            //Ypos++;
            i++;
            /* 每两个Y更新一次UV */
            if(!(i & 0x01)) 
            {
                Upos = Ypos - 1;
                Vpos = Ypos + 1;
            }
		}
	}
}

