// 双线性插值
// By Umeko 2024.08.03

#ifndef BIO_LINEAR_INTERPOLATION_H
#define BIO_LINEAR_INTERPOLATION_H

#include <Arduino.h>

#define SRC_W 32
#define SRC_H 24

#define DST_W 32 * 9
#define DST_H 24 * 9

#define FASTER

static const float scale_x = 1. / 9.;
static const float scale_y = 1. / 9.;


float min(float a, float b){
    return a < b ? a : b;
}

inline int min(int a, int b){
    return a < b ? a : b;
}

inline float getValue(int y, int x, float *datas){
    return datas[x + (23 - y) * SRC_W];
}

inline int getValue(int y, int x, int *datas){
    return datas[x + (23 - y) * SRC_W];
}

#if defined(FASTER)
inline int bio_linear_interpolation(int dst_x, int dst_y, int *src_data){
    int src_x, src_y;
    int src_x0, src_y0, src_x1, src_y1;
    int value00, value01, value10, value11, v0, v1, frac_x, frac_y;
            
    // 目标在源数据上的坐标, 当作保留一位小数处理
    src_x = (dst_x*1024 + 512) / 9 - 512;
    src_y = (dst_y*1024 + 512) / 9 - 512;

    // 找到四个最近邻点的位置
    src_x0 = src_x / 1024;
    src_y0 = src_y / 1024;
    src_x1 = src_x0+1;
    src_y1 = src_y0+1;

    // 确保不超出源图像边界
    src_x1 = min(src_x1, 31);
    src_y1 = min(src_y1, 23);

    // 计算分数部分
    frac_x = src_x - src_x0 * 1024;
    frac_y = src_y - src_y0 * 1024;

    // 获取四个最近邻点的值
    value00 = getValue(src_y0, src_x0, src_data);
    value01 = getValue(src_y0, src_x1, src_data);
    value10 = getValue(src_y1, src_x0, src_data);
    value11 = getValue(src_y1, src_x1, src_data);
    // Serial.printf("%d, %d, %d, %d, %.2f\n", src_x0, src_y0, src_x1, src_y1, frac_x);
    // 沿x轴的线性插值
    v0 = value00 * (1024 - frac_x) + value01 * frac_x;
    v1 = value10 * (1024 - frac_x) + value11 * frac_x;

    v0 /= 1024;
    v1 /= 1024;
    // 沿y轴的线性插值
    return (v0 * (1024 - frac_y) + v1 * frac_y) / 1024;
}

// float bio_linear_interpolation(int dst_x, int dst_y, float *src_data){
//     int src_x, src_y;
//     int src_x0, src_y0, src_x1, src_y1;
//     float value00, value01, value10, value11, v0, v1, frac_x, frac_y;;
            
//     // 目标在源数据上的坐标, 当作保留一位小数处理
//     src_x = (dst_x*10 + 5) / 9 - 5;
//     src_y = (dst_y*10 + 5) / 9 - 5;

//     // 找到四个最近邻点的位置
//     src_x0 = src_x / 10;
//     src_y0 = src_y / 10;
//     src_x1 = src_x0+1;
//     src_y1 = src_y0+1;

//     // 确保不超出源图像边界
//     src_x1 = min(src_x1, 31);
//     src_y1 = min(src_y1, 23);

//     // 计算分数部分
//     frac_x = (float)(src_x - src_x0 * 10) / 10;
//     frac_y = (float)(src_y - src_y0 * 10) / 10;

//     // 获取四个最近邻点的值
//     value00 = getValue(src_y0, src_x0, src_data);
//     value01 = getValue(src_y0, src_x1, src_data);
//     value10 = getValue(src_y1, src_x0, src_data);
//     value11 = getValue(src_y1, src_x1, src_data);
//     // Serial.printf("%d, %d, %d, %d, %.2f\n", src_x0, src_y0, src_x1, src_y1, frac_x);
//     // 沿x轴的线性插值
//     v0 = value00 * (1 - frac_x) + value01 * frac_x;
//     v1 = value10 * (1 - frac_x) + value11 * frac_x;

//     // 沿y轴的线性插值
//     return v0 * (1 - frac_y) + v1 * frac_y;
// }
#else
// 双线性插值，根据坐标返回插值结果
float bio_linear_interpolation(int dst_x, int dst_y, float *src_data){
    float src_x, src_y, frac_x, frac_y;
    int src_x0, src_y0, src_x1, src_y1;
    float value00, value01, value10, value11, v0, v1;
            
    // 目标在源数据上的坐标
    src_x = (dst_x + 0.5) * scale_x - 0.5;
    src_y = (dst_y + 0.5) * scale_y - 0.5;

    // 找到四个最近邻点的位置
    src_x0 = (int)floor(src_x);
    src_y0 = (int)floor(src_y);
    src_x1 = (int)ceil(src_x);
    src_y1 = (int)ceil(src_y);

    // 确保不超出源图像边界
    src_x1 = min(src_x1, SRC_W - 1);
    src_y1 = min(src_y1, SRC_H - 1);

    // 计算分数部分
    frac_x = src_x - src_x0;
    frac_y = src_y - src_y0;

    // 获取四个最近邻点的值
    value00 = getValue(src_y0, src_x0, src_data);
    value01 = getValue(src_y0, src_x1, src_data);
    value10 = getValue(src_y1, src_x0, src_data);
    value11 = getValue(src_y1, src_x1, src_data);
    // Serial.printf("%d, %d, %d, %d\n", src_x0, src_y0, src_x1, src_y1);
    // 沿x轴的线性插值
    v0 = value00 * (1 - frac_x) + value01 * frac_x;
    v1 = value10 * (1 - frac_x) + value11 * frac_x;

    // 沿y轴的线性插值
    return v0 * (1 - frac_y) + v1 * frac_y;
}
#endif
// 2D bilinear interpolation
// x, y are the coordinates of the point to interpolate
// x0, y0, x1, y1 are the coordinates of the four corners of the interpolation rectangle
#endif