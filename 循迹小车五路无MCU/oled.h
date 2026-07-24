/**
 * @file    oled.h
 * @brief   SSD1306 OLED 显示器驱动 (I2C 接口)
 *
 * @note    适用于 128x64 分辨率, I2C 地址 0x3C
 *          使用 MSPM0 DriverLib I2C API 通信
 *          依赖 SysConfig 生成的 I2C 初始化代码
 */

#ifndef OLED_H
#define OLED_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*===========================================================================
 * 硬件参数 — 根据工程 SysConfig 配置修改
 *===========================================================================*/

/** @brief SSD1306 7位 I2C 从机地址 (模块 DC 脚接地=0x3C, 接高=0x3D) */
#define OLED_I2C_ADDR          0x3C

/**
 * @brief I2C 外设实例宏 (与 SysConfig 中 I2C 实例名对应)
 *
 * SysConfig 配置      →  此处填写
 * ────────────────────────────────
 * I2C1.$name = "OLED"   →  OLED_INST
 * I2C1.$name = "I2C"    →  I2C_INST
 * I2C1.$name = "OLED"   →  OLED_INST  (当前工程)
 *
 * 如果不确定，查看 Debug/ti_msp_dl_config.h 中 #define xxx_INST  I2C1 那一行
 */
#ifndef OLED_I2C_INST
#define OLED_I2C_INST           OLED_INST
#endif

/** @brief 显示分辨率 */
#define OLED_WIDTH             128
#define OLED_HEIGHT            64
#define OLED_PAGES             (OLED_HEIGHT / 8)   /* 8 页，每页 8 像素高 */

/** @brief I2C 控制字节 */
#define OLED_CTRL_CMD          0x00    /* 后续字节为命令 */
#define OLED_CTRL_DATA         0x40    /* 后续字节为显示数据 */

/*===========================================================================
 * SSD1306 基本命令
 *===========================================================================*/

#define OLED_CMD_DISPLAY_OFF       0xAE    /* 关闭显示 */
#define OLED_CMD_DISPLAY_ON        0xAF    /* 开启显示 */
#define OLED_CMD_NORMAL_DISPLAY    0xA6    /* 正常显示 (非反色) */
#define OLED_CMD_INVERT_DISPLAY    0xA7    /* 反色显示 */
#define OLED_CMD_SET_CONTRAST      0x81    /* 设置对比度 (后跟 1 字节) */
#define OLED_CMD_ENTIRE_DISPLAY_ON 0xA5    /* 全屏点亮 (忽略 GDDRAM) */
#define OLED_CMD_RESUME_TO_RAM     0xA4    /* 恢复显示 GDDRAM 内容 */
#define OLED_CMD_SET_MUX_RATIO     0xA8    /* 设置多路复用比 (后跟 1 字节) */
#define OLED_CMD_SET_DISPLAY_OFFSET 0xD3   /* 设置显示偏移 (后跟 1 字节) */
#define OLED_CMD_SET_START_LINE    0x40    /* 设置显示起始行 (0x40-0x7F) */
#define OLED_CMD_CHARGE_PUMP       0x8D    /* 电荷泵设置 (后跟 1 字节) */
#define OLED_CMD_MEM_ADDR_MODE     0x20    /* 内存寻址模式 (后跟 1 字节) */
#define OLED_CMD_SEG_REMAP         0xA0    /* 列地址重映射 (0xA0/0xA1) */
#define OLED_CMD_COM_SCAN_DIR      0xC0    /* COM 扫描方向 (0xC0/0xC8) */
#define OLED_CMD_SET_COM_PINS      0xDA    /* COM 引脚硬件配置 (后跟 1 字节) */
#define OLED_CMD_SET_PRECHARGE     0xD9    /* 预充电周期 (后跟 1 字节) */
#define OLED_CMD_SET_VCOMD         0xDB    /* VCOMH 电压 (后跟 1 字节) */
#define OLED_CMD_NOP               0xE3    /* 空操作 */
#define OLED_CMD_DEACTIVATE_SCROLL 0x2E    /* 停用滚动 */

/* 寻址模式 */
#define OLED_ADDR_MODE_HORIZONTAL  0x00    /* 水平寻址 */
#define OLED_ADDR_MODE_VERTICAL    0x01    /* 垂直寻址 */
#define OLED_ADDR_MODE_PAGE        0x02    /* 页寻址 (最常用) */

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief  OLED 初始化
 * @note   初始化 I2C1 外设并执行 SSD1306 上电序列
 *         调用前需确保 SYSCFG_DL_init() 已完成 GPIO/I2C 基础配置
 */
void OLED_Init(void);

/** @brief 清屏 (全部写入 0x00) */
void OLED_Clear(void);

/** @brief 全屏填充 (全部写入 0xFF) */
void OLED_Fill(void);

/** @brief 开启显示 */
void OLED_DisplayOn(void);

/** @brief 关闭显示 (进入休眠) */
void OLED_DisplayOff(void);

/**
 * @brief  设置对比度
 * @param  contrast  对比度值 [0, 255]，典型值 0x7F
 */
void OLED_SetContrast(uint8_t contrast);

/**
 * @brief  设置光标位置 (页寻址模式)
 * @param  page  页号 [0, OLED_PAGES-1]，每页 8 像素高
 * @param  col   列号 [0, OLED_WIDTH-1]
 */
void OLED_SetCursor(uint8_t page, uint8_t col);

/**
 * @brief  在指定位置显示一个 6x8 ASCII 字符
 * @param  page  页号 [0, OLED_PAGES-1]
 * @param  col   列号 [0, OLED_WIDTH-1]
 * @param  ch    要显示的 ASCII 字符
 */
void OLED_ShowChar(uint8_t page, uint8_t col, char ch);

/**
 * @brief  在指定位置显示字符串
 * @param  page  起始页号
 * @param  col   起始列号
 * @param  str   以 '\\0' 结尾的字符串
 * @note   自动换页，不自动换行
 */
void OLED_ShowString(uint8_t page, uint8_t col, const char *str);

/**
 * @brief  在指定位置显示有符号整数
 * @param  page  页号
 * @param  col   列号
 * @param  num   要显示的整数
 */
void OLED_ShowNum(uint8_t page, uint8_t col, int32_t num);

/**
 * @brief  显示位图
 * @param  page    起始页号
 * @param  col     起始列号
 * @param  width   位图宽度 (像素)
 * @param  height  位图高度 (像素)
 * @param  bmp     位图数据指针 (按列排列，每字节 8 垂直像素)
 */
void OLED_DrawBMP(uint8_t page, uint8_t col,
                  uint8_t width, uint8_t height, const uint8_t *bmp);

#endif /* OLED_H */
