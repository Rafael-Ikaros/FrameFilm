/*********************************************************************
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2026 kiritro
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *
 * FileName : /film_hal/src/hal_epd_398e4.c
 * Author: Kiritro  Version: v0.1  Date: 2026/10/28
 * Description: 3.98" 768x552 四色 EPD driver (JD79665AA, hardware SPI half-duplex)
 *              可见区 768x552，物理帧 800x600（左侧 32px 白边、底部 48 行不可见）
 *              4 色：黑/白/黄/红；无蓝、绿（按黑/黄近似）
 * ChangeLog: 初版，参数取自 jd79665aa-driver-main/docs
 *
 *********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "hal_epd.h"
#if EPD_SELECT_E4_3_98_768_552 == 1
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "sys_log.h"

/*********************************************************************
 * MACROS
 */
// 4bpp 输入（film 像素区）→ 2bpp 输出
#define EPD_INPUT_LINE      (EPD_WIDTH / 2)          // 384 字节/行（可见区）
#define EPD_PHYS_WIDTH      (800)                    // 物理列数
#define EPD_PHYS_HEIGHT     (600)                    // 物理行数
#define EPD_X_OFFSET        (32)                     // 可见区左边距
#define EPD_ROW_BYTES       (EPD_PHYS_WIDTH / 4)     // 200 字节/行
#define EPD_FRAME_BYTES     (EPD_ROW_BYTES * EPD_PHYS_HEIGHT)  // 120000 字节

// 参考驱动按“可见 x 加 32 后直接打包”写入，数据顺序不做镜像；
// 若上机发现画面左右颠倒，把这里置 1 即可（只影响打包顺序，不影响窗口）。
#define EPD_E4_MIRROR_X     (0)

#define FILM_HEADER_SIZE                  (32)
#define FILM_COLOR_TABLE_SIZE             (16)
#define FILM_OFFSET_FILESIZE              (0x00)
#define FILM_OFFSET_SCREENWIDTH           (0x04)
#define FILM_OFFSET_SCREENHEIGHT          (0x06)
#define FILM_OFFSET_COLORCOUNT            (0x08)
#define FILM_OFFSET_RESERVED              (0x09)
#define FILM_OFFSET_COLORTABLE            (0x10)

// JD79665AA commands
#define PSR                            0x00  // Panel setting
#define PWR                            0x01  // Power setting
#define POF                            0x02  // Power off
#define POFS                           0x03  // Power off sequence
#define PON                            0x04  // Power on
#define BTST1                          0x05  // Booster soft start 1
#define BTST2                          0x06  // Booster soft start 2
#define BTST3                          0x08  // Booster soft start 3
#define DTM                            0x10  // Data start transmission
#define DRF                            0x12  // Display refresh
#define PLL                            0x30  // PLL control
#define CDI                            0x50  // VCOM and data interval
#define TCON                           0x60  // TCON setting
#define TRES                           0x61  // Resolution setting
#define GSST                           0x65  // Gate/Source start setting
#define RSET                           0x83  // Window setting
#define PWS                            0xE3  // Power saving
#define CCC                            0x84  // VCOM/数据间隔补充

// 四色 2bit 编码：0=黑 1=白 2=黄 3=红
#define E4_BLACK  0
#define E4_WHITE  1
#define E4_YELLOW 2
#define E4_RED    3

// 4 个相同 2bit 值打包成一个字节
#define PACK4(v)  (((v) << 6) | ((v) << 4) | ((v) << 2) | (v))

#define EPD_RESET_TIMEOUT_MS     (3000)
#define EPD_OP_TIMEOUT_MS        (120000)

/*********************************************************************
* TYPEDEFS
*/
typedef struct
{
    uint32_t fileSize;
    uint16_t screenWidth;
    uint16_t screenHeight;
    uint8_t  colorCount;
    uint8_t  reserved[7];
    uint8_t  colorTable[FILM_COLOR_TABLE_SIZE];
} __attribute__((packed)) FilmHeader;

/*********************************************************************
 * LOCAL VARIABLES
 */
static spi_device_handle_t m_spi_device;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void spi_init(void);
static void epd_cmd(uint8_t command);
static void epd_data(const uint8_t *data, size_t len);
static void epd_data1(uint8_t value);
static esp_err_t epd_wait_ready(uint32_t timeout_ms, const char *stage);
static bool epd_reset(void);
static bool epd_init_registers(void);
static bool epd_full_window(void);
static bool epd_power_on(void);
static bool epd_power_off(void);
static void epd_write_frame(const uint8_t *pixelData, const uint8_t *map16);
static bool epd_refresh(void);
static esp_err_t film_parse_header(const unsigned char *filmData, FilmHeader *header);

/*********************************************************************
 * HARDWARE SPI (half-duplex, 3-wire mode)
 *********************************************************************/

static void spi_init(void)
{
    esp_err_t ret;

    spi_bus_config_t buscfg =
    {
        .miso_io_num = -1,
        .mosi_io_num = EPD_SDIN_PIN,
        .sclk_io_num = EPD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EPD_ROW_BYTES * 4
    };

    ret = spi_bus_initialize(EPD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        sys_loge(EPD_TAG, "SPI bus init failed: %d", ret);
        return;
    }

    // 该屏全刷建议 1MHz（参考驱动同值），全帧 120000 字节约 1s
    spi_device_interface_config_t devcfg =
    {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE,
    };

    ret = spi_bus_add_device(EPD_SPI_HOST, &devcfg, &m_spi_device);
    if (ret != ESP_OK)
    {
        sys_loge(EPD_TAG, "SPI add device failed: %d", ret);
    }
}

// 仅发数据，不碰 CS/DC；整帧传输期间 CS 需持续拉低
static void spi_send(const uint8_t *data, size_t len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_transmit(m_spi_device, &t);
}

static void epd_cmd(uint8_t command)
{
    EPD_W21_CS_0;
    EPD_W21_DC_0;
    spi_send(&command, 1);
    EPD_W21_CS_1;
}

static void epd_data(const uint8_t *data, size_t len)
{
    EPD_W21_CS_0;
    EPD_W21_DC_1;
    spi_send(data, len);
    EPD_W21_CS_1;
}

static void epd_data1(uint8_t value)
{
    epd_data(&value, 1);
}

/*********************************************************************
 * HELPER FUNCTIONS
 *********************************************************************/

// BUSY 低=忙，高=空闲；等待期间让出 CPU（全刷约 15s，不能死等）
static esp_err_t epd_wait_ready(uint32_t timeout_ms, const char *stage)
{
    TickType_t start = xTaskGetTickCount();

    while (isEPD_W21_BUSY == 0)
    {
        uint32_t elapsed = (uint32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
        if (elapsed >= timeout_ms)
        {
            sys_loge(EPD_TAG, "JD79665AA busy timeout (%s, %lu ms)",
                     stage, (unsigned long)timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }

    return ESP_OK;
}

static bool epd_reset(void)
{
    EPD_W21_RST_1;
    vTaskDelay(pdMS_TO_TICKS(20));
    EPD_W21_RST_0;
    vTaskDelay(1);                          // >=2ms，10ms tick 下取 1 tick
    EPD_W21_RST_1;
    vTaskDelay(pdMS_TO_TICKS(20));

    if (epd_wait_ready(EPD_RESET_TIMEOUT_MS, "reset") != ESP_OK)
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(30));
    return true;
}

static bool epd_init_registers(void)
{
    static const struct
    {
        uint8_t command;
        uint8_t length;
        uint8_t data[6];
    } init[] = {
        {0xAA, 6, {0x49, 0x55, 0x20, 0x08, 0x09, 0x18}},
        {0x01, 1, {0x3F}},
        {0x00, 2, {0x4B, 0x69}},
        {0x05, 4, {0x40, 0x1F, 0x1F, 0x2C}},
        {0x08, 4, {0x6F, 0x1F, 0x1F, 0x22}},
        {0x06, 4, {0x6F, 0x1F, 0x14, 0x14}},
        {0x03, 4, {0x00, 0x54, 0x00, 0x44}},
        {0x60, 2, {0x02, 0x00}},
        {0x30, 1, {0x08}},
        {0x50, 1, {0x3F}},
        {0x61, 4, {EPD_PHYS_WIDTH >> 8, EPD_PHYS_WIDTH & 0xFF,
                   EPD_PHYS_HEIGHT >> 8, EPD_PHYS_HEIGHT & 0xFF}},
        {0x65, 4, {0x10, 0x00, 0x20, 0x00}},
        {0xE3, 1, {0x2F}},
        {0x84, 1, {0x01}},
    };

    for (size_t i = 0; i < sizeof(init) / sizeof(init[0]); i++)
    {
        epd_cmd(init[i].command);
        epd_data(init[i].data, init[i].length);
    }

    // 初始化后给面板一个稳定时间
    epd_wait_ready(EPD_RESET_TIMEOUT_MS, "init");
    vTaskDelay(pdMS_TO_TICKS(10));

    return true;
}

static bool epd_full_window(void)
{
    const uint8_t window[] = {0x00, 0x00,
                              (EPD_PHYS_WIDTH - 1) >> 8, (EPD_PHYS_WIDTH - 1) & 0xFF,
                              0x00, 0x00,
                              (EPD_PHYS_HEIGHT - 1) >> 8, (EPD_PHYS_HEIGHT - 1) & 0xFF,
                              0x01};
    epd_cmd(RSET);
    epd_data(window, sizeof(window));
    return true;
}

static bool epd_power_on(void)
{
    epd_cmd(PON);
    return epd_wait_ready(EPD_OP_TIMEOUT_MS, "pon") == ESP_OK;
}

static bool epd_power_off(void)
{
    epd_cmd(POF);
    epd_data1(0x00);
    return epd_wait_ready(EPD_OP_TIMEOUT_MS, "poff") == ESP_OK;
}

static bool epd_refresh(void)
{
    epd_cmd(DRF);
    epd_data1(0x00);
    return epd_wait_ready(EPD_OP_TIMEOUT_MS, "refresh") == ESP_OK;
}

/**
 * @brief 写满一帧物理数据（800x600，2bpp）
 *
 * 可见区按物理坐标放在 x=32..799、y=0..551，其余补白。
 * 控制器 SHL=0 时 X 方向镜像，EPD_E4_MIRROR_X 控制是否在打包时反向。
 */
static void epd_write_frame(const uint8_t *pixelData, const uint8_t *map16)
{
    uint8_t px[EPD_PHYS_WIDTH];
    uint8_t row[EPD_ROW_BYTES];

    epd_cmd(DTM);
    EPD_W21_CS_0;
    EPD_W21_DC_1;

    for (int y = 0; y < EPD_PHYS_HEIGHT; y++)
    {
        if (y < EPD_HEIGHT)
        {
            const uint8_t *in = pixelData + (size_t)y * EPD_INPUT_LINE;

            for (int x = 0; x < EPD_X_OFFSET; x++)
            {
                px[x] = E4_WHITE;
            }
            for (int x = 0; x < EPD_WIDTH; x++)
            {
                uint8_t byte = in[x >> 1];
                uint8_t idx = (x & 1) ? (byte & 0x0F) : ((byte >> 4) & 0x0F);
                px[EPD_X_OFFSET + x] = map16[idx];
            }
            for (int x = EPD_X_OFFSET + EPD_WIDTH; x < EPD_PHYS_WIDTH; x++)
            {
                px[x] = E4_WHITE;
            }
        }
        else
        {
            memset(px, E4_WHITE, sizeof(px));
        }

        for (int i = 0; i < EPD_ROW_BYTES; i++)
        {
            uint8_t out = 0;
            for (int k = 0; k < 4; k++)
            {
#if EPD_E4_MIRROR_X
                int p = EPD_PHYS_WIDTH - 1 - (i * 4 + k);
#else
                int p = i * 4 + k;
#endif
                out = (uint8_t)((out << 2) | (px[p] & 0x03));
            }
            row[i] = out;
        }

        spi_send(row, EPD_ROW_BYTES);
    }

    EPD_W21_CS_1;
}

static esp_err_t film_parse_header(const unsigned char *filmData, FilmHeader *header)
{
    if (filmData == NULL || header == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&header->fileSize, &filmData[FILM_OFFSET_FILESIZE], sizeof(uint32_t));
    memcpy(&header->screenWidth, &filmData[FILM_OFFSET_SCREENWIDTH], sizeof(uint16_t));
    memcpy(&header->screenHeight, &filmData[FILM_OFFSET_SCREENHEIGHT], sizeof(uint16_t));
    memcpy(&header->colorCount, &filmData[FILM_OFFSET_COLORCOUNT], sizeof(uint8_t));
    memcpy(header->reserved, &filmData[FILM_OFFSET_RESERVED], 7);
    memcpy(header->colorTable, &filmData[FILM_OFFSET_COLORTABLE], FILM_COLOR_TABLE_SIZE);

    if (header->colorCount < 2 || header->colorCount > 6)
    {
        sys_loge(EPD_TAG, "Invalid color count: %d (expected 2-6)", header->colorCount);
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t expectedSize = (EPD_WIDTH * EPD_HEIGHT) / 2;
    if (header->fileSize != expectedSize)
    {
        sys_loge(EPD_TAG, "File size mismatch: %lu (expected %lu)",
                 (unsigned long)header->fileSize, (unsigned long)expectedSize);
        return ESP_ERR_INVALID_SIZE;
    }

    if (header->screenWidth != EPD_WIDTH || header->screenHeight != EPD_HEIGHT)
    {
        sys_loge(EPD_TAG, "Resolution mismatch: %ux%u (expected %ux%u)",
                 header->screenWidth, header->screenHeight, EPD_WIDTH, EPD_HEIGHT);
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

// film 颜色码（E6 6 色）→ 四色码：蓝按黑、绿按黄近似
static uint8_t e4_map_color(uint8_t color)
{
    switch (color)
    {
    case 0x00: return E4_BLACK;
    case 0xFF: return E4_WHITE;
    case 0xFC: return E4_YELLOW;
    case 0xE0: return E4_RED;
    case 0x03: return E4_BLACK;     // 蓝：该屏无蓝
    case 0x1C: return E4_YELLOW;    // 绿：该屏无绿
    default:   return E4_WHITE;     // 未使用项按白，避免出现黑块
    }
}

// 4bpp 中直接以 0..6 表示 E6 颜色的数据（hal_epd_display_pic）
static void build_pic_map(uint8_t *map16)
{
    for (int i = 0; i < 16; i++)
    {
        switch (i)
        {
        case 0:  map16[i] = E4_BLACK;  break;
        case 1:  map16[i] = E4_WHITE;  break;
        case 2:  map16[i] = E4_YELLOW; break;
        case 3:  map16[i] = E4_RED;    break;
        case 5:  map16[i] = E4_BLACK;  break;   // 蓝
        case 6:  map16[i] = E4_YELLOW; break;   // 绿
        default: map16[i] = E4_WHITE;  break;
        }
    }
}

static void epd_display_frame(const uint8_t *pixelData, const uint8_t *map16)
{
    if (!epd_power_on())
    {
        return;
    }

    epd_full_window();
    epd_write_frame(pixelData, map16);
    epd_full_window();
    epd_refresh();
    epd_power_off();
}

static void epd_display_solid(uint8_t e4_color)
{
    uint8_t px[EPD_ROW_BYTES];
    uint8_t fill = PACK4(e4_color);

    memset(px, fill, sizeof(px));

    if (!epd_power_on())
    {
        return;
    }

    epd_full_window();
    epd_cmd(DTM);
    EPD_W21_CS_0;
    EPD_W21_DC_1;
    for (int y = 0; y < EPD_PHYS_HEIGHT; y++)
    {
        spi_send(px, EPD_ROW_BYTES);
    }
    EPD_W21_CS_1;
    epd_full_window();
    epd_refresh();
    epd_power_off();
}

/*********************************************************************
 * GLOBAL FUNCTIONS
 *********************************************************************/

void hal_epd_init(void)
{
    gpio_config_t io_conf;

    // BUSY: input
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << EPD_BUSY_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // RST, DC, CS: output
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << EPD_RST_PIN) | (1ULL << EPD_DC_PIN) |
                           (1ULL << EPD_CS_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // 上电前先把控制脚拉高，避免 CS/RST 毛刺
    EPD_W21_CS_1;
    EPD_W21_DC_1;
    EPD_W21_RST_1;

    spi_init();

    sys_logi(EPD_TAG, "EPD initialized (JD79665AA 4-color, %dx%d)", EPD_WIDTH, EPD_HEIGHT);
}

void hal_epd_display_init(void)
{
    if (!epd_reset())
    {
        sys_loge(EPD_TAG, "EPD reset failed");
        return;
    }

    epd_init_registers();

    sys_logi(EPD_TAG, "EPD display initialized");
}

void hal_epd_display_white(void)
{
    epd_display_solid(E4_WHITE);
}

void hal_epd_display_black(void)
{
    epd_display_solid(E4_BLACK);
}

void hal_epd_display_yellow(void)
{
    epd_display_solid(E4_YELLOW);
}

void hal_epd_display_red(void)
{
    epd_display_solid(E4_RED);
}

void hal_epd_display_blue(void)
{
    // 该屏没有蓝色，按黑色出
    epd_display_solid(E4_BLACK);
}

void hal_epd_display_green(void)
{
    // 该屏没有绿色，按黄色出
    epd_display_solid(E4_YELLOW);
}

void hal_epd_display_pic(const unsigned char *picData)
{
    uint8_t map16[16];

    if (picData == NULL)
    {
        sys_loge(EPD_TAG, "picData is NULL");
        return;
    }

    build_pic_map(map16);
    epd_display_frame(picData, map16);

    sys_logi(EPD_TAG, "Display pic completed");
}

void hal_epd_display_film(const unsigned char *filmData)
{
    FilmHeader header;
    uint8_t map16[16];

    if (filmData == NULL)
    {
        sys_loge(EPD_TAG, "filmData is NULL");
        return;
    }

    if (film_parse_header(filmData, &header) != ESP_OK)
    {
        sys_loge(EPD_TAG, "Failed to parse film header");
        return;
    }

    for (int i = 0; i < 16; i++)
    {
        map16[i] = e4_map_color(header.colorTable[i]);
    }

    sys_logi(EPD_TAG, "Film: %ux%u, %d colors, %lu bytes, phys %dx%d",
             header.screenWidth, header.screenHeight,
             header.colorCount, (unsigned long)header.fileSize,
             EPD_PHYS_WIDTH, EPD_PHYS_HEIGHT);

    epd_display_frame(filmData + FILM_HEADER_SIZE, map16);

    sys_logi(EPD_TAG, "Film display completed");
}

int32_t hal_epd_partial_update(uint8_t panel, uint16_t x_start, uint16_t y_start,
                               uint16_t width, uint16_t height,
                               const unsigned char *data, uint8_t display_enable)
{
    // JD79665AA 的窗口更新仍走原厂全刷波形（约 15s），暂不提供局部刷新
    (void)panel;
    (void)x_start;
    (void)y_start;
    (void)width;
    (void)height;
    (void)data;
    (void)display_enable;

    sys_logw(EPD_TAG, "JD79665AA partial update not supported");
    return -1;
}

void hal_epd_sleep(void)
{
    epd_cmd(POF);
    epd_data1(0x00);
    epd_wait_ready(EPD_OP_TIMEOUT_MS, "sleep");

    sys_logi(EPD_TAG, "EPD sleep");
}

void hal_epd_pwroff(void)
{
    hal_epd_sleep();
    vTaskDelay(pdMS_TO_TICKS(100));

    sys_logi(EPD_TAG, "EPD power off");
}

void hal_epd_deinit(void)
{
    if (m_spi_device != NULL)
    {
        spi_bus_remove_device(m_spi_device);
        m_spi_device = NULL;
    }
    spi_bus_free(EPD_SPI_HOST);

    gpio_config_t io_conf =
    {
        .pin_bit_mask = (1ULL << EPD_BUSY_PIN) | (1ULL << EPD_RST_PIN) |
        (1ULL << EPD_DC_PIN)   | (1ULL << EPD_CS_PIN) |
        (1ULL << EPD_SCK_PIN)  | (1ULL << EPD_SDIN_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    sys_logi(EPD_TAG, "EPD deinit, SPI and GPIOs released");
}
#endif
