/*
 * Minimal SSD1306 driver + tank picture for the on-board 0.42" 72x40 OLED.
 * I2C SDA=GPIO5 SCL=GPIO6, address 0x3C (confirmed by the bring-up I2C scan).
 * The 72x40 glass sits at columns 28..99 of the controller's 128-column RAM
 * (same offset the vendor's u8g2 "SSD1306_72X40_ER" driver uses).
 * Runs in its own low-priority task so the ~10 ms I2C transfer never delays the model.
 */
#include "oled.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define W 72
#define H 40
#define X_OFFSET 28

static i2c_master_dev_handle_t s_dev;
static uint8_t s_fb[W * H / 8];
static volatile oled_view_t s_view;
static bool s_ok;

static esp_err_t cmd_list(const uint8_t *c, size_t n)
{
    uint8_t buf[32];
    buf[0] = 0x00;
    memcpy(&buf[1], c, n);
    return i2c_master_transmit(s_dev, buf, n + 1, 50);
}

static void px(int x, int y)
{
    if (x >= 0 && x < W && y >= 0 && y < H) {
        s_fb[(y / 8) * W + x] |= (uint8_t)(1u << (y % 8));
    }
}

static void rect(int x0, int y0, int x1, int y1, bool fill)
{
    for (int x = x0; x <= x1; x++) {
        for (int y = y0; y <= y1; y++) {
            if (fill || x == x0 || x == x1 || y == y0 || y == y1) px(x, y);
        }
    }
}

/* 3x5 glyphs, rows top->bottom, 3 bits each (bit2 = left). */
static const uint8_t *glyph(char ch)
{
    static const uint8_t digits[10][5] = {
        {7, 5, 5, 5, 7}, {2, 6, 2, 2, 7}, {7, 1, 7, 4, 7}, {7, 1, 7, 1, 7}, {5, 5, 7, 1, 1},
        {7, 4, 7, 1, 7}, {7, 4, 7, 5, 7}, {7, 1, 1, 1, 1}, {7, 5, 7, 5, 7}, {7, 5, 7, 1, 7},
    };
    static const uint8_t F[5] = {7, 4, 6, 4, 4}, S[5] = {7, 4, 7, 1, 7}, P[5] = {7, 5, 7, 4, 4},
                         V[5] = {5, 5, 5, 5, 2}, X[5] = {5, 5, 2, 5, 5}, BANG[5] = {2, 2, 2, 0, 2};
    if (ch >= '0' && ch <= '9') return digits[ch - '0'];
    switch (ch) {
    case 'F': return F;
    case 'S': return S;
    case 'P': return P;
    case 'V': return V;
    case 'X': return X;
    case '!': return BANG;
    default: return NULL;
    }
}

static void text(int x, int y, const char *s)
{
    for (; *s; s++, x += 4) {
        const uint8_t *g = glyph(*s);
        if (!g) continue;
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 3; c++) {
                if (g[r] & (4u >> c)) px(x + c, y + r);
            }
        }
    }
}

static void render(const oled_view_t *v, uint32_t frame)
{
    memset(s_fb, 0, sizeof(s_fb));
    /* tanks: 18 px wide, 28 px tall body (y 2..29), fill proportional to level */
    const int top = 2, bot = 29, inner = bot - top - 1;
    rect(1, top, 18, bot, false);
    rect(53, top, 70, bot, false);
    int hs = (int)((uint32_t)v->src_ml * (uint32_t)inner / 1000u);
    int hd = (int)((uint32_t)v->dst_ml * (uint32_t)inner / 1000u);
    if (hs > 0) rect(2, bot - hs, 17, bot - 1, true);
    if (hd > 0) rect(54, bot - hd, 69, bot - 1, true);
    /* pipe along the bottom, pump P then valve V */
    rect(19, 26, 52, 26, true);
    rect(19, 29, 52, 29, true);
    /* pump box at x 23..29: filled when running */
    rect(23, 24, 29, 31, v->pump_running);
    /* valve at x 39..45: hollow when open, X when closed */
    rect(39, 24, 45, 31, false);
    if (!v->valve_open) {
        for (int i = 0; i <= 6; i++) {
            px(39 + i, 24 + i);
            px(45 - i, 24 + i);
        }
    }
    /* moving dots inside the pipe when liquid flows */
    if (v->flow_ml_s > 0) {
        for (int x = 19 + (int)(frame % 4u); x < 53; x += 4) {
            if ((x < 23 || x > 29) && (x < 39 || x > 45)) {
                px(x, 27);
                px(x, 28);
            }
        }
    }
    /* labels: pump/valve, failsafe, fault-injection mark */
    text(24, 18, "P");
    text(41, 18, "V");
    if (v->failsafe && (frame / 2u) % 2u == 0u) text(28, 3, "FS");
    if (v->fault_injected) text(34, 10, "!");
    /* levels in mL under the tanks */
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", v->src_ml);
    text(2, 34, buf);
    snprintf(buf, sizeof(buf), "%u", v->dst_ml);
    text(54, 34, buf);
}

static void flush(void)
{
    static const uint8_t win[] = {0x21, X_OFFSET, X_OFFSET + W - 1, 0x22, 0, (H / 8) - 1};
    cmd_list(win, sizeof(win));
    static uint8_t buf[1 + sizeof(s_fb)];
    buf[0] = 0x40;
    memcpy(&buf[1], s_fb, sizeof(s_fb));
    i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static void oled_task(void *arg)
{
    (void)arg;
    for (uint32_t frame = 0;; frame++) {
        oled_view_t v;
        memcpy(&v, (const void *)&s_view, sizeof(v)); /* display only: tearing is harmless */
        render(&v, frame);
        flush();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

bool oled_init(void)
{
    i2c_master_bus_handle_t bus = NULL;
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 5,
        .scl_io_num = 6,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) return false;
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3C,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_dev) != ESP_OK) return false;
    static const uint8_t init[] = {
        0xAE,       /* display off */
        0xD5, 0x80, /* clock */
        0xA8, 0x27, /* multiplex 40 */
        0xD3, 0x00, /* display offset */
        0x40,       /* start line 0 */
        0x8D, 0x14, /* charge pump on */
        0x20, 0x00, /* horizontal addressing */
        0xA1, 0xC8, /* segment remap, COM scan dec */
        0xDA, 0x12, /* COM pins */
        0x81, 0xCF, /* contrast */
        0xD9, 0xF1, /* precharge */
        0xDB, 0x40, /* VCOMH */
        0xAD, 0x30, /* internal IREF (0.42" panel) */
        0xA4, 0xA6, /* resume RAM, normal */
        0xAF,       /* display on */
    };
    if (cmd_list(init, sizeof(init)) != ESP_OK) return false;
    s_ok = true;
    xTaskCreate(oled_task, "oled", 3072, NULL, 2, NULL);
    return true;
}

void oled_set_view(const oled_view_t *v)
{
    if (s_ok) {
        memcpy((void *)&s_view, v, sizeof(*v));
    }
}
