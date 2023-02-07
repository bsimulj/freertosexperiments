#include <cstring>

#include <driver/ledc.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <freertos/task.h>

#include <lvgl.h>

extern "C"
{
    void app_main(void);
}

#define CMD_SWRESET 0x01 // Software Reset
#define CMD_SLPIN 0x10   // Sleep in
#define CMD_SLPOUT 0x11  // Sleep out
#define CMD_NORON 0x13   // Normal Display Mode On
#define CMD_INVOFF 0x20  // Display Inversion Off
#define CMD_DISPON 0x29  // Display On
#define CMD_CASET 0x2A   // Column Address Set
#define CMD_RASET 0x2B   // Row Address Set
#define CMD_RAMWR 0x2C   // Memory Write
#define CMD_WRMC 0x3C    // Memory Write
#define CMD_MADCTL 0x36  // Memory Data Access Control
#define CMD_COLMOD 0x3A  // Interface Pixel Format
#define CMD_PGC 0E0      // Positive Gamma Control
#define CMD_NGC 0xE1     // Negative Gamma Control
#define CMD_CSCON 0xF0   // Command Set Control

#define MADCTL_MY 0x80  // Row Address Order - 0=Increment (Top to Bottom), 1=Decrement (Bottom to Top)
#define MADCTL_MX 0x40  // Column Address Order - 0=Increment (Left to Right), 1=Decrement (Right to Left)
#define MADCTL_MV 0x20  // Row/Column exchange - 0=Normal, 1=Row/Column exchanged
#define MADCTL_ML 0x10  // Vertical Refresh Order
#define MADCTL_BGR 0x08 // RGB/BGR Order - BGR
#define MADCTL_MH 0x10  // Horizontal Refresh Order
#define MADCTL_RGB 0x00 // RGB/BGR Order - RGB

#define COLMOD_RGB_16BIT 0x50
#define COLMOD_CTRL_16BIT 0x05
#define COLMOD_RGB656 (COLMOD_RGB_16BIT | COLMOD_CTRL_16BIT)

static spi_device_handle_t deviceHandle;

enum HMIConfiguration
{
    E_TFT_MISO = 12,
    E_TFT_MOSI = 13, // In some display driver board, it might be written as "SDA" and so on.
    E_TFT_SCLK = 14,
    E_TFT_CS = 15,  // Chip select control pin
    E_TFT_DC = 2,   // Data Command control pin
    E_TFT_RST = -1, // Reset pin (could connect to Arduino RESET pin)
    E_TFT_BL = 27,  // LED back-light
};

enum Backlight
{
    E_LEDC_TIMER = LEDC_TIMER_0,
    E_LEDC_MODE = LEDC_LOW_SPEED_MODE,
    E_LEDC_OUTPUT_IO = E_TFT_BL, // Define the output GPIO
    E_LEDC_CHANNEL = LEDC_CHANNEL_0,
    E_LEDC_DUTY_RES = LEDC_TIMER_13_BIT, // Set duty resolution to 13 bits
    E_LEDC_DUTY = 4095,                  // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
    E_LEDC_FREQUENCY = 5000              // Frequency in Hertz. Set frequency at 5 kHz
};

void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active = false)
{
    gpio_set_level(static_cast<gpio_num_t>(HMIConfiguration::E_TFT_DC), false);
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t)); // Zero out the transaction
    t.length = 8;                             // Command is 8 bits
    t.tx_buffer = &cmd;                       // The data is the cmd itself
    // t.user = (void *)0;    // D/C needs to be set to 0
    if (keep_cs_active)
    {
        // t.flags = SPI_TRANS_CS_KEEP_ACTIVE; // Keep CS active after data transfer
    }
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    ESP_ERROR_CHECK(ret);
    assert(ret == ESP_OK); // Should have had no issues.
}

void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    gpio_set_level(static_cast<gpio_num_t>(HMIConfiguration::E_TFT_DC), true);
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0)
        return;                                 // no need to send anything
    memset(&t, 0, sizeof(t));                   // Zero out the transaction
    t.length = len * 8;                         // Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;                         // Data
    t.user = (void *)1;                         // D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    ESP_ERROR_CHECK(ret);
    assert(ret == ESP_OK); // Should have had no issues.
}

void set_pixel(int32_t x, int32_t y, lv_color_t *color_p)
{
    lcd_data(deviceHandle, (uint8_t *)color_p, 2);
}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    int32_t x, y;
    lcd_cmd(deviceHandle, CMD_WRMC);
    // It's a very slow but simple implementation.
    //  `set_pixel` needs to be written by you to a set pixel on the screen
    for (y = area->y1; y <= area->y2; y++)
    {
        for (x = area->x1; x <= area->x2; x++)
        {
            set_pixel(x, y, color_p);
            color_p++;
        }
    }
    lv_disp_flush_ready(disp); /* Indicate you are ready with the flushing*/
}

void app_main()
{
    esp_err_t err;

    static spi_bus_config_t busConf;
    memset(&busConf, 0, sizeof(spi_bus_config_t));
    busConf.miso_io_num = E_TFT_MISO;
    busConf.mosi_io_num = E_TFT_MOSI;
    busConf.sclk_io_num = E_TFT_SCLK;
    busConf.max_transfer_sz = 32; // SOC_SPI_MAXIMUM_BUFFER_SIZE
    busConf.quadwp_io_num = -1;
    busConf.quadhd_io_num = -1;
    spi_host_device_t master = SPI2_HOST;
    err = spi_bus_initialize(master, &busConf, SPI_DMA_CH_AUTO); // SPI_DMA_DISABLED
    ESP_ERROR_CHECK(err);

    static spi_device_interface_config_t deviceConf;
    memset(&deviceConf, 0, sizeof(spi_device_interface_config_t));
    deviceConf.clock_speed_hz = SPI_MASTER_FREQ_10M; // 1 MHz
    deviceConf.mode = 0;                             // SPI mode 0
    deviceConf.spics_io_num = E_TFT_CS;
    deviceConf.queue_size = 1;
    deviceConf.flags = SPI_DEVICE_HALFDUPLEX;
    deviceConf.pre_cb = NULL;
    deviceConf.post_cb = NULL;

    err = spi_bus_add_device(master, &deviceConf, &deviceHandle);
    ESP_ERROR_CHECK(err);

    // Prepare and then apply the LEDC PWM timer configuration
    static ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
    ledc_timer.freq_hz = E_LEDC_FREQUENCY; // Set output frequency at 5 kHz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    static ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num = E_LEDC_OUTPUT_IO;
    ledc_channel.duty = E_LEDC_DUTY; // Set duty to 0;
    ledc_channel.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, E_LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    gpio_config_t conf = {};
    conf.mode = gpio_mode_t::GPIO_MODE_OUTPUT;
    conf.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_ENABLE;
    uint64_t outputBitMask = 0;
    // outputBitMask = outputBitMask | (1ULL<<BitNumber); this is now to configure which bits are outputs
    // GPIO is to used as BitNumber
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_15);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_2);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_4);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_16);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_17);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_5);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_18);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_19);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_21);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_3); // Used for a console application
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_1);    // Used for a console application
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_22);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_23);

    conf.pin_bit_mask = outputBitMask;
    gpio_config(&conf);

    lcd_cmd(deviceHandle, CMD_SWRESET, true); // Software reset
    vTaskDelay(100 / portTICK_PERIOD_MS);

    static const uint8_t colmod[] = {COLMOD_RGB656}; // 16 bits R5G6B5
    lcd_cmd(deviceHandle, COLMOD_RGB656, true);      // Software reset
    lcd_data(deviceHandle, colmod, sizeof(colmod));  // Set color mode

    // static const uint8_t madctl[] = {MADCTL_MY | MADCTL_RGB}; // Portrait 0 Degrees
    uint8_t madctl[] = {MADCTL_MV | MADCTL_RGB}; // Landscape 90 Degrees

    lcd_cmd(deviceHandle, CMD_MADCTL, true);        // Software reset
    lcd_data(deviceHandle, madctl, sizeof(madctl)); // Set color mode

    static const uint8_t pgc[] = {0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B};
    lcd_cmd(deviceHandle, CMD_PGC, true);     // Software reset
    lcd_data(deviceHandle, pgc, sizeof(pgc)); // Set color mode

    static const uint8_t ngc[] = {0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B};
    lcd_cmd(deviceHandle, CMD_NGC, true);     // Software reset
    lcd_data(deviceHandle, ngc, sizeof(ngc)); // Set color mode

    lcd_cmd(deviceHandle, CMD_INVOFF); // Inversion off
    lcd_cmd(deviceHandle, CMD_NORON);  // Normal display on
    lcd_cmd(deviceHandle, CMD_SLPOUT); // Out of sleep mode
    lcd_cmd(deviceHandle, CMD_DISPON); // Main screen turn on

    lv_init();

    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t buf1[480 * 320 / 10]; /*Declare a buffer for 1/10 screen size*/
    memset(buf1, 128, sizeof(buf1));
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, 480 * 320 / 10); /*Initialize the display buffer.*/

    static lv_disp_drv_t disp_drv;     /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);       /*Basic initialization*/
    disp_drv.flush_cb = my_disp_flush; /*Set your driver function*/
    disp_drv.draw_buf = &draw_buf;     /*Assign the buffer to the display*/
    disp_drv.hor_res = 480;            /*Set the horizontal resolution of the display*/
    disp_drv.ver_res = 320;            /*Set the vertical resolution of the display*/
    disp_drv.direct_mode = false;
    disp_drv.color_format = LV_COLOR_FORMAT_RGB565;
    lv_disp_drv_register(&disp_drv); /*Finally register the driver*/

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);

    /*Create a white label, set its text and align it to the center*/
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello world");
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    // lcd_cmd(deviceHandle, CMD_RAMWR); // Set color mode

    // vTaskDelay(10 / portTICK_PERIOD_MS); /*Sleep for 5 millisecond*/
    // lv_tick_inc(10);                     /*Tell LVGL that 5 milliseconds were elapsed*/
    lv_timer_handler();
}