#include <cstring>

#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>

#include <freertos/task.h>

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

class HMIIface
{

public:
  HMIIface(/* args */);
  ~HMIIface();

  // private:
  enum HMIConfiguration
  {
    E_TFT_WIDTH = 320,
    E_TFT_HEIGHT = 480, //
    E_TFT_MISO = 12,
    E_TFT_MOSI = 13, // In some display driver board, it might be written as "SDA" and so on.
    E_TFT_SCLK = 14,
    E_TFT_CS = 15,  // Chip select control pin
    E_TFT_DC = 2,   // Data Command control pin
    E_TFT_RST = -1, // Reset pin (could connect to Arduino RESET pin)
    E_TFT_BL = 27,  // LED back-light

    E_TOUCH_CS = 33, // Chip select pin (T_CS) of touch screen

    E_SPI_FREQUENCY = 65000000,
    // Optional reduced SPI frequency for reading TFT
    E_SPI_READ_FREQUENCY = 20000000,
    // The XPT2046 requires a lower SPI clock rate of 2.5MHz so we define that here:
    E_SPI_TOUCH_FREQUENCY = 2500000 // 2500000
  };
  enum HMICommands
  {
    E_CMD_SWRESET = 0x01, // Software Reset
    E_CMD_SLPIN = 0x10,   // Sleep in
    E_CMD_SLPOUT = 0x11,  // Sleep out
    E_CMD_NORON = 0x13,   // Normal Display Mode On
    E_CMD_INVOFF = 0x20,  // Display Inversion Off
    E_CMD_DISPON = 0x29,  // Display On
    E_CMD_CASET = 0x2A,   // Column Address Set
    E_CMD_RASET = 0x2B,   // Row Address Set
    E_CMD_RAMWR = 0x2C,   // Memory Write
    E_CMD_MADCTL = 0x36,  // Memory Data Access Control
    E_CMD_COLMOD = 0x3A,  // Interface Pixel Format
    E_CMD_PGC = 0xE0,     // Positive Gamma Control
    E_CMD_NGC = 0xE1,     // Negative Gamma Control
    E_CMD_CSCON = 0xF0,   // Command Set Control

    E_MADCTL_MY = 0x80,  // Row Address Order - 0=Increment (Top to Bottom), 1=Decrement (Bottom to Top)
    E_MADCTL_MX = 0x40,  // Column Address Order - 0=Increment (Left to Right), 1=Decrement (Right to Left)
    E_MADCTL_MV = 0x20,  // Row/Column exchange - 0=Normal, 1=Row/Column exchanged
    E_MADCTL_ML = 0x10,  // Vertical Refresh Order
    E_MADCTL_BGR = 0x08, // RGB/BGR Order - BGR
    E_MADCTL_MH = 0x10,  // Horizontal Refresh Order
    E_MADCTL_RGB = 0x00  // RGB/BGR Order - RGB
  };

  esp_err_t ret;
  spi_device_handle_t spi2;
  spi_bus_config_t buscfg;
  spi_device_interface_config_t devcfg;

  void spi_init();
  void st7796_send_init_commands();
  void gpio_init();
  void st7796_send_command(const uint8_t command, const uint8_t data[], const ushort length);
};

void HMIIface::spi_init()
{
  buscfg.miso_io_num = HMIConfiguration::E_TFT_MISO;
  buscfg.mosi_io_num = HMIConfiguration::E_TFT_MOSI;
  buscfg.sclk_io_num = HMIConfiguration::E_TFT_SCLK,
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;

  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  devcfg.clock_speed_hz = HMIConfiguration::E_SPI_FREQUENCY; // 1 MHz
  devcfg.mode = 0;                                           // SPI mode 0
  devcfg.spics_io_num = HMIConfiguration::E_TFT_CS;
  devcfg.queue_size = 1;
  devcfg.flags = SPI_DEVICE_HALFDUPLEX;
  devcfg.pre_cb = NULL;
  devcfg.post_cb = NULL;

  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi2);
  ESP_ERROR_CHECK(ret);
}

void HMIIface::gpio_init()
{
  gpio_config_t conf = {};
  conf.mode = gpio_mode_t::GPIO_MODE_OUTPUT;
  conf.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_ENABLE;
  uint64_t outputBitMask = 0;
  // outputBitMask = outputBitMask | (1ULL<<BitNumber); this is now to configure which bits are outputs
  // GPIO is to used as BitNumber
  outputBitMask = outputBitMask | (1ULL << GPIO_NUM_15);
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
}

void HMIIface::st7796_send_command(const uint8_t command, const uint8_t data[] = nullptr, const ushort length = 0)
{
  gpio_set_level(static_cast<gpio_num_t>(HMIConfiguration::E_TFT_DC), false);
  // digitalWrite(ST7796_PIN_DC, LOW); // Command mode => command
  // spi_st7796.beginTransaction(SPISettings(ST7796_SPI_FREQ, MSBFIRST, SPI_MODE0));
  gpio_set_level(static_cast<gpio_num_t>(HMIConfiguration::E_TFT_CS), false);
  // digitalWrite(ST7796_PIN_CS, LOW); // Chip select => enable
  // uint8_t tx_data[2] = {reg, value};

  spi_transaction_t t;
  t.tx_buffer = &command;
  t.length = length;
  // ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t));
  // spi_st7796.write(command);

  if (length > 0)
  {
    gpio_set_level(static_cast<gpio_num_t>(HMIConfiguration::E_TFT_DC), true);
    // digitalWrite(ST7796_PIN_DC, HIGH); // Command mode => data
    ret = spi_device_polling_transmit(spi2, &t); // Transmit!
    ESP_ERROR_CHECK(ret);                       // Should have had no issues.
  }
  gpio_set_level(static_cast<gpio_num_t>(HMIConfiguration::E_TFT_CS), true);
  // digitalWrite(ST7796_PIN_CS, HIGH); // Chip select => disable
  // spi_st7796.endTransaction();
}

void HMIIface::st7796_send_init_commands()
{
  st7796_send_command(HMICommands::E_CMD_SWRESET); // Software reset
  vTaskDelay(100 / portTICK_PERIOD_MS);

  
  static const uint8_t cscon1[] = {0xC3}; // Enable extension command 2 part I
  st7796_send_command(HMICommands::E_CMD_CSCON, cscon1, sizeof(cscon1));
  static const uint8_t cscon2[] = {0x96}; // Enable extension command 2 part II
  st7796_send_command(HMICommands::E_CMD_CSCON, cscon2, sizeof(cscon2));

  static const uint8_t colmod[] = {COLMOD_RGB656};         // 16 bits R5G6B5
  st7796_send_command(HMICommands::E_CMD_COLMOD, colmod, sizeof(colmod)); // Set color mode

  // static const uint8_t madctl[] = {MADCTL_MY | MADCTL_RGB};                         // Portrait 0 Degrees
  static const uint8_t madctl[] = {MADCTL_MV | MADCTL_RGB}; // Landscape 90 Degrees
  // static const uint8_t madctl[] = {MADCTL_MX | MADCTL_RGB};                         // Portrait inverted 180 Degrees
  // static const uint8_t madctl[] = {MADCTL_MY | MADCTL_MX | MADCTL_MV | MADCTL_RGB}; // Landscape inverted 270 Degrees

  st7796_send_command(HMICommands::E_CMD_MADCTL, madctl, sizeof(madctl));

  static const uint8_t pgc[] = {0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B};
  st7796_send_command(HMICommands::E_CMD_PGC, pgc, sizeof(pgc));
  static const uint8_t ngc[] = {0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B};
  st7796_send_command(HMICommands::E_CMD_NGC, ngc, sizeof(ngc));

  static const uint8_t cscon3[] = {0x3C}; // Disable extension command 2 part I
  st7796_send_command(HMICommands::E_CMD_CSCON, cscon3, sizeof(cscon3));
  static const uint8_t cscon4[] = {0x69}; // Disable extension command 2 part II
  st7796_send_command(HMICommands::E_CMD_CSCON, cscon4, sizeof(cscon4));

  st7796_send_command(HMICommands::E_CMD_INVOFF); // Inversion off
  st7796_send_command(HMICommands::E_CMD_NORON);  // Normal display on
  st7796_send_command(HMICommands::E_CMD_SLPOUT); // Out of sleep mode
  st7796_send_command(HMICommands::E_CMD_DISPON); // Main screen turn on

};

HMIIface::HMIIface(/* args */)
{
}

HMIIface::~HMIIface()
{
}

void app_main()
{
  HMIIface hmi;
  hmi.spi_init();
  hmi.gpio_init();
  hmi.st7796_send_init_commands();

  while(true)
  {

  }
}

/*

#include <stdio.h>
#include <sys/time.h>

#include "driver/ledc.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lvgl.h"

extern "C"
{
  void app_main(void);
}

#define GT_CMD_WR 0XBA // 写命令0xBA
#define GT_CMD_RD 0XBB // 读命令0XBB
//
// #define GT_CMD_WR           0X28         //
// #define GT_CMD_RD           0X29         //

#define GT911_MAX_WIDTH 320  // Touchscreen pad max width
#define GT911_MAX_HEIGHT 480 // Touchscreen pad max height

// GT911 部分寄存器定义
#define GT_CTRL_REG 0X8040  // GT911控制寄存器
#define GT_CFGS_REG 0X8047  // GT911配置起始地址寄存器
#define GT_CHECK_REG 0X80FF // GT911校验和寄存器
#define GT_PID_REG 0X8140   // GT911产品ID寄存器

#define GT_GSTID_REG 0X814E      // GT911当前检测到的触摸情况
#define GT911_READ_XY_REG 0x814E // 坐标寄存器
#define CT_MAX_TOUCH 5           // 电容触摸屏最大支持的点数

int IIC_SCL = 32;
int IIC_SDA = 33;
int IIC_RST = 25;

#define ST7796_DRIVER

#define TFT_WIDTH 320
#define TFT_HEIGHT 480 //

#define TFT_MISO 12
#define TFT_MOSI 13 // In some display driver board, it might be written as "SDA" and so on.
#define TFT_SCLK 14
#define TFT_CS 15  // Chip select control pin
#define TFT_DC 2   // Data Command control pin
#define TFT_RST -1 // Reset pin (could connect to Arduino RESET pin)
#define TFT_BL 27  // LED back-light

#define TOUCH_CS 33 // Chip select pin (T_CS) of touch screen

#define SPI_FREQUENCY 65000000
// Optional reduced SPI frequency for reading TFT
#define SPI_READ_FREQUENCY 20000000
// The XPT2046 requires a lower SPI clock rate of 2.5MHz so we define that here:
#define SPI_TOUCH_FREQUENCY 2500000 // 2500000

#define CMD_SWRESET 0x01 // Software Reset
#define CMD_SLPIN 0x10   // Sleep in
#define CMD_SLPOUT 0x11  // Sleep out
#define CMD_NORON 0x13   // Normal Display Mode On
#define CMD_INVOFF 0x20  // Display Inversion Off
#define CMD_DISPON 0x29  // Display On
#define CMD_CASET 0x2A   // Column Address Set
#define CMD_RASET 0x2B   // Row Address Set
#define CMD_RAMWR 0x2C   // Memory Write
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

spi_device_handle_t spi2;
static void spi_init()
{
  esp_err_t ret;

  spi_bus_config_t buscfg;
  buscfg.miso_io_num = TFT_MISO;
  buscfg.mosi_io_num = TFT_MOSI;
  buscfg.sclk_io_num = TFT_SCLK,
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 32;

  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  spi_device_interface_config_t devcfg;
  devcfg.clock_speed_hz = SPI_FREQUENCY; // 1 MHz
  devcfg.mode = 0;                       // SPI mode 0
  devcfg.spics_io_num = TFT_CS;
  devcfg.queue_size = 1;
  devcfg.flags = SPI_DEVICE_HALFDUPLEX;
  devcfg.pre_cb = NULL;
  devcfg.post_cb = NULL;

  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi2));
};

void st7796_send_command(const uint8_t command, const uint8_t data[] = nullptr, const ushort length = 0)
{
  gpio_set_level(GPIO_NUM_2, false);
  // digitalWrite(ST7796_PIN_DC, LOW); // Command mode => command
  // spi_st7796.beginTransaction(SPISettings(ST7796_SPI_FREQ, MSBFIRST, SPI_MODE0));
  gpio_set_level(GPIO_NUM_15, false);
  // digitalWrite(ST7796_PIN_CS, LOW); // Chip select => enable
  //uint8_t tx_data[2] = {reg, value};
  spi_transaction_t t;
  t.tx_buffer = command;
  t.length = 2 * 8;
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t));
  // spi_st7796.write(command);

  if (length > 0)
  {
    gpio_set_level(GPIO_NUM_2, true);
    // digitalWrite(ST7796_PIN_DC, HIGH); // Command mode => data
    SPI_SWAP_DATA_TX(data, length);
    // spi_st7796.writeBytes(data, length);
  }
  gpio_set_level(GPIO_NUM_15, true);
  // digitalWrite(ST7796_PIN_CS, HIGH); // Chip select => disable
  // spi_st7796.endTransaction();
}

void st7796_send_init_commands()
{
  st7796_send_command(CMD_SWRESET); // Software reset
  vTaskDelay(100 / portTICK_PERIOD_MS);

  static const uint8_t cscon1[] = {0xC3}; // Enable extension command 2 part I
  st7796_send_command(CMD_CSCON, cscon1, sizeof(cscon1));
  static const uint8_t cscon2[] = {0x96}; // Enable extension command 2 part II
  st7796_send_command(CMD_CSCON, cscon2, sizeof(cscon2));

  static const uint8_t colmod[] = {COLMOD_RGB656};         // 16 bits R5G6B5
  st7796_send_command(CMD_COLMOD, colmod, sizeof(colmod)); // Set color mode

  // static const uint8_t madctl[] = {MADCTL_MY | MADCTL_RGB};                         // Portrait 0 Degrees
  static const uint8_t madctl[] = {MADCTL_MV | MADCTL_RGB}; // Landscape 90 Degrees
  // static const uint8_t madctl[] = {MADCTL_MX | MADCTL_RGB};                         // Portrait inverted 180 Degrees
  // static const uint8_t madctl[] = {MADCTL_MY | MADCTL_MX | MADCTL_MV | MADCTL_RGB}; // Landscape inverted 270 Degrees

  st7796_send_command(CMD_MADCTL, madctl, sizeof(madctl));

  static const uint8_t pgc[] = {0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B};
  st7796_send_command(CMD_PGC, pgc, sizeof(pgc));
  static const uint8_t ngc[] = {0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B};
  st7796_send_command(CMD_NGC, ngc, sizeof(ngc));

  static const uint8_t cscon3[] = {0x3C}; // Disable extension command 2 part I
  st7796_send_command(CMD_CSCON, cscon3, sizeof(cscon3));
  static const uint8_t cscon4[] = {0x69}; // Disable extension command 2 part II
  st7796_send_command(CMD_CSCON, cscon4, sizeof(cscon4));

  st7796_send_command(CMD_INVOFF); // Inversion off
  st7796_send_command(CMD_NORON);  // Normal display on
  st7796_send_command(CMD_SLPOUT); // Out of sleep mode
  st7796_send_command(CMD_DISPON); // Main screen turn on
};

void app_main(void)
{
  gpio_config_t conf = {};
  conf.mode = gpio_mode_t::GPIO_MODE_OUTPUT;
  conf.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_ENABLE;
  uint64_t outputBitMask = 0;
  // outputBitMask = outputBitMask | (1ULL<<BitNumber); this is now to configure which bits are outputs
  // GPIO is to used as BitNumber
  outputBitMask = outputBitMask | (1ULL << GPIO_NUM_15);
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
  spi_init();

  while (true)
  {
  }
}
*/
/*
#define TAG "TEST"

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (12 * 1000 * 1000)

#define EXAMPLE_LCD_H_RES 800
#define EXAMPLE_LCD_V_RES 480

// see ESP32-4827S043-1.png
#define EXAMPLE_PIN_NUM_DCLK   (gpio_num_t)GPIO_NUM_42
#define EXAMPLE_PIN_NUM_VSYNC  (gpio_num_t)GPIO_NUM_41
#define EXAMPLE_PIN_NUM_HSYNC  (gpio_num_t)GPIO_NUM_39
#define EXAMPLE_PIN_NUM_DE     (gpio_num_t)GPIO_NUM_40

#define EXAMPLE_PIN_NUM_DATA0  (gpio_num_t)GPIO_NUM_8  // B3
#define EXAMPLE_PIN_NUM_DATA1  (gpio_num_t)GPIO_NUM_3  // B4
#define EXAMPLE_PIN_NUM_DATA2  (gpio_num_t)GPIO_NUM_46 // B5
#define EXAMPLE_PIN_NUM_DATA3  (gpio_num_t)GPIO_NUM_9  // B6
#define EXAMPLE_PIN_NUM_DATA4  (gpio_num_t)GPIO_NUM_1  // B7
#define EXAMPLE_PIN_NUM_DATA5  (gpio_num_t)GPIO_NUM_5  // G2
#define EXAMPLE_PIN_NUM_DATA6  (gpio_num_t)GPIO_NUM_6  // G3
#define EXAMPLE_PIN_NUM_DATA7  (gpio_num_t)GPIO_NUM_7  // G4
#define EXAMPLE_PIN_NUM_DATA8  (gpio_num_t)GPIO_NUM_15 // G5
#define EXAMPLE_PIN_NUM_DATA9  (gpio_num_t)GPIO_NUM_16 // G6
#define EXAMPLE_PIN_NUM_DATA10 (gpio_num_t)GPIO_NUM_4  // G7
#define EXAMPLE_PIN_NUM_DATA11 (gpio_num_t)GPIO_NUM_45 // R3
#define EXAMPLE_PIN_NUM_DATA12 (gpio_num_t)GPIO_NUM_48 // R4
#define EXAMPLE_PIN_NUM_DATA13 (gpio_num_t)GPIO_NUM_47 // R5
#define EXAMPLE_PIN_NUM_DATA14 (gpio_num_t)GPIO_NUM_21 // R6
#define EXAMPLE_PIN_NUM_DATA15 (gpio_num_t)GPIO_NUM_14 // R7

typedef struct {
  lv_obj_t *meter;
  lv_meter_indicator_t *indicator;
}TMeter;

TMeter *lvgl_demo(lv_disp_t *disp)
{
static TMeter lvgl_meter;

  lv_obj_t *scr = lv_disp_get_scr_act(disp);
  lv_obj_t *meter = lv_meter_create(scr);

  lv_obj_center(meter);
  lv_obj_set_size(meter, 400, 400);

  lv_meter_scale_t * scale = lv_meter_add_scale(meter);
  lv_meter_set_scale_range(meter, scale, 0, 60, 270, 135);
  lv_meter_set_scale_ticks(meter, scale, 5, 5, 32, lv_palette_main(LV_PALETTE_GREY));

  lv_meter_indicator_t *mi;

  mi = lv_meter_add_arc(meter, scale, 10, lv_palette_main(LV_PALETTE_BLUE), 0);
  lv_meter_set_indicator_start_value(meter, mi, 0);
  lv_meter_set_indicator_end_value(meter, mi, 20);

  mi = lv_meter_add_arc(meter, scale, 10, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_meter_set_indicator_start_value(meter, mi, 20);
  lv_meter_set_indicator_end_value(meter, mi, 40);

  mi = lv_meter_add_arc(meter, scale, 10, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_meter_set_indicator_start_value(meter, mi, 40);
  lv_meter_set_indicator_end_value(meter, mi, 60);

  lvgl_meter.indicator = lv_meter_add_needle_line(meter, scale, 5, lv_palette_main(LV_PALETTE_PINK), 0);
  lvgl_meter.meter = meter;

  return &lvgl_meter;
}

void BacklightInit(void)
{
ledc_timer_config_t ledTimer = {
    .speed_mode         = LEDC_LOW_SPEED_MODE,
    .duty_resolution    = LEDC_TIMER_12_BIT,
    .timer_num          = LEDC_TIMER_0,
    .freq_hz            = 5000,
    .clk_cfg            = LEDC_AUTO_CLK
};

ledc_channel_config_t ledChannel = {
    .gpio_num           = GPIO_NUM_2,
    .speed_mode         = LEDC_LOW_SPEED_MODE,
    .channel            = LEDC_CHANNEL_0,
    .intr_type          = LEDC_INTR_DISABLE,
    .timer_sel          = LEDC_TIMER_0,
    .duty               = 0,
    .hpoint             = 0,
    .flags.output_invert = 0
  };

  ESP_ERROR_CHECK(ledc_timer_config(&ledTimer));
  ESP_ERROR_CHECK(ledc_channel_config(&ledChannel));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

esp_lcd_panel_handle_t DisplayInit(void)
{
  ESP_LOGI(TAG, "Install RGB LCD panel driver");

  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .timings = {
          .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
          .h_res =   EXAMPLE_LCD_H_RES,
          .v_res =   EXAMPLE_LCD_V_RES,
          .hsync_pulse_width = 4,
          .hsync_back_porch = 8,
          .hsync_front_porch = 8,
          .vsync_pulse_width = 4,
          .vsync_back_porch = 8,
          .vsync_front_porch = 8,
          .flags.hsync_idle_low = true,
          .flags.vsync_idle_low = true,
          .flags.de_idle_high = false,
          .flags.pclk_active_neg = true,
          .flags.pclk_idle_high = false
        },
        .data_width = 16,
        .sram_trans_align = 8,
        .psram_trans_align = 64,
        .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
        .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
        .de_gpio_num =    EXAMPLE_PIN_NUM_DE,
        .pclk_gpio_num =  EXAMPLE_PIN_NUM_DCLK,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .disp_gpio_num = GPIO_NUM_NC,
        .on_frame_trans_done = NULL,
        .user_ctx = NULL,
        .flags.disp_active_low = false,
        .flags.relax_on_idle = false,
        .flags.fb_in_psram = true
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    return panel_handle;
}


static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;

  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
  lv_disp_flush_ready(drv);
}

static void example_increase_lvgl_tick(void *arg)
{
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void app_main(void)
{
static esp_lcd_panel_handle_t panel_handle;
TMeter *lvgl_meter;

  ESP_LOGI(TAG, "Initialize hardware");
  BacklightInit();
  panel_handle = DisplayInit();


  ESP_LOGI(TAG, "Initialize LVGL library");
  static lv_disp_draw_buf_t disp_buf;
  static lv_disp_drv_t disp_drv;
  lv_init();

  ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
  void *buf1 = NULL;
  void *buf2 = NULL;

  buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf1);
  buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf2);

  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 100);

  ESP_LOGI(TAG, "Register display driver to LVGL");
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = example_lvgl_flush_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
  disp_drv.full_refresh = true;

  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  ESP_LOGI(TAG, "Install LVGL tick timer");
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &example_increase_lvgl_tick,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "lvgl_tick",
      .skip_unhandled_events = true
  };

  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

  ESP_LOGI(TAG, "LVGL demo");
  lvgl_meter = lvgl_demo(disp);

  uint8_t lastSec=0;
  struct timeval tv_now;

  while(42)
  {
    gettimeofday(&tv_now, NULL);

    if (lastSec != tv_now.tv_sec%60)
    {
      lastSec = tv_now.tv_sec%60;
      lv_meter_set_indicator_value(lvgl_meter->meter, lvgl_meter->indicator, lastSec);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    lv_timer_handler();
  }
}
*/