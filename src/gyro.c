#include "gyro.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define SPI_HZ (10000000)

#define SPI_WRITE_BIT     (0b00000000)
#define SPI_READ_BIT      (0b10000000)
#define SPI_INCREMENT_BIT (0b01000000)

#define CTRL1_XEN_BIT (1u << 1)
#define CTRL1_YEN_BIT (1u << 0)
#define CTRL1_ZEN_BIT (1u << 2)
#define CTRL1_PD_BIT  (1u << 3)
#define CTRL1_BW_OFFSET (4)
#define CTRL1_DR_OFFSET (6)

#define CTRL4_FS_OFFSET (4)

#define CTRL5_OUT_SEL_MASK (0b111)
#define CTRL5_OUT_SEL_OFFSET (0)
#define CTRL5_HP_EN_BIT (1u << 4)

#define STATUS_ZYXDA_BIT (1u << 3)

#define LOW_ODR_I2C_DIS_BIT (1u << 3)
#define LOW_ODR_LOW_ODR_BIT (1u << 0)

#define L3GD20H_WHOAMI (0b11010111)

#define SPI_INST (spi0)
#define CS_PIN (17)
#define RX_PIN (16)
#define TX_PIN (19)
#define SCK_PIN (18)

enum REG
{
    WHO_AM_I       = 0x0F,

    CTRL1          = 0x20,
    CTRL2          = 0x21,
    CTRL3          = 0x22,
    CTRL4          = 0x23,
    CTRL5          = 0x24,
    REFERENCE      = 0x25,
    OUT_TEMP       = 0x26,
    STATUS         = 0x27,

    OUT_X_L        = 0x28,
    OUT_X_H        = 0x29,
    OUT_Y_L        = 0x2A,
    OUT_Y_H        = 0x2B,
    OUT_Z_L        = 0x2C,
    OUT_Z_H        = 0x2D,

    FIFO_CTRL      = 0x2E,
    FIFO_SRC       = 0x2F,

    IG_CFG         = 0x30,
    IG_SRC         = 0x31,
    IG_THS_XH      = 0x32,
    IG_THS_XL      = 0x33,
    IG_THS_YH      = 0x34,
    IG_THS_YL      = 0x35,
    IG_THS_ZH      = 0x36,
    IG_THS_ZL      = 0x37,
    IG_DURATION    = 0x38,

    LOW_ODR        = 0x39,
};

static const float MDPS_SCALE_FOR_FS[] = {
    8.75f,
    17.5f,
    70.f,
};

static enum gyro_fs g_chosen_fs;

static inline void cs_select()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(CS_PIN, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(CS_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static void read_reg(uint8_t reg, uint8_t *data, int count, bool increment)
{
    cs_select();

    uint8_t first_byte = reg;
    first_byte |= SPI_READ_BIT;
    if (increment)
        first_byte |= SPI_INCREMENT_BIT;

    spi_write_blocking(SPI_INST, &first_byte, 1);
    spi_read_blocking(SPI_INST, 0, data, count);

    cs_deselect();
}

static void write_reg(uint8_t reg, const uint8_t *data, int count, bool increment)
{
    cs_select();

    uint8_t first_byte = reg;
    first_byte |= SPI_WRITE_BIT;
    if (increment)
        first_byte |= SPI_INCREMENT_BIT;

    spi_write_blocking(SPI_INST, &first_byte, 1);
    spi_write_blocking(SPI_INST, data, count);

    cs_deselect();
}

static uint8_t who_am_i(void)
{
    uint8_t val;
    read_reg(WHO_AM_I, &val, 1, false);
    return val;
}

static void set_enable(bool power, bool x, bool y, bool z)
{
    uint8_t val;
    read_reg(CTRL1, &val, 1, false);

    if (power)
        val |= CTRL1_PD_BIT;
    else
        val &= ~CTRL1_PD_BIT;

    if (x)
        val |= CTRL1_XEN_BIT;
    else
        val &= ~CTRL1_XEN_BIT;

    if (y)
        val |= CTRL1_YEN_BIT;
    else
        val &= ~CTRL1_YEN_BIT;

    if (z)
        val |= CTRL1_ZEN_BIT;
    else
        val &= ~CTRL1_YEN_BIT;

    write_reg(CTRL1, &val, 1, false);
}

void gyro_init(int dr, int bw, enum gyro_fs fs, bool low_odr, bool hp_en, enum gyro_out_sel out_sel)
{
    spi_init(SPI_INST, SPI_HZ);
    gpio_set_function(RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);

    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);

    printf("WHO_AM_I: 0x%02x\n", who_am_i());

    uint8_t val = 0;
    val |= LOW_ODR_I2C_DIS_BIT;
    if (low_odr)
        val |= LOW_ODR_LOW_ODR_BIT;
    write_reg(LOW_ODR, &val, 1, false);

    val = 0;
    val |= fs << CTRL4_FS_OFFSET;
    write_reg(CTRL4, &val, 1, false);
    g_chosen_fs = fs;

    val = 0;
    val |= dr << CTRL1_DR_OFFSET;
    val |= bw << CTRL1_BW_OFFSET;
    write_reg(CTRL1, &val, 1, false);

    val = 0;
    val |= out_sel << CTRL5_OUT_SEL_OFFSET;
    if (hp_en) val |= CTRL5_HP_EN_BIT;
    write_reg(CTRL5, &val, 1, false);

    set_enable(true, false, false, true);
}

bool gyro_read(float *vel)
{
    uint8_t buf[3 * 2];
    int16_t gz;
    uint8_t status;

    read_reg(STATUS, &status, 1, false);
    if (!(status & STATUS_ZYXDA_BIT))
        return false;
    
    read_reg(OUT_X_L, buf, sizeof buf, true);

    gz = (int16_t) (buf[5] << 8 | buf[4]);

    *vel = MDPS_SCALE_FOR_FS[g_chosen_fs] * (float) gz / 1000.f;

    return true;
}
