#include "mpu6050.h"

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define SENSOR_RADIUS_M (0.010f) // 10mm

#define I2C_INST (i2c1) // (i2c0)
#define I2C_HZ (400000)
#define I2C_SDA_PIN (26) // (20)
#define I2C_SCL_PIN (27) // (21)

// 0x69 when AD0 pin to Vcc
#define MPU6050_ADDRESS (0x68)

#define MPU6050_REG_ACCEL_XOFFS_H     (0x06)
#define MPU6050_REG_ACCEL_XOFFS_L     (0x07)
#define MPU6050_REG_ACCEL_YOFFS_H     (0x08)
#define MPU6050_REG_ACCEL_YOFFS_L     (0x09)
#define MPU6050_REG_ACCEL_ZOFFS_H     (0x0A)
#define MPU6050_REG_ACCEL_ZOFFS_L     (0x0B)
#define MPU6050_REG_GYRO_XOFFS_H      (0x13)
#define MPU6050_REG_GYRO_XOFFS_L      (0x14)
#define MPU6050_REG_GYRO_YOFFS_H      (0x15)
#define MPU6050_REG_GYRO_YOFFS_L      (0x16)
#define MPU6050_REG_GYRO_ZOFFS_H      (0x17)
#define MPU6050_REG_GYRO_ZOFFS_L      (0x18)
#define MPU6050_REG_CONFIG            (0x1A)
#define MPU6050_REG_GYRO_CONFIG       (0x1B)
#define MPU6050_REG_ACCEL_CONFIG      (0x1C)
#define MPU6050_REG_FF_THRESHOLD      (0x1D)
#define MPU6050_REG_FF_DURATION       (0x1E)
#define MPU6050_REG_MOT_THRESHOLD     (0x1F)
#define MPU6050_REG_MOT_DURATION      (0x20)
#define MPU6050_REG_ZMOT_THRESHOLD    (0x21)
#define MPU6050_REG_ZMOT_DURATION     (0x22)
#define MPU6050_REG_INT_PIN_CFG       (0x37)
#define MPU6050_REG_INT_ENABLE        (0x38)
#define MPU6050_REG_INT_STATUS        (0x3A)
#define MPU6050_REG_ACCEL_XOUT_H      (0x3B)
#define MPU6050_REG_ACCEL_XOUT_L      (0x3C)
#define MPU6050_REG_ACCEL_YOUT_H      (0x3D)
#define MPU6050_REG_ACCEL_YOUT_L      (0x3E)
#define MPU6050_REG_ACCEL_ZOUT_H      (0x3F)
#define MPU6050_REG_ACCEL_ZOUT_L      (0x40)
#define MPU6050_REG_TEMP_OUT_H        (0x41)
#define MPU6050_REG_TEMP_OUT_L        (0x42)
#define MPU6050_REG_GYRO_XOUT_H       (0x43)
#define MPU6050_REG_GYRO_XOUT_L       (0x44)
#define MPU6050_REG_GYRO_YOUT_H       (0x45)
#define MPU6050_REG_GYRO_YOUT_L       (0x46)
#define MPU6050_REG_GYRO_ZOUT_H       (0x47)
#define MPU6050_REG_GYRO_ZOUT_L       (0x48)
#define MPU6050_REG_MOT_DETECT_STATUS (0x61)
#define MPU6050_REG_MOT_DETECT_CTRL   (0x69)
#define MPU6050_REG_USER_CTRL         (0x6A)
#define MPU6050_REG_PWR_MGMT_1        (0x6B)
#define MPU6050_REG_WHO_AM_I          (0x75)

static float g_dps_per_digit, g_range_per_digit;

static uint8_t read_reg_8(uint8_t reg)
{
    uint8_t data;
    i2c_write_blocking(I2C_INST, MPU6050_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_INST, MPU6050_ADDRESS, &data, 1, false);
    return data;
}

static void write_reg_8(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(I2C_INST, MPU6050_ADDRESS, buf, 2, false);
}

static uint16_t read_reg_16(uint8_t reg)
{
    uint8_t buf[2];
    i2c_write_blocking(I2C_INST, MPU6050_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_INST, MPU6050_ADDRESS, buf, 2, false);
    return (buf[0] << 8) | buf[1];
}

static void write_reg_16(uint8_t reg, uint16_t data)
{
    uint8_t buf[3] = {reg, data >> 8, data & 0xFF};
    i2c_write_blocking(I2C_INST, MPU6050_ADDRESS, buf, 3, false);
}

static bool read_reg_1(uint8_t reg, uint8_t bit)
{
    return read_reg_8(reg) & (1 << bit);
}

static void write_reg_1(uint8_t reg, uint8_t bit, bool data)
{
    uint8_t val = read_reg_8(reg);
    if (data)
        val |= (1 << bit);
    else
        val &= ~(1 << bit);
    write_reg_8(reg, val);
}

static uint8_t who_am_i(void)
{
    return read_reg_8(MPU6050_REG_WHO_AM_I);
}

static void set_clock_source(uint8_t source)
{
    uint8_t value;
    value = read_reg_8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    write_reg_8(MPU6050_REG_PWR_MGMT_1, value);
}

static void set_scale(mpu6050_dps_t scale)
{
    uint8_t value;

    switch (scale)
    {
	case MPU6050_SCALE_250DPS:
	    g_dps_per_digit = .007633f;
	    break;
	case MPU6050_SCALE_500DPS:
	    g_dps_per_digit = .015267f;
	    break;
	case MPU6050_SCALE_1000DPS:
	    g_dps_per_digit = .030487f;
	    break;
	case MPU6050_SCALE_2000DPS:
	    g_dps_per_digit = .060975f;
	    break;
	default:
	    break;
    }

    value = read_reg_8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);
    write_reg_8(MPU6050_REG_GYRO_CONFIG, value);
}

static void set_range(mpu6050_range_t range)
{
    uint8_t value;

    switch (range)
    {
	case MPU6050_RANGE_2G:
	    g_range_per_digit = .000061f;
	    break;
	case MPU6050_RANGE_4G:
	    g_range_per_digit = .000122f;
	    break;
	case MPU6050_RANGE_8G:
	    g_range_per_digit = .000244f;
	    break;
	case MPU6050_RANGE_16G:
	    g_range_per_digit = .0004882f;
	    break;
	default:
	    break;
    }

    value = read_reg_8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    write_reg_8(MPU6050_REG_ACCEL_CONFIG, value);
}

static void set_sleep_enabled(bool enabled)
{
    write_reg_1(MPU6050_REG_PWR_MGMT_1, 6, enabled);
}

static void read_norm_accel(float *rax, float *ray, float *raz)
{
    uint8_t reg;
    uint8_t buf[6];

    reg = MPU6050_REG_ACCEL_XOUT_H;
    i2c_write_blocking(I2C_INST, MPU6050_ADDRESS, &reg, 1, true);

    i2c_read_blocking(I2C_INST, MPU6050_ADDRESS, buf, 6, false);

    *rax = (int16_t)(buf[0] << 8 | buf[1]) * g_range_per_digit * 9.80665f;
    *ray = (int16_t)(buf[2] << 8 | buf[3]) * g_range_per_digit * 9.80665f;
    *raz = (int16_t)(buf[4] << 8 | buf[5]) * g_range_per_digit * 9.80665f;
}

void mpu6050_init(mpu6050_dps_t scale, mpu6050_range_t range)
{
    i2c_init(I2C_INST, I2C_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    uint8_t id = who_am_i();
    if (id != 0x68)
    {
        printf("MPU6050 not found\n");
        return;
    }
    printf("WHO_AM_I: 0x%02x\n", who_am_i);

    set_clock_source(MPU6050_CLOCK_PLL_XGYRO);
    set_scale(scale);
    set_range(range);
    set_sleep_enabled(false);
}

bool mpu6050_read_acc(float *ax, float *ay, float *az)
{
    read_norm_accel(ax, ay, az);
    return true;
}

bool mpu6050_read_gyro(float *gz)
{
    float ax, ay, az;
    read_norm_accel(&ax, &ay, &az);

    float a_planar = sqrtf(ax * ax + ay * ay);

    *gz = sqrtf(a_planar * (1 / SENSOR_RADIUS_M));
}
