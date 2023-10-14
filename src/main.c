#include <stdio.h>
#include <math.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>

#include "mpu6050.h"
#include "sbus.h"
#include "motors.h"

#define DEBUG

#define LED_PIN (PICO_DEFAULT_LED_PIN)

#define CHAN_LV (0)
#define CHAN_LH (3)
#define CHAN_RV (2)
#define CHAN_RH (1)
#define CHAN_POT (4)

#define DEG2RAD(x) ((x) * M_PI / 180.f)
#define RAD2DEG(x) ((x) * 180.f / M_PI)


int main()
{
    // watchdog_enable(100, 1);

    stdio_init_all();

    if (watchdog_caused_reboot())
    {
        printf("Watchdog caused reboot\n");
    }

    motors_init();
    sbus_init();
    mpu6050_init(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    struct sbus_packet packet;

    float ax = 0.f;
    float ay = 0.f;
    float az = 0.f;
    float vel0 = 0.f;
    float vel1 = 0.f;
    float ang = 0.f;
    bool new_gyro;

    for (uint timestep = 0;; timestep++)
    {
        sbus_read(&packet);

        // new_gyro = gyro_read(&vel1);
        new_gyro = false;
        if (new_gyro)
        {
            float vel_offset =
                (SBUS_CHAN_TO_F32(packet.channels[CHAN_POT]) * 2.f - 1.f) +
                50.f * (SBUS_CHAN_TO_F32(packet.channels[CHAN_LH]) * 2.f - 1.f);
            vel1 += vel_offset;
            ang += (vel0 + vel1) * ((1.f / 800.f) * 0.5f);
            if (ang > 180.f) ang -= 360.f;
            else if (ang < -180.f) ang += 360.f;
            vel0 = vel1;
        }

        mpu6050_read_acc(&ax, &ay, &az);

        gpio_put(LED_PIN, -30.f < ang && ang < 30.f);

        float speed_base = SBUS_CHAN_TO_F32(packet.channels[CHAN_LV]);
        float speed_offset = SBUS_CHAN_TO_F32(packet.channels[CHAN_RV]) * 2.f - 1.f;

        float left_offset = speed_offset * (cosf(DEG2RAD(ang)) + 1.f) * 0.5f;
        float speed_left = speed_base * (1.f - left_offset);

        float right_offset = speed_offset * (-cosf(DEG2RAD(ang)) + 1.f) * 0.5f;
        float speed_right = speed_base * (1.f - right_offset);

        motors_set(speed_left, speed_right);

        #ifdef DEBUG
        if (timestep % 10000 == 0)
        {
            printf("LV: %d\tLH: %d\tRV: %d\tRH: %d\tPOT: %d\tfailsafe: %d\tframe_lost: %d\n",
                (int) packet.channels[CHAN_LV],
                (int) packet.channels[CHAN_LH],
                (int) packet.channels[CHAN_RV],
                (int) packet.channels[CHAN_RH],
                (int) packet.channels[CHAN_POT],
                (int) packet.failsafe,
                (int) packet.frame_lost
            );
            printf("ang: %f\tvel: %f\n", ang, vel1);
            printf("ax: %f\tay: %f\taz: %f\n", ax, ay, az);
            printf("ml: %f\tmr: %f\n", speed_left, speed_right);
            printf("\n");
        }
        #endif
    }

    return 0;
}
