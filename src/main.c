#include <stdio.h>
#include "pico/stdlib.h"

#include "gyro.h"
#include "sbus.h"
#include "motors.h"


#define CHAN_LV (0)
#define CHAN_LH (1)
#define CHAN_RV (2)
#define CHAN_RH (3)
#define CHAN_POT (4)


int main()
{
    stdio_init_all();
    motors_init();
    sbus_init();
    gyro_init(0, 0, GYRO_FS_245DPS, false, false, GYRO_OUT_SEL_LPF1);

    struct sbus_packet packet;

    float vel0 = 0.f;
    float vel1 = 0.f;
    float ang = 0.f;
    bool new_gyro;

    for (uint timestep = 0;; timestep++)
    {
        sbus_read(&packet);

        new_gyro = gyro_read(&vel1);
        if (new_gyro)
        {
            float vel_offset = SBUS_CHAN_TO_F32(packet.channels[CHAN_POT]) * 2.f - 1.f;
            vel1 += vel_offset;
            ang += (vel0 + vel1) * ((1.f / 100.f) * 0.5f);
            if (ang > 180.f) ang -= 360.f;
            else if (ang < -180.f) ang += 360.f;
            vel0 = vel1;
        }

        uint8_t ml = SBUS_CHAN_TO_U8(packet.channels[CHAN_LV]);
        uint8_t mr = SBUS_CHAN_TO_U8(packet.channels[CHAN_POT]);
        motors_set(ml, mr);

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
            printf("ml: %d\tmr: %d\n", ml, mr);
            printf("\n");
        }
    }

    return 0;
}
