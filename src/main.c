#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "gyro.h"
#include "sbus.h"

int main()
{
    stdio_init_all();
    sbus_init();

    struct sbus_packet packet;
    for (;;)
    {
        sbus_read(&packet);
        printf("ch1: %d\tch2: %d\tch3: %d\tch4: %d\tch5: %d\tch15: %d\tfailsafe: %d\tframe_lost: %d\n",
            (int) packet.channels[0],
            (int) packet.channels[1],
            (int) packet.channels[2],
            (int) packet.channels[3],
            (int) packet.channels[4],
            (int) packet.channels[14],
            (int) packet.failsafe,
            (int) packet.frame_lost
        );
    }
}

int gyro_main()
{
    stdio_init_all();
    gyro_init(0, 0, GYRO_FS_245DPS, false, false, GYRO_OUT_SEL_LPF1);

    float vel = 0.f;
    float vel0 = 0.f;
    float vel1 = 0.f;
    float ang = 0.f;
    bool new_data;

    for (;;)
    {
        new_data = gyro_read(&vel);
        if (new_data)
        {
            vel0 = vel1;
            vel1 = vel;
            ang += (vel0 + vel1) * ((1.f / 100.f) * 0.5f);
            if (ang > 180.f)
                ang -= 360.f;
            else if (ang < -180.f)
                ang += 360.f;
            printf("ang: %f\tvel: %f\n", ang, vel1);
        }
    }
    return 0;
}
