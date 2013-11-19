#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include "rfm23bp.h"
#include "gpio.h"

#ifdef RSSI

static void rssi(void)
{
    float start = 434.7;
    float end = 435.3;
    float step = 0.008;
    float freq = start;
    uint8_t rssi;
    uint8_t stars;
    uint8_t i;

    RFM23BP__SetModeRx();

    printf("\033[2J");

    printf("\033[H");

    for (freq = start; freq < end; freq += step)
    {
        RFM23BP__SetFrequency(freq, 0.05);

        usleep(1000);

        rssi = RFM23BP__RSSIRead();
        stars = rssi / 4;

        printf("%7.3f:", freq);

        for (i = 0; i < stars; i++)
        {
            printf("*");
        }

        printf("\033[K\n"); // DElete to EOL
    }
}
#endif

// This works with test_rx below
void test_tx(void)
{
    uint8_t data[] = "hello................................12334455\0"; // Max 50....

    if (!RFM23BP__SetFrequency(433.500, 0.05))
    {
        printf("setFrequency failed\n");
    }

    if (!RFM23BP__SetModemConfig(GFSK_Rb38_4Fd19_6))
    {
        printf("setModemConfig failed\n");
    }

    RFM23BP__SetTxPower(TXPOW_28DBM);

    RFM23BP__Send(data, sizeof(data));

    RFM23BP__WaitPacketSent();

    printf("test_tx done!\n");
}

// This works with test_rx below
void test_rx()
{
    int i;

    uint8_t buf[MAX_MESSAGE_LEN];
    uint8_t len = 0;

    if (!RFM23BP__SetFrequency(433.500, 0.05))
    {
        printf("setFrequency failed\n");
    }

    if (!RFM23BP__SetModemConfig(GFSK_Rb38_4Fd19_6))
    {
        printf("setM odemConfig failed\n");
    }

    while (1)
    {
        RFM23BP__SetModeRx();

        RFM23BP__WaitAvailable();

        len = RFM23BP__GetRXBuffer(buf);

        for (i = 0; i < len; i++)
        {
            printf("0x%02x(%c) ", buf[i], buf[i]);fflush(stdout);
        }

        printf("\n");
    }
}

int main()
{

    if (RFM23BP__Init() == false)
    {
        printf("RFM23BP__Init FAILED!\n");
    }
    else
    {
        //uint8_t len = RFM23BP__ReadRegister(RECEIVED_PACKET_LENGTH);

        test_rx();

        printf("----------- DONE ---------\n");

        /*printf("Device Status = 0x%02x\n", RFM23BP__ReadRegister(DEVICE_STATUS));

         while (1 == 1)
         {
         RFM23BP__SetModeRx();
         printf("rx...\n");
         printf("Device Status = 0x%02x\n", RFM23BP__ReadRegister(DEVICE_STATUS));

         //test_tx();
         sleep(4);

         RFM23BP__SetModeTx();
         printf("tx...\n");
         printf("Device Status = 0x%02x\n", RFM23BP__ReadRegister(DEVICE_STATUS));

         sleep(4);

         RFM23BP__SetModeIdle();
         printf("idle...\n");
         printf("Device Status = 0x%02x\n", RFM23BP__ReadRegister(DEVICE_STATUS));

         sleep(4);

         }*/

        RFM23BP__Close();
        //rssi()
    }

    return 1;
}
