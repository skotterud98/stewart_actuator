#ifndef CANBUS_H
#define CANBUS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>


class CANbus
{
    public:
        // Constructor
        CANbus();

        // Destructor
        ~CANbus();

        float* send_data(const float stroke_len[], const float stroke_vel[]);

    private:
        int _socket;
        struct sockaddr_can addr;
        struct ifreq ifr;

        struct can_frame tx_len1;
        struct can_frame tx_len2;
        struct can_frame tx_vel1;
        struct can_frame tx_vel2;
};

#endif