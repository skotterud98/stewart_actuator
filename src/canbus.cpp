#include "canbus.h"


CANbus::CANbus()
{
    if ((this->_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Socket");
    }

    strcpy(ifr.ifr_name, "vcan0");
    ioctl(this->_socket, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(this->_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
    }

    
    this->tx_len1.can_id = 0xA0;
    this->tx_len1.can_dlc = 6;
    
    this->tx_len2.can_id = 0xA1;
    this->tx_len2.can_dlc = 6;
    
    this->tx_vel1.can_id = 0xA2;
    this->tx_vel1.can_dlc = 7;
    
    this->tx_vel2.can_id = 0xA3;
    this->tx_vel2.can_dlc = 7;
}


CANbus::~CANbus()
{
    if (close(this->_socket) < 0)
    {
        perror("Close");
    }
}


float* CANbus::send_data(const float stroke_len[], const float stroke_vel[])
{
    static uint16_t out_len[6];
    static uint16_t out_vel[6];
    
    uint16_t sign = 0b111111;

    for (uint8_t i = 0; i < 6; i++)
    {
        if(stroke_vel[i] < 0)
        {
            sign &= ~(1UL << i);
            out_vel[i] = (uint16_t)(-1. * stroke_vel[i] * 1000000. + 0.5);
        }
        else
        {
            out_vel[i] = (uint16_t)(stroke_vel[i] * 1000000. + 0.5);
        }
        out_len[i] = (uint16_t)(stroke_len[i] * 100000. + 0.5);
    }
    
    this->tx_len1.data[0] = (uint8_t)(out_len[0] >> 8);   // Actuator 1
    this->tx_len1.data[1] = (uint8_t) out_len[0];
    this->tx_len1.data[2] = (uint8_t)(out_len[1] >> 8);   // Actuator 2
    this->tx_len1.data[3] = (uint8_t) out_len[1];
    this->tx_len1.data[4] = (uint8_t)(out_len[2] >> 8);   // Actuator 3
    this->tx_len1.data[5] = (uint8_t) out_len[2];

    this->tx_len2.data[0] = (uint8_t)(out_len[3] >> 8);   // Actuator 4
    this->tx_len2.data[1] = (uint8_t) out_len[3];
    this->tx_len2.data[2] = (uint8_t)(out_len[4] >> 8);   // Actuator 5
    this->tx_len2.data[3] = (uint8_t) out_len[4];
    this->tx_len2.data[4] = (uint8_t)(out_len[5] >> 8);   // Actuator 6
    this->tx_len2.data[5] = (uint8_t) out_len[5];


    this->tx_vel1.data[0] = (uint8_t)(out_vel[0] >> 8);   // Actuator 1
    this->tx_vel1.data[1] = (uint8_t) out_vel[0];
    this->tx_vel1.data[2] = (uint8_t)(out_vel[1] >> 8);   // Actuator 2
    this->tx_vel1.data[3] = (uint8_t) out_vel[1];
    this->tx_vel1.data[4] = (uint8_t)(out_vel[2] >> 8);   // Actuator 3
    this->tx_vel1.data[5] = (uint8_t) out_vel[2];
    this->tx_vel1.data[6] = sign & 0b000111;

    this->tx_vel2.data[0] = (uint8_t)(out_vel[3] >> 8);   // Actuator 4
    this->tx_vel2.data[1] = (uint8_t) out_vel[3];
    this->tx_vel2.data[2] = (uint8_t)(out_vel[4] >> 8);   // Actuator 5
    this->tx_vel2.data[3] = (uint8_t) out_vel[4];
    this->tx_vel2.data[4] = (uint8_t)(out_vel[5] >> 8);   // Actuator 6
    this->tx_vel2.data[5] = (uint8_t) out_vel[5];
    this->tx_vel2.data[6] = sign >> 3;
    

    write(this->_socket, &this->tx_len1, sizeof(struct can_frame));
    write(this->_socket, &this->tx_len2, sizeof(struct can_frame));
    write(this->_socket, &this->tx_vel1, sizeof(struct can_frame));
    write(this->_socket, &this->tx_vel2, sizeof(struct can_frame));


    static float feedback[6] = {0., 0., 0., 0., 0., 0.};
    struct can_frame rx;
    static uint8_t j = 0;
    
    bool msg1 = false;
    bool msg2 = false;
    /*
    while(!msg1 && !msg2)
    {
        if (read(this->_socket, &rx, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            perror("Read");
        }
        

        if (rx.can_id == 0xF0)
        {
            j = 0;

            for (uint8_t i = 0; i < 3; i++)
            {
                uint16_t in_len = rx.data[j] << 8;
                in_len += rx.data[j + 1];

                feedback[i] = (float) in_len / 100000.;

                j += 2;
            }
            msg1 = true;
        }

        if (rx.can_id == 0xF1)
        {
            j = 0;
            
            for (uint8_t i = 3; i < 6; i++)
            {
                uint16_t in_len = rx.data[j] << 8;
                in_len += rx.data[j + 1];

                feedback[i] = (float) in_len / 100000.;

                j += 2;
            }
            msg2 = true;
        }
    }*/

    return feedback;
}