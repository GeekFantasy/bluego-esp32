#include <stdio.h>
#include <stddef.h>
#include "IMU.h"

int check_data(unsigned char buff[])
{
    int pass = 0;
    char checksum = 0;

    if (buff != NULL)
    {
        for (int i = 0; i < DATA_LEN; i++)
        {
            checksum += buff[i];
        }
        
        if(checksum == buff[DATA_LEN]) //  the last byte of the data is checksumm 
            pass = 1;
    }

    return pass;
}

/*
get acceleration from buff , which is raw data read from UART
Return NULL, if failed to read data
*/
acceleration get_acceleration(unsigned char buff[])
{
    acceleration acc;

    if (buff != NULL)
    {
        if (buff[0] == 0x55) // head of the package
        {
            if (buff[1] == 0x51) // indication for acceleration
            {
                acc.x = ((short)(buff[3] << 8 | buff[2])) / 32768.0 * 16;
                acc.y = ((short)(buff[5] << 8 | buff[4])) / 32768.0 * 16;
                acc.z = ((short)(buff[7] << 8 | buff[6])) / 32768.0 * 16;
                acc.temp = ((short)(buff[9] << 8 | buff[8])) / 340.0 + 36.25;
            }
        }
    }

    return acc;
}

/*
get acceleration from buff , which is raw data read from UART
Return NULL, if failed to read data
*/
angular_rate get_angular_rate(unsigned char buff[])
{
    angular_rate ar;

    if (buff != NULL)
    {
        if (buff[0] == 0x55) // head of the package
        {
            if (buff[1] == 0x52) // indication for angular_rate
            {
                ar.x = ((short)(buff[3] << 8 | buff[2])) / 32768.0 * 2000;
                ar.y = ((short)(buff[5] << 8 | buff[4])) / 32768.0 * 2000;
                ar.z = ((short)(buff[7] << 8 | buff[6])) / 32768.0 * 2000;
                ar.temp = ((short)(buff[9] << 8 | buff[8])) / 340.0 + 36.25;
            }
        }
    }

    return ar;
}

/*
get acceleration from buff , which is raw data read from UART
Return NULL, if failed to read data
*/
angle get_angle(unsigned char buff[])
{
    angle ang;
    if (buff != NULL)
    {
        if (buff[0] == 0x55) // head of the package
        {
            if (buff[1] == 0x53) // indication for angular
            {
                ang.x = ((short)(buff[3] << 8 | buff[2])) / 32768.0 * 180;
                ang.y = ((short)(buff[5] << 8 | buff[4])) / 32768.0 * 180;
                ang.z = ((short)(buff[7] << 8 | buff[6])) / 32768.0 * 180;
                ang.temp = ((short)(buff[9] << 8 | buff[8])) / 340.0 + 36.25;
            }
        }
    }

    return ang;
}