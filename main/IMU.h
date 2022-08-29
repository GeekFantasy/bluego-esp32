#ifndef IMU
#define IMU

#define DATA_LEN 10
#define IMU_UART_TXD 17
#define IMU_UART_RXD 16
#define IMU_UART_RTS (UART_PIN_NO_CHANGE)
#define IMU_UART_CTS (UART_PIN_NO_CHANGE)

#define IMU_UART_PORT_NUM (2)
#define IMU_UART_BAUD_RATE (115200)
#define IMU_TASK_STACK_SIZE (2048)
#define UART_LOG_PORT_NUM 0

#define DATA_FRAME_LEN (11)
#define READ_BUFF_SIZE (DATA_FRAME_LEN * 3)
#define BUF_SIZE (READ_BUFF_SIZE * 10)

typedef struct
{
    float x;
    float y;
    float z;
    float temp;
} acceleration;


typedef struct
{
    float x;
    float y;
    float z;
    float temp;
} angular_rate;

typedef struct
{
    float x;
    float y;
    float z;
    float temp;
} angle;

int check_data(unsigned char buff[]);
acceleration get_acceleration(unsigned char buff[]);
angular_rate get_angular_rate(unsigned char buff[]);
angle get_angle(unsigned char buff[]);

#endif
