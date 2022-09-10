#ifndef SERIAL_MEGAPI_H_
#define SERIAL_MEGAPI_H_

#define MAKEBLOCK_MSG_SIZE  0xa
#define MKBLK_MAX_MSG_SIZE  0x14

#define HEADER_MSG_SIZE     0x3

#define GYRO_ALL_AXES       0x0
#define GYRO_AXE_X          0x2
#define GYRO_AXE_Y          0x1
#define GYRO_AXE_Z          0x3
#define GYRO_DEV_ID         0x6
#define GYRO_MSG_SIZE       0x5
#define GYRO_PORT           0x0

#define USS_DEV_ID          0x1
#define USS_MSG_SIZE        0x4
#define USS_MAX_NB          0x4
#define USS_FIRST_SLOT      0x5
#define USS_LAST_SLOT       USS_FIRST_SLOT + USS_MAX_NB - 1

#define DATA_TYPE_FLOAT     0x2

#define ACTION_GET          0x1
#define ACTION_RUN          0x2
#define ACTION_RESET        0x4
#define ACTION_START        0x5

float get_gyro_x();
float get_gyro_y();
float get_gyro_z();
float get_uss(int port);

int request_gyro_all_axes(const int fd);
int request_gyro(const int fd, char axis);
int request_uss(const int fd, char port);

void receive_msg(const int fd);

#endif // SERIAL_MEGAPI_H_