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

#define SET_MOTOR_MSG_SIZE  0x7
#define READ_MOTOR_MSG_SIZE 0x6
  //Secondary command
  #define READ_MOTOR_POS    0x01
  #define READ_MOTOR_SPEED  0x02

#define MOTOR_DEV_ID        0x3d // 61
#define MOTOR_EXT_ID        0x0
#define ENCODER_PID_MOTION  0x3e // 62
  //Secondary command
  #define ENCODER_BOARD_POS_MOTION_MOVE    0x01
  #define ENCODER_BOARD_SPEED_MOTION       0x02
  #define ENCODER_BOARD_PWM_MOTION         0x03
  #define ENCODER_BOARD_SET_CUR_POS_ZERO   0x04
  #define ENCODER_BOARD_CAR_POS_MOTION     0x05
  #define ENCODER_BOARD_POS_MOTION_MOVETO  0x06
#define TWO_ENCODERS_POS_SPEED  0x3f // 63

#define USS_DEV_ID          0x1
#define USS_MSG_SIZE        0x4
#define USS_MAX_NB          0x4
#define USS_FIRST_SLOT      0x5
#define USS_LAST_SLOT       USS_FIRST_SLOT + USS_MAX_NB - 1

#define DATA_TYPE_FLOAT     0x2
#define DATA_TYPE_LONG      0x6

#define ACTION_GET          0x1
#define ACTION_RUN          0x2
#define ACTION_RESET        0x4
#define ACTION_START        0x5

#ifdef __cplusplus
extern "C" {
#endif
float get_gyro_x();
float get_gyro_y();
float get_gyro_z();
int   get_motor_position(int port);
float get_motor_speed(int port);
float get_uss(int port);

int request_gyro_all_axes(const int fd);
int request_gyro(const int fd, char axis);
int request_motor_position(const int fd, char motor);
int request_motor_speed(const int fd, char motor);
int request_two_motors_pos_speed(const int fd, char motor_1, char motor_2);
int request_uss(const int fd, char port);

void receive_msg(const int fd);

int set_speed(const int fd, char motor, int speed);

#ifdef __cplusplus
}
#endif

#endif // SERIAL_MEGAPI_H_