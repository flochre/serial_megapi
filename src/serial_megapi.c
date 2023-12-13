#include "serial_megapi.h"

#include <math.h>

char makeblock_response_msg[MKBLK_MAX_MSG_SIZE];

union
{
  char byteVal[2];
  int shortVal;
}valShort;

typedef struct SerialGyro
{
    float roll_;
    float pitch_;
    float yaw_;
    float ang_x_;
    float ang_y_;
    float ang_z_;
    float lin_x_;
    float lin_y_;
    float lin_z_;
} SerialGyro;

typedef struct SerialUss
{
    char new_data;
    char port;
    float distance_cm;
} SerialUss;

int gyro_new_data = 0;
SerialGyro data_gyro;

SerialUss data_uss[USS_MAX_NB];

int motor_new_data = 0;
int encoders_pos[4];
float encoders_speed[4];

int fd_ = -1;
int serial_initialized_ = 0;

PI_THREAD (read_serial)
{
  for (;;){
    delay(3);
    receive_msg(fd_);
  }
}

int send_data(const int fd, const char *s, const int buffer_size){
    return write(fd, s, buffer_size);
}

int init_gyro(const int fd, char port){
    if(6 > port || port > 8){
        return -1;
    }
    char axis = GYRO_ALL_AXES;
    char ext_id_gyro = (((GYRO_PORT+axis)<<4)+GYRO_DEV_ID);
    char gyro_msg[HEADER_MSG_SIZE + GYRO_MSG_SIZE] 
    =   {0xff, 0x55, GYRO_MSG_SIZE, 
        ext_id_gyro, ACTION_GET, GYRO_DEV_ID, port, axis};

    return send_data(fd, gyro_msg, HEADER_MSG_SIZE + GYRO_MSG_SIZE);
}

int stop_gyro(const int fd, char port){
    if(6 > port || port > 8){
        return -1;
    }
    char axis = GYRO_ALL_AXES;
    char ext_id_gyro = (((GYRO_PORT+axis)<<4)+GYRO_DEV_ID);
    char gyro_msg[HEADER_MSG_SIZE + GYRO_MSG_SIZE] 
    =   {0xff, 0x55, GYRO_MSG_SIZE, 
        ext_id_gyro, ACTION_GET, GYRO_DEV_ID, port+0x10, axis};

    return send_data(fd, gyro_msg, HEADER_MSG_SIZE + GYRO_MSG_SIZE);
}

int received_version = 0;
int init_serial(const char * device, int baud){
    if (serial_initialized_){
        return fd_;
    } else {
        if ((fd_ = serialOpen (device, baud)) < 0){
            fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
            fd_ = -1;
            return fd_;
        }

        if (wiringPiSetup () == -1){
            fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
            fd_ = -1;
            return fd_;
        }

        piThreadCreate (read_serial) ;

        int count = 0;
        while(!received_version && count < 20){
            request_version();
            count++;
            delay(100);
        }

        serial_initialized_ = 1;

        return fd_;
    }
}

char init_motor_1 = 0xff;
char init_motor_2 = 0xff;
int init_two_motors_info(const int fd, char motor_1, char motor_2){
    init_motor_1 = motor_1 - 1;
    init_motor_2 = motor_2 - 1;

    char motor = ((motor_1 - 1) & 0x3) + (((motor_2 - 1) & 0x3) << 2);
    char ext_id_motor = ((motor<<4)+((TWO_ENCODERS_POS_SPEED)&0xf)) & 0xff;
    // printf("\nrequest 2motors : motor : %d ext_id: %d / motor_1 %d  motor_2 %d\n", motor, ext_id_motor, motor_1, motor_2);
    char motor_msg[HEADER_MSG_SIZE + READ_MOTOR_MSG_SIZE] 
    =   {0xff, 0x55, READ_MOTOR_MSG_SIZE, 
        ext_id_motor, ACTION_GET, TWO_ENCODERS_POS_SPEED, 0x46, motor_1, motor_2};

    return send_data(fd, motor_msg, HEADER_MSG_SIZE + READ_MOTOR_MSG_SIZE);
}

int init_uss(const int fd, char port){
    if(USS_FIRST_SLOT > port || port > USS_LAST_SLOT){
        return -1;
    }

    data_uss[port-1-4].new_data = 0x0;

    char ext_id_uss = ((port<<4)+USS_DEV_ID);
    char uss_msg[HEADER_MSG_SIZE + USS_MSG_SIZE]
    =   {0xff, 0x55, USS_MSG_SIZE,
        ext_id_uss, ACTION_GET, USS_DEV_ID, port};

    return send_data(fd, uss_msg, HEADER_MSG_SIZE + USS_MSG_SIZE);
}

int check_valid_value(float value_to_check, float epsilon, float max){
    return (fabs(value_to_check) > epsilon && fabs(value_to_check) < max);
}

int decode_data(){
    int ret = -1;
    if(0xff == makeblock_response_msg[0] && 0x55 == makeblock_response_msg[1] 
        && VERSION_DEV_ID == makeblock_response_msg[2] && DATA_TYPE_STRING == makeblock_response_msg[3]
    ){
        received_version = 0x1;
        printf("\nArduino Version :%s", &makeblock_response_msg[4]);
    } else if(0xff == makeblock_response_msg[0] && 0x55 == makeblock_response_msg[1] 
        && 0xd == makeblock_response_msg[8] && 0xa == makeblock_response_msg[9]
    ){
        char ext_id = makeblock_response_msg[2];
        char data_type = makeblock_response_msg[3];
        char dev_id = ext_id & 0xf;

        // printf("dev_id : %d / data_type : %d", dev_id, data_type);

        switch (dev_id) {
            case MOTOR_DEV_ID & 0xf:

                piLock (MOTOR_MUTEX) ;
                piLock (OTHER_MUTEX) ;
                // ext_id_motor = ((motor<<4)+MOTOR_DEV_ID)&0xff;
                int motor = ((ext_id - MOTOR_DEV_ID) >> 4);
                if(DATA_TYPE_FLOAT == data_type){
                    // Float value -> motor speed
                    encoders_speed[motor-1] = *(float*)(makeblock_response_msg+4);
                    // printf("\nmotor speed decoder / %d : %f", motor, encoders_speed[motor-1]);
                    ret = 0;
                } else if(DATA_TYPE_LONG == data_type){
                    // Long value -> Motor encodeur
                    encoders_pos[motor-1] = *(int*)(makeblock_response_msg+4);
                    // printf("\nmotor pos decoder/ %d : %d",  motor, encoders_pos[motor-1]);
                    ret = 0;
                }
                piUnlock (OTHER_MUTEX);
                piUnlock (MOTOR_MUTEX);
                break;
            
            case USS_DEV_ID:
                piLock (USS_MUTEX) ;
                piLock (OTHER_MUTEX) ;
                int port = ((ext_id & 0xf0) >> 4);
                if(DATA_TYPE_FLOAT == data_type){
                    data_uss[port-1-4].distance_cm = *(float*)(makeblock_response_msg+4);
                    data_uss[port-1-4].new_data = 0x1;
                    ret = 0;
                }
                piUnlock (OTHER_MUTEX) ;
                piUnlock (USS_MUTEX) ;
                break;
            case GYRO_DEV_ID:
                piLock (IMU_MUTEX) ;
                piLock (OTHER_MUTEX) ;
                char axis = ((ext_id & 0xf0) >> 4) - GYRO_PORT;
                if(DATA_TYPE_FLOAT == data_type){
                    if(GYRO_AXE_X == axis){
                        data_gyro.roll_ = *(float*)(makeblock_response_msg+4);
                        ret = 0;
                    } else if(GYRO_AXE_Y == axis){
                        data_gyro.pitch_ = *(float*)(makeblock_response_msg+4);
                        ret = 0;
                    } else if(GYRO_AXE_Z == axis){
                        data_gyro.yaw_ = *(float*)(makeblock_response_msg+4);
                        ret = 0;
                    }
                    
                }
                gyro_new_data = 1;
                piUnlock (OTHER_MUTEX) ;
                piUnlock (IMU_MUTEX) ;
                break;

            default:
                break;
        }
    } else if (
        0xff == makeblock_response_msg[0] && 0x55 == makeblock_response_msg[1] 
        && 0xd == makeblock_response_msg[18] && 0xa == makeblock_response_msg[19]
    ){

        char ext_id = makeblock_response_msg[2];
        char data_type[3] = {makeblock_response_msg[3], makeblock_response_msg[8], makeblock_response_msg[13]};
        char dev_id = ext_id & 0xf;

        switch (dev_id) {
            case GYRO_DEV_ID:
                piLock (IMU_MUTEX) ;
                piLock (OTHER_MUTEX) ;
                for(int axis = 1; axis < 4; axis++){
                    if(DATA_TYPE_FLOAT == data_type[axis - 1]){
                        if(GYRO_AXE_X == axis){
                            data_gyro.roll_ = *(float*)(makeblock_response_msg+4 + 5*(axis - 1));
                        }
                        if(GYRO_AXE_Y == axis){
                            data_gyro.pitch_ = *(float*)(makeblock_response_msg+4 + 5*(axis - 1));
                        }
                        if(GYRO_AXE_Z == axis){
                            data_gyro.yaw_ = *(float*)(makeblock_response_msg+4 + 5*(axis - 1));
                        }
                    }
                }
                ret = 0;
                gyro_new_data = 1;
                piUnlock (OTHER_MUTEX) ;
                piUnlock (IMU_MUTEX) ;
                break;

            default:
                break;
        }
    } else if (
        0xff == makeblock_response_msg[0] && 0x55 == makeblock_response_msg[1] 
        && 0xd == makeblock_response_msg[23] && 0xa == makeblock_response_msg[24]
    ){
        // printf("\nBig makeblock new message check!!");
        char ext_id = makeblock_response_msg[2];
        char data_type[4] = {makeblock_response_msg[3], makeblock_response_msg[8], makeblock_response_msg[13], makeblock_response_msg[18]};

        piLock (MOTOR_MUTEX) ;
        piLock (OTHER_MUTEX) ;
        int motor = ((ext_id & 0xf0) >> 4);
        char motor_1 = motor & 0x3;
        char motor_2 = (motor & 0xc) >> 2;

        if(0xff != init_motor_1){
            motor_1 = init_motor_1;
        }
        if(0xff != init_motor_2){
            motor_2 = init_motor_2;
        }

        // printf("\nTWO_ENCODERS_POS_SPEED check!! ext_id %d / motor_1 %d / motor_2 %d", ext_id, motor_1, motor_2);
        
        for(int value = 0; value < 4; value++){
            if(0 == value || 1 == value){
                motor = motor_1;
            }

            if(2 == value || 3 == value){
                motor = motor_2;    
            }

            if(DATA_TYPE_FLOAT == data_type[value]){
                // Float value -> motor speed
                encoders_speed[motor] = *(float*)(makeblock_response_msg+4 + 5*value);
                // printf("\nmotor speed decoder / %d : %f", motor+1, encoders_speed[motor]);
                ret = 0;
            }

            if(DATA_TYPE_LONG == data_type[value]){
                // Long value -> Motor encodeur
                encoders_pos[motor] = *(int*)(makeblock_response_msg+4 + 5*value);
                // printf("\nmotor pos decoder/ %d : %d",  motor+1, encoders_pos[motor]);
                ret = 0;
            }
        }
        motor_new_data = 1;
        piUnlock (OTHER_MUTEX) ;
        piUnlock (MOTOR_MUTEX) ;

    } else if (
        0xff == makeblock_response_msg[0] && 0x55 == makeblock_response_msg[1]
        && 0xd == makeblock_response_msg[48] && 0xa == makeblock_response_msg[49]
    ){
        ret = 0;
        piLock (IMU_MUTEX) ;
        piLock (OTHER_MUTEX) ;
        // data_gyro.roll_ = *(float*)(makeblock_response_msg+4 + 5*0);
        // data_gyro.pitch_ = *(float*)(makeblock_response_msg+4 + 5*1);

        // this check that the value is not error 0 and the value is not above 2Pi
        // it should be between pi and -pi
        // I don't know why but it happens only for the yaw..
        // if(fabs( *(float*)(makeblock_response_msg+4 + 5*2)) > 0.000001 && fabs( *(float*)(makeblock_response_msg+4 + 5*2) < 6.28)){
        //     data_gyro.yaw_ = *(float*)(makeblock_response_msg+4 + 5*2);
        // }

        float epsilon = 0.000001;
        float max_angle = 6.28;
        float max_speed = 6.28;
        float max_acceleration = 12;
        if(check_valid_value(*(float*)(makeblock_response_msg+4 + 5*0), epsilon, max_angle)){
            data_gyro.roll_ = *(float*)(makeblock_response_msg+4 + 5*0);
        }
        if(check_valid_value(*(float*)(makeblock_response_msg+4 + 5*1), epsilon, max_angle)){
            data_gyro.pitch_ = *(float*)(makeblock_response_msg+4 + 5*1);
        }
        if(check_valid_value(*(float*)(makeblock_response_msg+4 + 5*2), epsilon, max_angle)){
            data_gyro.yaw_ = *(float*)(makeblock_response_msg+4 + 5*2);
        }


        if(check_valid_value(*(float*)(makeblock_response_msg+4 + 5*3), epsilon, max_speed)){
            data_gyro.ang_x_ = *(float*)(makeblock_response_msg+4 + 5*3);
        }
        if(check_valid_value(*(float*)(makeblock_response_msg+4 + 5*4), epsilon, max_speed)){
            data_gyro.ang_y_ = *(float*)(makeblock_response_msg+4 + 5*4);
        }
        if(check_valid_value(*(float*)(makeblock_response_msg+4 + 5*5), epsilon, max_speed)){
            data_gyro.ang_z_ = *(float*)(makeblock_response_msg+4 + 5*5);

        }

        if(check_valid_value(*(float*)(makeblock_response_msg+4 + 5*6), epsilon, max_acceleration)){
            data_gyro.lin_x_ = *(float*)(makeblock_response_msg+4 + 5*6);
        }
        if(check_valid_value(*(float*)(makeblock_response_msg+4 + 5*7), epsilon, max_acceleration)){
            data_gyro.lin_y_ = *(float*)(makeblock_response_msg+4 + 5*7);
        }
        if(check_valid_value(*(float*)(makeblock_response_msg+4 + 5*8), epsilon, max_acceleration)){
            data_gyro.lin_z_ = *(float*)(makeblock_response_msg+4 + 5*8);
        }

        gyro_new_data = 1;
        piUnlock (OTHER_MUTEX) ;
        piUnlock (IMU_MUTEX) ;
        // printf(" after : %f \n",  data_gyro.yaw_);

    }

    return ret;
}

int is_gyro_new_data(){
    piLock (IMU_MUTEX);
    int new_data = gyro_new_data;
    piUnlock (IMU_MUTEX) ;
    return new_data;
}

int is_motor_new_data(){
    piLock (MOTOR_MUTEX);
    int new_data = motor_new_data;
    piUnlock (MOTOR_MUTEX);
    return new_data;
}

int is_ultrasonic_new_data(char port){
    piLock (USS_MUTEX);
    char new_data = data_uss[port-1-4].new_data;
    piUnlock (USS_MUTEX) ;
    return new_data;
}

int get_orientation(float * ypr){
    int ret = 0x0;
    piLock (IMU_MUTEX) ;
    ret = 0x1;
    ypr[0] = data_gyro.yaw_;
    ypr[1] = data_gyro.pitch_;
    ypr[2] = data_gyro.roll_;
    gyro_new_data = 0x0;
    piUnlock (IMU_MUTEX) ;
    return ret;
}

int get_all_imu_infos(float * ypr, float * ang_vel, float * lin_acc){
    int ret = 0x0;
    piLock (IMU_MUTEX) ;
    ret = 0x1;

    ypr[0] = data_gyro.yaw_;
    ypr[1] = data_gyro.pitch_;
    ypr[2] = data_gyro.roll_;

    ang_vel[0] = data_gyro.ang_x_;
    ang_vel[1] = data_gyro.ang_y_;
    ang_vel[2] = data_gyro.ang_z_;

    lin_acc[0] = data_gyro.lin_x_;
    lin_acc[1] = data_gyro.lin_y_;
    lin_acc[2] = data_gyro.lin_z_;
    gyro_new_data = 0x0;
    piUnlock (IMU_MUTEX) ;
    return ret;
}

float get_gyro_roll(){
    piLock (IMU_MUTEX) ;
    float x = data_gyro.roll_;
    gyro_new_data = 0x0;
    piUnlock (IMU_MUTEX) ;
    return x;
}

float get_gyro_pitch(){
    piLock (IMU_MUTEX) ;
    float y = data_gyro.pitch_;
    gyro_new_data = 0x0;
    piUnlock (IMU_MUTEX) ;
    return y;
}

float get_gyro_yaw(){
    piLock (IMU_MUTEX) ;
    float z = data_gyro.yaw_;
    gyro_new_data = 0x0;
    piUnlock (IMU_MUTEX) ;
    return z;
}

float get_gyro_angular_x(){
    float x = 12.1;
    piLock (IMU_MUTEX) ;
    x = data_gyro.ang_x_;
    piUnlock (IMU_MUTEX) ;
    return x;
}

float get_gyro_angular_y(){
    float y = -15.6;
    piLock (IMU_MUTEX) ;
    y = data_gyro.ang_y_;
    piUnlock (IMU_MUTEX) ;
    return y;
}

float get_gyro_angular_z(){
    float z = 25.3;
    piLock (IMU_MUTEX) ;
    z = data_gyro.ang_z_;
    piUnlock (IMU_MUTEX) ;
    return z;
}

float get_gyro_linear_x(){
    piLock (IMU_MUTEX) ;
    float x = data_gyro.lin_x_;
    gyro_new_data = 0;
    piUnlock (IMU_MUTEX) ;
    return x;
}

float get_gyro_linear_y(){
    piLock (IMU_MUTEX) ;
    float y = data_gyro.lin_y_;
    gyro_new_data = 0;
    piUnlock (IMU_MUTEX) ;
    return y;
}

float get_gyro_linear_z(){
    piLock (IMU_MUTEX) ;
    float z = data_gyro.lin_z_;
    gyro_new_data = 0;
    piUnlock (IMU_MUTEX) ;
    return z;
}

int get_motor_position(int port){

    piLock (MOTOR_MUTEX);
    int encoder_pos = encoders_pos[port-1];
    motor_new_data = 0;
    piUnlock (MOTOR_MUTEX);

    return encoder_pos;

    // return encoders_pos[port-1];
}

float get_motor_speed(int port){

    piLock (MOTOR_MUTEX);
    float encoder_speed = encoders_speed[port-1];
    motor_new_data = 0;
    piUnlock (MOTOR_MUTEX);

    return encoder_speed;

    // return encoders_speed[port-1];
}

int get_uss_cm_safe(float * value_cm, int port){
    int ret = 0x0;
    piLock (USS_MUTEX);
    ret = 0x1;
    (*value_cm) = data_uss[port-1-4].distance_cm;
    data_uss[port-1-4].new_data = 0x0;
    piUnlock (USS_MUTEX);
    return ret;
}

float get_uss_cm(int port){
    piLock (USS_MUTEX);
    float distance_cm = data_uss[port-1-4].distance_cm;
    data_uss[port-1-4].new_data = 0x0;
    piUnlock (USS_MUTEX);
    return distance_cm;
}

int request_gyro_all_axes(const int fd){
    return request_gyro(fd, GYRO_ALL_AXES);
}

int request_gyro(const int fd, char axis){
    char ext_id_gyro = (((GYRO_PORT+axis)<<4)+GYRO_DEV_ID);
    char gyro_msg[HEADER_MSG_SIZE + GYRO_MSG_SIZE] 
    =   {0xff, 0x55, GYRO_MSG_SIZE, 
        ext_id_gyro, ACTION_GET, GYRO_DEV_ID, GYRO_PORT, axis};

    return send_data(fd, gyro_msg, HEADER_MSG_SIZE + GYRO_MSG_SIZE);
}

int request_motor_info(const int fd, char motor, char pos_speed){

    char ext_id_motor = ((motor<<4)+MOTOR_DEV_ID)&0xff;
    char motor_msg[HEADER_MSG_SIZE + READ_MOTOR_MSG_SIZE] 
    =   {0xff, 0x55, READ_MOTOR_MSG_SIZE, 
        ext_id_motor, ACTION_GET, MOTOR_DEV_ID, 0x0, motor, pos_speed};

    return send_data(fd, motor_msg, HEADER_MSG_SIZE + READ_MOTOR_MSG_SIZE);
}

int request_two_motors_info(const int fd, char motor_1, char motor_2){

    char motor = ((motor_1 - 1) & 0x3) + (((motor_2 - 1) & 0x3) << 2);
    char ext_id_motor = ((motor<<4)+((TWO_ENCODERS_POS_SPEED)&0xf)) & 0xff;
    // printf("\nrequest 2motors : motor : %d ext_id: %d / motor_1 %d  motor_2 %d\n", motor, ext_id_motor, motor_1, motor_2);
    char motor_msg[HEADER_MSG_SIZE + READ_MOTOR_MSG_SIZE] 
    =   {0xff, 0x55, READ_MOTOR_MSG_SIZE, 
        ext_id_motor, ACTION_GET, TWO_ENCODERS_POS_SPEED, 0x0, motor_1, motor_2};

    return send_data(fd, motor_msg, HEADER_MSG_SIZE + READ_MOTOR_MSG_SIZE);
}

int request_motor_position(const int fd, char motor){
    return request_motor_info(fd, motor, READ_MOTOR_POS);
}

int request_motor_speed(const int fd, char motor){
    return request_motor_info(fd, motor, READ_MOTOR_SPEED);
}

int request_two_motors_pos_speed(const int fd, char motor_1, char motor_2){
    return request_two_motors_info(fd, motor_1, motor_2);
}

int request_uss(const int fd, char port){

    char ext_id_uss = ((port<<4)+USS_DEV_ID);
    char uss_msg[HEADER_MSG_SIZE + USS_MSG_SIZE]
    =   {0xff, 0x55, USS_MSG_SIZE,
        ext_id_uss, ACTION_GET, USS_DEV_ID, port};

    return send_data(fd, uss_msg, HEADER_MSG_SIZE + USS_MSG_SIZE);
}

int request_version(){

    // char ext_id_version = VERSION_DEV_ID;
    char version_msg[HEADER_MSG_SIZE + VERSION_MSG_SIZE]
    =   {0xff, 0x55, VERSION_MSG_SIZE,
        0x0, ACTION_GET, VERSION_DEV_ID};

    return send_data(fd_, version_msg, HEADER_MSG_SIZE + VERSION_MSG_SIZE);
}

void receive_msg(int fd){
    int ii = 0;
    int print_version = 0x0;

    piLock (OTHER_MUTEX);
    while (serialDataAvail (fd)){
        makeblock_response_msg[ii] = serialGetchar (fd);
        // printf (" -> %3d", makeblock_response_msg[ii]) ;
        fflush (stdout) ;

        // in the case were we are receiving the serial via USB
        // the Arduino will reset and send the string Version over serial
        // so we need to wait for it before behing able to start
        // therefore we need to read also on other serials
        if(!received_version && 0x56 == makeblock_response_msg[0]){
            received_version = 0x1;
            if (0x56 == makeblock_response_msg[ii]){
                print_version = 0x1;
            }
        }

        // In the case of USS sensor the header is send before the USS data is there
        // therefore we need to wait for the data
        if (2 == ii && (USS_DEV_ID == (makeblock_response_msg[ii]&0xf))){
            delay(13);
        }

        ii++;
    }
    piUnlock (OTHER_MUTEX);

    if(ii >= MAKEBLOCK_MSG_SIZE ){
        if(0x0 == print_version){
            // printf ("\ndata_received of size: %d", ii) ;
            decode_data();
        } else if(0x1 == print_version){
            printf("Arduino %s", makeblock_response_msg);
        }
    }

}

int reset_motors(const int fd){
    char reset_msg[HEADER_MSG_SIZE + RESET_MOTOR_MSG_SIZE]
    = {0xff, 0x55, RESET_MOTOR_MSG_SIZE, 0x0, ACTION_RESET};
    return send_data(fd, reset_msg, HEADER_MSG_SIZE + RESET_MOTOR_MSG_SIZE);
}

int set_speed(const int fd, char motor, int speed){
    if(motor < 1 && motor > 4){
        return -1;
    }

    valShort.shortVal = speed;

    char motor_msg[HEADER_MSG_SIZE + SET_MOTOR_MSG_SIZE] 
    =   {0xff, 0x55, SET_MOTOR_MSG_SIZE, 
        MOTOR_EXT_ID, ACTION_RUN, ENCODER_PID_MOTION, ENCODER_BOARD_SPEED_MOTION, motor, valShort.byteVal[0], valShort.byteVal[1]};

    return send_data(fd, motor_msg, HEADER_MSG_SIZE + SET_MOTOR_MSG_SIZE);
}