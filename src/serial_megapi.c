#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "serial_megapi.h"

int decode_data(){ 
    int ret = -1;
    if(0xff == makeblock_response_msg[0] && 0x55 == makeblock_response_msg[1] 
        && 0xd == makeblock_response_msg[8] && 0xa == makeblock_response_msg[9])
    {
        char ext_id = makeblock_response_msg[2];
        char data_type = makeblock_response_msg[3];
        char dev_id = ext_id & 0xf;

        // printf("dev_id : %d / data_type : %d", dev_id, data_type);

        switch (dev_id) {
            case USS_DEV_ID:
                int port = ((ext_id & 0xf0) >> 4);
                if(DATA_TYPE_FLOAT == data_type){
                    data_uss[port-1].distance_cm = *(float*)(makeblock_response_msg+4);
                    ret = 0;
                }
                break;
            case GYRO_DEV_ID:
                char axis = ((ext_id & 0xf0) >> 4) - GYRO_PORT;
                if(DATA_TYPE_FLOAT == data_type){
                    if(GYRO_AXE_X == axis){
                        data_gyro.x_ = *(float*)(makeblock_response_msg+4);
                        ret = 0;
                    } else if(GYRO_AXE_Y == axis){
                        data_gyro.y_ = *(float*)(makeblock_response_msg+4);
                        ret = 0;
                    } else if(GYRO_AXE_Z == axis){
                        data_gyro.z_ = *(float*)(makeblock_response_msg+4);
                        ret = 0;
                    }
                    
                }

            default:
                break;
        }
    } else if (
        0xff == makeblock_response_msg[0] && 0x55 == makeblock_response_msg[1] 
        && 0xd == makeblock_response_msg[18] && 0xa == makeblock_response_msg[19])
    {

        char ext_id = makeblock_response_msg[2];
        char data_type[3] = {makeblock_response_msg[3], makeblock_response_msg[8], makeblock_response_msg[13]};
        char dev_id = ext_id & 0xf;

        switch (dev_id) {
            case GYRO_DEV_ID:
                for(int axis = 1; axis < 4; axis++){
                    if(DATA_TYPE_FLOAT == data_type[axis - 1]){
                        if(GYRO_AXE_X == axis){
                            data_gyro.x_ = *(float*)(makeblock_response_msg+4 + 5*(axis - 1));
                        }
                        if(GYRO_AXE_Y == axis){
                            data_gyro.y_ = *(float*)(makeblock_response_msg+4 + 5*(axis - 1));
                        }
                        if(GYRO_AXE_Z == axis){
                            data_gyro.z_ = *(float*)(makeblock_response_msg+4 + 5*(axis - 1));
                        }
                    }
                }
                ret = 0;

            default:
                break;
        }
    }

    return ret;
}

float get_gyro_x(){
    return data_gyro.x_;
}

float get_gyro_y(){
    return data_gyro.y_;
}

float get_gyro_z(){
    return data_gyro.z_;
}

float get_uss(int port){
    return data_uss[port-1].distance_cm;
}

int request_data(const int fd, const char *s, const int buffer_size){
    return write(fd, s, buffer_size);
}

int request_gyro_all_axes(const int fd){
    return request_gyro(fd, 0);
}

int request_gyro(const int fd, char axis){
    char ext_id_gyro = (((GYRO_PORT+axis)<<4)+GYRO_DEV_ID);
    char gyro_msg[HEADER_MSG_SIZE + GYRO_MSG_SIZE] 
    =   {0xff, 0x55, GYRO_MSG_SIZE, 
        ext_id_gyro, ACTION_GET, GYRO_DEV_ID, GYRO_PORT, axis};

    return request_data(fd, gyro_msg, HEADER_MSG_SIZE + GYRO_MSG_SIZE);
}

int request_uss(const int fd, char port){

    char ext_id_uss = ((port<<4)+USS_DEV_ID);
    char uss_msg[HEADER_MSG_SIZE + USS_MSG_SIZE] 
    =   {0xff, 0x55, USS_MSG_SIZE, 
        ext_id_uss, ACTION_GET, USS_DEV_ID, port};

    return request_data(fd, uss_msg, HEADER_MSG_SIZE + USS_MSG_SIZE);
}

void receive_msg(int fd){
    int ii = 0; 

    while (serialDataAvail (fd)){
        makeblock_response_msg[ii] = serialGetchar (fd);
        printf (" -> %3d", makeblock_response_msg[ii]) ;
        fflush (stdout) ;

        // In the case of USS sensor the header is send before the USS data is there
        // therefore we need to wait for the data
        if (2 == ii && (USS_DEV_ID == (makeblock_response_msg[ii]&0xf))){
            delay(13);
        }

        ii++;
    }

    if(ii >= MAKEBLOCK_MSG_SIZE ){
        // printf ("\ndata_received of size: %d", ii) ;
        decode_data();
    }

}