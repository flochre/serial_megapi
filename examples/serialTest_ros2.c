/*
 * serialTest.c:
 *	Very simple program to test the serial port. Expects
 *	the port to be looped back to itself
 *
 * Copyright (c) 2012-2013 Gordon Henderson. <projects@drogon.net>
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "serial_megapi.h"

int main ()
{
  int fd;
  int count ;
  unsigned int nextTime ;

  if ((fd = init_serial("/dev/ttyAMA0", 115200)) < 0){
  // if ((fd = init_serial("/dev/megapi", 115200)) < 0){
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  int uss_port = 0x7;
  int uss_port_1 = 0x8;
  // init_uss(fd, uss_port);
  // init_uss(fd, uss_port_1);
  // init_two_motors_info(fd, 2, 3);

  // init_gyro(fd, 0x6);

  

  nextTime = millis () + 300 ;

  for (count = 0 ; count < 256 ; )
  {
    if (millis () > nextTime)
    {
      printf ("\nOut: %3d / %d / ", count, fd) ;
      fflush (stdout) ;

      // write (fd, gyro_msg, 8) ;
      // write (fd, uss_msg, USS_MSG_SIZE) ;

      int my_test_speed = 20;

      if(35 == count){
        // set_speed(fd, 2, my_test_speed);
        // set_speed(fd, 3, -my_test_speed);
        init_gyro(fd, 0x6);
        
      }

      if(65 == count){
      //   set_speed(fd, 2, 0);
      //   set_speed(fd, 3, 0);

        stop_gyro(fd, 0x6);
        
      }

      if(105 == count){
      //   set_speed(fd, 2, 0);
      //   set_speed(fd, 3, 0);

        init_gyro(fd, 0x8);
        
      }

      if(120 == count){
        // set_speed(fd, 2, -my_test_speed);
        // set_speed(fd, 3, my_test_speed);
        stop_gyro(fd, 0x8);
      }

      // if(220 == count){
      //   set_speed(fd, 2, 0);
      //   set_speed(fd, 3, 0);
      // }

      // printf( "gyro_x : %f / gyro_z : %f / uss_cm : %f \n", get_gyro_x(), get_gyro_z(), get_uss(7));
      
      nextTime += 300 ;
      ++count ;
    }

    if(is_ultrasonic_new_data(uss_port)){
      printf("USS : %f \n", get_uss_cm(uss_port));
    }

    if(is_ultrasonic_new_data(uss_port_1)){
      printf("USS : %f \n", get_uss_cm(uss_port_1));
    }
      
    if(is_gyro_new_data()){
      printf("Gyro : %f \n", get_gyro_yaw());
    }

  }

  printf ("\n") ;
  return 0 ;
}
