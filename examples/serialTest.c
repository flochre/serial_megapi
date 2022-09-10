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
  int fd ;
  int count ;
  unsigned int nextTime ;

  if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  if (wiringPiSetup () == -1)
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    return 1 ;
  }

  nextTime = millis () + 300 ;

  for (count = 0 ; count < 256 ; )
  {
    if (millis () > nextTime)
    {
      printf ("\nOut: %3d: ", count) ;
      fflush (stdout) ;

      // write (fd, gyro_msg, 8) ;
      // write (fd, uss_msg, USS_MSG_SIZE) ;

      if(0 == count % 2){
        // printf ("\nOut Gyro: %f: ", get_gyro(fd, GYRO_AXE_X));  
        request_gyro_all_axes(fd);
        // request_gyro(fd, GYRO_AXE_Z);
      } else {
        // printf ("\nOut USS : %f: ", get_uss(fd, 0x7));
        request_uss(fd, 0x7);   
      }

      printf( "gyro_x : %f / gyro_z : %f / uss_cm : %f \n", get_gyro_x(), get_gyro_z(), get_uss(7));
      
      nextTime += 300 ;
      ++count ;
    }

    delay (3);

    receive_msg(fd);

  }

  printf ("\n") ;
  return 0 ;
}
