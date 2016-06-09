/*
  DynamixelController.h - Dynamixel Controller Library

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Based on Michael E. Ferguson BioloidController Library and KurtE https://github.com/KurtE
*/

#include "ax12Controller.h"

class DynamixelController
{
  public:
    uint8_t maxServos;
    int** angleLimits_;
    /* For compatibility with legacy code */
    // Changed to two step init...
    DynamixelController();               // baud usually 1000000
    void begin(long baud=1000000, Stream* pstream = (Stream*)PAX12Serial, int direction_pin = -1);
    void begin(long baud, Stream* pstream, int direction_pin, int max);

    int getServoPosition(int id);				            // get the position of a particular servo
    int getServoVoltage(int id);                    // get the voltage of a particular servo
    int getServoTemp(int id);                       // get the temperature of a particular servo
    int getServoSpeed(int id);                      // get the set speed of a particular servo
    int getServoTorque(int id);                     // get the Max Torque of a particular servo
    int * getAngleLimits(int id);

    void setServoSpeed(int id, int spd);		        // set the speed of a particular servo
    void setMaxTorque(int id, int trq);             // set Max Torque of a particular servo
    int setCWAngleLimit(int id, int ang);           // set Max Angle for Clockwise Movement
    int setCCWAngleLimit(int id, int ang);          // set Max Angle for Counter Clockwise Movement
    int setAngleLimits(int id, int ang1, int ang2);  // set Max Angle for both Clockwise and Counter Clockwise Movement

    int setServoPosition(int id, int pos);
    int setServoPosition(int id, int pos, int spd);
    int setDualServoPosition(int id_1, int id_2, int pos);
    int setDualServoPosition(int id_1, int id_2, int pos, int spd);

    int goHome(int id);                             // have a particular servo go to it's home position

    int waitForMoveToComplete(int id);

  private:  
    int angleInfo = 2;
    unsigned int * pose_;                       // the current pose, updated by Step(), set out by Sync()
    unsigned int * nextpose_;                   // the destination pose, where we put on load
    int * speed_;                               // speeds for interpolation 
    unsigned char * id_;                        // servo id for this index

//    unsigned long lastframe_;                   // time last frame was sent out  
    unsigned long nextframe_;                   //    
    // transition_t * sequence;                    // sequence we are running
    int transitions;                            // how many transitions we have left to load
   
};