#include "DynamixelController.h"
#include <avr/pgmspace.h>

/* initializes serial1 transmit at baud, 8-N-1 */
DynamixelController::DynamixelController(){
  
}

void DynamixelController::begin(long baud, Stream* pstream, int direction_pin){
    // int i;
    // // setup storage
    // id_ = (unsigned char *) malloc(AX12_MAX_SERVOS * sizeof(unsigned char));
    // pose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
    // nextpose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
    // speed_ = (int *) malloc(AX12_MAX_SERVOS * sizeof(int));
    // // initialize
    // for(i=0;i<AX12_MAX_SERVOS;i++){
    //     id_[i] = i+1;
    //     pose_[i] = 512;
    //     nextpose_[i] = 512;
    // }
    // // frameLength = BIOLOID_FRAME_LENGTH;
    // interpolating = 0;
    // playing = 0;
    // nextframe_ = millis();
    ax12Init(baud, pstream, direction_pin);  
}

int DynamixelController::getServoPosition(int id){
  return ax12GetRegister(id, AX_PRESENT_POSITION_L, 2);
}
int DynamixelController::getServoVoltage(int id){
  return ax12GetRegister(id, AX_PRESENT_VOLTAGE, 1);
}
int DynamixelController::getServoTemp(int id){
  return ax12GetRegister(id, AX_PRESENT_TEMPERATURE, 1);
}
int DynamixelController::getServoSpeed(int id){
  return ax12GetRegister(id, AX_GOAL_SPEED_L , 2);
}
void DynamixelController::setServoSpeed(int id, int spd){
  ax12SetRegister2(id, AX_GOAL_SPEED_L, spd);
}

int DynamixelController::setServoPosition(int id, int pos){
  ax12SetRegister2(id, AX_GOAL_POSITION_L, pos);
  return ax12ReadPacket(6);
}

int DynamixelController::setServoPosition(int id, int pos, int spd){
   ax12SetRegister2(id, AX_GOAL_SPEED_L, spd);
   ax12SetRegister2(id, AX_GOAL_POSITION_L, pos);
   return ax12ReadPacket(6);
}

int DynamixelController::setDualServoPosition(int id_1, int id_2, int pos){
  ax12SetRegister2(id_1, AX_GOAL_POSITION_L, pos);
  ax12SetRegister2(id_2, AX_GOAL_POSITION_L, pos);
  return ax12ReadPacket(6);
}

int DynamixelController::setDualServoPosition(int id_1, int id_2, int pos, int spd){
  ax12SetRegister2(id_1, AX_GOAL_SPEED_L, spd);
  ax12SetRegister2(id_2, AX_GOAL_SPEED_L, spd);
  ax12SetRegister2(id_1, AX_GOAL_POSITION_L, pos);
  ax12SetRegister2(id_2, AX_GOAL_POSITION_L, pos);
  return ax12ReadPacket(6);
}

void DynamixelController::waitForMoveToComplete(int id) {
  do {
    //    delay(1);
  } 
  while (ax12GetRegister(id, AX_MOVING, 1));
}



