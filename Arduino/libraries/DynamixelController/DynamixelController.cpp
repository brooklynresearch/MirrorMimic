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
    // for(int i=0; i < AX12_MAX_SERVOS; i++){
    //   angleLimits_[i][0] = 0;
    //   angleLimits_[i][1] = 4095;
    // }
    ax12Init(baud, pstream, direction_pin);  
}


void DynamixelController::begin(long baud, Stream* pstream, int direction_pin, int max){
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
    maxServos = max;

    if (maxServos > AX12_MAX_SERVOS){
      maxServos = AX12_MAX_SERVOS;
    }
    angleLimits_ = new int*[maxServos];
    for(int i=0; i < maxServos; i++){
      angleLimits_[i] = new int[angleInfo];
    }

    for(int i=0; i<maxServos; i++){
      for(int j=0; j<angleInfo; j += 2){
        angleLimits_[i][j] = 0;
        angleLimits_[i][j+1] = 4095;
      }
    }

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

int * DynamixelController::getAngleLimits(int id){
  static int angleParams[2];
  for(int i=0; i<angleInfo; i++){
    angleParams[i] = angleLimits_[id-1][i];
    //angleParams[i] = i;
    //cout("%d\t", angleLimits_[id][i]);
  }
  return angleParams;
}

void DynamixelController::setServoSpeed(int id, int spd){
  ax12SetRegister2(id, AX_GOAL_SPEED_L, spd);
}

void DynamixelController::setMaxTorque(int id, int trq){
  ax12SetRegister2(id, AX_MAX_TORQUE_L, trq);
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

int DynamixelController::setCWAngleLimit(int id, int ang){
  ax12SetRegister2(id, AX_CW_ANGLE_LIMIT_L, ang);
  return ax12ReadPacket(6);
}

int DynamixelController::setCCWAngleLimit(int id, int ang){
  ax12SetRegister2(id, AX_CCW_ANGLE_LIMIT_L, ang);
  return ax12ReadPacket(6);
}

int DynamixelController::setAngleLimits(int id, int ang1, int ang2){
    ax12SetRegister2(id, AX_CW_ANGLE_LIMIT_L, ang1);
    ax12SetRegister2(id, AX_CCW_ANGLE_LIMIT_L, ang2);
    return ax12ReadPacket(6);
}

int DynamixelController::waitForMoveToComplete(int id) {
  do {
    //    delay(1);
  } 
  while (ax12GetRegister(id, AX_MOVING, 1));
  return ax12ReadPacket(6);
}



