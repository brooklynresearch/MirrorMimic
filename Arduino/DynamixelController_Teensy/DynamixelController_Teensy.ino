#include <ax12Serial.h>
#include <BioloidSerial.h>

//#define HALF_ARM
#define FULL_ARM
//#define NECK

#ifdef HALF_ARM
#define SHOULDER_L  1
#define SHOULDER_R  2
#define NUM_SERVOS   2
#endif

#ifdef FULL_ARM
#define SHOULDER_L  1
#define SHOULDER_R  2
#define ELBOW_H     3
#define ELBOW_L     4
#define NUM_SERVOS   4
#endif

#ifdef NECK
#define NECK_L      1
#define NECK_H      2
#define NUM_SERVOS   2
#endif

int servoID[NUM_SERVOS] = {
#ifdef HALF_ARM
  SHOULDER_L, SHOULDER_R
#endif
#ifdef FULL_ARM
  SHOULDER_L, SHOULDER_R, ELBOW_H, ELBOW_L
#endif
#ifdef NECK
  NECK_L, NECK_H
#endif
};
// Global objects
/* IK Engine */
BioloidControllerEx bioloid = BioloidControllerEx();  // may use or not... may go direct to AX12

// other globals.
word           g_wVoltage;
char           g_aszCmdLine[80];
uint8_t        g_iszCmdLine;
boolean        g_fTrackServos = false;

// Values to use for servo position...
byte          g_bServoID;
word          g_wServoGoalPos;
word          g_wServoGoalSpeed;

//====================================================================================================
// Setup 
//====================================================================================================
void setup() {
  Serial.begin(9600);  // start off the serial port.  
  bioloid.poseSize = NUM_SERVOS;
  bioloid.begin(1000000, &Serial1, -1);

  delay(1000);
  Serial.print("System Voltage in 10ths: ");
  Serial.println(ax12GetRegister(servoID[0], AX_PRESENT_VOLTAGE, 1));
//  Serial.println(g_wVoltage = ax12GetRegister(PTEST_ID, AX_PRESENT_VOLTAGE, 1), DEC);
  
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  for(int i=0; i<NUM_SERVOS; i++){
    SetServoSpeed(servoID[i],500);
  }
}

void loop() {
  int newRandomShoulderPosition = (int)random(1000,2000);
  int newRandomElbowPosition = (int)random(2048);
//  int newRandomSpeed = (int)random(1000);
  //Serial.print("Move To: ");Serial.print(newRandomPosition);Serial.print("\tWith Speed: ");Serial.println(newRandomSpeed);
  Serial.print("ELBOW_L Success: ");Serial.println(SetServoPosition(ELBOW_L, newRandomElbowPosition));
  Serial.print("ELBOW_H Success: ");Serial.println(SetServoPosition(ELBOW_H, newRandomElbowPosition));
  Serial.print("SHOULDER Success: ");Serial.println(SetDualServoPosition(SHOULDER_L, SHOULDER_R, newRandomShoulderPosition)); 
  for(int i=0; i<NUM_SERVOS; i++){
      Serial.print("SERVO ");Serial.print(i+1);Serial.print(": ");Serial.println(GetServoPosition(servoID[i]));
  }
  Serial.println();
  delay(5000);
}

int GetServoPosition(int id){
  return ax12GetRegister(id, AX_PRESENT_POSITION_L, 2);
}
void SetServoSpeed(word id, word spd){
  ax12SetRegister2(id, AX_GOAL_SPEED_L, spd);
}

int SetServoPosition(word id, word pos){
  ax12SetRegister2(id, AX_GOAL_POSITION_L, pos);
  return ax12ReadPacket(6);
}

int SetServoPosition(word id, word pos, word spd){
   ax12SetRegister2(id, AX_GOAL_SPEED_L, spd);
   ax12SetRegister2(id, AX_GOAL_POSITION_L, pos);
   return ax12ReadPacket(6);
}

int SetDualServoPosition(word id_1, word id_2, word pos){
  ax12SetRegister2(id_1, AX_GOAL_POSITION_L, pos);
  ax12SetRegister2(id_2, AX_GOAL_POSITION_L, pos);
  return ax12ReadPacket(6);
}

int SetDualServoPosition(word id_1, word id_2, word pos, word spd){
  ax12SetRegister2(id_1, AX_GOAL_SPEED_L, spd);
  ax12SetRegister2(id_2, AX_GOAL_SPEED_L, spd);
  ax12SetRegister2(id_1, AX_GOAL_POSITION_L, pos);
  ax12SetRegister2(id_2, AX_GOAL_POSITION_L, pos);
  return ax12ReadPacket(6);
}
 

