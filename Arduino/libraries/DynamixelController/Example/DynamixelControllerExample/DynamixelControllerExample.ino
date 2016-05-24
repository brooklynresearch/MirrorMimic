#include <ax12Controller.h>
#include <DynamixelController.h>

#define NUM_SERVOS 4

int servoID[NUM_SERVOS] = {1, 2, 3, 4};
int angleLimit[NUM_SERVOS][2] = {{1023,2048},{1023,2048},{1023,2048},{1023,2048}};

int defaultSpeed = 200;

DynamixelController dynaControl = DynamixelController();

void setup() {
  Serial.begin(115200);  // start off the serial port.  
  dynaControl.begin(1000000, &Serial1, -1);

  delay(1000);

  for(int i=0; i<NUM_SERVOS; i++){
      dynaControl.setServoSpeed(servoID[i], defaultSpeed);
  	  Serial.print("SERVO ");Serial.print(servoID[i]);
  	  Serial.print("\t\tVOLTAGE: ");Serial.print(((float)dynaControl.getServoVoltage(servoID[i])/10));
  	  Serial.print("\tTEMP: ");Serial.print(dynaControl.getServoTemp(servoID[i]));
  	  Serial.print("\tPOS: ");Serial.print(dynaControl.getServoPosition(servoID[i]));
      Serial.print("\tSPEED: ");Serial.println(dynaControl.getServoSpeed(servoID[i]));
	}
}

void loop() {
  int randomDualPos = (int)random(angleLimit[0][0], angleLimit[0][1]);
  int randomSinglePos1 = (int)random(angleLimit[1][0], angleLimit[1][1]);
  int randomSinglePos2 = (int)random(angleLimit[2][0], angleLimit[2][1]);

  int newPos[3] = {randomDualPos, randomSinglePos1, randomSinglePos2};
  
  dynaControl.setDualServoPosition(servoID[0], servoID[1], newPos[0]);
  dynaControl.setServoPosition(servoID[2], newPos[1]);
  dynaControl.setServoPosition(servoID[3], newPos[2]);

  delay(2000);

}
