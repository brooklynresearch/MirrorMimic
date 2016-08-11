#include <ax12Controller.h>
#include <DynamixelController.h>
#include <Wire.h>


//====================================================================================================
// Motor Controller Defines 
//====================================================================================================
// Define HALF_ARM, FULL_ARM, or NECK to setup up configuration and amount of servos and steppers.

//#define HALF_ARM
#define FULL_ARM
//#define NECK

#ifdef HALF_ARM
#define SHOULDER_L  1
#define SHOULDER_R  2
#define SHOULDER_R_START 1100
#define NUM_SERVOS   2
String CONTROLLERNAME = "RIGHT ARM";
#define I2CADDRESS 8
#define UPWARDS 0
#define DOWNWARDS 1
#endif

#ifdef FULL_ARM
#define SHOULDER_L  1
#define SHOULDER_R  2
#define SHOULDER_R_START 990
#define ELBOW_R     3
#define ELBOW_R_START 0
#define ELBOW_P     4
#define ELBOW_P_START 3400
#define NUM_SERVOS   4
String CONTROLLERNAME = "LEFT ARM";
#define I2CADDRESS 7
#define UPWARDS 0
#define DOWNWARDS 1
#endif

#ifdef NECK
#define NECK_R      1
#define NECK_P      2
#define NUM_SERVOS   2
String CONTROLLERNAME = "NECK AND WAIST";
#define I2CADDRESS 6
#define UPWARDS 1
#define DOWNWARDS 0
#endif

#define MASTER_ADDRESS    5

int servoID[NUM_SERVOS] = {
#ifdef HALF_ARM
  SHOULDER_L, SHOULDER_R
#endif
#ifdef FULL_ARM
  SHOULDER_L, SHOULDER_R, ELBOW_R, ELBOW_P
#endif
#ifdef NECK
  NECK_R, NECK_P
#endif
};

//====================================================================================================
// Dynamixel Setup
//====================================================================================================
DynamixelController dynaControl = DynamixelController();  // Initialize the Dynamixel Controller
#define MAX_TORQUE 1023

//====================================================================================================
// Serial Callback Command Setup
//====================================================================================================
#define I2CCOMMANDDEBUG        // Debug Mode. Use this define statement for Serial Print out readings.
//#define SERIALCOMMAND Serial2     // Which UART will we be sending/receiving commands from

#define I2CCOMMANDBUFFER 256   // Buffer size of the command Serial port
#define MAXI2CCOMMANDS  10     // 
#define MAXDELIMETER 2
//#define SERIALBAUDRATE 115200

char inChar;                              // A character read from the serial stream 
char charBuffer[I2CCOMMANDBUFFER];   // Buffer of stored characters while waiting for terminator character
int  bufPos;                              // Current position in the buffer
char delim[MAXDELIMETER];                 // null-terminated list of character to be used as delimeters for tokenizing (default " ")
char terminator = '\r';                   // Character that signals end of command (default '\r')
char *token;                              // Returned token from the command buffer as returned by strtok_r
char *last;                               // State variable used by strtok_r during processing
typedef struct _callback {
  char i2cCommand[I2CCOMMANDBUFFER];
  void (*function)();
} I2CCommandCallback;            // Data structure to hold Command/Handler function key-value pairs
int numCommand;
I2CCommandCallback CommandList[MAXI2CCOMMANDS];   // Actual definition for command/handler array
void (*defaultHandler)();   


//====================================================================================================
// Stepper Motor Setup
//====================================================================================================
#define TOTALSTEPS 3150
#ifdef FULL_ARM
#define HOMEPOS 200
#endif
#ifdef HALF_ARM
#define HOMEPOS 2500
#endif
#ifdef NECK
#define HOMEPOS 1024
#endif

int dirpin = 3;
int steppin = 4;

int sensorPin0 = A0;    // select the input pin for the potentiometer
int sensorPin1 = A1;
int sensorValue = 0;  // variable to store the value coming from the sensor
int limitThreshold = 100;
int curPosition = 0;
int prevPosition = curPosition;
bool firstTime = true;

#ifdef HALF_ARM
int stepperPosition = 0;
#endif

#ifdef FULL_ARM
int stepperPosition = 0;
#endif

#ifdef NECK
int stepperPosition = 0;
#endif

int newPosition = HOMEPOS;
unsigned long stepperTime = 0;

#ifdef HALF_ARM
unsigned int stepperInterval = 300;
#endif

#ifdef FULL_ARM
unsigned int stepperInterval = 300;
#endif

#ifdef NECK
unsigned int stepperInterval = 300;
#endif

int stepDirection = UPWARDS;

int defaultSpeed  =   100;

void setup() {
  
  Serial.begin(9600);  // debug serial port
  
  //Set the direction Pin to -1 for half duplex control on the Teensy 
  dynaControl.begin(1000000, &Serial1, -1, NUM_SERVOS);
  
  
  strncpy(delim," ",MAXDELIMETER);  // strtok_r needs a null-terminated string
  numCommand=0;    // Number of callback handlers installed
  clearI2CBuffer();

//  addI2CCommand("HELLO", SayHello);
//  addI2CCommand("TEST",  serialMotorTest);
  
  #ifdef NECK
  addI2CCommand("H_ROT", rotateHead);
  addI2CCommand("H_PIV", pivotHead);
  addI2CCommand("HRPOS", getHeadRotatePos);
  addI2CCommand("HPPOS", getHeadPivotPos);
  addI2CCommand("H_RST", resetHead);
  addI2CCommand("W_ROT", rotateWaist);
  addI2CCommand("W_POS", getWaistPos);
  #else
  addI2CCommand("S_ROT", rotateShoulder);
  addI2CCommand("S_PIV", pivotShoulder);
  addI2CCommand("SRPOS", getShoulderRotatePos);
  addI2CCommand("SPPOS", getShoulderPivotPos);
  addI2CCommand("S_RST", resetShoulder);
  #endif

  #ifdef FULL_ARM
  addI2CCommand("E_ROT", rotateElbow);
  addI2CCommand("E_PIV", pivotElbow);
  addI2CCommand("ERPOS", getElbowRotatePos);
  addI2CCommand("EPPOS", getElbowPivotPos);
  addI2CCommand("E_RST", resetElbow);
  #endif
  
  addI2CCommand("A_POS", getAllMotorPositions);
  addDefaultHandler(unrecognized);

  Wire.begin(I2CADDRESS);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(respondEvent); // callback event

  pinMode(dirpin, OUTPUT);
  pinMode(steppin, OUTPUT);

  delay(1000);

  Serial.println(F("====================================================================="));
  Serial.print(F("TONI DOVE :: "));Serial.print(CONTROLLERNAME);Serial.println(" CONTROLLER");
  Serial.println(F("====================================================================="));

  //serialMotorTest();
  #ifdef NECK
  stepperPosition = HOMEPOS;
  curPosition = HOMEPOS;  
  Serial.print("Stepper Position: ");Serial.println(stepperPosition);
  #else
  findHome();
  #endif
  delay(1000);
  
  for(int i=0; i<NUM_SERVOS; i++){
      dynaControl.resetServoTorqueLimit(servoID[i], MAX_TORQUE);
      dynaControl.setServoSpeed(servoID[i], defaultSpeed);
      Serial.print("SERVO ");Serial.print(servoID[i]);
      Serial.print("\t\tVOLTAGE: ");Serial.print(((float)dynaControl.getServoVoltage(servoID[i])/10));
      Serial.print("\tTEMP: ");Serial.print(dynaControl.getServoTemp(servoID[i]));
      Serial.print("\tPOS: ");Serial.print(dynaControl.getServoPosition(servoID[i]));
      Serial.print("\tSPEED: ");Serial.print(dynaControl.getServoSpeed(servoID[i]));
      Serial.print("\t\tMAX ANGLES: ");
      int *p;
      p = dynaControl.getAngleLimits(servoID[i]);
        for(int j=0; j<2; j++){
          Serial.print(*(p + j));Serial.print("  ");
        }
      Serial.println();
  }
}

void loop()
{
  //readSerialCommand();
  moveStepperToPosition(newPosition);
}

void findHome(){
  #ifdef FULL_ARM
    dynaControl.setServoPosition(SHOULDER_R, SHOULDER_R_START);
    dynaControl.setServoPosition(ELBOW_R, ELBOW_R_START);
    dynaControl.setServoPosition(ELBOW_P, ELBOW_P_START);
  #endif
  #ifdef HALF_ARM
    dynaControl.setServoPosition(SHOULDER_R, SHOULDER_R_START);
  #endif
  
  if(checkSensor(sensorPin0) || checkSensor(sensorPin1)){
    
    digitalWrite(dirpin, HIGH);
    Serial.println("Looking For First Limit");
    
    while(checkSensor(sensorPin0)){
      digitalWrite(steppin, LOW);
      digitalWrite(steppin, HIGH);
      delay(1);
    }
    delay(1000);
    int stepCount = 0;
    digitalWrite(dirpin, LOW);

    Serial.println("Looking For Second Limit");
    
    while(checkSensor(sensorPin1)){
      digitalWrite(steppin, LOW);
      digitalWrite(steppin, HIGH);
      stepCount++;
      stepperPosition++;
      delay(1);
    }
    Serial.print("Total Step Count: ");Serial.println(stepCount);
    delay(1000);
    //digitalWrite(dirpin,LOW);
    
    while(moveStepperToPosition(HOMEPOS));

    curPosition = HOMEPOS;
    
    Serial.print("Stepper Position: ");Serial.println(stepperPosition);
  } else {
    Serial.println("You're stepper motor is out of bounds.");
    Serial.println("Please turn off power.");
    Serial.println("Check sensors and/or reset stepper motors to a proper position.");
  }
}

bool checkSensor(int sensorPin){
  bool sensorState = 0;
  
  if(analogRead(sensorPin) < limitThreshold){
    sensorState = 1;
  } else {
    sensorState = 0;
  }
  
  return sensorState;
}


#ifdef NECK
void rotateHead(){
  char *arg;  
  arg = nextArgument();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    int newPos = atoi(arg);
    #ifdef I2CCOMMANDDEBUG
    Serial.print("Head rotating to: "); 
    Serial.println(arg); 
    #endif
    dynaControl.setServoPosition(NECK_R, newPos);
  } 
  else {
    Serial.println("No position supplied.  Please enter a number between 0 - 2048"); 
  }
}

void pivotHead(){
  char *arg;  
  arg = nextArgument();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    int newPos = atoi(arg);
    #ifdef I2CCOMMANDDEBUG
    Serial.print("Head pivoting to: "); 
    Serial.println(arg); 
    #endif
    dynaControl.setServoPosition(NECK_P, newPos);
  } 
  else {
    Serial.println("No position supplied.  Please enter a number between 0 - 2048"); 
  }
}

void getHeadRotatePos(){
  curPosition = dynaControl.getServoPosition(NECK_R);
  #ifdef I2CCOMMANDDEBUG
    Serial.print("SHOULDER PIVOT POS: ");
    Serial.println(curPosition);
  #endif
}

void getHeadPivotPos(){
  curPosition = dynaControl.getServoPosition(NECK_P);
  #ifdef I2CCOMMANDDEBUG
    Serial.print("SHOULDER PIVOT POS: ");
    Serial.println(curPosition);
  #endif
}

void rotateWaist(){
  char *arg;  
  arg = nextArgument();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    newPosition = atoi(arg);
    #ifdef I2CCOMMANDDEBUG
    Serial.print("Waist rotating to: "); 
    Serial.println(arg); 
    #endif
  } 
  else {
    Serial.println("No position supplied.  Please enter a number between 0 - 3150"); 
  }
}

void getWaistPos(){
  curPosition = stepperPosition;
  #ifdef I2CCOMMANDDEBUG
    Serial.print("SHOULDER ROTATE POS: ");
    Serial.println(curPosition);
  #endif
}

void resetHead(){
  for(int i=0; i<NUM_SERVOS; i++){
      dynaControl.resetServoTorqueLimit(servoID[i], MAX_TORQUE);
  }
}

#else

void rotateShoulder(){
  char *arg;  
  arg = nextArgument();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    newPosition = atoi(arg);
    #ifdef I2CCOMMANDDEBUG
    Serial.print("Shoulder rotating to: "); 
    Serial.println(arg); 
    #endif
  } 
  else {
    Serial.println("No position supplied.  Please enter a number between 0 - 3150"); 
  }
}

void pivotShoulder(){
  char *arg;  
  arg = nextArgument();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    int newPos = atoi(arg);
    #ifdef I2CCOMMANDDEBUG
    Serial.print("Shoulder pivoting to: "); 
    Serial.println(arg); 
    #endif
    dynaControl.setDualServoPosition(SHOULDER_L, SHOULDER_R, newPos);
  } 
  else {
    Serial.println("No position supplied.  Please enter a number between 0 - 2048"); 
  }
}

void getShoulderRotatePos(){
  curPosition = stepperPosition;
  #ifdef I2CCOMMANDDEBUG
    Serial.print("SHOULDER ROTATE POS: ");
    Serial.println(curPosition);
  #endif
}

void getShoulderPivotPos(){
  curPosition = dynaControl.getServoPosition(SHOULDER_L);
  #ifdef I2CCOMMANDDEBUG
    Serial.print("SHOULDER PIVOT POS: ");
    Serial.println(curPosition);
  #endif
}

void resetShoulder(){
//  for(int i=0; i<2; i++){
//      dynaControl.resetServoTorqueLimit(servoID[i], MAX_TORQUE);
//  }
  dynaControl.resetServoTorqueLimit(SHOULDER_L, MAX_TORQUE);
  dynaControl.resetServoTorqueLimit(SHOULDER_R, MAX_TORQUE);
}
#endif

#ifdef FULL_ARM
void rotateElbow(){
  char *arg;  
  arg = nextArgument();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    int newPos = atoi(arg);
    #ifdef I2CCOMMANDDEBUG
    Serial.print("Elbow rotating to: "); 
    Serial.println(arg); 
    #endif
    dynaControl.setServoPosition(ELBOW_R, newPos);
  } 
  else {
    Serial.println("No position supplied.  Please enter a number between 0 - 2048"); 
  }
}

void pivotElbow(){
  char *arg;  
  arg = nextArgument();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    int newPos = atoi(arg);
    #ifdef I2CCOMMANDDEBUG
    Serial.print("Elbow Pivoting to: "); 
    Serial.println(arg); 
    #endif
    dynaControl.setServoPosition(ELBOW_P, newPos);
  } 
  else {
    Serial.println("No position supplied.  Please enter a number between 0 - 2048"); 
  }
}

void getElbowRotatePos(){
  curPosition = dynaControl.getServoPosition(ELBOW_R);
  #ifdef I2CCOMMANDDEBUG
    Serial.print("ELBOW ROTATE POS: ");
    Serial.println(curPosition);
  #endif
}

void getElbowPivotPos(){
  curPosition = dynaControl.getServoPosition(ELBOW_P);
  #ifdef I2CCOMMANDDEBUG
    Serial.print("ELBOW PIVOT POS: ");
    Serial.println(curPosition);
  #endif
}

void resetElbow(){
  for(int i=2; i<NUM_SERVOS; i++){
      dynaControl.resetServoTorqueLimit(servoID[i], MAX_TORQUE);
  }
}
#endif

/* TO DO:
 *  SET THIS UP FOR I2C CALLBACK
 */
void getAllMotorPositions(){  
  Serial.print(getShoulderPosition());
  for(int i=0; i<NUM_SERVOS; i++){
    Serial.print(", ");
    Serial.print(dynaControl.getServoPosition(servoID[i]));
  }
  #ifdef HALF_ARM
  Serial.println();
  #else
  Serial.print(", ");
  #endif
}

int getShoulderPosition(){
  return stepperPosition;
}

int moveStepperToPosition(int targetPos){

  targetPos = checkTargetPositionLimit(targetPos);

  int differenceOfSteps = (abs)(stepperPosition - targetPos);
  int addStep;
  
  if(stepperPosition < targetPos){
    digitalWrite(dirpin, UPWARDS);
    stepDirection = UPWARDS;
    addStep = 1;
  } else {
    digitalWrite(dirpin, DOWNWARDS);
    stepDirection = DOWNWARDS;
    addStep = -1;
  }

  if(differenceOfSteps){

    /*
    #ifdef I2CCOMMANDDEBUG
      Serial.print("Stepper moving to: "); 
      Serial.print(targetPos); 
      Serial.print(" in ");
      Serial.print(differenceOfSteps);
      Serial.print(" steps in the Direction of: ");
      Serial.println(stepDirection);
    #endif
    */
    
    //if(checkLimitSwitches(stepDirection)){
      if((micros() - stepperTime) > stepperInterval){
        pulseStepper();
        stepperPosition += addStep;
        stepperTime = micros();
      }
    //}
  }
  
  return differenceOfSteps;
}

int checkTargetPositionLimit(int tarPos){
  if(tarPos > TOTALSTEPS){
    tarPos = TOTALSTEPS;
  } else if(tarPos < 0){
    tarPos = 0;
  }

  return tarPos;
}

bool checkLimitSwitches(int stepperDir){
  bool sensorVal0, sensorVal1, stepperState;
  sensorVal0 = checkSensor(sensorPin0);
  sensorVal1 = checkSensor(sensorPin1);
  if(!sensorVal0){
    if(stepperDir == UPWARDS){
      stepperState = true;
    } else {
      stepperState = false;
    }
  } else if(!sensorVal1){
    if(stepperDir == DOWNWARDS){
      stepperState = true;
    } else {
      stepperState = false;
    }
  } else {
    stepperState = true;
  }
  
  return stepperState;
}

void pulseStepper(){
  digitalWrite(steppin, LOW);
  digitalWrite(steppin, HIGH);
}


void serialMotorTest()
{
  int i;

  digitalWrite(dirpin, LOW);     // Set the direction.

  #ifdef I2CCOMMANDDEBUG
  Serial.println("Moving Forward");
  #endif
  for (i = 0; i<1000; i++)       // Iterate for 4000 microsteps.
  {
    digitalWrite(steppin, LOW);  // This LOW to HIGH change is what creates the
    digitalWrite(steppin, HIGH); // "Rising Edge" so the easydriver knows to when to step.
    delayMicroseconds(200);      // This delay time is close to top speed for this
  }                              // particular motor. Any faster the motor stalls.

  digitalWrite(dirpin, HIGH);    // Change direction.
  delay(2000);

  #ifdef I2CCOMMANDDEBUG
  Serial.println("Moving Backward");
  #endif
  
  for (i = 0; i<1000; i++)       // Iterate for 4000 microsteps
  {
    digitalWrite(steppin, LOW);  // This LOW to HIGH change is what creates the
    digitalWrite(steppin, HIGH); // "Rising Edge" so the easydriver knows to when to step.
    delayMicroseconds(200);      // This delay time is close to top speed for this
  }      
  delay(2000);

}

void addI2CCommand(const char *command, void (*function)())
{
 if (numCommand < MAXI2CCOMMANDS) {
    #ifdef I2CCOMMANDDEBUG
    Serial.print(numCommand); 
    Serial.print("-"); 
    Serial.print("Adding command for "); 
    Serial.println(command); 
    #endif
    
    strncpy(CommandList[numCommand].i2cCommand,command,I2CCOMMANDBUFFER); 
    CommandList[numCommand].function = function; 
    numCommand++; 
  } else {
    // In this case, you tried to push more commands into the buffer than it is compiled to hold.  
    // Not much we can do since there is no real visible error assertion, we just ignore adding
    // the command
    #ifdef I2CCOMMANDDEBUG
    Serial.println("Too many handlers - recompile changing MAXSERIALCOMMANDS"); 
    #endif 
  }
}

void SayHello()
{
  char *arg;  
  arg = nextArgument();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    Serial.print("Hello "); 
    Serial.println(arg); 
  } 
  else {
    Serial.println("Hello, whoever you are"); 
  }
}
void respondEvent(){
  byte posBuffer[2];

  posBuffer[0] = curPosition >> 8;
  posBuffer[1] = curPosition & 255;
  Wire.write(posBuffer, 2);

  #ifdef I2CCOMMANDDEBUG
  Serial.print(posBuffer[0]);
  Serial.println(posBuffer[1]);
  #endif
}
void receiveEvent(int howMany) 
{
 // If we're using the Hardware port, check it.   Otherwise check the user-created SoftwareSerial Port
  while (Wire.available() > 0) 
  {
    int i; 
    boolean matched; 
    inChar = Wire.read();   // Read single available character, there may be more waiting
    #ifdef I2CCOMMANDDEBUG
    Serial.print(inChar);   // Echo back to serial stream
    #endif
    if (inChar == terminator) {     // Check for the terminator (default '\r') meaning end of command
      #ifdef I2CCOMMANDDEBUG
      Serial.print("Received: "); 
      Serial.println(charBuffer);
       #endif
      bufPos=0;           // Reset to start of buffer
      token = strtok_r(charBuffer,delim,&last);   // Search for command at start of buffer
      if (token == NULL) return; 
      matched=false; 
      for (i=0; i<numCommand; i++) {
        #ifdef I2CCOMMANDDEBUG
        Serial.print("Comparing ["); 
        Serial.print(token); 
        Serial.print("] to [");
        Serial.print(CommandList[i].i2cCommand);
        Serial.println("]");
        #endif
        // Compare the found command against the list of known commands for a match
        if (strncmp(token,CommandList[i].i2cCommand,I2CCOMMANDBUFFER) == 0) 
        {
          #ifdef I2CCOMMANDDEBUG
          Serial.print("Matched Command: "); 
          Serial.println(token);
          #endif
          // Execute the stored handler function for the command
          (*CommandList[i].function)(); 
          clearI2CBuffer(); 
          matched=true; 
          break; 
        }
      }
      if (matched==false) {
        (*defaultHandler)(); 
        clearI2CBuffer(); 
      }

    }
    if (isprint(inChar))   // Only printable characters into the buffer
    {
      charBuffer[bufPos++]=inChar;   // Put character into buffer
      charBuffer[bufPos]='\0';  // Null terminate
      if (bufPos > I2CCOMMANDBUFFER-1) bufPos=0; // wrap buffer around if full  
    }
  }
}

char *nextArgument() 
{
 char *nextToken;
  nextToken = strtok_r(NULL, delim, &last); 
  return nextToken; 
}

void clearI2CBuffer()
{
  for (int i=0; i<I2CCOMMANDBUFFER; i++) 
  {
    charBuffer[i]='\0';
  }
  bufPos=0; 
}

void addDefaultHandler(void (*function)())
{
  defaultHandler = function;
}

void unrecognized(){
  Serial.println("Sorry bro, not sure what you're trying to do here");
}
