//#include <Thread.h>
//#include <ThreadController.h>

#define SERIALCOMMANDDEBUG 1
#define SERIALCOMMAND Serial2

#define SERIALCOMMANDBUFFER 256
#define MAXSERIALCOMMANDS  10
#define MAXDELIMETER 2
#define SERIALBAUDRATE 115200

#define TOTALSTEPS 3150
#define HOMEPOS 200
#define UPWARDS 1
#define DOWNWARDS 0

char inChar;          // A character read from the serial stream 
char serialBuffer[SERIALCOMMANDBUFFER];   // Buffer of stored characters while waiting for terminator character
int  bufPos;                        // Current position in the buffer
char delim[MAXDELIMETER];           // null-terminated list of character to be used as delimeters for tokenizing (default " ")
char terminator = '\r';                          // Character that signals end of command (default '\r')
char *token;                        // Returned token from the command buffer as returned by strtok_r
char *last;                         // State variable used by strtok_r during processing
typedef struct _callback {
  char serialCommand[SERIALCOMMANDBUFFER];
  void (*function)();
} SerialCommandCallback;            // Data structure to hold Command/Handler function key-value pairs
int numCommand;
SerialCommandCallback CommandList[MAXSERIALCOMMANDS];   // Actual definition for command/handler array
void (*defaultHandler)();   

//ThreadController controll = ThreadController();

//My Thread (as a pointer)
//Thread* stepperThread = new Thread();

int dirpin = 3;
int steppin = 4;

int sensorPin0 = A0;    // select the input pin for the potentiometer
int sensorPin1 = A1;
int sensorValue = 0;  // variable to store the value coming from the sensor
int limitThreshold = 100;

int stepperPosition = 0;
int newPosition = HOMEPOS;
unsigned long stepperTime = 0;
unsigned int stepperInterval = 300;



void stepperCallback(){
  digitalWrite(steppin, LOW);
  digitalWrite(steppin, HIGH);
}


void setup() 
{
  SERIALCOMMAND.begin(115200);  // start off the serial port. 
  Serial.begin(9600);
  strncpy(delim," ",MAXDELIMETER);  // strtok_r needs a null-terminated string
  numCommand=0;    // Number of callback handlers installed
  clearSerialBuffer();

//  // Configure myThread
//  myThread->onRun(stepperCallback);
//  myThread->setInterval(.5);
//
//  control.add(stepperCallback);   

  addSerialCommand("HELLO", SayHello);
  addSerialCommand("TEST", serialMotorTest);
  addSerialCommand("SHOULDER_ROT", moveShoulder);
  addSerialCommand("GET_POS", getShoulderPosition);
  addDefaultHandler(unrecognized);
  
  pinMode(dirpin, OUTPUT);
  pinMode(steppin, OUTPUT);

  findHome();
}

void loop()
{
  readSerialCommand();
  moveStepperToPosition(newPosition);
}

void findHome(){
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

void moveShoulder(){
  char *arg;  
  arg = nextArgument();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    newPosition = atoi(arg);
    #ifdef SERIALCOMMANDDEBUG
    SERIALCOMMAND.print("Stepper moving to: "); 
    SERIALCOMMAND.println(arg); 
    SERIALCOMMAND.print("The New Position is: ");
    SERIALCOMMAND.println(newPosition);
    #endif
//    stepperPosition = moveStepperToPosition(newPos);
  } 
  else {
    SERIALCOMMAND.println("No position supplied.  Please enter a number between 0 - 3360"); 
  }
}

void getShoulderPosition(){
  SERIALCOMMAND.println(stepperPosition);
}

int moveStepperToPosition(int targetPos){

  targetPos = checkTargetPositionLimit(targetPos);

  int differenceOfSteps = (abs)(stepperPosition - targetPos);
  int addStep, stepDirection;
  
  if(stepperPosition < targePos){
    digitalWrite(dirpin, LOW);
    stepDirection = UPWARDS;
    addStep = 1;
  } else {
    digitalWrite(dirpin, HIGH);
    stepDirection = DOWNWARDS;
    addStep = -1;
  }

  if(differenceOfSteps){
    /*
    #ifdef SERIALCOMMANDDEBUG
      SERIALCOMMAND.print("Stepper moving to: "); 
      SERIALCOMMAND.print(targetPos); 
      SERIALCOMMAND.print(" in ");
      SERIALCOMMAND.print(differenceOfSteps);
      SERIALCOMMAND.println(" steps.");
    #endif
    */
    if(checkLimitSwitches(stepDirection)){
      if((micros() - stepperTime) > stepperInterval){
        pulseStepper();
        stepperPosition += addStep;
        stepperTime = micros();
      }
    }
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

  Serial.println("Moving Forward");
  for (i = 0; i<1000; i++)       // Iterate for 4000 microsteps.
  {
    digitalWrite(steppin, LOW);  // This LOW to HIGH change is what creates the
    digitalWrite(steppin, HIGH); // "Rising Edge" so the easydriver knows to when to step.
    delayMicroseconds(400);      // This delay time is close to top speed for this
  }                              // particular motor. Any faster the motor stalls.

  digitalWrite(dirpin, HIGH);    // Change direction.
  delay(2000);

  Serial.println("Moving Backward");
  for (i = 0; i<1000; i++)       // Iterate for 4000 microsteps
  {
    digitalWrite(steppin, LOW);  // This LOW to HIGH change is what creates the
    digitalWrite(steppin, HIGH); // "Rising Edge" so the easydriver knows to when to step.
    delayMicroseconds(400);      // This delay time is close to top speed for this
  }      
  delay(2000);

}

void addSerialCommand(const char *command, void (*function)())
{
 if (numCommand < MAXSERIALCOMMANDS) {
    #ifdef SERIALCOMMANDDEBUG
    SERIALCOMMAND.print(numCommand); 
    SERIALCOMMAND.print("-"); 
    SERIALCOMMAND.print("Adding command for "); 
    SERIALCOMMAND.println(command); 
    #endif
    
    strncpy(CommandList[numCommand].serialCommand,command,SERIALCOMMANDBUFFER); 
    CommandList[numCommand].function = function; 
    numCommand++; 
  } else {
    // In this case, you tried to push more commands into the buffer than it is compiled to hold.  
    // Not much we can do since there is no real visible error assertion, we just ignore adding
    // the command
    #ifdef SERIALCOMMANDDEBUG
    SERIALCOMMAND.println("Too many handlers - recompile changing MAXSERIALCOMMANDS"); 
    #endif 
  }
}

void SayHello()
{
  char *arg;  
  arg = nextArgument();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)      // As long as it existed, take it
  {
    SERIALCOMMAND.print("Hello "); 
    SERIALCOMMAND.println(arg); 
  } 
  else {
    SERIALCOMMAND.println("Hello, whoever you are"); 
  }
}

void readSerialCommand() 
{
 // If we're using the Hardware port, check it.   Otherwise check the user-created SoftwareSerial Port
  while (SERIALCOMMAND.available() > 0) 
  {
    int i; 
    boolean matched; 
    inChar = SERIALCOMMAND.read();   // Read single available character, there may be more waiting
    #ifdef SERIALCOMMANDDEBUG
    SERIALCOMMAND.print(inChar);   // Echo back to serial stream
    #endif
    if (inChar == terminator) {     // Check for the terminator (default '\r') meaning end of command
      #ifdef SERIALCOMMANDDEBUG
      SERIALCOMMAND.print("Received: "); 
      SERIALCOMMAND.println(serialBuffer);
        #endif
      bufPos=0;           // Reset to start of buffer
      token = strtok_r(serialBuffer,delim,&last);   // Search for command at start of buffer
      if (token == NULL) return; 
      matched=false; 
      for (i=0; i<numCommand; i++) {
        #ifdef SERIALCOMMANDDEBUG
        SERIALCOMMAND.print("Comparing ["); 
        SERIALCOMMAND.print(token); 
        SERIALCOMMAND.print("] to [");
        SERIALCOMMAND.print(CommandList[i].serialCommand);
        SERIALCOMMAND.println("]");
        #endif
        // Compare the found command against the list of known commands for a match
        if (strncmp(token,CommandList[i].serialCommand,SERIALCOMMANDBUFFER) == 0) 
        {
          #ifdef SERIALCOMMANDDEBUG
          SERIALCOMMAND.print("Matched Command: "); 
          SERIALCOMMAND.println(token);
          #endif
          // Execute the stored handler function for the command
          (*CommandList[i].function)(); 
          clearSerialBuffer(); 
          matched=true; 
          break; 
        }
      }
      if (matched==false) {
        (*defaultHandler)(); 
        clearSerialBuffer(); 
      }

    }
    if (isprint(inChar))   // Only printable characters into the buffer
    {
      serialBuffer[bufPos++]=inChar;   // Put character into buffer
      serialBuffer[bufPos]='\0';  // Null terminate
      if (bufPos > SERIALCOMMANDBUFFER-1) bufPos=0; // wrap buffer around if full  
    }
  }
}

char *nextArgument() 
{
 char *nextToken;
  nextToken = strtok_r(NULL, delim, &last); 
  return nextToken; 
}

void clearSerialBuffer()
{
  for (int i=0; i<SERIALCOMMANDBUFFER; i++) 
  {
    serialBuffer[i]='\0';
  }
  bufPos=0; 
}

void addDefaultHandler(void (*function)())
{
  defaultHandler = function;
}

void unrecognized(){
  SERIALCOMMAND.println("Sorry bro, not sure what you're trying to do here");
}
