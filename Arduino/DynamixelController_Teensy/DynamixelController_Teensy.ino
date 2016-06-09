#include <ax12Controller.h>
#include <DynamixelController.h>

#define SERIALCOMMANDDEBUG 1

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
DynamixelController dynaControl = DynamixelController();  // may use or not... may go direct to AX12
#define SERIALCOMMAND Serial2


#define SERIALCOMMANDBUFFER 256
#define MAXSERIALCOMMANDS  10
#define MAXDELIMETER 2
#define SERIALBAUDRATE 115200

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

// other globals.
int            g_wVoltage;
char           g_aszCmdLine[80];
uint8_t        g_iszCmdLine;
boolean        g_fTrackServos = false;

// Values to use for servo position...
int            g_bServoID;
int            g_wServoGoalPos;
int            g_wServoGoalSpeed;

int defaultSpeed  =   200;



//====================================================================================================
// Setup 
//====================================================================================================
void setup() {
  SERIALCOMMAND.begin(115200);  // start off the serial port. 
  Serial.begin(9600);
  //Set the direction Pin to -1 for half duplex control on the Teensy 
  dynaControl.begin(1000000, &Serial1, -1, NUM_SERVOS);
  strncpy(delim," ",MAXDELIMETER);  // strtok_r needs a null-terminated string
  numCommand=0;    // Number of callback handlers installed
  clearSerialBuffer();

  addSerialCommand("HELLO", SayHello);
  
  delay(1000);
  for(int i=0; i<NUM_SERVOS; i++){
      dynaControl.setServoSpeed(servoID[i], defaultSpeed);
      SERIALCOMMAND.print("SERVO ");SERIALCOMMAND.print(servoID[i]);
      SERIALCOMMAND.print("\t\tVOLTAGE: ");SERIALCOMMAND.print(((float)dynaControl.getServoVoltage(servoID[i])/10));
      SERIALCOMMAND.print("\tTEMP: ");SERIALCOMMAND.print(dynaControl.getServoTemp(servoID[i]));
      SERIALCOMMAND.print("\tPOS: ");SERIALCOMMAND.print(dynaControl.getServoPosition(servoID[i]));
      SERIALCOMMAND.print("\tSPEED: ");SERIALCOMMAND.print(dynaControl.getServoSpeed(servoID[i]));
      SERIALCOMMAND.print("\t\tMAX ANGLES: ");
      int *p;
      p = dynaControl.getAngleLimits(servoID[i]);
        for(int j=0; j<2; j++){
          SERIALCOMMAND.print(*(p + j));SERIALCOMMAND.print("  ");
        }
      SERIALCOMMAND.println();
  }

}

void loop() {
    readSerialCommand();
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


