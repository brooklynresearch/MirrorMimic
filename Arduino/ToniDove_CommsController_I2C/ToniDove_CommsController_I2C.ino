// UDP / OSC comms to serial comms for use with DynamixelController_Teensy.ino

#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>    
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <Wire.h>

#define DEBUG         

#define HEAD_ADDRESS    6
#define LEFT_ADDRESS    7
#define RIGHT_ADDRESS   8   


int motorCommAddress[3] = {HEAD_ADDRESS, LEFT_ADDRESS, RIGHT_ADDRESS};
int i2cAddress = 0;
//String motorAddress;

EthernetUDP Udp;

//the Arduino's IP
IPAddress ip(192, 168, 8, 123);
IPAddress outIp(192, 168, 8, 100);

//port numbers
const unsigned int inPort = 12345;
const unsigned int outPort = 12346;

//everything on the network needs a unique MAC 
#if defined(__MK20DX128__)
// Teensy 3 has MAC burned in
static byte mac[6];

void read(uint8_t word, uint8_t *mac, uint8_t offset) {
  FTFL_FCCOB0 = 0x41;             // Selects the READONCE command
  FTFL_FCCOB1 = word;             // read the given word of read once area

  // launch command and wait until complete
  FTFL_FSTAT = FTFL_FSTAT_CCIF;
  while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF));

  *(mac+offset) =   FTFL_FCCOB5;       // collect only the top three bytes,
  *(mac+offset+1) = FTFL_FCCOB6;       // in the right orientation (big endian).
  *(mac+offset+2) = FTFL_FCCOB7;       // Skip FTFL_FCCOB4 as it's always 0.
}
void read_mac() {
  read(0xe,mac,0);
  read(0xf,mac,3);
}

#else
void read_mac() {}
 byte mac[] = {  
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // you can find this written on the board of some Arduino Ethernets or shields
#endif

//outgoing messages
OSCBundle bundleOUT;

//converts the pin to an osc address
char * numToOSCAddress( int pin){
    static char s[10];
    int i = 9;
  
    s[i--]= '\0';
  do
    {
    s[i] = "0123456789"[pin % 10];
                --i;
                pin /= 10;
    }
    while(pin && i);
    s[i] = '/';
    return &s[i];
}

void setup() {
      // some local debug -- we can see this in serial port
  Serial.begin(9600);
  Wire.begin();
  
  read_mac();
  Ethernet.begin(mac,ip);
  Udp.begin(inPort);

  delay(1000);

  Serial.println(F("====================================================================="));
  Serial.println(F("TONI DOVE :: OSC TO SERIAL"));
  Serial.println(F("====================================================================="));

  #ifdef DEBUG
  OSCMessage msg("/hello");
  msg.add("world!");
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  #endif
}

void loop(){ 
  
   OSCMessage newMes;
   int size;
 
   if( (size = Udp.parsePacket())>0)
   {
         //Serial.println("Received Packet");
//         unsigned int outPort = Udp.remotePort();

         while(size--)
           newMes.fill(Udp.read());

        if(!newMes.hasError())
        {
            newMes.route("/move", routePositionMessage);
            newMes.route("/get", routeGetMessage);
        }     
    }
}

void routeGetMessage(OSCMessage &msg, int addrOffset){
      int pinMatched;
      char motorAddress[6];
      
      //Test OSC Routing
      pinMatched = msg.match("/test", addrOffset);
      if(pinMatched){
        Serial.println("OSC \"/get\" Comms Functional");
      }

      pinMatched = msg.match("/allPos", addrOffset);
      if(pinMatched){
        getAllPositions();
      }

      #ifdef DEBUG
      Serial.print("Message Sent: ");Serial.println(motorAddress);
      #endif
}

void routePositionMessage(OSCMessage &msg, int addrOffset){
      int pinMatched;
      String motorAddress;
      //Test OSC Routing
      pinMatched = msg.match("/test", addrOffset);
      if(pinMatched){
        Serial.println("OSC \"/move\" Comms Functional");
      }
      
      pinMatched = msg.match("/head", addrOffset);
      if(pinMatched){
        i2cAddress = HEAD_ADDRESS;
        if (msg.fullMatch("/rotate", pinMatched+addrOffset)){
          motorAddress = "H_ROT";
        } else if (msg.fullMatch("/pivot", pinMatched+addrOffset)){
          motorAddress = "H_PIV";
        }
      }
      
      pinMatched = msg.match("/waist", addrOffset);
      if(pinMatched){
        i2cAddress = HEAD_ADDRESS;
        if (msg.fullMatch("/rotate", pinMatched+addrOffset)){
          motorAddress = "W_ROT";
        } else if (msg.fullMatch("/pivot", pinMatched+addrOffset)){
          motorAddress = "W_PIV";
        }
      }

      pinMatched = msg.match("/left", addrOffset);
      if(pinMatched){
        i2cAddress = LEFT_ADDRESS;
        if (msg.fullMatch("/shoulder/rotate", pinMatched+addrOffset)){
          motorAddress = "S_ROT";
        } else if (msg.fullMatch("/shoulder/pivot", pinMatched+addrOffset)){
          motorAddress = "S_PIV";
        } else if (msg.fullMatch("/elbow/rotate", pinMatched+addrOffset)){
          motorAddress = "E_ROT";
        } else if (msg.fullMatch("/elbow/pivot", pinMatched+addrOffset)){
          motorAddress = "E_PIV";
        }
      }

      pinMatched = msg.match("/right", addrOffset);
      if(pinMatched){
        i2cAddress = RIGHT_ADDRESS;
        if (msg.fullMatch("/shoulder/rotate", pinMatched+addrOffset)){
          motorAddress = "S_ROT";
        } else if (msg.fullMatch("/shoulder/pivot", pinMatched+addrOffset)){
          motorAddress = "S_PIV";
        } 
      }
    
      // DEBUG message for what route was sent
      #ifdef DEBUG
      Serial.print("Message Sent: ");Serial.println(motorAddress);
      #endif
      moveToPosition(msg, addrOffset, i2cAddress, motorAddress);
}

void getAllPositions(){
  char positionString[128];
  char positionAddr[10] = "/motorPos";
  int  p = 0;
  //do something
  for(int i=0; i<3; i++){
    Wire.beginTransmission(motorCommAddress[i]);
    Wire.write("GET_POS ");
    Wire.write("\n\r");
    Wire.endTransmission();
    while(Wire.available() > 0){
      char c = Wire.read();
      positionString[p] = c;
      p++;
      #ifdef DEBUG
      Serial.print(c);
      #endif
    }
  }
  #ifdef DEBUG
  Serial.println();
  Serial.print("Motor Positions: ");Serial.println(positionString);
  #endif
  sendOSCString(positionAddr, positionString);
}

void sendOSCString(char* outgoingAddr, char* argument){
  OSCMessage msg(outgoingAddr);
  msg.add(argument);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}



void moveToPosition(OSCMessage &msg, int addrOffset, int i2cAddr, String tmpMotor){
  int nextPosition = 0;
  char OSCAddress[40];
  bool valid = true;

  if(msg.isInt(0)){
      nextPosition = msg.getInt(0);
  }
  else{
    valid = false;
    msg.getAddress((char*)OSCAddress, addrOffset);
    Serial.print("Unexpected value of, for: ");Serial.println(OSCAddress);
    // maybe print address name and message?
  }
    
  if(valid){
    
    char motor[6];                   // Create a char array
    char charPosition[5];
    sprintf(charPosition,"%d", nextPosition);
    strcpy(motor, tmpMotor.c_str()); // Covert String to char to be able to pass it through I2C.

    #ifdef DEBUG
    Serial.print("Next Position For ");Serial.print(motor);Serial.print(" motor is: ");Serial.println(nextPosition);
    #endif

    Wire.beginTransmission(i2cAddr); 
    for(int i=0; i<strlen(motor); i++){
      Wire.write(motor[i]);
    }
    Wire.write(" ");
    for(int i=0; i<strlen(charPosition); i++){
      Wire.write(charPosition[i]);
    }
    Wire.write("\n\r");
    Wire.endTransmission();
  }
}

void check(OSCMessage &msg, int addrOffset ){
  //get source ip address
  
  OSCMessage newMes("/echo");
  newMes.add("hello");
  // send the response bundle back to where the request came from
  Udp.beginPacket(Udp.remoteIP(), outPort); 
  newMes.send(Udp);
  Udp.endPacket();
  newMes.empty(); // empty the bundle ready to use for new messages
}

