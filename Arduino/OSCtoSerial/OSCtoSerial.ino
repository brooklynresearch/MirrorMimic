// UDP / OSC comms to serial comms for use with DynamixelController_Teensy.ino

#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>    
#include <OSCBundle.h>
#include <OSCBoards.h>

#define DEBUG         

#define HEADSERIAL    Serial1
#define LEFTSERIAL    Serial2
#define RIGHTSERIAL   Serial3
#define HEAD          0
#define LEFTSIDE      1
#define RIGHTSIDE     2

HardwareSerial motorCommPort[3] = {HEADSERIAL, LEFTSERIAL, RIGHTSERIAL};
int whichPort = 0;
String messageAddress;

EthernetUDP Udp;

//the Arduino's IP
IPAddress ip(192, 168, 0, 200);

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
  for(int i=0; i<3; i++){
    motorCommPort[i].begin(115200);
  }
  
  read_mac();
  Ethernet.begin(mac,ip);
  Udp.begin(inPort);

  Serial.println(F("====================================================================="));
  Serial.println(F("TONI DOVE :: OSC TO SERIAL"));
  Serial.println(F("====================================================================="));
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
            newMes.route("/move", routeMessage);
        }     
    }
}

void routeMessage(OSCMessage &msg, int addrOffset){
      int pinMatched;
      pinMatched = msg.match("/test", addrOffset);
      if(pinMatched){
        Serial.println("OSC Comms Functional");
      }
      pinMatched = msg.match("/head", addrOffset);
      if(pinMatched){
        whichPort = HEAD;
        if (msg.fullMatch("/rotate", pinMatched+addrOffset)){
          messageAddress = "H_ROT";
        } else if (msg.fullMatch("/pivot", pinMatched+addrOffset)){
          messageAddress = "H_PIV";
        }
      }
      
      pinMatched = msg.match("/waist", addrOffset);
      if(pinMatched){
        whichPort = HEAD;
        if (msg.fullMatch("/rotate", pinMatched+addrOffset)){
          messageAddress = "W_ROT";
        } else if (msg.fullMatch("/pivot", pinMatched+addrOffset)){
          messageAddress = "W_PIV";
        }
      }

      pinMatched = msg.match("/left", addrOffset);
      if(pinMatched){
        whichPort = LEFTSIDE;
        if (msg.fullMatch("/shoulder/rotate", pinMatched+addrOffset)){
          messageAddress = "S_ROT";
        } else if (msg.fullMatch("/shoulder/pivot", pinMatched+addrOffset)){
          messageAddress = "S_PIV";
        } else if (msg.fullMatch("/elbow/rotate", pinMatched+addrOffset)){
          messageAddress = "E_ROT";
        } else if (msg.fullMatch("/elbow/pivot", pinMatched+addrOffset)){
          messageAddress = "E_PIV";
        }
      }

      pinMatched = msg.match("/right", addrOffset);
      if(pinMatched){
        whichPort = RIGHTSIDE;
        if (msg.fullMatch("/shoulder/rotate", pinMatched+addrOffset)){
          messageAddress = "S_ROT";
        } else if (msg.fullMatch("/shoulder/pivot", pinMatched+addrOffset)){
          messageAddress = "S_PIV";
        } 
      }
      #ifdef DEBUG
      Serial.print("Message Sent: ");Serial.println(messageAddress);
      #endif
      moveToPosition(msg, addrOffset, whichPort, messageAddress);
}
// same format for all for right now, but  we may need to make specific conditions
// may also make more sense in a more generalized way to process
void printAddrOffset(OSCMessage &msg, int addrOffset){
  Serial.print("Address Offset: ");Serial.print(addrOffset);
}

void moveToPosition(OSCMessage &msg, int addrOffset, int portNumber, String addr){
  int nextPosition = 0;
  int newSpeed = 0;
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
    #ifdef DEBUG
    Serial.print("Next Position For ");Serial.print(addr);Serial.print(" is: ");Serial.println(nextPosition);
    #endif
    motorCommPort[portNumber].print(addr);motorCommPort[portNumber].println(nextPosition);
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

