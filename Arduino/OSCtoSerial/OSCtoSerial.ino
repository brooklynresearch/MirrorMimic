// UDP / OSC comms to serial comms for use with DynamixelController_Teensy.ino

#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>    
#include <OSCBundle.h>
#include <OSCBoards.h>

#define SERIALCOMMAND Serial2


EthernetUDP Udp;

//the Arduino's IP
IPAddress ip(172, 24, 1, 51);

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
  
  read_mac();
  Ethernet.begin(mac,ip);
  Udp.begin(inPort);
  
  Serial.println("starting");
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
//              Serial.println("Bundle has been received");
              newMes.route("/head/rotate", headRotate);
//              newMes.route("/head/pivot", headPivot);
//              newMes.route("/left/shoulder/rotate", leftShoulderRotate);
//              newMes.route("/left/shoulder/pivot", leftShoulderPivot);
//              newMes.route("/left/elbow/rotate", leftElbowRotate);
//              newMes.route("/left/elbow/pivot", leftElbowPivot);
//              newMes.route("/right/shoulder/rotate", rightShoulderRotate);
//              newMes.route("/right/shoulder/pivot", rightShoulderPivot);
//              newMes.route("/waist", waistRotate);
        }     
    }
}


// same format for all for right now, but  we may need to make specific conditions
// may also make more sense in a more generalized way to process
void headRotate(OSCMessage &msg, int addrOffset ){
  int position = 0;
  int speed = 0;
  bool valid = true;

  for(int i = 0; i < 2; i++){
    if(msg.isInt(i)){
      if(i = 0){
        position = msg.getInt(i);
      }
      else{
        speed = msg.getInt(i);
      }
    }
    else{
      valid = false;
      Serial.print("Unexpected value of, for: ");
      // maybe print address name and message?
    }
  }

  if(valid){
    // send to serial
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

