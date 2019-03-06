#include <SPI.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#define UDP_TX_PACKET_MAX_SIZE 256 //increase UDP size

LiquidCrystal_I2C lcd(0x3F,20,21);  // set the LCD address to 0x27 for a 16 chars and 2 line 

/************************************************************
// network stuff
/***********************************************************/

// local MAC address, fake
byte mac[] = { 0x04, 0x7D, 0x4B, 0x28, 0x51, 0x33 };
// local ip
IPAddress localIPNum(172, 16, 1, 105);
byte gatewayIPNum[] = { 172, 16, 0, 1 }; 
byte dnsIPNum[]  = { 8, 8, 8, 8 }; 
byte maskIPNum[] = { 255, 255, 0, 0 }; 

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

//****************************
// Set Pins for arduino
//****************************
const int eth_CS       = 10;  //cable select pin for ethernet shield
const int pin_CS       = 22;  // cable select pin
const int pin_IOUpdate = 23;  // update pin
const int pin_Reset    = 24;  // reset DDS pin
const int pin_RcvCmd = 30;    // TTL to receive commands over Ethernet connection
const int pin_NextCmd = 32;   // TTL to select next command from list of received commands
const int pin_ExeCmd = 31;    // TTL to execute the selected command
const int pin_P0    = 40;  // input pin P0
const int pin_P1    = 41;  // input pin P1
const int pin_P2    = 42;  // input pin P2
const int pin_P3    = 43;  // input pin P3
const int pin_IO1    = 44;  // input output pin IO1
const int pin_IO2    = 45;  // input output pin IO2
const int pin_IO3    = 46;  // input output pin IO3
int profilePin;
bool pinStatus = 0;

int idleDelay = 100;

//unsigned long startSweepTimer;
//unsigned long sweepTimer;
//unsigned long durt;
String commandString;       //String used to store our command list later
bool bHasReset = 0;         //Used to check whether the DDS was reset during the DDSinit function. This is for AOM protection, if the DDS has begun outputting and is reset it will stop outputting!
static float sineTable[] = {0.00, 0.02, 0.03, 0.05, 0.07, 0.09, 0.10, 0.12, 0.14, 0.16, 0.17, 0.19, 0.21, 0.22, 0.24, 0.26, 0.28, 0.29, 0.31, 0.33, 0.34, 0.36, 0.37, 0.39, 0.41, 0.42, 0.44, 0.45, 0.47, 0.48, 0.50, 0.51, 0.53, 0.54, 0.56, 0.57, 0.59, 0.60, 0.62, 0.63, 0.64, 0.66, 0.67, 0.68, 0.69, 0.71, 0.72, 0.73, 0.74, 0.75, 0.77, 0.78, 0.79, 0.80, 0.81, 0.82, 0.83, 0.84, 0.85, 0.86, 0.87, 0.87, 0.88, 0.89, 0.90, 0.91, 0.91, 0.92, 0.93, 0.93, 0.94, 0.95, 0.95, 0.96, 0.96, 0.97, 0.97, 0.97, 0.98, 0.98, 0.98, 0.99, 0.99, 0.99, 0.99, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00};
byte timeStep[90];
unsigned int delayTime;
double defaultFreq = 72.0;

typedef enum {
  CH0               = 0x10, //AD9959
  CH1               = 0x20, //AD9959
  CH2               = 0x40, //Channel 0 in AD9958, Channel2 in AD9959
  CH3               = 0x80,  //Channel 1 in AD9958, channel3 in AD9959
  BOTH              = 0xC0  //Channels 0 and 1 simultaneously
} ad9959_channels;

typedef enum {
  CSR               = 0x00, //Channel select register
  FR1               = 0x01, //Function Register 1
  FR2               = 0x02, //Function Register 2
  CFR               = 0x03, //Channel Function Register
  CTW0              = 0x04, //Channel Tuning Word 0
  CPW0              = 0x05, //Channel Phase Tuning Word 0
  ACR               = 0x06, //Amplitude Control Register
  LSR               = 0x07, //Linear Sweep Ramp Rate
  RDW               = 0x08, //Rising Delta Word
  FDW               = 0x09, //Falling Delta Word
  CTW1              = 0x0A, //Channel Tuning Register 1
  CTW2              = 0x0B, //...and so on
  CTW3              = 0x0C, //Only CTW0 and CTW1 are needed for our application, start and end frequencies, one rate.
  CTW4              = 0x0D,
  CTW5              = 0x0E,
  CTW6              = 0x0F,
  CTW7              = 0x10,
  CTW8              = 0x11,
  CTW9              = 0x12,
  CTW10             = 0x13,
  CTW11             = 0x14,
  CTW12             = 0x15,
  CTW13             = 0x16,
  CTW14             = 0x17,
  CTW15             = 0x18,
  READ              = 0x80 // not really a register
} ad9959_registers;

//Now initialising the data arrays for filling the registers
  byte CSRbyte      = 0x00; 
  byte FR1byte[3]   = {0x00, 0x00, 0x00};
  byte FR2byte[2]   = {0x00, 0x00};
  byte CFRbyte[3]   = {0x00, 0x00, 0x00};
  byte CTW0byte[4]  = {0x00, 0x00, 0x00, 0x00};
  byte CPW0byte[2]  = {0x00, 0x00};
  byte ACRbyte[3]   = {0x00, 0x00, 0x00};
  byte LSRbyte[2]   = {0x00, 0x00};
  byte RDWbyte[4]   = {0x00, 0x00, 0x00, 0x00};
  byte FDWbyte[4]   = {0x00, 0x00, 0x00, 0x00};
  byte CTW1byte[4]  = {0x00, 0x00, 0x00, 0x00};  
  //byte default72MHz[4] = {0x24, 0xdd, 0x2f, 0x1b}; SETTING DEFAULT FREQUENCY NOW IN clearCommandList() FUNCTION
  //byte default80MHz[4] = {0x28, 0xf5, 0xc2, 0x8f}; SETTING DEFAULT FREQUENCY NOW IN clearCommandList() FUNCTION
  byte chan = 0;

//data structure for command queue
struct tCommand {
  int rcvCmd;
  ad9959_channels chan;
  double startFreq;
  double stopFreq;
  double duration;
  byte startFreqWrite[4];
  byte stopFreqWrite[4];
  byte riseWord[4];
  byte fallWord[4];
  byte rampRate[2];
};

#define maxNCommands 5
tCommand commandVect[maxNCommands];
int usedNCommands = 0;      // number of commands used 
int actNCommand = 0;        // index of the active command

//****************************
// setup
//****************************
void setup ()  {
  // configure pins
  pinMode(pin_IOUpdate, OUTPUT); 
  pinMode(pin_Reset, OUTPUT);
  pinMode(pin_CS, OUTPUT); 
  pinMode(pin_P0, OUTPUT); 
  pinMode(pin_P1, OUTPUT); 
  pinMode(pin_P2, OUTPUT); 
  pinMode(pin_P3, OUTPUT); 
  pinMode(pin_IO1, OUTPUT); 
  pinMode(pin_IO2, OUTPUT); 
  pinMode(pin_IO3, OUTPUT);
  pinMode(pin_RcvCmd, INPUT);
  pinMode(pin_ExeCmd, INPUT);
  pinMode(pin_NextCmd, INPUT); 
  pinMode(eth_CS, OUTPUT);
  digitalWrite(eth_CS, LOW);
  Ethernet.begin(mac, localIPNum, dnsIPNum, gatewayIPNum, maskIPNum );
  Udp.begin(8020);
  digitalWrite(eth_CS, HIGH);

  //initialise lcd screen
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("0");
  lcd.setCursor(0,1);
  lcd.print("1");
  
  //set default output for DDS (safe value for AOM)
  setDefaultSetting();
  clearCommandList();
}

void loop()  {
  int retValue;
  bool bExeFlag = 0; //We are not in execute mode
  bool doIDisplay = 1; //Update the text on the LCD display
  
  if (!digitalRead(pin_RcvCmd)) //If the digital input to select receive mode is not active, go back to start to loop
  {
    delay(idleDelay);
    return;
  }
  digitalWrite(eth_CS, LOW);                    //Select ethernet shield so that it can be used without interfering with DDS
  retValue = receiveCommand();                  //Receive commands through ethernet port, returns value > 0 for successful read
  digitalWrite(eth_CS, HIGH);                   //Deselect ethernet shield in order to send commands to DDS again
  if (!retValue) return;                        //If we haven't successfully received data via ethernet port, go back to start of loop
  if (!parseCommandString()) return;            //run function parseCommandString (~line 500), if a 0 is returned, go back to start of loop
  if (commandVect[actNCommand].chan == CH2) lcd.setCursor(1,0);  //If the current command selects CH2, point to CH2 index on LCD screen 
  else if (commandVect[actNCommand].chan == CH3) lcd.setCursor(1,1); //If the current command selects CH3, point to CH3 index on LCD screen
  lcd.print("COMMANDS PARSED"); //Indicate readiness to go ahead with execution mode
      
  while(digitalRead(pin_RcvCmd)) delay(1);  //We have already received commands, do nothing until the receive pin is switched off
  while(actNCommand<=usedNCommands) //while active command is less than or equal to the number of commands in memory
  {  
    if(doIDisplay == 1) //Update LCD 
    {  
      if (commandVect[actNCommand].chan == CH2) lcd.setCursor(1,0);
      else if (commandVect[actNCommand].chan == CH3) lcd.setCursor(1,1);
      lcd.print("COMMAND ");
      lcd.print(actNCommand);
      lcd.print(" READY");
      doIDisplay = 0;
    }
    
    if (digitalRead(pin_RcvCmd)) return; //If rcvCmd pin is switched on during interpreter or execution mode, go back to start of loop
    else if (!bExeFlag && digitalRead(pin_ExeCmd)) //if a command is not currently being executed, and the execute pin is switched on
    {
      executeCommand(); //execute the selected command
      actNCommand++; //Increment the position in the command queue
      bExeFlag = 1; //We are now running a command
    }
    else if (bExeFlag && !digitalRead(pin_ExeCmd)) //If we are running a command and the execute pin is switched off
    {
      bExeFlag = 0; //get ready to execute the next command
      doIDisplay = 1; //Update LCD display to reflect the change in command
      if (actNCommand == usedNCommands) break; //exit while loop if we are at the end of the command list
    }
  }
  setDefaultSetting(); //Go back to default 72 MHz for AOM safe value
  return; //Go back to start
} 

//****************************
// set all commands to default command for safety
//****************************
void clearCommandList() {
  for (int i = 0; i < 5; i++){
    commandVect[i].rcvCmd    = 1;
    commandVect[i].chan      = BOTH;
    commandVect[i].startFreq = defaultFreq;
    commandVect[i].stopFreq  = defaultFreq;
    commandVect[i].duration  = 0;
//    80 MHz hex value used as default below
//    commandVect[i].startFreqWrite[0] = 0x28;
//    commandVect[i].startFreqWrite[1] = 0xf5;
//    commandVect[i].startFreqWrite[2] = 0xc2;
//    commandVect[i].startFreqWrite[3] = 0x8f;
//  72 MHz hex value used as default below
    commandVect[i].startFreqWrite[0] = 0x24;
    commandVect[i].startFreqWrite[1] = 0xdd;
    commandVect[i].startFreqWrite[2] = 0x2f;
    commandVect[i].startFreqWrite[3] = 0x1b;
    //
    for(int j = 0; j<4; j++){
      commandVect[i].stopFreqWrite[j] = 0x00;
      commandVect[i].riseWord[j] = 0x00;
      commandVect[i].fallWord[j] = 0x00;
    }
    for (int j=0;j<2;j++){    
      commandVect[i].rampRate[j] = 0x00;
    }
  }
  usedNCommands = 0; //ignore the list of commands until it's repopulated
  actNCommand   = 0;
}
/*
Frequency and sweep rate conversion - converts 
frequency in Hz to frequency word value as per 
equations in AD9958/AD9959 datasheet.
*/
void calcParam(int cIndex, ad9959_registers REG, double param){  
  if ((REG == 4) || (REG > 7))
  {
    unsigned long paramWord = (unsigned long)(pow(2,32)*(param/500));    //Convert from the value given to the word value understood by the DDS
    if (REG == 4)
    {
      commandVect[cIndex].startFreqWrite[3] = (byte) paramWord;
      commandVect[cIndex].startFreqWrite[2] = (byte) (paramWord >> 8);
      commandVect[cIndex].startFreqWrite[1] = (byte) (paramWord >> 16);
      commandVect[cIndex].startFreqWrite[0] = (byte) (paramWord >> 24);
    }
    else if (REG == 8)
    {
      commandVect[cIndex].riseWord[3] = (byte) paramWord;
      commandVect[cIndex].riseWord[2] = (byte) (paramWord >> 8);
      commandVect[cIndex].riseWord[1] = (byte) (paramWord >> 16);
      commandVect[cIndex].riseWord[0] = (byte) (paramWord >> 24);
    }
    else if (REG == 9)
    {
      commandVect[cIndex].fallWord[3] = (byte) paramWord;
      commandVect[cIndex].fallWord[2] = (byte) (paramWord >> 8);
      commandVect[cIndex].fallWord[1] = (byte) (paramWord >> 16);
      commandVect[cIndex].fallWord[0] = (byte) (paramWord >> 24);
    }
    else if (REG == 10)
    {
      commandVect[cIndex].stopFreqWrite[3] = (byte) paramWord;
      commandVect[cIndex].stopFreqWrite[2] = (byte) (paramWord >> 8);
      commandVect[cIndex].stopFreqWrite[1] = (byte) (paramWord >> 16);
      commandVect[cIndex].stopFreqWrite[0] = (byte) (paramWord >> 24);
    }
    else if (REG > 10)
    {
      //I haven't programmed that path yet
      return;
    }
  }
  else if (REG == 7)                                                        //If the register is Linear Sweep Ramp Rate
  {
    unsigned long paramWord = (unsigned long) param;
    commandVect[cIndex].rampRate[1] = (byte) paramWord;
    commandVect[cIndex].rampRate[0] = (byte) paramWord;
  }
}

void programDDS()
{
  if (commandVect[actNCommand].rcvCmd == 1)
  {
    writeReg(CSR, commandVect[actNCommand].chan);
    writeReg(CTW0, commandVect[actNCommand].startFreqWrite, 4);
  }
  else if (commandVect[actNCommand].rcvCmd == 2)  //if linear sweep
  {
    if (commandVect[actNCommand].chan == CH3) profilePin = pin_P3;
    else if (commandVect[actNCommand].chan == CH2) profilePin = pin_P2;    
    writeReg(CSR, commandVect[actNCommand].chan);
    if (commandVect[actNCommand].startFreq > commandVect[actNCommand].stopFreq) //if sweep down
    /*
    This requires a bit of trickery. The DDS would sweep up just fine,
    then jump straight back to the start frequency when we attempted to sweep down. 
    We trick the DDS by setting a sweep up with the end frequency being where we 
    would like to begin the down sweep from, and the start frequency being 0.12 Hz 
    below that (the best resolution the DDS will allow). Once this "sweep" has
    completed, the DDS is now outputting at the start frequency for the down sweep.
    We set the end frequency for the down sweep without changing anything else,
    causing some sort of "sweep accumulator" to reset, meaning that if we trigger
    a downwards sweep, it will actully sweep and not just jump down. 
    This is either a glitch in the DDS, or I don't understand something properly.
    Either way, it's a bit tedious to explain. 
    */
    {
      pinStatus = 1;      
      writeReg(RDW, commandVect[actNCommand].riseWord, 4);
      writeReg(FDW, commandVect[actNCommand].fallWord, 4);
      writeReg(LSR, commandVect[actNCommand].rampRate, 2);
      writeReg(CTW1, commandVect[actNCommand].startFreqWrite, 4);
      writeReg(CTW0, commandVect[actNCommand].startFreqWrite-1, 4);
      digitalWrite(profilePin, pinStatus);
      writeReg(CTW0, commandVect[actNCommand].stopFreqWrite, 4);
    }
    else //assumed sweeping up, much simpler than downwards sweep
    {
      pinStatus = 0;
      writeReg(CTW0, commandVect[actNCommand].startFreqWrite, 4);
      writeReg(CTW1, commandVect[actNCommand].stopFreqWrite, 4);
      writeReg(RDW, commandVect[actNCommand].riseWord, 4);
      writeReg(LSR, commandVect[actNCommand].rampRate, 2);
    }
  }
  else //procedures for new commands will be written here...
  {
    return;
  }
}

/*
In the event of a frequency sweep, we take the start and end frequencies and duration of the sweep, 
and attempt to create the smoothest sweep using those parameters. Smallest frequency steps, and duration
of time to stay on each frequency. This process is a little time consuming, is there some better way to do it?
*/
void calcFreqSweep(int cIndex)
{
  double hzStartFreq;
  double hzEndFreq;
  if (commandVect[cIndex].startFreq > commandVect[cIndex].stopFreq) //DDS requires for end frequency to be larger than start frequency
  {
    hzStartFreq = commandVect[cIndex].stopFreq * 1000000; //convert lower frequency to Hz 
    hzEndFreq = commandVect[cIndex].startFreq * 1000000;  //convert upper frequency to Hz
  }
  else
  {
    hzStartFreq = commandVect[cIndex].startFreq * 1000000;
    hzEndFreq = commandVect[cIndex].stopFreq * 1000000;
  }
  unsigned long n = 65536; //Coefficient to help us smooth out the sweep as much as possible
  bool foundFlag = 0;       //If we find n value that works, this will be set to 1
  float dTime, dFreq;       //How much should the frequency change, how long should it stay there?
  int k = 10000;    //k, l, m, just various values to increment/decrement in this loop
  int l = 0;
  int m = 0;
  int minNValue = 1;  //Smallest value for n that can yield acceptable results
  unsigned long maxNValue = min(commandVect[cIndex].duration/0.000000008, (hzEndFreq-hzStartFreq)/0.12); //max acceptable value for n
  do      
  {
    dTime = (125000000.0*commandVect[cIndex].duration)/n;   //calculate amount of time DDS should remain on each frequency during the sweep
    dFreq = 1.0*(hzEndFreq-hzStartFreq)/n;    //calculate the jump in frequency per step in the sweep
    if ((dFreq < 0.12) || (dTime < 1)) //if either of these variables are smaller than the best resolution allowed by the DDS
    {
      l = 1; //set direction of scan for optimal dTime and dFreq
      n = n - k;  //decrease n (by 10000 at first for speed, if we overshoot we change the order of magnitude for the scan)
    }
    else if ((dFreq > (hzEndFreq-hzStartFreq)) || (dTime > 255)) //if the frequency step is larger than the sweep itself, or time steps are larger than allowed
    {
      l = -1;   //set direction of scan for optimal dTime and dFreq
      n = n + k;  //increase n (by 10000 at first for speed, if we overshoot, we change the order of magnitude of the scan)
    }
    else if((dFreq <= (hzEndFreq-hzStartFreq)) && (dTime <= 255) && (dFreq >= 0.12) && (dTime >= 1)) //if dF and dT are within acceptable ranges
    {        
      if (k > 1)    //if increment/decrement value is more than 1 and we've overshot the optimal answer for dT and dF
      {
        m = l*k;    //flip the scan direction for one step
        n = n + m;  //go back one step
        k = k/10;   //lose an order of magnitude, we overshot and now we need to take smaller steps
      }
      else foundFlag = 1; //if k is one and we overshot, we've found our optimal answer
    }
    else
    {
      //error
      return;
    }
    if (((n<minNValue) || (n>maxNValue)) && (k != 1)) 
    {
        m = l*k;
        n = n + m;
        k = k/10;
    }
    else if (((n<minNValue) || (n>maxNValue)) && (k == 1)) break;
  }while (foundFlag == 0);
  

  if(foundFlag==0)    //we can't take the sweep if we could not find a solution for dFreq or dTime
  {  
    if (commandVect[cIndex].chan == CH2) lcd.setCursor(1,0);
    else if (commandVect[cIndex].chan == CH3) lcd.setCursor(1,1);
    lcd.print("PROBLEMi");
    lcd.print(cIndex);
    lcd.print("NOnVAL");
    
    return;
  }
  
  //convert dFreq to MHz, the conversion to frequency word starts from MHz.
  dFreq = dFreq / 1000000.0;

  //fill registers with calculated values
  calcParam(cIndex, CTW0, commandVect[cIndex].startFreq);
  calcParam(cIndex, CTW1, commandVect[cIndex].stopFreq);
  calcParam(cIndex, RDW, dFreq);
  calcParam(cIndex, FDW, dFreq);
  calcParam(cIndex, LSR, dTime);
}

//Simple settings back to default, safe values for AOM
void setDefaultSetting(){
  clearCommandList(); //Clear command vector and set 72 MHz constant output
  CFRbyte[0] = 0x00; CFRbyte[1] = 0x03; CFRbyte[2] = 0x00; 
  initDDS(BOTH);  //Reset DDS and quickly set default safe frequency output on both channels (AD9958)
  programDDS();   //Take default commands and fill registers with default values
  lcd.setCursor(1,0);   //Update LCD screen to reflect default settings are in effect
  lcd.print("DEFAULT        ");
  lcd.setCursor(1,1);
  lcd.print("DEFAULT        ");
}

void initDDS(byte initChan) { 
  digitalWrite(pin_P0, LOW); //Make sure all digital pins are set to low before we reset the DDS
  digitalWrite(pin_P1, LOW);
  digitalWrite(pin_P2, LOW);
  digitalWrite(pin_P3, LOW);
  digitalWrite(pin_IO1, LOW);
  digitalWrite(pin_IO2, LOW);
  digitalWrite(pin_IO3, LOW);

  digitalWrite(pin_CS, HIGH);  // ensure SS stays high so we don't communicate with DDS until it's been reset
  SPI.begin();
  SPI.setClockDivider(2);                 //Can be integer from 1 - 255 using Due, divides the 84MHz clock rate by this number
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);    

  if (!bHasReset){
    resetDDS();
    bHasReset = 1;
  }
  writeReg(CSR, initChan);  //Channel select register, select the desired channel
  FR1byte[0] = 0xD0; FR1byte[1] = 0x54;
  writeReg(FR1, FR1byte, sizeof(FR1byte));    //function register 1
  writeReg(FR2, FR2byte, sizeof(FR2byte));    //function register 2
  writeReg(CFR, CFRbyte, sizeof(CFRbyte));    //channel function register
}

 // flip reset pin
 void resetDDS() {
    digitalWrite(pin_Reset, HIGH);   
    digitalWrite(pin_Reset, LOW);    
 }

 // trigger update pin, has to happen whenever a register is written to
 void updateDDS() {
    digitalWrite(pin_IOUpdate, HIGH); 
    digitalWrite(pin_IOUpdate, LOW); 
 }


//Function to write to registers on the DDS that only require one byte to be sent
 void writeReg(ad9959_registers REG, byte value) {
    digitalWrite(pin_CS, LOW);  //We want to communicate with the DDS chip
    SPI.transfer(REG);    //Send register
    SPI.transfer(value);  //Send value this register should be filled with
    updateDDS();          //DDS should now update selected register
    digitalWrite(pin_CS, HIGH);  //Stop communicating with DDS chip
 }

//Function to write to registers that requre multiple bytes to be sent
 void writeReg(ad9959_registers REG, byte *buffer, int len){
    digitalWrite(pin_CS, LOW); //Open communication with DDS
    SPI.transfer(REG);         //Transfer register we want to fill
    SPI.transfer(buffer,len);  //Transfer bytes that should be contained in that register
    updateDDS();               //Update DDS with new register value
    digitalWrite(pin_CS, HIGH);  //Close communication with DDS
 }

//This is what we do whenever the Ethernet port is active
int receiveCommand()
{
  // buffer to receive string with UDP
  char UDPBuffer[UDP_TX_PACKET_MAX_SIZE];     //UDP Buffer of max possible size
  int ret = 0;                                //Set return value to 0, if commands are successfully received, this is set to 1 later
  int packLength = Udp.parsePacket();         //Check the size of the packet
  int expectedResponse = 12;                  //Minimum string size the Ardunio should accept, anything less and some data has probably been lost
  commandString = "";     // clear the commandString
  
  while (packLength>0) {                      //While there are still characters left in the received string
    if (packLength >= expectedResponse) {     //If the packet is longer than the minimum we would expect, read it
      Udp.read(UDPBuffer, UDP_TX_PACKET_MAX_SIZE);  // read packet into the buffer
      for (int i = 0; i < packLength; i++){         
        commandString = commandString + UDPBuffer[i];   //convert the UDP packet to a string
      }
      ret = 1;  //success!
    }
    packLength = Udp.parsePacket(); //check how much data is left over in case we didn't catch everything
  }
  return ret;
}

//Now that we have a command string, we need to interpret it
int parseCommandString() {
  int ret = 0;    //If this function is successful, ret will be set to 1 later
  String stringParse = "";    // dummy string 
  String stringParseAT = "";  // dummy string 
  
  int chSelect;               // channel
  int rcvCmd;                 // command mode  
  double startFreq, stopFreq, duration; //Rest of parameters 
  
  int cIndexAT;                     //position of @ symbol, marks the end of a command
  int cIndexHASH,cIndexStart;       //position of # symbol, marks end of each parameter
  int cIndexCounter = 0;            //This is how many separate commands have been detected
  
  clearCommandList();   

  commandString.trim();                       //remove any white space from commandString
  if (commandString.length()==0) return ret;  //If commandString has a length of 0, no point in continuing. Return 0 for error.
  cIndexAT = commandString.indexOf('@');      //Find position of @, so we know at what index the first command ends
    
  //while there is an @ symbol has been detected in the string and fewer than the max commands have already been detected
  while (cIndexAT!=-1 && cIndexCounter<maxNCommands) {  

    // get the substring up to next @
    stringParseAT = commandString.substring(0,cIndexAT);
    
    
    //Operation mode
    cIndexHASH = stringParseAT.indexOf('#',0);  //find position of next # symbol
    if (cIndexHASH==-1) return ret;           //return 0 for error if # isn't found
    stringParse = stringParseAT.substring(0,cIndexHASH);  //make substring from index 0 to the position before the next # symbol
    rcvCmd = stringParse.toInt();             //extract operation mode, convert to integer
    if (rcvCmd<1 || rcvCmd>3) return ret;     //return 0 for error if we receive a number we didn't expect for an operation mode
    commandVect[cIndexCounter].rcvCmd = rcvCmd;   //save this to the command list array
    cIndexStart = cIndexHASH+1;   //take index for start of next parameter

    //Output channel
    cIndexHASH = stringParseAT.indexOf('#',cIndexStart);  //find position of next # symbol
    if (cIndexHASH==-1) return ret;     //return error if # isn't found
    stringParse = stringParseAT.substring(cIndexStart,cIndexHASH);    //make substring from start of this parameter to next #
    chSelect = stringParse.toInt();   //extract ouput channel from string, convert to integer
    switch (chSelect) {               //React differently based on what channel is selected
    case 1:
      commandVect[cIndexCounter].chan = CH2; break;
    case 2:
      commandVect[cIndexCounter].chan = CH3; break;
    default:
      return ret;     //return 0 if anything other than a value corresponding to CH2 or CH3 is received
    }
    cIndexStart = cIndexHASH+1;   //Find index after next #
    
    //start frequency
    cIndexHASH = stringParseAT.indexOf('#',cIndexStart); 
    if (cIndexHASH==-1) return ret;
    stringParse = stringParseAT.substring(cIndexStart,cIndexHASH);  
    startFreq = stringParse.toDouble();               //take frequency value from string, convert it to double
    if (startFreq<60 || startFreq>110) return ret;    //compare frequency to accepted safe range for AOM, return error if outwith that range
    commandVect[cIndexCounter].startFreq = startFreq; //add to command array
    cIndexStart = cIndexHASH+1;

    //end frequency
    cIndexHASH = stringParseAT.indexOf('#',cIndexStart);
    if (cIndexHASH==-1) commandVect[cIndexCounter].stopFreq = defaultFreq;
    else {   
      stringParse = stringParseAT.substring(cIndexStart,cIndexHASH);
      stopFreq = stringParse.toDouble();                //find end frequency in string, convert to double
      if (stopFreq<60 || stopFreq>100) return ret;      //return error if this is outwith acceptable range
    }
    commandVect[cIndexCounter].stopFreq = stopFreq;
    cIndexStart = cIndexHASH+1;

    //sweep duration
    stringParse = stringParseAT.substring(cIndexStart);
    if (stringParse.length()<1) commandVect[cIndexCounter].duration = 0.0;
    else {   
      duration = stringParse.toDouble();                //find duration in string, convert to double
      if (duration<=0 || duration>10) return ret;        //return error if duration is outwith acceptable range
    }
    commandVect[cIndexCounter].duration = duration;      //add sweep duration to the command array
    calcFreqSweep(cIndexCounter);
    
    commandString.remove(0,cIndexAT+1);
    cIndexAT = commandString.indexOf('@');
    cIndexCounter++;
    usedNCommands++;
  }
  
  ret = 1;
  return ret;     //return success
}

void executeCommand() {

  if (actNCommand>=usedNCommands) return;
  
  String str;
  str = String(commandVect[actNCommand].rcvCmd) + " " + String(commandVect[actNCommand].chan) +  " " + String(commandVect[actNCommand].startFreq,6) + " " + String(commandVect[actNCommand].stopFreq,6) + " " + String(commandVect[actNCommand].duration,4);
  
  // nothing to be done?
  if (commandVect[actNCommand].rcvCmd == 0){
    setDefaultSetting();
    return;
  }

  // single frequency
  if (commandVect[actNCommand].rcvCmd == 1){
    // display
    if(commandVect[actNCommand].chan == CH2) lcd.setCursor(1,0);
    else if(commandVect[actNCommand].chan == CH3) lcd.setCursor(1,1);
    lcd.print("C");
    lcd.print(commandVect[actNCommand].startFreq);
    lcd.print("                ");
    CFRbyte[0] = 0x00; CFRbyte[1] = 0x03; CFRbyte[2] = 0x00;
    initDDS(commandVect[actNCommand].chan);
    programDDS();
    return;
  }   

  // linear sweep up or down
  if (commandVect[actNCommand].rcvCmd == 2){
    if(commandVect[actNCommand].chan == CH2) lcd.setCursor(1,0);
    else if(commandVect[actNCommand].chan == CH3) lcd.setCursor(1,1);
    lcd.print("L");
    lcd.print(commandVect[actNCommand].startFreq,1);
    lcd.print("-");
    lcd.print(commandVect[actNCommand].stopFreq,1);
    lcd.print(",");
    lcd.print(commandVect[actNCommand].duration);
    lcd.print("       ");
    CFRbyte[0] = 0x80; CFRbyte[1] = 0x43; CFRbyte[2] = 0x00;           //Set DDS to output frequency sweep, and enable linear sweep mode (for linear or sinusoidal sweeps)
    initDDS(commandVect[actNCommand].chan);
    programDDS();

    digitalWrite(profilePin, !pinStatus);       //Set the profile pin to whatever it isn't right now
    return;
  }

  //linear sweep up and down
  if (commandVect[actNCommand].rcvCmd == 3){
    if(commandVect[actNCommand].chan == CH2) lcd.setCursor(1,0);
    else if(commandVect[actNCommand].chan == CH3) lcd.setCursor(1,1);
    lcd.print("Z");
    lcd.print(commandVect[actNCommand].stopFreq,1);
    lcd.print("-");
    lcd.print(commandVect[actNCommand].startFreq,1);
    lcd.print(",");
    lcd.print(commandVect[actNCommand].duration);
    lcd.print("       ");
    CFRbyte[0] = 0x80; CFRbyte[1] = 0x43; CFRbyte[2] = 0x00;           //Set DDS to output frequency sweep, and enable linear sweep mode (for linear or sinusoidal sweeps)
    initDDS(commandVect[actNCommand].chan);
    programDDS();

    digitalWrite(profilePin, !pinStatus);         //Switch the profile pin to whatever it isn't set to right now
    return;
  }

  // sine frequency  
  if (commandVect[actNCommand].rcvCmd == 4){   
    if(commandVect[actNCommand].chan == CH2) lcd.setCursor(1,0);
    else if(commandVect[actNCommand].chan == CH3) lcd.setCursor(1,1);
    lcd.print("S");
    lcd.print(commandVect[actNCommand].startFreq,1);
    lcd.print("-");
    lcd.print(commandVect[actNCommand].stopFreq,1);
    lcd.print(",");
    lcd.print(commandVect[actNCommand].duration);
    lcd.print("       ");
    CFRbyte[0] = 0x80; CFRbyte[1] = 0x43; CFRbyte[2] = 0x00;           //Set DDS to output frequency sweep, and enable linear sweep mode (for linear or sinusoidal sweeps)
    initDDS(commandVect[actNCommand].chan);
  }
}
