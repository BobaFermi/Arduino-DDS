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

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("0");
  lcd.setCursor(0,1);
  lcd.print("1");
  setDefaultSetting();
  clearCommandList();
}

void loop()  {
  int retValue;
  bool bExeFlag = 0;
  bool doIDisplay = 1;
  
  if (!digitalRead(pin_RcvCmd))
  {
    delay(idleDelay);
    return;
  }
  digitalWrite(eth_CS, LOW);                    //Select ethernet shield so that it can be used without interfering with DDS
  retValue = receiveCommand();                             //Receive commands through ethernet port
  digitalWrite(eth_CS, HIGH);                   //Deselect ethernet shield in order to send commands to DDS again
  if (!retValue) return;
  if (!parseCommandString()) return;
  if (commandVect[actNCommand].chan == CH2) lcd.setCursor(1,0);
  else if (commandVect[actNCommand].chan == CH3) lcd.setCursor(1,1);
  lcd.print("COMMANDS PARSED");
      
  while(digitalRead(pin_RcvCmd)) delay(1);
  while(actNCommand<=usedNCommands)
  {  
    if(doIDisplay == 1)
    {  
      if (commandVect[actNCommand].chan == CH2) lcd.setCursor(1,0);
      else if (commandVect[actNCommand].chan == CH3) lcd.setCursor(1,1);
      lcd.print("COMMAND ");
      lcd.print(actNCommand);
      lcd.print(" READY");
      doIDisplay = 0;
    }
    
    if (digitalRead(pin_RcvCmd)) return;
    else if (!bExeFlag && digitalRead(pin_ExeCmd))
    {
      executeCommand();
      actNCommand++;
      bExeFlag = 1;
    }
    else if (bExeFlag && !digitalRead(pin_ExeCmd))
    {
      bExeFlag = 0;
      doIDisplay = 1;
      if (actNCommand == usedNCommands) break;      
    }
  }
  setDefaultSetting();
  return;
} 

//****************************
// clear command structure
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
  usedNCommands = 0;
  actNCommand   = 0;
}
//****************************
// Frequency
//****************************
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
      //NOT PROGRAMMED YET
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
    else //assumed sweeping up
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

void calcFreqSweep(int cIndex)
{
  double hzStartFreq;
  double hzEndFreq;
  if (commandVect[cIndex].startFreq > commandVect[cIndex].stopFreq)
  {
    hzStartFreq = commandVect[cIndex].stopFreq * 1000000;
    hzEndFreq = commandVect[cIndex].startFreq * 1000000;
  }
  else
  {
    hzStartFreq = commandVect[cIndex].startFreq * 1000000;
    hzEndFreq = commandVect[cIndex].stopFreq * 1000000;
  }
  unsigned long n = 65536;
  bool foundFlag = 0;
  float dTime, dFreq;
  int k = 10000;
  int l = 0;
  int m = 0;
  int minNValue = 1;
  unsigned long maxNValue = min(commandVect[cIndex].duration/0.000000008, (hzEndFreq-hzStartFreq)/0.12);
  do      
  {
    dTime = (125000000.0*commandVect[cIndex].duration)/n;
    dFreq = 1.0*(hzEndFreq-hzStartFreq)/n;
    if ((dFreq < 0.12) || (dTime < 1))
    {
      l = 1;
      n = n - k;
    }
    else if ((dFreq > (hzEndFreq-hzStartFreq)) || (dTime > 255))
    {
      l = -1;
      n = n + k;
    }
    else if((dFreq <= (hzEndFreq-hzStartFreq)) && (dTime <= 255) && (dFreq >= 0.12) && (dTime >= 1))
    {        
      if (k > 1)
      {
        m = l*k;
        n = n + m;
        k = k/10;
      }
      else foundFlag = 1;
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
  

  if(foundFlag==0)
  {  
    if (commandVect[cIndex].chan == CH2) lcd.setCursor(1,0);
    else if (commandVect[cIndex].chan == CH3) lcd.setCursor(1,1);
    lcd.print("PROBLEMi");
    lcd.print(cIndex);
    lcd.print("NOnVAL");
    
    return;
  }
  
  dFreq = dFreq / 1000000.0;

  calcParam(cIndex, CTW0, commandVect[cIndex].startFreq);
  calcParam(cIndex, CTW1, commandVect[cIndex].stopFreq);
  calcParam(cIndex, RDW, dFreq);
  calcParam(cIndex, FDW, dFreq);
  calcParam(cIndex, LSR, dTime);
}

void setDefaultSetting(){
  clearCommandList();
  CFRbyte[0] = 0x00; CFRbyte[1] = 0x03; CFRbyte[2] = 0x00;
  initDDS(BOTH);
  programDDS();
  lcd.setCursor(1,0);
  lcd.print("DEFAULT        ");
  lcd.setCursor(1,1);
  lcd.print("DEFAULT        ");
}

void initDDS(byte initChan) { 
  digitalWrite(pin_P0, LOW);
  digitalWrite(pin_P1, LOW);
  digitalWrite(pin_P2, LOW);
  digitalWrite(pin_P3, LOW);
  digitalWrite(pin_IO1, LOW);
  digitalWrite(pin_IO2, LOW);
  digitalWrite(pin_IO3, LOW);

  digitalWrite(pin_CS, HIGH);  // ensure SS stays high
  SPI.begin();
  SPI.setClockDivider(2);                 //Can be integer from 1 - 255 using Due, divides the 84MHz clock rate by this number
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);    

  if (!bHasReset){
    resetDDS();
    bHasReset = 1;
  }
  writeReg(CSR, initChan);
  FR1byte[0] = 0xD0; FR1byte[1] = 0x54;
  writeReg(FR1, FR1byte, sizeof(FR1byte));
  writeReg(FR2, FR2byte, sizeof(FR2byte));
  writeReg(CFR, CFRbyte, sizeof(CFRbyte));
}

 // flip reset pin
 void resetDDS() {
    digitalWrite(pin_Reset, HIGH);   
    digitalWrite(pin_Reset, LOW);    
 }

 // trigger update pin
 void updateDDS() {
    digitalWrite(pin_IOUpdate, HIGH); 
    digitalWrite(pin_IOUpdate, LOW); 
 }

 void writeReg(ad9959_registers REG, byte value) {
    digitalWrite(pin_CS, LOW); 
    SPI.transfer(REG); 
    SPI.transfer(value);
    updateDDS();
    digitalWrite(pin_CS, HIGH);  
 }

 void writeReg(ad9959_registers REG, byte *buffer, int len){
    digitalWrite(pin_CS, LOW); 
    SPI.transfer(REG); 
    SPI.transfer(buffer,len);
    updateDDS();
    digitalWrite(pin_CS, HIGH);  
 }

int receiveCommand()
{
  // buffer to receive string with UDP
  char UDPBuffer[UDP_TX_PACKET_MAX_SIZE];     
  int ret = 0;
  int packLength = Udp.parsePacket();
  int expectedResponse = 12;
  commandString = "";     // clear the commandString
  
  while (packLength>0) {
    if (packLength >= expectedResponse) {
      Udp.read(UDPBuffer, UDP_TX_PACKET_MAX_SIZE);  // read packet into the buffer
      for (int i = 0; i < packLength; i++){
        commandString = commandString + UDPBuffer[i];
      }
      ret = 1;
    }
    packLength = Udp.parsePacket();
  }
  return ret;
}


int parseCommandString() {
  int ret = 0;
  String stringParse = "";    // dummy string 
  String stringParseAT = "";  // dummy string 
  
  int chSelect;               // channel
  int rcvCmd;                 // command mode  
  double startFreq, stopFreq, duration;
  
  int cIndexAT;
  int cIndexHASH,cIndexStart;
  int cIndexCounter = 0;
  
  clearCommandList();

  commandString.trim();
  if (commandString.length()==0) return ret;
  cIndexAT = commandString.indexOf('@');
    
  // parse the commandStrings
  while (cIndexAT!=-1 && cIndexCounter<maxNCommands) {

    // get the substring up to next @
    stringParseAT = commandString.substring(0,cIndexAT);
    
    // get command number
    cIndexHASH = stringParseAT.indexOf('#',0);
    if (cIndexHASH==-1) return ret;
    stringParse = stringParseAT.substring(0,cIndexHASH);
    rcvCmd = stringParse.toInt();//setting
    if (rcvCmd<1 || rcvCmd>3) return ret;
    commandVect[cIndexCounter].rcvCmd = rcvCmd;
    cIndexStart = cIndexHASH+1;

    // get device number
    cIndexHASH = stringParseAT.indexOf('#',cIndexStart);
    if (cIndexHASH==-1) return ret;
    stringParse = stringParseAT.substring(cIndexStart,cIndexHASH);
    chSelect = stringParse.toInt();//setting
    switch (chSelect) {
    case 1:
      commandVect[cIndexCounter].chan = CH2; break;
    case 2:
      commandVect[cIndexCounter].chan = CH3; break;
    default:
      return ret;
    }
    cIndexStart = cIndexHASH+1;
    
    // get start freq
    cIndexHASH = stringParseAT.indexOf('#',cIndexStart);
    if (cIndexHASH==-1) return ret;
    stringParse = stringParseAT.substring(cIndexStart,cIndexHASH);
    startFreq = stringParse.toDouble();
    if (startFreq<60 || startFreq>110) return ret;
    commandVect[cIndexCounter].startFreq = startFreq;
    cIndexStart = cIndexHASH+1;

    // get stop freq
    cIndexHASH = stringParseAT.indexOf('#',cIndexStart);
    if (cIndexHASH==-1) commandVect[cIndexCounter].stopFreq = defaultFreq;
    else {   
      stringParse = stringParseAT.substring(cIndexStart,cIndexHASH);
      stopFreq = stringParse.toDouble();
      if (stopFreq<60 || stopFreq>100) return ret;
    }
    commandVect[cIndexCounter].stopFreq = stopFreq;
    cIndexStart = cIndexHASH+1;

    stringParse = stringParseAT.substring(cIndexStart);
    if (stringParse.length()<1) commandVect[cIndexCounter].duration = 0.0;
    else {   
      duration = stringParse.toDouble();
      if (duration<0 || duration>10) return ret;
      if (duration==0) return ret;
    }
    commandVect[cIndexCounter].duration = duration;
    calcFreqSweep(cIndexCounter);
    
    commandString.remove(0,cIndexAT+1);
    cIndexAT = commandString.indexOf('@');
    cIndexCounter++;
    usedNCommands++;
  }
  
  ret = 1;
  return ret;
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
