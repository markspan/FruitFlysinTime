#include <PID_v1.h>
#include <stdlib.h>
#include "Nextion.h"

double Kp=3;
double Ki=5;
double Kd=1;

bool debug = false;

#define SIGNAL_HIGH      (50)
#define SIGNAL_LOW       (20)
#define SIGNAL_RANGE     (SIGNAL_HIGH-SIGNAL_LOW)

#define NEXTION

NexText versionNumber   = NexText(0, 1, "version");
NexText blockTemp       = NexText(1,23,"CBTemp");
NexProgressBar startUp  = NexProgressBar(0, 2, "startup");
NexWaveform TopAxis     = NexWaveform(1, 1, "TopAxis");
NexWaveform MiddleAxis  = NexWaveform(1, 2, "MiddleAxis");
NexWaveform LowerAxis   = NexWaveform(1, 3, "LowerAxis");
int AMP = 90/SIGNAL_RANGE;

#define BOOSTDURATION (12)


// LEDs
int Long=2;
int Short=3;

//-------------------------------------------------------------------------------
//
// Firmware enabling the control of the "FruitFly Arena" by Ruijsink Dynamic
// Engineering. Communication to the arena is done using the Arduino Due
//
// Typical use: Example here
//
// M.M.Span / D.H.van Rijn
//
// Department of Psychology
// Faculty of Behavioral Sciences
// University of Groningen, the Netherlands
//
// (C) 2013, 2014, 2017
//
//
//-------------------------------------------------------------------------------

char version[5] = "2.0c";

// History:
//
// 1.0a - 1.0d: MS: Initial versions
// 1.0e: HvR: Added (again) the boost auto-shut-off functionality. After issueing a boost command, the boost will stay on for at most BOOSTDURATION.
// 1.0f: HvR: GetTemps now works as expected & added PID Control library to update temperature
//
// Problem with PID control is that our boost works way to fast to use derivatives. The old system (large increases if temperature is clearly below set
// value worked a lot better. Reinstate that for 1.0g, and get rid of the PID.
//2
// 1.0g: HvR: Get rid of PID control. Change PWM by 1 until at 100 Hz, and extreme changes if bigger temperature changes are requested. Also copied
// all control code to main loop, and disabled timer. Reason for this is that the missing characters in the serial input stream seem to be caused by
// the timing interrupt (i.e., it doesn't happen anymore after the timer was disabled). This does mean that the execution of *all* commands passed
// over the serial loop should be executed *quickly*, as PWM and boost will not be updated during commands that need long for execution. Therefore removed
// flash and demo. Added p command, which gives a human readable output of current temperatures and PWM values.
//
// 1.0h MS: Using native USB line (enabling higher resolution measurements) and adding LCD support. HvR: Changed order of tiles by switching pin positions. (The original code refered to the left tile as 
// right, and vice versa.)
// 2.0a MS: changed inteface from LCD to Nextion
// https://www.itead.cc/wiki/Nextion_HMI_Solution#Nextion_Editor_Quick_Start_Guide
// 2.0b MS: (re)using the PID library the control the temperatures of the tiles.
// https://github.com/br3ttb/Arduino-PID-Library
// 2.0c MS: code cleanup and PID calibration. Added PID, XAXIS and YAXIS commands
// https://github.com/br3ttb/Arduino-PID-Library


// Pin number definitions:
#define LEDPIN 13

// #define TILESINVERSED
#ifdef TILESINVERSED
int setTiles[3]         = {12, 11, 10}; // PinNumbers controlling the tiles (PWM)
int setBoost[3]         = { 7,  6,  5}; // PinNumbers controlling the boost function of the tiles
int getTiles[3]         = {A0, A1, A2}; // A0,A1 AND A2
#else
int setTiles[3]         = {10, 11, 12}; // PinNumbers controlling the tiles (PWM)
int setBoost[3]         = { 5,  6,  7}; // PinNumbers controlling the boost function of the tiles
int getTiles[3]         = {A2, A1, A0}; // A0,A1 AND A2
#endif

#define setCopperBlock       9
#define getCopperBlock       A3 // A3
#define getCoolingBlock      A4 // A4
#define getReferenceVoltage  A5 // A5

// Resolution of write/read operations (in bits)
#define analogWriteResolutionBits  8 // PWM resolution
#define analogReadResolutionBits   12 // PWM resolution


// BoostUntil contains the time until which we want to sent the boost. Time is counted in millis()
unsigned long BoostUntil[3] = {0, 0, 0};


// The next values will be immediately overwritten in the Sampler loop.
double copperBlockPWM =    0;
double tilesPWM[3]    =   {0, 0, 0};

unsigned long nextPWMAdjust = 0;

// Variables indicating which temperature we aim for on the tiles/copperblock
double tilesDesiredTemp[3] =   {20, 15, 20};
double copperBlockDesiredTemp = 15;

double tilesMeasuredTemp[3] = {0, 0, 0};
double copperBlockMeasuredTemp = 0;
double prevcopperBlockMeasuredTemp=0;

// Variables for input/command parsing.
String  inputString = "";         // a string to hold incoming data
boolean stringComplete = false;   // whether the string is complete
char temp[3];
int counter=0;
//
#define CHANNELS             9
#define SAMPLERATE           100

boolean isSampling = false;        // are we currently storing temperature information in the mSampledData?

float mSampledData[CHANNELS];
uint32_t startTime=millis();
uint32_t pt = millis();
char TempMsg[16];

// -------------------------------------------------------------
//
// Start of code
//
// -------------------------------------------------------------


PID* tilesPID[3];
PID  CopperPID(&copperBlockMeasuredTemp, &copperBlockPWM, &copperBlockDesiredTemp, Kp, Ki, Kd, REVERSE);

void SamplerCB() {
  if (isSampling == true) {
    mSampledData[0] =   tilesDesiredTemp[0];
    mSampledData[1] =   tilesMeasuredTemp[0];
    // Tile 2:
    mSampledData[2] =   tilesDesiredTemp[1];
    mSampledData[3] =   tilesMeasuredTemp[1];
    // Tile 3:
    mSampledData[4] =   tilesDesiredTemp[2];
    mSampledData[5] =   tilesMeasuredTemp[2];
    // Copperblock, coolingblock, PWM
    mSampledData[6] =   copperBlockMeasuredTemp;
    mSampledData[7] =   val2tempCoolingBlock(analogRead(getCoolingBlock));
    mSampledData[8] =   millis()-startTime;
    SerialUSB.write((uint8_t*)mSampledData, ((int)CHANNELS * sizeof(float)));
  }
#ifdef NEXTION   
  TopAxis.addValue(0, AMP*(tilesMeasuredTemp[0]-SIGNAL_LOW));
  MiddleAxis.addValue(0, AMP*(tilesMeasuredTemp[1]-SIGNAL_LOW));
  LowerAxis.addValue(0, AMP*(tilesMeasuredTemp[2]-SIGNAL_LOW));

  TopAxis.addValue(1, AMP*(tilesDesiredTemp[0]-SIGNAL_LOW));
  MiddleAxis.addValue(1, AMP*(tilesDesiredTemp[1]-SIGNAL_LOW));
  LowerAxis.addValue(1, AMP*(tilesDesiredTemp[2]-SIGNAL_LOW));
#endif
}


void setup() {
  
  startUp.setValue(0);
  for (int i = 0; i < 3; i++)  {
    pinMode(setTiles[i], OUTPUT);
    pinMode(getTiles[i], INPUT);
    analogWrite(setTiles[i], 0);
    analogWrite(setBoost[i], 0); // start with pins LOW!
    tilesPID[i] = new PID(&tilesMeasuredTemp[i], &tilesPWM[i], &tilesDesiredTemp[i], Kp, Ki, Kd, DIRECT);
  }

  pinMode(LEDPIN, OUTPUT);
  analogWrite(LEDPIN, 0);

  pinMode(setCopperBlock, OUTPUT);
  pinMode(getCopperBlock,  INPUT);
  pinMode(getCoolingBlock, INPUT);
  
  // LEDs
  pinMode(Long,OUTPUT);
  pinMode(Short,OUTPUT);
  digitalWrite(Long,HIGH);
  digitalWrite(Short,HIGH);
  
  SerialUSB.begin(115200);
  delay(250);

  analogWriteResolution(analogWriteResolutionBits);
  analogReadResolution(analogReadResolutionBits);
#ifdef NEXTION
      nexInit();
      TopAxis.Set_grid_height_gdh(5*AMP);
      MiddleAxis.Set_grid_height_gdh(5*AMP);
      LowerAxis.Set_grid_height_gdh(5*AMP);
      versionNumber.setText(version);
      sendCommand("dims=0");
      sendCommand("dim=80");
#endif
  
  analogWrite(setCopperBlock, 255); // full cooling
  
  while (millis()-startTime < 10100)
  {
    // maybe let the cooling block reach a certain temp here, then continue....
#ifdef NEXTION
      startUp.setValue(((millis()-startTime)/100));
#endif
      delay(30);
  }
  parseCommands("xaxis=100");
  sendCommand("page 1");
  // PID stuff --------------------------------------
  for (int i = 0; i < 3; i++)  {
    tilesPID[i]->SetOutputLimits(0,255);
    tilesPID[i]->SetMode(MANUAL);
    tilesPID[i]->SetSampleTime(100);
    tilesPID[i]->SetMode(AUTOMATIC);
  }  
  CopperPID.SetMode(AUTOMATIC);
  CopperPID.SetSampleTime(1000);
  parseCommands("PID=100,10,1");
 // PID stuff --------------------------------------
}

// -------------------------------------------------------------------------------------------------------
// Loop
// -------------------------------------------------------------------------------------------------------
void loop() {
  serialParser();
  
  if (stringComplete) {
    parseCommands(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  for (int i = 0; i <= 2; i++) {
      tilesMeasuredTemp[i] = val2tempTile(analogRead(getTiles[i]));
  }
  copperBlockMeasuredTemp = val2tempCopperBlock(analogRead(getCopperBlock));
  if (counter++>100)
  {
      counter =0;
      if (round(copperBlockMeasuredTemp) != prevcopperBlockMeasuredTemp){
        sprintf(temp, "%i", round(copperBlockMeasuredTemp));
        blockTemp.setText(temp);
        prevcopperBlockMeasuredTemp = round(copperBlockMeasuredTemp);
  }
}
  
  
  while (millis() < nextPWMAdjust); 

  nextPWMAdjust = millis() + 10;

  if (debug){
    int t = millis();
    SerialUSB.print(t-pt); SerialUSB.print('\t'); 
    pt=t;
  }
//  int ledval = HIGH;
  
  for (int i=0; i<=2; i++)    {
    tilesPID[i]->Compute();
    analogWrite(setTiles[i], min(tilesPWM[i],255));
    
    if (BoostUntil[i]>millis()) 
    { 
      analogWrite(setBoost[i], 255);
  //    ledval = LOW;
    }
    else 
    {
      analogWrite(setBoost[i], 0);
    }
    
    
    if (debug){
      SerialUSB.print(val2tempTile(analogRead(getTiles[i])));      SerialUSB.print('\t');      SerialUSB.print(tilesPWM[i]/10.0);      SerialUSB.print('\t');      SerialUSB.print(tilesDesiredTemp[i]);      SerialUSB.print('\t');}
  }
//  digitalWrite(Long,ledval);

  if (debug) SerialUSB.println();
  CopperPID.Compute();
  analogWrite(setCopperBlock,copperBlockPWM);
  SamplerCB();
}

// -------------------------------------------------------------------------------------------------------
// CommandParser Part
// -------------------------------------------------------------------------------------------------------

#define Switch(STR)      String _x; _x=STR; if (false)
#define Case(STR)        } else if (_x==STR){
#define Default          } else {

void parseCommands(String Input) {
  String Command = getValue(Input, '=', 0);
  Input = getValue(Input, '=', 1);
  Command.toUpperCase();
  Switch (Command) {
    // -------------------------------------------------------------------------
    Case ("VERSION")
        SerialUSB.println(version);
    //break;
    // -------------------------------------------------------------------------
    Case ("DEBUG")
        debug=!debug;
        
    //break;
    // -------------------------------------------------------------------------
    Case ("SETTILETEMP")
    if (getNumberOfArguments(Input, ',') == 3)
    {
      for (int i = 0; i <= 2; i++) {
        tilesDesiredTemp[i] = Str2Int(getValue(Input, ',', i));
        float td = tilesDesiredTemp[i]-val2tempTile(analogRead(getTiles[i]));
        if (td>3)
          BoostUntil[i] = millis() + (BOOSTDURATION*td); 
      }
    }
    //break;
    // -------------------------------------------------------------------------
    Case ("YAXIS")
    if (getNumberOfArguments(Input, ',') == 1)
    {
      int val = Str2Int(getValue(Input, ',', 0));
      TopAxis.Set_grid_height_gdh(val*AMP);
      MiddleAxis.Set_grid_height_gdh(val*AMP);
      LowerAxis.Set_grid_height_gdh(val*AMP);
    }
    //break;
    // -------------------------------------------------------------------------
    Case ("XAXIS")
    if (getNumberOfArguments(Input, ',') == 1)
    {
      int val = Str2Int(getValue(Input, ',', 0));
      TopAxis.Set_grid_width_gdw(val/10);
      MiddleAxis.Set_grid_width_gdw(val/10);
      LowerAxis.Set_grid_width_gdw(val/10);
    }
    //break;
    // -------------------------------------------------------------------------

    Case ("SETCOPPERTEMP")
    if (getNumberOfArguments(Input, ',') == 1) {
      copperBlockDesiredTemp = Str2Int(Input);
    }
    //break;
    // -------------------------------------------------------------------------

    Case ("BOOST")
    if (getNumberOfArguments(Input, ',') == 3)
    {
      for (int i = 0; i < 3; i++)       {
        boolean Boost = Str2Int(getValue(Input, ',', i)) > 0;
        if (Boost) {
          BoostUntil[i] = millis() + BOOSTDURATION;
        } else {
          BoostUntil[i] = 0;
        }
      }
    }
    //break;
    // -------------------------------------------------------------------------

    Case ("PID")
    if (getNumberOfArguments(Input, ',') == 3)
    {
        Kp=Str2Int(getValue(Input, ',', 0));
        Ki=Str2Int(getValue(Input, ',', 1));
        Kd=Str2Int(getValue(Input, ',', 2));
        for (int i=0; i<=2; i++)    
          tilesPID[i]->SetTunings(Kp,Ki,Kd);
    }
    //break;
    // -------------------------------------------------------------------------

    Case ("STARTSAMPLING")
      startTime=millis();
      isSampling = true;
    //break;
    // -------------------------------------------------------------------------

    Case ("STOPSAMPLING")
    isSampling = false;
    //break;
    // -------------------------------------------------------------------------

    Case ("INITIALIZE")
    parseCommands("SetTileTemp=20,20,20");
    parseCommands("Boost=0,0,0");
    parseCommands("StopSampling");
    for (int c = 0 ; c < CHANNELS; c++)     {
      mSampledData[c] = 0;
    }
    isSampling = false;
    //break;
    // -------------------------------------------------------------------------
    
   //LEDs
    Case ("LEDS")
    if (getNumberOfArguments(Input, ',') == 2) {
      for (int i = 0; i < 2; i++)       {
        boolean val = Str2Int(getValue(Input, ',', i)) > 0;
        digitalWrite(Long+i,!val);
      }
    } 
    //break;
    // -------------------------------------------------------------------------

    
    Case ("MESSAGE")
      SerialUSB.println(Input);
    //break;
    // -------------------------------------------------------------------------
    
    Case ("AREYOUTHERE")
      SerialUSB.println("Yes I am.");
    //break;
    // -------------------------------------------------------------------------
    Default
      SerialUSB.println("Unknown Command Received.");
      SerialUSB.println(Command);
  }
}


int Str2Int(String stringValue) {
  // helper function to convert a sting into an integer.
  // no checking whatsowever.
  char charArg[stringValue.length() + 1];
  stringValue.toCharArray(charArg, stringValue.length() + 1);
  return atoi(charArg);
}

int getNumberOfArguments(String data, char separator) {
  // Returns the number of arguments that are given on a commandline.
  // Counts the number of seperators.
  if (data.length() == 0) return 0;
  int found = 0;
  int maxIndex = data.length() - 1;

  for (int i = 0; i < maxIndex; i++) {
    if (data.charAt(i) == separator) {
      found++;
    }
  }
  return found + 1;
}

String getValue(String data, char separator, int index) {
  // Get the indexed value from a string. Starting with 0.
  // used for parsing the command line.
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

boolean isValid(char in) {
  // which characters are valid in the commands that can be given.
  //
  boolean retval = false;
  if ((in >= 'a') && (in <= 'z')) retval = true;
  if ((in >= 'A') && (in <= 'Z')) retval = true;
  if ((in >= '0') && (in <= '9')) retval = true;
  if ((in == '.') || (in == ',') || (in == '=')) retval = true;
  return retval;
}

void serialParser() {
  // This is where the communication takes place.
  // Commands that should be known by the board are parsed here.
  // Illigal commands should be 'silently ignored'.
  // This subroutine is called whenever serial data are available to the firmware.
  // Typically this is the mac mini asking the arena to do something.
  //
  // When stringComplete == true, the full command is available. In loop() this flag
  // is cleared and the command is parsed (calling parseCommands)
  //
  while (SerialUSB.available()) {
    // get the new byte:
    char inChar = (char)SerialUSB.read();
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      break;
    }
    else
      // add it to the inputString:
      if (isValid(inChar))
        inputString += inChar;
  }
}

float val2tempTile(int val) {
  float temp = ((.66*-96.1) * val / (pow(2, analogReadResolutionBits) - 1)) + 78.0;
  return (temp);
}

float val2tempCopperBlock(int val) {
  float temp = ((.66*-86.136) * val / (pow(2, analogReadResolutionBits) - 1)) + 55.33;
  return (temp);
}

float val2tempCoolingBlock(int val) {
  float temp = ((.66*-100.0) * val / (pow(2, analogReadResolutionBits) - 1)) + 80.2;
  return (temp);
}


