///////////////////////////////////////////////////////////////////////////////////
//   G6LBQ Irwell HF Transceiver VFO - Version 1.0
//   stm32 + si5351a VFO With BFO & Conversion Oscilator
//   
//   (Based on 'JAN2KD 2016.10.19 Multi Band DDS VFO Ver3.1')     
//   Expanded with Multiple SI5351 & I/O Expanders by G6LBQ  
//
//   Created by G6LBQ on 15/09/2020 
//   Last updated on 08/04/2021
//                                                        
//   All Changes to code made by G6LBQ are commented      
///////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////
//  RGB colours used for the display - Added by G6LBQ
//
//  255-0-0     Red
//  0-0-0       Black
//  255-255-0   Yellow
//  255-255-255 White
//  0-255-0     Green
//  50-50-50    Dark Grey
//  100-100-100 Grey
//  235-0-200   Pink
//  0-255-255   Turquoise
//  0-0-255     Blue
///////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
//  Main LowPass Filters For HF Ham Bands - Added by G6LBQ
//
//  1) To be added at a later stage
//  2) To be added at a later stage
//  3) To be added at a later stage
//  4) To be added at a later stage
//  5) To be added at a later stage
//  6) To be added at a later stage
//  7) To be added at a later stage
//  8) To be added at a later stage
//  
///////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
//  Main Bandpasss Filters For HF Ham Bands - Added by G6LBQ
//
//  1) 0.15 MHz to 1.599999 MHz
//  2) 1.600001 MHz to 1.999999 MHz
//  3) 2.000001 MHz to 2.999999 MHz
//  4) 3.000001 MHz to 3.999999 MHz
//  5) 4.000001 MHz to 5.999999 MHz
//  6) 6.000001 MHz to 7.999999 MHz
//  7) 8.000001 MHz to 10.999999 MHz
//  8) 11.000001 MHz to 14.999999 MHz
//  9) 15.000001 MHz to 21.999999 MHz
//  10)22.000001 MHz to 29.999999 MHz
//
///////////////////////////////////////////////////////////////////////////////////

#define BLUEPILL    // uncomment to tweak some i/o ports for the blue pill, and enable Serial
#define MOCKI2C     // uncomment this to mock transmission to the 2x pcf8574 , Si5351s
#define TRACEI2C    // uncomment to generate debug traces on I2C outputs. BLUEPILL must be defined for Serial to work
#define DEBUG

//---------- Library include ----------

//#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

#include "si5351a2.h" 
#include "Rotary.h"                     
#include "src/Ucglib.h"
#include "ButtonEvents.h"

//----------   Encoder setting  ---------------

#define ENC_A     PB12                    // Rotary encoder A
#define ENC_B     PB13                    // Rotary encoder B

Rotary r=Rotary(ENC_A,ENC_B);

//----------   TFT setting  ------------------- 

#define   __CS    PB10                    // G6LBQ changed from PB10 to PB3 to free up PB10 for 2nd I2C bus    
#define   __DC    PB0                     // D/C
#define   __RST   PB1                     // RESET   

Ucglib_ILI9341_18x240x320_HWSPI ucg(__DC, __CS, __RST);

//----------  Button Setting -----------------

// define event names for the key events
typedef enum {EVT_NOCHANGE, EVT_STEPUP, EVT_STEPDOWN, EVT_BAND, EVT_RIT, EVT_FREQ_ADJ, EVT_MODE, EVT_BFO_ADJ} keyEvents;

ButtonEvents b = ButtonEvents(EVT_NOCHANGE);

//----------   CW Tone  ------------------- 

#define   CW_TONE     700                 // 700Hz


//----------   I/O Assign  ------------------- 

#define   OUT_LSB      PB15               // Data line for controlling modes of operation                  
#define   OUT_USB      PA8                // Data line for controlling modes of operation                  
#define   OUT_CW       PA9                // G6LBQ added extra mode selection output                  
#define   OUT_AM       PA10               // G6LBQ added extra mode selection output
#define   SW_BAND      PA0                 
#define   SW_MODE      PC14                 
#define   SW_STEP      PA1                // G6LBQ changed from PB14 to PA1                 
#define   SW_RIT       PC15

#ifdef BLUEPILL
#define   LED          PC13
#define   SW_TX        PA3                // need to reassign to move away from led
#else                  
#define   SW_TX        PC13               // G6LBQ> PTT - connect to Gnd for TX
#endif

#define   METER        PA2                // G6LBQ changed from PA1 to PA2    

//----------   eeprom addresses, constants    ---------------
#define   EEP_BAND     0x00               // EEPROM BAND Address
#define   EEP_XTAL     0x08               // Xtal Address
#define   EEP_INIT     0x0e               // Address of eeprom version-id
#define   EEP_IFSHIFT  0xB0               // Address of the IFSHIFT table (4x longs - 1 each for LSB, USB, CW, AM

#define   EEP_BANDMAX  10                 // (Was 6 band) G6LBQ 
#define   EEP_VERSION  74                 // Update the version no. when the eeprom format changes

//---------  Si5351 Assignments ---------------------------
#define   VFO_PORT     0
#define   VFO_CHL      0
#define   BFO_PORT     1
#define   BFO_CHL      2
#define   CONV_PORT    2
#define   CONV_CHL     1

//---------- Modes ---------------------

typedef enum {MODE_LSB, MODE_USB, MODE_CW, MODE_AM} modes;

//---------- Variable setting ----------

long      freq    = 7100000;
long      freqmax = 7200000;
long      freqmin = 7000000;
long      freqold = 0;
long      freqrit = 0;

long      ifshift = 0;
long      ifshiftb;
long      romb[5];                        // EEPROM bfo copy buffer
long      vfofreq = 0;
long      vfofreqb;                 


int       rit        = 0;
uint16    fstep      = 2;     // index into the fstepRates table
static const long fstepRates[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
#define fstepRatesSize (sizeof(fstepRates)/sizeof(fstepRates[0]))
uint16    steprom    = 1;
uint16    fmode      = MODE_AM;
uint16    fmodeb     = MODE_AM;
int       flagrit    = 0;
int       fritold    = 0;
int       flagmode   = 0;
int       meterval1  = 0;
int       tmeterval  = 0;
int       romadd     = 0;
int       rombadd    = 0;
int       analogdata = 0;
uint16    band       = 0;                   // 3.5MHz

uint16    Status;
uint16    Data;            

//unsigned long eep_xtalfreq;
unsigned long eep_bfo[6];

int_fast32_t timepassed;                    // int to hold the arduino miilis since startup
int       flg_frqwt = 0;                    // Frequency data Wite Flag(EEPROM)
int       flg_bfowt = 0;                    // BFO Wite Flag(EEPROM)
int       flg_bfochg = 0;                   // BFO Wite Flag(EEPROM)
int       flg_freqadj = 0;                  // Frequency ADJ Flag

//uint32_t xtalFreq = XTAL_FREQ;
long      freqb = 0;


long interruptCount = 0;
long lastInterruptCount = -1;


//------------ 1st IF Frequency 45MHz For Dual Conversion IF -----------

unsigned long firstIF =   45000000L;        // Added by G6LBQ 01/07/2022


//------------------  Initialization  Program  -------------------------
 
void setup() {
#ifdef BLUEPILL
  while (!Serial)
    ;
  Serial.println("Starting setup");
  pinMode(LED, OUTPUT);
#endif
  
  timepassed = millis();

  //afio_cfg_debug_ports(AFIO_DEBUG_NONE);          // ST-LINK(PB3,PB4,PA15,PA12,PA11) Can be used
  
  pinMode( ENC_A, INPUT_PULLUP);                  // PC13 pull up
  pinMode( ENC_B, INPUT_PULLUP);                  // PC14
  attachInterrupt( ENC_A, Rotary_enc, CHANGE);    // Encoder A
  attachInterrupt( ENC_B, Rotary_enc, CHANGE);    //         B

  delay(100);
  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.clearScreen();
  ucg.setRotate270();

  b.add(SW_BAND, EVT_BAND, EVT_NOCHANGE);
  b.add(SW_MODE, EVT_MODE, EVT_BFO_ADJ);
  b.add(SW_STEP, EVT_STEPUP, EVT_STEPDOWN);
  b.add(SW_RIT, EVT_RIT, EVT_FREQ_ADJ);
  
  pinMode(SW_TX,INPUT_PULLUP);
  pinMode(OUT_LSB,OUTPUT);                    // LSB Mode 
  pinMode(OUT_USB,OUTPUT);                    // USB Mode
  pinMode(OUT_CW,OUTPUT);                     // CW Mode - G6LBQ added additional mode selection
  pinMode(OUT_AM,OUTPUT);                     // AM Mode - G6LBQ added additional mode selection
  
  init_PCF8574();

  Fnc_eepINIT();
  delay(100);
  band2eep();
  delay(100);

  xtalFreq = Fnc_eepRD(EEP_XTAL);
  Status = EEPROM.read(EEP_BAND,&band);          // EEPROM read(frequency)
  romadd=0x010+(band*0x10);
  freq =    Fnc_eepRD(romadd+ 0);
  freqmin = Fnc_eepRD(romadd+ 4);
  freqmax = Fnc_eepRD(romadd+ 8);
  Status = EEPROM.read(romadd+12,&fmode);
  Status = EEPROM.read(romadd+14,&fstep);

  int eep_rombadd=EEP_IFSHIFT;                             // EEPROM read(BFO)
  for (int i=0; i<4;i++){
    romb[i]=Fnc_eepRD((eep_rombadd+(4*i)));
    eep_bfo[i] = romb[i];    
  }

  banddataout();
  drawStaticScreen();
  chlcd();

  modeset();
  steplcd();
  freqlcd(freq);
}

//----------  Main program  ------------------------------------

 
void loop() {
#ifdef BLUEPILL
  digitalWrite(LED, !digitalRead(LED));
#endif

#ifdef DEBUG
  if (interruptCount != lastInterruptCount) {
    Serial.print("Interrupts: "); Serial.println(interruptCount);
    lastInterruptCount = interruptCount;
  }
#endif

  select_BPF(freq);
  
  int event = b.getButtonEvent();
  switch (event) {
    case EVT_NOCHANGE:
      break; // nothing to do
    case EVT_STEPUP:
      setstep();
      break;
    case EVT_STEPDOWN:
      setstepDown();
      break;
    case EVT_BAND:
      bandcall();
      break;
    case EVT_RIT:
      setrit();
      break;
    case EVT_FREQ_ADJ:
      freqAdjust();
      break;
    case EVT_MODE:
      modesw();
      break;
    case EVT_BFO_ADJ:
      bfoAdjust();
      break;
  }

  if (digitalRead(SW_TX)==LOW)                  // TX sw check
    txset();

  if (flagrit==1){
    if (freqrit == fritold){
      meter();
    }    
    if (freqrit!=fritold){
      PLL_write();
      ritlcd();
      fritold=freqrit;  
    }
  } else{
    if(flg_freqadj == 0){
      if (freq == freqold){
        meter();
      }
      PLL_write();
      freqlcd(freq);
      freqold=freq;
  } else{
      if(freq != freqb){
        xtalFreq = freq;
        setFrequency(VFO_PORT, VFO_CHL, 10000000);           // 01/07/2022 Updated to allow VFO calibration to work with new si5351 Library
        freqlcd(freq);
        freqb = freq;
      }
    }
  }

  if((flg_frqwt == 1) && (flg_bfochg == 0)){           // EEPROM auto save 2sec 
    if(timepassed+10000 < millis()){
#ifdef BLUEPILL
      Serial.println("EEPROM auto save");
#else
      bandwrite();  // don't save during testing - otherwise it will wear out the flash
#endif
      flg_frqwt = 0;
    } 
  }
}

//--------- PCF8574 Interfacing ----------------------------------

void write_PCF8574(uint8_t address, uint8_t data) {
#ifndef MOCKI2C
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0b00000000);
  Wire.endTransmission();
#endif
#ifdef TRACEI2C
  Serial.print("Write to "); Serial.print(address); Serial.print(": "); Serial.println(data);
#endif
}

void init_PCF8574() {
  write_PCF8574(0x38, 0b00000000);  //initialize PCF8574 I/O expander 1 for BPF filters 1 to 5 
  write_PCF8574(0x39, 0b00000000);  //initialize PCF8574 I/O expander 2 for BPF filters 6 to 10
}

//--------- Filter Bank Selection ---------------------------------

void select_bank(int8_t bank) {
  static int8_t currentBank = -1;
  // bank 0 corresponds to 0x38, port 1, bank 6 is 0x39, port 1 etc.
  if (bank == currentBank)
    // the correct bank is already selected; nothing to do here
    return;
  init_PCF8574();   // turn off all banks
  int bit = bank % 6;
  int bankOffset = bank/6;
  write_PCF8574(0x38 + bankOffset, 1<<bit);
  currentBank = bank;
}

void select_BPF(long f) {

  // HF Band Pass Filter logic added by G6LBQ
  if (freq >=150000 && freq<=1599999){
    select_bank(0);
  } else if(freq >=1600001 && freq<=1999999){
    select_bank(1);
  } else if(freq >=2000001 && freq<=2999999){
    select_bank(2);
  } else if(freq >=3000001 && freq<=3999999){
    select_bank(3);
  } else if(freq >=4000001 && freq<=5999999){
    select_bank(4);
  } else if (freq >=6000001 && freq<=7999999){
    select_bank(5);
  } else if(freq >=8000001 && freq<=10999999){
    select_bank(6);
  } else if(freq >=11000001 && freq<=14999999){
    select_bank(7);
  } else if(freq >=15000001 && freq<=21999999){
    select_bank(8);
  } else if(freq >=22000001 && freq<=29999999){
    select_bank(9);
  } else {
    
  }
}

//----------  EEPROM Data initialization  ---------------        

void band2eep(){
  Status = EEPROM.read(EEP_INIT,&Data);
  if(Data != EEP_VERSION){                       // Iinitialization check
    Status = EEPROM.write(EEP_BAND,1);

    xtalFreq = XTAL_FREQ;
    Fnc_eepWT(xtalFreq,EEP_XTAL);
    
    band2write(0x010,  1800000,  1800000,  2000000, MODE_LSB, 2);
    band2write(0x020,  3500000,  3500000,  3800000, MODE_LSB, 2);
    band2write(0x030,  7000000,  7000000,  7200000, MODE_LSB, 2);
    band2write(0x040, 14000000, 14000000, 14350000, MODE_USB, 2);
    band2write(0x050, 18000000, 18000000, 18200000, MODE_USB, 2);
    band2write(0x060, 21000000, 21000000, 24990000, MODE_USB, 2);
    band2write(0x070, 28000000, 28000000, 29700000, MODE_USB, 2);
    band2write(0x080,   500000,   500000, 30000000, MODE_AM, 2);
	
	  // Andy - repeat this pattern for your 2x additional filters
    band2write(0x090,   500000,   500000, 30000000, MODE_AM, 2);
    band2write(0x0a0,   500000,   500000, 30000000, MODE_AM, 2);
    
    eep_bfo[0]=11056570;                 // LSB BFO Frequency
    eep_bfo[1]=11059840;                 // USB BFO Frequency
    eep_bfo[2]=11058400;                 // CW  BFO Frequency
//  eep_bfo[3]=11058200;                 // AM Not needed for testing only

    int eep_rombadd=EEP_IFSHIFT;             // BFO ROMadd:0x090    
    for (int i=0;i<4;i++){
      Fnc_eepWT(eep_bfo[i],(eep_rombadd+4*i));
    }

    Status = EEPROM.write(EEP_INIT, EEP_VERSION); // Initialized End code
  }
}

//----------  Function Band Write to EEPROM  ---------------        

void band2write(int eep_romadd, unsigned long freq, unsigned long freqmin, unsigned long freqmax, int eep_fmode, int eep_fstep){
  Fnc_eepWT(freq,    eep_romadd+0);
  Fnc_eepWT(freqmin, eep_romadd+4);
  Fnc_eepWT(freqmax, eep_romadd+8);
  Status = EEPROM.write(eep_romadd+12,eep_fmode);
  Status = EEPROM.write(eep_romadd+14,eep_fstep);
}

//---------- PLL write ---------------------------
//
// Original code was for single conversion IF at 11.059MHz so 11.059MHz + VFO Frequency
// 01/07/2022 Changes are for dual conversion so 45MHz firstIF + VFO Frequency 

void PLL_write(){
  if(flg_bfochg == 0){
    if (flagrit==0)
      vfofreq=freq+firstIF;               // G6LBQ 01/07/2022 changed from vfofreq=freq+ifshift; to vfofreq=freq+firstIF;
    else
      vfofreq=freq+firstIF+freqrit;       // G6LBQ 01/07/2022 changed from vfofreq=freq+ifshift+freqrit; to vfofreq=freq+firstIF+freqrit;

    Vfo_out(vfofreq);                     // VFO output
    Bfo_out(ifshift);                     // BFO
  }
  else{
    ifshift = freq;
    Bfo_out(ifshift);                     // BFO
    freq = ifshift;
  }
}

// -----     Routines to interface to the Si5351s-----

long currentFrequency[] = { -1, -1, -1 };   // track output frequences for changes

void _setFrequency(uint8_t port, uint8_t channel, uint32_t frequency) {
#ifndef MOCKI2C
  si5351aSetFrequency(port, channel, frequency);
#endif
#ifdef TRACEI2C
  Serial.print("Si5351 port: "); Serial.print(port); Serial.print(", chl: "); Serial.print(channel); Serial.print(", freq: "); Serial.println(frequency);
#endif
}

void setFrequency(uint8_t port, uint8_t channel, uint32_t frequency) {
  // don't need to track port and channel; port is unique
  if (currentFrequency[port] == frequency)
    // it's already set to the correct frequency; nothing further to do here
    return;
  _setFrequency(port, channel, frequency);
  currentFrequency[port] = frequency;
}


//----------  VFO out  --------------- 

void Vfo_out(long frequency){
  if(vfofreq != vfofreqb){
    setFrequency(VFO_PORT, VFO_CHL, frequency);         // 01/07/2022 Updated to allow VFO to work with new si5351 Library
    flg_frqwt = 1;                                // EEP Wite Flag
    timepassed = millis();
    vfofreqb = vfofreq;  
  }
}

//----------  BFO out  ---------------        

void Bfo_out(long frequency){
  if(ifshift != ifshiftb){
    setFrequency(BFO_PORT, BFO_CHL, frequency);         // 01/07/2022 Updated to allow BFO to work with new si5351 Library
    flg_bfowt = 1;                                // EEP Wite Flag
    ifshiftb = ifshift;  
  }
}

//---------- meter --------------------------

void meter(){
 meterval1=analogRead(METER);
// meterval1=meterval1/50;                   // old 
 meterval1=meterval1/200;                  
 if (meterval1>15){meterval1=15;}
  int sx1=sx1+(meterval1*17);
  sx1=sx1+41;
  int sx2=0;
  sx2=sx2+(40+((15-meterval1)*17));
  ucg.setFont(ucg_font_fub35_tr);
  ucg.setColor(0,0,0);
  ucg.drawBox(sx1,180,sx2,16);
  ucg.setPrintPos(40,200);
  for(int i=1;i<=meterval1;i++){
    if (i<=9){
      ucg.setColor(0,255,255);
      ucg.print("-");  
    }
    else{
      ucg.setColor(255,0,0);
      ucg.print("-");
    }
  }
}

//---------- Encoder Interrupt -----------------------
void Rotary_encHide(){  // investigate very high interupt rate
  static long freq = 0;
  interruptCount++;
  unsigned char result = r.process();
  if (result == DIR_CW) {
    freq++;
  } else if (result == DIR_CCW) {
    freq--;
  }
}
void Rotary_enc(){
  interruptCount++;
  if (flagrit==1){
    unsigned char result = r.process();
    if(result) {
      if(result == DIR_CW){  
        freqrit=freqrit+fstepRates[fstep];
        if (freqrit>=10000){
          freqrit=10000;
        }
     }
     else{
      freqrit=freqrit-fstepRates[fstep];
      if (freqrit<=-10000){
        freqrit=-10000;
      }
    }
   }
  }

  else{
    unsigned char result = r.process();
    if(result) {
      if(result == DIR_CW){  
        freq=freq+fstepRates[fstep];
        if((flg_bfochg == 0) && (flg_freqadj == 0) && (freq>=freqmax)){freq=freqmax;}
      }
      else{
        freq=freq-fstepRates[fstep];
        if((flg_bfochg == 0) && (flg_freqadj == 0) && (freq<=freqmin)){freq=freqmin;}
      }  
    }
  }
}

//------------ On Air -----------------------------

void txset(){
  if(fmode == MODE_CW)                        // CW?
    Vfo_out(vfofreq + CW_TONE);               // Vfofreq+700Hz
  else
    Vfo_out(vfofreq);                         // vfo out
    

  ucg.setPrintPos(110,140);
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setColor(255,0,0);
  ucg.print("ON AIR");
  while(digitalRead(SW_TX) == LOW){
    meter();
  }

  ucg.setFont(ucg_font_fub17_tr);
  ucg.setColor(0,0,0);
  ucg.drawBox(100,120,250,30);  //45
}

//------------- Mode set(LSB-USB-CW-AM) ------------

void setHighLightText(int x, int y, bool highlight, String s) {
  ucg.setPrintPos(x, y);
  if (highlight) {
    ucg.setColor(255,255,0);
  } else {
    ucg.setColor(0,0,0);
  }
  ucg.print(s);
}

void modeset(){
  ucg.setFont(ucg_font_fub17_tr);
  setHighLightText(82,  82, fmode==MODE_USB, "USB");
  setHighLightText(12,  82, fmode==MODE_LSB, "LSB");
  setHighLightText(82, 112, fmode==MODE_AM,  "A M");
  setHighLightText(12, 112, fmode==MODE_CW,  "C W");

  switch(fmode){
    case MODE_LSB:
      ifshift = eep_bfo[0];
      setFrequency(CONV_PORT, CONV_CHL,  56059000);
      break;
    case MODE_USB:                                      
      ifshift = eep_bfo[1];
      setFrequency(CONV_PORT, CONV_CHL,  33941000);
      break;
    case MODE_CW:
      ifshift = eep_bfo[2];
      setFrequency(CONV_PORT, CONV_CHL,  56059000);
      break;
    case MODE_AM:
      setFrequency(CONV_PORT, CONV_CHL,  56059000);
      break;
  }

  digitalWrite(OUT_LSB, fmode==MODE_LSB);
  digitalWrite(OUT_USB, fmode==MODE_USB);
  digitalWrite(OUT_CW,  fmode==MODE_CW);       // G6LBQ added 1/11/20
  digitalWrite(OUT_AM,  fmode==MODE_AM);       // G6LBQ added 1/11/20
}

//------------- Mode set SW ------------

void modesw() {
  fmode = (fmode + 1) % 4;  // wrap around
  modeset();
  PLL_write();
}

//------------- BFO Adjust ------------

void bfoAdjust() {
  if (flg_bfochg) {
    // exit BFO ADJ mode
    ifshift = freq;
    Fnc_eepWT(ifshift,EEP_IFSHIFT+(fmode * 4));     // data write
    eep_bfo[fmode] = ifshift;
    romadd=0x010+(band*0x10);
    freq = Fnc_eepRD(romadd);             // restore freq to last freq tuned to in this band
    freqlcd(freq);  
    ucg.setFont(ucg_font_fub17_tr);
    ucg.setColor(0,0,0);
    ucg.drawBox(100,120,250,30);  //45
  } else {
    // enter BFO ADJ mode
    freq = Fnc_eepRD(0x090+(fmode * 4));
    freqlcd(freq);  
    ucg.setPrintPos(110,140);
    ucg.setFont(ucg_font_fub17_tr);
    ucg.setColor(255,255,0);
    ucg.print("BFO ADJ");
    fmodeb = fmode;
  }
  flg_bfochg = !flg_bfochg;

  modeset();  // needed?
  PLL_write();
}

//------------ Rit SET ------------------------------

void setrit() {
  if (flagrit) {
    vfofreq=freq+ifshift;

    Vfo_out(vfofreq);                       // VFO Out

    ucg.setFont(ucg_font_fub11_tr);
    ucg.setPrintPos(190,110);
    ucg.setColor(255,255,255);
    ucg.print("RIT");
    ucg.setColor(0,0,0);  
    ucg.drawRBox(222,92,91,21,3);
    freqrit=0;
  } else {
    ucg.setFont(ucg_font_fub11_tr);
    ucg.setPrintPos(190,110);
    ucg.setColor(255,0,0);
    ucg.print("RIT");
    ritlcd();    
  }
  flagrit = !flagrit;
}

//----------- Rit screen ----------------------

void ritlcd(){
  ucg.setColor(0,0,0);  
  ucg.drawBox(222,92,91,21);
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setColor(255,255,255); 
  ucg.setPrintPos(230,110);
  ucg.print(freqrit);
}

// --- Frequency Adjust -----------

void freqAdjust() {
  if(flg_freqadj) {
    // leave FREQ ADJ mode
    xtalFreq = freq;
    Fnc_eepWT(xtalFreq,EEP_XTAL);             // data write
    
    romadd=0x010+(band*0x10);
    freq = Fnc_eepRD(romadd);     // restore freq to last freq tuned to in this band
    freqlcd(freq);  
    ucg.setFont(ucg_font_fub17_tr);
    ucg.setColor(0,0,0);
    ucg.drawBox(100,120,250,30);              //45
    flg_freqadj = 0;
  } else {
    // enter FREQ ADJ mode
    freq = xtalFreq;
    setFrequency(VFO_PORT, VFO_CHL, 10000000);      // 01/07/2022 Updated to allow Rit to work with new si5351 Library 
    freqlcd(freq);  
    ucg.setPrintPos(110,140);
    ucg.setFont(ucg_font_fub17_tr);
    ucg.setColor(255,255,0);
    ucg.print("FREQ ADJ");
    flg_freqadj = 1;
    vfofreqb = 0;    
  }
}

//-------------- encoder frequency step set -----------

void setstep(){
  fstep = (fstep + 1) % fstepRatesSize;
  steplcd(); 
}

void setstepDown() {
  if (fstep==0) fstep = fstepRatesSize;
  fstep--;
  steplcd();
}

//------------- Step Screen ---------------------------
static const String stepText[] = {"     1Hz", "    10Hz", "   100Hz", "    1KHz", "   10KHz", " 100KHz", "    1MHz"};

void steplcd(){
  ucg.setColor(0,0,0);
  ucg.drawRBox(221,61,93,23,3);
  ucg.setFont(ucg_font_fub17_tr);
  ucg.setColor(255,255,255);
  ucg.setPrintPos(220,80);
  ucg.print(stepText[fstep]);
}

//----------- Main frequency screen -------------------

void freqlcd(long freq){
  static char f100m,f10m,fmega,f100k,f10k,f1k,f100,f10,f1 = -1;
  String freqt = String(freq);
  ucg.setFont(ucg_font_fub35_tn); 
  int mojisuu=(freqt.length());
  if(freq<100){
    ucg.setColor(0,0,0);
    ucg.drawBox(217,9,28,36);     
  }
  if(f10 !=(freqt.charAt(mojisuu-2))){
    ucg.setColor(0,0,0);
    ucg.drawBox(245,9,28,36);
    ucg.setPrintPos(245,45);
    ucg.setColor(0,255,0);
    ucg.print(freqt.charAt(mojisuu-2)); 
    f10 = (freqt.charAt(mojisuu-2));
  }
  if(freq<10){
    ucg.setColor(0,0,0);
    ucg.drawBox(245,9,28,36);    
     }
  if(f1 !=(freqt.charAt(mojisuu-1))){
    ucg.setColor(0,0,0);
    ucg.drawBox(273,9,28,36);
    ucg.setPrintPos(273,45);
    ucg.setColor(0,255,0);
    ucg.print(freqt.charAt(mojisuu-1));     
    f1  = (freqt.charAt(mojisuu-1));   
  }
  if(freq<1000){
    ucg.setColor(0,0,0);
    ucg.drawBox(202,9,15,36);        
    }
  if(f100 !=(freqt.charAt(mojisuu-3))){
    ucg.setColor(0,0,0);
    ucg.drawBox(217,9,28,36);
    ucg.setPrintPos(217,45);
    ucg.setColor(0,255,0);
    ucg.print(freqt.charAt(mojisuu-3)); 
    f100 = (freqt.charAt(mojisuu-3));
  }
  if(freq>=1000){
    ucg.setPrintPos(202,45);
    ucg.setColor(0,255,0);
    ucg.print(".");
  }
  if(freq<10000){
    ucg.setColor(0,0,0);
    ucg.drawBox(146,9,28,36);     
    }
  if(f1k !=(freqt.charAt(mojisuu-4))){
    ucg.setColor(0,0,0);
    ucg.drawBox(174,9,28,36);
    ucg.setPrintPos(174,45);
    ucg.setColor(0,255,0);
    ucg.print(freqt.charAt(mojisuu-4));       
    f1k  = (freqt.charAt(mojisuu-4));
  }
  if(freq<100000){
    ucg.setColor(0,0,0);
    ucg.drawBox(118,9,28,36); 
  }
  if(f10k !=(freqt.charAt(mojisuu-5))){
    ucg.setColor(0,0,0);
    ucg.drawBox(146,9,28,36);
    ucg.setPrintPos(146,45);
    ucg.setColor(0,255,0);
    ucg.print(freqt.charAt(mojisuu-5));   
    f10k = (freqt.charAt(mojisuu-5));
  }
   if(freq<1000000){
    ucg.setColor(0,0,0);
    ucg.drawBox(103,9,15,36); 
    }
  if(f100k !=(freqt.charAt(mojisuu-6))){
    ucg.setColor(0,0,0);
    ucg.drawBox(118,9,28,36);
    ucg.setPrintPos(118,45);
    ucg.setColor(0,255,0);
    ucg.print(freqt.charAt(mojisuu-6));   
    f100k = (freqt.charAt(mojisuu-6));
  }
       
   if(freq>=1000000){
    ucg.setPrintPos(103,45);
    ucg.setColor(0,255,0);
    ucg.print(".");
  }
   if(freq<10000000){
     ucg.setColor(0,0,0);
    ucg.drawBox(47,9,28,36);
     }
   if(fmega !=(freqt.charAt(mojisuu-7))){
    ucg.setColor(0,0,0);
    ucg.drawBox(75,9,28,36);
    ucg .setPrintPos(75,45);
    ucg.setColor(0,255,0);
    ucg.print(freqt.charAt(mojisuu-7));   
    fmega  = (freqt.charAt(mojisuu-7));
   }
   if(freq<100000000){
    ucg.setColor(0,0,0);
    ucg.drawBox(19,9,28,36);
       }
   if (f10m !=(freqt.charAt(mojisuu-8))){
    ucg.setColor(0,0,0);
    ucg.drawBox(47,9,28,36);
    ucg .setPrintPos(47,45);
    ucg.setColor(0,255,0);
    ucg.print(freqt.charAt(mojisuu-8));
    f10m = (freqt.charAt(mojisuu-8));
   }
}
//----------- Basic Screen -------------------------

void drawStaticScreen(){
  ucg.setColor(255,255,255);
  ucg.drawRFrame(1,1,314,55,5);
  ucg.drawRFrame(2,2,312,53,5);
  ucg.setColor(0,0,255);      //G6LBQ changed Grey To Blue
  ucg.drawRBox(5,60,60,25,3);
  ucg.drawRBox(75,60,60,25,3);
  ucg.drawRBox(5,90,60,25,3);
  ucg.drawRBox(75,90,60,25,3);
  ucg.drawRFrame(220,60,95,25,3);
  ucg.drawRFrame(220,90,95,25,3);
  ucg.setFont(ucg_font_fub11_tr);
  ucg.setColor(255,255,255);
  ucg.setPrintPos(175,80);
  ucg.print("STEP");
  ucg.setPrintPos(190,110);
  ucg.setColor(255,255,255);
  ucg.print("RIT");
  ucg.setColor(170,170,170);      //G6LBQ changed from 100,100,100
  ucg.setPrintPos(10,210);
  ucg.print(" S:  1-----3-------6-------9---Over--------");
  ucg.setPrintPos(10,177);
  ucg.print(" P:  1-----3-----5-----------10--------------");  
  ucg.setPrintPos(10,230);
  ucg.setColor(235,0,200);
  ucg.print( "        G6LBQ Irwell HF Transceiver      "); 
  //freqlcd(0);   
}

//---------- Band data call from eeprom ----------

void bandcall(){
  band=band+1;
  if (band>(EEP_BANDMAX-1)){band=0;}
  romadd=0x010+(band*0x010);
  freq    = Fnc_eepRD(romadd+ 0);
  freqmin = Fnc_eepRD(romadd+ 4);
  freqmax = Fnc_eepRD(romadd+ 8);
  Status = EEPROM.read(romadd+12,&fmode);
  Status = EEPROM.read(romadd+14,&fstep);

  modeset();
  steplcd();
  freqlcd(freq);  
  banddataout();
  chlcd();
}

//---------- Band data write to eeprom ----------

void bandwrite(){
  romadd=0x010+(band*0x010);
    Fnc_eepWT(freq,romadd);
  Status = EEPROM.write(EEP_BAND,band);
  Status = EEPROM.write(romadd+12,fmode);
  Status = EEPROM.write(romadd+14,fstep);
}

//----------  Function EEPROM Initialize  ---------

void Fnc_eepINIT(){
  uint16 dummy;
  //Refer to ...\arduino-1.8.13\portable\packages\stm32duino\hardware\STM32F1\2021.5.31\libraries\EEPROM\EEPROM.cpp, EEPROM.h 
  //Serial.print("EEPROM_PAGE0_BASE = "); Serial.print(EEPROM_PAGE0_BASE, HEX); Serial.print(" "); Serial.println(EEPROM_PAGE0_BASE == 0x801F000);
  //Serial.print("EEPROM_PAGE1_BASE = "); Serial.print(EEPROM_PAGE1_BASE, HEX); Serial.print(" "); Serial.println(EEPROM_PAGE1_BASE == 0x801F800);
  //Serial.print("EEPROM_PAGE_SIZE = ");  Serial.print(EEPROM_PAGE_SIZE,  HEX); Serial.print(" "); Serial.println(EEPROM_PAGE_SIZE  == 0x400);
  //long addr = (long)&bandwrite;
  //Serial.print("Addr = "); Serial.println(addr, HEX);
  EEPROM.PageBase0 = 0x801F000;         // 0x801F800 default values. So these settings move the "eeprom" storage down a bit. Why? Not for the bootloader - it's at the start of memory
  EEPROM.PageBase1 = 0x801F800;         // 0x801FC00 
  EEPROM.PageSize  = 0x400;             // 2kB
  dummy = EEPROM.init();
}

//----------  Function EEPROM Read(4byte)  ---------

long Fnc_eepRD(uint16 adr){
  long val = 0;
  uint16 dat,dummy;  

  dummy = EEPROM.read(adr,&dat);
  val = dat << 16;
  dummy = EEPROM.read(adr+1,&dat);
  return val | dat;
}

//----------  Function EEPROM Write(4byte)  ---------

void Fnc_eepWT(long dat,uint16 adr){
  uint16 dummy,val;

  val = dat & 0xffff;
  dummy = EEPROM.write(adr+1,val);
  val = dat >> 16;
  val = val & 0xffff;
  dummy = EEPROM.write(adr,val);
}

//---------- Band data output I/O ----------

void banddataout(){

}

//----------  Band No.(Chanel No.) write to LCD ----------

void chlcd(){
  ucg.setColor(0,0,0);
  ucg.drawBox(5,120,80,20);
  ucg.setFont(ucg_font_fub11_tr);
  ucg.setPrintPos(12,137);
  ucg.setColor(255,255,255);
  ucg.print("Band: ");              //G6LBQ changed wording from CH to Band
  ucg.print(band+1); 
}
  
