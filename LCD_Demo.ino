/*
   Tested on Arduino IDE 1.8.1  + Arduino_STM32.
   Blue Pill clones.
*/

char myInfo[] =
  "\n"
  "/------------------------------------\\\n"
  "| LCD/KBD Smart Serial Module V1.3    |\n"
  "| By Alfa Robotika 2017               |\n"
  "| http://www.alfa-robotika.rs         |\n"
  "| Type 'h' for help and press return  |\n"
  "\\------------------------------------/\n"
  ;

char myHelp[] =
  "\n"
  "/------------------------------------\\\n"
  "| LCD/KBD Smart Serial Module V1.3    |\n"
  "\\------------------------------------/\n"
  "Command overview \n"
  "h H          - Help\n"
  "t            - Test display\n"
  "e            - Toggle Edit mode\n"
  "r            - Toggle Rotate mode\n"
  "b            - Toggle BL Blink mode\n"
  "f[xx]        - Focus Item xx on screen\n"
  "p[xx]        - Print bargraph xx value on screen\n"
  "d            - Dump vars\n"
  "D            - Dump vars in C/C++ format\n"
  "0            - Machine Status = STOP\n"
  "1            - Machine Status = RUN\n"
  "2            - Machine Status = PAUSE\n"
  "3            - Machine Status = ERROR\n"
  "4            - Machine Status = SERVC\n"
  "s[xx]=[zzzz] - Set negative value zzzz @ reg xx\n"
  "S[xx]=[zzzz] - Set positive value zzzz @ reg xx\n"
  "m[sss...]    - Print message on 1st line\n"
  "M[sss...]    - Print message on 2nd line\n"
  "o[xxx]       - Set OUTs status for 1st line 0-255\n"
  ;

#include <LiquidCrystal.h>
//#include <HardWire.h>
//HardWire HWire(2, I2C_FAST_MODE); // I2c2

#define MySer Serial2 //koji seriski port koristimo za komunikaciju (Serial=USB, Serial1=HWport2, Serial2=HWport3)

//for debounce
int PortA, PortB;
int de_ms;
bool buttonState[8];       // the current reading from the input pin
bool lastButtonState[8];   // the previous reading from the input pin
bool de_state = false;     //temp var

// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime[8];  // the last time the output pin was toggled
unsigned long debounceDelay = 20;   // the debounce time; increase if the output flickers

//for serial
char inputString[32] = "";
bool stringComplete = false;  // whether the string is complete

//button states
boolean state_k_up =    1;
boolean state_k_down =  1;
boolean state_k_left =  1;
boolean state_k_right = 1;
boolean state_k_enter = 1;
boolean state_k_func =  1;
boolean state_edit_mode = 0;
boolean state_rotate_mode = 0;

long long_click_rate = 25; //Key Repeat (ms)
long long_click_delay = 300; //Delay Until Repeat (loop count)
unsigned long longclick_ts[8]; //for longclick
unsigned long longclick_flag[8]; //for longclick
int home_outs_status, home_outs_status_prev;

bool IS_INITIALISED = 0; //take care of this, means LCD is nost synced with master MCU

//KBD Definition (pull-up, switch to gnd)
#define K_UP    PA11
#define K_DOWN  PA12
#define K_LEFT  PA15
#define K_RIGHT PB5
#define K_ALT   PB7
#define K_ENTER PB8
#define K_FUNC  PB9

// HW definitions
#define LED     PC13  // Board LED
#define LCD_RS  PB13
#define LCD_E   PB14
#define LCD_D4  PB12
#define LCD_D5  PB15
#define LCD_D6  PA9
#define LCD_D7  PA10
#define LCD_LED PB6     // LCD backlight led  (PWM generated)
#define LCD_V0  PA8     // LCD contrast voltage (PWM generated)

//-------------------------------------------------------------------
//Struktura za dodatni properties instanci
typedef struct {
  uint8_t type;   //0=Integer, 1=decimal, 2=percent, 3=graphic scale
  uint8_t hidden;   //Don't show value
  uint8_t editable; //Is this item editable or read-only
  uint8_t action;   //Is this item action (send value only on click)
  uint8_t internal; //0=no, 1=loopCount 2=Contrast, 3=Backlight, ...???
  uint8_t len;    //dec len (0-16 digits)
  uint8_t spec1;  //free
  uint8_t synced;  //is this register synchronised with MASTER, 0=no 1=yes 2=don't care

} struProp;

//-------------------------------------------------------------------
//Struktura za Menu instance
typedef struct {
  String name;    //Second line name, max 16 char
  long parent;    //Parent menu IDX
  long val;       //Numerical value register
  long min;       //Minimum allowed value
  long max;       //Maximum allowed value
  long def;       //Default value
  struProp prop;  //Dodatni properties instance
} struMenuItem;

//rezervisi memoriju (ode jos 5k RAM-a cas!) **************
#define MAX_ITEMS 128
struMenuItem MenuItem[MAX_ITEMS];
int OffsetMap[16];
int ic = 0; //item counter
int pc = 0; //parent counter
byte idx = 0;

//global/master status
int MachineStatus = 0; // 0=STOP; 1=RUN, 2=PAUSE, 3=ERROR, 4=SERVC
int last_machine_status = 99;
String StatusType[10] = {
  " STOP", //0
  "  RUN", //1
  "READY", //2
  "PAUSE", //3
  "ERROR", //4
  "SERVC", //5
  "!SYNC", //6
  "!NULL", //7
  "!POWR", //8
  "SAFE!"  //9
};

//definicija Root menija
#define ROOT_ITEMS 9
String Root_menu[ROOT_ITEMS] = {
  "OUT07:",
  "1:Points set    ",
  "2:Mode set **   ",
  "3:Zero set      ",
  "4:Modbus set    ",
  "5:Action        ",
  "6:Diagnostic    ",
  "7:Info          ",
  "8:LCD setting   ",
};

int RPP = 0; //hold current possition of root menu
int RPP_max = ROOT_ITEMS;
int SPP[ROOT_ITEMS] = {}; //holds pos for each submenu

//LCD
const   uint16_t  maxPWM = 0xFFFF;        // Max value (hex) for PWM (see datasheet)
const   uint16_t  contrast = maxPWM / 3.2;  // 33% duty cycle should be a good "default" value (V0 = Vdd/3 = 1.1V)
const   uint16_t  bklight = 0;            // 0 means full bright backlight
LiquidCrystal     lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
uint32_t          cnt;
bool do_lcd_update = false;
bool lcd_bl_toggle = false;
bool lcd_bl_toggle_enable = false;
int last_root_menu = 0xff;

//todo
//here we connect internal parameters to menu items, 0=default without connection
int LinkItemValue_contrast = 53;
int LinkItemValue_bklight = 54;
int LinkItemValue_rotdelay = 55;

int menu_no = 0;

char tbs[128];
char nm[32];
char nm2[32];

bool data_pulled = false;

//=========================
void setup()
//=========================
{

  // Initialize LCD and the two used PWMs (for contrast and backlight)
  lcd.begin(16, 2);
  lcd.noCursor();
  lcd.print("LCD-KBD-SM  v1.3");
  lcd.setCursor(0, 1);
  lcd.print("~Alfa Robotika~");
  pinMode(LCD_V0, PWM);
  pinMode(LCD_LED, PWM_OPEN_DRAIN);   // This one must be "open drain" output
  pwmWrite(LCD_V0, contrast);
  pwmWrite(LCD_LED, bklight);

  //Init serial
  MySer.begin(115200);
  //MySer.println(myInfo);

  //Init I2C
  //HWire.begin();
  //Scan for devices
  //i2c_scanner();

  //=========================
  //Build menu
  //=========================
  ic = 0; //item counter
  pc = 0; //parent counter
  OffsetMap[0] = 0; //offset map

  //home
  MenuItem[ic] = {"POS", pc, 0, -165000, 165000, 255,  {0, 0, 0, 0, 0, 0, 0, 0}}; ic++;
  //MenuItem[ic] = {"", pc, 40, 0, 65000, 255,   {3, 0, 1, 0, 0, 0, 0, 0}}; ic++;//bargraph, editable
  pc++; OffsetMap[pc] = ic;

  //points set
  MenuItem[ic] = {"PT1 val", pc, 10000, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT1 <>01", pc, 1, 0, 2, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT2 val", pc, 0, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT2 <>01", pc, 0, 0, 2, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT3 val", pc, 0, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT3 <>01", pc, 0, 0, 2, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT4 val", pc, 0, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT4 <>01", pc, 0, 0, 2, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT5 val", pc, 0, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT5 <>01", pc, 0, 0, 2, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT6 val", pc, 0, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT6 <>01", pc, 0, 0, 2, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT7 val", pc, 0, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT7 <>01", pc, 0, 0, 2, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT8 val", pc, 0, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PT8 <>01", pc, 0, 0, 3, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  pc++; OffsetMap[pc] = ic;

  //mode set
  MenuItem[ic] = {"UVW/ENC", pc, 1, 0, 1, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"1x/4x", pc, 1, 0, 1, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"USW off", pc, 50, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"USW slope", pc, 125, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"USW freq", pc, 1, 0, 0xFFFF, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"16/32 len", pc, 1, 0, 1, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"Polarity", pc, 1, 0, 1, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"PolarityZ", pc, 1, 0, 1, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  pc++; OffsetMap[pc] = ic;

  //zero set
  MenuItem[ic] = {"Zero pt", pc, 10, -999999, 999999, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"Last pt", pc, 0, 0, 0xffff, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  pc++; OffsetMap[pc] = ic;

  //modbus
  MenuItem[ic] = {"Slave ADR", pc, 1, 1, 255, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"Baud Rate", pc, 115200, 4800, 230400, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"Bits", pc, 8, 8, 9, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"Parity", pc, 0, 0, 1, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable 0=nopar,nocrc|
  MenuItem[ic] = {"Stop Bits", pc, 1, 1, 2, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  MenuItem[ic] = {"Check RX CRC", pc, 1, 0, 1, 0, {0, 0, 1, 0, 0, 0, 0, 0}}; ic++; //editable
  pc++; OffsetMap[pc] = ic;

  //action
  MenuItem[ic] = {"Zerro Machine", pc, 3, 0, 0, 0, {0, 1, 0, 1, 0, 0, 0, 2}}; ic++; //action, hidden  //
  MenuItem[ic] = {"Reset Nulling", pc, 1, 0, 0, 0, {0, 1, 0, 1, 0, 0, 0, 2}}; ic++; //action, hidden //
  MenuItem[ic] = {"BACKUP MEM", pc, 2, 0, 0, 0, {0, 1, 0, 1, 0, 0, 0, 2}}; ic++; //action, hidden
  MenuItem[ic] = {"RESTORE MEM", pc, 0, 0, 0, 0, {0, 1, 0, 1, 0, 0, 0, 2}}; ic++; //action, hidden
  MenuItem[ic] = {"RESET MCU!!!", pc, 1, 0, 0, 0, {0, 1, 0, 1, 0, 0, 0, 2}}; ic++; //action, hidden //
  pc++; OffsetMap[pc] = ic;
  
  //diagnostic
  MenuItem[ic] = {"RX Pac", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  MenuItem[ic] = {"RX Byte", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  MenuItem[ic] = {"TX Pac", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  MenuItem[ic] = {"Framming ER", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  MenuItem[ic] = {"Noise ER", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  MenuItem[ic] = {"CRC ER", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  MenuItem[ic] = {"Broad Pac", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  MenuItem[ic] = {"Up Mins", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  MenuItem[ic] = {"Parity ER", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  MenuItem[ic] = {"TotMins", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  MenuItem[ic] = {"Reboots", pc, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; 
  pc++; OffsetMap[pc] = ic;

  //info
  MenuItem[ic] = {"Version: 1.3", pc, 0, 0, 0, 255, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; //
  MenuItem[ic] = {"Build: 24092017", pc, 1, 0, 0, 255, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; //
  MenuItem[ic] = {"By:Alfa Robotika", pc, 1.52, 0, 0, 255, {0, 0, 0, 0, 0, 0, 0, 2}}; ic++; //
  MenuItem[ic] = {"FW FRAM 64Kbit", pc, 1, 0, 0, 255, {0, 0, 0, 1, 0, 0, 0, 2}}; ic++; //
  pc++; OffsetMap[pc] = ic;

  //lcd
  MenuItem[ic] = {"Contrast", pc, 25000, 0, 65000, 0, {0, 0, 1, 0, 0, 0, 0, 2}}; ic++; //editable //23
  MenuItem[ic] = {"Brightness", pc, 0, 0, 65000, 0, {0, 0, 1, 0, 0, 0, 0, 2}}; ic++; //editable
  MenuItem[ic] = {"RotDelay", pc, 265000, 5000, 800000, 0, {0, 0, 1, 0, 0, 0, 0, 2}}; ic++; //editable
  MenuItem[ic] = {"AUX1", pc, 0, 0, 0, 0, {0, 0, 0, 1, 0, 0, 0, 2}}; ic++; //action
  MenuItem[ic] = {"AUX2", pc, 0, 0, 0, 0, {0, 0, 0, 1, 0, 0, 0, 2}}; ic++; //action
  MenuItem[ic] = {"AUX3", pc, 0, 0, 0, 0, {0, 0, 0, 1, 0, 0, 0, 2}}; ic++; //action
  MenuItem[ic] = {"AUX4", pc, 0, 0, 0, 0, {0, 0, 0, 1, 0, 0, 0, 2}}; ic++; //action
  pc++; OffsetMap[pc] = ic;

  //=========================
  //END Build menu
  //=========================

  //KBD input pins
  pinMode(K_UP, INPUT_PULLUP);
  pinMode(K_DOWN, INPUT_PULLUP);
  pinMode(K_LEFT, INPUT_PULLUP);
  pinMode(K_RIGHT, INPUT_PULLUP);
  pinMode(K_ALT, INPUT_PULLUP);
  pinMode(K_ENTER, INPUT_PULLUP);
  pinMode(K_FUNC, INPUT_PULLUP);

  // Initaialize LED and BUT button
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  //update contrast
  pwmWrite(LCD_V0, MenuItem[LinkItemValue_contrast].val);

  //Bargraph chars definition
  byte bg1[] = { B10000, B10000, B10000, B10000, B10000, B10000, B10000, B10000 };
  byte bg2[] = { B11000, B11000, B11000, B11000, B11000, B11000, B11000, B11000 };
  byte bg3[] = { B11100, B11100, B11100, B11100, B11100, B11100, B11100, B11100 };
  byte bg4[] = { B11110, B11110, B11110, B11110, B11110, B11110, B11110, B11110 };
  byte bg5[] = { B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111 };

  //Store custom chars in LCD
  lcd.createChar(0, bg1);
  lcd.createChar(1, bg2);
  lcd.createChar(2, bg3);
  lcd.createChar(3, bg4);
  lcd.createChar(4, bg5);

  //some animation of bargraph
  for (int i = 0; i <= 80; i++) {
    printBarGraph(i, 0);
    delay(5);
  }

  //pull new data from MASTER
  //RequestDataSync();
  //MySer.println("Q00");

  //show banner for 1 sec then go to Home screen
  //delay(5000);
  do_lcd_update = true;
}

//=========================
void loop()
//=========================
{

  //some counter
  cnt++;

  //----------------------
  //periodic tasks
  if (cnt > MenuItem[LinkItemValue_rotdelay].val) { //LCD delay
    cnt = 0;

    //Backlight LED toggle
    if (lcd_bl_toggle_enable) {
      lcd_bl_toggle = !lcd_bl_toggle;
      if (lcd_bl_toggle) {
        pwmWrite(LCD_LED, bklight);
      } else {
        pwmWrite(LCD_LED, 0x8888); // half PWM
      }
    }

    //auto rotate
    if (state_rotate_mode & !state_edit_mode) {
      SPP[RPP]++;
      if (SPP[RPP] > (OffsetMap[RPP + 1] - OffsetMap[RPP] - 1)) SPP[RPP] = 0;
      do_lcd_update = true;
      RequestDataSync();
    }

    //blink if edit mode
    if (state_edit_mode) toggleLED();
  }

  //-------------------------------------
  //read from serial and decode command
  //-------------------------------------
  while (MySer.available() > 0) {

    // get the new byte:
    char inChar = MySer.read();

    // add it to the inputString without \n \r:
    if (inChar != '\r' && inChar != '\n') {
      inputString[idx] = inChar;
      idx++;
      inputString[idx] = '\0';
    } else {

      //Decode serial command
      if (idx) decode_command(idx);

      //cleanup
      for (int y = 0; y < sizeof(inputString) / sizeof(char); y++) {
        inputString[y] = 0;
      }
      idx = 0;
    }
  }

  //Debounce
  de_ms = millis();

  //fast read whole ports
  //bool result = GPIOA->regs->IDR & 0b0000000000000100; //returns true if A2 is HIGH
  PortA = GPIOA->regs->IDR;
  PortB = GPIOB->regs->IDR;

  state_k_up =    MyDebounce (0, PortA & 0b0000100000000000, true); //PA11 - K_UP
  state_k_down =  MyDebounce (1, PortA & 0b0001000000000000, true); //PA12 - K_DOWN
  state_k_left =  MyDebounce (2, PortA & 0b1000000000000000, true); //PA15 - K_LEFT
  state_k_right = MyDebounce (3, PortB & 0b0000000000100000, true); //PB5  - K_RIGHT
  state_k_enter = MyDebounce (5, PortB & 0b0000000100000000, false); //PB8  - K_ENTER
  state_k_func  = MyDebounce (6, PortB & 0b0000001000000000, false); //PB9  - K_FUNC
  // PortB & 0b0000000010000000          //PB7  - K_ALT

  menu_no = SPP[RPP] + OffsetMap[RPP];

  //posalji zahtev za sinhronizaciju
  if (data_pulled == false) {
    if (de_ms > 3000) {
      MySer.println("Q00");
      data_pulled = true;
    }
  }

  //-----------------------------------------
  //akcije tastera
  //-----------------------------------------
  if (state_k_up) {//UP

    if (state_edit_mode) {
      if (PortB & 0b0000000010000000) {
        //inc @ adr
        MenuItem[menu_no].val++;
      } else {
        //inc x1000 @ adr
        MenuItem[menu_no].val += 1000;
      }
      //saturate
      if (MenuItem[menu_no].val > MenuItem[menu_no].max) MenuItem[menu_no].val = MenuItem[menu_no].max;
    } else {
      //inc root pos
      RPP++;
      if (RPP > RPP_max - 1) RPP = 0;
    }

    state_k_up = false;
  }

  //-----------------------------------------
  if (state_k_down) {//DOWN

    if (state_edit_mode) {
      if (PortB & 0b0000000010000000) {
        //dec @ adr
        MenuItem[menu_no].val--;
      } else {
        //dec x1000 @ adr
        MenuItem[menu_no].val -= 1000;
      }
      //saturate
      if (MenuItem[menu_no].val < MenuItem[menu_no].min) MenuItem[menu_no].val = MenuItem[menu_no].min;
    } else {
      //dec root pos
      RPP--;
      if (RPP < 0) RPP = RPP_max - 1;
    }

    state_k_down = false;
  }

  //-----------------------------------------
  if (state_k_right) {//RIGHT

    if (state_edit_mode) {
      if (PortB & 0b0000000010000000) {
        //dec x10 @ adr
        MenuItem[SPP[RPP] + OffsetMap[RPP]].val -= 10;
      } else {
        //dec x100 @ adr
        MenuItem[SPP[RPP] + OffsetMap[RPP]].val -= 100;
      }
      //saturate
      if (MenuItem[menu_no].val < MenuItem[menu_no].min) MenuItem[menu_no].val = MenuItem[menu_no].min;
    } else {
      //incr sub-pos
      SPP[RPP]++;
      if (SPP[RPP] > (OffsetMap[RPP + 1] - OffsetMap[RPP] - 1) ) SPP[RPP] = 0;
    }

    state_k_left = false;
  }

  //-----------------------------------------
  if (state_k_left) {//LEFT

    if (state_edit_mode) {
      if (PortB & 0b0000000010000000) {
        //inc x10 @ adr
        MenuItem[menu_no].val += 10;
      } else {
        //inc x100 @ adr
        MenuItem[menu_no].val += 100;
      }
      //saturate
      if (MenuItem[menu_no].val > MenuItem[menu_no].max) MenuItem[menu_no].val = MenuItem[menu_no].max;
    } else {
      //dec sub pos
      SPP[RPP]--;
      if (SPP[RPP] < 0) SPP[RPP] = (OffsetMap[RPP + 1] - OffsetMap[RPP] - 1);
    }

    state_k_right = false;
  }

  //-----------------------------------------
  if (state_k_func) {//FUNC

    if (PortB & 0b0000000010000000) {

      //check if is editable
      if (MenuItem[SPP[RPP] + OffsetMap[RPP]].prop.editable) {
        //editable
        state_edit_mode = !state_edit_mode;
        if (state_edit_mode) {
          lcd.blink();
        } else {
          lcd.noBlink();
        }
        do_lcd_update = true;
      } else {
        //not editable
      }
    } else {
      //schortcut, go home
      RPP = 0;
      SPP[0] = 0;
      do_lcd_update = true;
    }

    state_k_func = false;
  }

  //-----------------------------------------
  if (state_k_enter) {//ENTER

    if (PortB & 0b0000000010000000) {
      if (state_edit_mode) {

        //Contrast
        if (RPP == 8 && SPP[RPP] == 0) {
          pwmWrite(LCD_V0, MenuItem[SPP[RPP] + OffsetMap[RPP]].val);
        }

        //Backlight
        if (RPP == 8 && SPP[RPP] == 1) {
          pwmWrite(LCD_LED, MenuItem[SPP[RPP] + OffsetMap[RPP]].val);
        }

        state_edit_mode = false;
        lcd.noBlink();

        //Send value to master, need to be synced by master
        MenuItem[SPP[RPP] + OffsetMap[RPP]].prop.synced = 0;
        SentToMaster();
      } else {
        //is action, to do
        //Send value to master
        if (MenuItem[SPP[RPP] + OffsetMap[RPP]].prop.action) {
          SentToMaster();
        }
      }
    } else {
      //togle home rotate
      //state_rotate_mode = !state_rotate_mode;

      //shortcut da se posalje sync komanda MASTER
      RequestDataSync();
    }

    state_k_enter = false;
  }

  //-----------------------------------------
  //LCD UPDATE
  //-----------------------------------------
  if (do_lcd_update == true) {
    toggleLED();

    int menuNo = SPP[RPP] + OffsetMap[RPP];

    //if edditable show cursor
    if (MenuItem[menuNo].prop.editable) {
      lcd.cursor();
    } else {
      lcd.noCursor();
    }

    //1st LINE -----
    //do not print same thing on first line
//    if (last_root_menu != RPP || last_machine_status != MachineStatus) {
    if (last_root_menu != RPP || home_outs_status != home_outs_status_prev) {
      lcd.clear();
      lcd.print(Root_menu[RPP]); //root name
      //on home print OUTs status
      if (RPP == 0) {
        //lcd.print(home_outs_status);
        //lcd.print(StatusType[MachineStatus]);

        if (home_outs_status >= 256) {
          //nije nulirano
          lcd.print("NOT NULLED");
          
        } else {

          lcd.print(' ');
          
          lcd.print(home_outs_status & 0x01 ? '1' : '0');
          lcd.print(home_outs_status & 0x02 ? '1' : '0');
          lcd.print(home_outs_status & 0x04 ? '1' : '0');
          lcd.print(home_outs_status & 0x08 ? '1' : '0');
  
          lcd.print(' ');
  
          lcd.print(home_outs_status & 0x10 ? '1' : '0');
          lcd.print(home_outs_status & 0x20 ? '1' : '0');
          lcd.print(home_outs_status & 0x40 ? '1' : '0');
          lcd.print(home_outs_status & 0x80 ? '1' : '0');
        }
      }
    }

    //2nd LINE ---------
    //switch by type
    if (MenuItem[menuNo].prop.type == 0) {
      //normal
      int msg_len = MenuItem[menuNo].name.length();
      lcd.setCursor(0, 1);

      //indikator sinhronizacije podatka sa master, stavi * ako je 0
      if (MenuItem[menuNo].prop.synced == 0) {
        lcd.print("*");
        msg_len++;
      }

      lcd.print(MenuItem[menuNo].name); //sub item name
      //clear rest of LCD buffer
      for (int x = msg_len; x < 16; x++) {
        lcd.print(" ");
      }

      if (MenuItem[menuNo].prop.hidden != true) { //if not hidden
        lcd.setCursor(msg_len, 1);
        //lcd.print(":");
        //lcd.print(menuNo); //value
        lcd.print(":");
        lcd.print(MenuItem[menuNo].val); //value
        //MySer.println(MenuItem[SPP[RPP] + OffsetMap[RPP]].name.length());
      }

    }

    //bargraph
    if (MenuItem[menuNo].prop.type == 3) {
      //
      lcd.noCursor();
      lcd.noBlink();
      lcd.setCursor(0, 1);
      lcd.print(MenuItem[menuNo].name);
      printBarGraph(abs(MenuItem[menuNo].val), MenuItem[menuNo].name.length());

    }

    //save some states for next iteration
    do_lcd_update = false;
    last_root_menu = RPP;
    last_machine_status = MachineStatus;
    home_outs_status_prev = home_outs_status;
  }

}

//-----------------------------------------
//debounce rutina
//-----------------------------------------
bool MyDebounce (int reg, bool state, bool enable_longclick) {

  de_state = false;

  if (state != lastButtonState[reg])
    // reset the debouncing timer
    lastDebounceTime[reg] = de_ms;

  if ((de_ms - lastDebounceTime[reg]) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (state != buttonState[reg]) {
      buttonState[reg] = state;

      // only toggle if the new button state is HIGH
      if (buttonState[reg] == LOW) {
        de_state = true;
        do_lcd_update = true;
      }

    }
  }
  lastButtonState[reg] = state;

  if (enable_longclick) {

    //-----------long click
    if (state) {
      longclick_flag[reg] = 0; //no click
    } else {
      longclick_flag[reg]++;
    }

    //long click
    if ((de_ms - long_click_rate) > longclick_ts[0] && !(state)) {
      //imamo longclick akciju, trigeruj ponovo istu akciju

      longclick_ts[0] = de_ms;

      //treba nam samo u edit modu i proradi samo posle delay
      if (state_edit_mode) {
        if (longclick_flag[reg] > 40000) {
          de_state = true;
          do_lcd_update = true;
        }
      }
    }
  }

  return de_state;

}


//-----------------------------------------
//togle LED on board
//-----------------------------------------
void toggleLED() {
  digitalWrite(LED, !digitalRead(LED));
}

//-----------------------------------------
//Serial Command decoder ******************
//-----------------------------------------
void decode_command(int len) {

  //dekodiramo samo prva tri karaktera
  /*
    InputString[0]; //cmd 1,2,3,4,a...
    inputString[1]; //adr L 0-9 ASCII
    inputString[2]; //adr H 0-9 ASCII
    inputString[3]; //fiksirano/ignorise se
    inputString[4]; //0-9 ASCII
    inputString[5]; //0-9 ASCII
    inputString[6]; //0-9 ASCII
    inputString[7]; //0-9 ASCII
    inputString[8]; //0-9 ASCII
    inputString[9]; //0-9 ASCII
    inputString[10];//0-9 ASCII
  */

  char cmd = inputString[0];

  switch (cmd) {

    case '{':
      for (int i = 0; i < 256; i++) {
        MySer.print(char(i));
      }
      MySer.println(" ok");

      MySer.print("size of name[2]=");
      MySer.println(MenuItem[2].name.length());
      break;

    case 'e': //toggle edit mode
      state_edit_mode = !state_edit_mode;
      MySer.println(" ok");
      break;

    case 'r': //toggle rotate mode
      state_rotate_mode = !state_rotate_mode;
      MySer.println(" ok");
      break;

    case 'h': //help
    case 'H': //help
      MySer.println(myHelp);
      break;

    case 'b': //Toggle LCD LED backlight
      lcd_bl_toggle_enable = !lcd_bl_toggle_enable;
      MySer.println(" ok");
      break;


    /*
      " STOP",
      "  RUN",
      "READY",
      "PAUSE",
      "ERROR",
      "SERVC",
      "!SYNC",
      "!NULL",
    */
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7': //!null, treba da ga fokusiramo na Akciju Zero Machine
    case '8':
    case '9':

      MachineStatus = int(inputString[0] - 48);
      //MySer.print(StatusType[MachineStatus]);
      //MySer.println(" ok");

      if (cmd == '7') {
        RPP = 6; //Action menu
      } else {
        RPP = 0; //Home za sve ostale
      }

      do_lcd_update = true;


      break;

    case 't': //test
      lcd.clear();
      lcd.print("0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF01234567"); //
      MySer.println(" ok");
      break;
      
    case 'm': //print message on 1st line
      lcd.setCursor(0, 0);
      for (int x=1; x<=16; x++) {lcd.print(inputString[x]);}
      MySer.println(" ok");
      break;
      
    case 'M': //print message on 2st line
      lcd.setCursor(0, 1);
      for (int x=1; x<=16; x++) {lcd.print(inputString[x]);}
      MySer.println(" ok");
      break;
      
    case 'o': //set OUT status var
    case 'O': //set OUT status var

        //cleanup for atoi
        inputString[0] = '0'; //cmd
        
        //asign value to item
        home_outs_status = atoi(inputString);

        do_lcd_update = true;
      break;



    case 'd': //dump
      
      MySer.println("\nSPP idx:val");
      for (int x = 0; x < ROOT_ITEMS; x++) {
        MySer.print(x); MySer.print(":"); MySer.println(SPP[x]);
      }
      MySer.println("\nOffsetMap idx:val");
      for (int x = 0; x < ROOT_ITEMS; x++) {
        MySer.print(x); MySer.print(":"); MySer.println(OffsetMap[x]);
      }
      MySer.println("\nRoot_menu idx:val");
      for (int x = 0; x < ROOT_ITEMS; x++) {
        MySer.print(x); MySer.print(":"); MySer.println(Root_menu[x]);
      }
      

      MySer.println("\nMenuItem");
      MySer.println("------------------------------------------------------------------------------------------------");
      MySer.println("idx | root menu        | typ | name             | val     | min     | max     | hid | edt | act ");
      MySer.println("------------------------------------------------------------------------------------------------");
      for (int x = 0; x < ic; x++) {
        Root_menu[MenuItem[x].parent].toCharArray(nm2, 16);
        MenuItem[x].name.toCharArray(nm, 16);
        sprintf(tbs, "%-3d | %-16s | %-3d | %-16s | %-7d | %-7d | %-7d | %-3d | %-3d | %-3d",
                x, nm2, MenuItem[x].prop.type, nm, MenuItem[x].val, MenuItem[x].min, MenuItem[x].max, MenuItem[x].prop.hidden, MenuItem[x].prop.editable, MenuItem[x].prop.action);
        MySer.println (tbs);
        //if ((x+1) % 5 == 0) MySer.println("---------------------------------------------");
      }
      MySer.println();

      MySer.print("\nMaximum number of Menu Items: ");
      MySer.print(MAX_ITEMS);
      MySer.print("\nUsed Items: ");
      MySer.print(ic);
      MySer.print("\nMenuItem size: ");
      MySer.print(sizeof(MenuItem));
      MySer.println(" Bytes");
      MySer.print("Single Item size: ");
      MySer.print(sizeof(MenuItem) / MAX_ITEMS);
      MySer.println(" Bytes");
      break;

    case 'D': //dump sa kreiranjem C code-a

      MySer.println("");
      MySer.println("  //--------START COPY/PASTE --------------------------");

      for (int x = 0; x < ic; x++) {
        if (MenuItem[x].prop.synced < 2) {
          Root_menu[MenuItem[x].parent].toCharArray(nm2, 16); //root name
          MenuItem[x].name.toCharArray(nm, 16); //parent name
          sprintf(tbs, "  MyMemory[%d] = %d; //%s:%s",
                  x, MenuItem[x].val, nm2, nm);
          MySer.println (tbs);
        }
      }

      MySer.println("  //--------END COPY/PASTE ---------------------------");

      break;

    case 's':
    case 'S': //set reg = parsed ASCII value 0 - 99

      //validate input, we don't want pick and poke :)
      if (inputString[1] > 47 && inputString[1] < 58 && inputString[2] > 47 && inputString[2] < 58) {
        //number found
        int adr = int(inputString[2] - 48) + int(inputString[1] - 48) * 10;

        //cleanup for atoi
        inputString[0] = '0'; //cmd
        inputString[1] = '0'; //adrL
        inputString[2] = '0'; //adrH
        inputString[3] = '0'; //'='

        //asign value to item
        MenuItem[adr].val = atoi(inputString);

        if (cmd == 's') {
          //negative number
          MenuItem[adr].val *= -1;
        }

        //MySer.println(" ok");

        //updejtuj displej samo ako je u fokusu
        if ((RPP == MenuItem[adr].parent) && (SPP[RPP] == adr - OffsetMap[RPP]))
          do_lcd_update = true;

        //postavi flag da je ovaj podatak sinhronizovan sa masterom
        MenuItem[adr].prop.synced = 1;

      } else {
        MySer.println(" err invalid adr");
      }
      break;

    case 'f': //focus
      //validate input, we don't want pick and poke :)

      if (inputString[1] > 47 && inputString[1] < 58 && inputString[2] > 47 && inputString[2] < 58) {

        //number found

        int adr = int(inputString[2] - 48) + int(inputString[1] - 48) * 10;
        RPP = MenuItem[adr].parent; //setuj aktivni root menu

        //if (adr) {
        //setuj aktivni podmeni
        SPP[RPP] = adr - OffsetMap[RPP]; //
        //}

        do_lcd_update = true;
        MySer.print(adr);
        MySer.println(" ok");

      } else {
        MySer.println(" err invalid adr");
      }

      break;

    case 'p': //print bargraph
      //validate input, we don't want pick and poke :)
      if (inputString[1] > 47 && inputString[1] < 58 && inputString[2] > 47 && inputString[2] < 58) {
        //number found
        int val = int(inputString[2] - 48) + int(inputString[1] - 48) * 10;
        printBarGraph (val, 0);
        //do_lcd_update = true;
        MySer.println(" ok");
      }  else {
        MySer.println(" err invalid adr");
      }
      break;

    default:
      // if nothing else matches, do the default
      MySer.println(" err");
      //MySer.println(myInfo);
      break;
  }

}

//---------------------
//Print bargraph
//---------------------
void printBarGraph (int val, int cursor_at) {

  lcd.setCursor(cursor_at, 1);

  int fuls = floor(val / 5);
  int fuls_cnt = fuls;
  int rest = val - fuls * 5;
  int clean_from = 0;

  if (val) {// 1-80
    while (fuls_cnt--) {
      lcd.write(byte(4));
    }
    if (rest > 0) {
      lcd.write(byte(rest - 1));
      //cleanup to end of LCD buff
      clean_from = fuls + 1;
    } else {
      //cleanup to end of LCD buff
      clean_from = fuls;
    }
  } else {// zero value
    clean_from = cursor_at;
  }
  //cleanup to end of LCD buff
  for (int i = cursor_at; i < 16; i++) {
    lcd.print(" ");
  }
}

//---------------------
//Send value to master
//---------------------
void SentToMaster () {
  //MySer.print("R:"); //root menu
  //MySer.print(RPP);
  //MySer.print(" S:"); //sub menu
  //MySer.print(SPP[RPP]);

  long val = MenuItem[(SPP[RPP] + OffsetMap[RPP])].val;
  int ix =  (SPP[RPP] + OffsetMap[RPP]);
  char current_dir_cmd;

  if (val < 0 ) {
    //negative number
    current_dir_cmd = 's';
  } else {
    //positive number
    current_dir_cmd = 'S';
  }

  MySer.print(current_dir_cmd);
  if (ix <= 9) {
    MySer.print("0");
  }
  MySer.print(ix);
  MySer.print("=");

  //item value
  MySer.println(abs(val));

}

//---------------------
//I2C scanner
//---------------------
void i2c_scanner() {

/*

  byte error, address;
  int nDevices;

  MySer.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 128; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    HWire.beginTransmission(address);
    error = HWire.endTransmission();

    if (error == 0) {
      MySer.print("I2C device found at address 0x");
      if (address < 16)
        MySer.print("0");
      MySer.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      MySer.print("Unknown error at address 0x");
      if (address < 16)
        MySer.print("0");
      MySer.println(address, HEX);
    }
  }
  if (nDevices == 0)
    MySer.println("No I2C devices found");
  else
    MySer.println("done");

  delay(1000);           // wait 5 seconds for next scan
*/
}

//-----------------------------------------------------
//Request sinhronisation of data from MASTER MCU {
void RequestDataSync () {
  //ka MASTER
  MySer.println("Q00"); //cmd
}

