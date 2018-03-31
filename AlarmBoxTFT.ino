/*
  AlarmBoxTFT

  The code behind the touchscreen for the UB-25 Alarm System.

  The circuit:
    D2 -> Relay1, end of weld -- pin 2 from UB25
    D3 -> Relay2, alarm       -- pin 4
    A2 -> Relay3, limit       -- pin 6        
    D5 -> Alarm FET   
    A3 -> Inhibit FET

  EEPROM:
    Address |  Item            |  Length   
    ——————————————————————————————————————
    0-2     |  timer digits    |  3
    3-6     |  counter digits  |  4
    7-10    |  limit digits    |  4
    11      |  inhibit bool    |  1
    12      |  first setup     |  1
            |     check        |
  
  Created July 30th, 2017
  By Galen Wu
  Modified day month year
  By author's name
*/

#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>
#include <EEPROM.h>

#define TIMER_START_ADDRESS 0
#define TIMER_END_ADDRESS 2
#define COUNTER_START_ADDRESS 3
#define COUNTER_END_ADDRESS 6
#define LIMIT_START_ADDRESS 7
#define LIMIT_END_ADDRESS 10
#define INHIBIT_ADDRESS 11
#define FIRST_SETUP_ADDRESS 12

#define BUZZER_FET 5
#define INHIBIT_FET A3

/*  screen coordinates where top left point is (0, 0)
    ____________________________________
    |(0, 0) -> positive x dir   (+x, 0)|
    |  |                               |
    |  V                               |
    | positive y dir                   |
    |                                  |
    |(0, +y)                           |
*/
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

//touchscreen init, predetermined by Adafruit library
#define STMPE_CS 8
#define TFT_DC 9
#define TFT_CS 10

Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);       //touchscreen object
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);  //tft drawing object


/* ---------------------------DIMENSION CONSTANTS-----------------------------*/
/* dimension constants for drawing gui objects
   x = x initial coordinate
   y = y initial coordinate
   w = width of object
   h = height of object
*/

//top_left_btn used for backdoor top left field and drawing COUNTER btn
#define TOP_LEFT_BTN_X 0
#define TOP_LEFT_BTN_Y 0
#define TOP_LEFT_BTN_W 150
#define TOP_LEFT_BTN_H 50

//top_right_btn used for backdoor top right field, drawing RESET btn, and RESET btn touch field
#define TOP_RIGHT_BTN_X 200
#define TOP_RIGHT_BTN_Y 0
#define TOP_RIGHT_BTN_W 125
#define TOP_RIGHT_BTN_H 50

//used for drawing counters, system settings, and covering counter digits on gui
#define COUNTER_X 12
#define COUNTER_Y 75
#define COUNTER_W 75
#define COUNTER_H 150

//used for drawing timer digits
#define TIMER_X_MINUTE 80
#define TIMER_X_10SEC 174
#define TIMER_X_1Sec 245
#define TIMER_Y 30
#define TIMER_W 60
#define TIMER_H 85

//used for drawing limit digits
#define LIMIT_X 75
#define LIMIT_Y 120
#define LIMIT_W 60
#define LIMIT_H 75

//used for enter button and backdoor of bottom left field
#define LEFT_KEY_X 0
#define LEFT_KEY_Y 205
#define LEFT_KEY_W 125
#define LEFT_KEY_H 35

//used for unlock button and backdoor of bottom right field
#define RIGHT_KEY_X 195
#define RIGHT_KEY_Y 205
#define RIGHT_KEY_W 125
#define RIGHT_KEY_H 35


/* array locations - these two variables are used for quick access to the locations of limit and timer digits
   Used whenever you touch a digit, as seen in main loop - systems screen and blink
*/
const int timerLocation[] = {TIMER_X_MINUTE, TIMER_X_10SEC, TIMER_X_1Sec};        
const int limitLocation[] = {LIMIT_X, LIMIT_X + LIMIT_W, LIMIT_X + LIMIT_W*2, LIMIT_X + LIMIT_W*3 - 1}; //explain similar ^
/* ---------------------------------------------------------------------------*/


/*******************************************************************
*                       GLOBAL VARIABLES                           *
********************************************************************/



/* -----------------------------STATUS FLAGS----------------------------------*/
boolean inhibitStatus = false;          //false -> don't inhibit welder, true -> inhibit
boolean thresholdReached = false;       //false -> below weld count limit, true -> past weld count limit
boolean startScreen = true;             //true -> counter screen, false -> system settings screen

/*  Change limit is a little weird. it keeps track what you are changing on the system 
    settings screen. If True, change limit numbers. False, change alarm duration      */
boolean changeLimit = true;             //true -> change limit, false -> change timer

boolean backDoorCheck[4] = {false};     //array of flags to track progress through backdoor sequence - double tap top left, top right, bottom left, bottom right

int cdigit[] = {0, 0, 0, 0};            //counter digits, reads left to right {1, 5, 0, 0} = 1500
int tdigit[] = {0, 0, 0};               //timer digits, also reads left to right {2, 3, 0} = 2:30 minutes:seconds
int ldigit[] = {0, 0, 0, 0};            //weld limit digits, same as above like counter

/* 0 -> 0 _ _ _, 1 -> _ 0 _ _, 2 -> _ _ 0 _, 3 -> _ _ _ 0
   used for determining which digit to change and blink for system settings
   if digit state = 0, then you are changing LSB of either limit values or alarm values
   Ultimately, it keeps track of which digit  
*/
int digitState = 3;                     
/* ---------------------------------------------------------------------------*/


/* -----------------------------TIMER VARIABLES-----------------------------*/
//timer variables
unsigned long alarmDuration;              //time duration of alarm in ms, calculations comments in setup
boolean alarmOn = false;                  //true -> alarm is on, false -> alarm is off

/* variable to mark when the alarm goes off. This is
   is later used to determine how much time passed to compare
   if it meets the duration of the alarm
*/
unsigned long alarmStartTime;  
/* ---------------------------------------------------------------------------*/


/* -----------------------------INCREMENT FLAGS-------------------------------*/
//limit increments
/* Note, only 1 will be true at a time for byTenSec and byMin*/
boolean byTenSec = false, byMin = false;            //flags for the goto functions of increments, if true, increment by either 10 secs or a minute
boolean incrementA = false, incrementB = false;     //flags for changing limit numbers used by goto, see table below
/*
  A  B  | Outcome
  ---------------
  F  F  | +1
  F  T  | +10
  T  F  | +100
  T  T  | +1000
*/
/* ---------------------------------------------------------------------------*/


/* ------------------------BLINKING/DEBOUNCE VARIABLES-------------------------*/
//timing for blink
#define BLINK_TIME 500                //system settings screen for blinking digits on gui, in ms
unsigned long previousBlink = 0;      //keeps track of when the previousBlink occurs
boolean blinkOn = false;              //false -> print digit, true -> cover with white to blink

//debounce variables
#define DEBOUNCING_TIME 50            //debouncing time for interrupts in ms
volatile unsigned long last_micros;   //keeps track of last debounce for debouncing
/* ---------------------------------------------------------------------------*/



/*******************************************************************
*                         DRAW FUNCTIONS                           *
********************************************************************/
/*
  Draws and changes the current screen depending on the startScreen flag.
  If true, draw counter screen. False, draw system settings screen

  @param n/a
  @return draws a screen depending on startScreen flag
*/
void changeScreen() {
  tft.fillScreen(ILI9341_WHITE);          //Adafruit function for filling screen with white
  tft.setTextColor(ILI9341_BLACK);        
  Serial.println("Changing Screens");     //writing to serial monitor for debugging

  if (startScreen) { //counter screen
    tft.setCursor(TOP_LEFT_BTN_X + 13 , TOP_LEFT_BTN_Y + 13);   //draw counter btn
    tft.setTextColor(ILI9341_BLACK);
    tft.setTextSize(3);
    tft.println("COUNTER");

    drawCounters();
  }
  else {    //system settings screen
    tft.setTextSize(2);              //draw system settings text at top left
    tft.setCursor(3, 3);
    tft.println("System Settings");
    tft.fillTriangle(10, 105, 70, 105, 40, 30, ILI9341_BLACK);        //increment arrow
    tft.fillTriangle(10, 115, 70, 115, 40, 190, ILI9341_BLACK);       //decrement arrow

    //draw enter btn
    tft.setTextSize(3);
    tft.drawRect(LEFT_KEY_X, LEFT_KEY_Y, LEFT_KEY_W, LEFT_KEY_H, ILI9341_BLACK);
    tft.setCursor(LEFT_KEY_X+17, LEFT_KEY_Y+5);
    tft.print("ENTER");

    //draw unlock button
    tft.drawRect(RIGHT_KEY_X, RIGHT_KEY_Y, RIGHT_KEY_W, RIGHT_KEY_H, ILI9341_BLACK);
    tft.setCursor(RIGHT_KEY_X+10, RIGHT_KEY_Y+5);
    tft.print("UNLOCK");

    drawSettings();
  }
}

/*
  Draws the counter screen

  @param n/a
  @return draws the counter digits
*/
void drawCounters() {
  if (thresholdReached == false) {    //below limit
    tft.setTextColor(ILI9341_BLACK);

    //draws white over counter digits to clear
    tft.fillRect(TOP_RIGHT_BTN_X, TOP_RIGHT_BTN_Y, TOP_RIGHT_BTN_W, TOP_RIGHT_BTN_H, ILI9341_WHITE);
    tft.setTextSize(3);

    //draws current timer digits at bottom left
    tft.setCursor(COUNTER_X, COUNTER_Y + COUNTER_H/2 + 60);   //sets cursors where to draw timer digits
    tft.print(tdigit[0]);
    tft.print(":");
    tft.print(tdigit[1]);
    tft.print(tdigit[2]);

    //draw limit digits at bottom right
    tft.setCursor(COUNTER_X + COUNTER_W * 3 + 4, COUNTER_Y + COUNTER_H/2 + 60);
    tft.print(ldigit[0]);
    tft.print(ldigit[1]);
    tft.print(ldigit[2]);
    tft.print(ldigit[3]);
  }
  else {      //above limit so red
    //draw reset btn
    tft.drawRect(TOP_RIGHT_BTN_X, TOP_RIGHT_BTN_Y, TOP_RIGHT_BTN_W, TOP_RIGHT_BTN_H, ILI9341_BLACK); //draws border for reset btn 
    tft.setCursor(TOP_RIGHT_BTN_X + 19 , TOP_RIGHT_BTN_Y + 13);      //set cursor for top right btn
    tft.setTextColor(ILI9341_BLACK);  
    tft.setTextSize(3);
    tft.println("RESET");         
    
    //draws change
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(6);
    tft.setCursor(COUNTER_X + COUNTER_W/2 + 5, COUNTER_Y + COUNTER_H/2 + 40);
    tft.println("CHANGE");
  }
  /* Draws counter digits one at a time*/
  tft.setTextSize(12);
  //0 _ _ _
  tft.setCursor(COUNTER_X + 7, COUNTER_Y + COUNTER_H/10);
  tft.print(cdigit[0]);
  
  //_ 0 _ _
  tft.setCursor(COUNTER_X + COUNTER_W + 6, COUNTER_Y + COUNTER_H/10);
  tft.print(cdigit[1]);

  //_ _ 0 _
  tft.setCursor(COUNTER_X + COUNTER_W * 2 + 5, COUNTER_Y + COUNTER_H/10);
  tft.print(cdigit[2]);
  
  //_ _ _ 0
  tft.setCursor(COUNTER_X + COUNTER_W * 3 + 4, COUNTER_Y + COUNTER_H/10);
  tft.print(cdigit[3]);
}

/*
  Draws the system settings screen

  @param n/a
  @return draws system settings
*/
void drawSettings() {
  /* Draws digits one at a time*/
  tft.setTextSize(11);
  
  //timer digits
  //0 _ _ _
  tft.setCursor(TIMER_X_MINUTE, TIMER_Y);
  tft.print(tdigit[0]);
  
  //_ : _ _
  tft.setCursor(COUNTER_X + COUNTER_W + 40, TIMER_Y);
  tft.print(":");
  
  //_ _ 0 _
  tft.setCursor(TIMER_X_10SEC, TIMER_Y);
  tft.print(tdigit[1]);
  
  //_ _ _ 0
  tft.setCursor(TIMER_X_1Sec, TIMER_Y);
  tft.print(tdigit[2]);

  tft.setTextSize(10);
  
  //drawing the limit digits
  tft.setCursor(LIMIT_X, LIMIT_Y);
  tft.print(ldigit[0]);
  tft.print(ldigit[1]);
  tft.print(ldigit[2]);
  tft.print(ldigit[3]);
}


/*******************************************************************
*                  INCREMENT/DECREMENT FUNCTIONS                   *
********************************************************************/
/*
  Increments the counter by one and draws the counter

  general outline is - check digit, if not at the edge case then increment.
                                    Otherwise set to 0
                       then cover the digit thats changing with a white square
                       redraw

  @param n/a
  @return +1 to counter and draws
*/
void incrementTimer() {
  //shouldnt use goto but couldn't figure out a way
  //without writing a huge if block or more functions
  if (byTenSec){ //when you want to increment by 10 sec
    goto increment10sec;
  }
  else if (byMin){  //by 1 minute
    goto incrementmin;
  }

  //reminder that tdigit is from left to right, i.e. {2, 3, 0} = 2:30 minutes:seconds
  if (tdigit[2] == 9) { //if minute = 9, set it to 0 
    tdigit[2] = 0;
    tft.fillRect(TIMER_X_1Sec, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
    
    increment10sec:
    if (tdigit[1] == 5) { //if 10sec digit = 5, set it to 0
      tdigit[1] = 0;
      tft.fillRect(TIMER_X_10SEC, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);

      incrementmin:
      if (tdigit[0] == 9) { //if 1sec digit = 9, set it to 0
        tdigit[0] = 0;
        tft.fillRect(TIMER_X_MINUTE, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
      }
      else {      //otherwise increment the sec
        tdigit[0]++;
        tft.fillRect(TIMER_X_MINUTE, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
      }
    }
    else {      //otherwise increment by 10 sec
      tdigit[1]++;
      tft.fillRect(TIMER_X_10SEC, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
    }
  }
  else {      //otherwise increment by 1 minute
    tdigit[2]++;
    tft.fillRect(TIMER_X_1Sec, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
  }
  drawSettings();
}

/*
  Decrements the counter by one and draws the counter

  general outline is - check digit, if not at the edge case then decrement.
                                    Otherwise set to 9
                       then cover the digit thats changing with a white square
                       redraw

  @param n/a
  @return -1 to counter and draws
*/
void decrementTimer() {
  if (byTenSec){    //decrement by 10
    goto decrement10sec;
  }
  else if (byMin){  //decrement by 1 min
    goto decrementmin;
  }

  //reminder that tdigit is from left to right, i.e. {2, 3, 0} = 2:30 minutes:seconds
  if (tdigit[2] == 0) { //if min = 0, set to 9
    tdigit[2] = 9;
    tft.fillRect(TIMER_X_1Sec, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);

    decrement10sec:
    if (tdigit[1] == 0) { //if 10 sec = 0, set to 5
      tdigit[1] = 5;
      tft.fillRect(TIMER_X_10SEC, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
      
      decrementmin:
      if (tdigit[0] == 0) { //if 10 sec = 0, set to 9
        tdigit[0] = 9;
        tft.fillRect(TIMER_X_MINUTE, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
      }
      else {   //edge cases covered above, otherwise decrement
        tdigit[0]--;
        tft.fillRect(TIMER_X_MINUTE, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
      }
    }
    else {
      tdigit[1]--;
      tft.fillRect(TIMER_X_10SEC, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
    }
  }
  else {
    tdigit[2]--;
    tft.fillRect(TIMER_X_1Sec, TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
  }
  drawSettings();
}

/*
  Increments the limit by one and draws the weld limit

  @param n/a
  @return +1 to limit and draws
*/
void incrementLimit() {
  if ((incrementA == true) && (incrementB == true)){
    goto increment1000;
  }
  else if (incrementA){
    goto increment100;
  }
  else if (incrementB){
    goto increment10;
  }
  
  if (ldigit[3] == 9) {
    ldigit[3] = 0;
    tft.fillRect(LIMIT_X + LIMIT_W*3 - 1, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);

    increment10:
    if (ldigit[2] == 9) {
      ldigit[2] = 0;
      tft.fillRect(LIMIT_X + LIMIT_W*2, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
      
      increment100:
      if (ldigit[1] == 9) {
        ldigit[1] = 0;
        tft.fillRect(LIMIT_X + LIMIT_W, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
  
        increment1000:
        if (ldigit[0] == 9) {
          ldigit[0] = 0;
          tft.fillRect(LIMIT_X, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
        }
        else {
          ldigit[0]++;
          tft.fillRect(LIMIT_X, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
        }
      }
      else {
        ldigit[1]++;
        tft.fillRect(LIMIT_X + LIMIT_W, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
      }
    }
    else {
      ldigit[2]++;
      tft.fillRect(LIMIT_X + LIMIT_W*2, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
    }
  }
  else {
    ldigit[3]++;
    tft.fillRect(LIMIT_X + LIMIT_W*3 - 1, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
  }
  drawSettings();
}

/*
  Decrements the limit by one and draws the weld limit

  increment and decrement code structure is similar to above

  @param n/a
  @return -1 to limit and draws
*/
void decrementLimit() {
  /*
    A  B  | Outcome
    ---------------
    F  F  | +1
    F  T  | +10
    T  F  | +100
    T  T  | +1000
  */
  if ((incrementA == true) && (incrementB == true)){
    goto decrement1000;
  }
  else if (incrementA){
    goto decrement100;
  }
  else if (incrementB){
    goto decrement10;
  }

  if (ldigit[3] == 0) {
    ldigit[3] = 9;
    tft.fillRect(LIMIT_X + LIMIT_W*3 - 1, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
    
    decrement10:
    if (ldigit[2] == 0) {
      ldigit[2] = 9;
      tft.fillRect(LIMIT_X + LIMIT_W*2, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
  
      decrement100:
      if (ldigit[1] == 0) {
        ldigit[1] = 9;
        tft.fillRect(LIMIT_X + LIMIT_W,LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
        
        decrement1000:
        if (ldigit[0] == 0) {
          ldigit[0] = 9;
          tft.fillRect(LIMIT_X, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
        }
        else {
          ldigit[0]--;
          tft.fillRect(LIMIT_X, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
        }
      }
      else {
        ldigit[1]--;
        tft.fillRect(LIMIT_X + LIMIT_W, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
      }
    }
    else {
      ldigit[2]--;
      tft.fillRect(LIMIT_X + LIMIT_W*2, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
    }
  }
  else{
    ldigit[3]--;
    tft.fillRect(LIMIT_X + LIMIT_W*3 - 1, LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
  }
  drawSettings();
}
 
//why assign to touch screen fields
void increment() {
  if (changeLimit){
    incrementLimit();
  }
  else{
    incrementTimer();
  }
}

//why assign to touch screen fields
void decrement() {
  if (changeLimit){
    decrementLimit();
  }
  else{
    decrementTimer();
  }
}

/*******************************************************************
*                               SETUP                              *
********************************************************************/
boolean isLimit = false;    //debug

void setup() {
  Serial.begin(9600);
  tft.begin();      //init tft
  if (!ts.begin()) {  //debug for screen
    Serial.println("Unable to start touchscreen.");
  }
  else {
    Serial.println("Touchscreen started.");
  }

  noInterrupts();   //Arduino function that turns off interrupts

  //pin D2 for end of weld interrupt  
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);  //active high
  attachInterrupt(0, endofWeld, FALLING); //interrupt 0 is assigned to endofWeld ISR at the falling edge
  
  //pin D3 for alarm interrupt  
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);
  attachInterrupt(1, soundAlarm, FALLING);

  //pin A2 for weld limit scan, Arduino only has 2 interrupts, need A2 for scanning 3rd relay (pin 6 of UB25)
  pinMode(A2, INPUT);
  digitalWrite(A2, LOW);    //active low

  //turn on inhibit pin to FET that controls whether the UB25 is inhibit or not
  digitalWrite(A3, LOW);
  pinMode(A3, OUTPUT);

  //turn on alarm pin to FET that controls alarm buzzer
  digitalWrite(5, LOW);
  pinMode(5, OUTPUT);

  tft.fillScreen(ILI9341_WHITE);  //fill screen with white
  tft.setRotation(1);             //set touchscreen has horizonital view

  //Unused Arduinos EEPROM data initalizes to 0xFF so this initalize first 14 EEPROM address to 0
  if (EEPROM.read(FIRST_SETUP_ADDRESS) == 255){
    for (int i = 0; i < 13; i++){
      EEPROM.write(i, 0);     //write 0 to every used address
    }
  }

  else{
    //set variables to memory values
    for (int i = TIMER_START_ADDRESS; i <= TIMER_END_ADDRESS; i++) {
      tdigit[i] = EEPROM.read(i);
      delay(5);   //delay needed for W/R, just for setup
    }
    for (int i = COUNTER_START_ADDRESS; i < COUNTER_END_ADDRESS; i++) {
      cdigit[i-COUNTER_START_ADDRESS] = EEPROM.read(i);   //i - COUNTER_START_ADDRESS to start at index 0
      delay(5); 
    }
    for (int i = LIMIT_START_ADDRESS; i < LIMIT_END_ADDRESS; i++) {
      ldigit[i-LIMIT_START_ADDRESS] = EEPROM.read(i);     //i - LIMIT_START_ADDRESS to start at index 0
      delay(5);
    }
    inhibitStatus = EEPROM.read(INHIBIT_ADDRESS);        //whether it was inhibited before boot or not
  }

  /* quick overview of arrays again: 
     cdigit and ldigit, reads left to right {1, 5, 0, 0} = 1500
     tdigit, also reads left to right {2, 3, 0} = 2:30 minutes:seconds
     
     alarmduration takes each index of tdigit and convert the time to ms duration
     alarmDuration = how many minutes(tdigit[0]) * 60 secs * 1000 ms +
                     how many 10 seconds(tdigit[1]) * 10 sec * 1000 ms +
                     how many seconds(tdigit[2]) * 1 sec * 1000 ms = total duration in ms 

     countSum and limitSum is similar... i.e.
     limitSum = how many 1000(ldigit[0]) * 1000 +
                how many 100(ldigit[1]) * 100 +
                how many 10(ldigit[2]) * 10 +
                how many 1(ldigit[3]) * 1 = how many counts for the limit
  */
  alarmDuration = (long)tdigit[0]*60*1000 + (long)tdigit[1]*10*1000 + (long)tdigit[2]*1000; //ms
  int limitSum = ldigit[0]*1000 + ldigit[1]*100 + ldigit[2]*10 + ldigit[3];
  int countSum = cdigit[0]*1000 + cdigit[1]*100 + cdigit[2]*10 + cdigit[3];
  
  //see if counter is past limit from counter values
  if (countSum >= limitSum) {
    thresholdReached = true;
  }

  if (inhibitStatus){
    digitalWrite(INHIBIT_FET, HIGH);  //high to inhibit welder
  }

  changeScreen();
  Serial.println("end of setup");

  //clear interrupt flags by writing to the processor
  EIFR |= 0x01;
  EIFR  |= 0x02;
  interrupts();   //enable interrupts 
}

/*******************************************************************
*                                LOOP                              *
********************************************************************/
void loop() {
  /* Because Arduino only has 2 interrupts, the 3rd interrupt from the relay needs to 
     be scanned for. The main loop would continously scan if the pin is high or not
  */
  if (digitalRead(A2) == HIGH){    //50 ms debounce explain
    if (((long)(micros() - last_micros) >= DEBOUNCING_TIME * 1000) && (startScreen == true)) { //debounce logic
      EEPROM.write(INHIBIT_ADDRESS, 1);                       //write to EEPROM setting inhbit is on
      digitalWrite(BUZZER_FET, HIGH);                         //sets buzzer on
      digitalWrite(INHIBIT_FET, HIGH);                        //sets inhibit FET on to stop welder
      alarmOn = true;
      alarmStartTime = millis();
      Serial.println("Limit reached");  //debug
      last_micros = micros();           //debounce keep track of time
      isLimit = true;                   //debug
    }
  } 

  //if the alarm is on and past the alarm duration, turn off the buzzer
  if (alarmOn) {
    if (millis() - alarmStartTime >= alarmDuration){    //keeps tracks of duration
      int alarmEndTime = millis();
      digitalWrite(5, LOW);  //disable the buzzer
      Serial.println("Done!");
      Serial.print("Total time passed: ");
      Serial.println(alarmEndTime - alarmStartTime); //debug
      alarmOn = false;

      //debug
      if (isLimit){
        Serial.println("From Limit");
        isLimit = false;
      }
      else{
        Serial.println("From Alarm");
      }
    }
  }
  
  /* See program flow for higher level flow
     d
  */
  if (startScreen) {  //counter screen
    //Scans for touch
    if (ts.touched()) {
      TS_Point p = ts.getPoint(); //grabbing touch coordinates

      while (!ts.bufferEmpty() || ts.touched()) {
        p = ts.getPoint();
      }

      //maps coordinates to pixel 
      p.x = map(p.x, TS_MINY, TS_MAXY, 0, tft.height());
      p.y = map(p.y, TS_MINX, TS_MAXX, 0, tft.width());
      int y = tft.height() - p.x;
      int x = p.y;
      
      //debug 
      Serial.print("x: ");
      Serial.print(x);
      Serial.print(" y: ");
      Serial.println(y);
      
      //add 30 seconds to reset backdoor flags

      //backdoor 0
      if ((x > TOP_LEFT_BTN_X) && (x < TOP_LEFT_BTN_X + TOP_LEFT_BTN_W) && (y > TOP_LEFT_BTN_Y) && (y < TOP_LEFT_BTN_Y + TOP_LEFT_BTN_H)) {
        if (backDoorCheck[0] == true) {
          backDoorCheck[1] = true;
          Serial.println("double tap worked");
        }
        else{
            boolean backDoorCheck[4] = {false};
        }
        backDoorCheck[0] = true;
      }

      if ((x > TOP_RIGHT_BTN_X) && (x < TOP_RIGHT_BTN_X + TOP_RIGHT_BTN_W) && (y > TOP_RIGHT_BTN_Y) && (y < TOP_RIGHT_BTN_Y + TOP_RIGHT_BTN_H)) {
        if (backDoorCheck[1] == true){
          backDoorCheck[2] = true;
          Serial.println("top right tap worked");

          // //force interrupt for testing
          // Serial.println("Forced alarms");
          // digitalWrite(5, HIGH);
          // digitalWrite(A3, HIGH);
          // alarmOn = true;
          // alarmStartTime = millis();
        }
        else{
          boolean backDoorCheck[4] = {false}; 
        }  
      }
      //add description


      //reset button logic
      if ((x > TOP_RIGHT_BTN_X) && (x < TOP_RIGHT_BTN_X + TOP_RIGHT_BTN_W) && (y > TOP_RIGHT_BTN_Y) && (y < TOP_RIGHT_BTN_Y + TOP_RIGHT_BTN_H) && (thresholdReached == true)) {
        //cdigit length of 4
        for (int i = 0; i < 4; i++){
          cdigit[i] = 0;  //reset counter
        }

        //write to EEPROM address 3-6 because address
        for (int i = 3; i < 7; i++) {
          EEPROM.write(i, 0);
        }

        //reset means resetting threshold
        thresholdReached = false;
        tft.fillRect(COUNTER_X, COUNTER_Y, COUNTER_W * 4, COUNTER_H + 50, ILI9341_WHITE); //draw white square over everything
        drawCounters();
      }

      //rearrange ifs
      if ((x > LEFT_KEY_X) && (x < LEFT_KEY_X + LEFT_KEY_W) && (y > LEFT_KEY_Y) && (y < LEFT_KEY_Y + LEFT_KEY_H)) { 
        if (backDoorCheck[2] == true){
          backDoorCheck[3] = true;
          Serial.println("bottom left tap worked");
        }
        else{
          boolean backDoorCheck[4] = {false};
        } 
      }
      if ((x > RIGHT_KEY_X) && (x < RIGHT_KEY_X + RIGHT_KEY_W) && (y > RIGHT_KEY_Y) && (y < RIGHT_KEY_Y + RIGHT_KEY_H)) { 
        if (backDoorCheck[3] == true){
          Serial.println("bottom right tap worked");
          //explain
          startScreen = false;
          changeScreen();
          boolean backDoorCheck[4] = {false};
        }
        else{
          boolean backDoorCheck[4] = {false};
        } 
      }
    }
  }
  
  else { //system settings
    unsigned long currentMillis = millis();
    if (ts.touched()) {
      TS_Point p = ts.getPoint();

      while (!ts.bufferEmpty() || ts.touched()) {
        p = ts.getPoint();
      }
      p.x = map(p.x, TS_MINY, TS_MAXY, 0, tft.height());
      p.y = map(p.y, TS_MINX, TS_MAXX, 0, tft.width());
      int y = tft.height() - p.x;
      int x = p.y;

      //increment arrow
      if ((x > 10) && (x < 71) && (y > 30) && (y < 106)) {
        increment();
      }
      
      //decrement arrow
      if ((x > 10) && (x <  71) && (y > 115) && (y < 191)) {
        decrement();
      }

      //THIS IS EVERY DIGIT FIELD
      if ((x > TIMER_X_MINUTE) && (x < TIMER_X_MINUTE + TIMER_W) && (y > TIMER_Y) && (TIMER_Y + TIMER_H)){
        if (changeLimit){
          tft.setTextSize(10);
          tft.setCursor(limitLocation[digitState], LIMIT_Y);
          tft.print(ldigit[digitState]);
        }
        else{
          tft.setTextSize(11);
          tft.setCursor(timerLocation[digitState], TIMER_Y);
          tft.print(tdigit[digitState]);
        }
        digitState = 0;
        byMin = true;
        byTenSec = false;
        changeLimit = false;
      }

      if ((x > TIMER_X_10SEC) && (x < TIMER_X_10SEC + TIMER_W) && (y > TIMER_Y) && (TIMER_Y + TIMER_H)){
        if (changeLimit){
          tft.setTextSize(10);
          tft.setCursor(limitLocation[digitState], LIMIT_Y);
          tft.print(ldigit[digitState]);
        }
        else{
          tft.setTextSize(11);
          tft.setCursor(timerLocation[digitState], TIMER_Y);
          tft.print(tdigit[digitState]);
        }
        digitState = 1;
        byMin = false;
        byTenSec = true;
        changeLimit = false;
      }


      if ((x > TIMER_X_1Sec) && (x < TIMER_X_1Sec + TIMER_W) && (y > TIMER_Y) && (TIMER_Y + TIMER_H)){
        if (changeLimit){
          tft.setTextSize(10);
          tft.setCursor(limitLocation[digitState], LIMIT_Y);
          tft.print(ldigit[digitState]);
        }
        else{
          tft.setTextSize(11);
          tft.setCursor(timerLocation[digitState], TIMER_Y);
          tft.print(tdigit[digitState]);
        }
        digitState = 2;
        byMin = false;
        byTenSec = false;
        changeLimit = false;
      }

      if ((x > LIMIT_X) && (x < LIMIT_X + LIMIT_W) && (y > LIMIT_Y) && (LIMIT_Y + LIMIT_H)){
        if (changeLimit){
          tft.setTextSize(10);
          tft.setCursor(limitLocation[digitState], LIMIT_Y);
          tft.print(ldigit[digitState]);
        }
        else{
          tft.setTextSize(11);
          tft.setCursor(timerLocation[digitState], TIMER_Y);
          tft.print(tdigit[digitState]);
        }
        digitState = 0;
        incrementA = true;
        incrementB = true;
        changeLimit = true;
      }

      if ((x > LIMIT_X + LIMIT_W) && (x < LIMIT_X + LIMIT_W*2) && (y > LIMIT_Y) && (LIMIT_Y + LIMIT_H)){
        if (changeLimit){
          tft.setTextSize(10);
          tft.setCursor(limitLocation[digitState], LIMIT_Y);
          tft.print(ldigit[digitState]);
        }
        else{
          tft.setTextSize(11);
          tft.setCursor(timerLocation[digitState], TIMER_Y);
          tft.print(tdigit[digitState]);
        }
        digitState = 1;
        incrementA = true;
        incrementB = false;
        changeLimit = true;
      }

      if ((x > LIMIT_X + LIMIT_W*2) && (x < LIMIT_X + LIMIT_W*3) && (y > LIMIT_Y) && (LIMIT_Y + LIMIT_H)){
        if (changeLimit){
          tft.setTextSize(10);
          tft.setCursor(limitLocation[digitState], LIMIT_Y);
          tft.print(ldigit[digitState]);
        }
        else{
          tft.setTextSize(11);
          tft.setCursor(timerLocation[digitState], TIMER_Y);
          tft.print(tdigit[digitState]);
        }
        digitState = 2;
        incrementA = false;
        incrementB = true;
        changeLimit = true;
      }

      if ((x > LIMIT_X + LIMIT_W*3) && (x < LIMIT_X + LIMIT_W*4) && (y > LIMIT_Y) && (LIMIT_Y + LIMIT_H)){
        if (changeLimit){
          tft.setTextSize(10);
          tft.setCursor(limitLocation[digitState], LIMIT_Y);
          tft.print(ldigit[digitState]);
        }
        else{
          tft.setTextSize(11);
          tft.setCursor(timerLocation[digitState], TIMER_Y);
          tft.print(tdigit[digitState]);
        }
        digitState = 3;
        incrementA = false;
        incrementB = false;
        changeLimit = true;
      }

      //enter button, document explaination
      if ((x > LEFT_KEY_X) && (x < LEFT_KEY_X + LEFT_KEY_W) && (y > LEFT_KEY_Y) && (y < LEFT_KEY_Y + LEFT_KEY_H)){
        alarmDuration = (long)tdigit[0]*60*1000 + (long)tdigit[1]*10*1000 + (long)tdigit[2]*1000;
        for (int i = 0; i < 3; i++){
          EEPROM.write(i, tdigit[i]);
        }
        for (int i = 7; i < 11; i++){
          EEPROM.write(i, ldigit[i-7]);
        }
        int limitSum = ldigit[0]*1000 + ldigit[1]*100 + ldigit[2]*10 + ldigit[3];
        int countSum = cdigit[0]*1000 + cdigit[1]*100 + cdigit[2]*10 + cdigit[3];
        if (countSum >= limitSum) {
          thresholdReached = true;
        }
        else{
          thresholdReached = false;
        }
        startScreen = true;
        changeScreen();  
      }

      //unlock
      if ((x > RIGHT_KEY_X) && (x < RIGHT_KEY_X + RIGHT_KEY_W) && (y > RIGHT_KEY_Y) && (y < RIGHT_KEY_Y + RIGHT_KEY_H)){
        alarmDuration = (long)tdigit[0]*60*1000 + (long)tdigit[1]*10*1000 + (long)tdigit[2]*1000;
        digitalWrite(A3, LOW);  //only thing different is here, turns off inhibit
        for (int i = 0; i < 3; i++){
          EEPROM.write(i, tdigit[i]);
        }
        for (int i = 7; i < 11; i++){
          EEPROM.write(i, ldigit[i-7]);
        }
        EEPROM.write(11, 0);
        int limitSum = ldigit[0]*1000 + ldigit[1]*100 + ldigit[2]*10 + ldigit[3];
        int countSum = cdigit[0]*1000 + cdigit[1]*100 + cdigit[2]*10 + cdigit[3];
        if (countSum >= limitSum) {
          thresholdReached = true;
        }
        else{
          thresholdReached = false;
        }
        startScreen = true;
        changeScreen();        
      }
    }
    //blinking section
    if (currentMillis - previousBlink >= BLINK_TIME) {
      if (blinkOn){
          if (changeLimit){ //limit covering 
            tft.fillRect(limitLocation[digitState], LIMIT_Y, LIMIT_W, LIMIT_H, ILI9341_WHITE);
          }
          else{ //timer covering
              tft.fillRect(timerLocation[digitState], TIMER_Y, TIMER_W, TIMER_H, ILI9341_WHITE);
          }
      }
      else{
          if (changeLimit){
            tft.setTextSize(10);
            tft.setCursor(limitLocation[digitState], LIMIT_Y);
            tft.print(ldigit[digitState]);
          }
          else{
            tft.setTextSize(11);
            tft.setCursor(timerLocation[digitState], TIMER_Y);
            tft.print(tdigit[digitState]);
          }          
      }
      blinkOn = !blinkOn;
      previousBlink = currentMillis; //explain
    }
  }
}
/*******************************************************************
*                                ISR                               *
********************************************************************/
/*
  Weld Count ISR
  Counts the falling (doesn't matter) edge of Relay1 of the UB-25
  Relay1 should be programmed to output whenever a weld has ended
*/
void endofWeld() { //debounce of 50ms
  if (((long)(micros() - last_micros) >= DEBOUNCING_TIME * 1000) && (startScreen == true)) { //debounce logic
    //counter logic
    if (cdigit[3] == 9) {
      cdigit[3] = 0;
      tft.fillRect(COUNTER_X + (COUNTER_W - 1) * 3, COUNTER_Y, COUNTER_W, COUNTER_H, ILI9341_WHITE);  //remove 1st
      if (cdigit[2] == 9) {
        cdigit[2] = 0;
        tft.fillRect(COUNTER_X + (COUNTER_W - 1) * 2, COUNTER_Y, COUNTER_W, COUNTER_H, ILI9341_WHITE); //remove 2nd
        if (cdigit[1] == 9) {
          cdigit[1] = 0;
          tft.fillRect(COUNTER_X + (COUNTER_W - 1), COUNTER_Y, COUNTER_W, COUNTER_H, ILI9341_WHITE);   //remove 3rd
          if (cdigit[0] == 9) {
            cdigit[0] = 0;
            tft.fillRect(COUNTER_X, COUNTER_Y, COUNTER_W, COUNTER_H, ILI9341_WHITE);                  //remove 4th
          }
          else {
            cdigit[0]++;
            tft.fillRect(COUNTER_X, COUNTER_Y, COUNTER_W, COUNTER_H, ILI9341_WHITE);
          }
        }
        else {
          cdigit[1]++;
          tft.fillRect(COUNTER_X + (COUNTER_W - 1), COUNTER_Y, COUNTER_W, COUNTER_H, ILI9341_WHITE);
        }
      }
      else {
        cdigit[2]++;
        tft.fillRect(COUNTER_X + (COUNTER_W - 1) * 2, COUNTER_Y, COUNTER_W, COUNTER_H, ILI9341_WHITE);
      }
    }
    else {
      cdigit[3]++;
      tft.fillRect(COUNTER_X + (COUNTER_W - 1) * 3, COUNTER_Y, COUNTER_W, COUNTER_H, ILI9341_WHITE);
    }

    if (!thresholdReached){
      //check if it's past the limit
      int limitSum = ldigit[0]*1000 + ldigit[1]*100 + ldigit[2]*10 + ldigit[3];
      int countSum = cdigit[0]*1000 + cdigit[1]*100 + cdigit[2]*10 + cdigit[3];
      if (countSum >= limitSum) {
        thresholdReached = true;
        tft.fillRect(COUNTER_X, COUNTER_Y, COUNTER_W * 4, COUNTER_H + 50, ILI9341_WHITE);
      }
    }
    //writes to EEPROM of current weld count
    for (int i = 3; i < 7; i++){
       EEPROM.write(i, cdigit[i-3]);
    }
    drawCounters();
    last_micros = micros();
  }
}

/*
  Alarm ISR. 
  Interrupts from Relay2 (alarm relay)
*/
void soundAlarm() {
  EEPROM.write(INHIBIT_ADDRESS, 1);  //Store condition
  Serial.println("Alarm reached");
  digitalWrite(BUZZER_FET, HIGH);
  digitalWrite(INHIBIT_FET, HIGH);
  alarmOn = true;
  alarmStartTime = millis();
}