
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <RTClib.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SoftwareSerial.h>

# define Start_Byte 0x7E
# define Version_Byte 0xFF
# define Command_Length 0x06
# define End_Byte 0xEF
# define Acknowledge 0x00 //Returns info with command 0x41 [0x01: info, 0x00: no info]
# define ACTIVATED LOW
#define rs 7 
#define en 8 
#define d4 3
//3
#define d5 12
//12  
#define d6 11 
#define d7 4 

SoftwareSerial mySerial(17, 16);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  

SemaphoreHandle_t btnSemaphore;
RTC_DS3231 rtc;

byte ss=0, mi=0, hh=0, wd=6, dd=1, mo=1;
byte yy=0;
//uint16_t yy=0;
uint16_t yy2=0;
const int buttonNext = 48, buttonPlay = 51, buttonPause = 52, buttonPrevious = 50, trigPin = 42, echoPin = 40, 
          rightF = 9, rightB = 10, leftF = 5, leftB = 6, buzzer=32, ledPin =  46, ldrPin =  A2, brightness = 7; 

boolean firstTime, isPlaying = false;        
float volts=0.0, temp=0.0, tmp=0.0;

String JSstate, prevState;
int vel = 255, state = 'g', distance, ldrState = 0, value=0; // start stopped
long duration;
int trackSem = 0;

void TaskDrive(void *pvParameters);
void TaskDisplay(void *pvParameters);
void TaskPlaySongs(void *pvParameters);
void TaskLightDetection(void *pvParameters);
void digitalClockDisplay(void *pvParameters);


void setup() { 

Serial.begin(9600); // start the serial port for communication with the Bluetooth  
Serial1.begin(9600);
mySerial.begin (9600);                           
lcd.begin(16,2);      // set up the LCD's number of columns and rows
Wire.begin();

pinMode(rightF, OUTPUT);
pinMode(rightB, OUTPUT);
pinMode(leftF, OUTPUT);
pinMode(leftB, OUTPUT);
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT); 
pinMode(buzzer,OUTPUT);
pinMode(ledPin, OUTPUT);
pinMode(ldrPin, INPUT);
pinMode(buttonPause, INPUT);
digitalWrite(buttonPause,HIGH);
pinMode(buttonNext, INPUT);
digitalWrite(buttonNext,HIGH);
pinMode(buttonPrevious, INPUT);
digitalWrite(buttonPrevious,HIGH);
pinMode(buttonPlay, INPUT);
digitalWrite(buttonPlay,HIGH);
pinMode(13, OUTPUT);

JSstate = "N";
   value=analogRead(A15);          //read from A15
  volts=(value/1024.0)*5.0;      //conversion to volts
  temp= volts*100.0;             //conversion to temp Celsius
  lcd.setCursor(9,1);
  //lcd.print("TEMP= ");
  lcd.print(temp);
  lcd.print(" C");
//-------------------------------------freeRTOS---------------------------------------------------
btnSemaphore = xSemaphoreCreateBinary();

 xTaskCreate(
    TaskDrive
    ,  (const portCHAR *)"Drive"  
    ,  128  
    ,  NULL
    ,  2  
    ,  NULL );

 xTaskCreate(
    TaskPlaySongs
    ,  (const portCHAR *)"PlaySongs"  
    ,  1000  
    ,  NULL
    ,  2  
    ,  NULL );
    
 xTaskCreate(
    TaskDisplay
    ,  (const portCHAR *)"DisplayDateAndTime"  
    ,  1000
    ,  NULL
    ,  2  
    ,  NULL );

 xTaskCreate(
    TaskLightDetection
    ,  (const portCHAR *)"DisplayLightDetection"  
    ,  1000 
    ,  NULL
    ,  2  
    ,  NULL );


} 


//-----------------------------------------------------------------------Task Drive + Helper methods-----------------------------------------------------------------------------------------
void Left(){
  if(distance<25){
  digitalWrite(buzzer,HIGH);
  }
  if(distance>25){
     digitalWrite(buzzer,LOW);
    }
 Serial.println("L");
 analogWrite(rightB, 0); 
 analogWrite(leftB, 0);
 analogWrite(leftF, 0);
 analogWrite(rightF, vel);
    } 

void Right(){
  if(distance<25){
  digitalWrite(buzzer,HIGH);
  }
  if(distance>25){
     digitalWrite(buzzer,LOW);
    }
 Serial.println("R");
 analogWrite(rightB, 0); 
 analogWrite(leftB, 0); 
 analogWrite(rightF, 0); 
 analogWrite(leftF, vel);
    }
     
void Forward(){
  if(distance<25){
  digitalWrite(buzzer,HIGH);
  }
  if(distance>25){
     digitalWrite(buzzer,LOW);
    }
 Serial.println("F");
 analogWrite(rightB, 0); 
 analogWrite(leftB, 0); 
 analogWrite(rightF, vel); 
 analogWrite(leftF, vel); 
  }

void Backward(){
 Serial.println("B");
 analogWrite(rightF, 0); 
 analogWrite(leftF, 0);
 analogWrite(rightB, vel); 
 analogWrite(leftB, vel); 
}

  void Stop(){
analogWrite(rightF, 0); 
analogWrite(leftF, 0);
analogWrite(rightB, 0); 
analogWrite(leftB, 0);
  } 

void TaskDrive(void *pvParameters){
  while(1)
  {
  Serial.print("-TaskDisplayDrive-");
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance= duration*0.034/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  if(distance<10){  //Stop when detecting that a crash might happen
   Stop();
   digitalWrite(buzzer,LOW);
  if(state=='b'){ // Reverse
   Serial1.println(state);
   Backward();
   delay(100);  
   Serial.print(distance);
   Serial.println("Stopped");
  }
}
  else{
   if(Serial1.available()>0){ // read the bluetooth and store in state
    state = Serial1.read();
    }
   if(state=='f' ){ // Forward
    Serial1.println(state);
    Forward();
    delay(100);
    }
  if(state=='r'){ // right
    Serial1.println(state);
    Right();
    delay(100);
    }
    
  if(state=='s'){ // Stop
    Serial1.println(state);
    Stop();
    delay(100);
    }
  
  if(state=='l'){ // left
    Serial1.println(state);
    Left();
    delay(100);
    } 
  if(state=='b'){ // Reverse
   Serial1.println(state);
   Backward();
   delay(100);
   }
  }
 }
}


//-----------------------------------------------------------Display Time & Date, Current Gear and Temperature + Helper methods------------------------------------------------------

void calcTemp(void* parameters){
  const TickType_t xDelay = 10*pdMS_TO_TICKS ( 1000 );
  while(1){
value=analogRead(A15);          //read from A15
  volts=(value/1024.0)*5.0;      //conversion to volts
  temp= volts*100.0;             //conversion to temp Celsius
  vTaskDelay(xDelay);

  }
}


byte bcd2bin(byte x) {
  // converts from binary-coded decimal to a "regular" binary number
  return ((((x >> 4) & 0xF) * 10) + (x & 0xF)) ;
}


void printTime() {
  Serial.print ("\'");
  Serial.print(yy,DEC);
   Serial.print(yy2,DEC); Serial.print("-");
  Serial.print("sddd  "); Serial.println(wd);
  if (mo<10) Serial.print("0"); Serial.print(mo,DEC); Serial.print("-");
  if (dd<10) Serial.print("0"); Serial.print(dd,DEC); Serial.print("(");
  switch (wd) {
    case 1: Serial.print("Mon"); break;
    case 2: Serial.print("Tue"); break;
    case 3: Serial.print("Wed"); break;
    case 4: Serial.print("Thu"); break;
    case 5: Serial.print("Fri"); break;
    case 6: Serial.print("Sat"); break;
    case 7: Serial.print("Sun"); break;
    default: Serial.print(""); 
  }
  Serial.print(") ");
  if (hh<10) Serial.print("0"); Serial.print(hh,DEC); Serial.print(":");
  if (mi<10) Serial.print("0"); Serial.print(mi,DEC); Serial.print(":");
  if (ss<10) Serial.print("0"); Serial.print(ss,DEC); Serial.println("");

//---------------------------LCD-----------------------------------------------
  lcd.clear();   
  switch (wd) {
    case 1: lcd.print("Mon "); break;
    case 2: lcd.print("Tue "); break;
    case 3: lcd.print("Wed "); break;
    case 4: lcd.print("Thu "); break;
    case 5: lcd.print("Fri "); break;
    case 6: lcd.print("Sat "); break;
    case 7: lcd.print("Sun "); break;
    default: lcd.print(""); 
  }
  if (dd<10) lcd.print("0"); lcd.print(dd,DEC);  lcd.print("-");
  if (mo<10) lcd.print("0"); lcd.print(mo,DEC);  lcd.print("-");
 // lcd.print ("\'");
  lcd.print(yy2,DEC); 
  
    lcd.setCursor(0,1);
  if(hh>12) hh = hh-12;
  if (hh<10) lcd.print("0"); lcd.print(hh,DEC); lcd.print(":");
  if (mi<10) lcd.print("0"); lcd.print(mi,DEC); lcd.print(":");
  if (ss<10) lcd.print("0"); lcd.print(ss,DEC);

}


void TaskDisplay(void *pvParameters){
   const TickType_t xDelay = pdMS_TO_TICKS ( 1000 );
   int sensorX =0;
   int sensorY = 0;
   float angleX = 0.0;
   float angleY = 0.0;
   
   while(1){
    //1- Time & Date measurement & display
    //Serial.println("TaskDisplayDateAndTime");
   Wire.beginTransmission(0x68); // send request to receive data starting at register 0 // 0x68 is DS3231 device address
   Wire.write((byte)0); // start at register 0
   Wire.endTransmission();
   Wire.requestFrom(0x68, 7); // request seven bytes (ss, mi, hh, wd, dd, mo, yy)  
  if (Wire.available() >= 7) { // check for a reply from the RTC, and use it if we can
    ss = bcd2bin(Wire.read()); // get seconds
    mi = bcd2bin(Wire.read()); // get minutes
    hh = bcd2bin(Wire.read()); // get hours
    wd = bcd2bin(Wire.read());
    dd = bcd2bin(Wire.read());
    mo = bcd2bin(Wire.read());
    yy = bcd2bin(Wire.read()); //+2000;
    yy2 = yy+2000;
    //Serial.print("Date/Time: ");
    printTime();
  }
  else {
    //Unable to read the time
    Serial.println("Unable to read time from RTC");
  } 
  // 2- Joy stick mesurement & display
   prevState = JSstate;
   sensorX = analogRead(A0); //get readings for x,y-axis movement
   sensorY = analogRead(A1);
   angleX = sensorX * (180.0 / 1023.0); //convert value to angle
   angleY = sensorY * (180.0 / 1023.0);
   //Serial.print("JOYSTICKx= "); 
   //Serial.println(angleX);
   // Serial.print("JOYSTICKy= "); 
   // Serial.println(angleY);
   lcd.setCursor(15, 0);
   if (angleX > 100) {
      JSstate = "R";
   } else if ( angleX < 70) {
      JSstate = "D";
   } else {
     if (angleY > 105){
       JSstate = "N";
     } else if(angleY < 75) {
       JSstate = "P";
   }
      else if(angleX >=89 && angleY >= 88) 
      JSstate=prevState;
  }
 // Serial.println(JSstate);
 // Serial.println(prevState);
  lcd.print(JSstate);
 
  // 3- Temperature measurement & display
  lcd.setCursor(9,1);
  //lcd.print("TEMP= ");
  lcd.print(temp);
  lcd.print(" C");
  
  vTaskDelay(xDelay);
  }
}


//-------------------------------------------------------------------------Task Light Detection----------------------------------------------------------------------------------------------
void TaskLightDetection(void *pvParameters){
  //TickType_t xLastWakeTime;
  //xLastWakeTime= xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS ( 100 );

 while(1){
    Serial.println("TaskLIGHT");

   ldrState = analogRead(ldrPin);
   if (ldrState >= brightness) {                        
     digitalWrite(ledPin, LOW);
  }
  else if (ldrState < brightness) {                        
     digitalWrite(ledPin, HIGH);                     

  }
  Serial.print("LDR Value = ");                 
  Serial.println( ldrState );
  //delay(500);
  //    vTaskDelayUntil( xLastWakeTime , pdMS_TO_TICKS ( 50 ));
  vTaskDelay(xDelay);

}
}


//--------------------------------------------------------------------Task Play Songs Helper Methods-----------------------------------------------------------------------------------------
void playFirst()
{
  execute_CMD(0x3F, 0, 0);
  delay(500);
  setVolume(30);
  delay(500);
  execute_CMD(0x11,0,1); 
  delay(500);
}
void pause()
{
  execute_CMD(0x0E,0,0);
  delay(500);
}
void play()
{
  execute_CMD(0x0D,0,1); 
  //delay(500);
}
void playNext()
{
  execute_CMD(0x01,0,1);
  delay(500);
}
void playPrevious()
{
  execute_CMD(0x02,0,1);
  delay(500);
}
void setVolume(int volume)
{
  execute_CMD(0x06, 0, volume); // Set the volume (0x00~0x30)
  delay(2000);
}
void execute_CMD(byte CMD, byte Par1, byte Par2)
// Excecute the command and parameters
{
// Calculate the checksum (2 bytes)
word checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + Par1 + Par2);
// Build the command line
byte Command_line[10] = { Start_Byte, Version_Byte, Command_Length, CMD, Acknowledge,
Par1, Par2, highByte(checksum), lowByte(checksum), End_Byte};
//Send the command line to the module
for (byte k=0; k<10; k++)
{
mySerial.write( Command_line[k]);
}
}

void TaskPlaySongs(void *pvParameters){
  firstTime = true;
  while(1){
    Serial.println("-TaskPlaySongs-");
    if(firstTime && digitalRead(buttonPlay) == ACTIVATED){
//      Serial.println("firstTime");
      playFirst();
      isPlaying = true;
      firstTime = false;
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("Playing song =D");
      delay(900);
      if(trackSem==0){
      xSemaphoreGive ( btnSemaphore); 
      trackSem=1;
//        Serial.print("SEM GIVEN BY PLAY trackSem :::::::: ");
//        Serial.println(trackSem);
      }
    }
    //PLAY:
   else if (digitalRead(buttonPlay) == ACTIVATED )
    {
//      if(isPlaying)
//      {
        Serial.println("^----PLAY----^");
        // xSemaphoreTake ( btnSemaphore,  portMAX_DELAY);
        isPlaying =true;
      if(trackSem==0){ 
        xSemaphoreGive ( btnSemaphore); 
        trackSem=1;
        Serial.print("SEM GIVEN BY PLAY trackSem: ");
        Serial.println(trackSem);
        };
         play();

        
     // }
  }
    else if(digitalRead(buttonPause) == ACTIVATED ){
//       if(!isPlaying)
//       {
        //When song is paused, semaphore should not be available for next and previous buttons
        Serial.println("^---PAUSE---^");

        if(trackSem==1){
        xSemaphoreTake ( btnSemaphore, 999999999999999999999);
                trackSem=0;
        Serial.print("SEM TAKEN BY PLAY trackSem : ");
        Serial.println(trackSem);
        }
        pause(); 
     //  Serial.println("SEM TAKEN BY PAUSE");
       // xSemaphoreGive ( btnSemaphore);
        
        isPlaying = false;
//       }
     }
   else if (digitalRead(buttonNext) == ACTIVATED)
    {
    if(isPlaying)
    { 
      lcd.clear();
      lcd.setCursor(2,0);
      lcd.print("Playing next");
      lcd.setCursor(4,1);     
      lcd.print("song =D");
      delay(900);
      //if song is paused, semaphore can not be taken
             if(trackSem==1){
        xSemaphoreTake ( btnSemaphore, 999999999999999999999);
                trackSem=0;
        Serial.print("SEM TAKEN BY NEXT trackSem :::::::: ");
        Serial.println(trackSem);
        }
       Serial.println("^---NEXT---^");
       playNext();
       if(trackSem==0){
       xSemaphoreGive ( btnSemaphore);
        trackSem=1;
      Serial.print("SEM GIVEN BY NEXT trackSem:");
        Serial.println(trackSem);
       }
    }
  }
 else  if (digitalRead(buttonPrevious) == ACTIVATED)
  {
    if(isPlaying)
  {      
          lcd.clear();
      lcd.setCursor(2,0);
      lcd.print("Previous song");
      lcd.setCursor(0,1);     
      lcd.print("being played :D");
      delay(900);
       //if song is paused, semaphore can not be taken
            if(trackSem==1){
        xSemaphoreTake ( btnSemaphore, 999999999999999999999);
                trackSem=0;
        Serial.print("SEM TAKEN BY PRE trackSem : ");
        Serial.println(trackSem);
        }
    //  xSemaphoreTake ( btnSemaphore,999999999999999999999);
    //  Serial.println("SEM TAKEN BY PRE");
      Serial.println("^---PREVIOUS---^");
      playPrevious();
      if(trackSem==0){
        xSemaphoreGive ( btnSemaphore);
        trackSem=1;
      Serial.print("SEM GIVEN BY PRE trackSem : ");
        Serial.println(trackSem);
      }
    }
  }
}
}



void loop() { 
}
