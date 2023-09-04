#include <math.h> 
#include <Keypad.h>
//Keypad
const byte ROWS = 4;
const byte COLS = 3;
char hexaKeys[ROWS][COLS] = 
{
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
byte rowPins[ROWS] = {13, 12, 11, 10};
byte colPins[COLS] = {9, 8, 7};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
String inputString;
long inputInt;
//Flags
char inputTaken = 1;
char rotated = 1;
char xDone = 1;
char moved1 = 1;
char moved2 = 1;
//Encoder reading vatriables
unsigned int rpm1 = 0; 
unsigned int rpm2 = 0; 
int target;         
float velocity1=0;                  //Velocidad en [Km/h]
float velocity2=0; 
volatile int pulses1 = 0;  
volatile int pulses2 = 0;  
static volatile unsigned long debounce1 = 0;
static volatile unsigned long debounce2 = 0;
//position variables
int x;
int y;
signed int theta;
int ref=0;
int dist=0;
//PID gains
double Kp=13.83;
double Kd=7.3;
double Ki=2;
//1st Motor
signed int e1=0;
float PID1_Kp=0;
float PID1_Kd=0;
float PID1_Ki=0;
int PrevE1=0;
int PWM1=0;
int PWMmap1=0;
float PID1=0;
//2nd Motor
signed int e2=0;
float PID2_Kp=0;
float PID2_Kd=0;
float PID2_Ki=0;
int PrevE2=0;
int PWM2=0;
int PWMmap2=0;
float PID2=0;  
unsigned long prevTime = 0;
unsigned long timeold1 = 0;
unsigned long timeold2 = 0;  
unsigned long time1 = 0;
unsigned int pulsesperturn = 20; 
const int wheel_diameter = 64; 
void setup() {
  Serial.begin(9600); // Configuración del puerto serie  
  pinMode(14,OUTPUT);
  pinMode(15,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(16,OUTPUT);
  pinMode(17,OUTPUT);
  pinMode(18,OUTPUT);
  digitalWrite(14,LOW);
  digitalWrite(15,HIGH);
  digitalWrite(16,LOW);
  digitalWrite(17,HIGH);
  attachInterrupt(0, counter1, RISING);
  attachInterrupt(1, counter2, RISING);
  x=400;
  y=400;
  
 
}
void counter1(){
  if( digitalRead (2) && (micros()-debounce1 > 500) && digitalRead (2) ) 
       { 
        debounce1 = micros(); 
        pulses1++;
        }  
        else ; 
        } 
void counter2(){
  if( digitalRead (3) && (micros()-debounce2 > 500) && digitalRead (3) ) 
       { 
        debounce2 = micros(); 
        pulses2++;
        }  
        else ; 
        } 
void loop() {
  if(inputTaken)
  {
    char customKey = customKeypad.getKey();
    if(xDone){
    if (customKey =='0' ||customKey =='1'||customKey =='2'||customKey =='3'||customKey =='4'
    || customKey =='5'||customKey =='6'||customKey =='7'||customKey =='8'||customKey =='9') {     // only act on numeric keys
      inputString += customKey;  
      //Serial.println(inputString);// append new character to input string
    } else if (customKey == '*') {
      if (inputString.length() > 0) {
        inputInt = inputString.toInt(); // YOU GOT AN INTEGER NUMBER     
        //Serial.println(inputString);
        x = inputInt;
        Serial.print("x = ");
        Serial.println(x);
        inputString = "";                 // clear input
        xDone = 0;
        delay(50);
        
      }
    }
    }
    else if(!xDone) {
    if (customKey =='0' ||customKey =='1'||customKey =='2'||customKey =='3'||customKey =='4'
    || customKey =='5'||customKey =='6'||customKey =='7'||customKey =='8'||customKey =='9') {     // only act on numeric keys
      inputString += customKey;  
      //Serial.println(inputString);// append new character to input string
    } else if (customKey == '*') {
      if (inputString.length() > 0) {
        inputInt = inputString.toInt(); // YOU GOT AN INTEGER NUMBER
        //Serial.println(inputString);
        y = inputInt;
        Serial.print("y = ");
        Serial.println(y);
        inputString = "";                 // clear input
        xDone = 1;
        delay(50);
      
    }
    if(y == 0){
      theta = -1;
    }
    else{
      theta = x/y;
    }
     inputTaken = 0; 
     delay(5000);
    }
    
  }
  
  }
  //encoder readings
  time1 = millis();
  if (millis() - timeold1 >= 1000){  // Se actualiza cada segundo
      noInterrupts(); //Don't process interrupts during calculations // Desconectamos la interrupción para que no actué en esta parte del programa.
      rpm1 = (60UL * 1000 / pulsesperturn )/ (millis() - timeold1)* pulses1; // Calculamos las revoluciones por minuto
      velocity1 = rpm1 * 3.1416 * wheel_diameter * 60 / 1000000; // Cálculo de la velocidad en [Km/h] 
      timeold1 = millis(); // Almacenamos el tiempo actual.
      interrupts();
  }
      
  if (millis() - timeold2 >= 1000){  // Se actualiza cada segundo
      noInterrupts(); //Don't process interrupts during calculations // Desconectamos la interrupción para que no actué en esta parte del programa.
      rpm2 = (60UL * 1000 / pulsesperturn )/ (millis() - timeold2)* pulses2; // Calculamos las revoluciones por minuto
      velocity2 = rpm2 * 3.1416 * wheel_diameter * 60 / 1000000; // Cálculo de la velocidad en [Km/h] 
      timeold2 = millis(); // Almacenamos el tiempo actual.
      interrupts();
  }
  // Car rotation
  if(rotated & !inputTaken){
    Serial.println(theta);
    if(theta<0){
        target = (175/(4*3.1416))*(1.5708);
      }
      else{
        target = (175/(4*3.1416))*atan(theta);
      }
      while(pulses1 < target){
        //Serial.print(2);
        digitalWrite(14,LOW);
        digitalWrite(15,HIGH);
        analogWrite(5,160);
   }
        analogWrite(5,0);
        pulses1 = 0;
        pulses2 = 0;
        rotated = 0;
        delay(2500);
  }
  
 // Moving in straight line   
 if(!rotated & !inputTaken){   
  dist = sqrt((pow(x,2) + pow(y,2)));
  ref = (dist/(6.2832*32))*20;
 
  //PID1 readings
  e1 = ref - pulses1;
  PID1_Kp = Kp*e1;
  PID1_Ki = PID1_Ki +(Ki*e1);
  PID1_Kd  = Kd*(e1-PrevE1)/(time1-prevTime);
  PID1 = PID1_Kp + PID1_Ki + PID1_Kd;
  
    if(PID1 < 0){
    PWM1 = -1*PID1;
    digitalWrite(14,HIGH);
    digitalWrite(15,LOW);
    digitalWrite(16,LOW);
    digitalWrite(17,HIGH);
  }
  else{
    PWM1 = PID1;
    digitalWrite(14,LOW);
    digitalWrite(15,HIGH);
    digitalWrite(16,LOW);
    digitalWrite(17,HIGH);
    Serial.println(PID1);
  }
  
  //PID2 readings
  e2 = ref - pulses2;
  PID2_Kp = Kp*e2;
  PID2_Ki = PID2_Ki +(Ki*e2);
  PID2_Kd  = Kd*(e2-PrevE2)/(time1-prevTime);
  PID2 = PID2_Kp + PID2_Ki + PID2_Kd;
    if(PID2 < 0){
    PWM2 = -1*PID2;
    digitalWrite(14,LOW);
    digitalWrite(15,HIGH);
    digitalWrite(16,HIGH);
    digitalWrite(17,LOW);
  }
  else{
    PWM2 = PID2;
    digitalWrite(14,LOW);
    digitalWrite(15,HIGH);
    digitalWrite(16,LOW);
    digitalWrite(17,HIGH);
  }
  //Motor1
if(!(-1 < e1 <1)){
    analogWrite(5,PWM1);
  }
  else{
    analogWrite(5,0);
    moved1 = 0;
  }
  //Motor2
  if(!(-1 < e2 <1)){
    analogWrite(6,PWM2);
  }
  else{
    analogWrite(6,0);
    moved2 = 0;

  }
  prevTime = time1; 
  PrevE1 = e1;
 PrevE2 = e2;
 }
if((!moved1) & (!moved2) )
{ 
    delay(1000);
    
    inputTaken = 1;
    rotated = 1;
    xDone = 1;
    moved1 = 1;
    moved2 = 1;
    pulses1 = 0;
    pulses2 = 0;
}

}
