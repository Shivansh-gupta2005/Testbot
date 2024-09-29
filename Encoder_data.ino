#include <util/atomic.h> // For the ATOMIC_BLOCK macro

//Encoder Connections
#define ENCA1 4 
#define ENCB1 5
#define ENCA2 10
#define ENCB2 11
#define ENCA3 35
#define ENCB3 34
#define ENCA4 36
#define ENCB4 37

//MotorDriver Connections

#define DIR1 2
#define PWM1 3
#define DIR2 6
#define PWM2 9
#define DIR3 20
#define PWM3 22
#define DIR4 21
#define PWM4 23




volatile int posi1 = 0; 
volatile int posi2 = 0;
volatile int posi3 = 0;
volatile int posi4 = 0;


void setup() {
  Serial.begin(9600);
  pinMode(ENCA1,INPUT);
  pinMode(ENCB1,INPUT);
  pinMode(ENCA2,INPUT);
  pinMode(ENCB2,INPUT);
  pinMode(ENCA3,INPUT);
  pinMode(ENCB3,INPUT);
  pinMode(ENCA4,INPUT);
  pinMode(ENCB4,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA3),readEncoder3,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA4),readEncoder4,RISING);
  
  pinMode(PWM1,OUTPUT);
  pinMode(DIR1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(DIR3,OUTPUT);
  pinMode(PWM4,OUTPUT);
  pinMode(DIR4,OUTPUT);
  
  
  
}

void loop() {






  int pos1 = 0; 
  int pos2 = 0;
  int pos3 = 0;
  int pos4 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = posi1;
    pos2 = posi2;
    pos3 = posi3;
    pos4 = posi4;
  }
  



 
  Serial.println(pos1   );
  Serial.println(pos2   );
  Serial.println(pos3   );
  Serial.println(pos4   );
  
}


void readEncoder1(){
  int b = digitalRead(ENCB1);
  if(b > 0){
    posi1++;
  }
  else{
    posi1--;
  }
}


void readEncoder2(){
  int b = digitalRead(ENCB2);
  if(b > 0){
    posi2++;
  }
  else{
    posi2--;
  }
}


void readEncoder3(){
  int b = digitalRead(ENCB3);
  if(b > 0){
    posi3++;
  }
  else{
    posi3--;
  }
}


void readEncoder4(){
  int b = digitalRead(ENCB4);
  if(b > 0){
    posi4++;
  }
  else{
    posi4--;
  }
}
