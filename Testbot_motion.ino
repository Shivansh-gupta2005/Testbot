#define DIR1 2
#define PWM1 3
#define DIR2 6
#define PWM2 9
#define DIR3 20
#define PWM3 22
#define DIR4 21
#define PWM4 23


void setup(){
  Serial.begin(9600);
  pinMode(DIR1,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(DIR3,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(DIR4,OUTPUT);
  pinMode(PWM4,OUTPUT);
  digitalWrite(DIR1,LOW);
  digitalWrite(PWM1,LOW);
  digitalWrite(DIR2,LOW);
  digitalWrite(PWM2,LOW);
  digitalWrite(DIR3,LOW);
  digitalWrite(PWM3,LOW);
  digitalWrite(DIR4,LOW);
  digitalWrite(PWM4,LOW);
}

void loop(){
  moveforward();
  delay(8000);
  stopmotors();
  delay(1500);
  movebackward();
  delay(8000);
  stopmotors();
  delay(1500);
  moveleft();
  delay(8000);
  stopmotors();
  delay(1500);
  moveright();
  delay(8000);
  stopmotors();
  delay(1500);
   spinleft();
  delay(8000);
  stopmotors();
  delay(1500);
  spinright();
  delay(8000);
  stopmotors();
  delay(1500);

}


void moveforward(){
  digitalWrite(DIR1,HIGH);
  digitalWrite(DIR2,LOW);
  digitalWrite(DIR3,  LOW);
  digitalWrite(DIR4,LOW);
  analogWrite(PWM1,1024);
  analogWrite(PWM2,0);
  analogWrite(PWM3,1024);
  analogWrite(PWM4,0);
}

void moveleft(){ 
  digitalWrite(DIR1,LOW);
  digitalWrite(DIR2,HIGH);
  digitalWrite(DIR3,LOW);
  digitalWrite(DIR4,HIGH);
  analogWrite(PWM1,0);
  analogWrite(PWM2,1024);
  analogWrite(PWM3,0);
  analogWrite(PWM4,1024);
}

void movebackward(){
  digitalWrite(DIR1,LOW);
  digitalWrite(DIR2,LOW);
  digitalWrite(DIR3,HIGH);
  digitalWrite(DIR4,LOW);
  analogWrite(PWM1,1024);
  analogWrite(PWM2,0);
  analogWrite(PWM3,1024);
  analogWrite(PWM4,0);
}

void moveright(){
  digitalWrite(DIR1,LOW);
  digitalWrite(DIR2,LOW);
  digitalWrite(DIR3,LOW);
  digitalWrite(DIR4,LOW);
  analogWrite(PWM1,0);
  analogWrite(PWM2,1024);
  analogWrite(PWM3,0);
  analogWrite(PWM4,1024);
}

void spinleft(){
  digitalWrite(DIR1,LOW);
  // Serial.println("1");wq
  digitalWrite(DIR2,HIGH);
  digitalWrite(DIR3,LOW);
  digitalWrite(DIR4,LOW);digitalWrite(PWM1,HIGH);
  analogWrite(PWM1,1024);
  analogWrite(PWM2,1024);
  analogWrite(PWM3,1024);
  analogWrite(PWM4,1024);
  
}

 void spinright(){
  digitalWrite(DIR1,HIGH);
  digitalWrite(DIR2,LOW);
  digitalWrite(DIR3,HIGH);
  digitalWrite(DIR4,HIGH);
  analogWrite(PWM1,1024);
  analogWrite(PWM2,1024);
  analogWrite(PWM3,1024);
  analogWrite(PWM4,1024);
}


void stopmotors(){

  analogWrite(PWM1,0);
  analogWrite(PWM2,0);
  analogWrite(PWM3,0);
  analogWrite(PWM4,0);

}
