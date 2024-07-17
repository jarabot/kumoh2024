#include <SoftwareSerial.h>
int rxPin = 11;  // RX
int txPin = 12; // TX
SoftwareSerial mySerial(rxPin, txPin); // 소프트웨어 시리얼 객체 생성

#define MotL_WCnt_pin 2  //FS-Brown
#define MotR_WCnt_pin 3  //FS-Brown

#define MotL_FR_pin 7    //FR- White
#define MotR_FR_pin 8    //FR- White 

#define MotL_BRK_pin 9   //BRK- Green
#define MotR_BRK_pin 10   //BRK- Green

#define MotL_PWM_pin 5 //PWM - Blue for arduino nano timer0 
#define MotR_PWM_pin 6 //PWM - Blue for arduino nano timer0
//#define MotL_PWM_pin 9   //PWM - Blue mega2560 timer2  
//#define MotR_PWM_pin 10  //PWM - Blue mega2560 timer2

volatile long leftEncoderPulses = 0;
volatile long rightEncoderPulses = 0;

bool L_DIR = 0;
bool R_DIR = 0;
int roll, pitch;
int X_DIR = 0;
int Y_DIR = 0;;
int thrL, thrR;
int Xv = 0;
int Yv = 0;
int wl_cntL = 0;
int wl_cntR = 0;

void setup() {
  
  mySerial.begin(57600);
  Serial.begin(57600);

  pinMode(MotL_WCnt_pin,INPUT_PULLUP);
  pinMode(MotR_WCnt_pin,INPUT_PULLUP);
  pinMode(MotL_PWM_pin,OUTPUT);
  pinMode(MotR_PWM_pin,OUTPUT);
  pinMode(MotL_FR_pin,OUTPUT);
  pinMode(MotR_FR_pin,OUTPUT);
  pinMode(MotL_BRK_pin,OUTPUT);
  pinMode(MotR_BRK_pin,OUTPUT);

    // Pins D5 and D6 - 31.4 kHz
  //TCCR0B = 0b00000001; // x1
  //TCCR0A = 0b00000001; // phase correct
  // Pins D5 and D6 - 7.8 kHz
  //TCCR0B = 0b00000010; // x8
  //TCCR0A = 0b00000011; // fast pwm
  // Pins D5 and D6 - 4 kHz
  TCCR0B = 0b00000010; // x8
  TCCR0A = 0b00000001; // phase correct
  // Pins D5 and D6 - 976 Hz - default
  //TCCR0B = 0b00000011; // x64
  //TCCR0A = 0b00000011; // fast pwm
  // Pins D5 and D6 - 490 Hz
  //TCCR0B = 0b00000011; // x64
  //TCCR0A = 0b00000001; // phase correct

  _stop();
  analogWrite(MotL_PWM_pin,255);
  analogWrite(MotR_PWM_pin,255);

  attachInterrupt(digitalPinToInterrupt(MotL_WCnt_pin), ISR_wheelCount_L, RISING);
  attachInterrupt(digitalPinToInterrupt(MotR_WCnt_pin), ISR_wheelCount_R, RISING);
  
}

String rcvStr = "pwm,1500,1500";
String command;
String data1;
String data2;
int nIdx = 0;    
int nTry = 0; 
int strLength = 0;

 
void loop() {

  /* software시리얼은 노이즈가 껴들어 오는 경우가 있다........그래서 사용안함.
  if (mySerial.available()) {
    rcvStr = mySerial.readStringUntil('\n'); // 개행 문자('\n')까지 읽어옵니다.
    strLength = rcvStr.length();
    //Serial.println(rcvStr);
  } 
  */

  //RX/TX 핀을 제거 후 업로드 해야하고 업로드가 완료 되면 다시 RX/TX 핀을 연결해야 한다.
  if (Serial.available()) {
    rcvStr = Serial.readStringUntil('\n'); // 개행 문자('\n')까지 읽어옵니다.
    strLength = rcvStr.length();
    //Serial.println(rcvStr);
  } 

  if ((rcvStr != "") && (strLength == 14) ) {  //splitting a string....
    // 수신된 문자열을 쉼표로 분리하여 데이터 추출
    int firstCommaIndex = rcvStr.indexOf(','); // 첫 번째 쉼표의 인덱스
    int secondCommaIndex = rcvStr.indexOf(',', firstCommaIndex + 1); // 두 번째 쉼표의 인덱스

    if (firstCommaIndex != -1 && secondCommaIndex != -1) {
      command = rcvStr.substring(0, firstCommaIndex); // "pwm" or "rst" 추출
      data1 = rcvStr.substring(firstCommaIndex + 1, secondCommaIndex); 
      data2 = rcvStr.substring(secondCommaIndex + 1); 
      roll = data1.toInt();
      pitch = data2.toInt();
      // Serial.print(command);
      // Serial.print("," + data1);
      // Serial.println("," + data2);
    }
  }


  if (command == "rst"){   //reset wheel count
    leftEncoderPulses = 0;
    rightEncoderPulses = 0;
  }

  if (roll <= 1000) {roll = 1500;};
  if (pitch <= 1000) {pitch = 1500;};
  if (roll >= 2000) {roll = 1500;};
  if (pitch >= 2000) {pitch = 1500;};

  if (roll < 1410) { 
    Xv = map(roll,1460,1000,255,0);
    X_DIR = -1;           //left
    _go();
  } else if (roll > 1660) { 
    Xv = map(roll,1660,2000,255,0);
    X_DIR = 1;            //right
    _go();
  } else {
    Xv = 255;      
    X_DIR = 0;            //cneter 
  }

  if (pitch < 1410) {
    Yv = map(pitch,1410,1000,255,0);
    Y_DIR = - 1;          //backward
    _go();
  } else if (pitch > 1660) {
    Yv = map(pitch,1560,2000,255,0);
    Y_DIR = 1;            //forward
    _go();
  } else {
    Yv = 255;
    Y_DIR = 0;            //center
  }

  if (Y_DIR == 1) {                 //ForWard     
    L_DIR = 1;
    R_DIR = 0;             
    digitalWrite(MotL_FR_pin,LOW);      
    digitalWrite(MotR_FR_pin,HIGH);
    if ( X_DIR == -1 ) {      //left
      thrL = Yv + (255-Xv);              
      thrR = Yv;   
    }else if ( X_DIR == 1 ){  //right
      thrL = Yv;
      thrR = Yv + (255-Xv);   
    } else {
      thrL = Yv;
      thrR = Yv;   
    }
  } else if (Y_DIR == -1) {         //BackWard
    L_DIR = 0;
    R_DIR = 1;                 
    digitalWrite(MotL_FR_pin,HIGH); 
    digitalWrite(MotR_FR_pin,LOW);
    if ( X_DIR == -1 ) {      //left
      thrL = Yv;        //dec  
      thrR = Yv + (255-Xv);        
    }else if ( X_DIR == 1 ){      //right
      thrL = Yv + (255-Xv);
      thrR = Yv;   
    } else {
      thrL = Yv;
      thrR = Yv;   
    }    
  } else if (Y_DIR == 0) {    //Stop or Turn
    if (X_DIR == -1){         //turn left   
      L_DIR = 0;
      R_DIR = 0;                  
      digitalWrite(MotL_FR_pin,HIGH);     
      digitalWrite(MotR_FR_pin,HIGH);                        
      thrL = Xv;
      thrR = Xv;
    } else if (X_DIR == 1){   //turn right        
      L_DIR = 1;
      R_DIR = 1;                        
      digitalWrite(MotL_FR_pin,LOW);      //F             
      digitalWrite(MotR_FR_pin,LOW);      //B
      thrL = Xv;
      thrR = Xv;
    } else if (X_DIR == 0){      
      thrL = 255;
      thrR = 255;
      _stop();
    }
  }    
        
  if (thrL > 255){thrL = 255;};
  if (thrR > 255){thrR = 255;};   
      
  analogWrite(MotL_PWM_pin,thrL);
  analogWrite(MotR_PWM_pin,thrR); 
  
  
  Serial.print(leftEncoderPulses);
  Serial.print(",");
  Serial.println(rightEncoderPulses);
  
  
}

void _stop(){
  digitalWrite(MotR_BRK_pin,HIGH);
  digitalWrite(MotL_BRK_pin,HIGH);
}

void _go(){
  digitalWrite(MotR_BRK_pin,LOW);
  digitalWrite(MotL_BRK_pin,LOW);
}

void ISR_wheelCount_L()
{ 
  if (L_DIR == 1){
    ++leftEncoderPulses;
  } else if (L_DIR == 0){
    --leftEncoderPulses;
  }  
}

void ISR_wheelCount_R()
{ 
  if (R_DIR == 0){
    ++rightEncoderPulses;
  } else if (R_DIR == 1){
    --rightEncoderPulses;
  }  
}
