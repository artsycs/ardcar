#include <IRremote.h>
#include <Servo.h>  

#define f 16736925  // FORWARD
#define b 16754775  // BACK
#define l 16720605  // LEFT
#define r 16761405  // RIGHT
#define s 16712445  // STOP
#define KEY1 16738455 //Line tracking mode
#define KEY2 16750695 //Obstacles Avoidance mode
#define KEY3 16756815
#define KEY4 16724175
#define KEY5 16718055
#define KEY6 16743045
#define KEY7 16716015
#define KEY8 16726215
#define KEY9 16734885
#define KEY0 16730805
#define KEY_STAR 16728765
#define KEY_HASH 16732845

#define RECV_PIN  12
#define ECHO_PIN  A4
#define TRIG_PIN  A5 
#define ENA 5
#define ENB 6
#define IN1 7 //left wheel forward
#define IN2 8 //left wheel reverse
#define IN3 9 //right wheel forward
#define IN4 11 //right wheel reverse
#define LED_Pin_Center 13         //pins already used in this program = S2 - S12.  LIne tracking uses 10, 4, and 2
//#define LED_Pin_Left 1
//#define LED_Pin_Right 0
#define LineTracking_Pin_Right  10
#define LineTracking_Pin_Middle 4
#define LineTracking_Pin_Left   2
#define LineTracking_Read_Right   !digitalRead(10)  //not white/reflecting light.  If white line on black floor, remove !
#define LineTracking_Read_Middle  !digitalRead(4)
#define LineTracking_Read_Left    !digitalRead(2)
//-----------------------------------------------------------------
#define carSpeed 250 // -------can change btwn 0 through 255--------  Make second speed for right/left slower?
#define carTurnSpeed 150 // -------can change btwn 0 through 255--------  Make second speed for right/left slower?
//------------------------------------------------------------------

Servo servo;
IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long IR_PreMillis;
unsigned long LT_PreMillis;
int rightDistance = 0, leftDistance = 0, middleDistance = 0;

enum FUNCTIONMODE{
  IDLE,
  LineTracking,
  ObstaclesAvoidance,
  Bluetooth,
  IRremote
} func_mode = IDLE;

enum MOTIONMODE {
  STOP,
  FORWARD,
  BACK,
  LEFT,
  RIGHT
} mov_mode = STOP;

void delays(unsigned long t) {
  for(unsigned long i = 0; i < t; i++) {
    getBTData();
    getIRData();
    delay(1);
  }
}

int getDistance() {  //obstacle avoidance
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return (int)pulseIn(ECHO_PIN, HIGH) / 58;
}

void forward(bool debug = false){ 
  //digitalWrite(LED_Pin_Center,HIGH); //turn on center LED
  //digitalWrite(LED_Pin_Right,LOW); //turn off right LED
  //digitalWrite(LED_Pin_Left,LOW); //turn off left LED
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  //delays(100); //shorten from 360 to 270
  //digitalWrite(LED_Pin_Center,LOW); //turn off center LED
  if(debug) Serial.println("Go forward!");  //see if you can print to screen via bluetooth instead of usb cable
}

void back(bool debug = false){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  if(debug) Serial.println("Go back!");
}

void left(bool debug = false){
  //digitalWrite(LED_Pin_Center,LOW); //turn off center LED
  //digitalWrite(LED_Pin_Right,LOW); //turn off right LED
  //digitalWrite(LED_Pin_Left,HIGH); //turn on left  LED
  analogWrite(ENA,carTurnSpeed); //may want to slow down !!!!!!!!!!!!!!!!!!!!
  analogWrite(ENB,carTurnSpeed);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH); 
  if(debug) Serial.println("Go left!");
}

void right(bool debug = false){
  //digitalWrite(LED_Pin_Center,LOW); //turn off center LED
  //digitalWrite(LED_Pin_Right,HIGH); //turn on right LED
  //digitalWrite(LED_Pin_Left,LOW); //turn off left LED
  analogWrite(ENA,carTurnSpeed); //may want to slow down !!!!!!!!!!!!!!!!!!!!
  analogWrite(ENB,carTurnSpeed);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  if(debug) Serial.println("Go right!");
}

void stop(bool debug = false){
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  if(debug) Serial.println("Stop!");
}

void getBTData() {
  if(Serial.available()) {
    switch(Serial.read()) {
      case 'f': func_mode = Bluetooth; mov_mode = FORWARD;  break;
      case 'b': func_mode = Bluetooth; mov_mode = BACK;     break;
      case 'l': func_mode = Bluetooth; mov_mode = LEFT;     break;
      case 'r': func_mode = Bluetooth; mov_mode = RIGHT;    break;
      case 's': func_mode = Bluetooth; mov_mode = STOP;     break;
      case '1': func_mode = LineTracking;                   break;
      case '2': func_mode = ObstaclesAvoidance;             break;
      default:  break;
    } 
  }
}

void getIRData() {
  if (irrecv.decode(&results)){ 
    IR_PreMillis = millis();
    switch(results.value){
      case f:   func_mode = IRremote; mov_mode = FORWARD;  break;
      case b:   func_mode = IRremote; mov_mode = BACK;     break;
      case l:   func_mode = IRremote; mov_mode = LEFT;     break;
      case r:   func_mode = IRremote; mov_mode = RIGHT;    break;
      case s:   func_mode = IRremote; mov_mode = STOP;     break;
      case KEY1:  func_mode = LineTracking;                break;
      case KEY2:  func_mode = ObstaclesAvoidance;          break;
      default: break;
    }
    irrecv.resume();
  }
}

void bluetooth_mode() {
  if(func_mode == Bluetooth){
    switch(mov_mode){
      case FORWARD: forward();  break;
      case BACK:    back();     break;
      case LEFT:    left();     break;
      case RIGHT:   right();    break;
      case STOP:    stop();     break;
      default: break;
    }
  }
}

void irremote_mode() {
  if(func_mode == IRremote){
    switch(mov_mode){
      case FORWARD: forward();  break;
      case BACK:    back();     break;
      case LEFT:    left();     break;
      case RIGHT:   right();    break;
      case STOP:    stop();     break;
      default: break;
    }
    if(millis() - IR_PreMillis > 500){
      mov_mode = STOP;
      IR_PreMillis = millis();
    }
  }   
}

void line_tracking_mode() {
  if(func_mode == LineTracking){
    //servo.write(90);
    //delays(500);
    //middleDistance = getDistance();
    //if(middleDistance <= 20) {  //May be too close, increase to stop sooner before obstacle !!!!!!!!!!!!
      //stop(); }
    if(LineTracking_Read_Middle){
      forward();
      LT_PreMillis = millis();
    } else if(LineTracking_Read_Left) { 
      left();
      while(LineTracking_Read_Left) {
        getBTData();
        getIRData();
      }
      LT_PreMillis = millis();
    } else if(LineTracking_Read_Right) {
      right();
      while(LineTracking_Read_Right) {
        getBTData();
        getIRData();
      }
      LT_PreMillis = millis();
    } else {
      if(millis() - LT_PreMillis > 150){
        //logic to spin 360 to refind line?????????????????????
        left();
        LT_PreMillis = millis();
        //stop();
      }
    }
  }  
}


void obstacles_avoidance_mode() {
  if(func_mode == ObstaclesAvoidance){
    servo.write(90);
    delays(500);
    middleDistance = getDistance();
    if(middleDistance <= 40) {  //May be too close, increase to stop sooner before obstacle !!!!!!!!!!!!
      stop();
      //digitalWrite(LED_Pin_Center,HIGH); //turn on LED when found obstacle
      delays(500);
      servo.write(10); //why not 0?????
      delays(500);    // shorten or Remove Delays?????????????????????????????????????????????????
      rightDistance = getDistance();
      
      //delays(500);
      //servo.write(90);  //why move to center?
      delays(500);  //shorten to 500 from 1000?
      servo.write(170);
      delays(500); //shorten to 500
      leftDistance = getDistance();
      
      delays(500);
      servo.write(90);
      delays(500); //shorten to 500
      if(rightDistance > leftDistance) {
        //digitalWrite(LED_Pin_Center,LOW); //turn off LED
        right();
        delays(270); //shorten from 360 to 270
      } else if(rightDistance < leftDistance) {
        //digitalWrite(LED_Pin_Center,LOW); //turn off LED
        left();
        delays(270); //shorten from 360 to 270
      } else if((rightDistance <= 40) || (leftDistance <= 40)) {
        //digitalWrite(LED_Pin_Center,LOW); //turn off LED
        back();
        delays(180);
      } else {
        //digitalWrite(LED_Pin_Center,LOW); //turn off LED
        forward();
      }
    } else {
        forward();
    }  
  }  
}

void setup() {
  Serial.begin(9600);
  servo.attach(3,500,2400);// 500: 0 degree  2400: 180 degree
  servo.write(90);
  irrecv.enableIRIn();
  pinMode(LED_Pin_Center, OUTPUT); //LED!!!!!!!!!!!!!!
  //pinMode(LED_Pin_Left, OUTPUT); //LED!!!!!!!!!!!!!!
  //pinMode(LED_Pin_Right, OUTPUT); //LED!!!!!!!!!!!!!!  
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LineTracking_Pin_Right, INPUT);
  pinMode(LineTracking_Pin_Middle, INPUT);
  pinMode(LineTracking_Pin_Left, INPUT);
}

void loop() {
  getBTData();
  getIRData();
  bluetooth_mode();
  irremote_mode();
  line_tracking_mode();
  obstacles_avoidance_mode();
}
