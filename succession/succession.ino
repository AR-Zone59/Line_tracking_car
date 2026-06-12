#include <MsTimer2.h>
#include <Servo.h>
#define PIN_SERVO1 8//定义舵机控制端口
#define PIN_SERVO2 9
#define PIN_SERVO3 10
#define ENCODER_A_L 2 //电机 1 的编码器 A 项接 Arduino 的 2 中断口，用于编码器计数
#define ENCODER_B_L 4
#define ENCODER_A_R 3 //电机 2 的编码器 A 项接 Arduino 的 3 中断口，用于编码器计数
#define ENCODER_B_R 5
#define PWML 11 //用于电机 1 的 PWM 输出， 调节电机速度
#define DIRL 6
#define PWMR 12 //用于电机 2 的 PWM 输出，调节电机速度
#define DIRR 7
#define PERIOD 20
#define INFRARE0 A0//红外
#define INFRARE1 A1
#define INFRARE2 A2
#define INFRARE3 A3
#define INFRARE4 A4
#define INFRARE5 A5
#define INFRARE6 A6
#define INFRARE7 A7


Servo servo1; //创建一个舵机控制对象
Servo servo2;
Servo servo3;
int langle1=0,langle2=0,langle3=0;
const int angle1=120,angle2=145,angle3=0;


#define BASESPEED 6.0 
#define timeline 2
volatile float TARGET_L = BASESPEED; //定义电机 1 的目标速度值
volatile float TARGET_R = -BASESPEED; //定义电机 2 的目标速度值
volatile float encoderVal_L; //在中断里面使用的全局变量需要定义成 volatile 类型
volatile float encoderVal_R;
volatile float velocity_L;
volatile float velocity_R;
volatile float uL=0;
volatile float uR=0;
volatile float LeI; //电机 1 当前时刻的误差 e(k)
volatile float LeII; //上一时刻的误差 e(k-1)
volatile float LeIII; //上上时刻的误差 e(k-2)
volatile int Loutput;
volatile float ReI; //电机 2 当前时刻的误差 e(k)
volatile float ReII; //上一时刻的误差 e(k-1)
volatile float ReIII; //上上时刻的误差 e(k-2)
volatile int Routput;

volatile float delta=0;
volatile float delta_speed_max = 40;
volatile float delta_speed = 0;
volatile float FeI; 
volatile float FeII=0;
volatile int infrare[8][timeline] = {0};


//获取电机 1 的编码器值
void getEncoder_L(void){
if(digitalRead(ENCODER_A_L) == LOW){
if (digitalRead(ENCODER_B_L) == LOW){
encoderVal_L--;
}
else{
encoderVal_L++;}
}
else{
if (digitalRead(ENCODER_B_L) == LOW){
encoderVal_L++;
}
else{
encoderVal_L--;
}
}
}
//获取电机 2 的编码器值
void getEncoder_R(void){
if(digitalRead(ENCODER_A_R) == LOW){
if (digitalRead(ENCODER_B_R) == LOW){
encoderVal_R--;
}
else{
encoderVal_R++;
}
}
else{
if (digitalRead(ENCODER_B_R) == LOW){
encoderVal_R++;
}
else{
encoderVal_R--;
}
}
}


//电机 1 的 PID 算法
int pidcontrol_L(float target,float current)
{
LeI = target - current;
float kp = 30, TI =25 , TD =15,T = PERIOD;
float q0 = kp * (1+T/TI + TD/T);
float q1 = -kp * (1+2*TD/T);
float q2 = kp * TD / T;
uL = uL + q0 * LeI + q1 * LeII + q2 * LeIII;
LeIII = LeII;
LeII = LeI;
if (uL > 255){
uL = 255;
}else if (uL <= -255){
uL = -255;
}
Loutput = uL;
return (int)Loutput;
}
//电机 2 的 PID 算法
int pidcontrol_R(float target,float current)
{
ReI = target - current;
float kp = 30, TI =25 , TD =15,T = PERIOD;
float q0 = kp * (1+T/TI + TD/T);
float q1 = -kp * (1+2*TD/T);
float q2 = kp * TD / T;
uR = uR + q0 * ReI + q1 * ReII + q2 * ReIII;
ReIII = ReII;
ReII = ReI;
if (uR > 255){
uR = 255;
}
else if (uR <= -255){
uR = -255;
}
Routput = uR;
return (int)Routput;
}


//电机 1 速度和方向控制
void control_L()
{
velocity_L = (encoderVal_L/780) * 3.1415 * 2.0 * (1000/PERIOD); //计算轮子转动速度（弧度制的速度， 2π Rn）
Loutput = pidcontrol_L(TARGET_L,velocity_L);
if (Loutput > 0){
digitalWrite(DIRL,HIGH);
analogWrite(PWML,Loutput);
}
else {
digitalWrite(DIRL,LOW);
analogWrite(PWML,abs(Loutput));
}
encoderVal_L = 0;}
//电机 2 速度和方向控制
void control_R()
{
velocity_R = (encoderVal_R/780) * 3.1415 * 2.0 * (1000/PERIOD); //计算轮子转动速度（弧度制的速度 2π Rn）
Routput = pidcontrol_R(TARGET_R,velocity_R);
if (Routput > 0){
digitalWrite(DIRR,HIGH);
analogWrite(PWMR,Routput);
}
else {
digitalWrite(DIRR,LOW);
analogWrite(PWMR,abs(Routput));
}
encoderVal_R = 0;
}

bool isAllBlack(){
  for (int i=0;i<8;i++) if (infrare[i][0]==0) return false;
  return true;
}
bool isAllWhite(){
  for (int i=0;i<8;i++) if (infrare[i][0]==1) return false;
  return true;
}
//红外数据更新
void InfrareUpdate(){
  infrare[0][0] = digitalRead(INFRARE0);
  infrare[1][0] = digitalRead(INFRARE1);
  infrare[2][0] = digitalRead(INFRARE2);
  infrare[3][0] = digitalRead(INFRARE3);
  infrare[4][0] = digitalRead(INFRARE4);
  infrare[5][0] = digitalRead(INFRARE5);
  infrare[6][0] = digitalRead(INFRARE6);
  infrare[7][0] = digitalRead(INFRARE7);
  for (int i=0;i<8;i++){
    for (int j=timeline-1;j>0;j--){
      infrare[i][j]=infrare[i][j-1];
    }
  }
}
//红外获取方向偏差
volatile float err[]={-2,-1,-0.5,-0.1,0.1,0.5,1,2};
volatile float sum=0,temp=0;
float controlInfrare(){
  sum=0;
  temp=0;
  for (int i=0;i<8;i++){
    for (int j=0;j<timeline;j++){
      temp+=(float)infrare[i][j];
    }
    temp=temp;
    sum+=temp*err[i];
    temp=0;
  }
  return sum;
}
//红外pid->减速量
float pidToSlow(float error){
  float u=0,kp=8.0,kd=0.4;//0.4 
  FeI=error;
  //integal+=FeI;
  u=kp*FeI+kd*(FeI-FeII);
  FeII=FeI;
  if (u>delta_speed_max) u=delta_speed_max;
  if (u<-delta_speed_max) u=-delta_speed_max;
  return u;
}
//控制差速转向
bool isFinish=false;
void controlDelta(){
  if (!isFinish){
    InfrareUpdate();
    if (isAllBlack()) isFinish=true;
    else if (!isAllWhite()){
      delta=controlInfrare();
      delta_speed=pidToSlow(delta);
    }
    else{
      if (delta_speed>0&&delta_speed<delta_speed_max) delta_speed+=0.28*BASESPEED;
      else if(delta_speed<0&&delta_speed>-delta_speed_max) delta_speed-=0.28*BASESPEED;
    }
    if (delta_speed>0){
      TARGET_L=BASESPEED-abs(delta_speed);
      TARGET_R=-BASESPEED;
    }
    else if(delta_speed<0){
      TARGET_R=-BASESPEED+abs(delta_speed);
      TARGET_L=BASESPEED;
    }
  }
  else{
  }
}


//封装函数
bool isTrace=false;
void control(void)
{
  if(isTrace){
    controlDelta();
    control_L();
    control_R();
  }
}


// put your setup code here, to run once
void setup() {
TCCR1B = TCCR1B & B11111000 | B00000001; //PWM 频率调节，设置 9、 10 引脚的 PWM 输出频率为 31372Hz，适合于我们使用的电机
pinMode(PWML,OUTPUT);
pinMode(DIRL,OUTPUT);
pinMode(PWMR,OUTPUT);
pinMode(DIRR,OUTPUT);
pinMode(ENCODER_A_L,INPUT);
pinMode(ENCODER_B_L,INPUT);
pinMode(ENCODER_A_R,INPUT);
pinMode(ENCODER_B_R,INPUT);
pinMode(INFRARE0, INPUT);
pinMode(INFRARE1, INPUT);
pinMode(INFRARE2, INPUT);
pinMode(INFRARE3, INPUT);
pinMode(INFRARE4, INPUT);
pinMode(INFRARE5, INPUT);
pinMode(INFRARE6, INPUT);
pinMode(INFRARE7, INPUT);
servo1.attach(PIN_SERVO1); //定义舵机接口 10
servo2.attach(PIN_SERVO2);
servo3.attach(PIN_SERVO3);
attachInterrupt(0,getEncoder_L,CHANGE); //中断 0 设置，对应 2 引脚
attachInterrupt(1,getEncoder_R,CHANGE); //中断 1 设置，对应 3 引脚
Serial.begin(9600);
MsTimer2::set(PERIOD,control); //设定每隔 PERIOD 时间，执行一次 control
MsTimer2::start(); //开始时间
}
// put your main code here, to run repeatedly
bool allFinish=false;
void loop() {
  if(!isTrace){
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);
  delay(400);
  servo1.write(45);
  delay(300);
  servo3.write(140);
  delay(700);
  isTrace=true;

  }
  if (isFinish&&!allFinish){
  TARGET_L=0;
  TARGET_R=0;
  control_L();
  control_R();
  servo3.write(angle3);
  servo2.write(angle2);
  delay(400);
  servo1.write(angle1);
  delay(200);
  allFinish=true;
  }
}
