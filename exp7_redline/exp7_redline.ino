#include<MsTimer2.h>//定时器库的头文件
//---定义管脚
#define ENCODER_A1 2 //电机1 左电机
#define ENCODER_B1 4
#define ENCODER_A2 3 //电机1 右电机
#define ENCODER_B2 5
#define PWM1 11
#define PWM2 12
#define L1 A0 // 左红外
#define L2 A1
#define L3 A2
#define L4 A3 // 右红外
#define L5 A4
#define L6 A5
#define L7 A6 // 中红外
#define L8 A7

#define DIR1 6
#define DIR2 7

//定义常值
#define PERIOD 20
#define Kp 7.0
#define Ti 50.0
#define Td 10.0

//---新增：转向PD参数
#define STEER_KP 5.0   // 降低比例增益，减少过度转向
#define STEER_KD 0.8   // 增大微分增益，增强阻尼抑制振荡
#define DELTA_MAX 2.0  // 限制最大差速量，防止一侧轮子完全停转
#define IR_HISTORY 2

//---新增：红外传感器历史数据(8路×2帧)
volatile int infrared[8][IR_HISTORY] = {0};

//---新增：转向控制变量
volatile float position_error = 0;
volatile float last_position_error = 0;
volatile float delta_speed = 0;

//---新增：状态标志
volatile bool isTracing = true;
volatile bool isFinish = false;
volatile bool allFinish = false;

//----------------------------------- 全局变量
float target1 = 2.0, t1; //左 保守
float target2 = 2.0, t2; //右 速度
volatile long encoderVal1;//编码器1 值
float velocity1; //转速1
volatile long encoderVal2;//编码器2 值
float velocity2; //转速2
float T = PERIOD;
float q0 = Kp * (1 + T / Ti + Td / T);
float q1 = -Kp * (1 + 2 * Td / T);
float q2 = Kp * Td / T;
float u1, ek11, ek12;
float u2, ek21, ek22;

//测速与转向
#define V 3.0 //基础速度

//---新增：更新红外传感器数据（滑动窗口）
void updateInfrared() {
  int sensorPins[8] = {L1, L2, L3, L4, L5, L6, L7, L8};
  for (int i = 0; i < 8; i++) {
    for (int j = IR_HISTORY - 1; j > 0; j--) {
      infrared[i][j] = infrared[i][j - 1];
    }
    infrared[i][0] = digitalRead(sensorPins[i]);
  }
}

//---新增：检测全黑（终点线）
bool isAllBlack() {
  for (int i = 0; i < 8; i++) {
    if (infrared[i][0] == 0) return false;
  }
  return true;
}

//---新增：检测全白（丢失循线）
bool isAllWhite() {
  for (int i = 0; i < 8; i++) {
    if (infrared[i][0] == 1) return false;
  }
  return true;
}

//---新增：计算位置偏差（8传感器加权求和）
// 返回连续偏差值：正=偏左需右转，负=偏右需左转
float calcPositionError() {
  // 权重范围收窄，减小位置偏差信号幅度，配合降低后的 STEER_KP
  float weight[8] = {-1.5, -0.8, -0.4, -0.1, 0.1, 0.4, 0.8, 1.5};
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    float avg = 0;
    for (int j = 0; j < IR_HISTORY; j++) {
      avg += (float)infrared[i][j];
    }
    avg /= IR_HISTORY;          // 取历史平均，抑制噪声
    sum += avg * weight[i];     // 加权求和
  }
  return sum;
}

//---新增：转向PD控制器
// 输入位置偏差，输出差速量 delta_speed
float steerPD(float error) {
  float u = STEER_KP * error + STEER_KD * (error - last_position_error);
  last_position_error = error;
  if (u > DELTA_MAX) u = DELTA_MAX;
  if (u < -DELTA_MAX) u = -DELTA_MAX;
  return u;
}

void control(void)
{
  if (!isTracing) return;

  //--- 1. 更新红外传感器数据
  updateInfrared();

  //--- 2. 终点检测（全黑）
  if (isAllBlack()) {
    isFinish = true;
    target1 = 0;
    target2 = 0;
  }
  //--- 3. 正常巡线：计算位置偏差和转向量
  else if (!isAllWhite()) {
    position_error = calcPositionError();
    delta_speed = steerPD(position_error);
  }
  //--- 4. 全白（丢失黑线）：保持上一时刻转向方向并逐渐增强
  else {
    // 丢失黑线时保持转向方向，但减缓增强速度避免过度甩头
    if (delta_speed > 0 && delta_speed < DELTA_MAX) {
      delta_speed += 0.12 * V;
    }
    else if (delta_speed < 0 && delta_speed > -DELTA_MAX) {
      delta_speed -= 0.12 * V;
    }
  }

  //--- 5. 根据 delta_speed 调整左右轮目标速度
  if (isFinish) {
    target1 = 0;
    target2 = 0;
  }
  else {
    if (delta_speed > 0) {
      // 偏左，减速左轮实现右转
      target1 = V - abs(delta_speed);
      target2 = V;
    }
    else if (delta_speed < 0) {
      // 偏右，减速右轮实现左转
      target1 = V;
      target2 = V - abs(delta_speed);
    }
    else {
      // 正中，直行
      target1 = V;
      target2 = V;
    }
  }

  //--- 6. 目标速度平滑（保留原有低通滤波）
  // 减小平滑滞后（0.6→0.8），让速度指令更快响应转向需求
  target1 = target1 * 0.8 + t1 * 0.2;
  target2 = target2 * 0.8 + t2 * 0.2;

  //--- 7. 计算当前速度（保留原有公式）
  velocity1 = (encoderVal1 / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD);
  encoderVal1 = 0;
  velocity2 = (encoderVal2 / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD);
  encoderVal2 = 0;

  //--- 8. PID控制 + 电机方向控制
  // 左电机：正输出 = 正转(DIR1=HIGH)
  int output1 = pidController1(target1, velocity1);
  if (output1 > 0) {
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, output1);
  } else {
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, abs(output1));
  }

  // 右电机：物理安装方向与左电机相反，PID使用负目标值
  // 负输出 = 正转(DIR2=LOW)，正输出 = 反转(DIR2=HIGH)
  int output2 = pidController2(-target2, velocity2);
  if (output2 > 0) {
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, output2);
  } else {
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, abs(output2));
  }

  t1 = target1;
  t2 = target2;
}

//--------------------------------------
void setup() {
  //9,10 两个管脚的PWM 由定时器TIMER1 产生,这句程序改变PWM 的频率,勿删
  TCCR1B = TCCR1B & B11111000 | B00000001;
  MsTimer2::set(PERIOD, control);
  MsTimer2::start();
  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_B1, INPUT);
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  attachInterrupt(0, getEncoder1, CHANGE);
  attachInterrupt(1, getEncoder2, CHANGE);
  Serial.begin(9600);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);

  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(L3, INPUT);
  pinMode(L4, INPUT);
  pinMode(L5, INPUT);
  pinMode(L6, INPUT);
  pinMode(L7, INPUT);
  pinMode(L8, INPUT);

  MsTimer2::set(PERIOD, control);
  MsTimer2::start();
}
void loop() {
  // 终点停车
  if (isFinish && !allFinish) {
    target1 = 0;
    target2 = 0;
    // 让最后一轮 control() 将电机减速归零
    delay(200);
    allFinish = true;
  }

  Serial.print(velocity1); Serial.print("\t");
  Serial.println(velocity2);
  //Serial.print("\n");
  //Serial.println(u2);
  //Serial.println(u2);
}
//---------------------------------- 编码器中断函数
void getEncoder1(void)
{
  if (digitalRead(ENCODER_A1) == LOW)
  {
    if (digitalRead(ENCODER_B1) == LOW)
    {
      encoderVal1--;
    }
    else
    {
      encoderVal1++;
    }
  }
  else
  {
    if (digitalRead(ENCODER_B1) == LOW)
    {
      encoderVal1++;
    }
    else
    {
      encoderVal1--;
    }
  }
}
void getEncoder2(void)
{
  if (digitalRead(ENCODER_A2) == LOW)
  {
    if (digitalRead(ENCODER_B2) == LOW)
    {
      encoderVal2--;
    }
    else
    {
      encoderVal2++;
    }
  }
  else
  {
    if (digitalRead(ENCODER_B2) == LOW)
    {
      encoderVal2++;
    }
    else
    {
      encoderVal2--;
    }
  }
}
//控制器
int pidController1(float targetVelocity, float currentVelocity)
{
  float ek10;
  ek10 = targetVelocity - currentVelocity;
  u1 = u1 + q0 * ek10 + q1 * ek11 + q2 * ek12;
  if (u1 > 255)
  {
    u1 = 255;
  }
  if (u1 < -255)
  {
    u1 = -255;
  }
  ek12 = ek11;
  ek11 = ek10;
  return (int)u1;
}
int pidController2(float targetVelocity, float currentVelocity)
{
  float ek20;
  ek20 = targetVelocity - currentVelocity;
  u2 = u2 + q0 * ek20 + q1 * ek21 + q2 * ek22;
  if (u2 > 255)
  {
    u2 = 255;
  }
  if (u2 < -255)
  {
    u2 = -255;
  }
  ek22 = ek21;
  ek21 = ek20;
  return (int)u2;
}
