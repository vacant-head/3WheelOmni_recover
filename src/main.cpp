#include <Arduino.h>
#include <math.h>
#include <PS4Controller.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <MPU6050.h>

// 制御周期系の設定
const int control_period = 2000;                         // us
const float control_freq = 1.0f / (float)control_period; // Hz
unsigned long prevtime = 0;                              // us

// ジャイロセンサ
const uint8_t gyro_SCL = 22;
const uint8_t gyro_SDA = 23;

// モーターまわりのピン宣言
const uint8_t pwmPin1 = 4;
const int dirPin1_1 = 16;
const int dirPin1_2 = 17;
const int channel_motor1 = 1;
const uint8_t pwmPin2 = 15;
const int dirPin2_1 = 18;
const int dirPin2_2 = 19;
const int channel_motor2 = 2;
const uint8_t pwmPin3 = 25;
const int dirPin3_1 = 26;
const int dirPin3_2 = 27;
const int channel_motor3 = 3;
uint8_t pwmResolution = 8;

int motor1[5] = {pwmPin1, dirPin1_1, dirPin1_2, channel_motor1, -1}; // 1番モーター
int motor2[5] = {pwmPin2, dirPin2_1, dirPin2_2, channel_motor2, -1}; // 2番モーター
int motor3[5] = {pwmPin3, dirPin3_1, dirPin3_2, channel_motor3, -1}; // 3番モーター

// ps4の入力を速度の指令値に変換
double command_x = 0;
double command_y = 0;
double command_yaw = 0;

// モーターのセットアップmotornum[4] = {pwmPin, dirPin1, dirPin2, channel}
void motorSetup(int motornum[], int freq) // motornum[5] = {pwmPin, dirPin1, dirPin2, channel, default_dir}
{
  pinMode(motornum[0], OUTPUT); // pwm
  pinMode(motornum[1], OUTPUT); // IN1
  pinMode(motornum[2], OUTPUT); // IN2
  ledcSetup(motornum[3], freq, pwmResolution);
  ledcAttachPin(motornum[0], motornum[3]);
  // Serial.printf("setup\n");
}

// ps4の値
int ps4_x_raw;   // 左スティック横方向
int ps4_y_raw;   // 左スティック縦方向
int ps4_yaw_raw; // 右スティック横

// モーター駆動用関数
void motorDrive(int Drive_motornum[], double duty)
{
  duty = duty * Drive_motornum[4];
  bool dir = duty > 0; // 方向
  double DUTY = pow(2, pwmResolution) * abs(duty);
  bool dutycheck = DUTY < 257;
  switch (dutycheck)
  {
  case 1:
    digitalWrite(Drive_motornum[1], dir);  // 方向信号1
    digitalWrite(Drive_motornum[2], !dir); // 方向信号2
    ledcWrite(Drive_motornum[3], DUTY);    // pwm出力
    Serial.printf("DUTY=%lf", DUTY);
    break;
  case 0:
    ledcWrite(Drive_motornum[3], 0); // duty = 0を出力
    Serial.printf("DUTYerr=%lf\n", DUTY);
    break;
  }
}

// モーター電圧解放
void motorFree(int Free_motornum[])
{
  digitalWrite(Free_motornum[1], 0); // 方向信号1
  digitalWrite(Free_motornum[2], 0); // 方向信号2
  ledcWrite(Free_motornum[3], 0);    // duty=0で出力
}

// モーターブレーキ
void motorStop(int motornum[])
{
  digitalWrite(motornum[1], 1); // 方向信号1
  digitalWrite(motornum[2], 1); // 方向信号2
  ledcWrite(motornum[3], 1);    // duty=0を出力
}

// コントローラの値からオムニ行列を計算し，モーター出力
void OmniDrive(double ps4x, double ps4y, double ps4yaw)
{
  double omniArray[3][3] = {{-1.0, 0.0, 1.0}, {0.5, -0.8660254, 1.0}, {0.5, 0.8660254, 1.0}};
  double duty_calc[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++) // dutyの計算
  {
    duty_calc[i] = omniArray[i][0] * ps4x + omniArray[i][1] * ps4y + omniArray[i][2] * ps4yaw;
  }

  motorDrive(motor1, duty_calc[0]);
  motorDrive(motor2, duty_calc[1]);
  motorDrive(motor3, duty_calc[2]);
  Serial.printf("\r\n");
}

void setup()
{
  Serial.begin(115200);
  PS4.begin("0C:B8:15:C1:3C:66");

  // モーターのピン設定
  int ledcfreq = 2000; // ledcWriteの周波数設定
  motorSetup(motor1, ledcfreq);
  Serial.printf("motor1Setup\r\n");
  motorSetup(motor2, ledcfreq);
  Serial.printf("motor2Setup\r\n");
  motorSetup(motor3, ledcfreq);
  Serial.printf("motor3Setup\r\n");
}

void loop()
{
  if (PS4.isConnected()) // PS4に接続済のとき
  {
    //-trimval<=0<=trimvalの値を0にしてる
    // スティックの値を -1 <= stick <= 1 の範囲に変更
    double trimval = 10.0;
    if (PS4.LStickX() > trimval || PS4.LStickX() < (-1 * trimval))
    {
      command_x = (double)PS4.LStickX() / (128.0 - trimval);
    }
    else
    {
      command_x = 0;
    }
    if (PS4.LStickY() > trimval || PS4.LStickY() < -1 * trimval)
    {
      command_y = (double)PS4.LStickY() / (128.0 - trimval);
    }
    else
    {
      command_y = 0;
    }
    if (PS4.RStickX() > trimval || PS4.RStickX() < (-1 * trimval))
    {
      command_yaw = -1 * (double)PS4.RStickX() / (128.0 - trimval);
    }
    else
    {
      command_yaw = 0;
    }

    // Serial.printf("vx=%d,vy=%d,vyaw=%d\r\n", PS4.LStickX(), PS4.LStickY(), PS4.RStickX());
    Serial.printf("X=%lf,Y=%lf,YAW=%lf\r\n", command_x, command_y, command_yaw);

    OmniDrive(command_x, command_y, command_yaw);
    // delay(500);
  }
  else // 未接続の場合，モーターはフリー回転する
  {
    Serial.println("PS4NotConnected");
    motorFree(motor1);
    motorFree(motor2);
    motorFree(motor3);
  }

  unsigned long TIMER = micros() - prevtime;
  while (TIMER < control_period)
  {
    TIMER = micros() - prevtime;
  }
  prevtime = micros();
}