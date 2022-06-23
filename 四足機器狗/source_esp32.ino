#include "datatypes.h"

#include <Adafruit_PWMServoDriver.h> //使用 Arduino PWM的接腳來控制伺服器馬達， 呼叫伺服驅動程式函數
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

#include <PS4Controller.h> //利用PS4控制

//> 儲存循環函數的頻率
const float frequency = 440.0; // 定義頻率(單位 Hz)

/// 移動參數

//: 儲存主體的位置、旋轉和縮放 
const datatypes::Transform body_transform = {
  {0, 0, 0},  // 單位 {mm, mm, mm}        設定主體的位置三維參數單位mm
  {0, 0, 0},   // 角度單位 {deg, deg, deg} 設定主體的旋轉參數單位deg
  {300, 40, 180} // 單位 {mm, mm, mm}     設定主體的比例三維參數單位mm
};

//: 儲存相對於主體 parent joint 的位置(向量)
const datatypes::Vector p_joint_origin[] = {
  { -50, 0, 0}, // 單位 {mm, mm, mm} 
  { +50, 0, 0}, // 單位 {mm, mm, mm}
  { +50, 0, 0}, // 單位 {mm, mm, mm}
  { -50, 0, 0}  // 單位 {mm, mm, mm}
};
const float bone_length = 105; // 設定骨骼長度參數單位 mm

//: 階躍函數的高級參數
const datatypes::Vector step_extent = {40, 40, 26}; // 單位 {mm, mm}
float vrt_offset = - 16.50; // 單位 mm
float hrz_offset = - 6.00; // 單位 mm
float base_offset[] = { 0, -1, 0, -2};
const float precision = 0.001; // 定義精確值(單位 mm)

void setup() {
  Serial.begin(115200); 

  init_hardware();
  init_input();
}

//: 局部變量控制方向和周期
datatypes::Vector2D _direction = {0, 0};
float turn = 0; //> 旋轉的方向
float height = 0; //> 腿部的伸展

int state = 0; //> 表示類型，(0) 空閒，(1) 小跑，(2) 橫擺，(3) 俯仰滾動，(4) 物體檢測
float _period = 10.0; //> 每秒的步數

datatypes::Rotator _sRotation; //> 儲存了本體的相對旋轉
unsigned long duration; //設定持續時間變數

int sample_sum, sample_num = 10, //設定樣本和樣本數量
                sample_index;    //設定樣本索引
float freq;  //設定頻率

void loop() {
  duration = millis();

  handle_hardware(); 
  handle_kinematics(_direction, turn, height, _period); //引入(位置,旋轉方向,腿部伸展,每秒步數)

  handle_input(); 

  if (Serial.available())
    handle_serial(); 

  // 獲取循環函數的頻率
  
}

float vo, ho;
void init_input() {
  PS4.begin("F8:C3:9E:3F:F8:10"); // !! 替換成自己的 DualShock4 控制器藍牙 MAC 位址 
  vo = vrt_offset; //水平補償
  ho = hrz_offset; //垂直補償
}

bool _tb = false; //判斷狀態是否執行
float stick_min = 6.f; 
float lx, ly, rx, ry;
void handle_input() {
  if (PS4.isConnected()) {
    lx = inter(lx, PS4.data.analog.stick.lx / 4.f, 0.5f); //> 獲取左模擬搖桿的插值 x 位置
    ly = inter(ly, PS4.data.analog.stick.ly / 4.f, 0.5f); //> 獲取左模擬搖桿的插值 y 位置
    rx = inter(rx, PS4.data.analog.stick.rx / 4.f, 0.5f); //> 獲取右側模擬搖桿的插值 x 位置
    ry = inter(ry, PS4.data.analog.stick.ry / 4.f, 0.5f); //> 獲取右側模擬搖桿的插值 y 位置

    if (abs(lx) > stick_min) { //> 確認左邊的X位置是否在死角
      float x0 = lx - stick_min * sign(lx); //> 減去死角角度做修正
      if (state == 1) {
        _direction.y = 0;//x0 / 10.f; 
      } else if (state != 4) {
        _direction.y = x0 / 2;
      }
    } else _direction.y = 0;

    if (abs(ly) > stick_min) { //> 確認左邊的Y位置是否在死角
      float y0 = ly - stick_min * sign(ly); //> 減去死角角度做修正
      if (state == 1) {
        _direction.x = y0 / 10.f;
        if (y0 > 0)
          vrt_offset = inter(vrt_offset, vo - 6.f, 2.f);
        else
          vrt_offset = inter(vrt_offset, vo + 3.f, 2.f);
      } else if (state != 4) {
        _direction.x = y0 / 2;
        vrt_offset = vo;
      }
    } else {
      _direction.x = 0;
      vrt_offset = vo;
    };

    if (abs(rx) > stick_min) { //> 確認右邊的X位置是否在死角
      float x1 = rx - stick_min * sign(rx); //> 減去死角角度做修正
      if (state == 1)
        turn = x1 / 16.f;
      else if (state != 4)
        turn = x1;
    } else turn = 0;

    if (abs(ry) > stick_min) { //> 確認右邊的Y位置是否在死角
      float y1 = ry - stick_min * sign(ry); //> 減去死角角度做修正
      height = y1;
    } else height = 0;
  }

  if (PS4.data.button.touchpad) { //> 檢查觸控板狀態
    if (_tb == true) {
      _tb = false; state++;
      if (state > 4) state = 0;
    }
  } else _tb = true;
}

// !! 確保已啟用換行或回車
#define _mode 1 // (0) 用於校準和測試，(1) 作為輸入 
void handle_serial() {
  //: 讀取並儲存數據
  int i = 0; float buff[3] = {0, 0, 0};
  String s_buff = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 13 || c == 32 || c == '\n') {
      buff[i] = s_buff.toFloat();
      s_buff = "";
      i++;
    } else
      s_buff += c;
  }

  if (_mode == 0) //若_mode等於0，執行修正和測試
    commands_exe(buff[0], buff[1], buff[2]);
  else if (_mode == 1) //若_mode等於1，使用序列阜當作輸入
    if (state == 4) {
      _direction = {buff[0], buff[1]};
      turn = buff[2];
    }
}

//: 用於平滑的插值函數
float inter(float in, float en, float pl) {
  if (in < en - pl) {
    return ((in * 1000.f) + (pl * 1000.f)) / 1000.0;
  } else if (in > en + pl) {
    return ((in * 1000.f) - (pl * 1000.f)) / 1000.0;
  } else return en;
}

#define properties 0
void commands_exe(float val1, float val2, float val3) {
  //: 0 用於校準關節
  if (properties == 0) {
    int leg = val1;
    int joint = val2;
    int servo = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" joint ");
    Serial.print(joint);
    Serial.print(" set to ");
    Serial.print(servo);
    Serial.print(".\n");

    set_servo(leg, joint, servo);
  }
  //: 1 用於小調整以平衡權重
  else if (properties == 1) {
    int leg = val1;
    int empty = val2;
    int ammount = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" null ");
    Serial.print(empty);
    Serial.print(" set to ");
    Serial.print(ammount);
    Serial.print(".\n");

    base_offset[leg] = ammount; 
  }
}
