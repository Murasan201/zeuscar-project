/*******************************************************************
 * basic_move
 
  Control the direction and speed of motors rotation by pwm,
  to make the car go forward, backward, left turn, right turn and stop.

******************************************************************/
#include <Arduino.h>
#include <SoftPWM.h>

// 各動作に対応する定数の定義
#define FORWARD        "FORWARD"
#define BACKWARD       "BACKWARD"
#define LEFT           "LEFT"
#define RIGHT          "RIGHT"
#define LEFTFORWARD    "LEFTFORWARD"
#define RIGHTFORWARD   "RIGHTFORWARD"
#define LEFTBACKWARD   "LEFTBACKWARD"
#define RIGHTBACKWARD  "RIGHTBACKWARD"
#define TURNLEFT       "TURNLEFT"
#define TURNRIGHT      "TURNRIGHT"
#define STOP           "STOP"

/*
 *  [0]--|||--[1]
 *   |         |
 *   |         |
 *   |         |
 *   |         |
 *  [3]-------[2]
 */
/** Set the pins for the motors */
//モーターピンの設定
#define MOTOR_PINS \
  (uint8_t[8]) { \
    3, 4, 5, 6, A3, A2, A1, A0 \
  }
/** Set the positive and negative directions for the motors */
//モーターの正逆の方向を設定
#define MOTOR_DIRECTIONS \
  (uint8_t[4]) { \
    1, 0, 0, 1 \
  }

#define MOTOR_POWER_MIN 28  // 最小モーターパワー28/255

//PWM初期化処理のプロトタイプ宣言
void carBegin();

int8_t power = 80;

void setup() {
  Serial.begin(9600);  // シリアル通信の開始
  Serial.println("Zeus Car basic move");
  SoftPWMBegin();  //ソフトウェアPWMの初期化
  carBegin();      // モーターの初期化
}

void loop() {
  if (Serial.available() > 0) {
    //Raspberry Piから指令値を受信（改行単位）
    String receivedStr = Serial.readStringUntil('\n');
    Serial.flush();  // 受信バッファをクリア

    // デバッグ用の出力
    Serial.print("Received: ");
    Serial.println(receivedStr);

    // 文字列で比較してコマンドを処理
    if (receivedStr == FORWARD) {
      carForward(power);
    } else if (receivedStr == BACKWARD) {
      carBackward(power);
    } else if (receivedStr == LEFT) {
      carLeft(power);
    } else if (receivedStr == RIGHT) {
      carRight(power);
    } else if (receivedStr == LEFTFORWARD) {
      carLeftForward(power);
    } else if (receivedStr == RIGHTFORWARD) {
      carRightForward(power);
    } else if (receivedStr == LEFTBACKWARD) {
      carLeftBackward(power);
    } else if (receivedStr == RIGHTBACKWARD) {
      carRightBackward(power);
    } else if (receivedStr == TURNLEFT) {
      carTurnLeft(power);
    } else if (receivedStr == TURNRIGHT) {
      carTurnRight(power);
    } else if (receivedStr == STOP) {
      carStop();
    } else {
      Serial.println("Unknown command");
    }

    // 実行中のコマンドを送信（デバッグ用）
    Serial.print("Executing command: ");
    Serial.println(receivedStr);
    
    delay(100);  // 短めの待機時間を設定
  }
}

//PWM初期化処理
void carBegin() {
  for (uint8_t i = 0; i < 8; i++) {
    SoftPWMSet(MOTOR_PINS[i], 0);
    SoftPWMSetFadeTime(MOTOR_PINS[i], 100, 100);
  }
}

//PWM出力処理
/*
引数：各モータの出力値　power0,  power1, power2, power3
　　　+値：正転
　　　-値：逆転
*/
void carSetMotors(int8_t power0, int8_t power1, int8_t power2, int8_t power3) {
  bool dir[4];
  int8_t power[4] = { power0, power1, power2, power3 };
  int8_t newPower[4];

  for (uint8_t i = 0; i < 4; i++) {
    dir[i] = power[i] > 0;

    if (MOTOR_DIRECTIONS[i]) dir[i] = !dir[i];

    if (power[i] == 0) {
      newPower[i] = 0;
    } else {
      newPower[i] = map(abs(power[i]), 0, 100, MOTOR_POWER_MIN, 255);
    }
    SoftPWMSet(MOTOR_PINS[i * 2], dir[i] * newPower[i]);
    SoftPWMSet(MOTOR_PINS[i * 2 + 1], !dir[i] * newPower[i]);
  }
}

void carForward(int8_t power) {
  carSetMotors(power, power, power, power);
}

void carBackward(int8_t power) {
  carSetMotors(-power, -power, -power, -power);
}

void carLeft(int8_t power) {
  carSetMotors(-power, power, -power, power);
}

void carRight(int8_t power) {
  carSetMotors(power, -power, power, -power);
}

void carLeftForward(int8_t power) {
  carSetMotors(0, power, 0, power);
}

void carLeftBackward(int8_t power) {
  carSetMotors(-power, 0, -power, 0);
}

void carRightForward(int8_t power) {
  carSetMotors(power, 0, power, 0);
}

void carRightBackward(int8_t power) {
  carSetMotors(0, -power, 0, -power);
}

void carTurnLeft(int8_t power) {
  carSetMotors(-power, power, power, -power);
}

void carTurnRight(int8_t power) {
  carSetMotors(power, -power, -power, power);
}

void carStop() {
  carSetMotors(0, 0, 0, 0);
}
