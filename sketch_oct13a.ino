#include <cmath>

const int PULLUP_pin = 15;

//左前ステアリング
const int swFRU = 1;  //モーター出力ピン
const int swFRD = 2;
const int encoderPinA = 3;         // A相のピン (GPIO 3)
const int encoderPinB = 4;         // B相のピン (GPIO 4)
const int FRU_re = 5, FRD_re = 6;  //モーター反転ピン上下
int FRU_mo = 0, FRD_mo = 0;

const int Pulses = 24 * 4;

int lastEncode = 0;

int FR_en = 0;

const int pg = 0.2, ig = 0.001, dg = 0.05;

unsigned long previousMillis = 0;  // 前回時間を保存する変数
const int interval = 2;          //~ms秒周期で出力を変化させる

int swerve_time = 0;

double rv_i = 0.0;

void Swerve_control(int encode, int mo_u, int mo_d, double tx, double ty, int Umo_pin, int Dmo_pin, int Umo_re, int Dmo_re);
void updateLedAndSerial();



void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  pinMode(PULLUP_pin, INPUT_PULLUP);

  pinMode(swFRU, OUTPUT);
  pinMode(swFRD, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  int a = digitalRead(encoderPinA);
  int b = digitalRead(encoderPinB);
  pinMode(FRU_re, OUTPUT);
  pinMode(FRD_re, OUTPUT);

  delay(3000);

  analogWrite(swFRU,100);
  delay(1000);
  analogWrite(swFRD,100);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  int buttonState = digitalRead(PULLUP_pin);
  int a = digitalRead(encoderPinA);
  int b = digitalRead(encoderPinB);

  double FR_x = 1, FR_y = 0, FR_v;

  int encode = (a << 1) | b;             //二桁の二進数で今のエンコーダの値を記録している
  int sum = lastEncode | (encode << 2);  //前のエンコーダの値を入れて、4桁の二進数にしてる

  unsigned long currentMillis = millis();  // 現在の時刻を取得

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    FR_en += 1;
    if (FR_en >= Pulses) FR_en = 0;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    FR_en -= 1;
    if (FR_en < 0) FR_en = Pulses - 1;
  }


  if (currentMillis - previousMillis >= interval) {  //実行する量を少なくするために今の時間見て色々やってるところ
    previousMillis = currentMillis;                  // 時刻を更新

    //テスト走行
    if (swerve_time < 2000) {
      FR_x = 1;
      FR_y = 0;
    } else if (swerve_time >= 2000 && swerve_time < 4000) {
      FR_x = 0;
      FR_y = 1;
    } else if (swerve_time >= 4000 && swerve_time < 6000) {
      FR_x = -1;
      FR_y = 0;
    } else if (swerve_time >= 6000 && swerve_time < 8000) {
      FR_x = 1;
      FR_y = 0;
    } else if (swerve_time >= 8000 && swerve_time < 10000) {
      FR_x = 0.7;
      FR_y = 0.7;
    } else if (swerve_time >= 10000 && swerve_time < 12000) {
      FR_x = 1;
      FR_y = 0;
    } else if (swerve_time >= 12000 && swerve_time < 14000) {
      FR_x = (swerve_time - 12000)/2000.0;
      FR_y = 1 - (swerve_time - 12000)/2000.0;
    } else if (swerve_time >= 14000) {
      swerve_time = 0;
    }

    if(0 == swerve_time % 2000){
      rv_i = 0;
    }

    updateLedAndSerial();

    Swerve_control(FR_en, FRU_mo, FRD_mo, FR_x, FR_y, swFRU, swFRD, FRU_re, FRD_re);

    swerve_time++;
  }

  lastEncode = encode;
}


void updateLedAndSerial() {
  
  // Serial.println(FR_en);
}


//現在の向き(エンコーダの値),二つのモーターの現在の出力(up,down),目標の向き(x,y),モーターのpwmのピン番号,モーター反転のピン番号
void Swerve_control(int encode, int mo_u, int mo_d, double tx, double ty, int Umo_pin, int Dmo_pin, int Umo_re, int Dmo_re) {

  int objective_angle_encode = 0;  //目標角度(エンコーダの値)

  double rv = 0.0;       //どのくらい回転すればいいかを入れる変数(-24 ~ 24)
  double tv = 0.0;  //どのくらいの出力で回すべきか入れるところ。
  int Bsw = 0;      //後ろ方向にステアを回転するとき1にするやつ
  int U_mo_pwm = 0, D_mo_pwm = 0;

  int steering_rate = 220;  //行きたい方向からステアの向きがどのくらいずれているかで、モーター出力を旋回に使うか、移動に使うかを入れる値(旋回の量(0~255))


  //目標のエンコーダの値の計算。(アークタンジェント(x/y)/3.14) * 180 * エンコーダのパルス数 / 360
  objective_angle_encode = (atan2(ty, tx) / 3.14159) * Pulses / 2.0 ;

  //三平方の定理を使って移動する出力を計算
  tv = std::sqrt(tx * tx + ty * ty);



  //どの方向に回すかの計算。Bswにステアの回転方向、rvにどのくらい旋回するか入れてる。(計算方法もっと良さそうなのありそう)

  // if(encode < pulses / 4.0 * 3 && encode > pulses / 4.0){
  //   if(pulses > encode - pulses / 4.0 || pulses < ){
  // }
  // }

  // if(encode < 0){
  //   encode += Pulses;
  // }else{
  //   encode %= Pulses;
  // }


  //無駄な条件分岐があるので変更した方が良い。先に90°以上目標角度から離れているか見てからの方がよさそう。

  // if(Pulses >= (objective_angle_encode - encode) * 4 || Pulses <= (objective_angle_encode - encode) * 4){
  //   rv = objective_angle_encode - encode;
  // }else if(Pulses <= (objective_angle_encode - encode) * 4 / 3){
  //   rv = objective_angle_encode - encode - 360;
  // }else if(Pulses >= (objective_angle_encode - encode)){
  //   rv = objective_angle_encode - encode + 360;
  // }else{
  //   Bsw = 1;
  //   if(objective_angle_encode <= Pulses / 2.0){
  //     objective_angle_encode += Pulses / 2.0;
  //   }else{
  //     objective_angle_encode -= Pulses / 2.0;
  //   }
  //   if(Pulses >= (objective_angle_encode - encode) * 4 || Pulses <= (objective_angle_encode - encode) * 4){
  //     rv = objective_angle_encode - encode;
  //   }
  //   // else if(Pulses <= (objective_angle_encode - encode) * 4 / 3){
  //   //   rv = objective_angle_encode - encode - 360;
  //   // }else if(Pulses >= (objective_angle_encode - encode)){
  //   //   rv = objective_angle_encode - encode + 360;
  //   // }
  // }

  //新しい計算方法
  rv = ((int)(objective_angle_encode - encode + Pulses + Pulses / 2.0) % Pulses) - Pulses / 2.0;

  if (rv < -Pulses / 4) {
    rv += Pulses / 2.0;
    Bsw = 1;
  } else if (rv > Pulses / 4) {
    rv -= Pulses / 2.0;
    Bsw = 1;
  } else {
    Bsw = 0;
  }

  rv_i += rv * 0.1;

  if(objective_angle_encode == encode || objective_angle_encode % 48 == encode %48){
    rv_i = 0;
  }



  //更に新しい計算方法
  /*
  rv = ((objective_angle_encode - encode + Pulses) % Pulses);

  if(rv >= Pulses/4.0 * 3){
    rv -= Pulses
  }else if(rv >= Pulses /2.0){
    
  }

  
  */

  ////////////////////////////////////////////////////////////////


  //モーター出力反転
  // if(Bsw == 1){
  //   digitalWrite(Umo_pin,LOW);
  //   digitalWrite(Dmo_pin,HIGH);
  // }else{
  //   digitalWrite(Umo_pin,HIGH);
  //   digitalWrite(Dmo_pin,LOW);
  // }

  //(要調整)旋回と移動のモーター出力の割合を、目標角度と現在の角度で変更している
  if (rv > 4 || rv < -4) {
    steering_rate = 255;
  } else if (rv > 1) {
    steering_rate = (50 / 4) * rv;
  } else if (rv < -1) {
    steering_rate = (50 / 4) * -rv;
  } else {
    steering_rate = 10;
  }




  //反転したとき、旋回の出力を上と下逆にしなきゃいけないので(多分、あんまり考えてない、何となく)
  // if(Bsw == 1){
  //   rv *= -1;
  // }


  //モーター出力計算
  // U_mo_pwm = (255-steering_rate) * tv - (steering_rate/255) * rv * 0.2;
  // if(U_mo_pwm > mo_u + 4){//どのくらいゆっくり出力を上げていくか調整
  //   U_mo_pwm = mo_u + 4;
  // }
  // if(U_mo_pwm > 255){
  //   U_mo_pwm = 255;
  // }

  // D_mo_pwm = (255-steering_rate) * tv + steering_rate * rv * 0.2;
  // if(D_mo_pwm > mo_d + 4){//どのくらいゆっくり出力を上げていくか調整
  //   D_mo_pwm = mo_d + 4;
  // }
  // if(D_mo_pwm > 255){
  //   D_mo_pwm = 255;
  // }

  // Serial.print("rv = ");
  // Serial.println(rv);

  if (Bsw == 1) {
    U_mo_pwm = (255 - steering_rate) * tv + (steering_rate / 255.0) * (rv + rv_i) * 255 / (double)24;
    D_mo_pwm = -(255 - steering_rate) * tv + (steering_rate / 255.0) * (rv + rv_i) * 255 / (double)24;
  } else {
    U_mo_pwm = -(255 - steering_rate) * tv + (steering_rate / 255.0) * (rv + rv_i) * 255 / (double)24;
    D_mo_pwm = (255 - steering_rate) * tv + (steering_rate / 255.0) * (rv + rv_i) * 255 / (double)24;
  }


  // if(U_mo_pwm > mo_u){
  //   U_mo_pwm = mo_u + 2;
  // }else if(U_mo_pwm < mo_u){
  //   U_mo_pwm = mo_u - 2;
  // }

  // if(D_mo_pwm > mo_d){
  //   D_mo_pwm = mo_d + 2;
  // }else if(D_mo_pwm < mo_d){
  //   D_mo_pwm = mo_d - 2;
  // }

  if (U_mo_pwm < -255) {
    U_mo_pwm = -255;
  } else if (U_mo_pwm > 255) {
    U_mo_pwm = 255;
  }
  if (D_mo_pwm < -255) {
    D_mo_pwm = -255;
  } else if (D_mo_pwm > 255) {
    D_mo_pwm = 255;
  }

  // Serial.print("モーターの出力(U,D)\n");
  // Serial.println(U_mo_pwm);
  // Serial.println(D_mo_pwm);


  if (U_mo_pwm < 0) {
    digitalWrite(Umo_re, HIGH);
    U_mo_pwm *= -1;
  } else {
    digitalWrite(Umo_re, LOW);
  }

  if (D_mo_pwm < 0) {
    digitalWrite(Dmo_re, HIGH);
    D_mo_pwm *= -1;
  } else {
    digitalWrite(Dmo_re, LOW);
  }

  analogWrite(Umo_pin, U_mo_pwm);
  analogWrite(Dmo_pin, D_mo_pwm);

  // Serial.println(U_mo_pwm);
  // Serial.println(D_mo_pwm);
  Serial.print(rv + rv_i);
  Serial.print(",");
  Serial.print(rv);
  Serial.print(",");
  Serial.print(rv_i);
  Serial.print(",");
  Serial.print(encode);
  Serial.print(",");
  Serial.println(objective_angle_encode);
  // Serial.println(tx);
  // Serial.println(ty);


  FRU_mo = U_mo_pwm;
  FRD_mo = D_mo_pwm;
}
