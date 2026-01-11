#include <WiFi.h>
#include <WebServer.h>
#include <LiquidCrystal_I2C.h>

const char* ssid = "ESP32_ROBOT";
const char* password = "12345678";
// bool robotRun = false;   // false = STOP, true = RUN

WebServer server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================= PIN MOTOR L293N =================
#define IN1 25   // Motor Kiri
#define IN2 26
#define IN3 27   // Motor Kanan
#define IN4 14

// ================= SENSOR IR (ADC1) =================
#define IR_LEFT   32
#define IR_MID    33
#define IR_RIGHT  34

int sensorValue[3];
int threshold = 2750;   // SESUAIKAN (0–4095)

// ================= PID PARAMETER ====================
float Kp = 6.0;
// float Ki = 0.0;
float Kd = 0.0;

int P, I, D;
int previousError = 0;
int baseSpeed = 100;
// int maxSpeed = 150;

// ================= PWM ESP32 ========================
#define PWM_FREQ  1000
#define PWM_RES   8   // 0–255

#define CH_IN1 0
#define CH_IN2 1
#define CH_IN3 2
#define CH_IN4 3

// // ===================================================
// void handleStart() {
//   robotRun = true;
//   server.send(200, "text/html", "Robot START");
// }

// void handleStop() {
//   robotRun = false;
//   stopMotor();   // pastikan motor mati
//   server.send(200, "text/html", "Robot STOP");
// }

void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP Started");
  Serial.println(WiFi.softAPIP()); // biasanya 192.168.4.1

  // ===== REGISTER ROUTE DULU =====
  server.on("/", handleRoot);
  // server.on("/start", handleStart);
  // server.on("/stop", handleStop);
  server.on("/set", handleSet);
  // server.on("/reset", handleReset);   // kalau kamu pakai reset

  server.begin();   // <-- BARU BEGIN DI SINI

  // ===== LCD =====
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("ESP32 LF Ready");
  delay(1000);
  lcd.clear();

  // ===== PWM =====
  ledcSetup(CH_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN3, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN4, PWM_FREQ, PWM_RES);

  ledcAttachPin(IN1, CH_IN1);
  ledcAttachPin(IN2, CH_IN2);
  ledcAttachPin(IN3, CH_IN3);
  ledcAttachPin(IN4, CH_IN4);
  
  Serial.println("ESP32 Line Follower Ready");
}

// void stopMotor() {
//   // Motor kiri
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, LOW);

//   // Motor kanan
//   digitalWrite(IN3, LOW);
//   digitalWrite(IN4, LOW);
// }



void handleSet() {
  if (server.hasArg("kp")) {
    Kp = server.arg("kp").toFloat();
  }

  // if (server.hasArg("ki")) {
  //   Ki = server.arg("ki").toFloat();
  // }

  if (server.hasArg("kd")) {
    Kd = server.arg("kd").toFloat();
  }

  if (server.hasArg("spd")) {
    baseSpeed = server.arg("spd").toInt();
  }

  String response =
    "Updated!<br>"
    "Kp = " + String(Kp) + "<br>" +
    // "Ki = " + String(Ki) + "<br>" +
    "Kd = " + String(Kd) + "<br>" +
    "Speed = " + String(baseSpeed);

  server.send(200, "text/html", response);

    updateLCD();   // 🔥 update LCD di sini
  
  Serial.println(response);
}

void handleRoot() {
  String html =
    "<h2>ESP32 Robot Control</h2>"
    "<form action='/set'>"
    "Kp: <input type='text' name='kp'><br>"
    // "Ki: <input type='text' name='ki'><br>"
    "Kd: <input type='text' name='kd'><br>"
    "Speed: <input type='text' name='spd'><br>"
    "<input type='submit' value='Send'>"
    // "<a href="/start"><button>START</button></a>"
    // "<a href="/stop"><button>STOP</button></a>"
    "</form>";


  server.send(200, "text/html", html);
}

void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("KP:");
  lcd.print(Kp, 2);   // 2 angka di belakang koma
  lcd.print("   ");  // hapus sisa karakter

  // lcd.setCursor(0, 1);
  // lcd.print("KI:");
  // lcd.print(Ki, 2);   // 2 angka di belakang koma
  // lcd.print("   ");  // hapus sisa karakter

  lcd.setCursor(8, 0);
  lcd.print("KD:");
  lcd.print(Kd, 2);   // 2 angka di belakang koma
  lcd.print("   ");  // hapus sisa karakter

  lcd.setCursor(0, 1);
  lcd.print("SPD:");
  lcd.print(baseSpeed);
  lcd.print("   ");
}


// ===================================================
//                BACA SENSOR IR
// ===================================================
void bacaSensor() {
  sensorValue[0] = analogRead(IR_LEFT);
  sensorValue[1] = analogRead(IR_MID);
  sensorValue[2] = analogRead(IR_RIGHT);
}

// ===================================================



// ===================================================
//            HITUNG ERROR (3 SENSOR)
// ===================================================
int hitungError() {

  bool L = sensorValue[0] > threshold;
  bool M = sensorValue[1] > threshold;
  bool R = sensorValue[2] > threshold;

  /*
      Pola:
      L M R
  */

  if (M && !L && !R) return 0;
  if (L && !M && !R) return -2000;
  if (R && !M && !L) return 2000;

  if (L && M && !R) return -1000;
  if (R && M && !L) return 1000;

  // Garis hilang → lanjut arah terakhir
  if (!L && !M && !R) {
    if (previousError > 0) return 2000;
    else return -2000;
  }

  return 0;
}



// ===================================================
//                  PID CONTROL
// ===================================================
void PID_LineFollow(int error) {
  P = error;
  I += error;
  D = error - previousError;

  float PIDvalue = (Kp * P) + (Kd * D);
  previousError = error;

  int leftSpeed  = baseSpeed - PIDvalue;
  int rightSpeed = baseSpeed + PIDvalue;

  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

// leftSpeed  = constrain(leftSpeed, -maxSpeed, maxSpeed);
// rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  motor_L293N(leftSpeed, rightSpeed);
}

// ===================================================
//              MOTOR CONTROL L293N
// ===================================================
void motor_L293N(int left, int right) {

  // MOTOR KIRI
  if (left > 0) {
    ledcWrite(CH_IN3, left);
    ledcWrite(CH_IN4, 0);
  } else {
    ledcWrite(CH_IN3, 0);
    ledcWrite(CH_IN4, abs(left));
  }

  // MOTOR KANAN
  if (right > 0) {
    ledcWrite(CH_IN1, right);
    ledcWrite(CH_IN2, 0);
  } else {
    ledcWrite(CH_IN1, 0);
    ledcWrite(CH_IN2, abs(right));
  }
}

void loop() {
  server.handleClient();

  // if (!robotRun) {
  //   stopMotor();
  //   return;   // 🔥 hentikan eksekusi PID

  bacaSensor();
  int error = hitungError();
  PID_LineFollow(error);
  
}