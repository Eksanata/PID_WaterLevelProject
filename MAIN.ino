#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 200

int enA = 10;
int in1 = 9;
int in2 = 8;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
LiquidCrystal_I2C lcd(0x27, 16, 2); //Alamat I2C yang digunakan 0x27. 
                                    //Alamat I2C yang lain 0x3F
//konfigurasi pin I2C
//SDA A4
//SCL A5

float SV, PV, PVf, PVf_1, Kp, Ti, Td, Ki, Kd;
float SVfix, PVfix;
float RC, a, fc;
float PID;
float et, et_1;
float eint, eint_1, eint_update;
float edif;
int MV;

unsigned long t;
double t_1, Ts, Tsf;
float interval_elapsed, interval_limit;

void setup() {
  // put your setup code here, to run once:
  // Hasil desain kendali
  // L = 0.042444 dan T = 0.013231
  Kp = 0.374;//5 0.374;
  Ti = 4.553;//0.1;//0.254;//0.374, 0.085;
  Td = 1.382; 0.05;//0.021;

  if (Ti == 0){
    Ki = 0;
  }
  else{
    Ki = Kp/Ti;
  }
  Kd = Kp*Td;

  et_1 = 0;
  eint_1 = 0;
  interval_elapsed = 0;
  interval_limit = 2;
  t = millis();

  fc = 0.0089; //0.477; //0.047; //0.47; //0.0477 //0.377
  PVf_1 = 0;
  
  Serial.begin(9600);
  lcd.begin();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SV : ");
  lcd.setCursor(0,1);
  lcd.print("PV : ");
  lcd.setCursor(11,0);
  lcd.print("cm");
  lcd.setCursor(11,1);
  lcd.print("cm");

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
    Ts = 0.01;
//    Tsf = 0.7026;
    RC = 1/(6.28*fc);
    a = RC/Ts;
    SV = analogRead(A0)*0.004887; 
    SVfix = SV*2.8; // 1 Volt = 2.8 cm
//    SVfix = 5;
//    PV = sonar.ping_cm();
    float uS = sonar.ping();
    PV = (uS/US_ROUNDTRIP_CM);
    PVf = (PV + a*PVf_1)/(a + 1);
//    PVfix = (-2.64)*PVf + 38.28;
    PVfix = (-0.72)*PVf + 12.096; // kalibrasi sensor
//    PVfix = 15.47 - PVf;
//    PVfix = (-0.8)*PVf + 11.1;
//    PVfix = PVf;
    t = millis();
    et = SVfix - PVfix;
    eint_update = ((et + et_1)*Ts)/2;
    eint = eint_1 + eint_update;
    edif = (et - et_1)/Ts;
    PID = Kp*et + Ki*eint + Kd*edif;
    PID = PID/1;
    if(PID > 10){
      PID = 10;
    }
    else if(PID < 0){
      PID = 0;
    }
    else{
      PID = PID;
    }
    PID = PID/2;
    MV = PID*51;
//    MV = SV*51;
    nyala();
//    analogWrite(enA, MV);
//    digitalWrite(in1, HIGH);
//    digitalWrite(in2, LOW);
    interval_elapsed = interval_elapsed + Ts;
    if(interval_elapsed >= interval_limit){
      Serial.print("20");
      Serial.print(" ");
      Serial.print("0");
      Serial.print(" ");
      Serial.print(SVfix);
      Serial.print(" ");
      Serial.println(PVfix);
//      Serial.print(" ");
//      Serial.println(MV);
    
      lcd.setCursor(5,0);
      lcd.print(SVfix);
      lcd.setCursor(5,1);
      lcd.print(PVfix);
      interval_elapsed = 0;
      t_1 = t;
      et_1 = et;
      eint_1 = eint;
    }
    PVf_1 = PVf;
}

void nyala(){
  analogWrite(enA, MV);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
