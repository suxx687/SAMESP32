#pragma GCC optimize ("O3")   //code optimisation controls - "O2" & "O3" code performance, "Os" code size
//#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <LiquidCrystal_I2C.h>
 
//jhjhjhjjjhj
// use hardware SPI, just pass in the CS pin
// Use software SPI:         CS, DI, DO, CLK
Adafruit_MAX31865 max1 = Adafruit_MAX31865(26); //железный SPI
Adafruit_MAX31865 max2 = Adafruit_MAX31865(27);
#define RREF1 428.998 //референсное сопротивление1
#define RREF2 428.998 //референсное сопротивление2
#define COLUMS           16   //LCD столбцы
#define ROWS             2    //LCD строки
#define LCD_SPACE_SYMBOL 0x20 //space symbol from LCD ROM, see p.9 of GDM2004D datasheet
#define sb1_pin 34 // пин кнопки SB1
//#define sb2_pin 35 
#define hs1_pin 36
#define hs2_pin 39
#define hs3_pin 33
#define hs4_pin 25
#define k1_pin  19
#define k2_pin  18
#define k3_pin  17
#define k4_pin  16


//int i=0;
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
int sw = 1; // переключатель предыдущего режима работы  
float f_t1, f_t2, f_t1_ref, f_t3, f_t1_diff;

void IRAM_ATTR vlvClose(){ 
  digitalWrite (k1_pin, LOW);
  lcd.setCursor(8, 1);
  lcd.print("VC");  
}
void IRAM_ATTR vlvOpen(){
  digitalWrite (k1_pin, HIGH);
  lcd.setCursor(8, 1);
  lcd.print("VO");  
} 
void IRAM_ATTR htrOff(){
  digitalWrite (k2_pin, LOW);
  lcd.setCursor(11, 1);
  lcd.print("HOf");  
}
void IRAM_ATTR htrOn(){
  digitalWrite (k2_pin, HIGH);
  lcd.setCursor(11, 1);
  lcd.print("HOn");  
}
void IRAM_ATTR hlOn(){
  digitalWrite(k3_pin, HIGH);
}
void IRAM_ATTR hlOff(){
  digitalWrite(k3_pin, LOW);
}
void IRAM_ATTR haOn(){
  digitalWrite(k4_pin, HIGH);
}
void IRAM_ATTR haOff(){
  digitalWrite(k4_pin, LOW);
}

void setup() {
  Serial.begin(115200); //инициализация порта
  Serial.println("StartUp");
  max1.begin(MAX31865_3WIRE);  // инициализация T1
  max2.begin(MAX31865_3WIRE); // инициализация T2
  while (lcd.begin(COLUMS, ROWS, LCD_5x8DOTS) != 1) //colums, rows, characters size, SDA, SCL, I2C speed in Hz, I2C stretch time in usec 
  {
    Serial.println(F("16*2LCD is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);
  }

  lcd.print(F("LCD is OK...")); // !разобраться //(F()) saves string to flash & keeps dynamic memory free
  delay(2000);

  lcd.clear();
  max1.begin(MAX31865_3WIRE);  // инициализация вторичника 1
  max2.begin(MAX31865_3WIRE); // инициализация вторичника 2
  pinMode(sb1_pin, INPUT); 
  pinMode(hs1_pin, INPUT); pinMode(hs3_pin, INPUT); // Ext_PUp 
  pinMode(hs2_pin, INPUT); pinMode(hs4_pin, INPUT); // Ext_PUp 
  pinMode(k1_pin, OUTPUT); pinMode(k2_pin, OUTPUT); 
  pinMode(k3_pin, OUTPUT); pinMode(k4_pin, OUTPUT);
  attachInterrupt(hs1_pin, vlvOpen, FALLING);   attachInterrupt(hs2_pin, vlvClose, FALLING); //аппаратное прерывание
  attachInterrupt(hs3_pin, htrOn, FALLING);     attachInterrupt(hs4_pin, htrOff, FALLING);

 
}

void loop() {
  switch (digitalRead(sb1_pin))
      {
    case  HIGH: //ручной режим
        if (sw == 0) { 
        lcd.clear();
        sw = 1;
        vlvClose();
        htrOff();
        lcd.setCursor(15, 1);
        lcd.print("M");  
        Serial.println("Set to manual");
        }
        
      f_t1 = max1.temperature(100,RREF1); //верх колонны
      f_t2 = max2.temperature(100,RREF2); // куб
      f_t3 = random(0, 99.9); //рандом, далее вода на ВЫХОДЕ
      
      lcd.setCursor(0, 0); lcd.print("T1="); lcd.printf("%.1f ",f_t1); lcd.setCursor(8, 0); lcd.print("T2="); lcd.printf("%.1f ",f_t2); 
      lcd.setCursor(11, 1); lcd.print("T3="); lcd.printf("%.1f ",f_t3);
      if (f_t1>80 || f_t2>94.8) {
        htrOff();
        vlvClose();
      } 
      if (f_t1>79 || f_t2>93.8) {
        haOn();
        hlOn();
      } 
      if (f_t3>51) {
        haOn();
        hlOn();
      }
      if (f_t3>56) {
        htrOff();
      }
      break;                //конец ручного режима

     case LOW:              // авто режим
   if (sw == 1) {
  f_t1_ref = max1.temperature(100,RREF1);

  lcd.clear();
  lcd.setCursor(15, 1);
  lcd.print("A");
  sw = 0;
   }
   f_t1 = max1.temperature(100,RREF1); //верх колонны
     f_t2 = max2.temperature(100,RREF2); // куб
      f_t3 = random(0, 99.9); //рандом, далее вода на ВЫХОДЕ
 lcd.setCursor(0, 0); lcd.print("T1="); lcd.printf("%.1f ",f_t1); lcd.setCursor(8, 0); lcd.print("T2="); lcd.printf("%.1f ",f_t2); 
 lcd.setCursor(11, 1); lcd.print("T3="); lcd.printf("%.1f ",f_t3);   

f_t1_diff = abs(f_t1_ref-f_t1); // разница текущей и фиксированной т в колонне по модулю
  if (f_t1_diff>0.1) {   // если разница между фикс и текущей более 0,1град то включаем сигнализацию
    haOn();
    hlOn();
  }
  else {                // иначе выключаем
    haOff();
    hlOff();
  }
  if (f_t1_diff>0.2) {   // если разница более 0,2 то закрываем клапан
    vlvClose();} else if (f_t1_diff<0.1) { // если уменьшилась до 0,1 - открываем клапан
      vlvOpen();
    }
   if (f_t1>80 || f_t2>94.8) {
        htrOff();
        vlvClose();
      } 
      if (f_t1>79 || f_t2>93.8) {
        haOn();
        hlOn();
      } 
      if (f_t3>51) {
        haOn();
        hlOn();
      }
      if (f_t3>56) {
        htrOff();
      }  

/*
  uint16_t rtd = max1.readRTD();//delay(2000);
  uint16_t rtd1 = max2.readRTD();
Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();
  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  float ratio1 = rtd1;
  ratio /= 32768;
  ratio1 /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF1*ratio,8);
  Serial.print("Temperature = ");Serial.println(max1.temperature(100, RREF1));
  Serial.println();Serial.println("##################################################################");Serial.println();Serial.println();
  Serial.print("RTD1 value: "); Serial.println(rtd1);
  Serial.print("Ratio1 = "); Serial.println(ratio1,8);
  Serial.print("Resistance1 = "); Serial.println(RREF2*ratio1,8);
  Serial.print("Temperature1 = ");Serial.println(max2.temperature(100, RREF2));


//for (int i = 0; i>=0; i++ ) {
//ttemp += max1.temperature(100,RREF);
//Serial.print("meas "); Serial.println (i);
//}  
//ttemp = ttemp/50;
//Serial.print("AVGtemperature = "); Serial.println(ttemp);

  // Check and print any faults
 /* uint8_t fault = max1.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    max1.clearFault();
  }
 // Serial.println(); 
lcd.setCursor(0, 0);             //set 15-th colum & 3-rd  row, 1-st colum & row started at zero
  lcd.print("T1=");lcd.print(max1.temperature(100,RREF1));
  lcd.write(LCD_SPACE_SYMBOL);
lcd.setCursor(0, 1);             //set 15-th colum & 3-rd  row, 1-st colum & row started at zero
  lcd.print("T2=");lcd.print(max2.temperature(100,RREF2));
  lcd.write(LCD_SPACE_SYMBOL);lcd.write(LCD_SPACE_SYMBOL);
  lcd.print("AUTO");

  //delay(1000); */
   break;
  }
}

