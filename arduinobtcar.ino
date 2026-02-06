// arduinobtcar.ino
//--------------------------------------------------------------------------------------------------------------
// 2WD robot-car, модуль драйвера L298N. Управление с помощью Bluetooth-модуля JDY-31.
// Сонар HC-SR04, серводвигатель SG-90, OLED-дисплей 0.96" SSD1306, передние фары и задние стоп-сигналы, датчики
// скорости, пьезоэлемент. 
//--------------------------------------------------------------------------------------------------------------

#include <SoftwareSerial.h> // для управления по Seial Bluetooth
#include <Servo.h>          // для управления сервомотором
#include <Wire.h>           // для подключения дисплея по интерфейсу I2C
#include <Adafruit_GFX.h>         // для подключения графических дисплеев
#include <Adafruit_SSD1306.h>     // для OLED-дисплея SSD1306 (128x64 px),
#include <Fonts/FreeSerif9pt7b.h> // шрифты для OLED-дисплея

// подключение Bluetooth-модуля JDY-31 (дополнительно: GND, +3.3V)
#define RXpin 11
#define TXpin 12

// подключение модуля драйвера L298Т (дополнительно: GND, Vin, +5V, M1, M2)
#define ENA 5  // включение правого (по ходу движения) мотора, ШИМ
#define IN1 4  // вращение правого мотора
#define IN2 10 // вращение правого мотора
#define IN3 8  // вращение левого мотора
#define IN4 7  // вращение левого мотора
#define ENB 6  // включение левого мотора, ШИМ

// подключение фар, стоп сигналов и пьезоэлемента
#define HEADLIGHT 16  // фары (A2)
#define BACKLIGHT 13  // стоп сигналы
#define BUZZER    17  // пьезопищалка (A3)

// подключение сонара HC-SR04 (дополнительно: GND, +5V)
#define TRIG 14       // A0
#define ECHO 15       // A1

// подключение сервопривода SG-90 (дополнительно: GND, +5V)
#define SRVpin 9

#define MIN_SPEED 100         // минимальная скорость моторов; если меньше, моторы не смогут вращаться
#define MAX_SPEED 255         // максимальная скорость
#define REGULAR_SPEED 140     // регулярная скорость
#define INCREASED_SPEED 180   // повышенная скорость
#define CRITICAL 25           // критическое расстояние до препятствия в [см]
#define RADIUS_OF_WHEEL 3.3   // радиус колеса в [см]

int IRQ_left = 0;
int IRQ_right = 0;
int traveled_distance = 0;    // пройденный путь в [см]
int revolution = 0;           // число полных оборотов колеса при движении вперед
int rpm = 0;                  // число оборотов в минуту
volatile byte changecount = 0;// количество изменений импульсов (прерываний), формируемых датчиком скорости
float timetaken = 0.,         // период вращения колеса в мс
      dtime = 0.;
unsigned long prevtime;

boolean headLightOn = false;   // состояние световых приборов
boolean backLightOn = false;   
boolean forward = true;        // направление вращения колес
boolean stalemate = false;     // флаг состояния "безвыходное положение"
boolean horn = false;          // состояние звукового сигнала
SoftwareSerial mySerial(RXpin, TXpin);  // виртуальный серийный порт для Bluetooth
Servo myServo;                          // серводвигатель
byte command = 'S';                     // команда "Stop"
byte speed = 0; 
byte r;
int d=0;
Adafruit_SSD1306 display(128, 64, &Wire, -1); // инициализация OLED-дисплея

void setup() {
  pinMode(BUZZER, OUTPUT);    // зв.сигнал
  pinMode(HEADLIGHT, OUTPUT); // фары
  pinMode(BACKLIGHT, OUTPUT); // стоп-сигналы
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(RXpin, INPUT);
  pinMode(TXpin, OUTPUT);
  mySerial.begin(9600);   // виртуальный серийный порт для JDY-31
  myServo.attach(SRVpin); // привязать сервопривод к порту
  myServo.write(90);      // сонар смотрит прямо вперед
  sayBeep();
  digitalWrite(HEADLIGHT, LOW); // выключить фары
  digitalWrite(BACKLIGHT, LOW); // выключить стоп-сигналы
  Serial.begin(9600);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // если дисплей не инициализируется,
    while(true) {
      bee();  // эвуковой сигнал
      delay(500);
    }
  }
  display.setFont(&FreeSerif9pt7b);
  display.clearDisplay();
  display.setTextSize(1);             
  display.setTextColor(WHITE);        
  display.setCursor(0,15);             
  display.println("Hello, friend!");
  display.display();

  attachInterrupt(digitalPinToInterrupt(2), Left_ISR, CHANGE);  // функция вызывается по сигналу прерывания 0 от датчика левого колеса
  attachInterrupt(digitalPinToInterrupt(3), Right_ISR, CHANGE); // функция вызывается по сигналу прерывания 1 от датчика правого колеса
  delay(1000);
}

/////////////////////////////////////////
// варианты работы моторов:
// левый мотор вперед
void LMgo(byte sp) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, sp); 
}

// правый мотор вперед
void RMgo(byte sp) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, sp-3); 
}

// левый мотор назад
void LMback(byte sp) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, sp);
}

// правый мотор назад
void RMback(byte sp) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, sp-3);
}

// оба мотора стоп
void Stop() {  
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(BACKLIGHT, HIGH); // включить стоп-сигналы
  backLightOn = true;  
  rpm = 0;
}

void sayBeep() {
  tone(BUZZER, 988, 150);
  delay(200);
  tone(BUZZER, 1175, 150);
  delay(200);
  tone(BUZZER, 1568, 0);
  delay(600);
  noTone(BUZZER);
}

void boo() {
  tone(BUZZER, 185, 200);
  delay(300);
  tone(BUZZER, 65, 600);
  delay(600);
}

void bee() {
  tone(BUZZER, 1661, 90);
  delay(150);
  tone(BUZZER, 1661, 90);
  delay (150);
  tone(BUZZER, 2093, 400);
}

void Left_ISR() {
  if(forward)
    IRQ_left++;
}

void Right_ISR() {
  if(forward) {
    IRQ_right++;
    changecount++;
  }
  dtime = millis();
  if(changecount >= 40) { // 20 слотов на диске формируют 20 импульсов (или 20 передних + 20 задних фронтов = 40 прерываний) при полном обороте колеса
    timetaken = millis() - prevtime; // время в миллисекундах, затраченное на полный оборот колеса
    prevtime = millis();
    revolution++;
    changecount = 0;
  }
}

// определение дистанции до предмета с помощью сонара
int sonar_distance() {
  float dist;
  delay(100);
  digitalWrite(TRIG, LOW);    // необходимо для сброса сонара
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);   // начало измерения
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  noInterrupts();
  unsigned long duration = pulseIn(ECHO, HIGH, 23529.4); // max дистанция сенсора ~ 400 см, измерение длины импульса отклика
  interrupts();
  if(duration == 0) {  // если ошибка (сонар вне предела измерения 400 см)
    pinMode(ECHO, OUTPUT);
    digitalWrite(ECHO, LOW);
    pinMode(ECHO, INPUT);
    dist = 400;
  }
  else {
    dist = duration/58.8235; // перевод в сантиметры
  }  
  return (int)dist;
}

void showCounters() {
  char buf[20] = {0};
  display.setCursor(0, 15);
  display.println("              ");
  display.display();
  display.setCursor(0, 15);
  sprintf(buf, "iL=%d    iR=%d", IRQ_left, IRQ_right);
  display.println(buf);
  display.display();
  display.setCursor(0, 60);
  display.println("              ");
  display.display();
  for(int i = 0; i < 20; i++)
    buf[i] = ' ';
  display.setCursor(0, 60);
  sprintf(buf, "S=%d", traveled_distance);
  display.println(buf);
  display.display();
} 

void showSonarDistance(String s, int cm,  byte row, byte col) {
  char buf[20] = {0};
  display.setCursor(row, col);
  sprintf(buf, "%s=%d", s, cm);
  display.println(buf);
  display.display();
}

// запуск моторов и начало движения до встречи с препятствием
void launch(byte sp) {
  forward = true;
  stalemate = false;
  IRQ_left = 0;
  IRQ_right = 0;
  changecount = 0;
  myServo.write(90);
  int n = IRQ_right;
  unsigned long t1 = millis();
  unsigned long t2 = t1;
  do { // Движемся вперёд, пока расстояние до преграды > критического [cm] и положение не безвыходное
    LMgo(sp);          
    RMgo(sp);
    d =  sonar_distance();
    if(millis() - t2 > 200) {
      display.clearDisplay();
      showCounters();
      showSonarDistance("D", d, 40, 30);
      t2 = millis();
    }
    if((millis() - t1 > 2000) && (IRQ_right - n < 10)) // нет вращения правого колеса в течение 2 сек
      stalemate = true;
  } while((d < 400) && (d > CRITICAL) && !stalemate);
  display.clearDisplay();
  showCounters();
  showSonarDistance((char*)"D", d, 40, 30);
  Stop(); // Останов, т.к. впереди препятствие
}

// откат назад
void rollBack() {
  forward = false;
  RMback(INCREASED_SPEED);  // откатываемся назад
  LMback(INCREASED_SPEED);
  delay(500);
  Stop(); 
  myServo.write(135); // смотрим в левую сторону
  delay(500);         // задержка необходима для стабилизации положения сервопривода и сонара
  int dL = sonar_distance();
  showSonarDistance("DL", dL, 0, 45); // дистанция слева
  delay(200);
  myServo.write(45);   // смотрим в правую сторону
  delay(500);
  int dR = sonar_distance();
  showSonarDistance("DR", dR, 60, 45);  // дистанция справа
  delay(200);
  myServo.write(90);  // сонар смотрит прямо вперед
  if((dR > dL) && (dR > CRITICAL)) {
    LMgo(MIN_SPEED);   // едем правее 
    RMback(MIN_SPEED);  
    delay(700);
  }
  else if((dL > dR) && (dL > CRITICAL)) {
    RMgo(MIN_SPEED); // едем левее
    LMback(MIN_SPEED);
    delay(700);   
  }
  else if(dR <= CRITICAL) {
    RMgo(INCREASED_SPEED);    // левый разворот
    LMback(INCREASED_SPEED);  
    delay(300);
  }
  else if(dL <= CRITICAL) {
    LMgo(INCREASED_SPEED);    // правый разворот
    RMback(INCREASED_SPEED);  
    delay(300);   
  }
  Stop(); 
  tone(BUZZER, 600, 300);
} 

// выполнить команду Bluetooth
void execute(byte sp) {
  if (mySerial.available() > 0) { // если в буфере виртуального серийного порта есть данные
    command = mySerial.read();    // считываем их и запоминаем комманду
    display.clearDisplay();
    showCounters();
    display.setCursor(0,30);             
    display.println("Bluetooth mode:");
    display.display();
    display.setCursor(0,45);
    char buf[20];
    sprintf(buf, "command = %d", command);
    display.println(buf);
    display.display();
    Stop();
    switch(command) {
      case 70 : LMgo(sp);   // 'F' вперед
                RMgo(sp);
                break; 
      case 66:  LMback(sp); // 'B' назад
                RMback(sp);
                break; 
      case 76:  RMgo(MIN_SPEED);   // 'L' танковый разворот налево
                LMback(MIN_SPEED);
                break; 
      case 82:  LMgo(MIN_SPEED);   // 'R' танковый разворот направо
                RMback(MIN_SPEED);
                break; 
      case 71:  RMgo(sp);   // 'G' вперед налево
                LMgo(sp/2);
                break;
      case 73:  LMgo(sp);   // 'I' вперед направо
                RMgo(sp/2);
                break;
      case 72:  RMback(sp); // 'H' назад налево
                LMback(sp/2);
                break;
      case 74:  LMback(sp); // 'J' назад направо
                RMback(sp/2);
                break;
      case 83:  Stop();        // 'S' стоп
                speed = 100;
                break;  
      case 87:  headLightOn = true; // 'W' включить фары
                digitalWrite(HEADLIGHT, HIGH);
                break;   
      case 119: headLightOn = false; // 'w' отключить фары
                digitalWrite(HEADLIGHT, LOW);
                break; 
      case 85:  backLightOn = true;   // 'U' включить стоп-сигналы
                digitalWrite(BACKLIGHT, HIGH);
                break;
      case 117: backLightOn = false;  // 'u' отключить стоп-сигналы
                digitalWrite(BACKLIGHT, LOW);
                break;
      case 86:  horn = true;      // 'V' включить гудок
                tone(BUZZER, 500);
                break;
      case 118: horn = false;     // 'v' отключить гудок
                noTone(BUZZER);
                break;
      case 88:  Stop();// 'X' остановиться и включить аварийную сигнализацию на 10 сек
                for(int i=0; i < 10; i++) {
                  digitalWrite(HEADLIGHT, HIGH);
                  digitalWrite(BACKLIGHT, HIGH);
                  delay(500);
                  digitalWrite(HEADLIGHT, LOW);
                  digitalWrite(BACKLIGHT, LOW);
                  delay(500);
                }
                break;
      case 120: // 'x'     
                break;           
      case 113: speed = MAX_SPEED; // 'q'
                analogWrite(ENA, speed);
                analogWrite(ENB, speed);
                break;
      default:  // Символы '0'-'9' имеют коды 48-57, соответственно
                if((command > 47) && (command <= 57)) {
                  // вычитание числа 48 изменит диапазон значений с [48-57] в [0-9].
                  // умножение результата вычитания на 15 изменит диапазон с [0-9] на [0-135].
                  speed = REGULAR_SPEED + (command - 48) * 15; // в итоге speed = [0-235]
                  analogWrite(ENA, speed);
                  analogWrite(ENB, speed);
                }
    } // switch
  }
}

///////////////////////////////////////////////////////////////////////

void loop() {
  d = sonar_distance();
  delay(60); 
  if(d > CRITICAL) { // впереди путь свободен,
    speed = MIN_SPEED;
    while(true) { 
      launch(speed);     // движемся вперёд до встречи с препятствием
      boo();
      rollBack();   // откат назад с разворотом
    }
  }
  else {
    display.clearDisplay();
    display.setCursor(0,30);             
    display.println("Bluetooth mode:");
    display.display();
    display.setCursor(0,45);             
    display.println("command = ? ");
    display.display();
    speed = REGULAR_SPEED;  
    while(true) {
      execute(speed);  // выполнить команду Bluetooth
    }
  }
}
