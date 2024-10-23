#include <SPI.h> // библа для последовательной шины        
#include <nRF24L01.h> // библа для радиомодуля
#include <RF24.h> // библа для радио
#include <Servo.h> // библа для работы с серво
const uint64_t pipe = 0xF0F1F2F3F4LL; // канал связи
RF24 radio(9, 10); // регистры ячейки ид. № для общения
byte msg[9]; // массив данных
unsigned long time; // времечко
int data; // данные
int pos;
int fspeed;           // вперед скорость 
int bspeed;           // назад скорость
const int in1 = 2;    // пин 1
const int in2 = 4;    // пин 2
const int in3 = 7;    // ШИМ для скорости
const int in4 = 8;    // пин 4
const int ena = 5;    // ШИМ для скорости
const int enb = 6;    // ШИМ для скорости
unsigned char RightMotor[3] =   {in1, in2, ena};//моторчик правая ось
unsigned char LeftMotor[3] =   {in3, in4, enb};// моторчик левая ось
void Wheel (unsigned char * motor, int v)//функция колес
{
  if (v>100) v=100;//не более 100 значение
  if (v<-100) v=-100;//не менее -100 значение для движ назад
  if (v>0) {// если скорость положительная бежим вперед
    digitalWrite(motor[0], HIGH);
    digitalWrite(motor[1], LOW);
    analogWrite(motor[2], v*2.55);
  }
  else if (v<0) { // если отрицательная назад бежим
    digitalWrite(motor[0], LOW);
    digitalWrite(motor[1], HIGH);
    analogWrite(motor[2], (-v)*2.55);
  }
  else { // а иначе стоим ждем
    digitalWrite(motor[0], LOW);
    digitalWrite(motor[1], LOW);
    analogWrite(motor[2], 0);
  }
}
void setup(){// основная функция
  pinMode(in1, OUTPUT);    // пин 
  pinMode(in2, OUTPUT);    // пин
  pinMode(in3, OUTPUT);    // пин 
  pinMode(in4, OUTPUT);    // пин  
  pinMode(ena, OUTPUT);    // пин ШИМ
  pinMode(enb, OUTPUT);    // пин ШИМ
  Serial.begin(9600); // начали передачу
 delay(50); // прерывание 50 мс
 radio.begin(); // радио вкл
 radio.setChannel(9); // канальчик
 radio.setPayloadSize(8); // размерчик
 radio.setDataRate(RF24_250KBPS);  //битрейт
 radio.setPALevel(RF24_PA_HIGH); // приоритет
 radio.openReadingPipe(1,pipe); // прием передача
 radio.startListening(); // начинаем прием
}
void loop() {  // цикл вся тема тут 
 if (radio.available()){ // если все ок радио слышит
  time = millis(); // добавляем пинка
radio.read(&msg, sizeof(msg));} // читаем с пульта че там есть
if (millis() - time > 1000){ // если голяк
msg[0]=0;} // онуляем 0 ячейку
int x= map(msg[2], 0, 255, -100, 100);// кидаем в х значения полученные во 2 ячейке
int y= map(msg[1], 0, 255, -100, 100);// кидаем в у значения с 1 ячейки полученного массива
  Wheel (RightMotor, y - x);//крутим колесами правыми в зависимости от сигнала по осям
  Wheel (LeftMotor, y + x); //крутим колесами левыми в зависимости от сигнала по осям
Serial.println(msg[1]);//выводим у в отладчик
Serial.println(msg[2]);//выводим х в отладчик
}
