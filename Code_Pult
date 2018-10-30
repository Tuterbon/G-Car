#include <SPI.h>   //библа для последовательной шинки     
#include <nRF24L01.h> // библа для радиомодуля
#include <RF24.h> // тоже шняга для радиомодуля
const uint64_t pipe = 0xF0F1F2F3F4LL; // канал связи между тачкой и пультом
RF24 radio(9, 10); // канал ( регистры ) типа частота чтоб общаться с тачкой через пульт
byte button_jostik = 5; // кнопочка будет для переключения между режимом гиро и джойстик
byte transmit_data[9]; // массив для хранения данных
byte latest_data[9]; // еще один массив для хранения данных 
boolean flag; // переменная 1\0
#include <Wire.h> // библа для гирыча
#include "Kalman.h" // библа для расчета углов осей лес темный просто юзаем все просчитает
Kalman kalmanX;
Kalman kalmanY;
uint8_t IMUAddress = 0x68;
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
double accXangle; 
double accYangle;
double temp;
double gyroXangle = 180; 
double gyroYangle = 180;
double compAngleX = 180; 
double compAngleY = 180;
double kalAngleX; 
double kalAngleY;
uint32_t timer;
boolean lastButten = LOW;     
boolean currentButten = LOW;  
int status = 0; // переменная чтоб чекать статус      
void setup() { // основная функция
  Serial.begin(9600); // начинаем общение
  pinMode(button_jostik, INPUT_PULLUP);  // пулап для инвертирования сигнала (так проще)
  radio.begin(); // радио вкл
    delay(50); // прерывание в 50мс
    radio.setChannel(9); // канальчик связи
    radio.setPayloadSize(8); // размер массива данных
    radio.setRetries(1,1); // попытки связи с двух концов
    radio.setDataRate(RF24_250KBPS); // битрейт
    radio.setPALevel(RF24_PA_HIGH); //приоритет
    radio.openWritingPipe(pipe); // открываем канальчик
    Wire.begin(); // пошло чтение гира
  i2cWrite(0x6B,0x00); // выводим из сна гиру
  if(i2cRead(0x75,1)[0] != 0x68) { // чекаем на месте ли гира
    Serial.print(F("MPU-6050 with address 0x")); // если че пишем в отладчик мол нет гиры с адресом ...
    Serial.print(IMUAddress,HEX); // адрес ...
    Serial.println(F(" is not connected")); // не подключен ...
    while(1) // пошел цикл
  }    
  kalmanX.setAngle(180); // запускаем углы по оси Х в диапазоне 180
  kalmanY.setAngle(180); // запускаем углы по оси У в диапазоне 180
  timer = micros(); // ускоряем время рабочей операции
}
void loop() { // бесконечный цикл 
  currentButten = debvance (lastButten);  // дребезг контактов
  if (lastButten == LOW && currentButten == HIGH)  // если нажали
  {
    status = !status; // инвертируем статус получаем стабильно крутяг
  }
  lastButten =  currentButten;  // переносим значение в нашу переменную
  if (status == true) // если все окей врубается джойстик
  {
    transmit_data[1] = map(analogRead(A0), 0, 1023, 0, 255); //в 1 ячейку кидаем значения по горизонтали вращения джойстика
    transmit_data[2] = map(analogRead(A1), 0, 1023, 0, 255); // во 2 ячейку кидаем значения по вертикали вращения джойстика
    }
  else // иначе пошло поехало в гиры читаем данные и кидаем их ячйки массива хранения данных
  {
    uint8_t* data = i2cRead(0x3B,14);
    accX = ((data[0] << 8) | data[1]);
    accY = ((data[2] << 8) | data[3]);
    accZ = ((data[4] << 8) | data[5]);
    tempRaw = ((data[6] << 8) | data[7]);
    gyroX = ((data[8] << 8) | data[9]);
    gyroY = ((data[10] << 8) | data[11]);
    gyroZ = ((data[12] << 8) | data[13]);
    accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
    accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;  
    double gyroXrate = (double)gyroX/131.0;
    double gyroYrate = -((double)gyroY/131.0);
    gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); 
    gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
    gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); 
    gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
    compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle);
    compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);
    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); 
    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
    timer = micros();
    temp = ((double)tempRaw + 12412.0) / 340.0;
    Serial.print(kalAngleX);Serial.print("\t");
    Serial.print(kalAngleY);Serial.print("\t");
    Serial.print(temp);Serial.print("\t");
    Serial.print("\n");
     int y = compAngleY; 
     if (y>250) y=250;
     if (y< 110) y= 110;
     int x = compAngleX;
     if (x>250) x=250;
     if (x< 110) x= 110;
    transmit_data[1] = map(x, 110, 250, 0, 255); // кинули в 1 ячейку значения по оси Х
    transmit_data[2] = map(y, 110, 250, 0, 255); // кинули во 2 ячейку значения по оси У
  }
  transmit_data[0] = !digitalRead(button_jostik);  // тут все под запоминание нажата ли была или ща нажата кнопка переключения между гиро и джойстиком      
  for (int i = 0; i < 9; i++) { 
    if (transmit_data[i] != latest_data[i]) { 
      flag = 1; 
      latest_data[i] = transmit_data[i]; 
    }
  }
  if (flag == 1) { // если флаг поднят
  radio.powerUp(); // включаем передачу
    radio.write(&transmit_data, sizeof(transmit_data)); // кидаем данные
    flag = 0; // опускаем флаг
    radio.powerDown(); // передача окончена
   }
}
boolean debvance (boolean last) // функция избавления от перебоев работы кнопки чтоб стабильно 0/1 было
{
  boolean current = digitalRead (button_jostik); 
  if (last != current) 
  {
   delay (5);  
   current = digitalRead (button_jostik);
    return current; 
  }
}
void i2cWrite(uint8_t registerAddress, uint8_t data){ // функция передачи с гиры
  Wire.beginTransmission(IMUAddress);// начинаем с адресом гиры
  Wire.write(registerAddress);// адресок
  Wire.write(data);//данные
  Wire.endTransmission(); //конец
}
uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) { // функция чтения гиры
  uint8_t data[nbytes];//данные размер
  Wire.beginTransmission(IMUAddress);//начало с адресом гиры
  Wire.write(registerAddress);// адресок
  Wire.endTransmission(false); // конец
  Wire.requestFrom(IMUAddress, nbytes); // считывание че получили
  for(uint8_t i = 0; i < nbytes; i++)// циклик
    data[i] = Wire.read();// кидаем данные 
  return data;// возвращаем данные 
}
