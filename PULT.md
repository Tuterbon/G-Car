//G-Car
//Code_PULT
#include <SPI.h>          
#include <nRF24L01.h>
#include <RF24.h>
  const uint64_t pipe = 0xF0F1F2F3F4LL;
  RF24 radio(9, 10);
  byte button_jostik = 5;
  byte transmit_data[9]; 
  byte latest_data[9]; 
  boolean flag; 
#include <Wire.h>
#include "Kalman.h"
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
  int status = 0;       
    void setup() {
      Serial.begin(9600); 
        pinMode(button_jostik, INPUT_PULLUP);  
      radio.begin();
        delay(50);
        radio.setChannel(9);
        radio.setPayloadSize(8);
        radio.setRetries(1,1);
        radio.setDataRate(RF24_250KBPS);
        radio.setPALevel(RF24_PA_HIGH);
        radio.openWritingPipe(pipe);
      Wire.begin();
        i2cWrite(0x6B,0x00); // Disable sleep mode
          if(i2cRead(0x75,1)[0] != 0x68) { // Read "WHO_AM_I" register
            Serial.print(F("MPU-6050 with address 0x"));
            Serial.print(IMUAddress,HEX);
            Serial.println(F(" is not connected"));
          while(1)
          }    
            kalmanX.setAngle(180); // Set starting angle
            kalmanY.setAngle(180);
            timer = micros();
          }
          void loop() {
            currentButten = debvance (lastButten);  
              if (lastButten == LOW && currentButten == HIGH) 
                {
                status = !status; 
                }
                lastButten =  currentButten;  
                  if (status == true)
                    {
                    transmit_data[1] = map(analogRead(A0), 0, 1023, 0, 255); 
                    transmit_data[2] = map(analogRead(A1), 0, 1023, 0, 255);
                    }
                  else
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
                    transmit_data[1] = map(x, 110, 250, 0, 255); 
                    transmit_data[2] = map(y, 110, 250, 0, 255);
                    }
                      transmit_data[0] = !digitalRead(button_jostik);        
                      for (int i = 0; i < 9; i++) { 
                        if (transmit_data[i] != latest_data[i]) { 
                          flag = 1; 
                          latest_data[i] = transmit_data[i]; 
                          }
                          }
                            if (flag == 1) { 
                              radio.powerUp(); 
                              radio.write(&transmit_data, sizeof(transmit_data)); 
                              flag = 0; 
                              radio.powerDown(); 
                              }
                              }
                              boolean debvance (boolean last)
                              {
                                boolean current = digitalRead (button_jostik); 
                                if (last != current) 
                              {
                                delay (5);  
                                current = digitalRead (button_jostik);
                                return current; 
                              }
                              }
                                void i2cWrite(uint8_t registerAddress, uint8_t data){
                                  Wire.beginTransmission(IMUAddress);
                                  Wire.write(registerAddress);
                                  Wire.write(data);
                                  Wire.endTransmission(); 
                                  }
                                    uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
                                      uint8_t data[nbytes];
                                      Wire.beginTransmission(IMUAddress);
                                      Wire.write(registerAddress);
                                      Wire.endTransmission(false); 
                                      Wire.requestFrom(IMUAddress, nbytes); 
                                        for(uint8_t i = 0; i < nbytes; i++)
                                          data[i] = Wire.read();
                                          return data;
                                          }
