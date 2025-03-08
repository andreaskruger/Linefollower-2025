#include <Arduino.h>
#include <Wire.h>

#define BT1 14
#define BT2 21
#define BT3 42

#define I2C_SDA 48
#define I2C_SCL 47

#define LEFT_ADR 0x09
#define RIGHT_ADR 0x0A

#define MOTR_1 17
#define MOTR_2 18

#define MOTL_1 40
#define MOTL_2 41

uint8_t buf[20] = {0x00};

void readSens()
{
  Wire.beginTransmission(LEFT_ADR);
  Wire.write(0);
  Wire.endTransmission(false);
  Wire.requestFrom(LEFT_ADR, 10);

  for (size_t i = 0; i < 10; i++)
  {
    buf[i] = Wire.read();
    // USBSerial.println(buf[i]);
  }
  Wire.endTransmission(true);

  Wire.beginTransmission(RIGHT_ADR);
  Wire.write(0);
  Wire.endTransmission(false);
  Wire.requestFrom(RIGHT_ADR, 10);

  for (size_t i = 0; i < 10; i++)
  {
    buf[i + 10] = Wire.read();
    // USBSerial.println(buf[i]);
  }

  Wire.endTransmission(true);

  // Wire.endTransmission(true);

  for (size_t i = 0; i < 10; i++)
  {
    uint16_t val = (uint16_t)(buf[2 * i]) | (((uint16_t)(buf[2 * i + 1]) << 8) & 0xFF00);
    USBSerial.printf(" %d ", val);
  }
  USBSerial.println();

  // USBSerial.println("Hello World");
  // USBSerial.println(digitalRead(BT1));

  delay(200);
}

void setup()
{
  // put your setup code here, to run once:
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  USBSerial.begin(115200);
  pinMode(BT1, INPUT);
  delay(1000);

  pinMode(MOTR_1, OUTPUT);
  pinMode(MOTR_2, OUTPUT);

  analogWriteFrequency(20000);

}

void loop()
{
  // put your main code here, to run repeatedly:

  delay(1000);

  analogWrite(MOTR_1, 255);
  analogWrite(MOTR_2, 0);

  delay(2000);

  analogWrite(MOTR_1, 0);
  analogWrite(MOTR_2, 255);

  delay(2000);

  analogWrite(MOTR_1, 0);
  analogWrite(MOTR_2, 0);
}
