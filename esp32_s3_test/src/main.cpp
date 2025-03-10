#include <Arduino.h>
#include <Wire.h>
#include "WiFi.h"
#include "esp_timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Clamping
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define CLAMP(x, lower, upper) (MIN((upper), MAX((x), (lower))))

// Replace with your desired AP credentials
const char *AP_SSID = "MyESP32AP";
const char *AP_PASSWORD = "12345678";

// The TCP server will listen on port 3333 (arbitrary choice)
WiFiServer server(3333);

WiFiClient client;

hw_timer_t *My_timer = NULL;

#define backSensor_1 4
#define backSensor_2 5
#define backSensor_3 6
#define backSensor_4 7
#define backSensor_5 9

#define BT1 14
#define BT2 21
#define BT3 42
#define DIP1 1
#define DIP2 2

#define I2C_SDA 48
#define I2C_SCL 47

#define LEFT_ADR 0x09
#define RIGHT_ADR 0x0A

#define MOTR_1 17
#define MOTR_2 18

#define MOTL_1 40
#define MOTL_2 41

#define timer_interupt_prescaler (80)
#define timer_interupt_frequency (1000)

#define Kp 0.4f
#define Ki 0.0f
#define Kd 0.0f

#define Kp2 1.5f
#define Ki2 0.001f
#define Kd2 0.0f

int32_t speedy = 0;

bool request;
uint8_t buf[20] = {0x00};
uint16_t sens_vals[10] = {0};

struct encoderData
{
  int32_t ENC_2_A = 10;
  int32_t ENC_2_B = 8;
  int32_t ENC_1_A = 38;
  int32_t ENC_1_B = 39;
  volatile int32_t ENC_1_tick = 0;
  volatile int32_t ENC_2_tick = 0;
};

typedef enum
{
  STATE_IDLE,
  STATE_RUNNING,
  STATE_ERROR,
  STATE_FINISHED,
  STATE_CALIBRATING,
  STATE_TESTING_PWM,
  STATE_STOP
} system_state_t;

// FSM
system_state_t current_state = STATE_IDLE;

struct encoderData encoders_struct;

void IRAM_ATTR button1_isr()
{
  USBSerial.printf("Button 1 pressed.\n");
}
void IRAM_ATTR button2_isr()
{
  USBSerial.printf("Button 2 pressed.\n");
}
void IRAM_ATTR button3_isr()
{
  USBSerial.printf("Button 3 pressed.\n");
}
void IRAM_ATTR dip1_isr()
{
  int dipRead1 = digitalRead(DIP1);
  int dipRead2 = digitalRead(DIP2);
  USBSerial.printf("DIP switch configuration: (%d, %d).\n", dipRead1, dipRead2);
}

void IRAM_ATTR dip2_isr()
{
  if (current_state == STATE_FINISHED)
  {
    current_state = STATE_RUNNING;
  }
}

void IRAM_ATTR encoder1_isr()
{
  if (digitalRead(encoders_struct.ENC_1_A) == digitalRead(encoders_struct.ENC_1_B))
  {
    encoders_struct.ENC_1_tick++; // Forward
  }
  else
  {
    encoders_struct.ENC_1_tick--; // Reverse
  }
}

void parseString(String &input, String &stringPart, int &intPart)
{
  int delimiterIndex = input.indexOf(':'); // Find the position of ":"

  if (delimiterIndex != -1)
  {
    // Extract string part
    stringPart = input.substring(0, delimiterIndex);

    // Extract integer part
    String intString = input.substring(delimiterIndex + 1);
    intPart = intString.toInt(); // Convert to integer
  }
  else
  {
    Serial.println("Error: Delimiter not found");
    stringPart = "";
    intPart = 0;
  }
}

void setupAccessPoint()
{
  // Configure ESP32 as AP
  USBSerial.println("Setting up access point");
  WiFi.mode(WIFI_AP);
  // Start AP
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  // Start the TCP server
  USBSerial.println("Starting TCP server");
  server.begin();

  // For debugging
  USBSerial.print("Access Point \"");
  USBSerial.print(AP_SSID);
  USBSerial.println("\" started");
  USBSerial.print("IP address: ");
  USBSerial.println(WiFi.softAPIP());
}

void sendData()
{
  if (!client || !client.connected())
  {
    // No connected client, skip
    Serial.println("No connected client, skipping");
    return;
  }
  // Construct message
  String message = "message:";
  message += encoders_struct.ENC_1_tick;
  // Send message over TCP
  client.print(message);
}

void receiveMessage()
{
  if (!client || !client.connected())
  {
    USBSerial.println("No connected client, skipping");
  }
  else
  {
    String incoming = "";
    // Read all available bytes
    while (client.available())
    {
      char c = client.read();
      incoming += c;
    }
    if (incoming.length() > 0)
    {
      USBSerial.print("Received from client: ");
      USBSerial.println(incoming);

      String stringPart = ""; // Buffer to store the string part
      int intPart = 0;        // Variable to store the integer part
      parseString(incoming, stringPart, intPart);
      sendData();
      speedy = intPart;
    }
  }
}

void readBackSens()
{
  int sens_1 = analogRead(backSensor_1);
  int sens_2 = analogRead(backSensor_2);
  int sens_3 = analogRead(backSensor_3);
  int sens_4 = analogRead(backSensor_4);
  int sens_5 = analogRead(backSensor_5);
  USBSerial.printf("%d %d %d %d %d\n", sens_1, sens_2, sens_3, sens_4, sens_5);
}

uint16_t readSens()
{
  uint32_t num = 0;
  uint32_t denom = 0;

  Wire.beginTransmission(LEFT_ADR);
  Wire.write(0);
  Wire.endTransmission(false);
  Wire.requestFrom(LEFT_ADR, 10, 1);

  for (size_t i = 0; i < 10; i++)
  {
    buf[i] = Wire.read();
  }

  Wire.beginTransmission(RIGHT_ADR);
  Wire.write(0);
  Wire.endTransmission(false);
  Wire.requestFrom(RIGHT_ADR, 10, 1);

  for (size_t i = 0; i < 10; i++)
  {
    buf[i + 10] = Wire.read();
  }

  for (uint32_t i = 0; i < 10; i++)
  {
    uint16_t val = (uint16_t)(buf[2 * i]) | (((uint16_t)(buf[2 * i + 1]) << 8) & 0xFF00);
    num += val * i * 1000;
    denom += val;

    // USBSerial.printf(" %d ", val);
  }
  // USBSerial.println();
  return (uint32_t)(num / denom);
}

int32_t speedRefCalc(uint32_t setpoint, uint32_t line)
{
  static float prev_err = 0;

  float err = (float)setpoint - (float)line;
  float P = Kp * err;
  float D = Kd * (err - prev_err);
  int32_t out = (int32_t)(P + D);
  prev_err = err;

  return CLAMP(out, -500, 500);
}

int32_t PwmCalc(int32_t speedRef, int32_t speed)
{ 

  // Remove this and add struct PID struct later to work for different instances
  static float prev_err = 0;

  float err = (float) speedRef - (float) speed;
  float P = Kp2 * err;
  float D = Kd2 * (err - prev_err);
  int32_t out = (int32_t)(P + D);
  prev_err = err;

  return CLAMP(out, -2000, 2000);
}

void setupEncoders()
{
  delay(3000);
  USBSerial.printf("Setting up encoder pins, Encoder 1 A: %d, Encoder 1 B: %d.", encoders_struct.ENC_1_A, encoders_struct.ENC_1_B);
  pinMode(encoders_struct.ENC_1_A, INPUT);
  pinMode(encoders_struct.ENC_1_B, INPUT);

  USBSerial.printf("Attaching ISR.");
  attachInterrupt(encoders_struct.ENC_1_A, encoder1_isr, CHANGE);
  // attachInterrupt(encoders_struct.ENC_1_B, encoder1_isr, CHANGE);

  USBSerial.printf("Encoder setup done.");

  // pinMode(encoderData.ENC_1_A, INPUT_PULLUP);
  // pinMode(encoderData.ENC_1_B, INPUT_PULLUP);
  // pinMode(encoderData.ENC_2_A, INPUT_PULLUP);
  // pinMode(encoderData.ENC_2_B, INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(encoderData.ENC_1_A), encoder1_isr, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoderData.ENC_1_B), encoder1_isr, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoderData.ENC_2_A), encoder2_isr, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoderData.ENC_2_B), encoder2_isr, CHANGE);
}

inline void setSpeed(int16_t left, int16_t right)
{
  if (left > 0)
  {
    analogWrite(MOTL_1, 0);
    analogWrite(MOTL_2, left);
  }
  else
  {
    analogWrite(MOTL_1, -left);
    analogWrite(MOTL_2, 0);
  }

  if (right > 0)
  {
    analogWrite(MOTR_1, 0);
    analogWrite(MOTR_2, right);
  }
  else
  {
    analogWrite(MOTR_1, -right);
    analogWrite(MOTR_2, 0);
  }
}

void run()
{
  // receiveMessage();
  uint32_t line = readSens();
  int16_t ref = speedRefCalc(4500, line);
  int16_t PWM_left = PwmCalc(ref, 0); // Put encoder speed in second argument

  USBSerial.printf("line: %u ", line);
  USBSerial.printf("ref: %d ", ref);
  USBSerial.printf("pwm: %d \n\r", PWM_left);
  

  //setSpeed(300-ref, 300+ref);

  // analogWrite(MOTR_1, 0);
  // analogWrite(MOTR_2, 400);

  // delay(2000);

  // analogWrite(MOTR_1, 400);
  // analogWrite(MOTR_2, 0);

  // delay(2000);

  // analogWrite(MOTR_1, 0);
  // analogWrite(MOTR_2, 0);

  // delay(2000);

  current_state = STATE_FINISHED;
  // readBackSens();
}

void IRAM_ATTR onTimer()
{
  if (current_state == STATE_FINISHED)
  {
    current_state = STATE_RUNNING;
  }
  else if (current_state == STATE_IDLE)
  {
    receiveMessage();
  }
  else if (current_state == STATE_CALIBRATING)
  {
    // Do nothing
  }
  else if (current_state == STATE_TESTING_PWM)
  {
    // Do nothing
  }
  else
  {
    Serial.println("The running function is not yet completed, go to error state");
    current_state = STATE_ERROR;
  }
}

void setup()
{
  // put your setup code here, to run once:
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  USBSerial.begin(115200);
  pinMode(BT1, INPUT);
  delay(1000);

  pinMode(MOTL_1, OUTPUT);
  pinMode(MOTL_2, OUTPUT);

  delay(1000);
  setupEncoders();

  analogWriteResolution(11);
  analogWriteFrequency(16000);

  // // Rread dip switches for preset settings
  // bool connect_wifi = true;
  // if (connect_wifi == true){
  //   // Start the access point
  //   setupAccessPoint();

  //   // Check if a new client is trying to connect
  //   while(true){
  //     if (!client || !client.connected()) {
  //       WiFiClient newClient = server.available();
  //       if (newClient) {
  //         client = newClient;
  //         USBSerial.println("New client connected!");
  //         break;
  //       }
  //     }
  //     delay(500);
  //   }
  // }

  // Setup timer for interupt to run the main fsm
  My_timer = timerBegin(0, timer_interupt_prescaler, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, timer_interupt_frequency, true);
  timerAlarmEnable(My_timer);
  current_state = STATE_FINISHED;
}

void loop()
{
  switch (current_state)
  {
  case STATE_IDLE:
    break;
  case STATE_RUNNING:
    run();
    break;
  case STATE_FINISHED:
    break;
  default:
    break;
  }
}
