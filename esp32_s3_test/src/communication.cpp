/*


*/
#include "communication.h"
#include "defines.h"
#include "robot_states.h"
#include "controllers.h"

String construct_message(struct robotStates_t* robotStates){
  String message = "";
  message += "description:data,";
  message += "X-position:";
  message += robotStates->x_pos;
  message += ",Y-posistion:";
  message += robotStates->y_pos;
  message += ",Velocity:";
  message += robotStates->velocity;
  message += ",Left control signal:";
  message += robotStates->left_controlSignal;
  message += ",Right control signal:";
  message += robotStates->right_controlSignal;
  message += ",Front line sensor:";
  message += robotStates->lineSensor_value_front;
  message += ",Back line sensor:";
  message += robotStates->lineSensor_value_back;
  message += "end\n";
  return message;
}

void parseString(String input, struct messageFields_t* messageParts) {
  messageParts->stringPart = "";
  messageParts->intStringPart ="";
  messageParts->floatPart_1 = 0.0;
  messageParts->floatPart_2 = 0.0;
  messageParts->floatPart_3 = 0.0;

  int colonIndex = input.indexOf(":");
  
  if (colonIndex == -1) { // No colon found
    messageParts->stringPart = input;
      return;
  }
  
  // Split at ':'
  messageParts->stringPart = input.substring(0, colonIndex);
  messageParts->intStringPart = input.substring(colonIndex + 1);
  
  int firstDash = messageParts->intStringPart.indexOf('-');
  int secondDash = messageParts->intStringPart.indexOf('-', firstDash + 1);
  
  // If there is 1 or two dashes, split there
  if (firstDash == -1) { // No dashes found
    messageParts->floatPart_1 = messageParts->intStringPart.toFloat();
  } else if (secondDash != -1) { // Two dashes found
    messageParts->floatPart_1 = messageParts->intStringPart.substring(0, firstDash).toFloat();
    messageParts->floatPart_2 = messageParts->intStringPart.substring(firstDash + 1, secondDash).toFloat();
    messageParts->floatPart_3 = messageParts->intStringPart.substring(secondDash + 1).toFloat();
  }
}

//void sendPIDparams(WiFiClient client, float Kp, float Ki, float Kd){
void sendPIDparams(WiFiClient client, struct PID_t* pid, struct PID_t* pid2){
  if (!client || !client.connected()){
    // No connected client, skip
    USBSerial.printf("No connected client, skipping\n");
    return;
  }
  USBSerial.printf("Sending PID parameters to the client.\n");
  String message = "";
  message += "description:parameters,";
  message += "parameters:";
  message += "PWM PID ";
  message += pid->Kp;
  message += "-";
  message += pid->Ki;
  message += "-";
  message += pid->Kd;
  message += " - Speed PID ";
  message += pid2->Kp;
  message += "-";
  message += pid2->Ki;
  message += "-";
  message += pid2->Kd;
  message += "end\n";
  client.print(message);
}

int32_t receiveMessage(WiFiClient client, struct messageFields_t* messageParts){
  int32_t returnCode = 0; 

  if (!client || !client.connected()){
    returnCode = -1;
  }
  else{
    String incoming = "";
    // Read all available bytes
    while (client.available()){
      char c = client.read();
      incoming += c;
    }
    if (incoming.length() > 0){
      parseString(incoming, messageParts);
      if(messageParts->stringPart == "start"){
        returnCode = 1;
      }
      else if(messageParts->stringPart == "stop"){
        returnCode = 2;
      }
      else if(messageParts->stringPart == "setPWMPID"){
        returnCode = 3;
      }
      else if(messageParts->stringPart == "setSpeedPID"){
        returnCode = 4;
      }
      else if(messageParts->stringPart == "getPID"){
        returnCode = 5;
      }
      else if(messageParts->stringPart == "setPWM"){
        returnCode = 7;
      }
      else if(messageParts->stringPart == "calibrate"){
        returnCode = 8;
      }
      else if(messageParts->stringPart == "readData"){
        returnCode = 9;
      }
      else if(messageParts->stringPart == "disconnect"){
        returnCode = 10;
      }
      else if(messageParts->stringPart == "setLeftPWMPID"){
        returnCode = 11;
      }
      else if(messageParts->stringPart == "setRightPWMPID"){
        returnCode = 12;
      }
      else if(messageParts->stringPart == "setBaseSpeed"){
        returnCode = 13;
      }
      else{
        returnCode = 0;
      }
    }
  }
  return returnCode;
}

void sendMessage(struct robotStates_t* robotStates, WiFiClient client){
  if (!client || !client.connected()){
    // No connected client, skip
    USBSerial.printf("No connected client, skipping\n");
    return;
  }
  // Construct message from robotStates struct
  String message = construct_message(robotStates);
  // Send message over TCP
  client.print(message);
}

void send_finishLine_found(WiFiClient client){
  String message = "description:finishLineFound\n";
  client.print(message);
}

void sendState(WiFiClient client, int32_t state){
  const char* stateNames[] = {
    "Init",
    "Idle",
    "Running",
    "Error",
    "Waiting",
    "Calibrating",
    "Testing PWM",
    "Stop",
    "Disconnected",
    "Line lost"
  };

  String message = "description:stateUpdate,";
  message += "state:";
  message += stateNames[state];
  message += "end\n";
  client.print(message);
}