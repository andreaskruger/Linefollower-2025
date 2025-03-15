/**
* Description:
* 
* Note:
* 
* Usage:
* 
**/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <Arduino.h>
#include "WiFi.h"

/**
 * @brief Struct for received messages where the message can be split into different parts and stored in this struct.
 * Messages are defined to have a descriptor followed by a delimiter (:) followed by digits, if multiple digits are sent they are separated by the delimiter (-).
 * 
 * @param stringPart: Descriptor of the message, (String).
 * @param intStringPart: Everhthing followed by the delimiter (:), (String).
 * @param floatPart_1: First float part of the message, (Float).
 * @param floatPart_2: Second float part of the message, (Float).
 * @param floatPart_3: Third float part of the message, (Float).
 */
struct messageFields_t{
    String stringPart = "";
    String intStringPart = "";
    float floatPart_1 = 0;
    float floatPart_2 = 0;
    float floatPart_3 = 0;
};

/**
 * @brief Construct a message from the robotStates struct. Used to send messages to the client in a structured way.
 * Returns a string with the following format: "descriptor:digit,descriptor:digit..."
 * 
 * @param robotStates: Pointer to the robotStates struct.
 * @return (String)
 */
String construct_message(struct robotStates_t* robotStates);

/**
 * @brief Check if there is a message from the client and parse it, store result into the messageFields struct.
 * A message is defined to have a descriptor followed by a delimiter (:) followed by digits, if multiple digits are sent they are separated by the delimiter (-).
 * If no delimiter is found only the descriptor is stored.
 * @note The messageFields_t struct is allways emptied at the start of the function.
 * @see parseString() to parse the message.
 * @param client: Pointer to the client.
 * @param messageParts: Pointer to the messageFields struct.
 * @return (int32_t) return code, , 0 if no message.
 * @retval -1 No client is connected
 * @retval 0 No message received
 * @retval 1 "Start" command received
 * @retval 2 "Stop" command received
 * @retval 3 "Set PWM PID" command received
 * @retval 4 "Set Speed PID" command received
 * @retval 5 "Get PID" command receive
 * @retval 7 "Set PWM" command receive
 * @retval 8 "Calibrate" command received
 */
int32_t receiveMessage(WiFiClient client, struct messageFields_t* messageParts);

/**
 * @brief Send a message to the client. Constructs a message from the robotStates struct and sends it over TCP.
 * @see construct_message() to construct the message.
 * @param robotStates: Pointer to the robotStates struct.
 * @param client: Pointer to the client.
 */
void sendMessage(struct robotStates_t* robotStates, WiFiClient client);

/**
 * @brief Parse a string into the messageFields struct.
 * @param input: String to parse.
 * @param messageParts: Pointer to the messageFields struct.
 */
void parseString(String input, struct messageFields_t* messageParts);

/**
 * @brief Send the PID parameters to the client.
 * 
 * @param client: Pointer to the client.
 * @param pid: Pointer to the PWM PID struct.
 * @param pid2: Pointer to the Speed PID struct.
 */
void sendPIDparams(WiFiClient client, struct PID_t* pid, struct PID_t* pid2);

/**
 * @brief Send a message to the client that the finish line has been found.
 * 
 * @param client: Pointer to the client.
 */
void send_finishLine_found(WiFiClient client);

/**
 * @brief Send a message to the client that contains the state that the robot is currently in.
 * 
 * @param client: Pointer to the client.
 * @param state: State that the robot is currently in.
 */
void sendState(WiFiClient client, int32_t state);

#endif

