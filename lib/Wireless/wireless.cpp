#include <WiFi.h>
#include "util.h"
#include "wireless.h"

void ControllerMessage::print() {
    Serial.print("Controller Message\n");
    Serial.printf("millis: %d\n", millis);
    Serial.printf("joystick1:\n");  joystick1.print(1);
    Serial.printf("joystick2:\n");  joystick2.print(1);
    Serial.printf("dPad:\n");  dPad.print(1);
    Serial.printf("buttonL: %s\n", buttonL);
    Serial.printf("buttonR: %s\n", buttonR);
    Serial.printf("touchPoint:\n"); touchPoint.print(1);
} 

bool ControllerMessage::operator==(const ControllerMessage& other) {
    return joystick1 == other.joystick1 &&
           joystick2 == other.joystick2 &&
           dPad == other.dPad &&
           buttonL == other.buttonL &&
           buttonR == other.buttonR &&
           touchPoint == other.touchPoint;
}

void RobotMessage::print() {
    Serial.print("Robot Message\n");
    Serial.printf("millis: %d\n", millis);
    Serial.printf("x: %.2f\n", x);
    Serial.printf("y: %.2f\n", y);
    Serial.printf("theta: %.2f\n", theta);
} 

bool RobotMessage::operator==(const RobotMessage& other) {
    return x == other.x &&
           y == other.y &&
           theta == other.theta;
}

void GripperControllerMessage::print() {
    Serial.print("Gripper Controller Message\n");
    Serial.printf("millis: %d\n", millis);
    Serial.printf("magnet_command: %d\n", magnet_command);
    Serial.printf("gripper_command: %d\n", gripper_command);
} 

bool GripperControllerMessage::operator==(const GripperControllerMessage& other) {
    return magnet_command == other.magnet_command &&
           gripper_command == other.gripper_command;
}

void GripperMessage::print() {
    Serial.print("Gripper Message\n");
    Serial.printf("gripper_state: %d, magnet_state: %d\n", gripper_state, magnet_state);
    Serial.print("AMG Message: \n[");
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
      Serial.print(pixels[i-1]);
      Serial.print(", ");
      if( i%8 == 0 ) Serial.println();
    }
    Serial.println("]");
    Serial.println();
} 

bool GripperMessage::operator==(const GripperMessage& other) {
    return gripper_state == other.gripper_state && magnet_state == other.magnet_state ;
}

void setupWireless() {
    // ESP_NOW Setup
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
    	if (Serial) Serial.println("Error initializing ESP-NOW.");
    	return;
    }

    // Tell the microcontroller which functions to call when
    // data is sent or received
	esp_now_register_send_cb(onSendData);
	esp_now_register_recv_cb(onRecvData);
    
    // Register peer
    memcpy(peerInfo.peer_addr, peerAddr, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
  
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
    	if (Serial) Serial.println("Failed to add peer");
    	return;
    }
    // ESP-NOW Setup Complete
}
