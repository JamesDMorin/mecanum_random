#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "wireless.h"

// #define PRINT_CONTROLLER
#define PRINT_GRIPPER

const uint8_t * peerAddr = gripperAddr;
esp_now_peer_info_t peerInfo;

bool freshWirelessData = false;
GripperControllerMessage gripperControllerMessage;
GripperMessage gripperMessage;

void onSendData(const uint8_t *mac_addr, esp_now_send_status_t status) {
    bool success = status == ESP_NOW_SEND_SUCCESS ;
    if (success && Serial) {
    	Serial.println("Sent");
		#ifdef PRINT_CONTROLLER
			gripperControllerMessage.print();
		#endif
    } else {
      	Serial.println("Failed");
    }
}

void onRecvData(const uint8_t * mac, const uint8_t *incomingData, int len) {
	memcpy(&gripperMessage, incomingData, sizeof(gripperMessage));
	freshWirelessData = true;
	#ifdef PRINT_GRIPPER
		if (Serial) gripperMessage.print();
	#endif
}

bool sendControllerData(){
	esp_err_t result = esp_now_send(gripperAddr, (uint8_t *) &gripperControllerMessage, sizeof(gripperControllerMessage));
	return result == ESP_OK;
}