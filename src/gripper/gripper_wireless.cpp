#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "wireless.h"

// #define PRINT_CONTROLLER
// #define PRINT_GRIPPER

const uint8_t * peerAddr = gripperControllerAddr;
esp_now_peer_info_t peerInfo;

bool freshWirelessData = false;
GripperControllerMessage gripperControllerMessage;
GripperMessage gripperMessage;

void onSendData(const uint8_t *mac_addr, esp_now_send_status_t status) {
    bool success = status == ESP_NOW_SEND_SUCCESS ;
    if (success && Serial) {
    	Serial.println("Sent");
		#ifdef PRINT_GRIPPER
			gripperMessage.print();
		#endif
    } else {
      	Serial.println("Failed");
    }
}

void onRecvData(const uint8_t * mac, const uint8_t *incomingData, int len) {
	memcpy(&gripperControllerMessage, incomingData, sizeof(gripperControllerMessage));
	freshWirelessData = true;
	#ifdef PRINT_CONTROLLER
		if (Serial) gripperControllerMessage.print();
	#endif
}

bool sendGripperData(){
	esp_err_t result = esp_now_send(gripperControllerAddr, (uint8_t *) &gripperMessage, sizeof(gripperMessage));
	return result == ESP_OK;
}