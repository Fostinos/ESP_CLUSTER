/**
 * @file main.cpp
 * @author ESP_CLUSTER
 * @brief 
 * @version 1.0
 * @date 2021-11-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>


// User Includes
#include "main.h"
#include "battery.h"
#include "rtc.h"

// Set your Board ID (ESP #1 = BOARD_ID_1, ESP #2 = BOARD_ID_2, etc)
//TODO
#define     BOARD_ID               ESP_BOARD_1


// Set ADC Mode 
ADC_MODE(ADC_VCC);


// Not to be modified : ESP_Data RECEIVER MAC ADDRESS
uint8_t addressESP_DataReceiver[ESP_ADDR] = {   broadcastAddresses[BOARD_ID - 1][0],
												broadcastAddresses[BOARD_ID - 1][1],
												broadcastAddresses[BOARD_ID - 1][2],
												broadcastAddresses[BOARD_ID - 1][3],
												broadcastAddresses[BOARD_ID - 1][4],
												broadcastAddresses[BOARD_ID - 1][5]
											};


// ESP variables to be sent (To be stored in RTC Memory before sleeping)

// Synchronization
uint8_t sync;
uint8_t count = 0;

unsigned long lastTime = 0;
unsigned long timerDelay = 10000;



void setup() {
	// Init Serial Monitor
	Serial.begin(115200);

	// Set device as a Wi-Fi Station
	WiFi.mode(WIFI_STA);
	WiFi.disconnect();

	// Deep Sleep
	// Serial.println("\nWaking up...");


	// Init ESP-NOW
	if (esp_now_init() != 0) {
		Serial.println("Error initializing ESP-NOW");
		return;
	} 
	// Set ESP-NOW role
	esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

	// Once ESPNow is successfully init, register CallBack functions
	esp_now_register_send_cb(OnDataSent);
	esp_now_register_recv_cb(OnDataRecv);

	// Register peers
	esp_now_add_peer(addressESP_DataReceiver, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
	if(BOARD_ID != (ESP_TOTAL - 1))
	{
		// Not to be modified : ESP_Command RECEIVER MAC ADDRESS
		uint8_t addressESP_CommandReceiver[ESP_ADDR] = {  	broadcastAddresses[BOARD_ID + 1][0],
															broadcastAddresses[BOARD_ID + 1][1],
															broadcastAddresses[BOARD_ID + 1][2],
															broadcastAddresses[BOARD_ID + 1][3],
															broadcastAddresses[BOARD_ID + 1][4],
															broadcastAddresses[BOARD_ID + 1][5]
														};
		esp_now_add_peer(addressESP_CommandReceiver, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

		// WAIT For First Incomming Data


		// Loading from RTC Memory
		Serial.println("Loading Sync Byte");
		Serial.print("Reading....... ");
		if(ESP.rtcUserMemoryRead(INFO_SYNC_OFFSET , (uint32_t*)&sync, INFO_SYNC_SIZE)){
			Serial.println("Succeed");

			// Waiting loop
			Serial.print("Synchronization....... ");
			while(sync != SYNC_ACK)
			{
				Serial.print(".");
				delay(500); // wait 0.5 seconds
			}
			Serial.println("\nSynchronized");

		}else{
			Serial.println("Failed");
			// TODO
		}

	}else  
	{
		// That is the last Board (begin ESP_NOW communication)
		beginDataSending(BOARD_ID);
	}
	
}
 
void loop() {

}


// IMPLEMENTATION OF CALLBACK FUNCTIONS

/**
 * @fn            		- OnDataSent 
 * 
 * @brief			  	- Callback Function when data is sent
 *
 * @param[in]			- Receiver MAC Address
 * 
 * @param[in]			- Sending Status
 *
 * @return				- none
 * 
 * @note          		- Callback Function 
 */
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("\r\nLast Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");

	// TODO
	
  }
  else{
    Serial.println("Delivery fail");

	//TODO

  }
}


/**
 * @fn          		- OnDataRecv 
 * 
 * @brief			  	- Callback Function when data is received
 *
 * @param[in]			- Sender MAC Address
 * 
 * @param[in]			- Pointer to Incoming Data
 * 
 * @param[in]			- Incoming Data Length
 *
 * @return				- none
 * 
 * @note          		- Callback Function 
 */
void OnDataRecv(uint8_t * mac_addr, uint8_t *incomingData, uint8_t len) {

	// Verify Data type 
	if(len == sizeof(ESP_Command))
	{
		// Incoming Data is ESP_Command

		// Save the new packet in the ESP_Command structure
		ESP_Command cmd;
		memcpy(&cmd, incomingData, sizeof(cmd));

		if(cmd.board_ID == ESP_BOARD_0)	// The Sender must be the Main ESP
		{
			// Check Command_type
			if( cmd.command == CMD_SLEEP) 
			{
				// Command_Type = Deep Sleep Time Configuration
				
				// Storing Deep Sleep Time to RTC Memory
				Serial.println("Deep Sleep Time Configuration");
				Serial.print("Writing....... ");
				if(ESP.rtcUserMemoryWrite(INFO_SLEEP_OFFSET, &cmd.time, INFO_SLEEP_SIZE)){
					Serial.println("Succeed");
					// TODO

				}else{
					Serial.println("Failed");
					// TODO

					return;
				}
			}
			else if( cmd.command == CMD_ACTIVITY)
			{	
				// Command_Type = Activity Time Configuration 

				// Storing Activity Time to RTC Memory
				Serial.println("Activity Time Configuration");
				Serial.print("Writing....... ");
				if(ESP.rtcUserMemoryWrite(INFO_ACTIVITY_OFFSET, &cmd.time, INFO_ACTIVITY_SIZE)){
					Serial.println("Succeed");
					// TODO

				}else{
					Serial.println("Failed");
					// TODO

					return;
				}
			}

			// Send ESP_Command to the next ESP
			if(BOARD_ID != (ESP_TOTAL - 1))
			{
				// Not to be modified : ESP_Command RECEIVER MAC ADDRESS
				uint8_t addressESP_CommandReceiver[ESP_ADDR] = {  	broadcastAddresses[BOARD_ID + 1][0],
																	broadcastAddresses[BOARD_ID + 1][1],
																	broadcastAddresses[BOARD_ID + 1][2],
																	broadcastAddresses[BOARD_ID + 1][3],
																	broadcastAddresses[BOARD_ID + 1][4],
																	broadcastAddresses[BOARD_ID + 1][5]
																};
				
				esp_now_send(addressESP_CommandReceiver, (uint8_t *)&cmd, sizeof(cmd));
			}

			// END
			return;
		}
		
	}else
	{
		// Incoming Data is ESP_Data

		// Check Synchronization
		if(sync != SYNC_ACK)
		{
			// Synchronize
			sync = SYNC_ACK;
			// Storing Sync Byte to RTC Memory
			Serial.println("Storing Sync Byte");
			Serial.print("Writing....... ");
			if(ESP.rtcUserMemoryWrite(INFO_SYNC_OFFSET , (uint32_t*)&sync, INFO_SYNC_SIZE)){
				Serial.println("Succeed");
			}else{
				Serial.println("Failed");
			}
		}

		// Calculate Offset of current ESP_Data in RTC Memory
		uint8_t offset = BOARD_ID * sizeof(ESP_Data);

		// Storing the new packet to RTC Memory
		Serial.println("Storing Incomming ESP_Data");
		Serial.print("Writing....... ");
		if(ESP.rtcUserMemoryWrite(RTC_DATA_OFFSET + offset + sizeof(ESP_Data), (uint32_t*)incomingData, len)){
			Serial.println("Succeed");
			// TODO

		}else{
			Serial.println("Failed");
			// TODO

			return;
		}

		// Get current ESP_Data from sensors
		ESP_Data data;
		data.board_ID = BOARD_ID;
		uint16_t vcc_value = ESP.getVcc();			// Get Vcc value (Depends on ADC_RESOLUTION)
		float vcc = vcc_value / ADC_RESOLUTION;		// Vcc Voltage
		data.battery = getBatteryPercentage(vcc);   // Get Percentage
		data.battery = getBatteryPercentage(VCC_VOLTAGE_MAX); // 100%
		data.temperature = random(0, 101);		// [0; 100]     (unit : °C)
		data.humidity = random(0, 101);			// [0%; 100%]     
		data.pressure = random(1000, 1051);		// [1000; 1050] (unit : mbar)
		data.luminosity = random(100, 10000);	// [100; 10000] (unit : lux)

		// Storing Current ESP_Data to RTC Memory
		Serial.println("Storing Current ESP_Data");
		Serial.print("Writing....... ");
		if(ESP.rtcUserMemoryWrite(RTC_DATA_OFFSET + offset, (uint32_t*)&data, sizeof(data))){
			Serial.println("Succeed");
			// TODO

		}else{
			Serial.println("Failed");
			// TODO

			return;
		}

		uint8_t allData[len + sizeof(ESP_Data)];
		// Loading from RTC Memory
		Serial.println("Loading All ESP_Data");
		Serial.print("Writing....... ");
		if(ESP.rtcUserMemoryWrite(RTC_DATA_OFFSET + offset, (uint32_t*)&data, sizeof(allData))){
			Serial.println("Succeed");

			// Send All ESP_Data
			esp_now_send(addressESP_DataReceiver, allData, sizeof(allData));

			// END
			return;
		}else{
			Serial.println("Failed");
			// TODO

			return;
		}

	}

}




/**
 * @fn          		- beginDataSending 
 * 
 * @brief			  	- This function sends the first ESP_Data
 *
 * @param[in]			- ESP Board ID (corresponding to Last Board ID)
 *
 * @return				- none
 * 
 * @note          		- none
 */
void beginDataSending(uint8_t board_ID)
{
	if(board_ID == (ESP_TOTAL - 1))
	{
		uint8_t offset = BOARD_ID * sizeof(ESP_Data);

		// Get current ESP_Data from sensors
		ESP_Data data;
		data.board_ID = BOARD_ID;
		// uint16_t vcc_value = ESP.getVcc();			// Get Vcc value (Depends on ADC_RESOLUTION)
		// float vcc = vcc_value / ADC_RESOLUTION;		// Vcc Voltage
		// data.battery = getBatteryPercentage(vcc);   // Get Percentage
		data.battery = getBatteryPercentage(VCC_VOLTAGE_MAX); // 100%
		data.temperature = random(0, 101);		// [0; 100]     (unit : °C)
		data.humidity = random(0, 101);			// [0%; 100%]     
		data.pressure = random(1000, 1051);		// [1000; 1050] (unit : mbar)
		data.luminosity = random(100, 10000);	// [100; 10000] (unit : lux)

		
		// Storing Current ESP_Data to RTC Memory
		Serial.println("Storing Current ESP_Data");
		Serial.print("Writing....... ");
		if(ESP.rtcUserMemoryWrite(RTC_DATA_OFFSET + offset, (uint32_t*)&data, sizeof(data))){
			Serial.println("Succeed");

			// Send ESP_Data
			esp_now_send(addressESP_DataReceiver, (uint8_t*)&data, sizeof(data));

			// END 
			return;
		}else{
			Serial.println("Failed");
			// TODO

			return;
		}

	}
}

















