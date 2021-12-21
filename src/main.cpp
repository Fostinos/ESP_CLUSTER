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


// Set ADC Mode 
ADC_MODE(ADC_VCC);

// Last board delay Before beginning ESP-NOW communication (in Milliseconds) 
#define DELAY_BEGINNIG_COMM  		500


// Not to be modified : ESP_Data RECEIVER MAC ADDRESS
uint8_t addressESP_DataReceiver[ESP_ADDR] = {
	broadcastAddresses[BOARD_ID - 1][0],
	broadcastAddresses[BOARD_ID - 1][1],
	broadcastAddresses[BOARD_ID - 1][2],
	broadcastAddresses[BOARD_ID - 1][3],
	broadcastAddresses[BOARD_ID - 1][4],
	broadcastAddresses[BOARD_ID - 1][5]
};


// Some global variables 

// Synchronization
uint8_t sync;

unsigned long wakeUpTime = 0;
unsigned long currentTime = 0;
unsigned long sleepTime = 0;
unsigned long activityTime = 0;
unsigned long lastTime = 0;			// used only in toggleLED function
unsigned long syncTime = 0;			// time elapsed in the re-synchronization loop (= 0 for last Board)
uint8_t channel = 1;

void setup() {
	// Save the time firstly
	wakeUpTime = micros();
	currentTime = wakeUpTime;
	lastTime = currentTime;

	// Init LED 
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW); // Turn on LED

	// Init Serial Monitor
	Serial.begin(115200);

	// Init Wi-Fi
	initWiFi();

	// Init ESP-NOW
	initESP_NOW();

	// Load system informations from RTC Memory
	initSystemInfo();

	Serial.println("Activity Time : " + String(activityTime));
	Serial.println("Sleep Time : " + String(sleepTime));

	// Register peers

	esp_now_add_peer(addressESP_DataReceiver, ESP_NOW_ROLE_COMBO, channel, NULL, 0);

	if(BOARD_ID != (ESP_TOTAL - 1))
	{
		// Not to be modified : ESP_Command RECEIVER MAC ADDRESS
		uint8_t addressESP_CommandReceiver[ESP_ADDR] = {
			broadcastAddresses[BOARD_ID + 1][0],
			broadcastAddresses[BOARD_ID + 1][1],
			broadcastAddresses[BOARD_ID + 1][2],
			broadcastAddresses[BOARD_ID + 1][3],
			broadcastAddresses[BOARD_ID + 1][4],
			broadcastAddresses[BOARD_ID + 1][5]
		};
		esp_now_add_peer(addressESP_CommandReceiver, ESP_NOW_ROLE_COMBO, channel, NULL, 0);

		// Waiting loop (data reception break this loop)
		Serial.print("Synchronization.......");
		syncTime = micros();
		while(sync != SYNC_ACK)
		{
			// Toggle LED every 200ms
			toggleLED( micros() );
			delay(10);		// wait 10ms
		} 	
		syncTime = micros() - syncTime;
		Serial.println("Sync Time : " + String(syncTime));
		Serial.println("\nSynchronized");

	}else  // BOARD_ID == (ESP_TOTAL - 1)
	{
		// That is the last Board (this board begins ESP_NOW communication)
		delay(DELAY_BEGINNIG_COMM); // wait 0.5 second before beginning ESP_NOW communication 
		beginDataSending(BOARD_ID);
	}
	
}
 
void loop() {
	currentTime = micros() - syncTime;				
	if ( (currentTime - wakeUpTime) < activityTime )
	{
		// Activity time
		if(sync != SYNC_ACK)
		{
			// Toggle LED every 200ms to indicate De-synchronization
			toggleLED(currentTime);

		}else
		{
			// Turn on LED to indicate Synchronization
			// Activity Time Over
			if(digitalRead(LED_BUILTIN) == HIGH)  // if LED off
			{
				digitalWrite(LED_BUILTIN, LOW); // Turn on LED 
			}
		}
	}else
	{
		// Activity Time Over

		if(digitalRead(LED_BUILTIN) == LOW)  // if LED on
		{
			digitalWrite(LED_BUILTIN, HIGH); // Turn off LED 
		}
		// Going to Sleep Mode
		Serial.println("\nActivity Time Over");
		if( sync == SYNC_ACK )
		{
			// Going to Sleep Mode with Synchronization
			Serial.println("Going to Sleep Mode with Synchronization");
			// ((currentTime - wakeUpTime) - activityTime) = additional time on activity time
			ESP.deepSleep(sleepTime - ((currentTime - wakeUpTime) - activityTime));
		}else
		{
			// Going to Sleep Mode with De-synchronization
			Serial.println("Going to Sleep Mode with De-Synchronization");
			// TODO
			ESP.deepSleep(SLEEP_TIME * s_TO_uS_FACTOR);
		}
	}
}


// IMPLEMENTATION OF MAIN.H FUNCTION PROTOTYPES



/**
 * @fn 					- initWiFi
 * 
 * @brief 				- This function initializes WiFi network
 * 
 * @return 				- none 
 */
void initWiFi(void) 
{
	WiFi.mode(WIFI_STA);

	wifi_promiscuous_enable(1);
	wifi_set_channel(channel);
	wifi_promiscuous_enable(0);

	WiFi.printDiag(Serial);
}

/**
 * @fn 					- initESP_NOW
 * 
 * @brief 				- This function initializes ESP_NOW network
 * 
 * @return 				- none 
 */
void initESP_NOW(void) 
{
	if (esp_now_init() != 0) {
		Serial.println("\nError initializing ESP-NOW");
		return;
	} 

	// Set ESP-NOW role
	esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

	// Once ESPNow is successfully init, register CallBack functions
	esp_now_register_send_cb(OnDataSent);
	esp_now_register_recv_cb(OnDataRecv);
}

/**
 * @fn 					- initSystemInfo
 * 
 * @brief 				- This function loads system informations from RTC Memory
 * 
 * @return 				- none 
 */
void initSystemInfo(void) 
{
	Serial.println("\nLoading Sync Byte");
	Serial.print("Reading....... ");
	if(ESP.rtcUserMemoryRead(INFO_SYNC_OFFSET , (uint32_t*)&sync, INFO_SYNC_SIZE))
	{
		Serial.println("Succeed");

	}else
	{
		Serial.println("Failed");
	}

	// Loading Times from RTC Memory
	Serial.println("\nLoading Times");
	Serial.print("Reading....... ");
	if(ESP.rtcUserMemoryRead(INFO_SLEEP_OFFSET , (uint32_t*)&sleepTime, INFO_SLEEP_SIZE))
	{
		Serial.println("Succeed");
		if(sleepTime <= 0 || sleepTime >= 3600) 
		{
			sleepTime = SLEEP_TIME * s_TO_uS_FACTOR;
		}else // sleepTime is less than 1 hour
		{
			sleepTime = sleepTime * s_TO_uS_FACTOR;
		}
	}else
	{
		Serial.println("Failed");
	}

	if(ESP.rtcUserMemoryRead(INFO_ACTIVITY_OFFSET , (uint32_t*)&activityTime, INFO_ACTIVITY_SIZE))
	{
		Serial.println("Succeed");
		if(activityTime <= 0 || activityTime >= 10)
		{
			activityTime = ACTIVITY_TIME * s_TO_uS_FACTOR;
		}else // activityTime is less than 10 seconds
		{
			activityTime =  activityTime * s_TO_uS_FACTOR;
		}
	}else
	{
		Serial.println("Failed");
	}
}

/**
 * @fn					- OnDataSent 
 * 
 * @brief				- Callback Function when data is sent
 *
 * @param[in]			- Receiver MAC Address
 * 
 * @param[in]			- Sending Status
 *
 * @return				- none
 * 
 * @note				- Callback Function 
 */
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
	Serial.print("\r\nLast Packet Send Status: ");
	if (sendStatus == 0){
		Serial.println("Delivery success");
		// Synchronization
		// Storing Sync byte to RTC Memory
		Serial.println("\nSync byte Configuration");
		Serial.print("Writing....... ");
		sync = SYNC_ACK;
		if(ESP.rtcUserMemoryWrite(INFO_SYNC_OFFSET, (uint32_t*)&sync, INFO_SYNC_SIZE)){
			Serial.println("Succeed");
		}else{
			Serial.println("Failed");
		}
	}
	else{
		Serial.println("Delivery fail");
		// Desynchronization
		// Storing De-Sync byte to RTC Memory
		Serial.println("\nSync byte Configuration");
		Serial.print("Writing....... ");
		sync = SYNC_NACK;
		if(ESP.rtcUserMemoryWrite(INFO_SYNC_OFFSET, (uint32_t*)&sync, INFO_SYNC_SIZE)){
			Serial.println("Succeed");
		}else{
			Serial.println("Failed");
		}
	}
}


/**
 * @fn					- OnDataRecv 
 * 
 * @brief				- Callback Function when data is received
 *
 * @param[in]			- Sender MAC Address
 * 
 * @param[in]			- Pointer to Incoming Data
 * 
 * @param[in]			- Incoming Data Length
 *
 * @return				- none
 * 
 * @note				- Callback Function 
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
				Serial.println("\nDeep Sleep Time Configuration");
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
				Serial.println("\nActivity Time Configuration");
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
				uint8_t addressESP_CommandReceiver[ESP_ADDR] = {
					broadcastAddresses[BOARD_ID + 1][0],
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

		digitalWrite(LED_BUILTIN, LOW);  // Turn on LED 

		// Check Synchronization
		if(sync != SYNC_ACK)
		{
			// Synchronize
			sync = SYNC_ACK;
			// Storing Sync Byte to RTC Memory
			Serial.println("\nStoring Sync Byte");
			Serial.print("Writing....... ");
			if(ESP.rtcUserMemoryWrite(INFO_SYNC_OFFSET , (uint32_t*)&sync, INFO_SYNC_SIZE)){
				Serial.println("Succeed");
			}else{
				Serial.println("Failed");
			}
		}

		// Get current ESP_Data from sensors
		ESP_Data data = getESPData();

		Serial.println("\n******* Current ESP_Data *******\n");
		printESPData(data);

		// All ESP_Data Array (its length = Current ESP_Data length + IncomingData length)
		uint8_t allData[len + sizeof(ESP_Data)];

		// Copy Current ESP_Data to All ESP_Data Array
		memcpy(&allData, &data, sizeof(ESP_Data));

		// Copy IncomingData to All ESP_Data Array
		memcpy(&allData[sizeof(ESP_Data)], incomingData, len);

		// Storing Current All ESP_Data to RTC Memory
		Serial.println("\nStoring All ESP_Data");
		Serial.print("Writing....... ");
		if(ESP.rtcUserMemoryWrite(RTC_DATA_OFFSET, (uint32_t*)&allData, sizeof(allData))){
			Serial.println("Succeed");

			// Print All ESP_Data
			printAllESPData();

			// Send All ESP_Data
			esp_now_send(addressESP_DataReceiver, allData, sizeof(allData));

		}else{
			Serial.println("Failed");
			// TODO

			return;
		}

		

	}

}




/**
 * @fn					- beginDataSending 
 * 
 * @brief				- This function sends the first ESP_Data
 *
 * @param[in] 			- ESP Board ID (corresponding to Last Board ID)
 *
 * @return				- none
 * 
 * @note				- none
 */
void beginDataSending(uint8_t board_ID)
{
	// Get current ESP_Data from sensors
	ESP_Data data = getESPData();
	
	// Storing Current ESP_Data to RTC Memory
	Serial.println("\nStoring Current ESP_Data");
	Serial.print("Writing....... ");
	if(ESP.rtcUserMemoryWrite(RTC_DATA_OFFSET, (uint32_t*)&data, sizeof(data)))
	{
		Serial.println("Succeed");

		// Display First ESP_Data
		printESPData(data);

		// Send ESP_Data
		esp_now_send(addressESP_DataReceiver, (uint8_t*)&data, sizeof(data));

		// END 
		return;
	}else
	{
		Serial.println("Failed");
		// TODO

		return;
	}
}


/**
 * @fn					- printAllESPData 
 * 
 * @brief				- This function prints all ESP_Data of RTC Memory
 *
 * @return				- none
 * 
 * @note 				- none
 */
void printAllESPData()
{
	uint8_t length = (ESP_TOTAL - BOARD_ID) * sizeof(ESP_Data);

	// Storing Current ESP_Data to RTC Memory
	uint8_t allData[length];
	Serial.println("\nLoading All ESP_Data");
	Serial.print("Reading....... ");
	if(ESP.rtcUserMemoryRead(RTC_DATA_OFFSET, (uint32_t*)&allData, length)){
		Serial.println("Succeed");
		
		// Print All ESP_Data
		ESP_Data data;
		Serial.println("\n\n*********** ESP_Data Informations **********\n");
		for(uint8_t i=0; i < (ESP_TOTAL - BOARD_ID); i++)
		{
			memcpy(&data, &allData[i * sizeof(ESP_Data)], sizeof(ESP_Data));
			printESPData(data);
		}
	}else{
		Serial.println("Failed");
		// TODO

		return;
	}

}


/**
 * @fn					- printESPData 
 * 
 * @brief				- This function prints ESP_Data on Serial Monitor
 *
 * @param[in]			- ESP_Data to be printed
 * 
 * @return				- none
 * 
 * @note				- none
 */
void printESPData(ESP_Data data)
{
	Serial.println();
	Serial.println("ESP BOARD ID    : " + String(data.board_ID));
	Serial.println("ESP Battery     : " + String(data.battery) + " %");
	Serial.println("ESP Temperature : " + String(data.temperature) + " °C");
	Serial.println("ESP Humidity    : " + String(data.humidity) + " %");
	Serial.println("ESP Pressure    : " + String(data.pressure) + " mbar");
	Serial.println("ESP Luminosity  : " + String(data.luminosity) + " lux");
	Serial.println();
}


/**
 * @fn					- getESPData 
 * 
 * @brief				- This function gets ESP_Data of the current ESP Board
 * 
 * @return				- Currrent ESP_Data
 * 
 * @note				- none
 */
ESP_Data getESPData(void)
{
	ESP_Data data;
	data.board_ID = BOARD_ID;
	uint16_t vcc_value = ESP.getVcc(); 						// Get Vcc value (in MilliVolts)
	float vcc = (float)vcc_value / VCC_DIVIDER;				// Vcc Voltage (in Volts)
	data.battery = getBatteryPercentage(vcc);				// Get Percentage 
	data.temperature = random(0, 101); 						// [0; 100]     (unit : °C)
	data.humidity = random(0, 101); 						// [0%; 100%]     
	data.pressure = random(1000, 1051); 					// [1000; 1050] (unit : mbar)
	data.luminosity = random(100, 10000); 					// [100; 10000] (unit : lux)
	return data;
}

/**
 * @fn					- toggleLED 
 * 
 * @brief				- This function toogles LED every 200ms
 * 
 * @param[in] 			- Current time
 * 
 * @return				- none
 * 
 * @note				- none
 */
void toggleLED(unsigned long currentTime)
{
	if( (currentTime - lastTime) > (200 * ms_TO_uS_FACTOR) )
	{
		if(digitalRead(LED_BUILTIN) == LOW)  // if LED on
		{
			digitalWrite(LED_BUILTIN, HIGH); // Turn off LED
		}else								 // else LED off
		{
			digitalWrite(LED_BUILTIN, LOW); // Turn on LED
		}
		lastTime = currentTime;
	}
}



