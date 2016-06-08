/**************************************************************************
    Souliss - Controllo Valvola Termosifone Bagno Mansarda
			  + verifica temperature di mandata e ritorno
			  all'ingresso dell'impianto

	It use static IP Addressing

    Load this code on ESP8266 board using the porting of the Arduino core
    for this platform.
        
***************************************************************************/
// Ultima cifra dell'indirizzo IP
#define IP_ADDRESS 136
#define HOSTNAME   "termobagno"

#define	VNET_RESETTIME_INSKETCH
#define VNET_RESETTIME			0x00042F7	// ((20 Min*60)*1000)/70ms = 17143 => 42F7
#define VNET_HARDRESET			ESP.reset()

// Configure the framework
#include "bconf/MCU_ESP8266.h"              // Load the code directly on the ESP8266

// **** Define the WiFi name and password ****
#include "D:\__User\Administrator\Documents\Privati\ArduinoWiFiInclude\wifi.h"
//To avoide to share my wifi credentials on git, I included them in external file
//To setup your credentials remove my include, un-comment below 3 lines and fill with
//Yours wifi credentials
//#define WIFICONF_INSKETCH
//#define WiFi_SSID               "wifi_name"
//#define WiFi_Password           "wifi_password"    

// Include framework code and libraries
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include "Souliss.h"
#include "OneWire.h"
#include <DallasTemperature.h>


// Define the network configuration according to your router settings
uint8_t ip_address[4]  = {192, 168, 1, IP_ADDRESS};
uint8_t subnet_mask[4] = {255, 255, 255, 0};
uint8_t ip_gateway[4]  = {192, 168, 1, 1};


// This identify the number of the Slot
#define T_RELE_1	0      
#define T_IN_1		1
#define T_TEMP_MAN  2
#define T_TEMP_RIT  4
     

// **** Define here the right pin for your ESP module **** 
#define	PIN_RELE_1	4

#define	PIN_IN_1	12
#define	PIN_BUTTON	0
#define PIN_LED		16
#define PIN_DALLAS_1 5
#define PIN_DALLAS_2 13

#define HDEADBAND		0.50	
#define DEADBAND		0.05			//Se la variazione è superio del 5% aggiorno
#define LDEADBAND		0.01			//Se la variazione è superio del 1% aggiorno
#define MDEADBAND		0.03
#define NODEADBAND		0				//Se la variazione è superio del 0,1% aggiorno

//Useful Variable
byte led_status = 0;
byte joined = 0;
U8 value_hold=0x068;


//Misurazione Temperature
#define TEMPERATURE_PRECISION	9	
OneWire ourWire_1(PIN_DALLAS_1);
OneWire ourWire_2(PIN_DALLAS_2);
DallasTemperature sensors_1(&ourWire_1);
DallasTemperature sensors_2(&ourWire_2);
float t_man;
float t_rit;


void setup()
{  
	delay((IP_ADDRESS - 128) * 5000);
    Initialize();

    digitalWrite(PIN_DALLAS_1, HIGH);

	//Pin Setup
	pinMode(PIN_RELE_1, OUTPUT);
	pinMode(PIN_IN_1, INPUT);
	pinMode(PIN_LED, OUTPUT);
	

    // Connect to the WiFi network with static IP
	Souliss_SetIPAddress(ip_address, subnet_mask, ip_gateway);
    
	Set_SimpleLight(T_RELE_1);
	Souliss_SetT13(memory_map, T_IN_1);
	Souliss_SetT52(memory_map, T_TEMP_MAN);
	Souliss_SetT52(memory_map, T_TEMP_RIT);

	// Init the OTA
	ArduinoOTA.setHostname(HOSTNAME);
	ArduinoOTA.begin();

	Serial.begin(115200);
    Serial.println("Node Init");
}

void loop()
{ 
    // Here we start to play
    EXECUTEFAST() {                     
        UPDATEFAST();   
        
        FAST_50ms() {   // We process the logic and relevant input and output every 50 milliseconds
			
			// Detect the button press. Short press toggle, long press reset the node
			U8 invalue = LowDigInHold(PIN_BUTTON,Souliss_T1n_ToggleCmd,value_hold,T_RELE_1);
			if(invalue==Souliss_T1n_ToggleCmd){
				Serial.println("TOGGLE");
				mInput(T_RELE_1)=Souliss_T1n_ToggleCmd;
			} else if(invalue==value_hold) {
				// reset
				Serial.println("REBOOT");
				delay(1000);
				ESP.reset();
			}
			
						//Check if joined and take control of the led
			if (joined==1) {
				if (mOutput(T_RELE_1)==1) {
					digitalWrite(PIN_LED,HIGH);
				} else {
					digitalWrite(PIN_LED,LOW);
				}
			}
        } 
		FAST_70ms() {
		  Souliss_LowDigIn2State(PIN_IN_1, Souliss_T1n_OnCmd, Souliss_T1n_OffCmd, memory_map, T_IN_1);
		  //Output Handling
		  DigOut(PIN_RELE_1, Souliss_T1n_Coil, T_RELE_1);
		}
		FAST_90ms() { 
			//Apply logic if statuses changed
			Logic_SimpleLight(T_RELE_1);
			Souliss_Logic_T13(memory_map, T_IN_1, &data_changed);

		}

		FAST_510ms() {
			//Check if joined to gateway
			check_if_joined();
		}

		FAST_1110ms() {
			Souliss_Logic_T52(memory_map, T_TEMP_MAN, LDEADBAND, &data_changed);
			Souliss_Logic_T52(memory_map, T_TEMP_RIT, LDEADBAND, &data_changed);
		}

        FAST_PeerComms();                                        
    }
	EXECUTESLOW() {
		UPDATESLOW();
		SLOW_10s() {
			DSRead();	//Routine per leggere il valore della sonda e importarlo in Souliss
		}
	}
	// Look for a new sketch to update over the air
	ArduinoOTA.handle();
} 

void DSRead() {
	//Leggo le sonde Dallas
	sensors_1.requestTemperatures();
	t_man = sensors_1.getTempCByIndex(0);

	sensors_2.requestTemperatures();
	t_rit = sensors_2.getTempCByIndex(0);
	
	Souliss_ImportAnalog(memory_map, T_TEMP_MAN, &t_man);
	Souliss_ImportAnalog(memory_map, T_TEMP_RIT, &t_rit);
	Serial.print("T_Man: ");
	Serial.println(t_man);
	Serial.print("T_Rit: ");
	Serial.println(t_rit);
}

//This routine check for peer is joined to Souliss Network
//If not blink the led every 500ms, else led is a mirror of rel� status
void check_if_joined() {
	if(JoinInProgress() && joined==0){
		joined=0;
		if (led_status==0){
			digitalWrite(PIN_LED,HIGH);
			led_status=1;
		}else{
			digitalWrite(PIN_LED,LOW);
			led_status=0;
		}
	}else{
		joined=1;
	}		
}
