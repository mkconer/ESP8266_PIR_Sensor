
//********************* Adafruit MQTT Library ********************************

#include "Adafruit_MQTT.h"                                  // Adafruit MQTT library
#include "Adafruit_MQTT_Client.h"                           // Adafruit MQTT library
//*****************************************************************************

#include "ESP8266WiFi.h"                                    // ESP8266 library     
#include <SoftwareSerial.h>                                 // Software Serial Library so we can use Pins for communication with the GPS module

//*********************** PIR Motion Sensor Setup *****************************                                                        
 
int ledPin = D5;                                            // choose the pin for the LED (Pin D5, GPIO 14) I tried using pin D4 but had problems for some reason
int inputPin = D6;                                          // choose the input pin (for PIR sensor) (Pin D6, GPIO 12)
int pirState = LOW;                                         // we start, assuming no motion detected
int val = 0;                                                // variable for reading the pin status

//************************* WiFi Access Point *********************************

#define WLAN_SSID       "**********"                        // Enter Your router SSID
#define WLAN_PASS       "**********"                        // Enter Your router Password

//************************* Adafruit.io Setup *********************************

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                                // use 8883 for SSL
#define AIO_USERNAME    "************"                      // Enter Your Adafruit IO Username
#define AIO_KEY         "************"                      // Enter Your Adafruit IO Key

//************ Global State (you don't need to change this!) ******************

WiFiClient client;                                          // Create an ESP8266 WiFiClient class to connect to the MQTT server.

const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;           // Store the MQTT server, username, and password in flash memory.
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

//****************************** Feeds ***************************************

// Setup a feed called 'motion' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char motion_FEED[] PROGMEM = AIO_USERNAME "/feeds/motion";
Adafruit_MQTT_Publish motion = Adafruit_MQTT_Publish(&mqtt, motion_FEED);

//************************** Setup *******************************************
 
void setup() 
{
  Serial.begin(115200);                                   // Serial Baud Rate
  pinMode(ledPin, OUTPUT);                                // declare LED for motion sensor as output
  pinMode(inputPin, INPUT);                               // declare Motion sensor pin as input

  WiFi.mode(WIFI_STA);                                    // Setup ESP8266 as a wifi station
  WiFi.disconnect();                                      // Disconnect if needed
  delay(100);                                             // short delay 
     
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
}
//************************ Main Loop *******************************************
void loop()
{
  connectWifi();
  MQTT_connect();                                         // Connect to Adafruit IO MQTT  
  
  val = digitalRead(inputPin);                            // read input value of PIR motion sensor
  if (val == HIGH)                                        // check if the input is HIGH
  {                                                       
    digitalWrite(ledPin, HIGH);                           // Turn the LED ON
    
    if (pirState == LOW) 
      {                                                        
       Serial.println("Motion detected!");                // we have just turned on                                                        
       pirState = HIGH;                                   // We only want to print on the output change, not state
        if (! motion.publish(val))                        // Publish to Adafruit the PIR sensor value '1'
        {                                
         Serial.println(F("Failed"));                     // If it failed to publish, print Failed
        } else 
           {             
             Serial.println(F("Data Sent!"));             // If data successfully published      
           }      
      }
  } else 
  {
    digitalWrite(ledPin, LOW);                            // Turn LED OFF
    if (pirState == HIGH)                                 // we have just turned off
    {                                                                                                             
      Serial.println("Motion ended!");                    // We only want to print on the output change, not state                                      
      pirState = LOW;
      motion.publish(val);                                // Publish to Adafruit the PIR sensor value '0'      
    }
  }
    
    if(! mqtt.ping()) {mqtt.disconnect();}                // Ping Adafruit.IO to keep the MQTT connection alive
  
}
//********************** WiFi Coonect *************************************
void connectWifi()
{   
  if (WiFi.status() == WL_CONNECTED){ return; }           // If already connected to WiFi, return to loop
   WiFi.begin(WLAN_SSID, WLAN_PASS);                      // Start a WiFi connection and enter SSID and Password
              while (WiFi.status() != WL_CONNECTED) {     // While waiting on connection to start display "..."
                  delay(500);
                  Serial.print(".");                  
              } 
              Serial.println("Connected");              
            MQTT_connect();                               // Run Procedure to connect to Adafruit IO MQTT              
}

// ******************* MQTT Connect - Adafruit IO **************************
void MQTT_connect() 
{
  int8_t ret;
  if (mqtt.connected()) { return; }                       // Stop if already connected to Adafruit
  Serial.println("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {                   // Connect to Adafruit, will return 0 if connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT...");
       mqtt.disconnect();
       delay(5000);                                       // wait 5 seconds
       retries--;
       if (retries == 0) {                                // basically die and wait for WatchDogTimer to reset me                                                          
         while (1);         
       }
  }
  Serial.println("MQTT Connected!");  
}
