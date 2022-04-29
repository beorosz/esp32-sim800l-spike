#include <Arduino.h>
#include <PubSubClient.h>

#define TINY_GSM_MODEM_SIM800       // Project is using SIM800 GSM modem
// #define DUMP_AT_COMMANDS         // See all AT commands, if wanted
// #define TINY_GSM_DEBUG Serial    // Define the serial console for debug prints, if needed
#define GSM_PIN "4170"              // set GSM PIN, if any
#define SerialGSM Serial2           // For ESP32 specific setup, see https://quadmeup.com/arduino-esp32-and-3-hardware-serial-ports/
#include <TinyGsmClient.h>

// GPRS credentials
const char            apn[]                         = "internet.vodafone.net";
const char            gprsUser[]                    = "";
const char            gprsPass[]                    = "";

// MQTT settings
const char            mwtt_broker[]                 = "broker.hivemq.com";
const int             mqtt_broker_port              = 1883;
const char            mqtt_client_id[]              = "esp32-sim800l-spike";
const char            mqtt_user[]                   = "mqtt_user";
const char            mqtt_password[]               = "mqtt_password";

// Open-close reed switch pin
const int             openClosePin                  = 19;

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger        debugger(SerialGSM, Serial);
TinyGsm               modem(debugger);
#else
 TinyGsm              modem(SerialGSM);
#endif
TinyGsmClient         gsmClient(modem);
PubSubClient          mqttClient(gsmClient);
bool                  openCloseStatusChanged;
bool                  isOpened;

/************************************/
/*      Connect to GSM network      */
/************************************/
bool connectToGsmNetwork()
{
  Serial.print("Initializing GSM modem...");
  modem.restart();
  Serial.println("done");

  if (GSM_PIN && modem.getSimStatus() != 3) 
  {
    Serial.print("Unlocking SIM card...");
    modem.simUnlock(GSM_PIN); 
    Serial.println("done");
  }

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println("failed");
    
    return false;
  }
  Serial.println("done");

  // GPRS connection parameters are usually set after network registration
  Serial.printf("Connecting to %s...", apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" failed");
    
    return false;
  }
  Serial.println(" success");

  return true;
}

/************************************/
/*      Connect to MQTT broker      */
/************************************/
int connectToMqttBroker()
{  
  mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password);
  
  return mqttClient.state();
}

/************************************/
/*     Open-close event handler     */
/************************************/
void IRAM_ATTR openCloseStatusChangedHandler()
{
  openCloseStatusChanged = true;
}


/************************************/
/*             Setup                */
/************************************/
void setup() {
  Serial.begin(115200);

  Serial.print("Setting up GSM serial port...");
  SerialGSM.begin(115200, SERIAL_8N1, 17, 16);  
  Serial.println("done");
    
  mqttClient.setServer(mwtt_broker, mqtt_broker_port);
  connectToGsmNetwork();
  pinMode(openClosePin, INPUT_PULLUP);
  attachInterrupt(openClosePin, openCloseStatusChangedHandler, CHANGE);  
}

/************************************/
/*           Main loop              */
/************************************/
void loop() {
  if (!modem.isGprsConnected())
  {
    connectToGsmNetwork();
  }

  if (openCloseStatusChanged)
  {
    isOpened = digitalRead(openClosePin) > 0 ? true : false;
    
    String networkTime = modem.getGSMDateTime(DATE_FULL);
    Serial.printf("Time: %s, IP:%s, battery:%d%%, Sensor: %s\n", 
      networkTime.c_str(), modem.getLocalIP().c_str(), modem.getBattPercent(), isOpened ? "opened" : "closed");
    
    int mqttConnectionState = connectToMqttBroker();
    if(mqttConnectionState != MQTT_CONNECTED)
    {
      Serial.printf("MQTT connection failed, state: %d\n", mqttConnectionState);
      return;
    }

    size_t bufSize = snprintf(NULL, 0, "{ \"opened\": %s }", isOpened ? "true" : "false");
    char *payload = (char *)malloc(bufSize + 1);
    sprintf(payload, "{ \"opened\": %s }", isOpened ? "true" : "false");
    
    mqttClient.publish("testtopic/openCloseStatus", payload);
    mqttClient.disconnect();

    openCloseStatusChanged = false;
    free(payload);
  }
}