#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP32Time.h>

#define TINY_GSM_MODEM_SIM800 // Project is using SIM800 GSM modem
// #define DUMP_AT_COMMANDS          // See all AT commands, if wanted
// #define TINY_GSM_DEBUG Serial // Define the serial console for debug prints, if needed
#define GSM_PIN "4170"    // set GSM PIN, if any
#define SerialGSM Serial2 // For ESP32 specific setup, see https://quadmeup.com/arduino-esp32-and-3-hardware-serial-ports/
#include <TinyGsmClient.h>

// GPRS credentials
const char apn[] = "internet.vodafone.net";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT settings
const char mqtt_broker[] = "broker.hivemq.com";
const int mqtt_broker_port = 1883;
const char mqtt_client_id[] = "esp32-sim800l-spike";
const char mqtt_user[] = "mqtt_user";
const char mqtt_password[] = "mqtt_password";

// Open-close reed switch pin
const int openClosePin = 19;
// GSM modem reset pin
const int modemResetPin = 18;

TaskHandle_t mqttKeepAliveTask;
TaskHandle_t setTimeTask;
TaskHandle_t heartbeatTask;
TaskHandle_t sendOpenCloseStatusChangeTask;

const long HEARTBEAT_PERIOD_IN_MS = 10000;
const TickType_t xDelay250ms = pdMS_TO_TICKS(250);

ESP32Time rtc;

struct openclose_event_payload
{
  bool isOpened;
  String datetime;
};

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialGSM, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialGSM);
#endif
TinyGsmClient gsmClient(modem);
PubSubClient mqttClient(gsmClient);

/********************************************/
/*                  Tasks                   */
/********************************************/

// Connects to MQTT broker
void connectToMqttBroker()
{
  int count = 0;

  Serial.printf("Connecting with client ID %s...", mqtt_client_id);
  while (!mqttClient.connected())
  {
    mqttClient.disconnect();
    mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password);
    vTaskDelay(xDelay250ms);
    count++;
    if (count == 5)
    {
      Serial.println(" failed");
      return;
    }
  }
  Serial.println(" success");
}

// Connects to GSM network
bool connectToGsmNetwork()
{
  if (!modem.isNetworkConnected())
  {
    if (GSM_PIN && modem.getSimStatus() != 3)
    {
      Serial.print("Unlocking SIM card...");
      modem.simUnlock(GSM_PIN);
      Serial.println("done");
    }

    Serial.print("Waiting for network...");
    if (!modem.waitForNetwork(1000, true))
    {
      Serial.println("failed");

      return false;
    }
    Serial.println("done");
  }

  // GPRS connection parameters are usually set after network registration
  Serial.printf("Connecting to %s...", apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    Serial.println(" failed");

    return false;
  }
  Serial.println(" success");

  return true;
}

// Task to keep MQTT connected (alive)
void mqttKeepAliveTaskFunction(void *pvParameters)
{
  mqttClient.setServer(mqtt_broker, mqtt_broker_port);
  mqttClient.setKeepAlive(90);

  digitalWrite(modemResetPin, LOW);
  delay(120);
  digitalWrite(modemResetPin, HIGH);

  Serial.print("Initializing GSM modem...");
  modem.restart();
  Serial.println("done");

  for (;;)
  {
    if (!modem.isGprsConnected())
    {
      bool connResult = connectToGsmNetwork();
      if (connResult == false)
      {
        continue;
      }
    }

    vTaskDelay(xDelay250ms);

    if (modem.isGprsConnected() && !mqttClient.connected())
    {
      connectToMqttBroker();
    }
  }
}

String getTime()
{
  // See http://www.cplusplus.com/reference/ctime/strftime/ for formatting control chars
  return rtc.getTime("%FT%H:%M:%S");
}
bool hasTimeSet()
{
  // When RTC is set, its task gets deleted
  return eTaskGetState(setTimeTask) == eTaskState::eDeleted ? true : false;
}
// Set time on ESP32 RTC task
void setTimeTaskFunction(void *pvParameters)
{
  int year;
  int month;
  int day;
  int hours;
  int minutes;
  int seconds;

  for (;;)
  {
    if (modem.isGprsConnected())
    {
      Serial.println("Getting time...");
      if (modem.getNetworkTime(&year, &month, &day, &hours, &minutes, &seconds, NULL))
      {
        tm gsmTime;
        gsmTime.tm_sec = (seconds);
        gsmTime.tm_min = (minutes);
        gsmTime.tm_hour = (hours);
        gsmTime.tm_mday = (day);
        gsmTime.tm_mon = (month - 1);
        gsmTime.tm_year = (year - 1900);
        rtc.setTimeStruct(gsmTime);

        Serial.printf("Time set to %s\n", getTime().c_str());

        vTaskDelete(setTimeTask);
      }
    }

    vTaskDelay(xDelay250ms);
  }
}

// Heartbeat sender task
void sendHeartbeatTaskFunction(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xHeartbeatFrequency = pdMS_TO_TICKS(10000);
  const char cHeartbeatMessage[] = "{ \"time\": %s, \"batterypcnt\": %d }";
  for (;;)
  {
    // Send MQTT heartbeat
    if (mqttClient.connected() && hasTimeSet())
    {
      String networkDateTime = getTime();
      int8_t cBatteryPercent = modem.getBattPercent();
      size_t bufSize = snprintf(NULL, 0, cHeartbeatMessage, networkDateTime.c_str(), cBatteryPercent);
      char *payload = (char *)malloc(bufSize + 1);
      sprintf(payload, cHeartbeatMessage, networkDateTime.c_str(), cBatteryPercent);
      mqttClient.publish("testtopic/heartbeat", payload);
      free(payload);
    }

    vTaskDelayUntil(&xLastWakeTime, xHeartbeatFrequency);
  }
}

// Open/close event sender task
void sendOpenCloseStatusChangeTaskFunction(void *pvParameters)
{
  const char cOpenCloseStatusMessage[] = "{ \"time\": %s, \"opened\": %s }";
  openclose_event_payload eventPayload;

  eventPayload = *(static_cast<openclose_event_payload *>(pvParameters));
  Serial.printf("%s %d\n", eventPayload.datetime.c_str(), eventPayload.isOpened);
  String state = eventPayload.isOpened ? "opened" : "closed";
  for (;;)
  {
    if (mqttClient.connected() && hasTimeSet())
    {
      size_t bufSize = snprintf(NULL, 0, cOpenCloseStatusMessage, eventPayload.datetime.c_str(), state.c_str());
      char *message_payload = (char *)malloc(bufSize + 1);
      sprintf(message_payload, cOpenCloseStatusMessage, eventPayload.datetime.c_str(), state.c_str());
      mqttClient.publish("testtopic/openCloseStatus", message_payload);
      free(message_payload);

      vTaskDelete(sendOpenCloseStatusChangeTask);
    }

    vTaskDelay(xDelay250ms);
  }
}

/********************************************/
/*            Interrupt handlers            */
/********************************************/

// Open-close changed event handler
void IRAM_ATTR openCloseStatusChangedHandler()
{
  openclose_event_payload event_payload;
  event_payload.isOpened = digitalRead(openClosePin) > 0 ? true : false;
  event_payload.datetime = "";
  if (hasTimeSet())
  {
    event_payload.datetime = getTime();
  }

  xTaskCreatePinnedToCore(sendOpenCloseStatusChangeTaskFunction, "sendOpenCloseStatusChangeTask", 10000, &event_payload, 2, &sendOpenCloseStatusChangeTask, 1);
}

/************************************/
/*             Setup                */
/************************************/
void setup()
{
  Serial.begin(115200);

  Serial.print("Setting up GSM serial port...");
  SerialGSM.begin(115200, SERIAL_8N1, 17, 16);
  Serial.println("done");

  pinMode(openClosePin, INPUT_PULLUP);
  pinMode(modemResetPin, INPUT_PULLUP);
  attachInterrupt(openClosePin, openCloseStatusChangedHandler, CHANGE);

  // Create all tasks on core 1
  xTaskCreatePinnedToCore(mqttKeepAliveTaskFunction, "mqttKeepAliveTask", 10000, NULL, 3, &mqttKeepAliveTask, 1);
  xTaskCreatePinnedToCore(setTimeTaskFunction, "setTimeTask", 10000, NULL, 3, &setTimeTask, 1);
  xTaskCreatePinnedToCore(sendHeartbeatTaskFunction, "sendHeartbeatTask", 10000, NULL, 2, &heartbeatTask, 1);
}

/************************************/
/*           Main loop              */
/************************************/
void loop()
{
  // if (openCloseStatusChanged)
  // {
  //   isOpened = digitalRead(openClosePin) > 0 ? true : false;

  //   String networkTime = modem.getGSMDateTime(DATE_FULL);
  //   Serial.printf("Time: %s, IP:%s, battery:%d%%, Sensor: %s\n",
  //                 networkTime.c_str(), modem.getLocalIP().c_str(), modem.getBattPercent(), isOpened ? "opened" : "closed");

  //   size_t bufSize = snprintf(NULL, 0, "{ \"opened\": %s }", isOpened ? "true" : "false");
  //   char *payload = (char *)malloc(bufSize + 1);
  //   sprintf(payload, "{ \"opened\": %s }", isOpened ? "true" : "false");

  //   mqttClient.publish("testtopic/openCloseStatus", payload);

  //   openCloseStatusChanged = false;
  //   free(payload);
  // }

  // vTaskDelay(xDelay250ms);
}