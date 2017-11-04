#include <FS.h>                   //Filessystem

#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>
#include <SoftwareSerial.h>

/*
 * Pin Configuration
 */

const int SW_UART_RX = 4;
const int SW_UART_TX = 5;
const int WLAN_RESET = 12;

/*
 * Software Serial for Arduino Communication
 */

SoftwareSerial swSer(SW_UART_RX, SW_UART_TX, false, 256);

/*
 * MQTT Client
 */

WiFiClient espClient;
PubSubClient client(espClient);

/*
 * MQQT Topics
 */

char TOPIC_SENSOR_TEMP[48] = "XXXXXXXX/balconyesp/sensor/temp";
char TOPIC_SENSOR_HUMIDITY[48] = "XXXXXXXX/balconyesp/sensor/humidity";
char TOPIC_SENSOR_PRESSURE[48] = "XXXXXXXX/balconyesp/sensor/pressure";
char TOPIC_SENSOR_HEIGHT[48] = "XXXXXXXX/balconyesp/sensor/height";
char TOPIC_SENSOR_DEWPOINT[48] = "XXXXXXXX/balconyesp/sensor/dewpoint";
char TOPIC_SENSOR_CPU_TEMP[48] = "XXXXXXXX/balconyesp/sensor/cpu_temp";
char TOPIC_SENSOR_NODE[48] = "XXXXXXXX/balconyesp/sensor/node_N";

char TOPIC_ACTOR_PUMP[48] = "XXXXXXXX/balconyesp/actor/pump_N";

char TOPIC_POWER_PANEL_VOLTAGE[48] = "XXXXXXXX/balconyesp/power/panel_voltage";
char TOPIC_POWER_PANEL_CURRENT[48] = "XXXXXXXX/balconyesp/power/panel_current";
char TOPIC_POWER_PANEL_POWER[48] = "XXXXXXXX/balconyesp/power/panel_power";
char TOPIC_POWER_SYSTEM_VOLTAGE[48] = "XXXXXXXX/balconyesp/power/system_voltage";
char TOPIC_POWER_SYSTEM_CURRENT[48] = "XXXXXXXX/balconyesp/power/system_current";
char TOPIC_POWER_SYSTEM_POWER[48] = "XXXXXXXX/balconyesp/power/system_power";

char TOPIC_SYSTEM_IP[48] = "XXXXXXXX/balconyesp/system/ip";
char TOPIC_SYSTEM_AP[48] = "XXXXXXXX/balconyesp/system/ap";

/*
 * Power saving 
 */

const int SLEEP_TIME = 280;

/*
 * Saving Flag and Callback
 */
 
bool shouldSaveConfig = false;

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

/*
 * Declare MQTT connect default parameters
 */

char mqtt_server[40] = "192.168.0.1";
char mqtt_port[6] = "8883";
char mqtt_topic[9] = "BalcoESP";

/*
 * Variables
 */

bool dataRetrieved = false;

/*
 * Callback MQTT
 */

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(4, LOW);   // Turn the LED on (Note that LOW is the voltage level
    Serial.println("The topic is positive");
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(4, HIGH);  // Turn the LED off by making the voltage HIGH
    Serial.println("The topic is negative");
  }
  retrieveSensorData();
  graceSleep();
}

/*
 * Setup Function
 */

void setup() {
  // initialize serial console
  Serial.begin(115200);
  Serial.println();
  swSer.begin(9600);

  // vent output
  pinMode(WLAN_RESET, INPUT_PULLUP);
  disconnectWifi();

  // Mount filesystem
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_topic, json["mqtt_topic"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  // Declare custom parameters
   
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", mqtt_topic, 9);
 
  // Declare WifiManager

  WiFiManager wifiManager;
  
  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  // Adding custom parameters
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_topic);
    
  // put your setup code here, to run once:
  wifiManager.autoConnect("BalconyESP");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_topic, custom_mqtt_topic.getValue());  

    //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_topic"] = mqtt_topic;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save

  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  Serial.println("connecting mqtt broker");
  unsigned int portInt = atoi (mqtt_port);
  client.setServer(mqtt_server, portInt);
  client.setCallback(mqttCallback);

  if (!client.connected()) {
    reconnectMQTT();
  }  
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void disconnectWifi(){
  int resetValue = digitalRead(WLAN_RESET);
    
  if(resetValue == 0){
    Serial.println("disconnecting wifi");
    WiFi.disconnect();
    Serial.println("resetting shortly, unbridge reset pin");
    for(int i = 10; i >= 1; i--){
      Serial.print("reset in ");
      Serial.print(i);
      Serial.print(" seconds");
      Serial.println();
      delay(1000);
    }
    ESP.reset();
  }
}

void reconnectMQTT(){
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {
    if (client.connect("BalconyESP")) {
      Serial.println("connected");
      client.subscribe("BalconyESP_vent1");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishMessage(char topic[], String message){
  for(int i = 0; i < 8; i++){
    topic[i] = mqtt_topic[i];
  }
  logger("publishMessage","TOPIC: " + String(topic));
  logger("publishMessage","VALUE: " + message);
  char messageBuffer[message.length()];
  message.toCharArray(messageBuffer, message.length());
  client.publish(topic, messageBuffer);
}

void publishMessageNumber(char topic[], String message, int modifyIndex, char modifyChar){
  for(int i = 0; i < 8; i++){
    topic[i] = mqtt_topic[i];
  }
  topic[modifyIndex] = modifyChar;
  logger("publishMessage","TOPIC: " + String(topic));
  logger("publishMessage","VALUE: " + message);
  char messageBuffer[message.length()];
  message.toCharArray(messageBuffer, message.length());
  client.publish(topic, messageBuffer);
}

void graceSleep(){
  Serial.print("Going to sleep for ");
  Serial.print(SLEEP_TIME);
  Serial.print(" seconds");
  Serial.println();
  client.disconnect();
  ESP.deepSleep(SLEEP_TIME * 1000000);
}

void forwardCommand(){
  
}

/*
 * Tell the Arduino to set its latest Sensor Data, wait until it is done and publish the results to MQTT
 */

void retrieveSensorData(){
  if(!dataRetrieved){
    logger("retrieveSensorData","getSensorData");
    swSer.println("getSensorData");
    delay(200);
    String sensorData = swSer.readString();
    logger("retrieveSensorData","gotSensorData");
    logger("retrieveSensorData",sensorData);
    //send temp data
    String tempData = "";
    int tempInt = 0;
    //forward sensor data
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);

    /*
     * Environmental Sensor Data
     */
    logger("retrieveSensorData", "TEMP: " + tempData);
    publishMessage(TOPIC_SENSOR_TEMP, tempData);
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "HUMIDITY: " + tempData);
    publishMessage(TOPIC_SENSOR_HUMIDITY, tempData);
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "PRESSURE: " + tempData);
    publishMessage(TOPIC_SENSOR_PRESSURE, tempData);
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "HEIGHT: " + tempData);
    publishMessage(TOPIC_SENSOR_HEIGHT, tempData);
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "DEWPOINT: " + tempData);
    publishMessage(TOPIC_SENSOR_DEWPOINT, tempData);
    sensorData = sensorData.substring(tempInt+1);
      /*
     * Humidity Sensors
     */
    logger("retrieveSensorData","getHydroData");
    swSer.println("getHydroData");
    delay(200);
    sensorData = swSer.readString();    
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt); 
    logger("retrieveSensorData", "NODE 1: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, '0');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 2: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, '1');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 3: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, '2');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 4: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, '3');
    sensorData = sensorData.substring(tempInt+1);
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 5: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, '4');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 6: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, '5');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 7: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, '6');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 8: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, '7');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 9: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, '8');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 10: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, '9');
    sensorData = sensorData.substring(tempInt+1);
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 11: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, 'A');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 12: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, 'B');
    sensorData = sensorData.substring(tempInt+1);  
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 13: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, 'C');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 14: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, 'D');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 15: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, 'E');
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "NODE 16: " + tempData);
    publishMessageNumber(TOPIC_SENSOR_NODE, tempData, 32, 'F');

    /*
     * Send System Temp Data
     */
    logger("retrieveSensorData","getSystemTemp");
    swSer.println("getSystemTemp");
    delay(200);
    sensorData = swSer.readString(); 
    publishMessage(TOPIC_SENSOR_CPU_TEMP, sensorData);

    /*
     * Send Solar Power Data
     */

    logger("retrieveSensorData","getSolarPower");
    swSer.println("getSolarPower");
    delay(200);
    sensorData = swSer.readString();    
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "TOPIC_POWER_PANEL_CURRENT: " + tempData);
    publishMessage(TOPIC_POWER_PANEL_CURRENT, tempData);
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt); 
    logger("retrieveSensorData", "TOPIC_POWER_PANEL_VOLTAGE: " + tempData);
    publishMessage(TOPIC_POWER_PANEL_VOLTAGE, tempData);
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "TOPIC_POWER_PANEL_POWER: " + tempData);
    publishMessage(TOPIC_POWER_PANEL_POWER, tempData);

    /*
     * Send System Power Data 
     */

    logger("retrieveSensorData","getSystemPower");
    swSer.println("getSystemPower");
    delay(200);
    sensorData = swSer.readString();   
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "TOPIC_POWER_SYSTEM_CURRENT: " + tempData);
    publishMessage(TOPIC_POWER_SYSTEM_CURRENT, tempData);
    sensorData = sensorData.substring(tempInt+1); 
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt); 
    logger("retrieveSensorData", "TOPIC_POWER_PANEL_VOLTAGE: " + tempData);
    publishMessage(TOPIC_POWER_SYSTEM_VOLTAGE, tempData);
    sensorData = sensorData.substring(tempInt+1);
    tempInt = sensorData.indexOf(";");
    tempData = sensorData.substring(0,tempInt);
    logger("retrieveSensorData", "TOPIC_POWER_SYSTEM_POWER: " + tempData);
    publishMessage(TOPIC_POWER_SYSTEM_POWER, tempData);

    /*
     * Send AP and IP Data
     */

    publishMessage(TOPIC_SYSTEM_AP, WiFi.SSID());
    publishMessage(TOPIC_SYSTEM_IP, WiFi.localIP().toString());
    
    dataRetrieved = true;
  }
}

void logger(String functionName, String logValue){
  Serial.println(functionName + " >> " + logValue);
}

void loop() {
  // put your main code here, to run repeatedly:
  //if (!client.connected()) {
  //  reconnectMQTT();
  //}  
  client.loop();
}
