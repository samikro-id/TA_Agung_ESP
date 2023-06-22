#include <WiFi.h>
#include <PubSubClient.h>                 // Install Library by Nick O'Leary version 2.7.0
#include "mqtt_secrets.h"

char nama_wifi[] = "RND_Wifi";
char password_wifi[] = "RND12345";

WiFiClient espClient;
PubSubClient client(espClient);

#define MQTT_LEN  100
char mqtt_payload[MQTT_LEN];

#define SERIAL_LEN   1000
char text[SERIAL_LEN];

#define LED_BUILTIN         2

typedef struct{
    bool connection;
    bool mqtt;
    bool led;
}STATUS_TypeDef;
STATUS_TypeDef status;

#define LED_TIME_MQTT       500
#define LED_TIME_CONNECTED  100
#define LED_TIME_DISCONNECT 2000

#define TIMEOUT_RECONNECT   60000
#define TIMEOUT_CHART       60000
typedef struct{
    uint32_t led;
    uint32_t connection;
    uint32_t chart;
}TIMEOUT_TypeDef;
TIMEOUT_TypeDef timeout;

typedef struct{
    float voltage;
    float current;
    float capacity;
    float temperature;
    float flag;
}SYSTEM_DataTypeDef;

#define MQTT_ID         "ddfdf6cd-f3eb-4936-a8cb-440ff3518b96"

#define MQTT_BROKER     "broker.emqx.io"            //
#define MQTT_PORT       1883                        //
#define MQTT_USERNAME   ""                          // Change to your Username from Broker
#define MQTT_PASSWORD   ""                          // Change to your password from Broker
#define MQTT_TIMEOUT    10
#define MQTT_QOS        0
#define MQTT_RETAIN     false

#define MQTT2_BROKER    "mqtt3.thingspeak.com"      
#define MQTT2_PORT      1883                       
#define MQTT2_TIMEOUT   10
#define MQTT2_QOS       0
#define MQTT2_RETAIN    false
#define CHANNEL_ID      2193286

void callback(char* topic, byte* payload, unsigned int length) { //A new message has been received
    memcpy(mqtt_payload, payload, length);
    status.mqtt = true;
}

void setup(){
    // delay(300);
    Serial.begin(115200);

    initLed();

    status.connection = false;
    status.mqtt = false;
    timeout.connection = millis() - TIMEOUT_RECONNECT;

    Serial.println("Init");
}

void loop(){
    if(WiFi.isConnected()){
        uint32_t led_timeout = LED_TIME_CONNECTED;
        if(status.connection){  led_timeout = LED_TIME_MQTT;    }
        toggleLed(led_timeout);

        if(status.connection){
            status.connection = client.loop();

            publishChart();
        }
        else{
            status.connection = mqttConnect();
        }
    }
    else{
        toggleLed(LED_TIME_DISCONNECT);
        
        wifiReconnect();
    }

    if(status.mqtt){
        status.mqtt = false;

        Serial.println(mqtt_payload);
        clearDataMqtt();
    }
}

void publishChart(){
    if((millis() - timeout.chart) > TIMEOUT_CHART){
        timeout.chart = millis();
        status.connection = false;

        bool chartIsConnected = false;
        uint8_t n = 0;

        Serial.println("chart");
        
        client.disconnect();
        client.setServer(MQTT2_BROKER, MQTT2_PORT);

        for(n=0; n<MQTT2_TIMEOUT; n++){
            if(client.connect(SECRET_MQTT_CLIENT_ID, SECRET_MQTT_USERNAME, SECRET_MQTT_PASSWORD)){
                chartIsConnected = true;

                break;
            }
        }

        if(chartIsConnected){
            float field1=0;
            float field2=0;
            float field3=0;
            float field4=0;
            float field5=0;
            float field6=0;
            float field7=0;
            float field8=0;
            SYSTEM_DataTypeDef sensor;
            
            field1 = sensor.temperature;
            field2 = sensor.voltage;
            
            field3 = sensor.current;
            field4 = sensor.capacity;
            field5 = sensor.flag;

            sprintf(text,"field1=%.0f&field2=%.2f&field3=%.0f&field4=%.0f&field5=%.0f&field6=%.0f&field7=%.0ffield8=%.0f&status=MQTTPUBLISH", 
                    field1, field2, field3, field4, field5, field6, field7, field8);

            char topic[50];
            memset(&topic, 0, 50);
            sprintf(topic,"channels/%d/publish", CHANNEL_ID);
            client.publish(topic,text,false);

        }
    }
}

bool mqttConnect(){
    Serial.println("mqtt");

    client.disconnect();
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setCallback(callback);
    
    if( client.connect(MQTT_ID, MQTT_USERNAME, MQTT_PASSWORD) ){
        delay(500);
        if(client.subscribe("samikro/cmd/project/6", MQTT_QOS)){
            return true;
        }
    }

    return false;
}

void clearDataMqtt(){
    uint8_t n;
    for(n=0; n<MQTT_LEN; n++){
        mqtt_payload[n] = 0;
    }
}

int findChar(char * data, char character, int start_index){
    int n;
    for(n=start_index; n < strlen(data); n++){
        if(data[n] == character){
            break;
        }
    }

    return n;
}

String charToString(char * data, int start, int end){
    String buff = "";
    for(int n=start; n<=end; n++){
        buff += data[n];
    }

    return buff;
}

/***** WIFI Handle *****/
bool wifiReconnect(){
    if((millis() - timeout.connection) > TIMEOUT_RECONNECT){
        timeout.connection = millis();

        Serial.println("wifi");

        status.connection = false;
        WiFi.disconnect();

        WiFi.disconnect(true);
        WiFi.mode(WIFI_STA);
        yield();

        WiFi.begin(nama_wifi, password_wifi);
    }
};


/***** LED Setting *****/
void initLed(){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);      // default mati
    status.led = false;
}

void toggleLed(uint32_t timer){
    if((millis() - timeout.led) > timer){
        timeout.led = millis();

        if(status.led == false){
            setLed(true);
        }else{
            setLed(false);
        }
    }
}

void setLed(bool state){
    if(state){    digitalWrite(LED_BUILTIN, LOW);     }// nyalakan LED
    else{    digitalWrite(LED_BUILTIN, HIGH);    }// matikan LED

    status.led = state;
}