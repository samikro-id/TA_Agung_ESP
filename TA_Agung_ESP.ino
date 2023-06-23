#include <WiFi.h>
#include <PubSubClient.h>                   // Install Library by Nick O'Leary version 2.7.0
#include "ADS1X15.h"                        // Install Library by Rob Tillaart version 0.3.9
#include <LiquidCrystal_I2C.h>              // Install Library by Marco Schwartz version 1.1.2

#include "mqtt_secrets.h"

char nama_wifi[] = "RND_Wifi";
char password_wifi[] = "RND12345";

WiFiClient espClient;
PubSubClient client(espClient);

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

#define MQTT_LEN  100
char mqtt_payload[MQTT_LEN];

#define SERIAL_LEN   1000
char text[SERIAL_LEN];

#define LED_BUILTIN         2

#define FLOW_PIN            16
float calibrationFactor = 4.5;

#define RELAY_ON            LOW
#define RELAY_OFF           HIGH

#define RELAY_1_PIN         19
#define RELAY_2_PIN         18
#define RELAY_3_PIN         5
#define RELAY_4_PIN         17

#define JARAK_TRIG_PIN      32
#define JARAK_ECHO_PIN      33
#define JARAK_SENSOR_POSISI 30

#define GAIN_TURBIDITY      2.5
#define GAIN_V_BAT          11
#define GAIN_I_BATT         0.050
#define GAIN_V_PANEL        18.2
#define GAIN_I_PANEL        0.050

ADS1115 ads1(0x48);
ADS1115 ads2(0x49);

LiquidCrystal_I2C lcd(0x27,20,4);

typedef struct{
    bool connection;
    bool mqtt;
    bool led;
    bool charge;
    bool valve1;
    bool valve2;
}STATUS_TypeDef;
STATUS_TypeDef status;

#define LED_TIME_MQTT       500
#define LED_TIME_CONNECTED  100
#define LED_TIME_DISCONNECT 2000

#define TIMEOUT_RECONNECT   60000
#define TIMEOUT_CHART       60000
#define TIMEOUT_UPDATE      1000
typedef struct{
    uint32_t led;
    uint32_t connection;
    uint32_t chart;
    uint32_t update;
}TIMEOUT_TypeDef;
TIMEOUT_TypeDef timeout;

typedef struct{
    float turbidity1;
    float turbidity2;
    float flow;
    uint16_t flow_counter;
    float water_level;
    float v_bat;
    float i_bat;
    float v_panel;
    float i_panel;
}DATA_TypeDef;
DATA_TypeDef data;

void setup(){
    delay(500);
    Serial.begin(115200);

    initLed();
    flowInit();
    relayInit();
    lcdInit();
    lcd.setCursor(0,0);
    lcd.print("TA Agung 2023");

    ads1.begin();
    ads1.setDataRate(7);

    ads2.begin();
    ads2.setDataRate(7);

    status.connection = false;
    status.led = false;
    status.mqtt = false;

    offCharge();
        
    timeout.connection = millis() - TIMEOUT_RECONNECT;
    timeout.update = millis();

    Serial.println("Init");
}

void loop(){
    if(WiFi.isConnected()){
        uint32_t led_timeout = LED_TIME_CONNECTED;
        if(status.connection){  led_timeout = LED_TIME_MQTT;    }
        toggleLed(led_timeout);

        if(status.connection){
            status.connection = client.loop();

            if(status.mqtt){
                status.mqtt = false;

                Serial.println(mqtt_payload);
                clearDataMqtt();
            }

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

    if((millis() - timeout.update) > TIMEOUT_UPDATE){
        uint32_t elapsed_time = millis() - timeout.update;

        Serial.println("=========== update ==============");

        data.flow = flowValue(elapsed_time);
        data.i_bat = iBatt();
        data.i_panel = iPanel();
        data.turbidity1 = turbidity1();
        data.turbidity2 = turbidity2();
        data.v_bat = vBatt();
        data.v_panel = vPanel();
        data.water_level = waterLevel();

        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("V Batt  : "); lcd.print(data.v_bat);
        lcd.setCursor(0,1);
        lcd.print("I Batt  : "); lcd.print(data.i_bat);

        lcd.setCursor(0,2);
        lcd.print("V Panel : "); lcd.print(data.v_panel);
        lcd.setCursor(0,3);
        lcd.print("I Panel : "); lcd.print(data.i_panel);

        Serial.print("Flow ");  Serial.println(data.flow);
        // Serial.print("I Bat ");  Serial.println(data.i_bat);
        // Serial.print("I Panel ");  Serial.println(data.i_panel);
        Serial.print("Turbidity1 ");  Serial.println(data.turbidity1);
        Serial.print("Turbidity2 ");  Serial.println(data.turbidity2);
        // Serial.print("V Bat ");  Serial.println(data.v_bat);
        // Serial.print("V Panel ");  Serial.println(data.v_panel);
        Serial.print("Level ");  Serial.println(data.water_level);
        
        flowEnable();
        timeout.update = millis();
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
            float field1=0;         // valve1
            float field2=0;         // valve2
            float field3=0;         // kejernihan
            float field4=0;         // ketinggihan
            float field5=0;         // tegangan baterai
            float field6=0;         // arus baterai
            float field7=0;         // tegangan pv
            float field8=0;         // arus pv

            sprintf(text,"field1=%d&field2=%d&field3=%.0f&field4=%.0f&field5=%.0f&field6=%.0f&field7=%.0ffield8=%.0f&status=MQTTPUBLISH", 
                    status.valve1, status.valve2,
                    data.turbidity1, data.water_level, 
                    data.v_bat, data.i_bat,
                    data.v_panel, data.i_panel);

            char topic[50];
            memset(&topic, 0, 50);
            sprintf(topic,"channels/%d/publish", CHANNEL_ID);
            client.publish(topic,text,false);

        }
    }
}

/***** Relay Handle ******/
void relayInit(){
    pinMode(RELAY_1_PIN, OUTPUT);
    pinMode(RELAY_2_PIN, OUTPUT);
    pinMode(RELAY_3_PIN, OUTPUT);
    pinMode(RELAY_4_PIN, OUTPUT);

    offCharge();
    closeValve1();
    closeValve2();
}

void onCharge(){
    digitalWrite(RELAY_2_PIN, RELAY_OFF);
    delay(10);
    digitalWrite(RELAY_1_PIN, RELAY_OFF);
    status.charge = false;
}

void offCharge(){
    digitalWrite(RELAY_1_PIN, RELAY_ON);
    delay(10);
    digitalWrite(RELAY_2_PIN, RELAY_ON);
    status.charge = false;
}

void openValve1(){
    digitalWrite(RELAY_3_PIN, RELAY_ON);
    status.valve1 = true;
}

void closeValve1(){
    digitalWrite(RELAY_3_PIN, RELAY_OFF);
    status.valve1 = false;
}

void openValve2(){
    digitalWrite(RELAY_4_PIN, RELAY_ON);
    status.valve2 = true;
}

void closeValve2(){
    digitalWrite(RELAY_4_PIN, RELAY_OFF);
    status.valve2 = false;
}

/***** MQTT Handle *******/
void callback(char* topic, byte* payload, unsigned int length) { //A new message has been received
    memcpy(mqtt_payload, payload, length);
    status.mqtt = true;
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

/***** WIFI Handle *****/
void wifiReconnect(){
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

/***** Turbidity Setting ******/
float turbidityNtu(float volt){
    if(volt < 2.5){
        return 3000;
    }else{
        return -1120.4*pow(volt,2)+5742.3*volt-4353.8; 
    }
}

float turbidity1(){
    ads2.setGain(2);     // GAIN 2.048

    int16_t raw = ads2.readADC(0);
    float vTurbidity = raw * ads2.toVoltage(1) * GAIN_TURBIDITY;

    if(vTurbidity < 0){   vTurbidity = 0;   }

    float ntu = turbidityNtu(vTurbidity);

    return ntu;
}

float turbidity2(){
    ads2.setGain(2);     // GAIN 2.048

    int16_t raw = ads2.readADC(1);
    float vTurbidity = raw * ads2.toVoltage(1) * GAIN_TURBIDITY;

    if(vTurbidity < 0){   vTurbidity = 0;   }

    float ntu = turbidityNtu(vTurbidity);

    return ntu;
}

/***** Voltage And Current Setting ****/
float vBatt(){
    ads1.setGain(2);     // GAIN 2.048

    int16_t raw = ads1.readADC(0);
    float vBat = raw * ads1.toVoltage(1) * GAIN_V_BAT;

    if(vBat < 0){   vBat = 0;   }

    return vBat;
}

float iBatt(){
    ads1.setGain(16);    // GAIN 0.254

    int16_t raw = ads1.readADC_Differential_1_3();
    float iLoad = (raw * ads1.toVoltage(1)) / GAIN_I_BATT;

    return iLoad;
}

float vPanel(){
    ads2.setGain(2);     // GAIN 2.048

    int16_t raw = ads2.readADC(2);
    float vPanel = raw * ads2.toVoltage(1) * GAIN_V_PANEL;

    if(vPanel < 0){   vPanel = 0;   }

    return vPanel;
}

float iPanel(){
    ads1.setGain(16);    // GAIN 0.254

    int16_t raw = ads1.readADC_Differential_1_3();
    float iPanel = (raw * ads1.toVoltage(1)) / GAIN_I_PANEL;

    return iPanel;
}

/***** Flow Setting ****/
void flowCounter(){
    data.flow_counter++;
}

void flowInit(){
    pinMode(FLOW_PIN, INPUT);
    flowEnable();
}

void flowEnable(){
    data.flow_counter = 0;
    attachInterrupt(FLOW_PIN, flowCounter, FALLING);
}

float flowValue(uint32_t elapsed_time){
    detachInterrupt(FLOW_PIN);

    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    float flowRate = ((1000.0 / elapsed_time) * data.flow_counter) / calibrationFactor;

    return flowRate;
}

/***** PING Setting *****/
float waterLevel(){
    // Clears the trigPin
    digitalWrite(JARAK_TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(JARAK_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(JARAK_TRIG_PIN, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    unsigned long duration = pulseIn(JARAK_ECHO_PIN, HIGH);
    // Calculating the distance
    float distance = duration * 0.034 / 2;

    return (float) JARAK_SENSOR_POSISI - distance;
}

/***** LCD Setting *****/
void lcdInit(){
    lcd.init();
    lcd.clear();         
    lcd.backlight();      // Make sure backlight is on
}

/***** LED Setting *****/
void initLed(){
    pinMode(LED_BUILTIN, OUTPUT);
    setLed(true);
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

/***** Tambahan *******/
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