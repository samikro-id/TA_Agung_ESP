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
// char mqtt_payload[MQTT_LEN];
String mqtt_payload;

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
    bool via_filter;
}STATUS_TypeDef;
STATUS_TypeDef status;

#define LED_TIME_MQTT       100
#define LED_TIME_CONNECTED  1000
#define LED_TIME_DISCONNECT 2000

#define TIMEOUT_RECONNECT       60000
#define TIMEOUT_CHART           60000
#define TIMEOUT_UPDATE          1000
#define TIMEOUT_UPDATE_CHARGE   180000
#define TIMEOUT_DISPLAY_PAGE1   5000
#define TIMEOUT_DISPLAY_PAGE2   10000
typedef struct{
    uint32_t led;
    uint32_t connection;
    uint32_t chart;
    uint32_t update;
    uint32_t update_charge;
    uint32_t display;
}TIMEOUT_TypeDef;
TIMEOUT_TypeDef timeout;

#define VOLT_PANEL_CHARGE       10.0
#define VOLT_PANEL_DISCHARGE    8.0
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

uint8_t sensor_n = 0;

void setup(){
    delay(3000);
    Serial.begin(115200);

    initLed();
    flowInit();
    relayInit();
    lcdInit();
    lcd.setCursor(0,0);
    lcd.print("TA Agung 2023");

    ads1.begin();   ads1.setDataRate(7);
    ads2.begin();   ads2.setDataRate(7);

    status.connection = false;
    status.mqtt = false;
    status.via_filter = false;
    
    timeout.connection = millis() - TIMEOUT_RECONNECT;
    timeout.update = millis();
    timeout.display = millis();

    Serial.println("Init");

    delay(3000);
}

void loop(){
    if(WiFi.isConnected()){
        uint32_t led_timeout = LED_TIME_CONNECTED;
        if(status.connection){  led_timeout = LED_TIME_MQTT;    }
        toggleLed(led_timeout);

        if(status.connection){
            status.connection = client.loop();

            processData();
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
        data.water_level = waterLevel();

        lcd.clear();
        if((millis() - timeout.display) < TIMEOUT_DISPLAY_PAGE1){
            lcd.setCursor(0,0); lcd.print("-----ELEKTRIKAL-----");
            lcd.setCursor(0,1); lcd.print(" BATERAI   SOLAR P  ");
            lcd.setCursor(0,2); lcd.print("        V         V ");
            lcd.setCursor(0,3); lcd.print("        A         A ");

            lcd.setCursor(1,2); lcd.printf("%0.2f", data.v_bat);
            lcd.setCursor(11,2); lcd.printf("%0.2f", data.v_panel);

            lcd.setCursor(1,3); lcd.printf("%0.2f", data.i_bat);
            lcd.setCursor(11,3); lcd.printf("%0.2f", data.i_panel);
        }
        else {
            lcd.setCursor(0,0); lcd.print("--------AIR---------");
            lcd.setCursor(0,1); lcd.print("SAMPLE:      NTU");
            lcd.setCursor(0,2); lcd.print("FILTER:      NTU");
            lcd.setCursor(0,3); lcd.print("LEVEL:    cm  Flow: ");

            lcd.setCursor(8,1); lcd.printf("%0.0f", data.turbidity1);
            lcd.setCursor(8,2); lcd.printf("%0.0f", data.turbidity2);

            lcd.setCursor(7,3); lcd.printf("%0.0f", data.water_level);
            lcd.setCursor(19,3);
            if(data.flow > 0){  lcd.print("1"); }
            else{               lcd.print("0"); }

            if((millis() - timeout.display) > TIMEOUT_DISPLAY_PAGE2){
                timeout.display = millis();
            }
        }

        // Serial.print("Flow ");  Serial.println(data.flow);
        // Serial.print("I Bat ");  Serial.println(data.i_bat);
        // Serial.print("I Panel ");  Serial.println(data.i_panel);
        // Serial.print("Turbidity1 ");  Serial.println(data.turbidity1);
        // Serial.print("Turbidity2 ");  Serial.println(data.turbidity2, 0);
        // Serial.print("V Bat ");  Serial.println(data.v_bat);
        // Serial.print("V Panel ");  Serial.println(data.v_panel);
        // Serial.print("Level ");  Serial.println(data.water_level);
        
        flowEnable();
        timeout.update = millis();
    }
    else{
        sensor_n++;
        switch(sensor_n){
            case 1: data.turbidity1 = turbidity1(); break;
            case 2: data.turbidity2 = turbidity2(); break;
            case 3: data.v_bat = vBatt(); break;
            case 4: data.i_bat = iBatt(); break;
            case 5: 
                /* Charge Control */
                if(status.charge){
                    if((millis() - timeout.update_charge) > TIMEOUT_UPDATE_CHARGE){
                        timeout.update_charge = millis();

                        Serial.println("Panel Charge");
                        offCharge();
                        delay(200);
                    }

                    if(data.v_panel < VOLT_PANEL_DISCHARGE){
                        offCharge();
                    }
                }   
                else{
                    data.v_panel = vPanel(); 
                    if(data.v_panel > VOLT_PANEL_CHARGE){
                        onCharge();
                    }
                }
                break;
            case 6: data.i_panel = iPanel(); break;
            default : sensor_n = 0; break;
        }
    }

    if(data.flow > 0){

    }
}

void processData(){
    if(status.mqtt){
        Serial.println(mqtt_payload);

        int index = mqtt_payload.indexOf("|");

        if(mqtt_payload.substring(0, index) == "GET"){
            index++;
            if(mqtt_payload.substring(index) == "DATA"){
                publishData();
            }
        }

        clearDataMqtt();
        
        status.mqtt = false;
    }
}

void publishData(){
    /* DATA|valve1|valve2|turbidity1|turbidity2|level|flow|vbat|ibat|vpanel|ipanel*/
    sprintf(text, "DATA|%d|%d|%0.0f|%0.0f|%0.0f|%0.0f|%0.2f|%0.2f|%0.2f|%0.2f", 
                status.valve1, status.valve2, data.turbidity1, data.turbidity2, data.water_level, data.flow, data.v_bat, data.i_bat, data.v_panel, data.i_panel);

    mqttPublish(text);
}

void publishChart(){
    if((millis() - timeout.chart) > TIMEOUT_CHART){
        timeout.chart = millis();
        status.connection = false;

        Serial.println("chart");
        
        client.disconnect();
        client.setServer(MQTT2_BROKER, MQTT2_PORT);

        for(uint8_t n=0; n<MQTT2_TIMEOUT; n++){
            if(client.connect(SECRET_MQTT_CLIENT_ID, SECRET_MQTT_USERNAME, SECRET_MQTT_PASSWORD)){

                sprintf(text,"field1=%d&field2=%d&field3=%.0f&field4=%.0f&field5=%.2f&field6=%.3f&field7=%.2f&field8=%.3f&status=MQTTPUBLISH", 
                        status.valve1, status.valve2,
                        data.turbidity1, data.water_level, 
                        data.v_bat, data.i_bat,
                        data.v_panel, data.i_panel);

                char topic[50];
                memset(&topic, 0, 50);
                sprintf(topic,"channels/%d/publish", CHANNEL_ID);
                client.publish(topic,text,false);

                break;
            }
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
    if(status.charge == false){
        timeout.update_charge = millis();
    }

    digitalWrite(RELAY_2_PIN, RELAY_OFF);
    delay(10);
    digitalWrite(RELAY_1_PIN, RELAY_OFF);
    status.charge = true;
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
    if(status.mqtt == false){
        for(uint16_t i=0; i < length; i++){
            mqtt_payload += (char) payload[i];
        }

        status.mqtt = true;
    }
}

bool mqttConnect(){
    Serial.println("mqtt");

    clearDataMqtt();
    status.mqtt = false;

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

void mqttPublish(char *payload){
    client.publish("samikro/data/project/6",payload,false);
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

    Serial.print("Turbidity 1 ");

    int16_t raw = ads2.readADC(0);
    float vTurbidity = raw * ads2.toVoltage(1) * GAIN_TURBIDITY;

    if(vTurbidity < 0){   vTurbidity = 0;   }

    float ntu = turbidityNtu(vTurbidity);

    Serial.println("end");
    return ntu;
}

float turbidity2(){
    ads2.setGain(2);     // GAIN 2.048

    Serial.print("Turbidity 2 ");

    int16_t raw = ads2.readADC(1);
    float vTurbidity = raw * ads2.toVoltage(1) * GAIN_TURBIDITY;

    if(vTurbidity < 0){   vTurbidity = 0;   }

    float ntu = turbidityNtu(vTurbidity);

    Serial.println("end");
    return ntu;
}

/***** Voltage And Current Setting ****/
float vBatt(){
    ads1.setGain(2);     // GAIN 2.048

    Serial.print("VBatt ");

    int16_t raw = ads1.readADC(0);
    float vBat = raw * ads1.toVoltage(1) * GAIN_V_BAT;

    if(vBat < 0){   vBat = 0;   }

    Serial.println("end");
    return vBat;
}

float iBatt(){
    ads1.setGain(16);    // GAIN 0.254

    Serial.print("IBatt ");

    int16_t raw = ads1.readADC_Differential_1_3();
    float iLoad = (raw * ads1.toVoltage(1)) / GAIN_I_BATT;

    Serial.println("end");
    return iLoad;
}

float vPanel(){
    ads2.setGain(2);     // GAIN 2.048

    Serial.print("VPanel ");

    int16_t raw = ads2.readADC(2);
    float vPanel = raw * ads2.toVoltage(1) * GAIN_V_PANEL;

    vPanel -= data.v_bat;

    if(vPanel < 0){   vPanel = 0;   }

    Serial.println("end");
    return vPanel;
}

float iPanel(){
    ads1.setGain(16);    // GAIN 0.254

    Serial.print("IPanel ");

    int16_t raw = ads1.readADC_Differential_2_3();
    float iPanel = -1 * (raw * ads1.toVoltage(1)) / GAIN_I_PANEL;

    Serial.println("end");
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
    unsigned long duration = pulseIn(JARAK_ECHO_PIN, HIGH, 500);
    // Calculating the distance
    float distance = duration * 0.034 / 2;

    if(distance > JARAK_SENSOR_POSISI){
        distance = 0;
    }

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
    mqtt_payload = "";
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