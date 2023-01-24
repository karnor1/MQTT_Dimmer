
#include <WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <RCSwitch.h>

#include <DallasTemperature.h>

Ticker ticker;

#define Dimmer1_PIN  12
#define Dimmer2_PIN  14
#define LDR_PIN  34
#define PIR_PIN 25
#define Switch_PIN1  32
#define Switch_PIN2  33
#define RFT_PIN 23
#define RFR_PIN 22
#define LED1_PIN 2
#define LED2_PIN 15

#define recconnectTimeout 60000
#define connectTimeout 10000

bool USE_PIR = true;
volatile bool PIR_ACTIVE = false;



void IRAM_ATTR PIR_ACTIVADED()
{
      detachInterrupt(PIR_PIN);
      PIR_ACTIVE = true;
}


long Timer2, Timer1, Timer3, lastReconnectAttempt_WIFI=0;
int LDR_Value;
int LED = 2;
int LEDB = 15;

float TempDS18B20(){
    //sensors.requestTemperatures();
    //return (sensors.getTempCByIndex(0));
}


const char *WIFI_SSID = "5G/CHIP Test";
const char *WIFI_PASSWORD = "subraskem";

const char *MQTT_HOST = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "HUB_VALDIKLIS";
const char *MQTT_USER = "karolis";
const char *MQTT_PASSWORD = "223";
const char *TOPIC = "testukas";
const char *TOPIC2 = "/korssf/";



WiFiClient client;
PubSubClient mqttClient(client);



    char* Koridoriaus_Temp_Topic =       (char*)malloc(strlen(TOPIC)*sizeof(TOPIC[0])+strlen(TOPIC2)*sizeof(TOPIC2[0])+15);
    char* Koridoriaus_LDR_Topic =        (char*)malloc(strlen(TOPIC)*sizeof(TOPIC[0])+strlen(TOPIC2)*sizeof(TOPIC2[0])+15);  
    char* Koridoriaus_PIR_Topic =        (char*)malloc(strlen(TOPIC)*sizeof(TOPIC[0])+strlen(TOPIC2)*sizeof(TOPIC2[0])+15);  
    char* Koridoriaus_RFreceive_Topic =  (char*)malloc(strlen(TOPIC)*sizeof(TOPIC[0])+strlen(TOPIC2)*sizeof(TOPIC2[0])+15); 
    char* Koridoriaus_MISC_Topic =       (char*)malloc(strlen(TOPIC)*sizeof(TOPIC[0])+strlen(TOPIC2)*sizeof(TOPIC2[0])+15); 


class physical_switch { 

  private:
    byte pin;
    String title;
    bool ReadyToRead =false;
    bool state = false;
    bool switch1pre = false;
    bool State_Has_Changed=false;
    unsigned int passed =0;


    char *State_topic = (char*)malloc(strlen(TOPIC)*sizeof(TOPIC[0])+strlen(TOPIC2)*sizeof(TOPIC2[0])+15);

  public:

  char *State_topic_return ()
  {
  return State_topic;
  }
    physical_switch (byte Pin, char* name) 
      {

    State_topic[0]='\0';
    strcat(State_topic,TOPIC);
    strcat(State_topic,TOPIC2);
    strcat(State_topic,name);    
    strcat(State_topic,"/state");

    pin=Pin;
    pinMode(pin, INPUT_PULLDOWN);
      }

    bool State_changed()
    {
      if (State_Has_Changed==true)
      {
        State_Has_Changed=false;
        return true;
      }
      else return false;
    }

    void ReadButtonInput() 
    {

      if (ReadyToRead == true ) 
      {
        state = digitalRead(pin);
        passed = millis();
        ReadyToRead = false;      
      }

    if (millis()-passed>250){ReadyToRead=true;}

      if (state != switch1pre) 
      {
        switch1pre = state;

        State_Has_Changed=true;
      }
    }

};

class Dimmer { 

  private:
  uint8_t Dim_setpoint=0, Dim_current=0, Dim_pin=0;
  int Dim_speed=0;
  bool Turned_on_pir=false;
  bool Turned_on_switch=false;
  bool Turned_on_MQTT=false;
  bool Got_new_value = false;
  unsigned long ON_time=0;


  uint8_t by_pir_by_switch=0;

  char *Control_topic = (char*)malloc(strlen(TOPIC)*sizeof(TOPIC[0])+strlen(TOPIC2)*sizeof(TOPIC2[0])+15);
  char *control = "/control";
  char *State_topic = (char*)malloc(strlen(TOPIC)*sizeof(TOPIC[0])+strlen(TOPIC2)*sizeof(TOPIC2[0])+15);
  char *state = "/state";
  
  public:



  bool State_changed ()
  {
    return Got_new_value;
  }
  void Reset_state_changed()
  {
    Got_new_value=false;
  }

  int Dimmer_current_state() 
  {
    return Dim_current;  
  }

  bool ON_BY_PIR ()
  {
    return Turned_on_pir;
  }

  bool ON_BY_SWITCH ()
  {
    return Turned_on_switch;
  }

  unsigned long return_ON_time()
  {
    return ON_time;
  }

  void Dimmer_toggle()
  {
    if(Dimmer_current_state()<50) {Dimmer_set(100,1000,2);Serial.println ("Isjungtas tai ijungiame"); }
    else {Dimmer_set(0,1000,0);Serial.println ("Ijungtas tai isjunigame"); }
  }



  uint8_t Dimmer_set (uint8_t setpoint, int speed, uint8_t By_pir_by_switch)
  {
    ON_time=millis();
    by_pir_by_switch=By_pir_by_switch; // by what it was turned ON to later know how it should be turned OFF |=1 by pir|, |=2 by switch|, |=0 other causes|
                                       // if it was activated by pir it should be turned off by timer if by switch or by direct mqtt message it should be turned of only by that again and not by pir timer
    if       (By_pir_by_switch==1)  {Turned_on_pir=true;}
    else if  (By_pir_by_switch==2)  {Turned_on_switch=true;} 
    else {Turned_on_pir=false; Turned_on_switch=false;}
    Serial.print ("Dimmer SETPoint= ");Serial.println(setpoint);

    Dim_setpoint=setpoint;
    Serial.print ("Dimmer SETPoint1= ");Serial.println(Dim_setpoint);

    Dim_speed=speed;
    Serial.print ("Dimmer speed1= ");Serial.println(Dim_speed);
    Dim_current = Dim_setpoint;
    Dim_setpoint = map(Dim_setpoint, 0, 100, 0, 255);
    

    Serial.print ("Dimmer SETPoint= ");Serial.println(Dim_setpoint);
    
    for (int i=Dim_current; i<=Dim_setpoint; i++)
      {
      analogWrite(Dim_pin, i);
      delayMicroseconds(Dim_speed);
      }

    for (int i=Dim_current; i>=Dim_setpoint;i--)
      {
      analogWrite(Dim_pin, i);
      delayMicroseconds(Dim_speed);
      }
    Dim_setpoint = 0;
    Got_new_value=true;
    return Dim_current;
  }
  



  char *Control_topic_return ()
  {
  return Control_topic;
  }

  char *State_topic_return ()
  {
  return State_topic;
  }

  Dimmer (uint8_t pin, uint8_t speed, char* name)  //Pradiniai parametrai
  {  

    Control_topic[0]='\0';
    strcat(Control_topic,TOPIC);
    strcat(Control_topic,TOPIC2);
    strcat(Control_topic,name);    
    strcat(Control_topic,control);

    State_topic[0]='\0';
    strcat(State_topic,TOPIC);
    strcat(State_topic,TOPIC2);
    strcat(State_topic,name);    
    strcat(State_topic,state);

    Dim_pin=pin;
    Dim_speed=speed;
    pinMode(Dim_pin, OUTPUT);
  }

};

void PublishMqtt(int DATA, char *topikas){

  if (WiFi.status() == WL_CONNECTED && mqttClient.connected() )
  {
    String temp_str;
    char temps[50];
    temp_str=(String)DATA;
    temp_str.toCharArray(temps, temp_str.length() + 1);
    mqttClient.publish(topikas,temps );
    Serial.print("Publishinta I ");
    Serial.print(topikas);
    Serial.print(" reiksme: ");
    Serial.println(temps);
  }
}

void PublishMqtt(float DATA, char *topikas){
  
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected() )
  {
    char temps[50];
    dtostrf(DATA,2,2,temps);
    mqttClient.publish(topikas,temps );
    Serial.print("Publishinta I ");
    Serial.print(topikas);
    Serial.print(" reiksme: ");
    Serial.println(temps);
  }
}

Dimmer Dimmer1 (Dimmer1_PIN,200,"Dimmer1");
Dimmer Dimmer2 (Dimmer2_PIN,200,"Dimmer2");
physical_switch Switch1 (Switch_PIN1,"vidaus");
physical_switch Switch2 (Switch_PIN2,"lauko");
RCSwitch mySwitch = RCSwitch();
RCSwitch mySwitchT = RCSwitch();


void Print_MQTT_topics ()
{
  Serial.println(Dimmer1.Control_topic_return());
  Serial.println(Dimmer1.State_topic_return());
  Serial.println(Dimmer2.Control_topic_return());
  Serial.println(Dimmer2.State_topic_return());
  Serial.println(Koridoriaus_Temp_Topic);
  Serial.println(Koridoriaus_RFreceive_Topic);
  Serial.println(Koridoriaus_PIR_Topic);
  Serial.println(Koridoriaus_LDR_Topic);
  Serial.println(Koridoriaus_MISC_Topic);
}



void Decode_RF_payload (int RFPAYLOAD){
  
  Serial.print("GAUTA PER RF: "); Serial.println(RFPAYLOAD);

  if (!(~RFPAYLOAD & 0b000111101000110110100001))
  {
    Serial.println("Button 1");        
  }
  if (!(~RFPAYLOAD & 0b001000101111111100000010))
  {
    Serial.println("Button 2 (ON) "); PublishMqtt(605, Koridoriaus_MISC_Topic);    Dimmer1.Dimmer_set(100,200,0);
  }
  if (!(~RFPAYLOAD & 0b001000101111111100000100))
  {
    Serial.println("Button 2 (OFF) "); PublishMqtt(606, Koridoriaus_MISC_Topic);    Dimmer1.Dimmer_set(0,200,0);
  }
  if (!(~RFPAYLOAD & 0b001111010010011111110110))
  {
    Serial.println("Door open sensor) "); PublishMqtt(100, Koridoriaus_MISC_Topic); 
  }  


  //PublishMqttlishMqtt(RFPAYLOAD,Koridoriaus_RFreceive_Topic);
  RFPAYLOAD=0;
  mySwitch.resetAvailable();
}

void reconnect_WIFIMQTT()
{
  if (millis()-lastReconnectAttempt_WIFI>recconnectTimeout)
    {
      Serial.println("Trying to reconnect");
      if (WiFi.begin(WIFI_SSID, WIFI_PASSWORD))
      {
        delay(2000);
        Serial.println("WIFI reconnect success");
      }
    }
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected() )
    {
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD )) 
      { 
        delay(500);
        mqttClient.subscribe(Dimmer2.Control_topic_return());
        mqttClient.subscribe(Dimmer1.Control_topic_return());
        Serial.println("MQTT reconnect sucess");
      }
      else Serial.println("MQTT reconnect failed");
    }
  lastReconnectAttempt_WIFI=millis();
  }

void callback(char* topic, byte* payload, unsigned int length)
{
    payload[length] = '\0';
    int value = String((char*) payload).toInt();
    
  if (strcmp(topic,Dimmer1.Control_topic_return())==0) 
  {   Serial.println("gauta zinute");
      Dimmer1.Dimmer_set(value,200,2);   }
          
  if (strcmp(topic,Dimmer2.Control_topic_return())==0) 
  {   Serial.println("gauta zinute");
      Dimmer2.Dimmer_set(value,200,2);   }

    Serial.println(topic);Serial.println(value);
}

void setup()
{

    Serial.begin(9600);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCallback(callback);

    mySwitch.enableReceive(RFR_PIN);
    mySwitchT.enableTransmit(RFT_PIN);
    mySwitchT.setRepeatTransmit(3);

    int connectTimer=millis();
    while (WiFi.status() != WL_CONNECTED && millis()-connectTimer<connectTimeout) 
    {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {Serial.println("Connected to Wi-Fi")}
      else Serial.println("Initial connection to Wi-Fi failed and hit timeout");
        


    while (!mqttClient.connected()) {
        if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD )) {
            Serial.println("Connected to MQTT broker");
        } else {
            delay(500);
            Serial.print(".");
        }
    }


    mqttClient.subscribe(Dimmer2.Control_topic_return());
    mqttClient.subscribe(Dimmer1.Control_topic_return());

    Koridoriaus_Temp_Topic[0]='\0';
    Koridoriaus_LDR_Topic[0]='\0';
    Koridoriaus_PIR_Topic[0]='\0';
    Koridoriaus_RFreceive_Topic[0]='\0';
    Koridoriaus_MISC_Topic[0]='\0';

    strcat(Koridoriaus_MISC_Topic,TOPIC); strcat(Koridoriaus_MISC_Topic,TOPIC2);strcat(Koridoriaus_MISC_Topic,"MISC"); 
    strcat(Koridoriaus_Temp_Topic,TOPIC);  strcat(Koridoriaus_Temp_Topic,TOPIC2); strcat(Koridoriaus_Temp_Topic,"Temp1");   
    strcat(Koridoriaus_LDR_Topic,TOPIC);  strcat(Koridoriaus_LDR_Topic,TOPIC2); strcat(Koridoriaus_LDR_Topic,"LDR");   
    strcat(Koridoriaus_PIR_Topic,TOPIC);  strcat(Koridoriaus_PIR_Topic,TOPIC2); strcat(Koridoriaus_PIR_Topic,"PIR");   
    strcat(Koridoriaus_RFreceive_Topic,TOPIC);  strcat(Koridoriaus_RFreceive_Topic,TOPIC2); strcat(Koridoriaus_RFreceive_Topic,"RFreceive");   

  Print_MQTT_topics();
  
}





void loop()
{

    if (mySwitch.available()) 
    {
      Decode_RF_payload(mySwitch.getReceivedValue());  
    }

  mqttClient.loop();

  Switch1.ReadButtonInput();
  Switch2.ReadButtonInput();

  if (Dimmer1.State_changed()){ PublishMqtt(Dimmer1.Dimmer_current_state(),Dimmer1.State_topic_return()); Dimmer1.Reset_state_changed();}
  if (Dimmer2.State_changed()){ PublishMqtt(Dimmer2.Dimmer_current_state(),Dimmer2.State_topic_return()); Dimmer2.Reset_state_changed();}

  if (Switch1.State_changed()) { Dimmer1.Dimmer_toggle();}
  if (Switch2.State_changed()) { Dimmer2.Dimmer_toggle();}

  

// Read and PublishMqttlish LDR values to mqtt and reactivate interrupt that was deactivated to avoid bounce
  if (millis()-Timer1 >3000)
  {
    if (Dimmer1.Dimmer_current_state()<10){LDR_Value = analogRead(LDR_PIN);}
    PublishMqtt(LDR_Value,Koridoriaus_LDR_Topic);
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), PIR_ACTIVADED, RISING); 
    Timer1 = millis();

    PublishMqtt(Dimmer1.Dimmer_current_state(),Dimmer1.State_topic_return());
    PublishMqtt(Dimmer2.Dimmer_current_state(),Dimmer2.State_topic_return());

  }


  if (millis() - Timer2 >= 30000)
  {
    //Serial.println (TempDS18B20());
    Timer2 = millis();
  }

//Turn off both lights if set time has passed 30mins in this case.
if (millis() - Dimmer1.return_ON_time() >= 1800000 || millis() - Dimmer2.return_ON_time() >= 18000000)
  {    
    if( Dimmer1.ON_BY_SWITCH())
    {
    Dimmer1.Dimmer_set(0,300,0);    
    }
    if( Dimmer2.ON_BY_SWITCH())
    {
    Dimmer2.Dimmer_set(0,300,0);    
    }
  }

//Turn ON dimmer if someone passed by PIR sensor and dimmer is not activated by switch and flag to use pir is true
  if (PIR_ACTIVE && USE_PIR  && !Dimmer1.ON_BY_SWITCH())
  {
    //if (LDR_Value > 500)
    //{
      Dimmer1.Dimmer_set(30,200,1);
   // }
      PIR_ACTIVE=false;
      //on pir =1, on switch =2
  }

//Renew PIR timer if someone is still moving 
  if (Dimmer1.ON_BY_PIR() && PIR_ACTIVE)
  {
    Dimmer1.Dimmer_set(30,1,1);
    PIR_ACTIVE=false;
  }

  
//Turn off PIR if set time has passed (15000 ms) in this case
  if (Dimmer1.ON_BY_PIR() && millis() - Dimmer1.return_ON_time() >= 15000 )
  {
    Dimmer1.Dimmer_set(0,300,0);
    PIR_ACTIVE = false;
    PublishMqtt(0,Koridoriaus_PIR_Topic);
  }



  if (WiFi.status() != WL_CONNECTED || !mqttClient.connected() ){
    reconnect_WIFIMQTT();
  }

}