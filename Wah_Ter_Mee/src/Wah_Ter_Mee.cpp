/* 
 * Project Wah_Ter_Mee
 * Author: Benjamin Brown
 * Date: 07172025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

#include "Particle.h"
#include "Air_Quality_Sensor.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "IoTTimer.h"
#include "Colors.h"
#include "neopixel.h"
#include "Adafruit_BME280.h"
#include "JsonParserGeneratorRK.h"
#include "credentials.h"

TCPClient TheClient; 

Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

Adafruit_MQTT_Subscribe water_Button_Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/wah-ter-mee-button"); 

Adafruit_MQTT_Publish AQFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/seeed-air-qual");
Adafruit_MQTT_Publish Soil_Moisture_Feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil_moisture");
Adafruit_MQTT_Publish enviroFeed= Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/enviromental_cond");

 
SYSTEM_MODE(AUTOMATIC);


const int EM_F_PUMP_PIN = D3;
const int CAP_SENSOR_PIN = A5;
//const int LED_CONT_PIN = D15;

int quality;
int start = 0;
int end = 11;
int c;
const int pixcount = 12;

float tempC,tempF;
float PressPA;
float PressHg;
float HumidRH;

const int DRY_SOIL = 3300;
const int WET_SOIL = 2800;
const int WATER = 1650;

const unsigned int PUMP_ON_TIME = 500;
const unsigned int PUBLISH_TIME = (60000*60);
const unsigned int FLASHTIME = 500;
//unsigned int PIXEL_FLASH_TIMER;
unsigned int lastTime, lastSoil, lastAQ, lastBME, lastAQ_PUB, pixel_Flash_Timer;
const unsigned int TWICE_PER_DAY = ((60000*60)*12);
const unsigned int ONCE_PER_HOUR = (60000*60);
const unsigned int ONCE_PER_MIN = 60000;

bool shouldBreatheGreen, shouldBreatheBlue, shouldBlinkRed;
bool lightControl = false;

bool pumpIsRunning = false;
bool status;
char degree = 0xF8;
int capValue;
float qualValue;
float AQ_Adjusted;
int soil_Adjusted; 
int OnOff;

struct Enviromental_Cond{
  float TempF;
  float Hum;
  float PressHG;
};
Enviromental_Cond env_Cond;

//Declarations
Adafruit_BME280 bme;

IoTTimer pumpTimer;

Adafruit_NeoPixel pixel (pixcount, SPI1, WS2812B);

AirQualitySensor sensor(A1);

const int OLED_RESET =-1;
Adafruit_SSD1306 display(OLED_RESET);

void eventPayLoad(Enviromental_Cond env_Conditions);
void MQTT_connect();
bool MQTT_ping();
void pixelFullBreathe(int start, int end, int c);
void pixelFullBlink(int stpix, int enpix, int c);
void pixelFill (int start, int end, int c);
//void pump_OnOff(const int EM_F_PUMP_PIN, const unsigned int PUMP_ON_TIME);


//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);
  display.clearDisplay();

  pixel.begin();
  pixel.setBrightness(45);
  pixel.show();

  // WiFi.on();
  // WiFi.connect();
  // while(WiFi.connecting()) {
  //   Serial.printf(".");
  // }
  // Serial.printf("\n\n");

  status = bme.begin(0x76);
  if(!status){
    Serial.printf("BME280 at 0x%02x failed to start\n", status);
  }
  pixelFill(start,end,indigo);
  pixel.show();

  pinMode(CAP_SENSOR_PIN,INPUT);
  pinMode(EM_F_PUMP_PIN,OUTPUT);
  //pinMode(LED_CONT_PIN,OUTPUT);

  pixel_Flash_Timer = millis();
  lastAQ = millis();
  lastBME = millis();
  lastSoil = millis();
  lastTime = millis();

  // Setup MQTT subscription
  mqtt.subscribe(&water_Button_Feed);


}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if(lightControl){
    if (shouldBreatheGreen){
      pixelFullBreathe(start,end,green);
     // pixelFullBreathe(start,end,indigo);
    }
    if(shouldBreatheBlue){
      pixelFullBreathe(start,end,blue);
      //pixelFullBreathe(start,end,tomato);
    }
    if(shouldBlinkRed){
      pixelFullBlink(start,end,red);
    }
  }//else{
    //lightControl = false;
    //pixel.clear;
    //pixel.show;
  
  //}
///////////////////////////////////////////////////////////////////////////////////
//BME_280 control
///////////////////////////////////////////////////////////////////////////////////

  if ((millis()-lastBME)>ONCE_PER_MIN/2){

  tempC = bme.readTemperature();
  tempF = ((tempC*1.8)+32);

  PressPA = bme.readPressure();
  PressHg = (PressPA/3386.38672536);
  HumidRH = bme.readHumidity();

  env_Cond.PressHG = PressHg;
  env_Cond.Hum = HumidRH;
  env_Cond.TempF = tempF;


  Serial.printf("The tempurature is %.001f%cC\n%.001f%cF\n", tempC,degree,tempF,degree);
  Serial.printf("Current pressure is %.001fPa\n%.001f or Hg\n",PressPA,PressHg);
  Serial.printf("Humidity is %.001f\n",HumidRH);

  eventPayLoad(env_Cond);
  Serial.printf(" BME CONDITIONS SENT\n");

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();// clear, set cursor and print IN THIS ORDER
    display.setCursor(0,6);
    display.printf("Temp is %.001f\n",tempF);
    display.setCursor(0,22);
    display.printf("Pressure is %.001fHg\n",PressHg);
    display.setCursor(0,38);
    display.printf("Humidity is %.001f\n",HumidRH);
    display.display();
    
    lastBME = millis();
  }

///////////////////////////////////////////////////////////////////////////////////
//SEEED AQ Sensor logic
///////////////////////////////////////////////////////////////////////////////////

  quality =sensor.slope();
  if(millis()-lastAQ>ONCE_PER_HOUR/120){
    Serial.printf("AIR QUALITY Sensor value: %i\n \n",sensor.getValue());
    AQ_Adjusted = (sensor.getValue()*.0244);
    Serial.printf("AQ adjusted = %00f\n \n",AQ_Adjusted);
    if (quality == AirQualitySensor::FORCE_SIGNAL) {
      AQFeed.publish(sensor.getValue()*.0244);

        lightControl = true;
        shouldBlinkRed = true;
        shouldBreatheBlue = false;
        shouldBreatheGreen = false;
        // SEND SMS or EMAIL LOGIC here;
        Serial.printf("High pollution! Force signal active.\n");
      
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
        AQFeed.publish(sensor.getValue()*.0244);
 
        lightControl = true;
        //shouldBlinkRed = true;
        shouldBreatheBlue = false;
        shouldBreatheGreen = false;
          // SEND SMS or EMAIL LOGIC here;
          Serial.printf("High pollution!\n");
        
      } else if (quality == AirQualitySensor::LOW_POLLUTION) {
          Serial.printf("Low pollution!\n");

      } else if (quality == AirQualitySensor::FRESH_AIR) {        
          Serial.printf("Fresh air.\n");
      }

    lastAQ=millis();

  }
   if(millis()-lastAQ_PUB>PUBLISH_TIME/12){
    AQFeed.publish(sensor.getValue()*.0244);
    Serial.printf("Air Quality Sent!\n");
    lastAQ_PUB=millis();
  }

///////////////////////////////////////////////////////////////////////////////////
// Soil Sensor Read and display
///////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////// Reads the soil sensor , sends value to SERIAL and OLED
  if(millis()-lastSoil>ONCE_PER_HOUR/120){//(TWICE_PER_DAY/12)/240){
    capValue = analogRead(CAP_SENSOR_PIN);
    soil_Adjusted = 100 - (capValue*100/4095);
    Serial.printf("Cap sense = %i\n",soil_Adjusted );
    Soil_Moisture_Feed.publish(soil_Adjusted);
    Serial.printf("Soil Moisture Sent!\n");
    if(capValue>=DRY_SOIL){
      if(!pumpIsRunning){
        digitalWrite(EM_F_PUMP_PIN,HIGH);
        pumpTimer.startTimer(PUMP_ON_TIME);
        Serial.printf("SOIL IS DRY\n-------PUMP ON!\n");
        lightControl = true;
        shouldBlinkRed = false;
        shouldBreatheBlue = false;
        shouldBreatheGreen = true;
        pumpIsRunning = true;
      }
     }
      if(capValue<=WET_SOIL){
        lightControl = true;
        shouldBreatheBlue = true;        
        shouldBlinkRed = false;
        shouldBreatheGreen = false;
        Serial.printf("SOIL IS WET\n");
      }
    lastSoil=millis();
  }
  if((pumpIsRunning) && pumpTimer.isTimerReady()){
    digitalWrite(EM_F_PUMP_PIN,LOW);
    Serial.printf("Pump is OFF OFF\n");
    pumpIsRunning = false;
  }


  //////////////////////////////////////////////////////////////////////////////////////
  //Adafruit Wah_Ter_Mee button
  //////////////////////////////////////////////////////////////////////////////////////
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &water_Button_Feed) {
    OnOff = atoi ((char*)water_Button_Feed.lastread);
    digitalWrite(EM_F_PUMP_PIN,OnOff);
    Serial.printf("OnOff Button = %i\n",OnOff);
  }

    
        // SEND SMS or EMAIL LOGIC HERE;
  }

     
  MQTT_connect();
  MQTT_ping();
}
 

/////////////////////////////////////////////////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////////////////////////////////////////////////

  void pixelFullBreathe(int stpix, int enpix, int c){
  float t; //Time
  int b; //Brightness
  float off=255/2;
  float amp=255/2;
  int j;

  t=millis()/1000.0; //returns time from particle in ms
  b=amp*sin(2*M_PI*1.0/5.0*t)+off;

  pixel.setBrightness(b);

  for (j=stpix; j<=enpix; j++){
  pixel.setPixelColor(j, c);

  }
   pixel.show();
}

  void pixelFullBlink(int stpix, int enpix, int c){
  float t; //Time
  int b; //Brightness
  float off=255/2;
  float amp=255/2;
  int j;

  t=millis()/1000.0; //returns time from particle in ms
  b=amp*sin(2*M_PI*1.0/1.5*t)+off;

  pixel.setBrightness(b);

  for (j=stpix; j<=enpix; j++){
  pixel.setPixelColor(j, c);

  }
   pixel.show();
}

// void pump_OnOff(const int PUMP_PIN, const unsigned int PUMP_TIME) {
//   static unsigned int lastPump = 0;
//   static bool pumpFlag = false;
//     if (!pumpFlag) {
//       Serial.printf("Pumping!\n");
//       digitalWrite(PUMP_PIN,HIGH);
//       pumpFlag = true;
//       lastPump = millis();
//     }
//     else if ((millis()-lastPump>PUMP_TIME)) {
//       digitalWrite(PUMP_PIN, LOW);
//       Serial.printf("PUMP OFF OFF OFF\n");
//       pumpFlag = false;
//     }
// }

 void pixelFill (int stpix, int enpix, int c){
  int j;
  for (j=stpix; j<=enpix; j++){
    pixel.setBrightness(255); 
    pixel.setPixelColor(j, c);
 
  // pixel.clear();
  // pixel.show();
  }
   pixel.show();
}

void eventPayLoad(Enviromental_Cond env_Conditions){
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("Humidity",env_Conditions.Hum);
    jw.insertKeyValue("Tempurate F",env_Conditions.TempF);
    jw.insertKeyValue("Pressure HG",env_Conditions.PressHG);
  }
  enviroFeed.publish(jw.getBuffer());
}

void MQTT_connect() {
  int8_t ret;
 
  //Return if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

