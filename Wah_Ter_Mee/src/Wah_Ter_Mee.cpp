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
#include "Colors.h"
#include "neopixel.h"
#include "Adafruit_BME280.h"
#include "JsonParserGeneratorRK.h"
#include "credentials.h"

//TCPClient TheClient; 

//Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

// Adafruit_MQTT_Subscribe water_Button_Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/input-data"); 

// Adafruit_MQTT_Publish AQFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/seeed-air-qual");
// Adafruit_MQTT_Publish Soil_Moisture_Feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil_moisture");
// Adafruit_MQTT_Publish enviroFeed= Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/enviromental_cond");

 
SYSTEM_MODE(MANUAL);


const int EM_F_PUMP_PIN = D2;
const int CAP_SENSOR_PIN = A5;
const int LED_CONT_PIN = D15;

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

bool status;
char degree = 0xF8;
int capValue;
float qualValue;

struct Enviromental_Cond{
  float TempF;
  float Hum;
  float PressHG;
};
Enviromental_Cond env_Cond;

//Declarations
Adafruit_BME280 bme;

Adafruit_NeoPixel pixel (pixcount, SPI1, WS2812B);

AirQualitySensor sensor(A1);

const int OLED_RESET =-1;
Adafruit_SSD1306 display(OLED_RESET);

void eventPayLoad(Enviromental_Cond env_Conditions);
// void MQTT_connect();
// bool MQTT_ping();
void pixelFullBreathe(int start, int end, int c);
void pixelFill (int start, int end, int c);
void pump_OnOff(const int EM_F_PUMP_PIN, const unsigned int PUMP_ON_TIME);

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);
  display.clearDisplay();

  status = bme.begin(0x76);
  if(!status){
    Serial.printf("BME280 at 0x%02x failed to start\n", status);
  }
  pixelFill(start,end,indigo);
  pinMode(CAP_SENSOR_PIN,INPUT);

  pixel_Flash_Timer = millis();
  lastAQ = millis();
  lastBME = millis();
  lastSoil = millis();
  lastTime = millis();

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

///////////////////////////////////////////////////////////////////////////////////
//BME_280 control
///////////////////////////////////////////////////////////////////////////////////

  if ((millis()-lastBME)>ONCE_PER_MIN/2){

  tempC = bme.readTemperature();
  tempF = ((tempC*1.8)+32);

  PressPA = bme.readPressure();
  PressHg = (PressPA/3386.38672536);
  HumidRH = bme.readHumidity();

    Serial.printf("The tempurature is %.001f%cC\n%.001f%cF\n", tempC,degree,tempF,degree);
    Serial.printf("Current pressure is %.001fPa\n%.001f or Hg\n",PressPA,PressHg);
    Serial.printf("Humidity is %.001f\n",HumidRH);
    
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
    Serial.printf("Sensor value: %i\n",sensor.getValue());

    if (quality == AirQualitySensor::FORCE_SIGNAL) {
     // AQFeed.publish(sensor.getValue());
      if(millis()-pixel_Flash_Timer>FLASHTIME){
        pixelFill(start,end,red);
        pixel_Flash_Timer = millis();
        // SEND SMS or EMAIL LOGIC here;
        Serial.printf("High pollution! Force signal active.\n");
      }
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
      //  AQFeed.publish(sensor.getValue());
        pixel_Flash_Timer = millis();
        if(millis()-pixel_Flash_Timer>FLASHTIME){
          pixelFill(start,end,red);
          pixel_Flash_Timer = millis();
          // SEND SMS or EMAIL LOGIC here;
          Serial.printf("High pollution!\n");
        }
      } else if (quality == AirQualitySensor::LOW_POLLUTION) {
          Serial.printf("Low pollution!\n");

      } else if (quality == AirQualitySensor::FRESH_AIR) {        
          Serial.printf("Fresh air.\n");
      }

    lastAQ=millis();

  }
   if(millis()-lastAQ_PUB>PUBLISH_TIME/12){
    //AQFeed.publish(sensor.getValue());
    Serial.printf("Air Quality Sent!\n");
    lastAQ_PUB=millis();
  }

///////////////////////////////////////////////////////////////////////////////////
// Soil Sensor Read and display
///////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////// Reads the soil sensor , sends value to SERIAL and OLED
  if(millis()-lastSoil>ONCE_PER_HOUR/240){//(TWICE_PER_DAY/12)/240){
    capValue = analogRead(CAP_SENSOR_PIN);
    Serial.printf("Cap sense = %i\n",capValue);
    //Soil_Moisture_Feed.publish(capValue*.0244);
    Serial.printf("Soil Moisture Sent!\n");
      if(capValue>=DRY_SOIL){
        pump_OnOff(EM_F_PUMP_PIN,PUMP_ON_TIME);
        Serial.printf("SOIL IS DRY\nh");
        pixelFullBreathe(start,end,green);
      }else if(capValue>=WET_SOIL){
        pixelFullBreathe(start,end,blue);
        Serial.printf("SOIL IS WET\n");
        // SEND SMS or EMAIL LOGIC HERE;
      }
      // display.clearDisplay();
      // display.setRotation(0);
      // display.setTextSize(1);
      // display.setTextColor(WHITE);
      // display.setCursor(0,0);
      // display.printf("Cap sense = %i\n",capValue);
      // display.display();
    lastSoil=millis();
  }
    
    
  // MQTT_connect();
  // MQTT_ping();
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

  for (j=stpix; j<=enpix; j++){
  pixel.setPixelColor(j, c);
  pixel.setBrightness(b);
  }
   pixel.show();
}

// unsigned long lastPump = 0;  // Declare this globally or in the setup function

// void pump_OnOff(const int PUMP_PIN, const unsigned int PUMP_TIME){
//     if(millis() - lastPump > PUMP_TIME) {
//         Serial.printf("Pumping!\n");
//         digitalWrite(PUMP_PIN, HIGH);  // Turn on the pump
//         lastPump = millis();  // Update the lastPump time
//     } else {
//         digitalWrite(PUMP_PIN, LOW);  // Turn off the pump
//     }
// }
    static unsigned int lastPump = 0;
void pump_OnOff(const int PUMP_PIN, const unsigned int PUMP_TIME) {

    if (millis()-lastPump<=PUMP_TIME) {
        Serial.printf("Pumping!\n");
        digitalWrite(PUMP_PIN, HIGH);
    }
    if (millis()-lastPump>PUMP_TIME) {
        digitalWrite(PUMP_PIN, LOW);
        lastPump=millis();
    }
}
  
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
  //enviroFeed.publish(jw.getBuffer());
}

// void MQTT_connect() {
//   int8_t ret;
 
  // Return if already connected.
//   if (mqtt.connected()) {
//     return;
//   }
//   Serial.print("Connecting to MQTT... ");
 
//   while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
//        Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
//        Serial.printf("Retrying MQTT connection in 5 seconds...\n");
//        mqtt.disconnect();
//        delay(5000);  // wait 5 seconds and try again
//   }
//   Serial.printf("MQTT Connected!\n");
// }

// bool MQTT_ping() {
//   static unsigned int last;
//   bool pingStatus;

//   if ((millis()-last)>120000) {
//       Serial.printf("Pinging MQTT \n");
//       pingStatus = mqtt.ping();
//       if(!pingStatus) {
//         Serial.printf("Disconnecting \n");
//         mqtt.disconnect();
//       }
//       last = millis();
//   }
//   return pingStatus;
// }

