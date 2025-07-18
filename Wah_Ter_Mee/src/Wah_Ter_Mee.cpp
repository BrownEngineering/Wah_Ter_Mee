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
#include "credentials.h"

TCPClient TheClient; 

Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

// Adafruit_MQTT_Subscribe water_Button_Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/input-data"); 
Adafruit_MQTT_Publish AQFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/seeed-air-qual");
Adafruit_MQTT_Publish Soil_Moisture_Feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil_moisture");

 
SYSTEM_MODE(MANUAL);


const int EM_F_PUMP_PIN = D2;
const int CAP_SENSOR_PIN = A5;
const int LED_CONT_PIN = D15;
unsigned int lastTime, lastSoil, lastAQ, lastBME;
const unsigned int TWICE_PER_DAY = ((60000*60)*12);
const unsigned int ONCE_PER_HOUR = (60000*60);
const unsigned int ONCE_PER_MIN = 60000;
int quality;
int start = 0;
int end = 11;
int c;
// float PressPA;
// float PressHg;
// float HumidRH;

const int DRY_SOIL = 3200;
const int WET_SOIL = 2800;
const int WATER = 1600;

const unsigned int PUMP_ON_TIME = 500;
const unsigned int PUBLISH_TIME = 3000;
const unsigned int FLASHTIME = 500;
const unsigned int PIXEL_FLASH_TIMER;

bool status;
const int pixcount = 12;
int capValue;
float qualValue;


//Declarations
Adafruit_BME280 bme;

void pixelFullBreathe(int start, int end, int c);
void pixelFill (int start, int end, int c);
void pump_OnOff(const int EM_F_PUMP_PIN, const int PUMP_ON_TIME)

Adafruit_NeoPixel pixel (pixcount, SPI1, WS2812B);

AirQualitySensor sensor(A0);

const int OLED_RESET =-1;
Adafruit_SSD1306 display(OLED_RESET);
;


void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);
  display.clearDisplay();
  status = bme.begin(0x76);

  pinMode(CAP_SENSOR_PIN,INPUT);

  
  lastAQ = millis();
  lastBME = millis();
  lastSoil = millis();
  lastTime = millis();

}

void loop() {



///////////////////////////////////////////////////////////////////////////////////
//BME_280 control
///////////////////////////////////////////////////////////////////////////////////

  // if ((millis()-lastSecond)>1000){
  //   lastSecond = millis();
  //   Serial.printf(".\n");
  // }
  // tempC = bme.readTemperature();
  // tempF = ((tempC*1.8)+32);

  // PressPA = bme.readPressure();
  // PressHg = (PressPA/3386.38672536);
  // HumidRH = bme.readHumidity();

  // if (timer.isTimerReady()){
  //   Serial.printf("The tempurature is %.001f%cC\n%.001f%cF\n", tempC,degree,tempF,degree);
  //   Serial.printf("Current pressure is %.001fPa\n%.001f or Hg\n",PressPA,PressHg);
  //   Serial.printf("Humidity is %.001f\n",HumidRH);
    
  //   display.setTextSize(1);
  //   display.setTextColor(WHITE);
  //   display.clearDisplay();// clear, set cursor and print IN THIS ORDER
  //   display.setCursor(0,16);
  //   display.printf("Temp is %.001f%c\n",tempF);
  //   display.setCursor(0,33);
  //   display.printf("Pressure is %.001fHg\n",PressHg);
  //   display.display();
  //   timer.startTimer(2000);
  // }




  // MQTT_connect();
  // MQTT_ping();


  
///////////////////////////////////////////////////////////////////////////////////
//SEEED AQ Sensor logic
///////////////////////////////////////////////////////////////////////////////////


  quality =sensor.slope();
  if(millis()-lastAQ>ONCE_PER_HOUR/20){
    Serial.printf("Sensor value: %i\n",sensor.getValue());

    if (quality == AirQualitySensor::FORCE_SIGNAL) {
      AQFeed.publish(sensor.getValue());
      PIXEL_FLASH_TIMER = millis();
      if(millis()-PIXEL_FLASH_TIMER>FLASHTIME){
      pixelFill(start,end,red);
      PIXEL_FLASH_TIMER = millis();
      // SEND SMS or EMAIL LOGIC here;
      Serial.printf("High pollution! Force signal active.\n");
      
      } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
          AQFeed.publish(sensor.getValue());
          PIXEL_FLASH_TIMER = millis();
          if(millis()-PIXEL_FLASH_TIMER>FLASHTIME){
          pixelFill(start,end,red);
          PIXEL_FLASH_TIMER = millis();
          // SEND SMS or EMAIL LOGIC here;
          Serial.printf("High pollution!\n");
   
      } else if (quality == AirQualitySensor::LOW_POLLUTION) {
          Serial.printf("Low pollution!\n");

      } else if (quality == AirQualitySensor::FRESH_AIR) {        
          Serial.printf("Fresh air.\n");
      }
    lastAQ=millis();
  }
   if(millis()-lastAQ_PUB>PUBLISH_TIME){
    AQFeed.publish(sensor.getValue());
    Serial.printf("Air Quality Sent!\n");
    lastAQ_PUB=millis();
  }



///////////////////////////////////////////////////////////////////////////////////
// Soil Sensor Read and display
///////////////////////////////////////////////////////////////////////////////////



  ////////////////////////////////////////////// Reads the soil sensor , sends value to SERIAL and OLED
  if(millis()-lastSoil>TWICE_PER_DAY){
  capValue = analogRead(CAP_SENSOR_PIN);
  Serial.printf("Cap sense = %i\n",capValue);
    if(capValue>=DRY_SOIL){
      pump_OnOff();
      pixelFullBreathe(start,end,green);
    }else if(capValue>=WET_SOIL){
      pixelFullBreathe(start,end,blue);
      // SEND SMS or EMAIL LOGIC HERE;

    }

  display.clearDisplay();
  display.setRotation(0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.printf("Cap sense = %i\n",capValue);
  display.display();

  lastSoil=millis();
  }

  if(millis()-lastSoil>PUBLISH_TIME){
    Soil_Moisture_Feed.publish(capValue*.0244);
    Serial.printf("Soil Moisture Sent!\n");
    lastSoil = millis();

  }

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

void pump_OnOff(const int PUMP_PIN, const unsigned int PUMP_TIME){
  const int PUMP_PIN;
  unsigned int lastPump;
 // const int PUMP_TIME = 500;

  lastPump = millis();

  digitalWrite(PUMP_PIN, HIGH);
  if(millis()-lastPump>PUMP_TIME){
    digitalWrite(PUMP_PIN,LOW);
    lastPump=millis();
  }
}

 void pixelFill (int stpix, int enpix, int c){
  int j;
  for (j=stpix; j<=enpix; j++){
   
  pixel.setPixelColor(j, c);
 
  // pixel.clear();
  // pixel.show();
  }
   pixel.show();
}
