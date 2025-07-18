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

TCPClient TheClient; 

Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

// Adafruit_MQTT_Subscribe water_Button_Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/input-data"); 
Adafruit_MQTT_Publish AQFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/seeed-air-qual");

 
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
const int pixcount = 12;
int capValue;
float qualValue;
unsigned int PUBLISH_TIME = 3000;
unsigned int READTIME = 1000;

//Declarations

void pixelFullBreathe(int start, int end, int color);

Adafruit_NeoPixel pixel (pixcount, SPI1, WS2812B);

AirQualitySensor sensor(A0);

const int OLED_RESET =-1;
Adafruit_SSD1306 display(OLED_RESET);


void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);
  display.clearDisplay();

  pinMode(CAP_SENSOR_PIN,INPUT);

  
  lastAQ = millis();
  lastBME = millis();
  lastSoil = millis();
  lastTime = millis();

}

void loop() {
  // MQTT_connect();
  // MQTT_ping();
  
  quality =sensor.slope();
  if(millis()-lastAQ>ONCE_PER_HOUR/20){

    Serial.printf("Sensor value: %i\n",sensor.getValue());

    if (quality == AirQualitySensor::FORCE_SIGNAL) {
        Serial.printf("High pollution! Force signal active.\n");
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
        Serial.printf("High pollution!\n");
    } else if (quality == AirQualitySensor::LOW_POLLUTION) {
        Serial.printf("Low pollution!\n");
    } else if (quality == AirQualitySensor::FRESH_AIR) {
        Serial.printf("Fresh air.\n");
        pixelFullBreathe(start,end,green);

    }
    lastTime=millis();
    //delay(1000);
  }

   if(millis()-lastSoil>PUBLISH_TIME){
    AQFeed.publish(sensor.getValue());
    Serial.printf("Air Quality Sent!\n");
    lastSoil=millis();
  }

  ////////////////////////////////////////////// Reads the soil sensor , sends value to SERIAL and OLED
  // if(millis()-lastTime>READTIME){
  // capValue = analogRead(CAP_SENSOR_PIN);
  // Serial.printf("Cap sense = %i\n",capValue);

  // display.clearDisplay();
  // display.setRotation(0);
  // display.setTextSize(1);
  // display.setTextColor(WHITE);
  // display.setCursor(0,0);
  // display.printf("Cap sense = %i\n",capValue);
  // display.display();
  // //delay(1000);
  //   lastTime=millis();
  // }

}

  
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

