/*----------------------------------------------------------------------------------------------------
  Project Name : Solar Powered WiFi Weather Station V2.34
  Features: temperature, dewpoint, dewpoint spread, heat index, humidity, absolute pressure, relative pressure, battery status and
  the famous Zambretti Forecaster (multi lingual)
  Authors: Keith Hungerford, Debasish Dutta and Marc Stähli
  Website : www.opengreenenergy.com

******* configuration control constant for use of Blynk and/or Thingspeak ***/

const String App1 = "";         // empty string if not applicable -> "" else "BLYNK" 
const String App2 = "";    // empty string if not applicable -> "" else "THINGSPEAK"


/****** Blink or ThingSpeak Settings ****************************************/

char auth[] = "your Blynk Auth Token"; // Blynk Auth Token

char ssid[] = "Dhome";                           // WiFi Router ssid
char pass[] = "TiffanyZ7";             // WiFi Router password

const char* server = "api.thingspeak.com";        // Thingspeak Write API
const char* api_key = "313Q7NID6CV71OT0";         // API write key 
unsigned long ch_no = 1465640;//Replace
/****** MQTT Settings ********************************************************/

const char* mqtt_server = "192.168.1.212";      // MQTT Server (broker) address


/****** Additional Settings **************************************************/

#define LANGUAGE 'EN'               //check translation.h for available languages. Currently EN/DE/FR/IT/PL/RO/SP/TR/NL/NO

#define TEMP_CORR (+1.2)              //Manual correction of temp sensor (mine reads 1 degree too high)
#define HUMI_CORR (-23)              // not used anymore in V2.34 (automatically calculated)

#define ELEVATION (27.7)             //Enter your elevation in m ASL to calculate rel pressure (ASL/QNH) at your place

#define sleepmin (10)           //setting of deepsleep time in minutes (default: 10)

// NTP   --> Just a remark - the program needs the time only for the timestamp, so for the Zambretti forecast
//           the timezone and the DST (Daylight Saving Time) is irrelevant. This is why I did not take care of DST 
//           in the code. I saw a fork on Github (truckershitch) which I believe has covered this.

#define NTP_SERVER      "us.pool.ntp.org"
#define TZ              -5           // (utc+) TZ in hours
#define DST_MN          60          // use 60mn for summer time in some countries

#define TZ_SEC          ((TZ)*3600)  // don't change this
#define DST_SEC         ((DST_MN)*60)// don't change this
//*************************auto update*****************
#define VERSION "v22.05.0901"
#define HOST "solarWeather"
const char* urlBase = "http://192.168.1.212:5000/update";

/**********Blynk & ThingSpeak assginments ---------------------------------

Blynk:

virtual pin 0 Temperature (Celcius)
virtual pin 1 Humidity (%)
virtual pin 2 Absolute Pressure (hPa)
virtual pin 3 Relative Pressure (hPa)
virtual pin 4 Battery Volts (V)
virtual pin 5 Dewpoint (Celcius)
virtual pin 6 HeatIndex (Celcius)
virtual pin 7 Zambrettis Words
virtual pin 8 Accuracy in percent (%)
virtual pin 9 Trend in Words
virtual pin 10 Dewpoint Spread

ThingSpeak:

Field 1: Relative Pressure (hPa)
Field 2: Temperature (Celcius)
Field 3: Humidity (%)
Field 4: Battery (V)
Field 5: Absolute Pressure (hPa)
Field 6: Dewpoint (Celcius)
Field 7: HeatIndex (Celcius) 
Status: Zambrettis Words + Trend in Words + Accuracy

MQTT:

home/weather/solarweatherstation/tempc
home/weather/solarweatherstation/heatindexc
home/weather/solarweatherstation/dewpointc
home/weather/solarweatherstation/spreadc
home/weather/solarweatherstation/abshpa
home/weather/solarweatherstation/relhpa
home/weather/solarweatherstation/humi
home/weather/solarweatherstation/battv
home/weather/solarweatherstation/zambrettisays
home/weather/solarweatherstation/trend
home/weather/solarweatherstation/accuracy

***************************************************************************/
