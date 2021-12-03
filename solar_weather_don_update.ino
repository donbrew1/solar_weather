
#include "Settings.h"
#include "Translation.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BlynkSimpleEsp8266.h>  //https://github.com/blynkkk/blynk-library
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "FS.h"
#include <EasyNTPClient.h>       //https://github.com/aharshac/EasyNTPClient
#include <Timelib.h>             //https://github.com/PaulStoffregen/Time.git

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

#define VERSION "v21.11.11241"
#define HOST "solarweather"
const char* urlBase = "http://192.168.1.207:5000/update";

const char* ssid = "Dhome";
const char* password = "TiffanyZ7";
uint8_t connection_state = 0;
uint16_t reconnect_interval = 10000;
WiFiClient  client;

uint8_t WiFiConnect(const char* nSSID = nullptr, const char* nPassword = nullptr)
{
  static uint16_t attempt = 0;
  Serial.print("Connecting to ");
  if (nSSID) {
    WiFi.begin(nSSID, nPassword);
    Serial.println(nSSID);
  }

  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 50)
  {
    delay(2000);
    Serial.print(".");
  }
  ++attempt;
  Serial.println("");
  if (i == 51) {
    Serial.print("Connection: TIMEOUT on attempt: ");
    Serial.println(attempt);
    if (attempt % 2 == 0)
      Serial.println("Check if access point available or SSID and Password\r\n");
    return false;
  }
  Serial.println("Connection: ESTABLISHED");
  Serial.print("Got IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

void Awaits()
{
  uint32_t ts = millis();
  while (!connection_state)
  {
    delay(500);
    if (millis() > (ts + reconnect_interval) && !connection_state) {
      connection_state = WiFiConnect();
      ts = millis();
    }
  }
}
//in setup:
// uint32_t startTime = millis();
//  connection_state = WiFiConnect(ssid, password);
//  if (!connection_state) // if not connected to WIFI
//    Awaits();          // constantly trying to connect 



void checkForUpdates(void)
{
  String checkUrl = String( urlBase);
  checkUrl.concat( "?ver=" + String(VERSION) );
  checkUrl.concat( "&dev=" + String(HOST) );

  Serial.println("INFO: Checking for updates at URL: " + String( checkUrl ) );
  
  t_httpUpdate_return ret = ESPhttpUpdate.update( checkUrl );

  switch (ret) {
    default:
    case HTTP_UPDATE_FAILED:
      Serial.println("ERROR: HTTP_UPDATE_FAILD Error (" + String(ESPhttpUpdate.getLastError()) + "): " + ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("INFO: HTTP_UPDATE_NO_UPDATES");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("INFO status: HTTP_UPDATE_OK");
      break;
    }
}



Adafruit_BME280 bme;             // I2C
WiFiUDP udp;
EasyNTPClient ntpClient(udp, NTP_SERVER, TZ_SEC + DST_SEC);

//
float measured_temp;
float measured_tempF;
float measured_humi;
float measured_pres;
float measured_presHg;
float adjusted_temp;
float adjusted_humi;
float SLpressure_hPa;               // needed for rel pressure calculation
float HeatIndex;  // Heat Index in °C
float HeatIndexF;
float volt;
int rel_pressure_rounded;
double DewpointTemperature;
double DewpointTemperatureF;
float DewPointSpread;  // Difference between actual temperature and dewpoint
float DewPointSpreadF;
int signal_dBM[] = { -100, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80, -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1};
int signal_percent[] = {0, 0, 0, 0, 0, 0, 4, 6, 8, 11, 13, 15, 17, 19, 21, 23, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45, 46, 48, 50, 52, 53, 55, 56, 58, 59, 61, 62, 64, 65, 67, 68, 69, 71, 72, 73, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 90, 91, 92, 93, 93, 94, 95, 95, 96, 96, 97, 97, 98, 98, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int strength = 0;
int percentage = 0;

// FORECAST CALCULATION
unsigned long current_timestamp;    // Actual timestamp read from NTPtime_t now;
unsigned long saved_timestamp;      // Timestamp stored in SPIFFS

float pressure_value[12];           // Array for the historical pressure values (6 hours, all 30 mins)
// where as pressure_value[0] is always the most recent value
float pressure_difference[12];      // Array to calculate trend with pressure differences

// FORECAST RESULT
int accuracy;                       // Counter, if enough values for accurate forecasting
String ZambrettisWords;             // Final statement about weather forecast
String trend_in_words;              // Trend in words
String forecast_in_words;           // Weather forecast in words
String pressure_in_words;           // Air pressure in words
String accuracy_in_words;           // Zambretti's prediction accuracy in words
void(* resetFunc) (void) = 0;       // declare reset function @ address 0

//WiFiClient client;

void setup() {

  Serial.begin(115200);
  Serial.println();
  Serial.println("Start of SolarWiFiWeatherStation V2.34");
//******Battery Voltage Monitoring*********************************************
  unsigned long raw = 0;
//  for (unsigned int i = 0; i < 10; i++) {
 //   raw = raw + analogRead(A0);
 raw = analogRead(A0);
    delay(1000);                              //ADC stable
//  }
  // Voltage divider R1 = 100k+220k =540k and R2=100k
  float calib_factor = 4.3
                       ; // change this value to calibrate the battery voltage

 // volt = (((raw / 10) * (calib_factor)) / 1023);
 volt = (raw * (calib_factor)) / 1023;
float batPower=map(raw,2931,4096,3,4.2);
  Serial.print( "Voltage = ");
  Serial.print(volt, 2); // print with 2 decimal places
  Serial.print (" V  ");
  Serial.print("Battery Power = ");
  Serial.print(batPower, 2);
  Serial.println("%");
 

  // **************Application going online**********************************
 uint32_t startTime = millis();
  connection_state = WiFiConnect(ssid, password);
  if (!connection_state) // if not connected to WIFI
    Awaits();          // constantly trying to connect 

  WiFi.mode(WIFI_STA);
  WiFi.hostname("SolarWeatherStation"); //This changes the hostname of the ESP8266 to display neatly on the network esp on router.
    checkForUpdates();


  //*****************Checking if SPIFFS available********************************

  Serial.println("SPIFFS Initialization: (First time run can last up to 30 sec - be patient)");

  boolean mounted = SPIFFS.begin();               // load config if it exists. Otherwise use defaults.
  if (!mounted) {
    Serial.println("FS not formatted. Doing that now... (can last up to 30 sec).");
    SPIFFS.format();
    Serial.println("FS formatted...");
    SPIFFS.begin();
  }

  //******** GETTING THE TIME FROM NTP SERVER  ***********************************

  Serial.println("---> Now reading time from NTP Server");
  int ii = 0;
  while (!ntpClient.getUnixTime()) {
    delay(100);
    ii++;
    if (ii > 20) {
      Serial.println("Could not connect to NTP Server!");
      Serial.println("Doing a reset now and retry a connection from scratch.");
      resetFunc();
    }
    Serial.print(".");
  }
  current_timestamp = ntpClient.getUnixTime();      // get UNIX timestamp (seconds from 1.1.1970 on)

  Serial.print("Current UNIX Timestamp: ");
  Serial.println(current_timestamp);

  Serial.print("Time & Date: ");
  Serial.print(hour(current_timestamp));
  Serial.print(":");
  Serial.print(minute(current_timestamp));
  Serial.print(":");
  Serial.print(second(current_timestamp));
  Serial.print("; ");
  Serial.print(day(current_timestamp));
  Serial.print(".");
  Serial.print(month(current_timestamp));         // needed later: month as integer for Zambretti calcualtion
  Serial.print(".");
  Serial.println(year(current_timestamp));

  //******** GETTING RELATIVE PRESSURE DATA FROM SENSOR (BME280)  ********************

  bool bme_status;
  bme_status = bme.begin(0x76);  //address either 0x76 or 0x77
  if (!bme_status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
  Serial.println("filter off");

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );

  measurementEvent();            //get all data from the different sensors

  //*******************SPIFFS operations***************************************************************

  ReadFromSPIFFS();              //read stored values and update data if more recent data is available

  Serial.print("Timestamp difference: ");
  Serial.println(current_timestamp - saved_timestamp);

  if (current_timestamp - saved_timestamp > 21600) {   // last save older than 6 hours -> re-initialize values
    FirstTimeRun();
  }
  else if (current_timestamp - saved_timestamp > 1800) { // it is time for pressure update (1800 sec = 30 min)

    for (int i = 11; i >= 1; i = i - 1) {
      pressure_value[i] = pressure_value[i - 1];        // shifting values one to the right
    }

    pressure_value[0] = rel_pressure_rounded;             // updating with acutal rel pressure (newest value)

    if (accuracy < 12) {
      accuracy = accuracy + 1;                            // one value more -> accuracy rises (up to 12 = 100%)
    }
    WriteToSPIFFS(current_timestamp);                   // update timestamp on storage
  }
  else {
    WriteToSPIFFS(saved_timestamp);                     // do not update timestamp on storage
  }

  //**************************Calculate Zambretti Forecast*******************************************

  int accuracy_in_percent = accuracy * 94 / 12;        // 94% is the max predicion accuracy of Zambretti
 // if ( volt > 3.4 ) {                                  // check if batt is still ok
    ZambrettisWords = ZambrettiSays(char(ZambrettiLetter()));
    forecast_in_words = TEXT_ZAMBRETTI_FORECAST;
    pressure_in_words = TEXT_AIR_PRESSURE;
    accuracy_in_words = TEXT_ZAMBRETTI_ACCURACY;
//  }
//  else {
//    ZambrettisWords = ZambrettiSays('0');              // send Message that battery is empty
//  }

  Serial.println("********************************************************");
  Serial.print(forecast_in_words);
  Serial.print(": ");
  Serial.println(ZambrettisWords);
  Serial.print(pressure_in_words);
  Serial.print(": ");
  Serial.println(trend_in_words);
  Serial.print(accuracy_in_words);
  Serial.print(": ");
  Serial.print(accuracy_in_percent);
  Serial.println("%");
  if (accuracy < 12) {
    Serial.println("Reason: Not enough weather data yet.");
    Serial.print("We need ");
    Serial.print((12 - accuracy) / 2);
    Serial.println(" hours more to get sufficient data.");
  }
  Serial.println("********************************************************");

  //**************************Sending Data to Blynk and ThingSpeak*********************************
  // code block for uploading data to BLYNK App

  if (App1 == "BLYNK") {
    Blynk.virtualWrite(0, adjusted_temp);            // virtual pin 0
    Blynk.virtualWrite(1, adjusted_humi);            // virtual pin 1
    Blynk.virtualWrite(2, measured_pres);            // virtual pin 2
    Blynk.virtualWrite(3, rel_pressure_rounded);     // virtual pin 3
    Blynk.virtualWrite(4, volt);                     // virtual pin 4
    Blynk.virtualWrite(5, DewpointTemperature);      // virtual pin 5
    Blynk.virtualWrite(6, HeatIndex);                // virtual pin 6
    Blynk.virtualWrite(7, ZambrettisWords);          // virtual pin 7
    Blynk.virtualWrite(8, accuracy_in_percent);      // virtual pin 8
    Blynk.virtualWrite(9, trend_in_words);           // virtual pin 9
    Blynk.virtualWrite(10, DewPointSpread);          // virtual pin 10
    Serial.println("Data written to Blink ...");
  }

  //*******************************************************************************
  // code block for uploading data to Thingspeak website

  if (App2 == "THINGSPEAK") {
    // Send data to ThingSpeak
    WiFiClient client;
    if (client.connect(server, 80)) {
      Serial.println("Connect to ThingSpeak - OK");

      String postStr = "";
      postStr += "GET /update?api_key=";
      postStr += api_key;
      //      postStr+="&field1=";
      //      postStr+=String(rel_pressure_rounded);
      postStr += "&field2=";
      postStr += String(measured_temp);
     postStr += String(adjusted_temp);
      postStr += "&field3=";
      postStr += String(measured_humi);
    // postStr += String(adjusted_humi);
      postStr += "&field4=";
      postStr += String(volt);
      postStr += "&field5=";
      postStr += String(measured_presHg);
      postStr += "&field6=";
      postStr += String(DewpointTemperatureF);
      postStr += "&field7=";
      postStr += String(HeatIndexF);
      postStr += "&field8=";
      postStr += String(strength);
      postStr += "&status=";
      postStr += String(forecast_in_words + ": " + ZambrettisWords + ". " + pressure_in_words + " " + trend_in_words + ". " + accuracy_in_words + " " + accuracy_in_percent + "%25."); // Percentage sign needs to be URL-encoded
      postStr += " HTTP/1.1\r\nHost: a.c.d\r\nConnection: close\r\n\r\n";
      postStr += "";
      client.print(postStr);
      Serial.println("Data written to Thingspeak ...");
    }
    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
  }
 //if (volt > 2.5) {          //check if batt still ok, if yes
  
   ESP.deepSleep(10 * 60e6); 
 // }
//  else{                      //if not,
//    ESP.deepSleep(0);            //hybernate because batt is empty
//  }


} // end of void setup()

void loop() {                //loop is not used
} // end of void loop()

void measurementEvent() {

  //Measures absolute Pressure, Temperature, Humidity, Voltage, calculate relative pressure,
  //Dewpoint, Dewpoint Spread, Heat Index

  bme.takeForcedMeasurement();
  // get RSSI
  strength = WiFi.RSSI();
  Serial.print("WiFi Strength");
  Serial.print(strength);
  Serial.print("dBm;");
  

  // Get temperature
  measured_temp = bme.readTemperature();
  measured_tempF = (1.8 * bme.readTemperature()) +32;
  // print on serial monitor
  Serial.print("Temp: ");
  Serial.print(measured_temp);
  Serial.print("°C; ");

  // Get humidity
  measured_humi = bme.readHumidity();
  // print on serial monitor
  Serial.print("Humidity: ");
  Serial.print(measured_humi);
  Serial.print("%; ");

  // Get pressure
  measured_pres = bme.readPressure() / 100.0F;
  measured_presHg = measured_pres / 33.8;
  // print on serial monitor
  Serial.print("Pressure: ");
  Serial.print(measured_pres);
  Serial.print("inHg; ");

  // Calculate and print relative pressure
  SLpressure_hPa = ((((measured_pres / .03) * 100.0) / pow((1 - ((float)(ELEVATION)) / 44330), 5.255)) / 100.0);
  rel_pressure_rounded = (int)(SLpressure_hPa + .5);
  // print on serial monitor
  Serial.print("Pressure rel: ");
  Serial.print(rel_pressure_rounded);
  Serial.print("hPa; ");

  // Calculate dewpoint C
  double a = 17.271;
  double b = 237.7;
  double tempcalc = (a * measured_temp) / (b + measured_temp) + log(measured_humi * 0.01);
  DewpointTemperature = (b * tempcalc) / (a - tempcalc);
  DewpointTemperatureF = (DewpointTemperature * 1.8) + 32;
  Serial.print("Dewpoint: ");
  Serial.print(DewpointTemperature);
  Serial.println("°F; ");

  // With the dewpoint calculated we can correct temp and automatically calculate humidity
  adjusted_temp = measured_temp + TEMP_CORR;
  if (adjusted_temp < DewpointTemperature) adjusted_temp = DewpointTemperature; //compensation, if offset too high
  //August-Roche-Magnus approximation (http://bmcnoldy.rsmas.miami.edu/Humidity.html)
  adjusted_humi = 100 * (exp((a * DewpointTemperature) / (b + DewpointTemperature)) / exp((a * adjusted_temp) / (b + adjusted_temp)));
  if (adjusted_humi > 100) adjusted_humi = 100;    // just in case
  // print on serial monitor
  Serial.print("Temp adjusted: ");
  Serial.print(adjusted_temp);
  Serial.print("°C; ");
  Serial.print("Humidity adjusted: ");
  Serial.print(adjusted_humi);
  Serial.print("%; ");

  // Calculate dewpoint spread (difference between actual temp and dewpoint -> the smaller the number: rain or fog

  DewPointSpread = measured_temp - DewpointTemperature;
  Serial.print("Dewpoint Spread: ");
  Serial.print(DewPointSpread);
  Serial.println("°F; ");

  // Calculate HI (heatindex in °C) --> HI starts working above 26,7 °C
  if (measured_tempF > 80.6) {
    double c1 = -8.784, c2 = 1.611, c3 = 2.338, c4 = -0.146, c5 = -1.230e-2, c6 = -1.642e-2, c7 = 2.211e-3, c8 = 7.254e-4, c9 = -2.582e-6  ; //C
    double d1 = -42.379, d2 = 2.049, d3 = 10.143, d4 = -0.225, d5 = -6.838e-3, d6 = -5.482e-2, d7 = 1.229e-3, d8 = 8.528e-4, d9 = -1.99e-6  ; // F
    double T = measured_temp;
    double R = measured_humi;
    double TF = measured_tempF;

    double A = (( c5 * T) + c2) * T + c1;
    double B = ((c7 * T) + c4) * T + c3;
    double C = ((c9 * T) + c8) * T + c6;
    HeatIndex = (C * R + B) * R + A;

    double D = (( d5 * TF) + d2) * TF + d1;
    double E = ((d7 * TF) + d4) * TF + d3;
    double F = ((d9 * TF) + d8) * TF + d6;
    HeatIndexF = (F * R + E) * R + D;
  }
  else {
    HeatIndexF = measured_tempF;
    Serial.println("Not warm enough (less than 80.6 °F) for Heatindex");
  }
  Serial.print("HeatIndex: ");
  Serial.print(HeatIndex);
  Serial.print("°F; ");
 
} // end of void measurementEvent()

int CalculateTrend() {
  int trend;                                    // -1 falling; 0 steady; 1 raising
  Serial.println("---> Calculating trend");

  //--> giving the most recent pressure reads more weight
  pressure_difference[0] = (pressure_value[0] - pressure_value[1])   * 1.5;
  pressure_difference[1] = (pressure_value[0] - pressure_value[2]);
  pressure_difference[2] = (pressure_value[0] - pressure_value[3])   / 1.5;
  pressure_difference[3] = (pressure_value[0] - pressure_value[4])   / 2;
  pressure_difference[4] = (pressure_value[0] - pressure_value[5])   / 2.5;
  pressure_difference[5] = (pressure_value[0] - pressure_value[6])   / 3;
  pressure_difference[6] = (pressure_value[0] - pressure_value[7])   / 3.5;
  pressure_difference[7] = (pressure_value[0] - pressure_value[8])   / 4;
  pressure_difference[8] = (pressure_value[0] - pressure_value[9])   / 4.5;
  pressure_difference[9] = (pressure_value[0] - pressure_value[10])  / 5;
  pressure_difference[10] = (pressure_value[0] - pressure_value[11]) / 5.5;

  //--> calculating the average and storing it into [11]
  pressure_difference[11] = (  pressure_difference[0]
                               + pressure_difference[1]
                               + pressure_difference[2]
                               + pressure_difference[3]
                               + pressure_difference[4]
                               + pressure_difference[5]
                               + pressure_difference[6]
                               + pressure_difference[7]
                               + pressure_difference[8]
                               + pressure_difference[9]
                               + pressure_difference[10]) / 11;

  Serial.print("Current trend: ");
  Serial.println(pressure_difference[11]);

  if      (pressure_difference[11] > 3.5) {
    trend_in_words = TEXT_RISING_FAST;
    trend = 1;
  }
  else if (pressure_difference[11] > 1.5   && pressure_difference[11] <= 3.5)  {
    trend_in_words = TEXT_RISING;
    trend = 1;
  }
  else if (pressure_difference[11] > 0.25  && pressure_difference[11] <= 1.5)  {
    trend_in_words = TEXT_RISING_SLOW;
    trend = 1;
  }
  else if (pressure_difference[11] > -0.25 && pressure_difference[11] < 0.25)  {
    trend_in_words = TEXT_STEADY;
    trend = 0;
  }
  else if (pressure_difference[11] >= -1.5 && pressure_difference[11] < -0.25) {
    trend_in_words = TEXT_FALLING_SLOW;
    trend = -1;
  }
  else if (pressure_difference[11] >= -3.5 && pressure_difference[11] < -1.5)  {
    trend_in_words = TEXT_FALLING;
    trend = -1;
  }
  else if (pressure_difference[11] <= -3.5) {
    trend_in_words = TEXT_FALLING_FAST;
    trend = -1;
  }

  Serial.println(trend_in_words);
  return trend;
}

char ZambrettiLetter() {
  Serial.println("---> Calculating Zambretti letter");
  char z_letter;
  int(z_trend) = CalculateTrend();
  // Case trend is falling
  if (z_trend == -1) {
    float zambretti = 0.0009746 * rel_pressure_rounded * rel_pressure_rounded - 2.1068 * rel_pressure_rounded + 1138.7019;
    //A Winter falling generally results in a Z value lower by 1 unit
    if (month(current_timestamp) < 4 || month(current_timestamp) > 9) zambretti = zambretti + 1;
    if (zambretti > 9) zambretti = 9;
    Serial.print("Calculated and rounded Zambretti in numbers: ");
    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'D'; break;       //Fine Becoming Less Settled
      case 4:  z_letter = 'H'; break;       //Fairly Fine Showers Later
      case 5:  z_letter = 'O'; break;       //Showery Becoming unsettled
      case 6:  z_letter = 'R'; break;       //Unsettled, Rain later
      case 7:  z_letter = 'U'; break;       //Rain at times, worse later
      case 8:  z_letter = 'V'; break;       //Rain at times, becoming very unsettled
      case 9:  z_letter = 'X'; break;       //Very Unsettled, Rain
    }
  }
  // Case trend is steady
  if (z_trend == 0) {
    float zambretti = 138.24 - 0.133 * rel_pressure_rounded;
    Serial.print("Calculated and rounded Zambretti in numbers: ");
    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'E'; break;       //Fine, Possibly showers
      case 4:  z_letter = 'K'; break;       //Fairly Fine, Showers likely
      case 5:  z_letter = 'N'; break;       //Showery Bright Intervals
      case 6:  z_letter = 'P'; break;       //Changeable some rain
      case 7:  z_letter = 'S'; break;       //Unsettled, rain at times
      case 8:  z_letter = 'W'; break;       //Rain at Frequent Intervals
      case 9:  z_letter = 'X'; break;       //Very Unsettled, Rain
      case 10: z_letter = 'Z'; break;       //Stormy, much rain
    }
  }
  // Case trend is rising
  if (z_trend == 1) {
    float zambretti = 142.57 - 0.1376 * rel_pressure_rounded;
    //A Summer rising, improves the prospects by 1 unit over a Winter rising
    if (month(current_timestamp) >= 4 && month(current_timestamp) <= 9) zambretti = zambretti - 1;
    if (zambretti < 0) zambretti = 0;
    Serial.print("Calculated and rounded Zambretti in numbers: ");
    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'C'; break;       //Becoming Fine
      case 4:  z_letter = 'F'; break;       //Fairly Fine, Improving
      case 5:  z_letter = 'G'; break;       //Fairly Fine, Possibly showers, early
      case 6:  z_letter = 'I'; break;       //Showery Early, Improving
      case 7:  z_letter = 'J'; break;       //Changeable, Improving
      case 8:  z_letter = 'L'; break;       //Rather Unsettled Clearing Later
      case 9:  z_letter = 'M'; break;       //Unsettled, Probably Improving
      case 10: z_letter = 'Q'; break;       //Unsettled, short fine Intervals
      case 11: z_letter = 'T'; break;       //Very Unsettled, Finer at times
      case 12: z_letter = 'Y'; break;       //Stormy, possibly improving
      case 13: z_letter = 'Z'; break;;      //Stormy, much rain
    }
  }
  Serial.print("This is Zambretti's famous letter: ");
  Serial.println(z_letter);
  return z_letter;
}

String ZambrettiSays(char code) {
  String zambrettis_words = "";
  switch (code) {
    case 'A': zambrettis_words = TEXT_ZAMBRETTI_A; break;  //see Tranlation.h
    case 'B': zambrettis_words = TEXT_ZAMBRETTI_B; break;
    case 'C': zambrettis_words = TEXT_ZAMBRETTI_C; break;
    case 'D': zambrettis_words = TEXT_ZAMBRETTI_D; break;
    case 'E': zambrettis_words = TEXT_ZAMBRETTI_E; break;
    case 'F': zambrettis_words = TEXT_ZAMBRETTI_F; break;
    case 'G': zambrettis_words = TEXT_ZAMBRETTI_G; break;
    case 'H': zambrettis_words = TEXT_ZAMBRETTI_H; break;
    case 'I': zambrettis_words = TEXT_ZAMBRETTI_I; break;
    case 'J': zambrettis_words = TEXT_ZAMBRETTI_J; break;
    case 'K': zambrettis_words = TEXT_ZAMBRETTI_K; break;
    case 'L': zambrettis_words = TEXT_ZAMBRETTI_L; break;
    case 'M': zambrettis_words = TEXT_ZAMBRETTI_M; break;
    case 'N': zambrettis_words = TEXT_ZAMBRETTI_N; break;
    case 'O': zambrettis_words = TEXT_ZAMBRETTI_O; break;
    case 'P': zambrettis_words = TEXT_ZAMBRETTI_P; break;
    case 'Q': zambrettis_words = TEXT_ZAMBRETTI_Q; break;
    case 'R': zambrettis_words = TEXT_ZAMBRETTI_R; break;
    case 'S': zambrettis_words = TEXT_ZAMBRETTI_S; break;
    case 'T': zambrettis_words = TEXT_ZAMBRETTI_T; break;
    case 'U': zambrettis_words = TEXT_ZAMBRETTI_U; break;
    case 'V': zambrettis_words = TEXT_ZAMBRETTI_V; break;
    case 'W': zambrettis_words = TEXT_ZAMBRETTI_W; break;
    case 'X': zambrettis_words = TEXT_ZAMBRETTI_X; break;
    case 'Y': zambrettis_words = TEXT_ZAMBRETTI_Y; break;
    case 'Z': zambrettis_words = TEXT_ZAMBRETTI_Z; break;
    case '0': zambrettis_words = TEXT_ZAMBRETTI_0; break;
    default: zambrettis_words = TEXT_ZAMBRETTI_DEFAULT; break;
  }
  return zambrettis_words;
}

void ReadFromSPIFFS() {
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "r");       // Open file for reading
  if (!myDataFile) {
    Serial.println("Failed to open file");
    FirstTimeRun();                                   // no file there -> initializing
  }

  Serial.println("---> Now reading from SPIFFS");

  String temp_data;

  temp_data = myDataFile.readStringUntil('\n');
  saved_timestamp = temp_data.toInt();
  Serial.print("Timestamp from SPIFFS: ");  Serial.println(saved_timestamp);

  temp_data = myDataFile.readStringUntil('\n');
  accuracy = temp_data.toInt();
  Serial.print("Accuracy value read from SPIFFS: ");  Serial.println(accuracy);

  Serial.print("Last 12 saved pressure values: ");
  for (int i = 0; i <= 11; i++) {
    temp_data = myDataFile.readStringUntil('\n');
    pressure_value[i] = temp_data.toInt();
    Serial.print(pressure_value[i]);
    Serial.print("; ");
  }
  myDataFile.close();
  Serial.println();
}

void WriteToSPIFFS(int write_timestamp) {
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w");        // Open file for writing (appending)
  if (!myDataFile) {
    Serial.println("Failed to open file");
  }

  Serial.println("---> Now writing to SPIFFS");

  myDataFile.println(write_timestamp);                 // Saving timestamp to /data.txt
  myDataFile.println(accuracy);                        // Saving accuracy value to /data.txt

  for ( int i = 0; i <= 11; i++) {
    myDataFile.println(pressure_value[i]);             // Filling pressure array with updated values
  }
  myDataFile.close();

  Serial.println("File written. Now reading file again.");
  myDataFile = SPIFFS.open(filename, "r");             // Open file for reading
  Serial.print("Found in /data.txt = ");
  while (myDataFile.available()) {
    Serial.print(myDataFile.readStringUntil('\n'));
    Serial.print("; ");
  }
  Serial.println();
  myDataFile.close();
}

void FirstTimeRun() {
  Serial.println("---> Starting initializing process.");
  accuracy = 1;
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w");            // Open a file for writing
  if (!myDataFile) {
    Serial.println("Failed to open file");
    Serial.println("Stopping process - maybe flash size not set (SPIFFS).");
    exit(0);
  }
  myDataFile.println(current_timestamp);                   // Saving timestamp to /data.txt
  myDataFile.println(accuracy);                            // Saving accuracy value to /data.txt
  for ( int i = 0; i < 12; i++) {
    myDataFile.println(rel_pressure_rounded);              // Filling pressure array with current pressure
  }
  Serial.println("** Saved initial pressure data. **");
  myDataFile.close();
  Serial.println("---> Doing a reset now.");
  resetFunc();                                              //call reset
}
