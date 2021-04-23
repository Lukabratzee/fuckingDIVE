#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoJson.h>
#include <ArduinoHttpClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <OneWire.h>
#include <DallasTemperature.h>

//DS18 - Temp
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress Thermometer = {0x28, 0x7D, 0x7B, 0x6A, 0x18, 0x20, 0x01, 0xAAE};

// sen0161 - pH
#define SensorPin 0 // the pH meter Analog output is connected with the Arduino’s Analog

// Configure the pins used for the ESP32 connection
#if defined(ADAFRUIT_FEATHER_M4_EXPRESS) ||   \
    defined(ADAFRUIT_FEATHER_M0_EXPRESS) ||   \
    defined(ADAFRUIT_FEATHER_M0) ||           \
    defined(ARDUINO_AVR_FEATHER32U4) ||       \
    defined(ARDUINO_NRF52840_FEATHER) ||      \
    defined(ADAFRUIT_ITSYBITSY_M0) ||         \
    defined(ADAFRUIT_ITSYBITSY_M4_EXPRESS) || \
    defined(ARDUINO_AVR_ITSYBITSY32U4_3V) ||  \
    defined(ARDUINO_NRF52_ITSYBITSY)
// Configure the pins used for the ESP32 connection
#define SPIWIFI SPI     // The SPI port
#define SPIWIFI_SS 13   // Chip select pin
#define ESP32_RESETN 12 // Reset pin
#define SPIWIFI_ACK 11  // a.k.a BUSY or READY pin
#define ESP32_GPIO0 -1
#elif defined(ARDUINO_AVR_FEATHER328P)
#define SPIWIFI SPI    // The SPI port
#define SPIWIFI_SS 4   // Chip select pin
#define ESP32_RESETN 3 // Reset pin
#define SPIWIFI_ACK 2  // a.k.a BUSY or READY pin
#define ESP32_GPIO0 -1
#elif defined(TEENSYDUINO)
#define SPIWIFI SPI    // The SPI port
#define SPIWIFI_SS 5   // Chip select pin
#define ESP32_RESETN 6 // Reset pin
#define SPIWIFI_ACK 9  // a.k.a BUSY or READY pin
#define ESP32_GPIO0 -1
#elif defined(ARDUINO_NRF52832_FEATHER)
#define SPIWIFI SPI     // The SPI port
#define SPIWIFI_SS 16   // Chip select pin
#define ESP32_RESETN 15 // Reset pin
#define SPIWIFI_ACK 7   // a.k.a BUSY or READY pin
#define ESP32_GPIO0 -1
#elif !defined(SPIWIFI_SS) // if the wifi definition isnt in the board variant
// Don't change the names of these #define's! they match the variant ones
#define SPIWIFI SPI
#define SPIWIFI_SS 10  // Chip select pin
#define SPIWIFI_ACK 7  // a.k.a BUSY or READY pin
#define ESP32_RESETN 5 // Reset pin
#define ESP32_GPIO0 -1 // Not connected
#endif

//#define SPIWIFI SPI
//#define SPIWIFI_SS 10  // Chip select pin
//#define SPIWIFI_ACK 7  // a.k.a BUSY or READY pin
//#define ESP32_RESETN 5 // Reset pin
//#define ESP32_GPIO0 -1 // Not connected
//#endif


#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)
char key[] = SECRET_KEY;   // X-Api-Key
int keyIndex = 0;          // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(74,125,232,128);  // numeric IP for Google (no DNS)

char server[] = "192.168.1.124"; // name address for adafruit test
String path = "/data";

char serverAddress[] = "192.168.1.124"; // server address
int port = 5000;
int temperature;
float volt;
int ntu;
float phValue;

WiFiClient wifi;
HttpClient client = HttpClient(wifi, serverAddress, port);

const long utcOffsetInSeconds = 3600;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String currentTime;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

void sen0161(float &phValue)
{
  unsigned long int avgValue; //Store the average value of the sensor feedback
  float b;
  int buf[10], temp;
  {
    for (int i = 0; i < 10; i++) //Get 10 sample value from the sensor for smooth the value
    {
      buf[i] = analogRead(SensorPin);
      delay(10);
    }
    for (int i = 0; i < 9; i++) //sort the analog from small to large
    {
      for (int j = i + 1; j < 10; j++)
      {
        if (buf[i] > buf[j])
        {
          temp = buf[i];
          buf[i] = buf[j];
          buf[j] = temp;
        }
      }
    }
    avgValue = 0;
    for (int i = 2; i < 8; i++) //take the average value of 6 center sample
      avgValue += buf[i];
    phValue = (float)avgValue * 5.0 / 1024 / 6; //convert the analog into millivolt
    phValue = 3.5 * phValue;                          //convert the millivolt into pH value
    Serial.print("pH: ");
    Serial.print(phValue, 1);
    Serial.println(" ");
  }
}

void sen0189(float &volt, int &ntu)
{

  int sensorPin = A0;

  volt = 0;
  for (int i = 0; i < 800; i++)
  {
    volt += ((float)analogRead(sensorPin) / 1023) * 5;
  }
  volt = volt / 800;
  volt = round_to_dp(volt, 2);
  if (volt < 0.4)
  {
    ntu = 3000;
    Serial.println("What the fuck? Did you put me in milk?");
  }
  else if (volt > 4.2)
  {
    ntu = 0;
    Serial.println("Ah, a refreshing beverage!");
  }
  else if (volt >= 2.1 && volt <= 2.2)
  {
    //ntu = 0;
    Serial.println("Nice brew!");
  }
  else
  {
    ntu = -1120.4 * sq(volt) + 5742.3 * volt - 4353.8;
  }

  Serial.println("Voltage: ");
  Serial.println(volt);
  Serial.println("NTU: ");
  Serial.println(ntu);
}

float round_to_dp(float in_value, int decimal_place)
{
  float multiplier = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplier) / multiplier;
  return in_value;
}


void ds18b20(int &temperature)
{
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus

  sensors.requestTemperatures(); // Send the command to get temperature readings
  Serial.print("Temperature is: ");
  Serial.print(sensors.getTempCByIndex(0), 1);
  temperature = sensors.getTempCByIndex(0);
  Serial.println("\n");
  
}

void sendJson(String &json)
{

  jsonBuild(json);

  String contentType = "application/json";
  client.beginRequest();
  client.post("/data");
  client.sendHeader(HTTP_HEADER_CONTENT_TYPE, "application/json");
  client.sendHeader(HTTP_HEADER_CONTENT_LENGTH, json.length());
  client.sendHeader("X-Api-Key", key);
  client.endRequest();
  client.write((const byte *)json.c_str(), json.length());
}

void jsonBuild(String &json)
{

  //JSON data, this is correct. I just need to send this to an IP
  
  StaticJsonDocument<128> doc;
  doc["X-Api-Key"] = key;
//  doc["date"] = currentTime;
  doc["sensor"] = "pH";
  doc["value"] = phValue;

  serializeJson(doc, Serial);

  serializeJson(doc, json);
  

 
}

void printWifiStatus()
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void setup()
{

  Serial.begin(9600);  
  while (!Serial)
    continue;

  // check for the WiFi module:
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
  while (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    delay(1000);
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  do
  {
    status = WiFi.begin(ssid, pass);
    delay(100); // wait until connection is ready!
  } while (status != WL_CONNECTED);

  Serial.println("Connected to wifi");
  printWifiStatus();
  timeClient.begin();
}

void loop()
{

    String json;
    int statusCode;
    String response;
    
  if (WiFi.status() == WL_CONNECTED)
  { //Check WiFi connection status

    timeClient.update();
    currentTime = timeClient.getFormattedDate();
    currentTime.replace("T", " ");
    currentTime.replace("Z", "");

    // JSON send
    sendJson(json);

    statusCode = client.responseStatusCode();
    //response = client.responseBody();

    Serial.print("POST Status code: ");
    
    if (statusCode != 200)
    {
      Serial.println("Error uploading");
      Serial.println(statusCode);

    }
    else if (statusCode == "Error Uploading")
    {
       Serial.println("Borky");
       pass;
    }
    
    else
    {
      Serial.println(statusCode);
      Serial.print("POST Response: ");
      Serial.println(response);
      Serial.print("Posted\n");
    }

    delay(60000);

    
  }
}
