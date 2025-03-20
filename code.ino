//-------------------------------------------------------------------
// Modem & Debug Configuration
//-------------------------------------------------------------------
#define TINY_GSM_MODEM_SIM7600

#include <SPI.h>
#include <SD.h>
#include <Ticker.h>
#include <TinyGsmClient.h>

// SerialMon for debug output (USB Serial)
// SerialAT for modem communication (Serial1)
#define SerialMon Serial
#define SerialAT  Serial1

#define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG SerialMon

// APN & APRS configuration
const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";
const char* aprsServer = "euro.aprs2.net";
const int aprsPort = 14580;
const char* aprsCallsign = "SQ4LOL-19";    // Replace with your callsign
const char* aprsPasscode = "23475";         // Replace with your APRS passcode

#ifndef DBG
#define DBG(x,y) do { SerialMon.print(x); SerialMon.println(y); } while(0)
#endif

//-------------------------------------------------------------------
// Pin Definitions
//-------------------------------------------------------------------
#define UART_BAUD        115200
#define MODEM_TX         27
#define MODEM_RX         26
#define MODEM_PWRKEY     4
#define MODEM_DTR        32
#define MODEM_RI         33
#define MODEM_FLIGHT     25
#define MODEM_STATUS     34

// SD card pins (if used)
#define SD_MISO          2
#define SD_MOSI          15
#define SD_SCLK          14
#define SD_CS            13

// LED pin for status indication: it blinks when no GPS fix and remains solid when fixed.
#define LED_PIN          12

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// APRS communication client using the modem
TinyGsmClient client(modem);

//-------------------------------------------------------------------
// SmartBeaconing Parameters (for APRS)
//-------------------------------------------------------------------
const unsigned long MIN_INTERVAL = 60000;  // Fast beacon (e.g., when moving fast, 1 minute)
const unsigned long MAX_INTERVAL = 300000;  // Slow beacon (e.g., when moving slow, 300 seconds)
const float TURN_THRESHOLD = 30.0;         // Degrees change that triggers beaconing
const float DISTANCE_THRESHOLD = 0.5;      // km traveled for triggering beacon
const float SPEED_THRESHOLD = 5.0;        // km/h below which slow interval is used
const float MAX_BEACON_SPEED = 70.0;         // Maximum speed cap for dynamic interval

unsigned long lastTransmitTime = 0;
float lastLat = 0;
float lastLon = 0;
float lastCourse = 0;

// Satellite counts from GNSS info
int gps_sats = 0;
int glonass_sats = 0;
int beidou_sats = 0;

//-------------------------------------------------------------------
// Battery Voltage & ADC Configuration
//-------------------------------------------------------------------
#define BATTERY_ADC_PIN   35
#define ADC_REF_VOLTAGE   3.3     // Reference voltage (volts)
#define ADC_MAX           4095.0  // 12-bit ADC maximum
#define BATTERY_DIVIDER   2.0     // Voltage divider factor (adjust as required)

//-------------------------------------------------------------------
// LED Blink Configuration (when no GPS fix)
//-------------------------------------------------------------------
#define LED_BLINK_INTERVAL 500   // in milliseconds
unsigned long lastBlinkTime = 0;
bool ledState = false;

//-------------------------------------------------------------------
// Deep Sleep Configuration
//-------------------------------------------------------------------
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  3600        // Time ESP32 will go to sleep (in seconds) - 1 hour
#define BATTERY_THRESHOLD 3.2      // Voltage threshold for low battery

//-------------------------------------------------------------------
// Function Declarations
//-------------------------------------------------------------------
float getBatteryVoltage();
void enableAGPS();
float computeBearing(float lat1, float lon1, float lat2, float lon2);
float haversine(float lat1, float lon1, float lat2, float lon2);
unsigned long getDynamicInterval(float speed);
void getGNSSInfo();
void sendAPRSPacket(float lat, float lon, float speed, float alt, float course, int totalSats);
void blinkLED();
void enterDeepSleep();
void initializeSerial();
void powerOnModem();
void initializeModem();
void connectToNetwork();
void enableGNSS();
void checkWakeupReason();
void setup();
void loop();

//-------------------------------------------------------------------
// Function: getBatteryVoltage()
// Reads the analog value from BATTERY_ADC_PIN and converts it to voltage.
// If the value is near zero then it is assumed the board is externally powered (charging).
//-------------------------------------------------------------------
float getBatteryVoltage() {
  int raw = analogRead(BATTERY_ADC_PIN);
  float voltage = raw * (ADC_REF_VOLTAGE / ADC_MAX) * BATTERY_DIVIDER;
  return voltage;
}

//-------------------------------------------------------------------
// Function: enableAGPS()
// Requests AGPS assistance data with the AT+CGPSXE command.
//-------------------------------------------------------------------
void enableAGPS() {
  SerialMon.println("Requesting AGPS data...");
  String response = "";
  modem.sendAT("+CGPSXE=1");
  if (modem.waitResponse(10000L, response) == 1) {
    SerialMon.println("AGPS data retrieved successfully");
  } else {
    SerialMon.println("Failed to retrieve AGPS data.");
  }
}

//-------------------------------------------------------------------
// Function: computeBearing()
// Computes and returns the bearing (in degrees, 0-360) from (lat1, lon1) to (lat2, lon2).
//-------------------------------------------------------------------
float computeBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float brng = atan2(y, x);
  brng = degrees(brng);
  if (brng < 0)
    brng += 360;
  return brng;
}

//-------------------------------------------------------------------
// Function: haversine()
// Returns the great-circle distance (in km) between two coordinates using the Haversine formula.
//-------------------------------------------------------------------
float haversine(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371.0;  // Earth's radius in km
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

//-------------------------------------------------------------------
// Function: getDynamicInterval()
// Returns a dynamic interval (in ms) for beaconing based on the current speed.
//-------------------------------------------------------------------
unsigned long getDynamicInterval(float speed) {
  if (speed < SPEED_THRESHOLD)
    return MAX_INTERVAL;
  if (speed > MAX_BEACON_SPEED)
    speed = MAX_BEACON_SPEED;
  unsigned long interval = MAX_INTERVAL - (unsigned long)(
      ((speed - SPEED_THRESHOLD) / (MAX_BEACON_SPEED - SPEED_THRESHOLD)) * (MAX_INTERVAL - MIN_INTERVAL));
  if (interval < MIN_INTERVAL)
    interval = MIN_INTERVAL;
  return interval;
}

//-------------------------------------------------------------------
// Function: getGNSSInfo()
// Uses the AT+CGNSSINFO command to retrieve satellite info from the modem.
//-------------------------------------------------------------------
void getGNSSInfo() {
  String response = "";
  modem.sendAT("+CGNSSINFO");
  if (modem.waitResponse(10000L, response) == 1) {
    // Check if the response contains real data
    if (response.indexOf("+CGNSSINFO: ,,,,,,,,,,,,,,,") != -1) {
      SerialMon.println("No GNSS fix or empty data");
      return;
    }

    int colonIndex = response.indexOf(':');
    if (colonIndex != -1) {
      response = response.substring(colonIndex + 1);
      response.trim();  // Remove leading/trailing whitespace

      if (sscanf(response.c_str(), "%*d,%d,%d,%d", &gps_sats, &glonass_sats, &beidou_sats) == 3) {
        SerialMon.print("GNSS info: GPS=");
        SerialMon.print(gps_sats);
        SerialMon.print(" GLONASS=");
        SerialMon.print(glonass_sats);
        SerialMon.print(" BeiDou=");
        SerialMon.println(beidou_sats);
      } else {
        SerialMon.println("GNSS info parsing error");
      }
    } else {
      SerialMon.println("GNSS info format error");
    }
  } else {
    SerialMon.println("No GNSS info response");
  }
}

//-------------------------------------------------------------------
// Function: sendAPRSPacket()
// Constructs and sends an APRS packet. The position packet contains
// latitude, longitude, speed, altitude and course.
// The status packet includes satellite counts and battery status.
// If the battery voltage is nearly 0 (e.g., < 0.01V), the string "CHARGING" is sent.
//-------------------------------------------------------------------
void sendAPRSPacket(float lat, float lon, float speed, float alt, float course, int totalSats) {
  char positionPacket[150];
  snprintf(positionPacket, sizeof(positionPacket),
           "%s>APRS,TCPIP*:!%02d%05.2f%c/%03d%05.2f%cf%03d/%03d/A=%06d Rafal in Mazda CX5 QTH:Hajnowka",
           aprsCallsign,
           (int)abs(lat),
           (abs(lat) - (int)abs(lat)) * 60,
           (lat >= 0) ? 'N' : 'S',
           (int)abs(lon),
           (abs(lon) - (int)abs(lon)) * 60,
           (lon >= 0) ? 'E' : 'W',
           (int)course,
           (int)(speed * 1.852),   // Convert speed if needed
           (int)(alt * 3.28084));   // Convert altitude from meters to feet

  // Determine battery status string
  float battVoltage = getBatteryVoltage();
  char batteryStatus[16];  // Buffer for battery status string
  if (battVoltage < 0.01) {
    snprintf(batteryStatus, sizeof(batteryStatus), "PLUGGED IN");
  } else {
    snprintf(batteryStatus, sizeof(batteryStatus), "%.2fV", battVoltage);
  }

  char statusPacket[150];
  snprintf(statusPacket, sizeof(statusPacket),
           "%s>APRS,TCPIP*:>Sats: GPS=%d GLONASS=%d BeiDou=%d Batt=%s",
           aprsCallsign, gps_sats, glonass_sats, beidou_sats, batteryStatus);

  SerialMon.print("Connecting to ");
  SerialMon.print(aprsServer);
  if (!client.connect(aprsServer, aprsPort)) {
    SerialMon.println(" fail");
    return;
  }
  SerialMon.println(" success");

  client.print("user ");
  client.print(aprsCallsign);
  client.print(" pass ");
  client.print(aprsPasscode);
  client.print(" vers TinyGSM 0.1\r\n");

  client.println(positionPacket);
  SerialMon.print("APRS position packet sent: ");
  SerialMon.println(positionPacket);

  client.println(statusPacket);
  SerialMon.print("APRS status packet sent: ");
  SerialMon.println(statusPacket);

  client.stop();
}

//-------------------------------------------------------------------
// Function: blinkLED()
// Toggles the LED each LED_BLINK_INTERVAL ms when no GPS fix is acquired.
//-------------------------------------------------------------------
void blinkLED() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastBlinkTime >= LED_BLINK_INTERVAL) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    lastBlinkTime = currentMillis;
  }
}

//-------------------------------------------------------------------
// Function: enterDeepSleep()
// Powers off the modem and puts the ESP32 into deep sleep mode.
//-------------------------------------------------------------------
void enterDeepSleep() {
  SerialMon.println("Entering deep sleep...");
  modem.poweroff();  // Power off the modem
  delay(1000);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

//-------------------------------------------------------------------
// Function: initializeSerial()
// Initializes the serial communication.
//-------------------------------------------------------------------
void initializeSerial() {
  SerialMon.begin(115200);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

//-------------------------------------------------------------------
// Function: powerOnModem()
// Powers on the modem by toggling the power key.
//-------------------------------------------------------------------
void powerOnModem() {
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(300);
  digitalWrite(MODEM_PWRKEY, LOW);
  pinMode(MODEM_FLIGHT, OUTPUT);
  digitalWrite(MODEM_FLIGHT, HIGH);
}

//-------------------------------------------------------------------
// Function: initializeModem()
// Initializes the modem and waits for response to AT commands.
//-------------------------------------------------------------------
void initializeModem() {
  SerialMon.println("Starting modem...");
  delay(3000);
  while (!modem.testAT()) {
    delay(10);
  }
  bool ret = modem.setNetworkMode(2);
  DBG("setNetworkMode:", ret);
}

//-------------------------------------------------------------------
// Function: connectToNetwork()
// Connects to the network and APN.
//-------------------------------------------------------------------
void connectToNetwork() {
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
  enableAGPS();
}

//-------------------------------------------------------------------
// Function: enableGNSS()
// Enables GNSS and waits for initialization.
//-------------------------------------------------------------------
void enableGNSS() {
  modem.setGNSSMode(1, 1);
  delay(1000);
  DBG("Enabling GPS/GNSS/GLONASS", true);
  modem.enableGPS();
  delay(2000);
}

//-------------------------------------------------------------------
// Function: checkWakeupReason()
// Checks the reason for waking up from deep sleep and logs it.
//-------------------------------------------------------------------
void checkWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    SerialMon.println("Woken up from deep sleep");
  }
}

//-------------------------------------------------------------------
// Function: setup()
// Initializes serial communication, powers up the modem, waits for network,
// connects to the APN, requests AGPS data, and enables GNSS.
//-------------------------------------------------------------------
void setup() {
  initializeSerial();
  powerOnModem();
  initializeModem();
  connectToNetwork();
  enableGNSS();
  checkWakeupReason();
}

//-------------------------------------------------------------------
// Function: loop()
// Attempts to get a GPS fix. When a fix is acquired, it processes beacon conditions,
// sends APRS packets (with updated battery/satellite data) and prints battery voltage.
// If no GPS fix is available, the LED blinks and it retries after a delay.
//-------------------------------------------------------------------
void loop() {
  float lat = 0, lon = 0, speed = 0, alt = 0;
  int vsat = 0, usat = 0;
  float accuracy = 0;
  int year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;

  float batteryVoltage = getBatteryVoltage();
  SerialMon.print("Battery Voltage: ");
  SerialMon.print(batteryVoltage, 2);
  SerialMon.println("V");

  if (batteryVoltage > 0.01 && batteryVoltage < BATTERY_THRESHOLD) {
    SerialMon.println("Low battery detected. Entering deep sleep.");
    enterDeepSleep();
    return;
  }

  DBG("Requesting current GPS/GNSS location", true);

  if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
                   &year, &month, &day, &hour, &min, &sec)) {
    if (lat != 0 && lon != 0) {
      digitalWrite(LED_PIN, HIGH);

      float course = 0;
      if (lastLat != 0 || lastLon != 0)
        course = computeBearing(lastLat, lastLon, lat, lon);

      getGNSSInfo();

      unsigned long currentTime = millis();
      float distanceTraveled = haversine(lastLat, lastLon, lat, lon);
      float courseChange = fabs(course - lastCourse);
      unsigned long dynamicInterval = getDynamicInterval(speed);
      bool timeTrigger = (currentTime - lastTransmitTime >= dynamicInterval);
      bool distanceTrigger = (distanceTraveled >= DISTANCE_THRESHOLD);
      bool turnTrigger = (courseChange >= TURN_THRESHOLD);
      bool isStationary = (speed < 1.0);

      if ((timeTrigger && !isStationary) || distanceTrigger || turnTrigger) {
        int totalSats = gps_sats + glonass_sats + beidou_sats;
        sendAPRSPacket(lat, lon, speed, alt, course, totalSats);
        lastTransmitTime = currentTime;
        lastLat = lat;
        lastLon = lon;
        if (turnTrigger)
          lastCourse = course;
      } else if (isStationary && (currentTime - lastTransmitTime >= MAX_INTERVAL)) {
        int totalSats = gps_sats + glonass_sats + beidou_sats;
        sendAPRSPacket(lat, lon, speed, alt, course, totalSats);
        lastTransmitTime = currentTime;
      }

      if (batteryVoltage < 0.01) {
        SerialMon.println("Battery voltage: PLUGGED IN");
      } else {
        SerialMon.print("Battery voltage: ");
        SerialMon.print(batteryVoltage, 2);
        SerialMon.println(" V");
      }
    } else {
      SerialMon.println("Invalid GPS position (0,0). Skipping.");
    }
  } else {
    blinkLED();
    SerialMon.println("Failed to get GPS fix");

    if (batteryVoltage < 0.01) {
      SerialMon.println("Battery voltage: PLUGGED IN");
    } else {
      SerialMon.print("Battery voltage: ");
      SerialMon.print(batteryVoltage, 2);
      SerialMon.println(" V");
    }
  }
  delay(1000);
}
