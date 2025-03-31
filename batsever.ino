#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1
#define SerialMon Serial

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS

#include <TinyGsmClient.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  300         // Time ESP32 will go to sleep (in seconds)

#define UART_BAUD 115200

#define MODEM_TX 27
#define MODEM_RX 26
#define MODEM_PWRKEY 4
#define MODEM_DTR 32
#define MODEM_RI 33
#define MODEM_FLIGHT 25
#define MODEM_STATUS 34

#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCLK 14
#define SD_CS 13

#define LED_PIN 12
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem);

const char apn[] = "internet";
const char* aprsServer = "euro.aprs2.net";
const int aprsPort = 14580;

#define APRS_CALLSIGN "SQ4LOL"
#define APRS_SSID 9
#define APRS_PASSCODE "23475"
#define APRS_SYMBOL "/f"

bool gps_fix = false;
unsigned long last_blink = 0;
const int blink_interval = 500; // 0.5 second blink interval
int signalPercentage = -1;  // Stores GSM signal strength percentage

// SmartBeacon variables
float lastLat = 0, lastLon = 0;
float lastSpeed = 0, lastCourse = 0;
unsigned long lastTransmitTime = 0;
const float SB_LOW_SPEED = 5.0;     // km/h
const float SB_HIGH_SPEED = 70.0;   // km/h
const unsigned long SB_SLOW_RATE = 300000;  // 5 minutes in milliseconds
const unsigned long SB_FAST_RATE = 60000;   // 1 minute in milliseconds
const float SB_TURN_MIN_ANGLE = 30.0;       // degrees
const float SB_TURN_SLOPE = 255.0;          // deg*mph
const unsigned long SB_TURN_TIME = 15000;   // 15 seconds in milliseconds

// Store last position variables for position comparison
//RTC_DATA_ATTR float last_lat = 0;
//RTC_DATA_ATTR float last_lon = 0;
//RTC_DATA_ATTR float last_speed = 0;
//RTC_DATA_ATTR float last_course = 0;


// Function to send AT commands and wait for response
bool sendATCommand(String command, String expected_response, unsigned long timeout) {
  SerialAT.println(command);
  unsigned long start_time = millis();
  String response = "";
  
  while (millis() - start_time < timeout) {
    if (SerialAT.available()) {
      response += SerialAT.readString();
      if (response.indexOf(expected_response) >= 0) {
        return true;
      }
    }
  }
  return false;
}

// Function to read battery voltage
float readBatteryVoltage() {
  int vref = 1100;
  int rawValue = analogRead(35);
  float voltage = ((float)rawValue / 4095.0) * 2.0 * 3.3 * (vref / 1000.0); // Assuming a voltage divider
  return voltage;
}

float haversine(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371; // Earth's radius in km
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}
// Function to check signal strength
void checkSignalStrength() {
    SerialMon.println("Checking signal strength...");
    SerialAT.println("AT+CSQ"); // Send AT+CSQ command

    unsigned long startTime = millis();
    String response = "";

    // Wait for a response from the modem
    while (millis() - startTime < 2000) { // 2-second timeout
        if (SerialAT.available()) {
            response += SerialAT.readStringUntil('\n');
            if (response.indexOf("+CSQ:") >= 0) {
                break; // Exit loop once we find "+CSQ:"
            }
        }
    }

    // Parse response
    int rssi = -1; // Default value if parsing fails
    if (response.indexOf("+CSQ:") >= 0) {
        int startIdx = response.indexOf("+CSQ:") + 6;
        int endIdx = response.indexOf(",", startIdx);
        String rssiStr = response.substring(startIdx, endIdx);
        rssi = rssiStr.toInt(); // Convert RSSI string to integer
    }

    // Calculate percentage and update global variable
    if (rssi >= 0 && rssi <= 31) {
        signalPercentage = map(rssi, 0, 31, 0, 100); // Map RSSI value to percentage (0-100%)
        SerialMon.println("Signal Strength: " + String(signalPercentage) + "%");
    } else if (rssi == 99) {
        signalPercentage = -1; // Not detectable
        SerialMon.println("Signal Strength: Not detectable (No network available)");
    } else {
        signalPercentage = -1; // Failed to read signal strength
        SerialMon.println("Failed to read signal strength.");
    }
}

// Function to prepare for deep sleep
void prepareForDeepSleep() {
  SerialMon.println("Preparing for deep sleep...");
  
  // Turn off LED
  digitalWrite(LED_PIN, LOW);
  
  SerialMon.println("Enter modem sleep mode!");

  // Pull up DTR to put the modem into sleep
  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, HIGH);
  
  // Set DTR to keep at high level, if not set, DTR will be invalid after ESP32 goes to sleep
  gpio_hold_en((gpio_num_t)MODEM_DTR);
  gpio_deep_sleep_hold_en();

  if (modem.sleepEnable(true) != true) {
    SerialMon.println("Modem sleep failed!");
  } else {
    SerialMon.println("Modem entered sleep mode!");
  }

  // Wait to verify modem is sleeping
  delay(2000);
  SerialMon.println("Check modem response...");
  if (!modem.testAT()) {
    SerialMon.println("Modem is not responding, modem has entered sleep mode");
  }

  SerialMon.println("ESP32 going to deep sleep for " + String(TIME_TO_SLEEP) + " seconds");
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  delay(200);
  esp_deep_sleep_start();
}

void setup() {
  SerialMon.begin(115200);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(100);
  
  // Set up LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Check if waking from deep sleep
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER) {
    SerialMon.println("Cold boot, initializing modem...");
    
    // Initialize modem
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(100);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    
    // Enable flight mode control
    pinMode(MODEM_FLIGHT, OUTPUT);
    digitalWrite(MODEM_FLIGHT, HIGH);
  } else {
    SerialMon.println("Waking up from deep sleep!");
    
    // Need to cancel GPIO hold if wake from sleep
    gpio_hold_dis((gpio_num_t)MODEM_DTR);
    
    // Pull down DTR to wake up modem
    pinMode(MODEM_DTR, OUTPUT);
    digitalWrite(MODEM_DTR, LOW);
    delay(2000);
    modem.sleepEnable(false);
  }
  
  // Wait for modem to respond
  SerialMon.println("Checking if modem is online...");
  int attempts = 0;
  while (!modem.testAT() && attempts < 10) {
    SerialMon.print(".");
    delay(1000);
    attempts++;
  }
  
  if (attempts >= 10) {
    SerialMon.println("Failed to connect to modem, restarting...");
    ESP.restart();
  }
  
  SerialMon.println("Modem is online!");
  
  // Initialize GPS
  sendATCommand("AT+CGNSSPWR=1", "OK", 1000);
  sendATCommand("AT+CGNSSMODE=1", "OK", 1000);
  SerialMon.println("GPS enabled and configured");
  
  // Connect to network (add your APN details)
  SerialMon.println("Connecting to network...");
  if (!modem.gprsConnect(apn, "", "")) {
    SerialMon.println("Failed to connect to network");
  } else {
    SerialMon.println("Connected to network");
  }
}

// Parse GNSS info from AT command response
bool parseGNSSInfo(String info, float &lat, float &lon, float &alt, float &speed, float &course, int &gpsSats, int &glonassSats, int &beidouSats) {
  // Remove "+CGNSSINFO: " prefix and trim
  info = info.substring(11);
  info.trim();
  
  // Split the string into parts
  String parts[16];
  int partIndex = 0;
  int commaIndex = 0;
  int nextCommaIndex = info.indexOf(',');
  
  while (nextCommaIndex != -1 && partIndex < 16) {
    parts[partIndex++] = info.substring(commaIndex, nextCommaIndex);
    commaIndex = nextCommaIndex + 1;
    nextCommaIndex = info.indexOf(',', commaIndex);
  }
  if (partIndex < 16) {
    parts[partIndex] = info.substring(commaIndex);
  }

  SerialMon.println("Parsed GNSS Info:");
  SerialMon.println("Fix mode: " + parts[0] + (parts[0] == "2" ? " (2D fix)" : parts[0] == "3" ? " (3D fix)" : ""));
  SerialMon.println("GPS Satellites Used: " + parts[1]);
  SerialMon.println("GLONASS Satellites Used: " + parts[2]);
  SerialMon.println("BEIDOU Satellites Used: " + parts[3]);
  SerialMon.println("Latitude: " + parts[4] + " " + parts[5]);
  SerialMon.println("Longitude: " + parts[6] + " " + parts[7]);
  SerialMon.println("Date: " + parts[8]);
  SerialMon.println("UTC Time: " + parts[9]);
  SerialMon.println("Altitude: " + parts[10] + " meters");
  SerialMon.println("Speed: " + parts[11] + " knots");
  SerialMon.println("Course: " + parts[12] + " degrees");

  // Convert latitude and longitude to float
  lat = parts[4].toFloat();
  if (parts[5] == "S") lat = -lat;
  lon = parts[6].toFloat();
  if (parts[7] == "W") lon = -lon;
  
  alt = parts[10].toFloat();
  speed = parts[11].toFloat() * 1.852; // Convert knots to km/h
  course = parts[12].toFloat();

  gpsSats = parts[1].toInt();
  glonassSats = parts[2].toInt();
  beidouSats = parts[3].toInt();

  // Return true if we have a valid fix (2D or 3D)
  return (parts[0] == "2" || parts[0] == "3");
}

// Check if we should transmit based on distance/time
bool shouldTransmit(float lat, float lon, float speed, float course) {
  unsigned long currentTime = millis();
  float distance = haversine(lastLat, lastLon, lat, lon);
  unsigned long timeSinceLastTx = currentTime - lastTransmitTime;
  float courseDiff = abs(course - lastCourse);
  
  // Adjust course difference for 0/360 degree crossover
  if (courseDiff > 180) {
    courseDiff = 360 - courseDiff;
  }

  // Calculate the beacon rate based on speed
  unsigned long beaconRate;
  if (speed < SB_LOW_SPEED) {
    beaconRate = SB_SLOW_RATE;
  } else if (speed > SB_HIGH_SPEED) {
    beaconRate = SB_FAST_RATE;
  } else {
    // Linear interpolation between slow and fast rates
    float speedRatio = (speed - SB_LOW_SPEED) / (SB_HIGH_SPEED - SB_LOW_SPEED);
    beaconRate = SB_SLOW_RATE - speedRatio * (SB_SLOW_RATE - SB_FAST_RATE);
  }

  // Check if it's time for a beacon based on speed
  bool timeForSpeedBeacon = timeSinceLastTx >= beaconRate;

  // Calculate turn threshold
  float turnThreshold = SB_TURN_MIN_ANGLE;
  if (speed > 0) {
    turnThreshold += SB_TURN_SLOPE / speed;
  }

  // Check if it's time for a beacon based on course change
  bool timeForTurnBeacon = (courseDiff > turnThreshold) && (timeSinceLastTx >= SB_TURN_TIME);

  // Transmit if:
  // 1. It's time for a speed-based beacon
  // 2. There's been a significant course change and minimum turn time has elapsed
  // 3. Moved more than 1 km since last transmission
  return timeForSpeedBeacon || timeForTurnBeacon || (distance > 0.5);
}

// Generate APRS packet
String generateAPRSPacket(float lat, float lon, float alt, float speed, float course) {
  char packet[150];  // Increased buffer size to accommodate the comment
  char latStr[9], lonStr[10];
  
  // Convert lat/lon to APRS format
  int latDeg = (int)(lat / 100);
  float latMin = (lat - (latDeg * 100));
  int lonDeg = (int)(lon / 100);
  float lonMin = (lon - (lonDeg * 100));
  
  snprintf(latStr, sizeof(latStr), "%02d%05.2f", latDeg, latMin);
  snprintf(lonStr, sizeof(lonStr), "%03d%05.2f", lonDeg, lonMin);
  
  char latHemi = (lat >= 0) ? 'N' : 'S';
  char lonHemi = (lon >= 0) ? 'E' : 'W';
  
  // Generate APRS packet with comment
  snprintf(packet, sizeof(packet), "%s-%d>APRS,TCPIP*:!%s%c%c%s%c%c%03.0f/%03.0f/A=%06.0fRafal in Mazda QTH: Hajnowka",
           APRS_CALLSIGN, APRS_SSID, latStr, latHemi, APRS_SYMBOL[0], lonStr, lonHemi, APRS_SYMBOL[1],
           course, speed / 1.852, alt * 3.28084); // Convert speed back to knots, altitude to feet
  
  return String(packet);
}

// Generate APRS status packet
String generateAPRSStatusPacket(int gpsSats, int glonassSats, int beidouSats) {
  char statusPacket[150];
  float batteryVoltage = readBatteryVoltage();
  // Update signal strength before generating the packet
    checkSignalStrength(); // Call the function to update signalPercentage
  String batteryStatus = (batteryVoltage == 0.00) ? "Plugged in" : String(batteryVoltage, 2) + "V";
  
  // Format signal strength as a string
    String signalStatus = (signalPercentage >= 0) ? String(signalPercentage) + "%" : "Not detectable";

// Create the APRS status packet
    snprintf(statusPacket, sizeof(statusPacket),
             "%s-%d>APRS,TCPIP*:>Sats: GPS=%d GLONASS=%d BeiDou=%d Batt: %s Signal: %s",
             APRS_CALLSIGN, APRS_SSID, gpsSats, glonassSats, beidouSats, batteryStatus.c_str(), signalStatus.c_str());
  return String(statusPacket);
}

// Send APRS packet
bool sendAPRSPacket(String packet) {
  if (client.connect(aprsServer, aprsPort)) {
    SerialMon.println("Connected to APRS-IS server");
    
    // Send login
    client.print("user " APRS_CALLSIGN "-" + String(APRS_SSID) + " pass " APRS_PASSCODE " vers YourDeviceName 1.0\r\n");
    delay(1000);
    
    // Send packet
    client.println(packet);
    client.stop();
    return true;
  } else {
    SerialMon.println("Failed to connect to APRS-IS server");
    return false;
  }
}

// Update last position
void updateLastPosition(float lat, float lon, float speed, float course) {
  lastTransmitTime = millis();
  lastLat = lat;
  lastLon = lon;
  lastSpeed = speed;
  lastCourse = course;
}

// Blink LED fast
void blinkLEDFast(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH);
}

// Update LED based on GPS fix
void updateLED() {
  if (gps_fix) {
    // Steady LED when GPS has a fix
    digitalWrite(LED_PIN, HIGH);
  } else {
    // Blink LED when no GPS fix
    unsigned long current_time = millis();
    if (current_time - last_blink >= blink_interval) {
      last_blink = current_time;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
}

void loop() {
  SerialMon.println("Requesting GPS info...");
  SerialAT.println("AT+CGNSSINFO");
  
  unsigned long startTime = millis();
  bool packetSent = false;
//  bool gps_fix = false;
  
  while (millis() - startTime < 5000) {
    if (SerialAT.available()) {
      String response = SerialAT.readStringUntil('\n');
      SerialMon.println("Raw GPS data: " + response);
      
      if (response.startsWith("+CGNSSINFO:")) {
        float lat, lon, alt, speed, course;
        int gpsSats, glonassSats, beidouSats;
        
        if (parseGNSSInfo(response, lat, lon, alt, speed, course, gpsSats, glonassSats, beidouSats)) {
          gps_fix = true;
          
          if (shouldTransmit(lat, lon, speed, course)) {
            // Send position packet
            String aprsPacket = generateAPRSPacket(lat, lon, alt, speed, course);
            if (sendAPRSPacket(aprsPacket)) {
              SerialMon.println("APRS position packet sent successfully");
              blinkLEDFast(3);
              updateLastPosition(lat, lon, speed, course);
              packetSent = true;
              
              // Send status packet
              String statusPacket = generateAPRSStatusPacket(gpsSats, glonassSats, beidouSats);
              if (sendAPRSPacket(statusPacket)) {
                SerialMon.println("APRS status packet sent successfully");
                blinkLEDFast(2);
              } else {
                SerialMon.println("Failed to send APRS status packet");
              }
            } else {
              SerialMon.println("Failed to send APRS position packet");
            }
          }
        } else {
          gps_fix = false;
        }
        break;
      }
    }
  }
  
  updateLED();
  // Check battery voltage for deep sleep decision
  float batteryVoltage = readBatteryVoltage();
  
  // If we're on battery power and packet was sent, go to deep sleep
  if (batteryVoltage != 0.00 && packetSent) {
    SerialMon.println("On battery power (" + String(batteryVoltage, 2) + "V), going to deep sleep");
    prepareForDeepSleep();
  } else if (batteryVoltage != 0.00) {
    SerialMon.println("On battery power (" + String(batteryVoltage, 2) + "V), waiting for packet transmission");
  } else {
    SerialMon.println("Device is plugged in, continuing normal operation");
  }
  
  delay(1000); // Check GPS every second
}
