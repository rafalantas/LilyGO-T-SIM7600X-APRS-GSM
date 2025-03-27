//-------------------------------------------------------------------
// Modem & Debug Configuration
//-------------------------------------------------------------------
#define TINY_GSM_MODEM_SIM7600

#define UART_BAUD           115200

#define MODEM_TX            27
#define MODEM_RX            26
#define MODEM_PWRKEY        4
#define MODEM_DTR           32
#define MODEM_RI            33
#define MODEM_FLIGHT        25
#define MODEM_STATUS        34

#define LED_PIN             12

// See all AT commands, if wanted
 #define DUMP_AT_COMMANDS

#define SerialMon Serial
#define SerialAT Serial1

#define APRS_CALLSIGN "SQ4LOL"
#define APRS_SSID 19
#define APRS_PASSCODE "23475"
#define APRS_SYMBOL "/f"  // Van symbol

#include <TinyGsmClient.h>
#include <esp_adc_cal.h>

const char apn[] = "internet";
const char* aprsServer = "euro.aprs2.net";
const int aprsPort = 14580;

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem);

bool gps_fix = false;
unsigned long last_blink = 0;
const int blink_interval = 500; // 0.5 second blink interval

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


void setup() {
  SerialMon.begin(115200);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  SerialMon.println("Initializing modem...");
  
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_FLIGHT, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Turn on the modem
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(300);
  digitalWrite(MODEM_PWRKEY, LOW);

  // Disable flight mode
  digitalWrite(MODEM_FLIGHT, HIGH);

  // Wait for modem to respond
  while (!sendATCommand("AT", "OK", 1000)) {
    SerialMon.println("Waiting for modem...");
    delay(1000);
  }

  SerialMon.println("Modem is responsive");

  // Connect to network
  SerialMon.println("Connecting to network...");
  if (!modem.gprsConnect(apn, "", "")) {
    SerialMon.println("Failed to connect to network");
  } else {
    SerialMon.println("Connected to network");
  }
  // Enable GPS
  sendATCommand("AT+CGNSSPWR=1", "OK", 1000);
  
  // Set GPS mode (1 for GPS)
  sendATCommand("AT+CGNSSMODE=1", "OK", 1000);

  SerialMon.println("GPS enabled and configured");

}

void loop() {
  SerialMon.println("Requesting GPS info...");
  
  SerialAT.println("AT+CGNSSINFO");
  
  unsigned long startTime = millis();
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
  delay(1000); // Check GPS every second
}

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

void updateLastPosition(float lat, float lon, float speed, float course) {
  lastTransmitTime = millis();
  lastLat = lat;
  lastLon = lon;
  lastSpeed = speed;
  lastCourse = course;
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
float readBatteryVoltage() {
  int vref = 1100;
  int rawValue = analogRead(35);
  float voltage = ((float)rawValue / 4095.0) * 2.0 * 3.3 * (vref / 1000.0); // Assuming a voltage divider
  return voltage;
}

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
  snprintf(packet, sizeof(packet), "%s-%d>APRS,TCPIP*:!%s%c%c%s%c%c%03.0f/%03.0f/A=%06.0fRafal in Mazda CX5 QTH:Hajnowka",
           APRS_CALLSIGN, APRS_SSID, latStr, latHemi, APRS_SYMBOL[0], lonStr, lonHemi, APRS_SYMBOL[1],
           course, speed / 1.852, alt * 3.28084); // Convert speed back to knots, altitude to feet
  
  return String(packet);
}

String generateAPRSStatusPacket(int gpsSats, int glonassSats, int beidouSats) {
  char statusPacket[100];
  float batteryVoltage = readBatteryVoltage();
  String batteryStatus = (batteryVoltage == 0.00) ? "Plugged in" : String(batteryVoltage, 2) + "V";
  
  snprintf(statusPacket, sizeof(statusPacket), "%s-%d>APRS,TCPIP*:>Sats: GPS=%d GLONASS=%d BeiDou=%d Batt: %s",
           APRS_CALLSIGN, APRS_SSID, gpsSats, glonassSats, beidouSats, batteryStatus.c_str());
  return String(statusPacket);
}


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

void blinkLEDFast(int times) {
  bool lastLEDState = digitalRead(LED_PIN);
  for (int i = 0; i < times * 2; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(LED_PIN, lastLEDState);
}

bool sendATCommand(const char* command, const char* expected_answer, unsigned int timeout) {
  SerialAT.println(command);
  if (expected_answer == NULL) return true;
  unsigned long timer_start = millis();
  bool answer_detected = false;
  String response = "";
  
  while ((millis() - timer_start) < timeout) {
    if (SerialAT.available()) {
      char c = SerialAT.read();
      response += c;
      if (response.indexOf(expected_answer) != -1) {
        answer_detected = true;
        break;
      }
    }
  }
  
  SerialMon.println(response);
  return answer_detected;
}
