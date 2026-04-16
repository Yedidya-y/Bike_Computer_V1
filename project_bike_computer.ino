/*
 * ==========================================================
 * ==   ESP32 Smart Bike Computer                          ==
 * ==   Version: BLE + GPS + SD Card + GPX Analysis        ==
 * ==========================================================
 *
 * This project reads speed and cadence from BLE sensors,
 * reads GPS location and altitude, and reads a GPX route
 * from SD card to anticipate upcoming climbs and suggest
 * gear shifts accordingly.
 *
 * Industrial-style annotated version for education & portfolio
 */

// -----------------------------------------------------------------
// --- BLOCK 1: Libraries (Toolbox) ---
// -----------------------------------------------------------------
#include <Wire.h>              // I2C communication for OLED display
#include <Adafruit_GFX.h>      // Graphics library (shapes, text)
#include <Adafruit_SSD1306.h>  // OLED display driver

#include <BLEDevice.h>         // BLE device main library
#include <BLEUtils.h>          
#include <BLEScan.h>           
#include <BLEClient.h>         
#include <BLERemoteCharacteristic.h>

#include <SPI.h>               // SPI communication (SD card)
#include <SD.h>                // SD card management
#include <TinyGPS++.h>         // GPS parsing library

// -----------------------------------------------------------------
// --- BLOCK 2: Hardware Definitions & Constants ---
// -----------------------------------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Pins for GPS & SD card
#define SD_CS_PIN 5            // Chip Select for SD
#define GPS_RX_PIN 16          // RX pin ESP32 listens to GPS
#define GPS_TX_PIN 17          // TX pin ESP32 sends to GPS

static const uint32_t GPSBaud = 9600; // GPS serial baud rate

// BLE addresses of sensors (Updated with your actual MACs!)
static const char* speedSensorAddressStr   = "fa:77:64:cc:c4:fb"; 
static const char* cadenceSensorAddressStr = "ce:30:ae:74:5e:32"; 

// Standard CSC service UUIDs (Cycling Speed & Cadence)
static BLEUUID serviceUUID("00001816-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_CSC("00002a5b-0000-1000-8000-00805f9b34fb");

// -----------------------------------------------------------------
// --- BLOCK 3: Global Variables (System Memory) ---
// -----------------------------------------------------------------
TinyGPSPlus gps;               // GPS parsing object
HardwareSerial gpsSerial(1);   // Serial1 dedicated for GPS

volatile float crankRPM = 0.0;   // Pedal rotations per minute
volatile float wheelRPM = 0.0;   // Wheel rotations per minute
volatile float gearRatio = 0.0;  // Calculated gear ratio

// BLE connection flags
static bool connectedSpeed = false;
static bool connectedCadence = false;
static BLERemoteCharacteristic* pRemoteCharSpeed = nullptr;
static BLERemoteCharacteristic* pRemoteCharCadence = nullptr;

// Previous data storage (for delta calculations)
uint32_t prevWheelRevCount = 0;
uint16_t prevWheelEventTime = 0;
bool hasPrevWheel = false;

uint16_t prevCrankRevCount = 0;
uint16_t prevCrankEventTime = 0;
bool hasPrevCrank = false;

// GPX-related variables
volatile float currentAltitude = 0.0; 
String gearAlert = ""; 
unsigned long alertTimestamp = 0;

// Timing variables for non-blocking operation
unsigned long lastDisplayUpdate = 0;
unsigned long lastScanTime = 0;
unsigned long lastGpxCheck = 0;

const unsigned long DISPLAY_INTERVAL_MS = 200;   // refresh 5x/sec
const unsigned long SCAN_INTERVAL_MS = 5000;     // BLE scan every 5s
const unsigned long GPX_CHECK_INTERVAL_MS = 10000; // GPX check every 10s

// -----------------------------------------------------------------
// --- BLOCK 4: Helper Functions ---
// -----------------------------------------------------------------

// Wrap-around safe difference for 32-bit counters
inline uint32_t diff32(uint32_t newv, uint32_t oldv) {
  if (newv >= oldv) return newv - oldv;
  return (uint32_t)((uint64_t)newv + (uint64_t)(UINT32_MAX - oldv) + 1ULL);
}

// Wrap-around safe difference for 16-bit counters
inline uint32_t diff16(uint16_t newv, uint16_t oldv) {
  if (newv >= oldv) return (uint32_t)(newv - oldv);
  return (uint32_t)((uint32_t)newv + (uint32_t)(UINT16_MAX - oldv) + 1U);
}

// Parse GPX values from line
float parseValue(String line, String tag) {
  int startIndex = line.indexOf(tag) + tag.length();
  int endIndex = line.indexOf("\"", startIndex);
  return line.substring(startIndex, endIndex).toFloat();
}

// Parse <ele> (elevation) from GPX line
float parseElevation(String line) {
  int startIndex = line.indexOf("<ele>") + 5;
  int endIndex = line.indexOf("</ele>", startIndex);
  return line.substring(startIndex, endIndex).toFloat();
}

// -----------------------------------------------------------------
// --- BLOCK 5: BLE Core (Connection & Data Parsing) ---
// -----------------------------------------------------------------

// Main BLE notification callback
static void cscNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                              uint8_t* pData, size_t length, bool isNotify) {
  if (length < 1) return; // Protection: ignore corrupted packet

  uint8_t idx = 0;
  uint8_t flags = pData[idx++];
  bool wheelPresent = flags & 0x01;
  bool crankPresent = flags & 0x02;

  // --- Wheel data ---
  if (wheelPresent) {
    if (idx + 6 <= length) {
      uint32_t cumulativeWheelRevs = (uint32_t)pData[idx] | ((uint32_t)pData[idx+1] << 8) |
                                     ((uint32_t)pData[idx+2] << 16) | ((uint32_t)pData[idx+3] << 24);
      idx += 4;
      uint16_t lastWheelEventTime = (uint16_t)pData[idx] | ((uint16_t)pData[idx+1] << 8);
      idx += 2;

      if (hasPrevWheel) {
        uint32_t deltaRevs = diff32(cumulativeWheelRevs, prevWheelRevCount);
        uint32_t deltaEventTime = diff16(lastWheelEventTime, prevWheelEventTime);

        if (deltaEventTime > 0) {
          float deltaTimeSec = (float)deltaEventTime / 1024.0f;
          wheelRPM = ((float)deltaRevs / deltaTimeSec) * 60.0f; // update global
        }
      }
      prevWheelRevCount = cumulativeWheelRevs;
      prevWheelEventTime = lastWheelEventTime;
      hasPrevWheel = true;
    } else return;
  }

  // --- Crank (pedal) data ---
  if (crankPresent) {
    if (idx + 4 <= length) {
      uint16_t cumulativeCrankRevs = (uint16_t)pData[idx] | ((uint16_t)pData[idx+1] << 8);
      idx += 2;
      uint16_t lastCrankEventTime = (uint16_t)pData[idx] | ((uint16_t)pData[idx+1] << 8);
      idx += 2;

      if (hasPrevCrank) {
        uint32_t deltaRevs = diff16(cumulativeCrankRevs, prevCrankRevCount);
        uint32_t deltaEventTime = diff16(lastCrankEventTime, prevCrankEventTime);

        if (deltaEventTime > 0) {
          float deltaTimeSec = (float)deltaEventTime / 1024.0f;
          crankRPM = ((float)deltaRevs / deltaTimeSec) * 60.0f; // update global
        }
      }
      prevCrankRevCount = cumulativeCrankRevs;
      prevCrankEventTime = lastCrankEventTime;
      hasPrevCrank = true;
    } else return;
  }
}

// BLE client callbacks
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("Client connected (BLE).");
  }
  void onDisconnect(BLEClient* pclient) {
    Serial.println("Client disconnected (BLE).");
  }
};

// Connect to a specific BLE sensor
bool connectToSensor(const char* addressStr, BLERemoteCharacteristic** outChar, bool isSpeed) {
  BLEAddress addr(addressStr);
  Serial.print("Trying to connect to: "); Serial.println(addressStr);

  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(addr)) {
    Serial.println("Failed to connect to device.");
    pClient->disconnect(); delete pClient; return false;
  }

  BLERemoteService* pRemoteService = nullptr;
  try { pRemoteService = pClient->getService(serviceUUID); } catch(...) { pRemoteService = nullptr; }
  if (!pRemoteService) { pClient->disconnect(); delete pClient; return false; }

  BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
  try { pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID_CSC); } catch(...) { pRemoteCharacteristic = nullptr; }
  if (!pRemoteCharacteristic) { pClient->disconnect(); delete pClient; return false; }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(cscNotifyCallback);

  *outChar = pRemoteCharacteristic;
  return true;
}

// -----------------------------------------------------------------
// --- BLOCK 6: GPS / GPX Core ---
// -----------------------------------------------------------------

void readGps() {
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
  if (gps.altitude.isUpdated()) currentAltitude = gps.altitude.meters();
}

// Returns estimated altitude change in next 100 meters
float getUpcomingSlope() {
  if (!gps.location.isValid() || !gps.location.isUpdated()) return 0.0;

  float myLat = gps.location.lat();
  float myLon = gps.location.lng();

  File gpxFile = SD.open("/route.gpx");
  if (!gpxFile) return 0.0;

  bool foundStartPoint = false;
  float startLat, startLon, startAlt;
  float currentLat, currentLon, currentAlt;
  float distanceAccumulated = 0.0;

  while (gpxFile.available()) {
    String line = gpxFile.readStringUntil('\n');

    if (line.indexOf("<trkpt") != -1) {
      currentLat = parseValue(line, "lat=\"");
      currentLon = parseValue(line, "lon=\"");

      line = gpxFile.readStringUntil('\n'); 
      if (line.indexOf("<ele>") != -1) currentAlt = parseElevation(line);

      if (!foundStartPoint) {
        float distanceToPoint = TinyGPSPlus::distanceBetween(myLat, myLon, currentLat, currentLon);
        if (distanceToPoint < 25) { // within 25m
          foundStartPoint = true;
          startLat = currentLat; startLon = currentLon; startAlt = currentAlt;
        }
      } else {
        distanceAccumulated += TinyGPSPlus::distanceBetween(startLat, startLon, currentLat, currentLon);
        startLat = currentLat; startLon = currentLon;
        if (distanceAccumulated >= 100.0) { 
          gpxFile.close();
          return currentAlt - startAlt;
        }
      }
    }
  }

  gpxFile.close();
  return 0.0;
}

// -----------------------------------------------------------------
// --- BLOCK 7: Setup ---
// -----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Note: Since OLED is not connected yet, it will fail silently here, which is fine.
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESP32 BLE Bike Computer");
  display.println("Initializing...");
  display.display();

  BLEDevice::init("ESP32 Bike Computer");
  gpsSerial.begin(GPSBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (!SD.begin(SD_CS_PIN)) {
    display.println("SD Failed");
    Serial.println("SD Not Connected - This is expected right now."); 
  } else {
    display.println("SD OK");
  }

  lastDisplayUpdate = millis();
  lastScanTime = millis();
  lastGpxCheck = millis() + 5000;
  display.display();
}

// -----------------------------------------------------------------
// --- BLOCK 8: Main Loop ---
// -----------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // 1) GPS reading
  readGps();

  // 2) BLE connection management
  if (!connectedSpeed || !connectedCadence) {
    if (now - lastScanTime > SCAN_INTERVAL_MS) {
      lastScanTime = now;
      if (!connectedSpeed)
        connectedSpeed = connectToSensor(speedSensorAddressStr, &pRemoteCharSpeed, true);
      if (!connectedCadence)
        connectedCadence = connectToSensor(cadenceSensorAddressStr, &pRemoteCharCadence, false);
    }
  }

  // 3) GPX slope analysis (low frequency)
  if (now - lastGpxCheck > GPX_CHECK_INTERVAL_MS) {
    lastGpxCheck = now;
    float altitudeChange = getUpcomingSlope();
    if (altitudeChange > 5.0 && crankRPM > 70.0) {
      gearAlert = "SHIFT DOWN!";
      alertTimestamp = millis();
    }
  }

  // 4) Gear ratio calculation
  gearRatio = (crankRPM > 1e-6 && wheelRPM > 1e-6) ? wheelRPM / crankRPM : 0.0f;

  // 5) Display update
  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = now;
    
    // --- Added for Serial Monitor Testing (Since OLED isn't connected yet) ---
    Serial.printf("BLE: %s/%s | Speed RPM: %.1f | Cadence RPM: %.1f | Gear: %.2f\n", 
                  connectedSpeed ? "OK" : "-", 
                  connectedCadence ? "OK" : "-", 
                  wheelRPM, crankRPM, gearRatio);
    // ------------------------------------------------------------------------

    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("BLE:%s%s GPS:%s SD:%s\n",
                   connectedSpeed ? "S" : "-",
                   connectedCadence ? "C" : "-",
                   gps.location.isValid() ? "OK" : "NO",
                   SD.begin(SD_CS_PIN) ? "OK" : "NO");

    display.printf("Crank RPM: %.1f\n", crankRPM);
    display.printf("Wheel RPM: %.1f\n", wheelRPM);
    display.printf("Gear Ratio: %.2f\n", gearRatio);

    if (now - alertTimestamp <= 3000 && gearAlert.length() > 0)
      display.printf("%s\n", gearAlert.c_str());

    display.display();
  }
}
