#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <vector>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>
#include "FS.h" // For SPIFFS
#include "SPIFFS.h"

// --- Configuration ---
// These will be loaded from SPIFFS, defaults provided here as fallback
const char* BUS_ID_DEFAULT = "B000-EDIT-ME"; // Default if config fails or file is placeholder
String BUS_ID = BUS_ID_DEFAULT; // Will be overwritten by config file
const char* CONFIG_FILE_PATH_BUS = "/config_bus.json";

// FIXME: GPS - Define the UART pins for your GPS module connection if not using default Serial2 pins.
// Most ESP32 boards have Serial2 on GPIO16 (RX2) and GPIO17 (TX2).
HardwareSerial GPSSerial(2); // Use UART2 for GPS
const unsigned long GPS_BAUD = 9600; // Common baud rate for GPS modules
const unsigned long GPS_PROCESS_INTERVAL_MS = 1000; // How often to process GPS data
const unsigned long WATCHDOG_TIMEOUT_S_BUS = 25;    // Watchdog timeout

// BLE Advertising (Optional)
#define BLE_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // Example UUID
#define BLE_BUS_NODE_NAME_PREFIX "AkitaBus-" // Prefix, will append BUS_ID
bool ENABLE_BLE_ADVERTISING = true; // Default, will be overwritten by config file

// Meshtastic Serial Communication
HardwareSerial& MeshtasticSerial = Serial1; // Using ESP32's UART1
const long MESHTASTIC_BAUD = 115200;      // Must match Meshtastic firmware Serial API config
// FIXME: MESH PIN - If Serial1 pins are not default for your board, define them here:
// Example: #define MESHTASTIC_SERIAL_RX_PIN GPIO_NUM_9
// Example: #define MESHTASTIC_SERIAL_TX_PIN GPIO_NUM_10
// And then use MeshtasticSerial.begin(MESHTASTIC_BAUD, SERIAL_8N1, MESHTASTIC_SERIAL_RX_PIN, MESHTASTIC_SERIAL_TX_PIN);

const size_t JSON_DOC_CONFIG_BUS_SIZE = 1024; // For reading config_bus.json (adjust if many geofences)
const size_t JSON_DOC_OUTPUT_SIZE_BUS = 256; // For sending commands to Meshtastic

// Geofence Definitions (will be loaded from SPIFFS)
struct GeofenceStop {
  int stopId;
  double lat;
  double lon;
  float radiusMeters;
};
std::vector<GeofenceStop> stopGeofences;
int currentStopId = 0; // 0 means not currently at any known stop
int lastReportedStopId = -1; // To avoid re-sending for the same stop repeatedly

// --- Global Objects ---
TinyGPSPlus gps;
BLEAdvertising *pAdvertising = nullptr;

// --- Forward Declarations ---
bool loadConfigurationBus();
bool createDefaultBusConfigFile(); // New
void feedWatchdogBus();
bool setupGPS();
void processGPS();
bool setupBLEBus();
void startBLEAdvertisingBus();
bool setupMeshtasticSerialBus();
bool sendMeshtasticJsonCommandBus(const JsonObject& commandJson);
bool sendMeshtasticTextBus(const String& textPayload, const String& destination = "^all");
void reportBusAtStop(int stopId);

// --- Main Setup ---
void setup() {
  Serial.begin(115200);
  // while(!Serial && millis() < 3000); // Optional wait for serial

  // Initialize SPIFFS once at the beginning
  Serial.println("Bus Node: Mounting SPIFFS...");
  if (!SPIFFS.begin(true)) { // true = format SPIFFS if mount failed
      Serial.println("CRITICAL ERROR: Bus Node SPIFFS Mount Failed. Configuration cannot be loaded/created. Halting.");
      // Consider a visual error indication if possible before halting (e.g., onboard LED)
      while(1) { delay(1000); } // Halt
  } else {
      Serial.println("Bus Node SPIFFS mounted successfully.");
  }
  feedWatchdogBus();


  if (!loadConfigurationBus()) {
      Serial.println("ERROR: Bus Node failed to load configuration from SPIFFS after attempt to load/create. Using defaults. Functionality will be SEVERELY LIMITED.");
      BUS_ID = BUS_ID_DEFAULT; // Ensure default ID is used
      stopGeofences.clear();   // No geofences loaded
      ENABLE_BLE_ADVERTISING = true; // Fallback BLE state
  }
  feedWatchdogBus();

  Serial.printf("\n--- Akita Bus Node ID: %s Initializing ---\n", BUS_ID.c_str());
  if (stopGeofences.empty()) {
    Serial.println("WARNING: No geofences loaded from config. GPS stop detection will NOT function.");
  }
  if (BUS_ID == BUS_ID_DEFAULT || BUS_ID == "B000-EDIT-ME"){ // Check against the exact default placeholder
    Serial.println("WARNING: Bus ID is default. Please edit /config_bus.json via SPIFFS!");
  }

  Serial.println("Initializing Watchdog...");
  if (esp_task_wdt_init(WATCHDOG_TIMEOUT_S_BUS, true) != ESP_OK) Serial.println("ERROR: WDT init fail");
  else if (esp_task_wdt_add(NULL) != ESP_OK) Serial.println("ERROR: WDT add task fail");
  feedWatchdogBus();

  if (!setupGPS()) Serial.println("WARNING: GPS setup failed. Check wiring/module. Geofencing disabled.");
  feedWatchdogBus();

  if (ENABLE_BLE_ADVERTISING) {
    if (!setupBLEBus()) Serial.println("WARNING: BLE Advertising setup failed.");
    else startBLEAdvertisingBus();
  } else {
    Serial.println("INFO: BLE Advertising disabled by configuration.");
  }
  feedWatchdogBus();

  if (!setupMeshtasticSerialBus()) Serial.println("WARNING: Meshtastic Serial setup failed. Mesh communication disabled.");
  feedWatchdogBus();

  Serial.println("Bus Node Setup Complete.");
}

// --- Main Loop ---
void loop() {
  feedWatchdogBus();
  static unsigned long lastGpsProcessTime = 0;
  if (millis() - lastGpsProcessTime > GPS_PROCESS_INTERVAL_MS) {
    processGPS(); // Reads GPS serial, parses, and checks geofences
    lastGpsProcessTime = millis();
  }
  feedWatchdogBus();
  // Other tasks for the bus node can go here
  delay(10); // Yield for other tasks
}

// --- Function Implementations ---
void feedWatchdogBus() { esp_task_wdt_reset(); }

bool createDefaultBusConfigFile() {
    Serial.printf("Attempting to create default Bus Node configuration file: %s\n", CONFIG_FILE_PATH_BUS);
    StaticJsonDocument<JSON_DOC_CONFIG_BUS_SIZE> doc;
    doc["bus_id"] = BUS_ID_DEFAULT; // User MUST change this
    doc["enable_ble_advertising"] = true;

    JsonArray geofences = doc.createNestedArray("geofences");
    JsonObject fence1 = geofences.createNestedObject();
    fence1["stop_id"] = 1;
    fence1["lat"] = 42.886400; // Example Lat (Port Colborne City Hall approx)
    fence1["lon"] = -79.249500; // Example Lon
    fence1["radius_m"] = 30.0;

    JsonObject fence2 = geofences.createNestedObject();
    fence2["stop_id"] = 2;
    fence2["lat"] = 42.882000; // Example Lat (HH Knoll Park approx)
    fence2["lon"] = -79.255000; // Example Lon
    fence2["radius_m"] = 35.0;

    File configFile = SPIFFS.open(CONFIG_FILE_PATH_BUS, "w");
    if (!configFile) {
        Serial.printf("ERROR: Failed to create/open default bus config file '%s' for writing.\n", CONFIG_FILE_PATH_BUS);
        return false;
    }
    if (serializeJson(doc, configFile) == 0) {
        Serial.println("ERROR: Failed to write JSON to default bus config file.");
        configFile.close();
        return false;
    }
    configFile.close();
    Serial.println("Default config_bus.json created. PLEASE EDIT IT with correct Bus ID and Geofence data via SPIFFS upload, then REBOOT device!");
    return true;
}


bool loadConfigurationBus() {
    // Assumes SPIFFS.begin() was called successfully in setup()
    if (!SPIFFS.exists(CONFIG_FILE_PATH_BUS)) {
        Serial.printf("Bus Node Config file %s not found. Attempting to create default.\n", CONFIG_FILE_PATH_BUS);
        if (!createDefaultBusConfigFile()) {
            Serial.println("ERROR: Failed to create default bus config. Using hardcoded defaults.");
            BUS_ID = BUS_ID_DEFAULT;
            stopGeofences.clear();
            ENABLE_BLE_ADVERTISING = true; // Default
            return false;
        }
        // After creating, inform user to reboot after editing.
        // For this run, it will effectively use defaults.
        Serial.println("Default bus config file created. Please edit and reboot.");
        return false; // Indicate that it wasn't loaded from an existing user-edited file this pass.
    }

    File configFile = SPIFFS.open(CONFIG_FILE_PATH_BUS, "r");
    if (!configFile) {
        Serial.printf("ERROR: Failed to open Bus Node config file %s for reading.\n", CONFIG_FILE_PATH_BUS);
        return false;
    }

    Serial.printf("Reading Bus Node configuration from %s\n", CONFIG_FILE_PATH_BUS);
    StaticJsonDocument<JSON_DOC_CONFIG_BUS_SIZE> doc;
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close(); // Close file immediately
    feedWatchdogBus();

    if (error) {
        Serial.print("ERROR: Failed to parse Bus Node config file: "); Serial.println(error.c_str());
        return false;
    }

    // Load BUS_ID
    if (doc.containsKey("bus_id") && doc["bus_id"].is<const char*>()) {
        BUS_ID = doc["bus_id"].as<String>();
        Serial.printf("Loaded Bus ID: %s\n", BUS_ID.c_str());
        if (BUS_ID == BUS_ID_DEFAULT || BUS_ID == "B000-EDIT-ME") { // Check against the exact default placeholder
            Serial.println("WARNING: Bus ID is still default. Please edit config_bus.json!");
        }
    } else {
        Serial.println("Warning: 'bus_id' not found or invalid in bus config. Using default.");
        BUS_ID = BUS_ID_DEFAULT;
    }

    // Load Geofences
    stopGeofences.clear(); // Clear any defaults or previous loads
    if (doc.containsKey("geofences") && doc["geofences"].is<JsonArray>()) {
        JsonArray geofenceArray = doc["geofences"].as<JsonArray>();
        Serial.printf("Loading %d geofences from config...\n", geofenceArray.size());
        for (JsonObject fenceJson : geofenceArray) {
            // Robustly check if all keys exist and are of the correct type
            if (fenceJson.containsKey("stop_id") && fenceJson["stop_id"].is<int>() &&
                fenceJson.containsKey("lat") && fenceJson["lat"].is<double>() &&
                fenceJson.containsKey("lon") && fenceJson["lon"].is<double>() &&
                fenceJson.containsKey("radius_m") && fenceJson["radius_m"].is<float>()) {

                GeofenceStop stop;
                stop.stopId = fenceJson["stop_id"].as<int>();
                stop.lat = fenceJson["lat"].as<double>();
                stop.lon = fenceJson["lon"].as<double>();
                stop.radiusMeters = fenceJson["radius_m"].as<float>();

                if (stop.stopId > 0 && stop.radiusMeters > 0) { // Basic validation for sensible values
                    stopGeofences.push_back(stop);
                } else {
                    Serial.println("Warning: Invalid geofence entry data skipped (stop_id/radius <= 0).");
                }
            } else {
                Serial.println("Warning: Incomplete or malformed geofence entry in config (missing keys or wrong types).");
            }
        }
        Serial.printf("Successfully loaded %d geofences into table.\n", stopGeofences.size());
    } else {
        Serial.println("Warning: 'geofences' array not found or invalid in bus config. No geofences loaded.");
    }

    // Load BLE Advertising preference
    if (doc.containsKey("enable_ble_advertising") && doc["enable_ble_advertising"].is<bool>()) {
        ENABLE_BLE_ADVERTISING = doc["enable_ble_advertising"].as<bool>();
    } else {
        ENABLE_BLE_ADVERTISING = true; // Default to true if not specified in config
    }
    Serial.printf("BLE Advertising: %s\n", ENABLE_BLE_ADVERTISING ? "Enabled" : "Disabled");

    Serial.println("Bus Node configuration loaded successfully.");
    return true;
}


bool setupGPS() {
  Serial.printf("Initializing GPS on UART2 at %lu baud...\n", GPS_BAUD);
  // FIXME: GPS - If using non-default pins for Serial2, pass them here:
  // GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  GPSSerial.begin(GPS_BAUD);
  if (!GPSSerial) { Serial.println("ERROR: GPSSerial (UART2) failed to begin!"); return false; }
  Serial.println("GPS Serial initialized. Waiting for data from GPS module...");
  return true;
}

void processGPS() {
  bool gpsDataReceivedThisCycle = false;
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
    gpsDataReceivedThisCycle = true;
  }

  // Check for a valid and recent GPS fix with reasonable accuracy
  if (gps.location.isUpdated() && gps.location.isValid() &&
      gps.satellites.isValid() && gps.satellites.value() >= 3 && // Need at least 3 sats for 2D, 4 for 3D
      gps.hdop.isValid() && gps.hdop.value() <= 5.0) { // Horizontal Dilution of Precision (lower is better)

    double currentLat = gps.location.lat();
    double currentLon = gps.location.lng();
    Serial.printf("GPS Fix: Lat=%.6f, Lon=%.6f, Age=%lums, Sats=%d, HDOP=%.2f\n",
                  currentLat, currentLon, gps.location.age(), gps.satellites.value(), gps.hdop.value());
    feedWatchdogBus(); // Good data, feed watchdog

    int previousStopId = currentStopId; // Store the stop ID from the last valid check
    currentStopId = 0; // Reset, assume not in any geofence for this check

    if (stopGeofences.empty()) {
        // Serial.println("No geofences defined, cannot determine current stop."); // Can be noisy
    } else {
        for (const auto& stop : stopGeofences) {
          double distanceToStopCenter = TinyGPSPlus::distanceBetween(
              currentLat, currentLon,
              stop.lat, stop.lon
          );

          if (distanceToStopCenter <= stop.radiusMeters) {
            currentStopId = stop.stopId;
            Serial.printf("Bus is INSIDE geofence for Stop %d (Dist: %.1fm)\n", currentStopId, distanceToStopCenter);
            break; // Assume bus can only be in one stop geofence at a time
          }
        }
    }

    // Report only if the stop has changed since last report, or if it's a new valid stop
    if (currentStopId != 0 && currentStopId != lastReportedStopId) {
      reportBusAtStop(currentStopId);
      lastReportedStopId = currentStopId;
    } else if (currentStopId == 0 && previousStopId != 0) {
      // Bus has left a geofence and is not in another known one
      Serial.printf("Bus has LEFT geofence of Stop %d and is now in transit.\n", previousStopId);
      lastReportedStopId = 0; // Reset so it can report entering any stop again
      // Optionally send a "bus in transit" message
      // String transitPayload = String("!BUS:") + BUS_ID + ":TRANSIT:" + String(previousStopId);
      // sendMeshtasticTextBus(transitPayload);
    }

  } else { // No valid fix or poor quality fix this cycle
    if (gpsDataReceivedThisCycle) { // If we got data but it wasn't good enough for a fix
        if (!gps.location.isValid()) Serial.println("GPS: Location data not valid this cycle.");
        else if (!gps.satellites.isValid() || gps.satellites.value() < 3) Serial.printf("GPS: Insufficient satellites for fix: %d\n", gps.satellites.isValid() ? gps.satellites.value() : 0);
        else if (!gps.hdop.isValid() || gps.hdop.value() > 5.0) Serial.printf("GPS: Poor HDOP for fix: %.2f\n", gps.hdop.isValid() ? gps.hdop.value() : 99.0);
    }
    // currentStopId remains unchanged from the last valid determination if no new valid fix
  }
}

void reportBusAtStop(int stopId) {
  Serial.printf("Reporting Bus %s AT Stop %d via Meshtastic.\n", BUS_ID.c_str(), stopId);
  String payload = String("!BUS:") + BUS_ID + ":" + String(stopId);
  if (!sendMeshtasticTextBus(payload)) {
      Serial.println("ERROR: Failed to send bus location update to Meshtastic.");
      // Consider retry logic or flagging an error state for the bus node
  }
}

bool setupBLEBus() {
  if (!ENABLE_BLE_ADVERTISING) {
    Serial.println("INFO: Bus BLE Advertising is disabled by configuration.");
    return true; // Successfully "setup" as disabled
  }
  String bleName = String(BLE_BUS_NODE_NAME_PREFIX) + BUS_ID;
  Serial.printf("Initializing BLE Device for Advertising as %s...\n", bleName.c_str());
  BLEDevice::init(bleName.c_str()); // Initialize with the dynamic name

  BLEServer *pServer = BLEDevice::createServer();
  if (!pServer) { Serial.println("ERROR: Failed to create BLE Server on Bus!"); return false; }

  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  if (!pService) { Serial.println("ERROR: Failed to create BLE Service on Bus!"); return false; }
  // No characteristics needed just for advertising the service UUID

  pAdvertising = BLEDevice::getAdvertising();
  if (!pAdvertising) { Serial.println("ERROR: Failed to get Advertising object on Bus!"); return false; }
  Serial.println("Bus BLE Setup Complete.");
  return true;
}

void startBLEAdvertisingBus() {
  if (!ENABLE_BLE_ADVERTISING || !pAdvertising) return; // Don't start if disabled or not setup

  BLEAdvertisementData advertData;
  advertData.setFlags(0x06); // GENERAL_DISC_MODE | BR_EDR_NOT_SUPPORTED
  advertData.addServiceUUID(BLE_SERVICE_UUID);
  // advertData.setName(String(BLE_BUS_NODE_NAME_PREFIX) + BUS_ID); // Set name in advertisement data if desired (can make packet larger)
  pAdvertising->setAdvertisementData(advertData);
  // pAdvertising->setScanResponseData(scanResponseData); // Optional scan response
  pAdvertising->setMinPreferred(0x06); // Advertising interval hint for iOS
  pAdvertising->setMaxPreferred(0x12);

  Serial.println("Starting Bus BLE Advertising...");
  BLEDevice::startAdvertising(); // This is non-blocking
  Serial.println("Bus BLE Advertising started.");
}

bool setupMeshtasticSerialBus() {
  Serial.printf("Initializing Meshtastic Serial on UART1 at %ld baud for Bus Node...\n", MESHTASTIC_BAUD);
  // FIXME: MESH PIN - If using non-default pins for Serial1, pass them here:
  // MeshtasticSerial.begin(MESHTASTIC_BAUD, SERIAL_8N1, MESHTASTIC_SERIAL_RX_PIN, MESHTASTIC_SERIAL_TX_PIN);
  MeshtasticSerial.begin(MESHTASTIC_BAUD);
  delay(100); // Allow serial to initialize
  if (!MeshtasticSerial) { Serial.println("ERROR: MeshtasticSerial (UART1) failed to begin on Bus Node!"); return false; }
  Serial.println("Bus Node Meshtastic Serial initialized. Ensure Meshtastic firmware is configured for this UART, baud, and JSON output.");
  return true;
}

bool sendMeshtasticJsonCommandBus(const JsonObject& commandJson) {
    if (!MeshtasticSerial) { Serial.println("ERROR: Bus Node Meshtastic Serial port not ready!"); return false; }
    Serial.print("Bus Node Sending to Meshtastic: "); serializeJsonPretty(commandJson, Serial); Serial.println(); // Debug
    serializeJson(commandJson, MeshtasticSerial); // Send compact JSON
    MeshtasticSerial.println(); // Meshtastic Serial API usually expects newline terminated commands
    feedWatchdogBus();
    // FIXME: MESH - Consider implementing logic to wait for an ACK/NACK from Meshtastic if the command supports it and reliability is paramount.
    return true; // Assume sent for now
}

bool sendMeshtasticTextBus(const String& textPayload, const String& destination /* = "^all" */) {
    StaticJsonDocument<JSON_DOC_OUTPUT_SIZE_BUS> doc;
    // FIXME: MESH - CRITICAL: Verify this JSON structure for sending text with your Meshtastic firmware's Serial API version.
    // This is a common pattern for interacting with a Meshtastic daemon/Python API,
    // the direct device Serial API might require a different or more "raw" packet structure.
    // Common options for direct Serial API:
    // 1. A simple `{"sendtext":"your_payload_here"}` if the API has a direct passthrough.
    // 2. A more structured command like `{"set_router_message":"base64_encoded_protobuf_here"}`.
    // 3. A command to send a pre-formed packet: `{"send_packet":{"to":"^all", "decoded": {"portnum":"TEXT_MESSAGE_APP", "payload":"your_payload_here"}}}`
    // The example below uses a common high-level abstraction.
    doc["to"] = destination;        // Target: "^all" for broadcast to all nodes on the channel
    doc["text"] = textPayload;      // The text message content
    doc["want_ack"] = false;        // For broadcasts, ACKs are usually not requested/meaningful

    Serial.printf("Bus Node attempting to send text: '%s' to %s\n", textPayload.c_str(), destination.c_str());
    return sendMeshtasticJsonCommandBus(doc.as<JsonObject>());
}
