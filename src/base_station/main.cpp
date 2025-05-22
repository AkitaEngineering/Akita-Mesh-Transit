#include <Arduino.h>
#include <U8g2lib.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <vector>
#include <string>
#include <algorithm>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>
#include "FS.h" // For SPIFFS
#include "SPIFFS.h"

// --- Configuration ---
int STATION_ID = 0; // Default, will be overwritten by config file
const char* CONFIG_FILE_PATH = "/config.json";
const char* ETA_TABLE_FILE_PATH = "/eta_table.json";

const char* BUS_BLE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const int BLE_RSSI_THRESHOLD = -60;
const int BLE_SCAN_TIME_SECONDS = 3;
const unsigned long WATCHDOG_TIMEOUT_S_BASE = 20;
const unsigned long ARRIVAL_DEBOUNCE_MS = 2500;
const unsigned long DEPARTURE_GRACE_PERIOD_MS = 15000;
const unsigned long MESH_STATUS_QUERY_INTERVAL_MS = 60000;
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 1000;
const unsigned long BUS_DATA_TIMEOUT_MS = 5 * 60 * 1000;

const size_t JSON_DOC_CONFIG_SIZE = 512;     // For reading/writing config.json
const size_t JSON_DOC_ETA_TABLE_SIZE = 1024; // For reading/writing eta_table.json
const size_t JSON_DOC_INPUT_SIZE = 1024;     // For parsing Meshtastic responses
const size_t JSON_DOC_OUTPUT_SIZE = 256;    // For sending commands to Meshtastic

std::vector<String> knownBusIDs;

// --- Hardware Pins ---
// FIXME: PIN - Define actual pins for your ESP32 board (OLED I2C, Meshtastic Serial UART)
#define OLED_SDA 17 // Example I2C SDA Pin
#define OLED_SCL 18 // Example I2C SCL Pin
#define OLED_RST 21 // Example OLED Reset Pin (-1 if not used)

HardwareSerial& MeshtasticSerial = Serial1; // Using ESP32's UART1 for Meshtastic comms
const long MESHTASTIC_BAUD = 115200;
// FIXME: MESH PIN - If Serial1 pins are not default for your board, define them here:
// Example: #define MESHTASTIC_SERIAL_RX_PIN GPIO_NUM_9
// Example: #define MESHTASTIC_SERIAL_TX_PIN GPIO_NUM_10
// And then use MeshtasticSerial.begin(MESHTASTIC_BAUD, SERIAL_8N1, MESHTASTIC_SERIAL_RX_PIN, MESHTASTIC_SERIAL_TX_PIN);

// --- Display Setup ---
// FIXME: DISPLAY - Adjust U8G2 constructor for your specific OLED/LCD model and connection (I2C/SPI)
// This example is for an SH1106 based 128x64 I2C display.
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ OLED_RST, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA);

// --- Global State Variables ---
String myNodeId = "N/A"; // This node's Meshtastic ID (e.g., "!aabbccdd")
int numMeshPeers = 0;    // Number of other nodes in the mesh (excluding self)

// Configuration status flags
bool mainConfigLoadedOk = false;
bool etaTableLoadedOk = false;

// --- Data Structures ---
struct ETARecord { int fromStopId; int toStopId; int travelTimeMinutes; };
std::vector<ETARecord> etaTable;

struct BusInfo {
  String id;
  int currentStopId;
  unsigned long lastUpdateTimeMs;
  int etaToThisStationMinutes;
  bool isAtThisStation;
  bool displayStale;
  BusInfo() : currentStopId(0), lastUpdateTimeMs(0), etaToThisStationMinutes(-1), isAtThisStation(false), displayStale(false) {}
};
std::vector<BusInfo> trackedBuses;
const size_t MAX_DISPLAYED_BUSES = 2; // Max buses shown on display simultaneously
const size_t MAX_TRACKED_BUSES_IN_MEMORY = 10; // Max buses to keep data for

bool aBusIsPhysicallyPresentBLE = false; // If base station's own BLE scan detects a bus
unsigned long bleBusPotentialArrivalTime = 0;
bool bleBusPotentiallyArriving = false;
unsigned long bleBusLastSeenAtThisStop = 0;


// --- Forward Declarations ---
bool loadConfiguration();
bool createDefaultConfigFile();
bool loadEtaTableFromSpiffs();
bool createDefaultEtaTableFile();
bool setupDisplay(); void updateDisplay(bool forceUpdate = false);
bool setupBLEBase(); bool startBLEScanBase();
int calculateETA(int fromStop, int toStop);
bool setupMeshtasticSerialBase();
bool sendMeshtasticJsonCommandBase(const JsonObject& commandJson);
void processIncomingMeshSerialDataBase();
void requestMyNodeInfoBase();
void cleanupStaleBusData();
void feedWatchdogBase();
bool isKnownBusID(const String& busId);

// --- BLE Scan Callback (for Base Station's optional local detection) ---
class BaseStationBLECallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(BLEUUID(BUS_BLE_UUID))) {
      int rssi = advertisedDevice.getRSSI();
      Serial.printf("BaseStation BLE: Found a bus beacon, RSSI: %d\n", rssi);
      feedWatchdogBase();
      if (rssi > BLE_RSSI_THRESHOLD) {
        bleBusLastSeenAtThisStop = millis();
        if (!aBusIsPhysicallyPresentBLE) {
           if (!bleBusPotentiallyArriving) {
             bleBusPotentiallyArriving = true; bleBusPotentialArrivalTime = millis();
           } else if (millis() - bleBusPotentialArrivalTime > ARRIVAL_DEBOUNCE_MS) {
             Serial.println("BaseStation BLE: A bus confirmed physically present.");
             aBusIsPhysicallyPresentBLE = true; bleBusPotentiallyArriving = false;
             updateDisplay(true); // Update display to reflect local BLE detection
           }
        }
      } else {
        if (bleBusPotentiallyArriving) bleBusPotentiallyArriving = false;
        if (aBusIsPhysicallyPresentBLE && (millis() - bleBusLastSeenAtThisStop > DEPARTURE_GRACE_PERIOD_MS)) {
             Serial.println("BaseStation BLE: Physical bus presence ended.");
             aBusIsPhysicallyPresentBLE = false;
             updateDisplay(true);
        }
      }
    }
  }
};
BaseStationBLECallbacks myBaseBLECallbacks;
BLEScan* pBLEScanBase = nullptr;

// --- Main Setup ---
void setup() {
  Serial.begin(115200);
  // while(!Serial && millis() < 3000); // Optional: wait for serial monitor

  // Initialize SPIFFS once at the beginning
  Serial.println("Mounting SPIFFS...");
  if (!SPIFFS.begin(true)) { // true = format SPIFFS if mount failed
      Serial.println("CRITICAL ERROR: SPIFFS Mount Failed. Configuration cannot be loaded/created. System Halting.");
      // Attempt to display error on OLED if possible, before halting
      // Note: u8g2 might not be initialized yet if SPIFFS fails very early.
      // This is a best-effort display for critical failure.
      u8g2.begin(); // Attempt to init display
      u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0, 10, "FATAL ERROR:"); u8g2.drawStr(0, 22, "SPIFFS Mount Fail");
      u8g2.drawStr(0, 34, "System Halted."); u8g2.sendBuffer();
      while(1) { delay(1000); } // Halt
  } else {
      Serial.println("SPIFFS mounted successfully.");
  }
  feedWatchdogBase(); // Feed after SPIFFS init

  mainConfigLoadedOk = loadConfiguration(); // Loads STATION_ID and knownBusIDs
  feedWatchdogBase();
  etaTableLoadedOk = loadEtaTableFromSpiffs(); // Loads ETA table
  feedWatchdogBase();


  if (!mainConfigLoadedOk) {
      Serial.println("ERROR: Main configuration missing/invalid after attempt to load/create. System functionality will be SEVERELY LIMITED.");
  }
  if (!etaTableLoadedOk) {
      Serial.println("WARNING: ETA table missing/invalid after attempt to load/create. ETA calculations will be disabled or use defaults.");
  }

  Serial.printf("\n--- Akita Base Station ID: %d Initializing ---\n", STATION_ID);
  Serial.println("SECURITY NOTE: Ensure a STRONG and UNIQUE Pre-Shared Key (PSK) is configured for your Meshtastic channel!");
  if (knownBusIDs.empty() && mainConfigLoadedOk) {
    Serial.println("SECURITY WARNING: Known Bus ID list is empty in config. Will not validate incoming bus messages!");
  } else if (!mainConfigLoadedOk) {
    Serial.println("SECURITY WARNING: Main config not loaded, so no known Bus IDs. Bus message validation disabled!");
  } else {
    Serial.print("Loaded Known Bus IDs: ");
    for(const auto& id : knownBusIDs) { Serial.print(id); Serial.print(" "); }
    Serial.println();
  }

  if (etaTable.empty() && etaTableLoadedOk) {
      Serial.println("WARNING: ETA Table is empty in config. ETA calculations will not work.");
  } else if (!etaTableLoadedOk) {
      Serial.println("WARNING: ETA Table not loaded from SPIFFS. Calculations disabled.");
  }


  Serial.println("Initializing Watchdog...");
  if (esp_task_wdt_init(WATCHDOG_TIMEOUT_S_BASE, true) != ESP_OK) Serial.println("ERROR: WDT init fail");
  else if (esp_task_wdt_add(NULL) != ESP_OK) Serial.println("ERROR: WDT add task fail");
  feedWatchdogBase();

  if (!setupDisplay()) { Serial.println("CRITICAL ERROR: Display setup failed! Halting."); while(1) { feedWatchdogBase(); delay(1000); } }
  if (!setupBLEBase()) Serial.println("WARNING: Base Station BLE setup failed. Local BLE detection disabled.");
  if (!setupMeshtasticSerialBase()) Serial.println("WARNING: Meshtastic Serial setup failed. Mesh communication disabled.");

  requestMyNodeInfoBase(); // Request initial node info from Meshtastic
  Serial.println("Base Station Setup Complete.");
  updateDisplay(true); // Force initial display with config status
  if(pBLEScanBase != nullptr) startBLEScanBase(); // Start BLE scanning if setup was successful
}

// --- Main Loop ---
void loop() {
  feedWatchdogBase();
  processIncomingMeshSerialDataBase(); // Check for and process messages from Meshtastic
  feedWatchdogBase();

  unsigned long now = millis();
  static unsigned long lastScanStartTime = 0;
  // Periodically start BLE scan if BLE is setup and not currently scanning
  if (pBLEScanBase != nullptr && !pBLEScanBase->isScanning() && (now - lastScanStartTime > (BLE_SCAN_TIME_SECONDS * 1000 + 1000))) {
     if (startBLEScanBase()) lastScanStartTime = now;
     // else Serial.println("Warning: Failed to start periodic BLE scan."); // Can be noisy
  }
  feedWatchdogBase();

  static unsigned long lastDisplayUpdateTime = 0;
  if (now - lastDisplayUpdateTime > DISPLAY_UPDATE_INTERVAL_MS) {
     updateDisplay(); // Regular display refresh
     lastDisplayUpdateTime = now;
  }
  feedWatchdogBase();

  static unsigned long lastMeshQueryTime = 0;
  if (now - lastMeshQueryTime > MESH_STATUS_QUERY_INTERVAL_MS) {
      requestMyNodeInfoBase(); // Periodically ask Meshtastic for our node info
      lastMeshQueryTime = now;
  }

  static unsigned long lastStaleDataCleanupTime = 0;
  if (now - lastStaleDataCleanupTime > BUS_DATA_TIMEOUT_MS / 2) { // Check for stale buses more often than timeout
      cleanupStaleBusData();
      lastStaleDataCleanupTime = now;
  }
  delay(10); // Small delay to yield to other tasks (like WiFi/BLE background tasks)
}

// --- Function Implementations ---
void feedWatchdogBase() { esp_task_wdt_reset(); }

bool createDefaultConfigFile() {
    Serial.printf("Attempting to create default configuration file: %s\n", CONFIG_FILE_PATH);
    StaticJsonDocument<JSON_DOC_CONFIG_SIZE> doc;
    doc["station_id"] = 0; // User MUST change this after creation
    JsonArray busIds = doc.createNestedArray("known_bus_ids");
    busIds.add("B101-EDIT-ME"); // Placeholder, user must edit

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "w"); // Open for writing
    if (!configFile) {
        Serial.printf("ERROR: Failed to create/open default config file '%s' for writing.\n", CONFIG_FILE_PATH);
        return false;
    }
    if (serializeJson(doc, configFile) == 0) {
        Serial.println("ERROR: Failed to write JSON to default config file.");
        configFile.close();
        return false;
    }
    configFile.close();
    Serial.println("Default config.json created successfully. PLEASE EDIT IT with correct values (Station ID, Known Bus IDs) via SPIFFS upload, then REBOOT device!");
    return true;
}

bool loadConfiguration() {
    // Assumes SPIFFS.begin() was called successfully in setup()
    if (!SPIFFS.exists(CONFIG_FILE_PATH)) {
        Serial.printf("Main config file %s not found. Attempting to create default.\n", CONFIG_FILE_PATH);
        if (!createDefaultConfigFile()) {
            Serial.println("ERROR: Failed to create default config. Using hardcoded defaults for STATION_ID and no knownBusIDs.");
            STATION_ID = 0; // Critical Fallback
            knownBusIDs.clear();
            return false; // Indicate config load failure
        }
        // After creating, inform user to reboot after editing the new default file.
        // For this current run, it will effectively use defaults.
        Serial.println("Default config file created. Please edit and reboot.");
        return false; // Indicate that it wasn't loaded from an existing user-edited file this pass.
    }

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "r");
    if (!configFile) {
        Serial.printf("ERROR: Failed to open main config file %s for reading.\n", CONFIG_FILE_PATH);
        return false;
    }

    Serial.printf("Reading main configuration from %s\n", CONFIG_FILE_PATH);
    StaticJsonDocument<JSON_DOC_CONFIG_SIZE> doc;
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close(); // Close file immediately after reading
    feedWatchdogBase();

    if (error) {
        Serial.print("ERROR: Failed to parse main config file: "); Serial.println(error.c_str());
        return false;
    }

    // Load STATION_ID
    if (doc.containsKey("station_id") && doc["station_id"].is<int>()) {
        STATION_ID = doc["station_id"].as<int>();
        Serial.printf("Loaded Station ID: %d\n", STATION_ID);
        if (STATION_ID == 0 && doc["station_id"].as<int>() == 0) { // If default was left unedited
            Serial.println("WARNING: Station ID is 0 (default from config). Please edit config.json!");
        }
    } else {
        Serial.println("WARNING: 'station_id' not found or invalid in config. Using default/previous value.");
        // STATION_ID retains its initialized value (0) if not found
    }

    // Load knownBusIDs
    knownBusIDs.clear(); // Clear any previous values
    if (doc.containsKey("known_bus_ids") && doc["known_bus_ids"].is<JsonArray>()) {
        JsonArray busIdArray = doc["known_bus_ids"].as<JsonArray>();
        for (JsonVariant id_variant : busIdArray) {
            if (id_variant.is<const char*>()) {
                 String busId = id_variant.as<String>();
                 if (busId.length() > 0 && busId != "B101-EDIT-ME") { // Don't add the placeholder if unedited
                    knownBusIDs.push_back(busId);
                 }
            }
        }
        Serial.printf("Loaded %d known bus IDs from main config.\n", knownBusIDs.size());
        if (knownBusIDs.empty() && busIdArray.size() > 0) {
             Serial.println("WARNING: known_bus_ids contained only placeholder or invalid entries.");
        }
    } else {
        Serial.println("WARNING: 'known_bus_ids' array not found or invalid in main config. No bus IDs loaded for validation.");
    }
    Serial.println("Main configuration loaded successfully.");
    return true;
}

bool createDefaultEtaTableFile() {
    Serial.printf("Attempting to create default ETA table file: %s\n", ETA_TABLE_FILE_PATH);
    StaticJsonDocument<JSON_DOC_ETA_TABLE_SIZE> doc;
    JsonArray etaArray = doc.to<JsonArray>();

    JsonObject record1 = etaArray.createNestedObject();
    record1["from"] = 1; record1["to"] = 2; record1["time_min"] = 5;
    JsonObject record2 = etaArray.createNestedObject();
    record2["from"] = 2; record2["to"] = 3; record2["time_min"] = 7;
    // Add more placeholders if desired

    File etaFile = SPIFFS.open(ETA_TABLE_FILE_PATH, "w");
    if (!etaFile) {
        Serial.printf("ERROR: Failed to create/open default ETA table file '%s' for writing.\n", ETA_TABLE_FILE_PATH);
        return false;
    }
    if (serializeJson(doc, etaFile) == 0) {
        Serial.println("ERROR: Failed to write JSON to default ETA table file.");
        etaFile.close();
        return false;
    }
    etaFile.close();
    Serial.println("Default eta_table.json created. PLEASE EDIT IT with correct calibrated ETA values via SPIFFS upload, then REBOOT device!");
    return true;
}


bool loadEtaTableFromSpiffs() {
    // Assumes SPIFFS.begin() was called successfully in setup()
    if (!SPIFFS.exists(ETA_TABLE_FILE_PATH)) {
        Serial.printf("ETA table file %s not found. Attempting to create default.\n", ETA_TABLE_FILE_PATH);
        if (!createDefaultEtaTableFile()) {
            Serial.println("ERROR: Failed to create default ETA table. ETA calculations will use empty table.");
            etaTable.clear();
            return false;
        }
        etaTable.clear(); // Ensure table is empty as default was just created.
        return false; // Indicate it wasn't loaded from an existing user-edited file this pass.
    }

    File etaFile = SPIFFS.open(ETA_TABLE_FILE_PATH, "r");
    if (!etaFile) {
        Serial.printf("ERROR: Failed to open ETA table file %s for reading.\n", ETA_TABLE_FILE_PATH);
        etaTable.clear();
        return false;
    }

    Serial.printf("Reading ETA table from %s\n", ETA_TABLE_FILE_PATH);
    StaticJsonDocument<JSON_DOC_ETA_TABLE_SIZE> doc;
    DeserializationError error = deserializeJson(doc, etaFile);
    etaFile.close(); // Close file immediately
    feedWatchdogBase();

    if (error) {
        Serial.print("ERROR: Failed to parse ETA table file: "); Serial.println(error.c_str());
        etaTable.clear();
        return false;
    }

    etaTable.clear(); // Clear any previous/default entries
    if (doc.is<JsonArray>()) {
        JsonArray etaArray = doc.as<JsonArray>();
        Serial.printf("Loading %d ETA records from file...\n", etaArray.size());
        for (JsonObject recordJson : etaArray) {
            // Robust checking for all fields and their types
            if (recordJson.containsKey("from") && recordJson["from"].is<int>() &&
                recordJson.containsKey("to") && recordJson["to"].is<int>() &&
                recordJson.containsKey("time_min") && recordJson["time_min"].is<int>()) {
                ETARecord record;
                record.fromStopId = recordJson["from"].as<int>();
                record.toStopId = recordJson["to"].as<int>();
                record.travelTimeMinutes = recordJson["time_min"].as<int>();
                // Add validation for sensible values
                if (record.fromStopId > 0 && record.toStopId > 0 && record.travelTimeMinutes >= 0) {
                    etaTable.push_back(record);
                } else Serial.println("Warning: Invalid ETA record data skipped (e.g., stopID <=0 or time_min <0).");
            } else Serial.println("Warning: Incomplete or malformed ETA record in config (missing keys or wrong types).");
        }
        Serial.printf("Successfully loaded %d ETA records into table.\n", etaTable.size());
    } else {
        Serial.println("ERROR: ETA table file content is not a valid JSON array.");
        etaTable.clear();
        return false;
    }
    Serial.println("ETA table loaded successfully from SPIFFS.");
    return true;
}


bool isKnownBusID(const String& busId) {
    if (!mainConfigLoadedOk || knownBusIDs.empty()) {
        Serial.println("Warning: Known Bus ID list not loaded from config or is empty. Allowing all bus IDs for now (SECURITY RISK for production).");
        return true; // Fallback to allow if config is problematic, log indicates risk
    }
    return std::find(knownBusIDs.begin(), knownBusIDs.end(), busId) != knownBusIDs.end();
}

bool setupDisplay() {
  Serial.println("Setting up display...");
  #if OLED_RST > 0 // Conditionally compile reset logic
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, HIGH); delay(1); // Ensure reset is high
    digitalWrite(OLED_RST, LOW);  delay(10); // Pulse low
    digitalWrite(OLED_RST, HIGH); delay(10); // Back to high
  #endif
  // Consider Wire.begin(OLED_SDA, OLED_SCL); if pins are non-default for I2C and not handled by board variant
  u8g2.begin();
  u8g2.enableUTF8Print(); // Good practice
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr); // A slightly larger font for title
  u8g2.drawStr(0, 10, "Akita Mesh Transit");
  u8g2.setFont(u8g2_font_6x10_tf); // Smaller font for status lines
  u8g2.setCursor(0, 22); u8g2.print("Station "); u8g2.print(STATION_ID); // STATION_ID is now from config
  u8g2.drawStr(0, 34, "Initializing...");
  if (!mainConfigLoadedOk) u8g2.drawStr(0, 46, "MainCFG ERR!");
  else if (!etaTableLoadedOk) u8g2.drawStr(0, 46, "EtaCFG ERR!");
  u8g2.sendBuffer();
  Serial.println("Display setup complete."); return true;
}

void cleanupStaleBusData() {
    unsigned long currentTime = millis(); bool changed = false;
    trackedBuses.erase(std::remove_if(trackedBuses.begin(), trackedBuses.end(),
        [&](const BusInfo& bus) {
            if (currentTime - bus.lastUpdateTimeMs > BUS_DATA_TIMEOUT_MS) {
                Serial.printf("Removing stale data for Bus ID: %s\n", bus.id.c_str());
                changed = true; return true;
            } return false;
        }), trackedBuses.end());
    if (changed) updateDisplay(true);
}

void updateDisplay(bool forceUpdate /*= false*/) {
    for (auto& bus : trackedBuses) {
        if (bus.currentStopId == STATION_ID) {
            bus.etaToThisStationMinutes = 0; bus.isAtThisStation = true;
        } else if (bus.currentStopId != 0) { // If we know its location
            bus.etaToThisStationMinutes = calculateETA(bus.currentStopId, STATION_ID);
            bus.isAtThisStation = false;
        } else { // Unknown location
            bus.etaToThisStationMinutes = -99; bus.isAtThisStation = false;
        }
        bus.displayStale = (millis() - bus.lastUpdateTimeMs > BUS_DATA_TIMEOUT_MS / 2);
    }
    std::sort(trackedBuses.begin(), trackedBuses.end(), [](const BusInfo& a, const BusInfo& b) {
        if (a.isAtThisStation && !b.isAtThisStation) return true;
        if (!a.isAtThisStation && b.isAtThisStation) return false;
        if (a.etaToThisStationMinutes >= 0 && b.etaToThisStationMinutes >= 0) {
            if (a.etaToThisStationMinutes != b.etaToThisStationMinutes) return a.etaToThisStationMinutes < b.etaToThisStationMinutes;
        } else if (a.etaToThisStationMinutes >= 0) return true; // a is valid, b is not
        else if (b.etaToThisStationMinutes >= 0) return false;// b is valid, a is not
        return a.id < b.id; // Fallback sort by ID
    });

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf); // Use a consistent small font for data lines
    char line[48]; int yPos = 8; // Adjusted max line length and starting yPos

    // Line 1: Station ID, Meshtastic Node ID, Peer count
    snprintf(line, sizeof(line), "S%d M:%s(%d)", STATION_ID, myNodeId.c_str(), numMeshPeers);
    u8g2.drawStr(0, yPos, line); yPos += 10;

    // Line 2: Configuration Status (if errors)
    if (!mainConfigLoadedOk) {
        u8g2.setDrawColor(1); // Ensure text is visible
        snprintf(line, sizeof(line), "Err: Main Config!");
        u8g2.drawStr(0, yPos, line); yPos += 10;
    } else if (!etaTableLoadedOk) {
        u8g2.setDrawColor(1);
        snprintf(line, sizeof(line), "Err: ETA Config!");
        u8g2.drawStr(0, yPos, line); yPos += 10;
    }


    // Lines 3 onwards: Bus Data
    if (trackedBuses.empty()) {
        if (aBusIsPhysicallyPresentBLE && yPos < 64 - 8) { // Check space before drawing
            snprintf(line, sizeof(line), "Bus Near (BLE)");
            u8g2.drawStr(0, yPos, line); yPos += 10;
        }
        if (yPos < 64-8) {
            snprintf(line, sizeof(line), "No bus data yet...");
            u8g2.drawStr(0, yPos, line);
        }
    } else {
        size_t busesToShow = 0;
        // Dynamically determine how many buses can be shown based on remaining space
        if (yPos <= 18) busesToShow = MAX_DISPLAYED_BUSES;       // Enough space for 2 buses + status
        else if (yPos <= 28) busesToShow = MAX_DISPLAYED_BUSES -1; // Enough space for 1 bus + status
        // else busesToShow remains 0

        busesToShow = std::min(trackedBuses.size(), busesToShow);


        for (size_t i = 0; i < busesToShow && yPos < 64 - 8; ++i) { // Ensure text fits on display
            const auto& bus = trackedBuses[i]; String busIdShort = bus.id;
            if (busIdShort.length() > 5) busIdShort = busIdShort.substring(0, 5); // Truncate ID for display

            if (bus.isAtThisStation) snprintf(line, sizeof(line), "%s: ARRIVED%s", busIdShort.c_str(), bus.displayStale ? "*" : "");
            else if (bus.etaToThisStationMinutes >= 0) snprintf(line, sizeof(line), "%s@S%d ETA:%dmin%s", busIdShort.c_str(), bus.currentStopId, bus.etaToThisStationMinutes, bus.displayStale ? "*" : "");
            else if (bus.etaToThisStationMinutes == -1) snprintf(line, sizeof(line), "%s@S%d Passed%s", busIdShort.c_str(), bus.currentStopId, bus.displayStale ? "*" : "");
            else snprintf(line, sizeof(line), "%s@S%d N/A%s", busIdShort.c_str(), bus.currentStopId, bus.displayStale ? "*" : ""); // -99 (unknown) or -2 (path error)
            u8g2.drawStr(0, yPos, line); yPos += 10;
        }

        if (busesToShow < trackedBuses.size() && yPos < 64 -8){ // If more buses than shown
            snprintf(line, sizeof(line), "...%d more buses", (int)(trackedBuses.size() - busesToShow));
            u8g2.drawStr(0, yPos, line);
        } else if (busesToShow == 0 && trackedBuses.size() > 0 && yPos < 64 -8){ // No space to show any bus, but data exists
            u8g2.drawStr(0, yPos, "See Serial Log...");
        } else if (busesToShow == 0 && aBusIsPhysicallyPresentBLE && yPos < 64-8) { // No bus data, but BLE detected one
             u8g2.drawStr(0, yPos, "Bus Near (BLE)");
        }
    }
    u8g2.sendBuffer();
}

bool setupBLEBase() {
  Serial.println("Setting up Base Station BLE Scanner...");
  if (!BLEDevice::init("")) { Serial.println("ERROR: Failed to initialize BLEDevice for Base"); return false; }
  pBLEScanBase = BLEDevice::getScan();
  if (pBLEScanBase == nullptr) { Serial.println("ERROR: Failed to get Base BLEScan object"); return false; }
  pBLEScanBase->setAdvertisedDeviceCallbacks(&myBaseBLECallbacks);
  pBLEScanBase->setActiveScan(true); pBLEScanBase->setInterval(100); pBLEScanBase->setWindow(99);
  Serial.println("Base Station BLE Scanner Setup Complete."); return true;
}

bool startBLEScanBase() {
    if (pBLEScanBase == nullptr) { Serial.println("ERROR: Base BLE Scan object not initialized."); return false; }
    if (pBLEScanBase->isScanning()) return false; // Don't restart if already scanning
    if (!pBLEScanBase->start(BLE_SCAN_TIME_SECONDS, false /*is_async*/)) { // false means scan for duration then stop
        Serial.println("ERROR: Failed to start Base BLE scan."); return false;
    }
    return true;
}

int calculateETA(int fromStop, int toStop) {
  if (!etaTableLoadedOk || etaTable.empty()) { /*Serial.println("ETA calculation skipped: ETA table empty or not loaded.");*/ return -99; } // -99 for unknown/unavailable ETA
  if (fromStop <= 0 || toStop <= 0) return -2; // Invalid stop IDs (0 is often "unknown location")
  if (fromStop == toStop) return 0; // Arrived

  int totalTime = 0; int current = fromStop;
  std::vector<int> visitedStops; // To detect loops in ETA path
  visitedStops.push_back(current);

  while (current != toStop) {
      // Increased safety break limit slightly, depends on max number of stops in a single route
      if (visitedStops.size() > etaTable.size() + 12) { // Max number of segments + some buffer
          Serial.printf("ERROR: ETA calculation loop detected or path too long (from %d to %d)\n", fromStop, toStop);
          return -2; // Path error
      }
      bool foundNextLeg = false;
      for (const auto& record : etaTable) {
          if (record.fromStopId == current) {
              totalTime += record.travelTimeMinutes;
              current = record.toStopId;
              foundNextLeg = true;

              // Check if we've already visited the new 'current' stop in this calculation path (loop detection)
              // This check is important for non-linear routes or misconfigured ETA tables
              for(int visited : visitedStops) {
                  if (visited == current && current != toStop) { // Loop detected if we return to a stop that isn't the final target
                      Serial.printf("ERROR: ETA calculation encountered loop at stop %d (path from %d to %d)\n", current, fromStop, toStop);
                      return -2; // Loop detected
                  }
              }
              visitedStops.push_back(current);


              // Check if we just passed the target stop
              // This logic assumes that if 'toStop' appears in 'visitedStops' *before* 'current' becomes 'toStop',
              // then the bus has passed it on its way to a further stop.
              bool passedTarget = false;
              for(size_t i = 0; i < visitedStops.size() -1 ; ++i) { // Exclude the current 'current' stop from this check
                  if (visitedStops[i] == toStop) {
                      passedTarget = true;
                      break;
                  }
              }
              if (passedTarget && current != toStop) { // If target was visited, and we are now at a different stop
                  return -1; // Indicates the bus has already passed the destination stop
              }
              break; // Found the next leg of the journey, continue while loop
          }
      }
      if (!foundNextLeg) {
          Serial.printf("ERROR: No ETA path found from stop %d towards %d in etaTable.\n", visitedStops.back(), toStop);
          return -2; // Path error: No route defined from current stop
      }
  }
  return totalTime;
}

bool setupMeshtasticSerialBase() {
  Serial.printf("Initializing Meshtastic Serial on UART1 at %ld baud for Base Station...\n", MESHTASTIC_BAUD);
  // FIXME: MESH PIN - If using non-default pins for Serial1, pass them here:
  // MeshtasticSerial.begin(MESHTASTIC_BAUD, SERIAL_8N1, MESHTASTIC_SERIAL_RX_PIN, MESHTASTIC_SERIAL_TX_PIN);
  MeshtasticSerial.begin(MESHTASTIC_BAUD);
  delay(100); // Allow serial to initialize
  if (!MeshtasticSerial) { Serial.println("ERROR: MeshtasticSerial (UART1) failed to begin on Base Station!"); return false; }
  Serial.println("Base Station Meshtastic Serial initialized. Ensure Meshtastic firmware is configured for this UART, baud, and JSON output.");
  return true;
}

bool sendMeshtasticJsonCommandBase(const JsonObject& commandJson) {
    if (!MeshtasticSerial) { Serial.println("ERROR: Base Meshtastic Serial port not ready!"); return false; }
    Serial.print("Base Sending to Meshtastic: "); serializeJsonPretty(commandJson, Serial); Serial.println(); // Use Pretty for debug
    serializeJson(commandJson, MeshtasticSerial); // Send compact JSON to device
    MeshtasticSerial.println(); // Meshtastic Serial API usually expects newline terminated commands
    feedWatchdogBase();
    // FIXME: MESHTASTIC API - Consider implementing logic to wait for an ACK/NACK from Meshtastic if the command supports it.
    return true;
}

void requestMyNodeInfoBase() {
    StaticJsonDocument<JSON_DOC_OUTPUT_SIZE> doc;
    // FIXME: MESHTASTIC API - CRITICAL: Verify this is the correct command JSON for your Meshtastic firmware version's Serial API.
    // This command requests the Meshtastic firmware to send back information about the current node.
    // Common patterns include "get_my_info", or it might be nested like "request": {"get_my_node_info": true}
    // The simplest form that some APIs might accept:
    doc["get_my_node_info"] = true; // This is a common guess.
    // Alternatively, it might be:
    // doc["get_my_info"] = true;

    doc["want_response"] = true;    // Usually good to include if you expect data back.

    Serial.println("Base Requesting My Node Info from Meshtastic...");
    if (!sendMeshtasticJsonCommandBase(doc.as<JsonObject>())) {
        Serial.println("Failed to send MyNodeInfo request from Base.");
    }
}

void processIncomingMeshSerialDataBase() {
    String line = "";
    if (MeshtasticSerial.available()) { line = MeshtasticSerial.readStringUntil('\n'); line.trim(); }

    if (line.length() > 0) {
        Serial.printf("Base MeshRX: %s\n", line.c_str()); feedWatchdogBase();
        StaticJsonDocument<JSON_DOC_INPUT_SIZE> doc;
        DeserializationError error = deserializeJson(doc, line);

        if (error) {
            Serial.print("Base deserializeJson() failed: "); Serial.println(error.c_str());
            return;
        }

        // --- FIXME: MESHTASTIC API - CRITICAL: Verify these JSON paths and structures with your Meshtastic Serial API output ---
        // The parsing below tries to be robust but depends on common Meshtastic JSON patterns.
        // You MUST inspect the actual JSON output from your Meshtastic device's Serial API.

        const char* rootTypeChar = nullptr;
        if (doc.containsKey("type") && doc["type"].is<const char*>()) {
            rootTypeChar = doc["type"];
        }
        String rootType = rootTypeChar ? String(rootTypeChar) : String("");

        JsonObject packet;
        if (doc.containsKey("packet") && doc["packet"].is<JsonObject>()) {
            packet = doc["packet"].as<JsonObject>();
        }

        JsonObject decoded; // Decoded part of the packet
        if (!packet.isNull() && packet.containsKey("decoded") && packet["decoded"].is<JsonObject>()) {
            decoded = packet["decoded"].as<JsonObject>();
        }

        String textPayload = "";
        String fromNodeIdStr = ""; // Sender's Meshtastic Node ID (e.g., "!aabbccdd")
        String portnumStr = "";    // Portnum (e.g., "TEXT_MESSAGE_APP" or "256")

        // Try to extract sender Node ID (from various common fields)
        if (doc.containsKey("from") && doc["from"].is<String>()) fromNodeIdStr = doc["from"].as<String>();
        else if (!packet.isNull() && packet.containsKey("from") && packet["from"].is<String>()) fromNodeIdStr = packet["from"].as<String>();
        else if (!packet.isNull() && packet.containsKey("fromId") && packet["fromId"].is<String>()) fromNodeIdStr = packet["fromId"].as<String>();
        else if (!packet.isNull() && packet.containsKey("senderNode") && packet["senderNode"].is<String>()) fromNodeIdStr = packet["senderNode"].as<String>();


        // Try to extract text payload and portnum from "decoded" if present
        if (!decoded.isNull() && decoded.containsKey("portnum")) {
            if(decoded["portnum"].is<const char*>()) portnumStr = decoded["portnum"].as<String>();
            else if(decoded["portnum"].is<int>()) portnumStr = String(decoded["portnum"].as<int>());

            if ((portnumStr == "TEXT_MESSAGE_APP" || portnumStr == "256") && // Check for text message port
                decoded.containsKey("payload") && decoded["payload"].is<String>()) {
                textPayload = decoded["payload"].as<String>();
            }
        }
        // Fallback: Check for direct "text" field at root (less common for mesh packets, more for direct serial commands)
        else if (doc.containsKey("text") && doc["text"].is<String>()) {
             textPayload = doc["text"].as<String>();
        }


        // --- Logic for !BUS: messages ---
        if (textPayload.startsWith("!BUS:")) {
            int firstColon = textPayload.indexOf(':');
            int secondColon = textPayload.indexOf(':', firstColon + 1);
            if (firstColon != -1 && secondColon != -1 && secondColon > firstColon) {
                String receivedBusId = textPayload.substring(firstColon + 1, secondColon);
                String receivedStopIdStr = textPayload.substring(secondColon + 1);
                int receivedStopId = receivedStopIdStr.toInt();

                if (!isKnownBusID(receivedBusId)) { // Security Check
                    Serial.printf("SECURITY WARNING: Msg from UNKNOWN Bus ID: '%s'. Sender Node: '%s'. Ignoring.\n", receivedBusId.c_str(), fromNodeIdStr.c_str());
                    return; // Ignore message
                }

                if (receivedStopId > 0) { // Basic validation for stop ID
                    Serial.printf("Parsed Bus Update: ID=%s, Stop=%d (From Node: %s)\n", receivedBusId.c_str(), receivedStopId, fromNodeIdStr.c_str());
                    BusInfo* busToUpdate = nullptr;
                    for (auto& bus : trackedBuses) { if (bus.id == receivedBusId) { busToUpdate = &bus; break; } }

                    if (busToUpdate == nullptr) { // New bus seen
                        if (trackedBuses.size() < MAX_TRACKED_BUSES_IN_MEMORY) {
                            trackedBuses.emplace_back(); busToUpdate = &trackedBuses.back();
                            busToUpdate->id = receivedBusId;
                            Serial.printf("Tracking new bus: %s\n", receivedBusId.c_str());
                        } else {
                            Serial.println("Max tracked buses reached, ignoring new bus. Consider increasing MAX_TRACKED_BUSES_IN_MEMORY.");
                        }
                    }

                    if (busToUpdate != nullptr) {
                        busToUpdate->currentStopId = receivedStopId;
                        busToUpdate->lastUpdateTimeMs = millis();
                        busToUpdate->isAtThisStation = (receivedStopId == STATION_ID);
                        busToUpdate->displayStale = false;
                        if (busToUpdate->isAtThisStation) { // If this bus arrived at our station
                            for (auto& otherBus : trackedBuses) { if (otherBus.id != busToUpdate->id) otherBus.isAtThisStation = false; }
                        }
                    }
                    updateDisplay(true); // Force display update with new bus data
                } else Serial.printf("Error parsing StopID from mesh message: '%s'\n", textPayload.c_str());
            } else Serial.printf("Malformed !BUS: message from mesh (colon format error): '%s'\n", textPayload.c_str());

        // --- Logic for MY_NODE_INFO response ---
        // Check for various ways Node Info might be presented by the Serial API
        } else if ( rootType.equalsIgnoreCase("MY_NODE_INFO") || rootType.equalsIgnoreCase("NODE_INFO_APP") ||
                    doc.containsKey("my_node_info") || // Common root key
                    (!decoded.isNull() && (portnumStr == "NODEINFO_APP" || portnumStr == "1")) ) { // NodeInfoApp packet

            JsonObject nodeInfoSource = doc.as<JsonObject>(); // Start with root
            if (doc.containsKey("my_node_info") && doc["my_node_info"].is<JsonObject>()) nodeInfoSource = doc["my_node_info"].as<JsonObject>();
            else if (doc.containsKey("node_info") && doc["node_info"].is<JsonObject>()) nodeInfoSource = doc["node_info"].as<JsonObject>(); // Another pattern
            else if (!decoded.isNull() && (portnumStr == "NODEINFO_APP" || portnumStr == "1")) nodeInfoSource = decoded; // If it's a decoded NodeInfoApp packet


            // Extract this node's ID
            if (nodeInfoSource.containsKey("my_node_num") && nodeInfoSource["my_node_num"].is<unsigned long>()) {
                char nodeIdHex[10];
                snprintf(nodeIdHex, sizeof(nodeIdHex), "!%lx", nodeInfoSource["my_node_num"].as<unsigned long>());
                myNodeId = String(nodeIdHex);
            } else if (nodeInfoSource.containsKey("node_id") && nodeInfoSource["node_id"].is<String>()) {
                 myNodeId = nodeInfoSource["node_id"].as<String>();
            } else if (nodeInfoSource.containsKey("id") && nodeInfoSource["id"].is<String>()) { // Sometimes just "id"
                 myNodeId = nodeInfoSource["id"].as<String>();
            }


            // Extract number of peers/nodes
            if (nodeInfoSource.containsKey("num_online") && nodeInfoSource["num_online"].is<int>()) numMeshPeers = nodeInfoSource["num_online"].as<int>() -1;
            else if (nodeInfoSource.containsKey("num_online_nodes_in_mesh") && nodeInfoSource["num_online_nodes_in_mesh"].is<int>()) numMeshPeers = nodeInfoSource["num_online_nodes_in_mesh"].as<int>() - 1;
            else if (nodeInfoSource.containsKey("num_nodes") && nodeInfoSource["num_nodes"].is<int>()) numMeshPeers = nodeInfoSource["num_nodes"].as<int>() -1; // Total nodes in list
            else if (nodeInfoSource.containsKey("peer_count") && nodeInfoSource["peer_count"].is<int>()) numMeshPeers = nodeInfoSource["peer_count"].as<int>(); // Direct peer count


            if (numMeshPeers < 0) numMeshPeers = 0; // Ensure non-negative

            Serial.printf("Base My Node Info Updated: ID=%s, Peers=%d\n", myNodeId.c_str(), numMeshPeers);
            updateDisplay(true); // Update display with new node info
        } else {
             if (textPayload.length() > 0) { // It was some other text message
                Serial.printf("Base MeshRX: Received other text: '%s' (From Node: %s)\n", textPayload.c_str(), fromNodeIdStr.c_str());
             } else {
                Serial.println("Base MeshRX: Unrecognized JSON structure or non-!BUS text/non-nodeinfo message.");
             }
        }
    }
}
