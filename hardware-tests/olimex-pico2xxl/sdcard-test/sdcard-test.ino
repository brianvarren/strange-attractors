/*
This sketch will:
1. Initialize the SD card with error handling
2. Show the card size
3. List all files and directories with sizes
4. Give troubleshooting info if it fails
*/
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// Olimex Pico2-XXL SD card configuration (from schematic)
const int SD_CS_PIN = 9;    // GPIO9\SPI1_CSn
const int SD_SCK_PIN = 10;  // GPIO10\SPI1_SCK\SD_CLK
const int SD_MOSI_PIN = 11; // GPIO11\SPI1_TX\SD_CMD
const int SD_MISO_PIN = 24; // GPIO12\SPI1_RX\SD_DAT0

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("=== SD Card Test - Olimex Pico2-XXL ===");
  
  // Debug: Print pin configuration
  Serial.printf("Using SPI1 with pins: CS=%d, SCK=%d, MOSI=%d, MISO=%d\n", 
                SD_CS_PIN, SD_SCK_PIN, SD_MOSI_PIN, SD_MISO_PIN);
  
  // Configure SPI1 pins explicitly
  SPI1.setRX(SD_MISO_PIN);
  SPI1.setTX(SD_MOSI_PIN);
  SPI1.setSCK(SD_SCK_PIN);
  
  // Set CS pin as output and pull high initially
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  
  Serial.println("SPI1 pins configured, attempting SD card initialization...");
  
  // Try different SPI speeds - start slow
  SPI1.begin();
  
  // Try SD.begin with explicit SPI instance
  if (!SD.begin(SD_CS_PIN, SPI1)) {
    Serial.println("❌ SD card initialization failed with SPI1!");
    
    // Try with default SPI (SPI0) as fallback
    Serial.println("Trying with default SPI (SPI0)...");
    SPI.setRX(SD_MISO_PIN);
    SPI.setTX(SD_MOSI_PIN);
    SPI.setSCK(SD_SCK_PIN);
    
    if (!SD.begin(SD_CS_PIN)) {
      Serial.println("❌ SD card initialization failed with SPI0 too!");
      Serial.println("\nTroubleshooting checklist:");
      Serial.println("  ✓ SD card is inserted properly");
      Serial.println("  ✓ SD card is formatted as FAT16 or FAT32");
      Serial.println("  ✓ SD card is not write-protected");
      Serial.println("  ✓ You have a Pico2-XXL (not regular Pico2)");
      Serial.println("  ✓ Hardware revision B or newer");
      Serial.println("  ✓ Try a different SD card if available");
      return;
    } else {
      Serial.println("✅ SD card working with SPI0!");
    }
  } else {
    Serial.println("✅ SD card working with SPI1!");
  }
  
  Serial.println("✅ SD card initialized successfully!");
  
  // Get SD card info
  uint32_t card_size = SD.size() / (1024 * 1024);  // Size in MB
  Serial.printf("SD Card Size: %u MB\n", card_size);
  
  // List files in root directory
  Serial.println("\n=== Files on SD Card ===");
  File root = SD.open("/");
  if (!root) {
    Serial.println("❌ Failed to open root directory");
    return;
  }
  
  int file_count = 0;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;  // No more files
    
    file_count++;
    
    // Print file info
    if (entry.isDirectory()) {
      Serial.printf("[DIR]  %s/\n", entry.name());
    } else {
      uint32_t size_kb = entry.size() / 1024;
      if (size_kb == 0 && entry.size() > 0) size_kb = 1;  // Show <1KB as 1KB
      
      Serial.printf("[FILE] %-20s %6u KB\n", entry.name(), size_kb);
    }
    
    entry.close();
  }
  
  root.close();
  
  if (file_count == 0) {
    Serial.println("(No files found - SD card is empty)");
  } else {
    Serial.printf("\nTotal items found: %d\n", file_count);
  }
  
  Serial.println("\nSD card is ready for use!");
}

void loop() {
  // Nothing to do
}