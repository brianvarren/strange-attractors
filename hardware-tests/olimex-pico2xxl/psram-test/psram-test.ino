/*
This sketch will:
1. Check if PSRAM is detected
2. Show chips size and heap info
3. Test a basic pmalloc/free cycle
4. Output results to serial monitor
*/

#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("=== PSRAM Test - Olimex Pico2-XXL ===");
  
  // Check PSRAM detection
  uint32_t psram_size = rp2040.getPSRAMSize();
  
  if (psram_size == 0) {
    Serial.println("❌ No PSRAM detected!");
    return;
  }
  
  Serial.println("✅ PSRAM detected!");
  Serial.printf("PSRAM Chip Size: %u MB (%u KB)\n", 
                psram_size / 1048576, psram_size / 1024);
  
  Serial.printf("Total Heap: %u KB\n", rp2040.getTotalPSRAMHeap() / 1024);
  Serial.printf("Free Heap:  %u KB\n", rp2040.getFreePSRAMHeap() / 1024);
  Serial.printf("Used Heap:  %u KB\n", rp2040.getUsedPSRAMHeap() / 1024);
  
  // Simple allocation test
  Serial.println("\nTesting allocation...");
  void* test_ptr = pmalloc(100000);  // 100KB
  
  if (test_ptr) {
    Serial.println("✅ Allocation successful");
    Serial.printf("Free after alloc: %u KB\n", rp2040.getFreePSRAMHeap() / 1024);
    free(test_ptr);
    Serial.printf("Free after free:  %u KB\n", rp2040.getFreePSRAMHeap() / 1024);
  } else {
    Serial.println("❌ Allocation failed");
  }
  
  Serial.println("\nPSRAM is ready for use!");
}

void loop() {
  // Nothing to do
}