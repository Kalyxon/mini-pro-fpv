#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin definitions for Arduino Uno
#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Testing nRF24L01 Module ===");
  Serial.println("Make sure VCC is connected to 3.3V!");
  Serial.println("===============================\n");
  
  delay(1000);
  
  // Initialize radio
  Serial.print("1. Initializing radio... ");
  bool radioStarted = radio.begin();
  
  if (radioStarted) {
    Serial.println("SUCCESS");
  } else {
    Serial.println("FAILED");
    Serial.println("Check:");
    Serial.println("- VCC to 3.3V (NOT 5V)");
    Serial.println("- GND connected");
    Serial.println("- SPI pins (9,10,11,12,13)");
    while(1);
  }
  
  // Check if chip is connected
  Serial.print("2. Checking chip connection... ");
  if (radio.isChipConnected()) {
    Serial.println("CHIP DETECTED");
  } else {
    Serial.println("NO CHIP DETECTED");
    Serial.println("Module may be damaged or wiring wrong");
  }
  
  // Test radio settings
  Serial.print("3. Testing settings... ");
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  Serial.println("OK");
  
  // Print details
  Serial.println("\n=== nRF24L01 Test Complete ===");
  if (radioStarted && radio.isChipConnected()) {
    Serial.println("✅ Module is WORKING");
    Serial.println("Connect to 3.3V for future use!");
  } else {
    Serial.println("❌ Module may be DAMAGED");
    Serial.println("Check wiring or replace module");
  }
}

void loop() {
  // Blink LED to show program running
  static bool ledState = false;
  digitalWrite(LED_BUILTIN, ledState);
  ledState = !ledState;
  delay(500);
}