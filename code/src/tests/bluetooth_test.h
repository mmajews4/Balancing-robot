#include "BluetoothSerial.h" // Include BluetoothSerial library

BluetoothSerial ESP_BT; // Create a BluetoothSerial object

void setup() {
  Serial.begin(115200);           // Start the Serial Monitor
  ESP_BT.begin("ESP32_BT");       // Bluetooth device name
  Serial.println("Bluetooth Initialized. Waiting for connections...");
}

void loop() {
  // Check if Bluetooth is connected
  if (ESP_BT.connected()) {
    // Send data to Bluetooth
    ESP_BT.println("Hello from ESP32!");
    Serial.println("Data sent to Bluetooth: Hello from ESP32!");

    delay(1000); // Wait for a second

    // Receive data from Bluetooth
    if (ESP_BT.available()) {
      String receivedData = ESP_BT.readString(); // Read incoming data
      Serial.println("Data received from Bluetooth: " + receivedData);
    }
  } else {
    Serial.println("Bluetooth not connected. Waiting...");
    delay(1000);
  }
}
