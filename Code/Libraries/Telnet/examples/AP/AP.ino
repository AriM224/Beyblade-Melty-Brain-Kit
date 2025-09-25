#include <Telnet.h>

Telnet telnet;

// Command handler to turn the LED on
void ledOn(WiFiClient &client, const String &args) {
    client.println("Turning the LED ON...");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // LED is active LOW
    client.println("LED is now ON.");
}

// Command handler to turn the LED off
void ledOff(WiFiClient &client, const String &args) {
    client.println("Turning the LED OFF...");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // LED is active LOW
    client.println("LED is now OFF.");
}

// Command handler to show the Wi-Fi connection details
void showWiFiDetails(WiFiClient &client, const String &args) {
    client.println("Wi-Fi Details:");
    client.println("  SSID: " + WiFi.SSID());
    client.println("  IP Address: " + WiFi.localIP().toString());
    client.println("  Signal Strength: " + String(WiFi.RSSI()) + " dBm");
}

void setup() {
    Serial.begin(115200);

    // Start the Telnet server in AP mode
    telnet.beginAP("MyESP8266Network", "MySecretPassword");
    
    telnet.setPrompt("root", "esp");

    // Start the Telnet server
    telnet.startServer();

    // Add commands
    telnet.addCommand("ledon", ledOn, "Turns the LED ON.");
    telnet.addCommand("ledoff", ledOff, "Turns the LED OFF.");
    telnet.addCommand("wifi", showWiFiDetails, "Shows current Wi-Fi connection details.");

    Serial.println("Waiting for a client to connect...");
}

void loop() {
    // Handle Telnet client connections and commands
    telnet.handleClient();
}
