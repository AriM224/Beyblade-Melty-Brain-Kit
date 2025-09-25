#include <Telnet.h>

Telnet telnet;

// Command handler for turning the LED on
void ledOn(WiFiClient &client, const String &args) {
    client.println("Turning the LED ON...");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // LED is active LOW
    client.println("LED is now ON.");
}

// Command handler for turning the LED off
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

    // Connect to Wi-Fi in Client Mode
    const char *ssid = "YOURSSID"; // FBI Surveillance Van 4
    const char *password = "YOURPWD";

    telnet.beginClient(ssid, password);

    // Wait until connected to Wi-Fi
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi!");
    Serial.println("IP Address: " + WiFi.localIP().toString());

    telnet.setPrompt("root", "esp");

    // Start Telnet server on port 23
    telnet.startServer();

    // Add commands
    telnet.addCommand("ledon", ledOn, "Turns the LED ON.");
    telnet.addCommand("ledoff", ledOff, "Turns the LED OFF.");
    telnet.addCommand("wifi", showWiFiDetails, "Shows current Wi-Fi connection details.");
}

void loop() {
    // Handle Telnet client connections and commands
    telnet.handleClient();
}
