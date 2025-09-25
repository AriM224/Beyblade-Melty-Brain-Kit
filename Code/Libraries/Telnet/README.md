# **Telnet**

A lightweight and flexible library for implementing Telnet servers on ESP8266/ESP32-based devices. The library supports both Access Point (AP) and Client modes for seamless wireless communication. It allows you to create custom prompts, handle client connections, register commands, and manage Telnet sessions with ease. This library is ideal for controlling your ESP device over Telnet with custom commands and aliases.

## **Features**

- **Access Point Mode**: Start your device as a Wi-Fi access point to allow clients to connect directly.
- **Client Mode**: Connect your device to a Wi-Fi network and allow remote Telnet access.
- **Custom Prompts**: Configure custom prompts for each client connection.
- **Command Registration**: Register custom commands that clients can execute via Telnet.
- **Command Aliases**: Define command aliases for easier and quicker access to frequently used commands.
- **Help Command**: Built-in help command that lists all registered commands and their descriptions.
- **Client Management**: Manage multiple client connections and handle command execution for each client.

## **Installation**

1. Download the library as a ZIP file.
2. Open the Arduino IDE and navigate to **Sketch** > **Include Library** > **Add .ZIP Library...**.
3. Select the downloaded `.zip` file and click **Open**.
4. The library will be added to the Arduino IDE and available for use in your projects.

## **Usage Example**

### **1. Example - Access Point Mode with Custom Prompt**

```cpp
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
```

### **2. Example - Client Mode with Custom Prompt**

```cpp
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
```

## **API Reference**

### **Telnet**

#### **Methods**

- **`Telnet()`**: Constructor to create an instance of the `Telnet` class.

- **`beginAP(const char *ssid, const char *password)`**: Starts the device in Access Point mode with the specified SSID and password.

- **`beginClient(const char *ssid, const char *password)`**: Connects the device to a Wi-Fi network in Client mode.

- **`startServer(uint16_t port)`**: Starts the Telnet server on the specified port.

- **`stopServer()`**: Stops the Telnet server and closes all client connections.

- **`handleClient()`**: Handles incoming client connections and processes their input.

- **`addCommand(const char *cmd, std::function<void(WiFiClient &client, const String &args)> handler, const char *help)`**: Registers a new command with a handler function and an optional help description.

- **`setAlias(const char *alias, const char *cmd)`**: Sets an alias for an existing command.

- **`setPrompt(const char *username, const char *deviceName)`**: Sets a custom prompt to be displayed when the client is connected.

- **`setTimeout(uint32_t timeoutMillis)`**: Sets the timeout for client connections.

- **`showBanner(WiFiClient &client)`**: Displays a banner message when a client connects.

- **`showPrompt(WiFiClient &client)`**: Displays the prompt for user input.

- **`handleCommand(WiFiClient &client, const String &input)`**: Processes the command received from the client.

- **`showHelp(WiFiClient &client, const String &args)`**: Displays a list of all registered commands and their descriptions.

## **Customization**

- You can customize the banner that is displayed when a client connects by modifying the `banner` member variable. For example:

```cpp
telnetServer.banner = "Custom Banner Text"; // WE HAVE A COOL BANNER SO DONT SET IT
```

- You can set a custom prompt with the `setPrompt` method. The prompt will be displayed before each command input:

```cpp
telnetServer.setPrompt("username", "ESP32_Device");
```

## **Contributing**

Feel free to fork this repository and submit pull requests with improvements, bug fixes, or new features. Please make sure to follow the coding standards used in this library, and include tests when applicable.

## **License**

This library is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
