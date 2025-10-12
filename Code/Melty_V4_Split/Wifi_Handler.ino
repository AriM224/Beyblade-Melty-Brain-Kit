void wifi_mode()   //turns on wifi mode to connect wirelessly
{
  bool exportdata = false;
  failsafe();
  esp_task_wdt_init(WDT_TIMEOUT, false);  // disable watchdog
  WiFi.softAP(ssid, password);
  //Serial.print("Wifi Mode");
  ArduinoOTA.begin();  //starts ota
  while(duty[5] > 50)  //stuck in loop so robot cannot run while in wifi mode, channel 5 initiates wifi
  {
    failsafe();
    update_channels();
    ArduinoOTA.handle();
    if(duty[6] < 50)  //if export switch is not triggered, will not export
    {
      exportdata = false;
    }
    if((duty[6] > 50) && (exportdata == false)) //if export switch is triggered and it has not exported yet, call export function
    {
      LEDStatus = "export";
      updateLED();
      data_export();
      exportdata = true;
      escR.init();
      delayMicroseconds(200);
      escL.init();
      for (int i = 0; i < 2; i++)
	    {
        delay(1);
		    escR.beep(i);
        delay(1);
        escL.beep(i);
	    }
    }
  }
  WiFi.softAPdisconnect(false);  //turn off wifi
  esp_task_wdt_init(WDT_TIMEOUT, true);  // enable watchdog
}

void data_export()    //exports data to telnet client for diagnostics, wifi mode must be turned on first
{
  //Serial.println("data export");
  TelnetStream.begin(); //start telnet
  delay(2000);
    // Print data
  TelnetStream.println("Telnet stream started");
  TelnetStream.println("==== Sensor Data Export ====");
  TelnetStream.print("Max RPM: ");
  TelnetStream.println(max_rpm);
  TelnetStream.print("heading offset: ");
  TelnetStream.println(offset);
  TelnetStream.print("battery: ");
  TelnetStream.println(volts);
  TelnetStream.print("Max G-Force: ");
  TelnetStream.println(max_gforce);
  TelnetStream.println("============================");
  TelnetStream.stop(); // Close telnet connection
}
