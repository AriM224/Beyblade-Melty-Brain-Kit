void updateLED() 
{
  unsigned long currentMillis = millis();

  // advance animation index only on interval
  if (currentMillis - lastUpdate > animSpeed) 
  {
    animIndex += direction;

    // bounce logic for chase animation
    if (animIndex >= 9) {
      animIndex = 9;
      direction = -1;
    } else if (animIndex <= 0) {
      animIndex = 0;
      direction = 1;
    }

    lastUpdate = currentMillis;
  }

  // LED logic depending on status
  if (LEDStatus == "failsafe") 
  {
    if (rc_status == false) 
    {
      // yellow bounce chase
      top_strip.clear();
      bottom_strip.clear();
      top_strip.setPixelColor(animIndex, top_strip.Color(100, 80, 0));
      bottom_strip.setPixelColor(animIndex, bottom_strip.Color(100, 80, 0));
    }
    else
    {
    // breathing blue effect
    // use millis() directly for smoother breathing
    float phase = (millis() % 2000) / 2000.0 * 2 * PI; // 0 to 2π every 2 sec
    int brightness = (sin(phase) * 50) + 50; // sine wave 0–255

    uint32_t color = top_strip.Color(0, 0, brightness);
    top_strip.fill(color, 0, 10);
    bottom_strip.fill(color, 0, 10);
    }
  }
  else if (LEDStatus == "export")
  {
    top_strip.fill(top_strip.Color(100, 100, 100), 0, 10); 
    bottom_strip.fill(bottom_strip.Color(100, 100, 100), 0, 10);
  }
  else if (LEDStatus == "armed")
  {
    if((millis() - stripstart) > 2000)
    {
    stripstart = millis();
    top_strip.clear();
    bottom_strip.clear();
    top_strip.fill(top_strip.Color(100, 0, 0), 0, round(10 - (abs(volts - 12.4) * 5))); 
    bottom_strip.fill(bottom_strip.Color(100, 0, 0), 8 -round(8 - (abs(volts - 12.4) * 5)), 8);
    }
  }
  else if (LEDStatus == "heading on") 
  {
    if (volts < 10.8) {
      top_strip.fill(top_strip.Color(255, 165, 0), 0, 10); 
      bottom_strip.fill(bottom_strip.Color(255, 165, 0), 0, 10);
    } else {
      top_strip.fill(top_strip.Color(0, 255, 0), 0, 10);
      bottom_strip.fill(bottom_strip.Color(0, 255, 0), 0, 10);
    }
  }
  else if (LEDStatus == "motor on") 
  {
    top_strip.fill(top_strip.Color(0, 0, 255), 0, 10);
    bottom_strip.fill(bottom_strip.Color(0, 0, 255), 0, 10);
  }
  else 
  {
    top_strip.clear();
    bottom_strip.clear();
  }

  top_strip.show();
  bottom_strip.show();
}