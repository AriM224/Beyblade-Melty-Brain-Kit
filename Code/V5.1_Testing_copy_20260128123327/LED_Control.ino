void updateLED() 
{
  unsigned long currentMillis = millis();
  int graph = 0;
  const uint32_t ringColors[] = {
  top_strip.Color(100, 0, 0),   // red
  top_strip.Color(100, 80, 0),  // yellow
  top_strip.Color(0, 100, 0),   // green
  top_strip.Color(0, 80, 100),  // cyan
  top_strip.Color(80, 0, 100)   // purple
  };

  const int NUM_COLORS = sizeof(ringColors) / sizeof(ringColors[0]);

  // advance animation index only on interval
  if (currentMillis - lastUpdate > animSpeed) 
  {
    animIndex += direction;

    // bounce logic for chase animation
    if (animIndex >= NUMPIXELST) {
      animIndex = NUMPIXELST;
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
    top_strip.fill(color, 0, NUMPIXELST);
    bottom_strip.fill(color, 0, NUMPIXELST);
    }
  }
  else if (LEDStatus == "export")
  {
    top_strip.fill(top_strip.Color(100, 100, 100), 0, NUMPIXELST); 
    bottom_strip.fill(bottom_strip.Color(100, 100, 100), 0, NUMPIXELST);
  }
  else if (LEDStatus == "armed")
  {
    if((millis() - stripstart) > 2000)
    {
    stripstart = millis();
    top_strip.clear();
    bottom_strip.clear();
    if(volts > 14)
    {
      graph = round(NUMPIXELST - (abs((volts/4.0) - 4.2) * NUMPIXELST * 1.4));
    }
    else
    {
      graph = round(NUMPIXELST - (abs((volts/3.0) - 4.2) * NUMPIXELST * 1.4));
    }
    if(rvs_led_B == true)
    {
      bottom_strip.fill(bottom_strip.Color(100, 15, 0), NUMPIXELST - graph, NUMPIXELST);
    }
    else
    {
      bottom_strip.fill(bottom_strip.Color(100, 15, 0), 0, graph);
    }
    if(rvs_led_T == true)
    {
      top_strip.fill(top_strip.Color(100, 0, 0), NUMPIXELST - graph, NUMPIXELST); 
    }
    else
    {
      top_strip.fill(top_strip.Color(100, 0, 0), 0, graph);
    }
    }
  }
  else if (LEDStatus == "heading on") 
  {
    if (volts < 10.8) {
      top_strip.fill(top_strip.Color(255, 100, 0), 0, 10); 
      bottom_strip.fill(bottom_strip.Color(255, 100, 0), 0, 10);
    } else {
      top_strip.fill(top_strip.Color(0, 255, 0), 0, 10);
      bottom_strip.fill(bottom_strip.Color(0, 255, 100), 0, 10);
    }
  }
  else if (LEDStatus == "motor on") 
  {
    top_strip.fill(top_strip.Color(80, 0, 0), 0, 10);
    bottom_strip.fill(bottom_strip.Color(80, 0, 0), 0, 10);
  }
  else if (LEDStatus == "celebrate")
  {
    if (currentMillis - lastUpdate > animSpeed)
    {
      ringRadius++;

      if (ringRadius >= NUMPIXELST)
      {
        ringRadius = 0;
        ringColorIndex = (ringColorIndex + 1) % NUM_COLORS;
      }

      lastUpdate = currentMillis;
    }

    top_strip.clear();
    bottom_strip.clear();

    uint32_t c = ringColors[ringColorIndex];

    top_strip.setPixelColor(ringRadius, c);
    bottom_strip.setPixelColor(ringRadius, c);
  }
  
  else if (LEDStatus == "boot") //not used
  {
    top_strip.fill(top_strip.Color(50, 50, 50), 0, 10);
    bottom_strip.fill(bottom_strip.Color(50, 50, 50), 0, 10);
  }
  else 
  {
    top_strip.clear();
    bottom_strip.clear();
  }

  top_strip.show();
  bottom_strip.show();
}