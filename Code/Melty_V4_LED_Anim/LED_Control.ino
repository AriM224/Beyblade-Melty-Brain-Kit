void updateLED() 
{
  unsigned long currentMillis = millis();

  // advance animation index only on interval
  if (currentMillis - lastUpdate > animSpeed) {
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
  if (LEDStatus == "failsafe") {
    // breathing red effect
    // use millis() directly for smoother breathing
    float phase = (millis() % 2000) / 2000.0 * 2 * PI; // 0 to 2π every 2 sec
    int brightness = (sin(phase) * 127) + 128; // sine wave 0–255

    uint32_t color = top_strip.Color(brightness, 0, 0);
    top_strip.fill(color, 0, 10);
    bottom_strip.fill(color, 0, 10);
  }
  else if (LEDStatus == "armed") {
    // green bounce chase
    top_strip.clear();
    bottom_strip.clear();
    top_strip.setPixelColor(animIndex, top_strip.Color(0, 255, 0));
    bottom_strip.setPixelColor(animIndex, bottom_strip.Color(0, 255, 0));
  }
  else if (LEDStatus == "heading on") {
    if (volts < 11.2) {
      top_strip.fill(top_strip.Color(255, 165, 0), 0, 10); 
      bottom_strip.fill(bottom_strip.Color(255, 165, 0), 0, 10);
    } else {
      top_strip.fill(top_strip.Color(0, 0, 255), 0, 10);
      bottom_strip.fill(bottom_strip.Color(0, 0, 255), 0, 10);
    }
  }
  else {
    top_strip.clear();
    bottom_strip.clear();
  }

  top_strip.show();
  bottom_strip.show();
}

bool isAngleInRange(float center, float range)
{
  center = fmod((center + 360), 360);

  // Compute smallest difference, -180..180
  float diff = fabs(fmod((angle - center + 540), 360) - 180);

  return diff <= range; // within ±range degrees
}
void heading_funct()
{
  if(reversed == true)
  {
    heading = (map(duty[1], 0, 100, 50, -50) * 0.001) + 1;
  }
  else
  {
    heading = (map(duty[1], 0, 100, -50, 50) * 0.001) + 1;
  }
}
void heading_adj()
{
  if(offset <= 0.5)
  {
    offset = 1.0;
  }
  if(duty[4] < 30 && off_set == false)
  {
    offset = offset + 0.001;
    EEPROM.put(OFFSET_ADDR, offset);
    EEPROM.commit();  // Required on ESP32 to finalize the write
    off_set = true;
  }
  else if(duty[4] > 70 && off_set == false)
  {
    offset = offset - 0.001;
    EEPROM.put(OFFSET_ADDR, offset);
    EEPROM.commit();  // Required on ESP32 to finalize the write
    off_set = true;
  }
  else if(duty[4] > 40 && duty[4] < 60)
  {
    off_set = false;
  }
}

float readOffsetFromEEPROM()
{
  float val;
  EEPROM.get(OFFSET_ADDR, val);
  if (isnan(val) || abs(val) > 360.0) { // sanity check
    return 0.0;
  }
  return val;
}