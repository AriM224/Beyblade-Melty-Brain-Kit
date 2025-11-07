void calcrpm()  // calculates the rpm based on g-force
{
  float gforce;
  // Sample current g-force
  gforce = fabs(get_accel_force_g());
  rpm = round(sqrt(gforce) * denom);

  // Update max_rpm if this is the highest so far
  if(rpm > max_rpm) 
  {
    max_rpm = rpm;
  }
}

long long spintimefunct()
{
  unsigned long long spintime;
  if(rpm > 200)
  {
    spintime = (60000000LL / rpm) * offset * heading; //calculates microseconds per revolution
  }
  else
  {
    spintime = 1000; //some random placeholder
  }
  return spintime;
}
float get_accel_force_g() 
{
  static const int NUM_SAMPLES = 10;  // average over last 10 readings
  static float samples[NUM_SAMPLES];
  static int index = 0;
  static bool filled = false;
  static float total = 0.0;

  if ((esp_timer_get_time() - loopstart) > 20000) 
  {
    float gforce;
    int16_t x, y, z;
    xl.readAxes(x, y, z);
    gforce = xl.convertToG(ACCEL_MAX_SCALE, y);
    loopstart = esp_timer_get_time();

    // update rolling average
    total -= samples[index];
    samples[index] = gforce;
    total += gforce;

    index = (index + 1) % NUM_SAMPLES;
    if (index == 0) filled = true;

    if (round(gforce) > max_gforce) {
      max_gforce = round(gforce);
    }
  }

  // return current average
  if (filled) return total / NUM_SAMPLES;
  else return total / (index + 1); // partial average while filling buffer
}
void rotation_angle()
{
  unsigned long long currentime;
  unsigned long long spintime;
  spintime = spintimefunct();
  currentime = esp_timer_get_time();
  //angle = ((currentime - ((irVisibleEnd + irVisibleStart) / 2) ) / spintime) * 360;
  angle = ((double)(currentime - previoustime) / (double)spintime) * 360.0;
  if((currentime - previoustime) >= spintime)  //resets timer every revolution
  {
    previoustime = currentime;
  }
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