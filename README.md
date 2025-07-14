# STM32-GPS-NMEA

The GPS parser is written by MINMEA. I got the buffer thing and the whole string to token craziness taken care off. There are plenty that can be improved but this is good enough to use in my project. 

The main problem is that it is not synced to the uart out of GPS module. 

To use this example and to use GPS in general with STM32.
1. initialize uart default setting is fine.
2. initialize   struct GPS_Data gpsData;
  gpsData.bufferIndex = 0;
  gpsData.receiving = false;
  char uartBuffer[128];
3. Call "GPS_Data_Receive(&huart1, &gpsData);"
4. Then you are good to go
      if (!gpsData.receiving) {
      float latitude = gpsData.latitude;
      float longitude = gpsData.longitude;
      float altitude = gpsData.altitude;
      float speed = gpsData.speed;
      uint8_t hours = gpsData.time.hours;
      uint8_t minutes = gpsData.time.minutes;
      uint8_t seconds = gpsData.time.seconds;
      uint8_t day = gpsData.date.day;
      uint8_t month = gpsData.date.month;
      uint8_t year = gpsData.date.year;
      int_least32_t True_track_deg = gpsData.true_track_degrees.value / gpsData.true_track_degrees.scale;
      int_least32_t Magnetic_track_deg = gpsData.magnetic_track_degrees.value / gpsData.magnetic_track_degrees.scale;
      int_least32_t speed_knots = gpsData.speed_knots.value / gpsData.speed_knots.scale;
      int_least32_t speed_kph = gpsData.speed_kph.value / gpsData.speed_kph.scale;
