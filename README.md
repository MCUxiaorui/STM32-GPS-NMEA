# STM32-GPS-NMEA Integration

This project demonstrates how to parse GPS data using MINMEA on an STM32 microcontroller. All the buffer handling and NMEA sentence tokenization is handled—you can use it in your project right away.

## Known Limitation

The main caveat: it's not synchronized with the UART output of the GPS module. You need to ensure DMA or interrupt-based reception is aligned with NMEA sentence boundaries (usually starts with `$`).

## How to Use

### 1. Initialize UART

Default UART settings (9600 baud, 8N1) are usually fine for GPS modules.

### 2. Set Up GPS Data Structure

```c
struct GPS_Data gpsData;
gpsData.bufferIndex = 0;
gpsData.receiving = false;
char uartBuffer[128];
````

### 3. Start Receiving

Call this in the init or loop:

```c
GPS_Data_Receive(&huart1, &gpsData);
```

### 4. Access Parsed Data

Once a complete sentence has been parsed:

```c
if (!gpsData.receiving) {
    float latitude     = gpsData.latitude;
    float longitude    = gpsData.longitude;
    float altitude     = gpsData.altitude;
    float speed        = gpsData.speed;

    uint8_t hours      = gpsData.time.hours;
    uint8_t minutes    = gpsData.time.minutes;
    uint8_t seconds    = gpsData.time.seconds;

    uint8_t day        = gpsData.date.day;
    uint8_t month      = gpsData.date.month;
    uint8_t year       = gpsData.date.year;

    int32_t true_deg   = gpsData.true_track_degrees.value / gpsData.true_track_degrees.scale;
    int32_t mag_deg    = gpsData.magnetic_track_degrees.value / gpsData.magnetic_track_degrees.scale;
    int32_t speed_kts  = gpsData.speed_knots.value / gpsData.speed_knots.scale;
    int32_t speed_kph  = gpsData.speed_kph.value / gpsData.speed_kph.scale;
}
```

## CMake Setup

### Enable `printf()` Support for Floats

```cmake
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "-Wl,-u,_printf_float"
)
```

### Add GPS Parsing Source File

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    lib/nmea/src/minmea.c
)
```

### Add Include Directories

```cmake
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    lib/nmea/inc
)
```

## Final Notes

* Tested on STM32F1 series using HAL + DMA.
* MINMEA is lightweight and doesn't require dynamic memory.
* Use an oscilloscope or logic analyzer if you have trouble syncing UART.

```
```
