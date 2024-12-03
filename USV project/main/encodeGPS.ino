/*
 * Processes GPS data to extract time, date, location, and other relevant information.
 * Updates system time and date arrays for use in other parts of the program.
 */
void encodeGPS() {
  // Attempt to encode data from the GPS module
  if (gps.encode(gpsSerial.read())) {
    // If valid time data is available from GPS
    if (gps.time.isValid()) {
      Minute = gps.time.minute();  // Extract minute
      Second = gps.time.second();  // Extract second
      Hour = gps.time.hour();      // Extract hour
      // Serial.println("Time Valid"); // Optional debug statement
    }

    // If valid date data is available from GPS
    if (gps.date.isValid()) {
      Day = gps.date.day();      // Extract day
      Month = gps.date.month();  // Extract month
      Year = gps.date.year();    // Extract year
      // Serial.println("Date Valid"); // Optional debug statement
    }

    // If valid location data is available from GPS
    if (gps.location.isValid()) {
      Longitude = String(gps.location.lng(), 7);  // Extract longitude as a string with 7 decimals
      LongitudeDouble = gps.location.lng();       // Extract longitude as a double
      Latitude = String(gps.location.lat(), 7);   // Extract latitude as a string with 7 decimals
      LatitudeDouble = gps.location.lat();        // Extract latitude as a double
      etatGPS = 2;                                // Set GPS state to indicate valid location data
    } else {
      etatGPS = 1;  // Set GPS state to indicate invalid location data
    }

    // Check and update fix status
    if (gps.location.isValid()) {
      fixage = "OK";  // Indicate a successful GPS fix
    }

    // If valid speed data is available from GPS
    if (gps.speed.isValid()) {
      speed1 = String(gps.speed.kmph(), 2);  // Extract speed in km/h as a string with 2 decimals
    }

    // Update system time and date only when the second changes
    if (last_second != gps.time.second()) {
      last_second = gps.time.second();  // Store the new second value

      setTime(Hour, Minute, Second, Day, Month, Year);  // Set system time using extracted GPS data
      adjustTime(time_offset);                          // Adjust time based on a predefined offset

      // Update the Time array with current system time
      Time[0] = hour() / 10 + '0';    // Tens digit of hour
      Time[1] = hour() % 10 + '0';    // Units digit of hour
      Time[3] = minute() / 10 + '0';  // Tens digit of minute
      Time[4] = minute() % 10 + '0';  // Units digit of minute
      Time[6] = second() / 10 + '0';  // Tens digit of second
      Time[7] = second() % 10 + '0';  // Units digit of second

      // Update the Date array with current system date
      Date[0] = day() / 10 + '0';          // Tens digit of day
      Date[1] = day() % 10 + '0';          // Units digit of day
      Date[3] = month() / 10 + '0';        // Tens digit of month
      Date[4] = month() % 10 + '0';        // Units digit of month
      Date[8] = (year() / 10) % 10 + '0';  // Tens digit of year
      Date[9] = year() % 10 + '0';         // Units digit of year
    }
  }
}
