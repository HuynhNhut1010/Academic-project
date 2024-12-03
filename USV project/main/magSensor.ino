// Function to read data from the magnetometer and calculate the heading
float magSensor() {
  // Declare an event object to store sensor data
  sensors_event_t event;

  // Get a new event from the magnetometer sensor (assumed to be 'mag')
  mag.getEvent(&event);

  // Calculate the heading based on the magnetometer's X and Y axis readings
  // atan2() calculates the angle in radians between the point (x, y) and the origin
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // The declination angle is the angular difference between magnetic north and true north
  // Here it is set to 1.3 degrees for correction based on your location's magnetic declination
  float declinationAngle = 1.3;

  // Add the declination angle to adjust the heading
  heading += declinationAngle;

  // If the heading is negative, adjust it by adding 360 degrees (2 * PI radians)
  if (heading < 0)
    heading += 2 * PI;

  // If the heading exceeds 360 degrees (2 * PI radians), subtract 360 degrees to wrap it back into range
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert the heading from radians to degrees for readability
  float headingDegrees = heading * 180 / M_PI;

  // Map the heading so that 360 degrees (north) becomes 0, 90 degrees (east) becomes 90, etc.
  headingDegrees = map(headingDegrees, 0, 360, 360, 0);

  // Print the heading in degrees to the Serial Monitor for debugging
  Serial.print("Heading (degrees): ");
  Serial.println(headingDegrees);

  // Return the heading in degrees
  return headingDegrees;
}
