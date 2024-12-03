// Function to extract a value from a string based on a separator and index
String getValue(String data, char separator, int index) {
  int found = 0;                     // Counter for found separators
  int strIndex[] = { 0, -1 };        // Array to hold start and end indices of the substring
  int maxIndex = data.length() - 1;  // Index of the last character in the string

  // Iterate through the string to find the separator or end of string
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    // Check if the current character is the separator or we're at the last character
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;  // Increment the counter of found separators

      // Update the start index for the next value
      strIndex[0] = strIndex[1] + 1;

      // If we're at the end of the string, include it as part of the substring
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  // If the desired index was found, return the substring
  // Otherwise, return an empty string
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
