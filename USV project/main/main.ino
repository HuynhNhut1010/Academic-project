/*************************************************************************************************************/
/*                                                                                                           */
/*                                                LIBRAIRIE                                                  */
/*                                                                                                           */
/*************************************************************************************************************/

/*******Library Server ********/
#include "ESP8266WiFi.h"       // for WIFI connexion
#include "FS.h"                // for mount file inside the flash
#include <ESP8266WebServer.h>  // for http communication
#include "stdlib.h"
#include <ESP8266HTTPClient.h>  // for http client server communication
#include <WiFiClient.h>         // for http client communication WIFI

/*******Library GPS ***************/
#include <TinyGPS++.h>       // for calculate the heading the distant between é Points GPS
#include <TimeLib.h>         // for the date and timeheadingActuel
#include <SoftwareSerial.h>  // for the communication serial withe GPS module


/*******Library compass***********/
#include <Wire.h>                // For the protocole I2C
#include <Adafruit_Sensor.h>     // For all sensor ardafruit
#include <Adafruit_HMC5883_U.h>  // for the MAGNETIC COMPAS


/*******Library IO Expand***********/
#include "PCF8574.h"


/*******Library Servo***********/
#include <Adafruit_PWMServoDriver.h>


/*******Library PID***********/
#include <PID_v1.h>

/*******Library ESP_now***********/
#include <espnow.h>
/*************************************************************************************************************/
/*                                                                                                           */
/*                                                VARIABLE                                                   */
/*                                                                                                           */
/*************************************************************************************************************/

// IP address and port number of the server (likely a Raspberry Pi)
String ipRas = "192.168.43.151";
String port = "8080";

/********* PWM Motor Configuration ************/
// GPIO pins for motor control
#define pinMotorR 2   // Right motor control pin
#define pinMotorL 0   // Left motor control pin
#define pinMotorT 14  // Turn motor control pin
#define pinDebug 0    // Debug pin for testing or diagnostics

// Variables related to motor control
int incomingAngle;          // Angle received for motor or servo control
String speed_edit = "0";    // String repreSEndation of edited speed
String speed_motorT = "0";  // String repreSEndation of motor T's speed

/********* Server Constants **************/
// Wi-Fi credentials for connecting to the server
const char* ssid = "redmi note";      // Wi-Fi network name
const char* password = "1234554321";  // Wi-Fi password

/******** Variables for GPS Route Handling *****/

// Route storage arrays for latitude and longitude (2D arrays)
const int nbLatLng = 15;                               // Maximum number of latitude and longitude points
double latLng[2][nbLatLng], latLng_temp[2][nbLatLng];  // Stores route and temporary points

// Index variables for managing GPS data
short i = 0;  // Current index for latitude
short j = 0;  // Current index for longitude

// Variables for parsing and handling messages
String Smessage;                  // String message
double Fmessage;                  // Floating-point message
String token;                     // Authentication token or GPS-related token
String SLong;                     // Longitude as a string
double FLat;                      // Latitude as a floating-point number
String SDec, SEnd, SLat, SPoint;  // Various temporary strings for data handling
int Latu = 0;                     // Temporary latitude value
bool Lat = false;                 // Latitude update flag
bool Long = false;                // Longitude update flag
double nextLong, nextLati;        // Coordinates for the next GPS point
double latHome, longHome;         // Coordinates for home location

/******** Variables for Web Communication *****/
// Variables for HTTP requests and responses
String httpPayload;     // Payload from the HTTP response
int httpCode;           // HTTP response code
WiFiClient clientEwen;  // Wi-Fi client object for communication
HTTPClient httpEwen;    // HTTP client object for handling requests
String adresseHttp;     // HTTP address for communication

/******** Variables for GPS *****/
// Time and date variables
#define time_offset 75600    // Offset to match UTC+7 (7 hours difference)
char Time[] = "00:00:00";    // Current time (hh:mm:ss)
char Date[] = "00-00-2000";  // Current date (dd-mm-yyyy)

// Variables for time tracking
byte last_second, Second, Minute, Hour, Day, Month;  // Components of time
int Year;                                            // Current year

// Default GPS coordinates
String Longitude = "106.626569";  // Default longitude
String Latitude = "10.757347";    // Default latitude

// GPS-related temporary variables
String alt, sat, fixage, course1;        // Altitude, satellites, fix age, course
String speed1 = "0";                     // Speed as a string
double LongitudeDouble, LatitudeDouble;  // GPS coordinates as doubles

/******** Variables for GPS Calculations ********/
unsigned long distanceToNextPoint;  // Distance to the next waypoint
double courseToNextPoint;           // Bearing to the next waypoint
short courseTo180;                  // Adjusted course for heading
short heading180;                   // Adjusted heading
short heading180_setpoint;          // Heading setpoint

/******** Variables for Compass ********/
float actualBoussole = 0;  // Current compass reading

/******** Variables for Motor Control ********/
int speed_set = 0;                   // Motor speed setpoint
bool babord = true, tribord = true;  // Flags for motor control (left/right)
bool arretMoteur = false;            // Motor stop flag

/******** Variables for Ultrasonic Sensor ********/
int distance;  // Measured distance from the ultrasonic sensor

/******** General Variables ********/
int state = 0;        // General state variable
int etatGPS = 1;      // GPS state
String status = "0";  // Status as a string

/******** Timers ********/
unsigned long timerDistance;  // Timer for distance measurements
unsigned long timerStop;      // Timer for motor stop
unsigned long timerGps;       // Timer for GPS updates
unsigned long timerDebug;     // Timer for debug operations
unsigned long timerPosition;  // Timer for position updates

/******** Variables for Servo Motors ********/
int pwm_temp = 90;    // Temporary PWM value for servos
#define SERVOMIN 80   // Minimum servo PWM value
#define SERVOMAX 600  // Maximum servo PWM value
#define SER0 6        // Pin for Servo Motor 0
#define SER1 7        // Pin for Servo Motor 1

// Variables for servo motor control
String round1;
int pwm0;
int pwm1;
int motorA = 0;                                      // Pin for motor A
int motorB = 1;                                      // Pin for motor B
int posDegrees;                                      // Servo position in degrees
int state_avodance = 0;                              // Obstacle avoidance state
int distance_1, distance_2, distance_3, distance_4;  // Distance sensor readings
int obtacle_distance = 70;                           // Obstacle detection threshold distance
int rever_servo = 0;                                 // Servo reversal flag

/******** Variables for PID Control ********/
double originalSetpoint = 175.3;     // Original setpoint for PID control
double setpoint = originalSetpoint;  // Current setpoint
double movingAngleOffset = 0.1;      // Offset for moving angle
double input, output;                // Input and output for PID controller
double Kp = 7;                       // Proportional gain
double Kd = 0;                       // Derivative gain
double Ki = 30;                      // Integral gain

// PID controller object with parameters
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


/*************************************************************************************************************/
/*                                                                                                           */
/*                                                  SETUP                                                    */
/*                                                                                                           */
/*************************************************************************************************************/

/*************Setup GPS**************************/
TinyGPSPlus gps;                   // The TinyGPS++ object
SoftwareSerial gpsSerial(13, 15);  // configuration gpsSerial library (RX pin, TX pin)

/*************Setup Compass*********************/
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
/

  /*************Setup sever**********************/
  ESP8266WebServer server(80);


/************Setup Wifi**************************/
WiFiClient client;
HTTPClient http;

/************Setup IO expand**************************/
PCF8574 pcf8574(0x38);
PCF8574 pcf_MUI(0x38);

/************Setup Driver servo**************************/
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);


/******Variable ESP_now*********/

uint8_t broadcastAddress[] = { 0x3C, 0xE9, 0x0E, 0xAD, 0xA9, 0xBC };  //MAC:3C:E9:0E:AD:BB:64
int val;
String success;
typedef struct struct_message {
  // int id;
  // int heading180_SEnd;
  int angle;
  int distance1;
  int distance2;
  int distance3;
  int distance4;
} struct_message;
int BOARD_ID = 1;
struct_message myData;
struct_message incomingReadings;

int timerPosition1 = 0;


void setup() {
  Serial.begin(115200);
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-180, 180);
  gpsSerial.begin(9600);
  analogWriteFreq(50);
  pinMode(pinMotorL, OUTPUT);
  pinMode(pinMotorR, OUTPUT);
  pca9685.begin();
  pca9685.setPWMFreq(50);
  if (!mag.begin())  // initiate the magnetic compas
  {

    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1)
      ;
  }

  Serial.print("Init pcf8574...");
  if (pcf8574.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }

  pcf8574.pinMode(P0, OUTPUT);
  pcf8574.pinMode(P1, OUTPUT);
  pcf8574.pinMode(P2, OUTPUT);
  pcf8574.pinMode(P3, OUTPUT);
  pcf8574.pinMode(P4, OUTPUT);
  pcf8574.pinMode(P5, OUTPUT);
  pcf8574.pinMode(P6, OUTPUT);
  pcf8574.pinMode(P7, OUTPUT);
  pcf_MUI.pinMode(P0, OUTPUT);
  pcf_MUI.pinMode(P1, OUTPUT);



  // Set ESP-NOW
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
    analogWrite(pinMotorL, 70);
    analogWrite(pinMotorR, 10);
    analogWrite(pinMotorT, 10);
  }
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSEnd);

  // Register peer
  //esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_CONTROLLER, 1, NULL, 0);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  analogWrite(pinMotorL, 70);
  analogWrite(pinMotorR, 10);
  analogWrite(pinMotorT, 10);
}

void loop() {

  // Periodically update the GPS position data and send it to the server
  if (timerPosition + 5000 < millis()) {  // Every 5000 ms (5 seconds)
    timerPosition = millis();             // Update the timer

    // Construct the HTTP request URL with GPS and sensor data
    adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/post-coor?lat=" + Latitude + "&lng=" + Longitude + "&distance_1=" + distance_1 + "&distance_2=" + distance_2 + "&distance_3=" + distance_3 + "&distance_4=" + distance_4 + "&heading180=" + heading180 + "&distance=" + distanceToNextPoint + "&speed=" + speed_set + "&j=" + j + "&speed1=" + speed1;

    Serial.println(adresseHttp);  // Print the HTTP request URL for debugging

    http.begin(client, adresseHttp);  // Initiate HTTP connection
    httpCode = http.POST("");         // Send the HTTP POST request

    if (httpCode > 0) {                // If the server responds successfully
      httpPayload = http.getString();  // Get the response payload
      Serial.print("httpPayload: ");
      Serial.println(httpPayload);

      // Extract data from the response and update variables
      status = getValue(httpPayload, '/', 0);        // Get status
      round1 = getValue(httpPayload, '/', 1);        // Get round information
      speed_edit = getValue(httpPayload, '/', 2);    // Get speed edit value
      speed_motorT = getValue(httpPayload, '/', 3);  // Get motor T speed

      // Debug prints for extracted data
      Serial.print("status: ");
      Serial.println(status);
      Serial.print("Round: ");
      Serial.println(round1);
      Serial.print("Speed edit: ");
      Serial.println(speed_edit);
      Serial.print("Speed set: ");
      Serial.println(speed_set);
      Serial.print("Speed set motor T: ");
      Serial.println(speed_motorT);

      if (status == "0") {           // If the status indicates "stop"
        state = 6;                   // Set state to "return home"
        analogWrite(pinMotorL, 70);  // Adjust motor speed
        analogWrite(pinMotorR, 10);
        analogWrite(pinMotorT, 10);
        Serial.println("Stop - Comeback home");
      } else {
        state = (state == 6) ? 0 : state;  // Reset state if previously stopped
        Serial.print("Keep going at state ");
        Serial.println(state);
      }
    }
    http.end();               // End the HTTP connection
    printIncomingReadings();  // Function to display sensor readings (presumed)
  }

  // Handle different operational states using a switch-case structure
  switch (state) {
    case 0:                               // Receive data from the database
      if (timerStop + 2000 < millis()) {  // Every 2000 ms (2 seconds)
        timerStop = millis();
        adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/get-all-coor?id=CO";  // API call to get GPS coordinates
        Serial.println(adresseHttp);
        http.begin(client, adresseHttp);
        httpCode = http.GET();             // Send HTTP GET request
        if (httpCode > 0) {                // If the server responds successfully
          httpPayload = http.getString();  // Get the response payload
          Serial.println(httpPayload);

          // Parse the GPS data from the response
          for (int i = 0; i <= 14; i++) {
            SPoint = getValue(httpPayload, '/', i);  // Extract each GPS point
            for (int k = 0; k <= 1; k++) {
              SLat = getValue(SPoint, '*', k);  // Split latitude and longitude
              SEnd = getValue(SLat, '.', 0);
              SDec = getValue(SLat, '.', 1);
              SLat = SEnd + SDec;  // Combine integer and decimal parts
              FLat = SLat.toDouble();
              for (int ha = 0; ha < SDec.length(); ha++) {
                FLat /= 10;  // Convert to double
              }
              latLng[k][i] = FLat;  // Store in the route array
              delay(20);            // Delay for stability
            }
          }
          displayArray();  // Display the GPS array (presumed function)
          http.end();      // End HTTP connection
          j = 0;           // Reset index
          state = 1;       // Transition to state 1
        }
      }
      break;

    case 1:                                  // Perform navigation tasks
      if (timerDistance + 700 < millis()) {  // Every 700 ms
        timerDistance = millis();
        nextLati = latLng[0][j];  // Get the next latitude
        nextLong = latLng[1][j];  // Get the next longitude

        Serial.print("State: ");
        Serial.println(state);

        calculOrderheadinging(magSensor());  // Update heading based on magnetic sensor
        calculSpeed();                       // Adjust speed
        Serial.print("distance1: ");
        Serial.println(distance_1);

        // Check for obstacles
        if (distance_1 < obtacle_distance || distance_2 < obtacle_distance || distance_3 < obtacle_distance || distance_4 < obtacle_distance) {
          Serial.println("Obstacle detected");
          state = 5;                   // Change state to obstacle avoidance
          analogWrite(pinMotorR, 10);  // Adjust motor speeds
          analogWrite(pinMotorL, 70);
          delay(1000);
        } else if (nextLati == 0 || nextLong == 0) {  // Check if route is complete
          state = (round1.toInt() == 1) ? 4 : 0;      // Transition state
          if (state == 0) {
            adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/delete-coor";  // API to delete route
            http.begin(client, adresseHttp);
            httpCode = http.POST("");  // Send POST request
            Serial.println(httpCode);
            j = 0;  // Reset index
            http.end();
          }
        } else if (distanceToNextPoint <= 3) {  // Check if the next point is reached
          Serial.println("Reach the point");
          j++;  // Move to the next point
        }
      }
      break;

    case 4:                               // Return to home
      double latLng_temp[2][14] = { 0 };  // Temporary array for route reversal
      // Reverse the route
      for (int h = 0; h <= 1; h++) {
        for (int k = 13, kk = 0; k >= 0; k--) {
          if (latLng[h][k] != 0) {
            latLng_temp[h][kk++] = latLng[h][k];
          }
        }
      }
      // Copy reversed route back to latLng
      for (int k = 0; k <= 14; k++) {
        for (int h = 0; h <= 1; h++) {
          latLng[h][k] = latLng_temp[h][k];
        }
      }
      state = 1;  // Transition to navigation state
      j = 0;      // Reset index
      break;

    case 5:                 // Obstacle avoidance
      obstacle_avodance();  // Call obstacle avoidance function (presumed)
      break;
  }

  // Process incoming GPS data
  while (gpsSerial.available() > 0) {
    encodeGPS();  // Parse GPS data (presumed function)
  }
}
// Function to handle the status of the last sent packet
void OnDataSEnd(uint8_t* mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");  // Packet successfully delivered
  } else {
    Serial.println("Delivery fail");  // Packet delivery failed
  }
}

// Function to handle incoming data via ESP-NOW
void OnDataRecv(uint8_t* mac_addr, uint8_t* incomingData, uint8_t len) {
  // Copies the sender's MAC address into a string for debugging
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  // Copies the incoming data into the `incomingReadings` structure
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  // Extract data from the structure and assign to respective variables
  distance_1 = incomingReadings.distance1;  // Sensor reading from distance sensor 1
  distance_2 = incomingReadings.distance2;  // Sensor reading from distance sensor 2
  distance_3 = incomingReadings.distance3;  // Sensor reading from distance sensor 3
  distance_4 = incomingReadings.distance4;  // Sensor reading from distance sensor 4
  incomingAngle = incomingReadings.angle;   // Angle data from the sender
}

// Function to print the incoming sensor readings for debugging
void printIncomingReadings() {
  Serial.println("INCOMING READINGS");  // Header for debug output
  Serial.print("Distance1: ");
  Serial.println(distance_1);  // Print sensor 1 reading
  Serial.print("Distance2: ");
  Serial.println(distance_2);  // Print sensor 2 reading
  Serial.print("Distance3: ");
  Serial.println(distance_3);  // Print sensor 3 reading
  Serial.print("Distance4: ");
  Serial.println(distance_4);  // Print sensor 4 reading
  Serial.print("Angle: ");
  Serial.println(incomingAngle);  // Print angle reading
}
