
#define BLYNK_TEMPLATE_ID "TMPL6Dxh-Qly-"
#define BLYNK_TEMPLATE_NAME "esp32cam"
#define BLYNK_AUTH_TOKEN "r0K50xn48-98f57siNDKkSR6KvquSjdo"
#define BLYNK_PRINT Serial
// Enter your WiFi ssid and password
const char* ssid = "New wifi-cam";                                                                                  //your network SSID
const char* password = "123dat123";                                                                             //your network password
String myScript = "/macros/s/AKfycbxPghpYKHvdT-Y9Qp3mfc4vtiewke3YoLQB1e1eVr-0E5OMuboA88B9-HoG1kaJ5hiLEA/exec";  //Create your Google Apps Script and replace the "myScript" path.
String myLineNotifyToken = "myToken=**********";                                                                //Line Notify Token. You can set the value of xxxxxxxxxx empty if you don't want to send picture to Linenotify.
String myFoldername = "&myFoldername=ESP32-CAM";
String myFilename = "&myFilename=ESP32-CAM.jpg";
String myImage = "&myFile=";
int led = 33;
int temp_pos1 = 90;
int temp_pos2 = 90;
#include <BlynkSimpleEsp32.h>
//#include "thingProperties.h"
//#include "arduino_secrets.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Base64.h"
#include "esp_camera.h"
#include <SPI.h>
#include <ESP32Servo.h>

void startCameraServer();
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled

//servo
#define SERVO_1 14
#define SERVO_2 15

#define SERVO_STEP 5

Servo servoN1;
Servo servoN2;
Servo servo1;
Servo servo2;

int servo1Pos = 90;
int servo2Pos = 90;
int sercurity = 0;
int capture = 0;
int reset=0;
int capture_1=0;


//blynk

// #define BLYNK_TEMPLATE_ID "TMPL6Dxh-Qly-"
// #define BLYNK_TEMPLATE_NAME "esp32cam"
// #define BLYNK_AUTH_TOKEN "r0K50xn48-98f57siNDKkSR6KvquSjdo"


char auth[] = BLYNK_AUTH_TOKEN;
#define LED_PIN 33


//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
IPAddress dns (8, 8, 8, 8);
// Set your Static IP address
IPAddress local_IP(192, 168, 0, 26);
// Set your Gateway IP address
IPAddress gateway(192, 168, 0, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress Blynk_Server_IP;
void setup() {
  servo1.setPeriodHertz(50);  // standard 50 hz servo
  servo2.setPeriodHertz(50);  // standard 50 hz servo
  servoN1.attach(2, 544, 2400);
  servoN2.attach(13, 544, 2400);

  servo1.attach(SERVO_1, 544, 2400);
  servo2.attach(SERVO_2, 544, 2400);

  servo1.write(servo1Pos);
  servo2.write(servo2Pos);

  pinMode(33, OUTPUT);
  pinMode(13, INPUT);
  digitalWrite(33, 0);
  //digitalWrite(13,0);


  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(100);
  WiFi.config(local_IP, gateway, subnet,dns);
  WiFi.begin(ssid, password);
  Blynk.config(auth);
  while (Blynk.connect() == false) { 
  }  
  WiFi.hostByName("blynk-cloud.com", Blynk_Server_IP);
  Serial.print("blynk.cloud.com IP address is ");
  Serial.println(Blynk_Server_IP);

  // WiFi.mode(WIFI_STA);
  // Serial.println("");
  // Serial.print("Connecting to ");
  // Serial.println(ssid);
  // WiFi.begin(ssid, password);
  long int StartTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if ((StartTime + 10000) < millis()) break;
  }
  Serial.println("");
  startCameraServer();
  Serial.println("STAIP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reset");

    ledcAttachPin(4, 3);
    ledcSetup(3, 5000, 8);
    ledcWrite(3, 10);
    delay(200);
    ledcWrite(3, 0);
    delay(200);
    ledcDetachPin(3);

    delay(1000);
    ESP.restart();
  } else {
    ledcAttachPin(4, 3);
    ledcSetup(3, 5000, 8);
    for (int i = 0; i < 5; i++) {
      ledcWrite(3, 10);
      delay(200);
      ledcWrite(3, 0);
      delay(200);
    }
    ledcDetachPin(3);
  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  //drop down frame size for higher initial frame rate
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {

  Blynk.run();
  if (sercurity == 1) {
    if (digitalRead(13) == LOW) {
      digitalWrite(33, 1);
    } else {
      digitalWrite(33, 0);
      SendCapturedImage();
    }
  }
  if(capture_1==1)
  {
    Blynk.disconnect();
    SendCapturedImage();
    Blynk.connect();
    delay(3000);
  }
  if(reset==1){ESP.restart();}
  if (WiFi.status() != WL_CONNECTED){
    ESP.restart();
  }
}
BLYNK_WRITE(V0) {  //đọc gtri nút từ app blynk
  capture = param.asInt();
  if(capture)
  {
    capture_1=1;
  }
  else{
    capture_1=0;
  }
  delay(500);
}
BLYNK_WRITE(V1) {  //đọc gtri nút từ app blynk
  servo1Pos = param.asInt();
  if (temp_pos1 > servo1Pos) {
    while (temp_pos1 > servo1Pos) {
      servo1.write(temp_pos1);
      temp_pos1--;
      Serial.println(temp_pos1);
      delay(20);
    }
    temp_pos1 = servo1Pos;
  } else {
    while (temp_pos1 < servo1Pos) {
      servo1.write(temp_pos1);
      temp_pos1++;
      Serial.println(temp_pos1);
      delay(20);
    }
    temp_pos1 = servo1Pos;
  }
  delay(500);
}
BLYNK_WRITE(V2) {  //đọc gtri nút từ app blynk
  servo2Pos = param.asInt();
  if (temp_pos2 > servo2Pos) {
    while (temp_pos2 > servo2Pos) {
      servo2.write(temp_pos2);
      temp_pos2--;
      Serial.println(temp_pos2);
      delay(20);
    }
    temp_pos2 = servo2Pos;
  } else {
    while (temp_pos2 < servo2Pos) {
      servo2.write(temp_pos2);
      temp_pos2++;
      Serial.println(temp_pos2);
      delay(20);
    }
    temp_pos2 = servo2Pos;
  }
  delay(500);
}

BLYNK_WRITE(V3) {  //đọc gtri nút từ app blynk
  sercurity = param.asInt();
  Serial.println(sercurity);
}
BLYNK_WRITE(V4) {  //đọc gtri nút từ app blynk
  reset = param.asInt();
}
String SendCapturedImage() {
  const char* myDomain = "script.google.com";
  String getAll = "", getBody = "";

  camera_fb_t* fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  Serial.println("Connect to " + String(myDomain));
  WiFiClientSecure client_tcp;
  client_tcp.setInsecure();  //run version 1.0.5 or above

  if (client_tcp.connect(myDomain, 443)) {
    Serial.println("Connection successful");

    char* input = (char*)fb->buf;
    char output[base64_enc_len(3)];
    String imageFile = "data:image/jpeg;base64,";
    for (int i = 0; i < fb->len; i++) {
      base64_encode(output, (input++), 3);
      if (i % 3 == 0) imageFile += urlencode(String(output));
    }
    String Data = myLineNotifyToken + myFoldername + myFilename + myImage;

    client_tcp.println("POST " + myScript + " HTTP/1.1");
    client_tcp.println("Host: " + String(myDomain));
    client_tcp.println("Content-Length: " + String(Data.length() + imageFile.length()));
    client_tcp.println("Content-Type: application/x-www-form-urlencoded");
    client_tcp.println("Connection: keep-alive");
    client_tcp.println();

    client_tcp.print(Data);
    int Index;
    for (Index = 0; Index < imageFile.length(); Index = Index + 1000) {
      client_tcp.print(imageFile.substring(Index, Index + 1000));
    }
    esp_camera_fb_return(fb);

    int waitTime = 10000;  // timeout 10 seconds
    long startTime = millis();
    boolean state = false;

    while ((startTime + waitTime) > millis()) {
      Serial.print(".");
      delay(100);
      while (client_tcp.available()) {
        char c = client_tcp.read();
        if (state == true) getBody += String(c);
        if (c == '\n') {
          if (getAll.length() == 0) state = true;
          getAll = "";
        } else if (c != '\r')
          getAll += String(c);
        startTime = millis();
      }
      if (getBody.length() > 0) break;
    }
    client_tcp.stop();
    Serial.println(getBody);
  } else {
    getBody = "Connected to " + String(myDomain) + " failed.";
    Serial.println("Connected to " + String(myDomain) + " failed.");
  }

  return getBody;
}


String urlencode(String str) {
  String encodedString = "";
  char c;
  char code0;
  char code1;
  char code2;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == ' ') {
      encodedString += '+';
    } else if (isalnum(c)) {
      encodedString += c;
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9) {
        code0 = c - 10 + 'A';
      }
      code2 = '\0';
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
      //encodedString+=code2;
    }
    yield();
  }
  return encodedString;
}
