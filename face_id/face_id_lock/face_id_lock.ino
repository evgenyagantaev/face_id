// Eugeny Agantaev
// Saint-Petersburg
// 11 april 2021

#include "esp_camera.h"
#include <WiFi.h>

#define CAMERA_MODEL_AI_THINKER

#define RED_LED 12
#define GREEN_LED 13
#define YELLOW_LED 14
#define ENROLL_FACE_BUTTON_PIN 15
#define LOCK_OPEN_PIN 16

#include "camera_pins.h"

//const char* ssid = "SPMT_9_2.4";
//const char* password = "123098qq!";
const char* ssid = "faceidlock";
const char* password = "123456789";

boolean face_matched = false;
boolean lock_open = false;

const int open_period_millis = 7000;
unsigned int frozen_milliseconds = 0;


void startCameraServer();

void setup() 
{
    pinMode(LOCK_OPEN_PIN, OUTPUT);
    digitalWrite(LOCK_OPEN_PIN, LOW); // close the lock
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(ENROLL_FACE_BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
  
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();
  
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
    if(psramFound()){
      config.frame_size = FRAMESIZE_UXGA;
      config.jpeg_quality = 10;
      config.fb_count = 2;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.jpeg_quality = 12;
      config.fb_count = 1;
    }
  
  #if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
  #endif
  
    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }
  
    sensor_t * s = esp_camera_sensor_get();
    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);//flip it back
      s->set_brightness(s, 1);//up the blightness just a bit
      s->set_saturation(s, -2);//lower the saturation
    }
    //drop down frame size for higher initial frame rate
    s->set_framesize(s, FRAMESIZE_QVGA);
  
  #if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  #endif

    /*
    WiFi.begin(ssid, password);
  
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    //*/

    WiFi.softAP(ssid, NULL);
  
    startCameraServer();
  
    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");
}

void loop() 
{
    if(face_matched == true && lock_open == false)
    {
        lock_open = true;
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(YELLOW_LED, LOW);

        frozen_milliseconds = millis();
        digitalWrite(LOCK_OPEN_PIN, HIGH);
    }

    if((lock_open == true) && ((millis() - frozen_milliseconds) > open_period_millis))
    {
        digitalWrite(LOCK_OPEN_PIN, LOW);
      
        lock_open = false;
        face_matched = false;

        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(YELLOW_LED, LOW);

    }


    
}
