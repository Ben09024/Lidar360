#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
#include <Adafruit_NeoPixel.h>
#include <AccelStepper.h>

#define PIN_TMC_RX 3
#define PIN_TMC_TX 13
#define PIN_LIDAR_RX 4
#define PIN_LIDAR_TX 5
#define PIN_EN 8
#define PIN_STEP 11
#define PIN_DIR 12
#define PIN_LED 21 

#define MOTOR_CURRENT 800
#define MICROSTEPS 16

TMC2209Stepper driver(&Serial1, 0.11f, 0b00);
Adafruit_NeoPixel statusLed(1, PIN_LED, NEO_GRB + NEO_KHZ800);
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

bool motor_active = false;
float current_speed = 400.0;

struct __attribute__((packed)) Packet {
  uint8_t head[2] = {0xAA, 0xBB};
  float angle;                    
  uint16_t dist;                  
  uint16_t str;                   
  uint8_t crc;                    
  uint8_t padding;                
};

uint8_t calcCRC8(uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    uint8_t extract = *data++;
    for (uint8_t tempI = 8; tempI; tempI--) {
      uint8_t sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) crc ^= 0x8C;
      extract >>= 1;
    }
  }
  return crc;
}

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  statusLed.setPixelColor(0, statusLed.Color(r,g,b));
  statusLed.show();
}

void setup() {
  statusLed.begin(); 
  statusLed.setBrightness(20); 
  setLed(50,0,50);
  
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);
  Serial2.begin(115200, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);

  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);

  driver.begin();
  driver.toff(4);
  driver.rms_current(MOTOR_CURRENT);
  driver.microsteps(MICROSTEPS);
  driver.pwm_autoscale(true);

  stepper.setMaxSpeed(5000);
  stepper.setSpeed(current_speed);

  uint8_t fps[] = {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06};
  Serial2.write(fps, 8);

  setLed(0,0,255);
}

void loop() {
  if (motor_active) {
    stepper.runSpeed();
  }

  if (Serial.available()) {
    String c = Serial.readStringUntil('\n'); 
    c.trim(); 
    c.toUpperCase();
    if (c == "M100") { 
      motor_active = true; 
      setLed(0,255,0); 
    }
    else if (c == "M101") { 
      motor_active = false; 
      setLed(0,0,255); 
    }
    else if (c.startsWith("M203")) {
      current_speed = c.substring(5).toFloat();
      stepper.setSpeed(current_speed);
    }
  }

  static uint8_t buf[9];
  static int idx = 0;
  static Packet pkt;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    
    if (idx == 0 && b != 0x59) continue;
    if (idx == 1 && b != 0x59) { idx=0; continue; }
    buf[idx++] = b;

    if (idx == 9) {
      uint16_t cs = 0; 
      for(int i=0; i<8; i++) cs += buf[i];
      if ((cs & 0xFF) == buf[8]) {
        long current_step = stepper.currentPosition();
        float steps_per_rev = 3200.0;
        pkt.angle = fmod((current_step * 360.0 / steps_per_rev), 360.0);
        
        pkt.dist = buf[2] | (buf[3]<<8);
        pkt.str = buf[4] | (buf[5]<<8);
        
        pkt.crc = calcCRC8((uint8_t*)&pkt.angle, 8);

        if (motor_active) {
          Serial.write((uint8_t*)&pkt, sizeof(Packet));
        }
      }
      idx = 0;
    }
  }
}
