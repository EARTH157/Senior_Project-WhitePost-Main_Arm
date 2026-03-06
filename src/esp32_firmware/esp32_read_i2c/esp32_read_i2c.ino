#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> // ไลบรารีสำหรับ PCA9685

#define MUX_ADDR 0x70
#define AS5600_ADDR 0x36

// สร้างออบเจกต์สำหรับ PCA9685 (Address เริ่มต้นคือ 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ค่าความกว้างของสัญญาณ PWM สำหรับ Servo ทั่วไป (อาจต้องปรับจูนตามรุ่นของ Servo)
#define SERVOMIN  150 // ค่า PWM สำหรับ 0 องศา
#define SERVOMAX  600 // ค่า PWM สำหรับ 180 องศา

void selectI2CChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

int readAS5600Raw() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); 
  if (Wire.endTransmission(false) != 0) return -1; 
  
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() == 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    return (highByte << 8) | lowByte; 
  }
  return -1;
}

// ฟังก์ชันสำหรับสั่งงาน Servo ผ่าน PCA9685
void setServoAngle(uint8_t channel, uint8_t angle) {
  // แปลงค่าองศา (0-180) เป็นความยาวคลื่น PWM (SERVOMIN-SERVOMAX)
  uint16_t pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulseLength);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  
  // เริ่มต้นการทำงานของบอร์ด PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); // ตั้งค่าความถี่ที่ 50Hz (มาตรฐานของ Analog Servo)
  delay(10);
}

void loop() {
  // --- ส่วนที่ 1: รับคำสั่งจาก Pi ---
  // รูปแบบคำสั่ง: S<channel>:<angle> เช่น S0:90 หรือ S15:180
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); 
    input.trim(); 
    
    if (input.startsWith("S")) {
      int colonIndex = input.indexOf(':'); // หาตำแหน่งของเครื่องหมาย :
      if (colonIndex != -1) {
        // แยกตัวเลขช่องและตัวเลของศาออกจากกัน
        int servoNum = input.substring(1, colonIndex).toInt();
        int angle = input.substring(colonIndex + 1).toInt();
        
        // ตรวจสอบความถูกต้องของค่า (ช่อง 0-15, องศา 0-180)
        if (servoNum >= 0 && servoNum <= 15 && angle >= 0 && angle <= 180) {
          setServoAngle(servoNum, angle);
        }
      }
    }
  }

  // --- ส่วนที่ 2: อ่าน AS5600 ทั้ง 8 ช่องและส่งเป็น JSON ---
  String jsonOutput = "{\"as5600\": [";
  
  for (uint8_t chan = 0; chan < 8; chan++) {
    selectI2CChannel(chan);
    int rawValue = readAS5600Raw();
    jsonOutput += String(rawValue);
    if (chan < 7) jsonOutput += ", ";
  }
  jsonOutput += "]}";
  
  Serial.println(jsonOutput);
  delay(50); 
}