#include <Wire.h>

#define MUX_ADDR 0x70
#define AS5600_ADDR 0x36
#define PCA9685_ADDR 0x40

bool pca_initialized[8] = {false}; // เก็บสถานะว่าช่องไหน Init Servo ไปแล้วบ้าง

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  Serial.setTimeout(10); // ให้รับคำสั่ง Serial ไวขึ้น
  delay(1000);
}

// ----------------- I2C Functions -----------------
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool pingDevice(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

int readAS5600Raw() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);
  if (Wire.endTransmission() != 0) return -1;
  
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() <= 2) {
    uint8_t hi = Wire.read() & 0x0F;
    uint8_t lo = Wire.read();
    return (hi << 8) | lo;
  }
  return -1;
}

// --- Servo (PCA9685) Setup แบบไม่ต้องง้อ Library ---
void initPCA9685(uint8_t mux_ch) {
  selectMuxChannel(mux_ch);
  
  // 1. Sleep
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x00); // MODE1
  Wire.write(0x10); 
  Wire.endTransmission();

  // 2. Set Prescale for 50Hz (ประมาณ 121)
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0xFE); // PRESCALE
  Wire.write(121);
  Wire.endTransmission();

  // 3. Wake up & Auto Increment
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x00); // MODE1
  Wire.write(0xA1);
  Wire.endTransmission();
  delay(5);
}

void setServoAngle(uint8_t mux_ch, uint8_t servo_ch, int angle) {
  selectMuxChannel(mux_ch);
  
  // แปลง 0-180 องศา เป็น Pulse (150 - 600)
  if(angle < 0) angle = 0;
  if(angle > 180) angle = 180;
  uint16_t pulse = 150 + (angle / 180.0) * (600 - 150);

  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x06 + (4 * servo_ch)); // LED0_ON_L
  Wire.write(0);           // ON L
  Wire.write(0);           // ON H
  Wire.write(pulse & 0xFF); // OFF L
  Wire.write(pulse >> 8);   // OFF H
  Wire.endTransmission();
}

// ----------------- Main Loop -----------------
void loop() {
  // 1. เช็คคำสั่งจาก Pi (เช่น S3:0:90 แปลว่า Mux ช่อง 3, Servo ช่อง 0, ไปที่ 90 องศา)
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("S")) {
      int c1 = cmd.indexOf(':');
      int c2 = cmd.lastIndexOf(':');
      if (c1 > 0 && c2 > c1) {
        int target_mux = cmd.substring(1, c1).toInt();
        int servo_ch = cmd.substring(c1 + 1, c2).toInt();
        int angle = cmd.substring(c2 + 1).toInt();
        
        // ถ้าไม่เคย Init บอร์ด Servo ช่องนี้ ให้ Init ก่อน
        if (!pca_initialized[target_mux]) {
          initPCA9685(target_mux);
          pca_initialized[target_mux] = true;
        }
        setServoAngle(target_mux, servo_ch, angle);
      }
    }
  }

  // 2. สแกน 8 ช่องแล้วส่งสถานะให้ Pi ทุกรอบ
  String mapData = "\"devices\": {";
  String as5600Data = "\"as5600\": [";
  
  for (int i = 0; i < 8; i++) {
    selectMuxChannel(i);
    bool hasAS5600 = pingDevice(AS5600_ADDR);
    bool hasServo = pingDevice(PCA9685_ADDR);
    
    mapData += "\"" + String(i) + "\":";
    if (hasAS5600) mapData += "\"as5600\"";
    else if (hasServo) mapData += "\"servo\"";
    else mapData += "\"none\"";
    
    if (hasAS5600) as5600Data += String(readAS5600Raw());
    else as5600Data += "0";
    
    if (i < 7) { mapData += ", "; as5600Data += ", "; }
  }
  mapData += "}"; as5600Data += "]";
  
  Serial.println("{" + mapData + ", " + as5600Data + "}");
  delay(15); // รันเร็วๆ ได้เลย ประมาณ 60Hz
}