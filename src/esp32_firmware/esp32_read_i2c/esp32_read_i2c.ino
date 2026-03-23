#include <Wire.h>

#define MUX_ADDR 0x70
#define AS5600_ADDR 0x36
#define PCA9685_ADDR 0x40

bool pca_initialized[8] = {false};

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  
  // --- เพิ่มความทนทานต่อ Noise ---
  Wire.setClock(100000); // ใช้ Standard Mode (100kHz) ทนสายยาวและ Noise ได้ดีกว่า
  Wire.setTimeOut(20);   // ตั้ง Timeout 20ms ป้องกันบัสค้าง (I2C Hang)
  // --------------------------------
  
  Serial.setTimeout(10);
  delay(1000);
}

// ----------------- I2C Functions (พร้อมระบบ Retry) -----------------

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;
  
  // ลองสั่งสลับช่อง Mux สูงสุด 3 รอบ ถ้ามี Noise กวน
  for (int retry = 0; retry < 3; retry++) {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << channel);
    if (Wire.endTransmission() == 0) return true; // สำเร็จ
    delay(2); // พักแป๊บเดียวก่อนลองใหม่
  }
  return false;
}

bool pingDevice(uint8_t address) {
  for (int retry = 0; retry < 3; retry++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) return true;
    delay(2);
  }
  return false;
}

int readAS5600Raw() {
  for (int retry = 0; retry < 3; retry++) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E); // Register มุม Raw Angle
    if (Wire.endTransmission() != 0) {
      delay(2);
      continue; // ถ้าเขียนไม่เข้า ให้ลองรอบใหม่
    }
    
    // ขออ่าน 2 ไบต์
    uint8_t bytesReceived = Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    
    // แก้ไข: ต้องเช็กว่าได้มา "ครบ 2 ไบต์" จริงๆ ค่อยเอามาคำนวณ
    if (bytesReceived == 2) {
      uint8_t hi = Wire.read();
      uint8_t lo = Wire.read();
      return ((hi & 0x0F) << 8) | lo;
    }
    delay(2);
  }
  return -1; // อ่านล้มเหลวทั้ง 3 รอบ
}

// --- Servo (PCA9685) Setup แบบไม่ต้องง้อ Library ---
void initPCA9685(uint8_t mux_ch) {
  if (!selectMuxChannel(mux_ch)) return; // ถ้าเลือกช่อง Mux ไม่ผ่าน ไม่ต้องทำต่อ
  
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x00); // MODE1
  Wire.write(0x10); 
  Wire.endTransmission();

  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0xFE); // PRESCALE
  Wire.write(121);
  Wire.endTransmission();

  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x00); // MODE1
  Wire.write(0xA1);
  Wire.endTransmission();
  delay(5);
}

void setServoAngle(uint8_t mux_ch, uint8_t servo_ch, int angle) {
  if (!selectMuxChannel(mux_ch)) return;
  
  if(angle < 0) angle = 0;
  if(angle > 180) angle = 180;
  uint16_t pulse = 150 + (angle / 180.0) * (600 - 150);

  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(0x06 + (4 * servo_ch)); // LED0_ON_L
  Wire.write(0);           // ON L
  // แก้ไข: รวมคำสั่งเขียนรวดเดียว เพื่อลดโอกาสที่ Noise จะแทรกกลางคัน
  Wire.write(0);           // ON H
  Wire.write(pulse & 0xFF); // OFF L
  Wire.write(pulse >> 8);   // OFF H
  Wire.endTransmission();
}

// ----------------- Main Loop -----------------
void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("S")) {
      int c1 = cmd.indexOf(':');
      int c2 = cmd.lastIndexOf(':');
      if (c1 > 0 && c2 > c1) {
        int target_mux = cmd.substring(1, c1).toInt();
        int servo_ch = cmd.substring(c1 + 1, c2).toInt();
        int angle = cmd.substring(c2 + 1).toInt();
        
        if (!pca_initialized[target_mux]) {
          initPCA9685(target_mux);
          pca_initialized[target_mux] = true;
        }
        setServoAngle(target_mux, servo_ch, angle);
      }
    }
  }

  // ใช้ Buffer เพื่อลดภาระการต่อ String (ESP32 ไม่ค่อยชอบ String ยาวๆ)
  String mapData = "\"devices\": {";
  String as5600Data = "\"as5600\": [";
  
  for (int i = 0; i < 8; i++) {
    selectMuxChannel(i); // เลือกช่องแล้วรอแป๊บนึง (ฟังก์ชันจัดการ retry ให้แล้ว)
    
    bool hasAS5600 = pingDevice(AS5600_ADDR);
    bool hasServo = pingDevice(PCA9685_ADDR);
    
    mapData += "\"" + String(i) + "\":";
    if (hasAS5600) mapData += "\"as5600\"";
    else if (hasServo) mapData += "\"servo\"";
    else mapData += "\"none\"";
    
    if (hasAS5600) {
      int pos = readAS5600Raw();
      as5600Data += String(pos);
    } else {
      as5600Data += "0";
    }
    
    if (i < 7) { 
      mapData += ", "; 
      as5600Data += ", "; 
    }
  }
  mapData += "}"; 
  as5600Data += "]";
  
  Serial.println("{" + mapData + ", " + as5600Data + "}");
  delay(15); 
}