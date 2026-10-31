#include <Wire.h>
#include <MPU6050.h>
#include <ArduinoJson.h>

MPU6050 mpu;
unsigned long lastSampleTime = 0;
const int sampleInterval = 500; 

void setup() {
  Serial.begin(115200);
  while (!Serial); // MEGA는 없어도 되지만 안정성을 위해 추가

  Serial.println("MPU6050 초기화 중...");

  Wire.begin();  // I2C 시작

  mpu.initialize();  // 센서 초기화

  // 센서 연결 테스트
  if (mpu.testConnection()) {
    Serial.println("✅ MPU6050 연결 성공!");
  } else {
    Serial.println("❌ MPU6050 연결 실패!");
    Serial.println("🔎 점검 목록:");
    Serial.println("- VCC는 5V에 연결되어 있나요?");
    Serial.println("- SDA → 핀 20 / SCL → 핀 21 연결 확인");
    Serial.println("- GND 연결 확인");
    Serial.println("- 라이브러리 설치 확인");
    while (1); // 연결 실패 시 멈춤
  }
  
  // 초기화 완료 메시지 (JSON 형식)
  StaticJsonDocument<200> initDoc;
  initDoc["status"] = "initialized";
  initDoc["message"] = "MPU6050 초기화 완료";
  serializeJson(initDoc, Serial);
  Serial.println();
}

void loop() {
  unsigned long currentTime = millis();
  
  // 설정된 간격마다 데이터 전송
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;
    
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 가속도 값 변환 (단위: g, ±2g 범위)
    float accel_x = ax / 16384.0;
    float accel_y = ay / 16384.0;
    float accel_z = az / 16384.0;
    
    // 자이로 값 변환 (단위: 도/초, ±250도/초 범위)
    float gyro_x = gx / 131.0;
    float gyro_y = gy / 131.0;
    float gyro_z = gz / 131.0;
    
    // 진동 강도 계산 (가속도 벡터의 크기) - 주석 처리
    // float vibration = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    
    // 데이터를 JSON 형식으로 전송
    StaticJsonDocument<256> doc;
    // doc["timestamp"] = currentTime;  // 타임스탬프 주석 처리
    // doc["interval"] = sampleInterval;  // interval 필드 주석 처리
    
    // 가속도 데이터
    JsonObject accel = doc.createNestedObject("accel");
    accel["x"] = accel_x;
    accel["y"] = accel_y;
    accel["z"] = accel_z;
    
    // 자이로 데이터
    JsonObject gyro = doc.createNestedObject("gyro");
    gyro["x"] = gyro_x;
    gyro["y"] = gyro_y;
    gyro["z"] = gyro_z;
    
    // 계산된 값 - 주석 처리
    // doc["vibration"] = vibration;
    
    // JSON 직렬화 및 전송
    serializeJson(doc, Serial);
    Serial.println();
  }
}
