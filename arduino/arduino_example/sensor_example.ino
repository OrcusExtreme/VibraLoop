/*
 * CNC 밀링 머신 모니터링 시스템을 위한 아두이노 센서 예제
 * 진동(가속도) 센서와 온도 센서 데이터를 읽어 시리얼 통신으로 전송합니다.
 */

// 아날로그 핀 정의
const int ACCEL_X_PIN = A0;  // X축 가속도 센서 (ADXL335 등)
const int ACCEL_Y_PIN = A1;  // Y축 가속도 센서
const int ACCEL_Z_PIN = A2;  // Z축 가속도 센서
const int TEMP_PIN = A3;     // 온도 센서 (LM35 등)

// 센서 보정값 (센서마다 조정 필요)
const float ACCEL_REF_VOLTAGE = 3.3;  // 가속도계 기준 전압
const float ACCEL_ZERO_G = 1.65;      // 0g에서의 전압값 (일반적으로 Vcc/2)
const float ACCEL_SENSITIVITY = 0.330; // 330mV/g (ADXL335 기준)

// 샘플링 관련 변수
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 10; // 10ms마다 샘플링 (100Hz, 진동 분석 모델의 학습 데이터 주기와 동일)

void setup() {
  // 시리얼 통신 초기화 (115200 bps, 서버 cnc_app.py의 ARDUINO_BAUD_RATE와 동일)
  Serial.begin(115200);
  
  // 아날로그 레퍼런스 설정 (Arduino Uno/Mega의 경우 기본값은 5V)
  analogReference(DEFAULT);
  
  // 초기화 메시지
  Serial.println("Arduino Sensor Monitor for CNC Milling Machine");
  Serial.println("----------------------------------------------");
  delay(1000);
}

void loop() {
  // 샘플링 간격 제어
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;
    
    // 센서 값 읽기
    int rawAccelX = analogRead(ACCEL_X_PIN);
    int rawAccelY = analogRead(ACCEL_Y_PIN);
    int rawAccelZ = analogRead(ACCEL_Z_PIN);
    int rawTemp = analogRead(TEMP_PIN);
    
    // 가속도 값 변환 (g 단위)
    float accelX = convertRawToG(rawAccelX);
    float accelY = convertRawToG(rawAccelY);
    float accelZ = convertRawToG(rawAccelZ);
    
    // 온도 값 변환 (섭씨 단위, LM35 기준)
    float tempC = (rawTemp * 5.0 * 100.0) / 1024.0;
    
    // 진동 강도 계산 (루트 평균 제곱)
    float vibrationIntensity = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    
    // JSON 형식으로 데이터 전송
    Serial.print("{");
    Serial.print("\"accelX\":");
    Serial.print(accelX, 3);
    Serial.print(",\"accelY\":");
    Serial.print(accelY, 3);
    Serial.print(",\"accelZ\":");
    Serial.print(accelZ, 3);
    Serial.print(",\"vibration\":");
    Serial.print(vibrationIntensity, 3);
    Serial.print(",\"temp\":");
    Serial.print(tempC, 1);
    Serial.println("}");
  }
}

// 아날로그 값을 g 단위로 변환하는 함수
float convertRawToG(int rawValue) {
  // 전압으로 변환
  float voltage = (rawValue / 1024.0) * 5.0;
  
  // g 단위로 변환
  float acceleration = (voltage - ACCEL_ZERO_G) / ACCEL_SENSITIVITY;
  
  return acceleration;
} 