// 고속 진동 측정 펌웨어 (MPU6050, 1 kHz, ±8 g, 저역통과 약 184 Hz)
//
// 기존 펌웨어(arduino/MPU6050)는 10 ms마다 JSON 한 줄을 보내 100 Hz가 한계이고, 저역통과 필터가 꺼져 있어
// 높은 주파수 진동이 낮은 주파수로 접혀(앨리어싱) 기록되며, ±2 g에서 값이 잘립니다 (edit_analysis.md 2.6, 3.1).
// 이 펌웨어는 센서 내부 FIFO로 일정한 간격의 샘플을 모아 일련번호와 함께 짧은 텍스트로 보냅니다.
//
// 서버 연결: 화면의 센서 통신 속도에서 500000을 선택 (src/sensor_protocol.py가 해석)
// 출력 형식 (한 줄에 하나):
//   #H,device=MPU6050,fs=<측정한 샘플링 Hz>,range_g=8,dlpf_hz=184   헤더: 시작할 때와 5초마다
//   A,<seq>,<ax>,<ay>,<az>                                          샘플: 센서 원시값 (±8 g에서 4096 = 1 g)
//   #O,<seq>                                                        FIFO가 넘쳐 샘플이 빠짐: 다음 샘플의 seq
//   #E,<메시지>                                                     오류
// 샘플 시각은 seq를 측정한 샘플링으로 나눈 값이라, USB 전송 지연과 관계없이 실제 측정 간격이 유지됩니다.
//
// 주의: MPU6050 가속도 대역폭은 최대 약 260 Hz라 날 통과·채터 주파수(수백 Hz 이상)는 볼 수 없습니다.
//       주파수 분석이 필요하면 광대역 진동 센서(예: ACCEL 14 CLICK)를 같은 출력 형식으로 연결하세요.
// 필요 라이브러리: MPU6050 (Electronic Cats 또는 I2Cdevlib) — 기존 펌웨어와 같음
// (이 펌웨어는 실제 보드에서 검증하지 않았습니다. 처음 사용할 때 시리얼 모니터 500000 bps로 출력을 확인하세요.)

#include <Wire.h>
#include <MPU6050.h>
#include <stdlib.h>
#include <string.h>

MPU6050 mpu;

const long BAUD_RATE = 500000;
const uint8_t RATE_DIVIDER = 0;                 // 샘플링 = 1 kHz / (1 + RATE_DIVIDER)  (저역통과 필터를 켜면 기준 1 kHz)
const float NOMINAL_FS = 1000.0 / (1 + RATE_DIVIDER);
const int RANGE_G = 8;
const unsigned long HEADER_INTERVAL_MS = 5000;
const uint8_t BYTES_PER_SAMPLE = 6;             // FIFO에는 가속도 3축만 (축마다 2바이트)
const uint8_t MAX_BATCH = 40;                   // 한 번에 읽을 샘플 수 (getFIFOBytes 길이 한도 255바이트)

uint8_t fifoBuffer[BYTES_PER_SAMPLE * MAX_BATCH];
uint32_t seq = 0;                // 다음에 보낼 샘플 번호
uint32_t anchorProduced = 0;     // 샘플링 측정 기준점 (센서가 만든 샘플 수, 읽지 않고 FIFO에 남은 것 포함)
unsigned long anchorMicros = 0;
unsigned long lastHeaderMs = 0;
float measuredFs = NOMINAL_FS;

void printHeader() {
  Serial.print(F("#H,device=MPU6050,fs="));
  Serial.print(measuredFs, 1);
  Serial.print(F(",range_g="));
  Serial.print(RANGE_G);
  Serial.println(F(",dlpf_hz=184"));
}

void restartFifo() {
  mpu.setFIFOEnabled(false);
  mpu.resetFIFO();
  mpu.setFIFOEnabled(true);
}

void writeSample(int16_t ax, int16_t ay, int16_t az) {
  // 문자열 조립 후 한 번에 전송 (Serial.print를 여러 번 부르는 것보다 빠름)
  char line[40];
  char *p = line;
  *p++ = 'A';
  *p++ = ',';
  ultoa(seq, p, 10);
  p += strlen(p);
  *p++ = ',';
  itoa(ax, p, 10);
  p += strlen(p);
  *p++ = ',';
  itoa(ay, p, 10);
  p += strlen(p);
  *p++ = ',';
  itoa(az, p, 10);
  p += strlen(p);
  *p++ = '\n';
  Serial.write((const uint8_t *)line, p - line);
  seq++;
}

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();
  Wire.setClock(400000);  // I2C 400 kHz: FIFO를 제때 비우기 위해

  mpu.initialize();
  if (!mpu.testConnection()) {
    while (true) {
      Serial.println(F("#E,MPU6050 connection failed: check VCC/GND/SDA/SCL wiring"));
      delay(1000);
    }
  }
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  // ±8 g: 채터 중 큰 진동도 잘리지 않도록
  mpu.setDLPFMode(MPU6050_DLPF_BW_188);            // 가속도 저역통과 약 184 Hz: 1 kHz 샘플링의 앨리어싱 방지
  mpu.setRate(RATE_DIVIDER);
  mpu.setAccelFIFOEnabled(true);
  restartFifo();

  anchorMicros = micros();
  anchorProduced = 0;
  lastHeaderMs = millis();
  printHeader();
}

void loop() {
  if (mpu.getIntFIFOBufferOverflowStatus()) {
    // FIFO(1024바이트, 약 170 ms 분량)가 넘치면 빠진 샘플 수를 경과 시간으로 추정해 seq를 건너뜀
    restartFifo();
    uint32_t expected = anchorProduced + (uint32_t)((micros() - anchorMicros) * (measuredFs / 1000000.0));
    if (expected > seq) {
      seq = expected;
    }
    Serial.print(F("#O,"));
    Serial.println(seq);
  }

  uint16_t count = mpu.getFIFOCount();
  if (count >= BYTES_PER_SAMPLE) {
    uint16_t samples = count / BYTES_PER_SAMPLE;
    if (samples > MAX_BATCH) {
      samples = MAX_BATCH;
    }
    mpu.getFIFOBytes(fifoBuffer, samples * BYTES_PER_SAMPLE);
    for (uint16_t i = 0; i < samples; i++) {
      const uint8_t *b = fifoBuffer + i * BYTES_PER_SAMPLE;
      writeSample((int16_t)((b[0] << 8) | b[1]), (int16_t)((b[2] << 8) | b[3]), (int16_t)((b[4] << 8) | b[5]));
    }
  }

  unsigned long nowMs = millis();
  if (nowMs - lastHeaderMs >= HEADER_INTERVAL_MS) {
    // 센서 클럭 오차(±1~2%)를 반영하도록 실제 샘플링을 측정해 헤더로 알림
    unsigned long nowMicros = micros();
    uint32_t produced = seq + mpu.getFIFOCount() / BYTES_PER_SAMPLE;
    unsigned long elapsed = nowMicros - anchorMicros;
    if (elapsed > 0 && produced > anchorProduced) {
      measuredFs = (produced - anchorProduced) * 1000000.0 / elapsed;
    }
    anchorProduced = produced;
    anchorMicros = nowMicros;
    lastHeaderMs = nowMs;
    printHeader();
  }
}
