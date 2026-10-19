// Мини BD-1: стенд серво и первая походка.
// Сборка: при 1500 мкс на всех каналах ноги прямые (качалки надеты на прямую ногу).
// Углы суставов в градусах: 0 — прямо; плюс — как сгибается колено (голень к пятке).
// Команды в Serial Monitor (115200, перевод строки — LF):
//   stand      — плавно в стойку (полуприсед, колени вперёд)
//   straight   — плавно выпрямить ноги
//   walk 4     — 4 цикла шага (walk без числа — пока не stop); stop — закончить шаг в стойке
//   period 2   — длительность цикла шага, с
//   trim 6 -100 — поправка центра сустава 6 (0–7, не канала) в мкс (≈10 мкс на градус), до перезагрузки
//   us 3 1520  — канал PCA 3 импульсом 1520 мкс (сырой сигнал, для проверки)
//   off 3      — снять сигнал с канала 3 (серво можно крутить рукой)
//   all 90     — все 16 каналов в 1500 мкс по очереди
//   status     — сколько раз сбоила связь с PCA9685 и сколько раз прошивка её восстановила
#include <Arduino.h>
#include <Wire.h>

const uint8_t LEG = 8;       // суставы 0–3 левая нога, 4–7 правая: бедро, колено, голеностоп, наклон стопы
// каналы PCA9685, куда воткнуты суставы: левая нога в 4–7, правая в 8–11; шея — 12, голова — 13
const uint8_t CH[LEG] = {4, 5, 6, 7, 8, 9, 10, 11};
const uint8_t CH_NECK = 12, CH_HEAD = 13;
const float US_PER_DEG = 10.5f;            // MG90S: ~500–2400 мкс на ~180°
const int US_CENTER = 1500, US_LO = 600, US_HI = 2400;
// левая нога: больше импульс — сустав сгибается в «плюс»; правая стоит зеркально
const int8_t SIGN[LEG] = {+1, +1, +1, +1, -1, -1, -1, -1};
// поправки центра, если качалка села не на тот зубчик (подобраны на собранных ногах)
int16_t trimUs[LEG] = {0, 0, 0, 0, 0, 0, -158, 0};

const float STAND[4] = {-30, 60, -30, 0};   // бедро, колено, голеностоп, наклон стопы

float q[LEG];                               // текущие углы суставов, °

void pcaWriteUs(uint8_t ch, int us);

void writeJoint(uint8_t ch, float deg) {
  q[ch] = deg;
  int us = US_CENTER + trimUs[ch] + SIGN[ch] * deg * US_PER_DEG + 0.5f;
  pcaWriteUs(CH[ch], constrain(us, US_LO, US_HI));
}

void setUs(uint8_t ch, int us) {
  us = constrain(us, 500, 2500);
  pcaWriteUs(ch, us);
  Serial.printf("канал %u: %d мкс\n", ch, us);
}

// ---------- плавные переходы и шаг ----------
const uint32_t TICK_MS = 20;               // 50 Гц, как обновляются сами серво
enum Mode { IDLE, MOVING, WALKING } mode = IDLE;
float from[LEG], to[LEG];
float moveT = 0, moveDur = 1;
float period = 2.0f, phase = 0, blend = 0;
bool stopping = false;
int cyclesLeft = -1;
uint32_t lastTick = 0;

void moveTo(const float target[LEG], float secs) {
  for (int i = 0; i < LEG; i++) { from[i] = q[i]; to[i] = target[i]; }
  moveT = 0; moveDur = secs; mode = MOVING;
}

void standPose(float out[LEG]) {
  for (int i = 0; i < 4; i++) out[i] = out[4 + i] = STAND[i];
}

// фаза 0..0.5 — нога в воздухе выносится вперёд, 0.5..1 — стоит и уходит назад;
// голеностоп всегда −(бедро + колено): стопа параллельна полу
const float HIP_SWING = 20, KNEE_LIFT = 25;
void legGait(float ph, float out[4]) {
  ph -= floorf(ph);
  float lift = ph < 0.5f ? sq(sinf(TWO_PI * ph)) : 0;
  out[0] = STAND[0] + HIP_SWING * cosf(TWO_PI * ph);   // ph 0 — нога сзади, 0.5 — впереди
  out[1] = STAND[1] + KNEE_LIFT * lift;
  out[2] = -(out[0] + out[1]);
  out[3] = 0;
}

void startWalk(int cycles) {
  float s[LEG]; standPose(s);
  for (int i = 0; i < LEG; i++) if (fabsf(q[i] - s[i]) > 1) { Serial.println("сначала stand"); return; }
  cyclesLeft = cycles; stopping = false; phase = 0.75f; blend = 0; mode = WALKING;
  Serial.printf("иду, цикл %.1f с\n", period);
}

void tick() {
  uint32_t now = millis();
  if (now - lastTick < TICK_MS) return;
  lastTick = now;
  float dt = TICK_MS / 1000.0f;
  if (mode == MOVING) {
    moveT += dt;
    float k = moveT >= moveDur ? 1 : (1 - cosf(PI * moveT / moveDur)) / 2;
    for (int i = 0; i < LEG; i++) writeJoint(i, from[i] + (to[i] - from[i]) * k);
    if (moveT >= moveDur) { mode = IDLE; Serial.println("готово"); }
  } else if (mode == WALKING) {
    float prev = phase;
    phase += dt / period;
    if (cyclesLeft > 0 && floorf(phase - 0.75f) > floorf(prev - 0.75f) && --cyclesLeft == 0) stopping = true;
    blend += (stopping ? -1 : 1) * dt / 0.8f;
    blend = constrain(blend, 0.0f, 1.0f);
    float L[4], R[4], s[LEG];
    legGait(phase, L); legGait(phase + 0.5f, R); standPose(s);
    for (int i = 0; i < 4; i++) {
      writeJoint(i, s[i] + (L[i] - s[i]) * blend);
      writeJoint(4 + i, s[4 + i] + (R[i] - s[4 + i]) * blend);
    }
    if (stopping && blend <= 0) { mode = IDLE; Serial.println("стою"); }
  }
}

// ---------- PCA9685 своими силами, с проверкой каждой записи ----------
// Провода I2C идут рядом с проводами серво: импульс тока может сбить передачу, и шина «залипает»
// (SDA остаётся прижатой). Драйвер ESP32 сам из этого не выходит — дальше запись молча пропадает,
// ESP32 отвечает «готово», а серво стоят. Поэтому: каждая запись проверяется, при ошибке шину
// раскачиваем 9 тактами SCL, перезапускаем I2C и заново пишем настройки и все положения.
// Плата может и сама сброситься (после сброса спит с выключенными выходами) — это ловим чтением делителя.
const float PCA_OSC = 27000000, PWM_HZ = 50;
const uint8_t PRESCALE = (uint8_t)(PCA_OSC / (4096 * PWM_HZ) + 0.5f - 1);
const uint8_t PCA_ADDR = 0x40, REG_MODE1 = 0x00, REG_LED0 = 0x06, REG_PRESCALE = 0xFE;
const int PIN_SDA = 21, PIN_SCL = 22;
uint16_t lastTicks[16];                    // 0 — канал не задан или выключен
bool busBad = false;
uint32_t lastPcaRefresh = 0, lastBlind = 0;
uint32_t i2cWriteErrors = 0, busRecovers = 0, pcaRestores = 0, readFails = 0;

bool pcaWriteBytes(uint8_t reg, const uint8_t *d, uint8_t n) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(d, n);
  if (Wire.endTransmission() == 0) return true;
  i2cWriteErrors++;
  busBad = true;
  return false;
}
bool pcaWrite8(uint8_t reg, uint8_t v) { return pcaWriteBytes(reg, &v, 1); }
bool pcaSetPWM(uint8_t ch, uint16_t on, uint16_t off) {
  uint8_t d[4] = {(uint8_t)on, (uint8_t)(on >> 8), (uint8_t)off, (uint8_t)(off >> 8)};
  return pcaWriteBytes(REG_LED0 + 4 * ch, d, 4);
}

void pcaWriteUs(uint8_t ch, int us) {
  uint16_t ticks = us * (PCA_OSC / 1e6f) / (PRESCALE + 1) + 0.5f;
  lastTicks[ch] = ticks;
  pcaSetPWM(ch, 0, ticks);
}

bool pcaRead8(uint8_t reg, uint8_t &v) {
  Wire.beginTransmission(PCA_ADDR); Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(PCA_ADDR, (uint8_t)1) != 1) return false;
  v = Wire.read();
  return true;
}

// сон → делитель → пробуждение с автоинкрементом → рестарт выходов → все положения заново
bool pcaInit() {
  bool ok = pcaWrite8(REG_MODE1, 0x10) && pcaWrite8(REG_PRESCALE, PRESCALE) && pcaWrite8(REG_MODE1, 0x20);
  delayMicroseconds(600);      // генератору нужно 500 мкс
  ok = ok && pcaWrite8(REG_MODE1, 0xA0);
  for (int ch = 0; ch < 16 && ok; ch++) if (lastTicks[ch]) ok = pcaSetPWM(ch, 0, lastTicks[ch]);
  return ok;
}

void i2cStart() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
}

void busRecover() {
  Wire.end();
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9 && !digitalRead(PIN_SDA); i++) {   // выталкиваем застрявший байт
    digitalWrite(PIN_SCL, LOW); delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);                      // условие STOP
  digitalWrite(PIN_SDA, LOW); delayMicroseconds(5);
  digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_SDA, HIGH); delayMicroseconds(5);
  i2cStart();
  busRecovers++;
}

void pcaRefresh() {
  uint32_t now = millis();
  if (busBad) {                // запись не прошла: чиним шину сразу, но не чаще 5 раз в секунду
    if (now - lastPcaRefresh < 200) return;
    lastPcaRefresh = now;
    busBad = false;
    busRecover();
    bool ok = pcaInit();
    Serial.printf("I2C: запись не прошла, перезапустил шину (%lu)%s\n", (unsigned long)busRecovers, ok ? "" : ", пока безуспешно");
    return;
  }
  if (now - lastPcaRefresh < 500) return;
  lastPcaRefresh = now;
  uint8_t pre;
  if (pcaRead8(REG_PRESCALE, pre)) {
    if (pre != PRESCALE && pcaInit()) Serial.printf("PCA9685 сбросилась, восстановил (%lu)\n", (unsigned long)++pcaRestores);
  } else if (++readFails, now - lastBlind > 3000) {   // читать не выходит — восстанавливаем вслепую, редко
    lastBlind = now;
    pcaInit();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  i2cStart();
  Wire.beginTransmission(PCA_ADDR);
  if (Wire.endTransmission() != 0)
    Serial.println("PCA9685 не отвечает на 0x40: проверь SDA/SCL, 3V3 и GND");
  pcaInit();
  uint8_t pre;
  if (!pcaRead8(REG_PRESCALE, pre)) Serial.println("PCA9685: чтение по I2C не проходит — проверь провод GND к ESP32");
  for (int i = 0; i < LEG; i++) { writeJoint(i, 0); delay(100); }   // ноги прямо, по одной
  Serial.println("Готово. Команды: stand | straight | walk 4 | stop | period 2 | us 3 1520 | off 3 | all 90");
}

void loop() {
  tick();
  pcaRefresh();
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  int a, b;
  float f;
  if (line == "stand") {
    float s[LEG]; standPose(s); moveTo(s, 1.5f);
  } else if (line == "straight") {
    float z[LEG] = {0}; moveTo(z, 1.5f);
  } else if (line == "walk") {
    startWalk(-1);
  } else if (sscanf(line.c_str(), "walk %d", &a) == 1) {
    startWalk(a > 0 ? a : -1);
  } else if (line == "status") {
    Serial.printf("I2C: ошибок записи %lu, перезапусков шины %lu, сбросов PCA %lu, неудачных чтений %lu\n",
                  (unsigned long)i2cWriteErrors, (unsigned long)busRecovers, (unsigned long)pcaRestores, (unsigned long)readFails);
  } else if (line == "stop") {
    stopping = true;
  } else if (sscanf(line.c_str(), "period %f", &f) == 1) {
    period = constrain(f, 0.8f, 5.0f);
    Serial.printf("цикл %.1f с\n", period);
  } else if (sscanf(line.c_str(), "trim %d %d", &a, &b) == 2 && a >= 0 && a < LEG) {
    trimUs[a] = b;
    writeJoint(a, q[a]);
    Serial.print("trim:");
    for (int i = 0; i < LEG; i++) Serial.printf(" %d", trimUs[i]);
    Serial.println();
  } else if (line.startsWith("all ")) {
    mode = IDLE;
    for (uint8_t ch = 0; ch < 16; ch++) { setUs(ch, US_CENTER); delay(150); }
    for (int i = 0; i < LEG; i++) q[i] = 0;
  } else if (sscanf(line.c_str(), "us %d %d", &a, &b) == 2) {
    mode = IDLE; setUs(a, b);
  } else if (sscanf(line.c_str(), "off %d", &a) == 1) {
    if (a >= 0 && a < 16) { lastTicks[a] = 0; pcaSetPWM(a, 0, 4096); }   // полностью выключить выход
    Serial.printf("канал %d выключен\n", a);
  } else {
    Serial.println("не понял команду");
  }
}
