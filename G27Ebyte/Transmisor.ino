#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// =========================================================
// --------------------- CONFIG GLOBAL ---------------------
// =========================================================

// -------- NRF24 --------
#define CE_PIN   9
#define CSN_PIN  10
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "CTRL1";

// -------- Pines --------
#define ENC_A 2
#define ENC_B 3
#define RESET_BTN 4

#define PIN_LIGHT_BTN 7
#define PIN_FAIL_LED 8

#define PIN_LED_RED    5
#define PIN_LED_GREEN  6

#define PIN_ACCEL A0
#define PIN_BRAKE A1
#define PIN_TRIM A2
#define PIN_CAMX A3
#define PIN_CAMY A4

// -------- Variables del encoder --------
volatile long encoderPos = 0;
long encoderCenter = 0;
bool forceSteerCenter = false;
int lastEncoded = 0;

// -------- Parámetros --------
int steeringRange = 900;
int leftLimit  = -2871;
int rightLimit = 2871;

int accelMin = 76, accelMax = 955;
int brakeMin = 54, brakeMax = 913;

bool lightsOn = false;

// -------- Fail Safe Tx --------
unsigned long lastOkTime = 0;
bool failSafeActive = false;
unsigned long blinkTimer = 0;
bool blinkState = false;

// -------- Estructura a enviar --------
struct ControlData {
  int16_t throttle;
  int16_t steer;
  int16_t camX;
  int16_t camY;
  uint8_t lights;
  uint8_t mode;
};
ControlData ctrl;

// =========================================================
// ---------------------- FUNCIONES ------------------------
// =========================================================

// ---------- Encoder ISR ----------
void updateEncoder() {
  int MSB = digitalRead(ENC_A);
  int LSB = digitalRead(ENC_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderPos++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderPos--;

  lastEncoded = encoded;
}

// ---------------------- LED rango ----------------------
void updateRangeLED() {
  if (steeringRange == 900) {
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, HIGH);
  }
  else if (steeringRange == 720) {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_GREEN, HIGH);
  }
  else if (steeringRange == 540) {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_GREEN, LOW);
  }
}

// ---------- Ajustar límites ----------
void applySteeringRange() {
  switch (steeringRange) {
    case 900: leftLimit = -4904; rightLimit = 4904; break;
    case 720: leftLimit = -3841; rightLimit = 3841; break;
    case 540: leftLimit = -2871; rightLimit = 2871; break;
  }
  updateRangeLED();
}

// ---------- Anti-rebote ----------
bool readButtonDebounced(uint8_t pin, uint16_t debounceTime = 30) {
  static unsigned long lastTime[20] = {0};
  static bool lastState[20] = {HIGH};

  bool reading = digitalRead(pin);
  if (reading != lastState[pin]) {
    lastTime[pin] = millis();
  }

  if ((millis() - lastTime[pin]) > debounceTime) {
    lastState[pin] = reading;
    return reading;
  }

  return lastState[pin];
}

// ---------- Throttle ----------
void computeThrottle() {
  int accelRaw = analogRead(PIN_ACCEL);
  int brakeRaw = analogRead(PIN_BRAKE);

  int accelMapped = constrain(map(accelRaw, accelMin, accelMax, 0, 1000), 0, 1000);
  int brakeMapped = constrain(map(brakeRaw, brakeMin, brakeMax, 0, 1000), 0, 1000);

  ctrl.throttle = accelMapped - brakeMapped;

  if (abs(ctrl.throttle) < 20) ctrl.throttle = 0;
  ctrl.throttle = constrain(ctrl.throttle, -1000, 1000);
}

// ---------- Dirección ----------
void computeSteering() {
  long pos = encoderPos - encoderCenter;

  int steerRaw = constrain(
    map(pos, leftLimit, rightLimit, -1000, 1000),
    -1000, 1000
  );

  int steerTrim = map(analogRead(PIN_TRIM), 0, 1023, -600, 600);

  ctrl.steer = constrain(steerRaw + steerTrim, -1000, 1000);
}

// ---------- Cámara ----------
void computeCamera() {
  ctrl.camX = map(analogRead(PIN_CAMX), 0, 1023, -1000, 1000);
  ctrl.camY = map(analogRead(PIN_CAMY), 0, 1023, -1000, 1000);
}

// ---------- Manejo del botón luz / rango ----------
void handleLightAndRange() {
  static unsigned long pressStart = 0;
  static bool longPressDone = false;

  bool btn = digitalRead(PIN_LIGHT_BTN);  // lectura simple

  if (btn == LOW) {
    if (pressStart == 0) {
      pressStart = millis();
      longPressDone = false;
    } else if (!longPressDone && millis() - pressStart >= 2500) {
      // Cambio de rango
      if (steeringRange == 900) steeringRange = 720;
      else if (steeringRange == 720) steeringRange = 540;
      else steeringRange = 900;

      applySteeringRange(); // Actualiza límites y LED
      longPressDone = true;

      Serial.print("Nuevo rango: "); Serial.println(steeringRange);
    }
  } else {
    if (pressStart != 0 && !longPressDone) {
      lightsOn = !lightsOn;
      Serial.print("Luces: "); Serial.println(lightsOn ? "ON" : "OFF");
    }
    pressStart = 0;
    longPressDone = false;
  }

  ctrl.lights = lightsOn ? 1 : 0;
}

// ---------- Reset de dirección ----------
void handleSteeringReset() {
  static bool lastState = HIGH;
  bool state = digitalRead(RESET_BTN);

  if (state == LOW && lastState == HIGH) {
    noInterrupts();        // <-- CLAVE
    encoderPos = 0;        // reset REAL, como el sketch antiguo
    encoderCenter = 0;     // coherencia interna
    interrupts();

    Serial.println("Encoder centrado (RESET REAL)");
  }

  lastState = state;
}

// =========================================================
// ------------------------- SETUP --------------------------
// =========================================================
void setup() {
  Serial.begin(115200);

  // Encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, CHANGE);

  pinMode(RESET_BTN, INPUT_PULLUP);
  pinMode(PIN_LIGHT_BTN, INPUT_PULLUP);
  pinMode(PIN_FAIL_LED, OUTPUT);

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);

  applySteeringRange();

  // Radio
  radio.begin();
  radio.setChannel(90);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(2, 5);
  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("Transmisor listo (versión optimizada)");
}

// =========================================================
// -------------------------- LOOP -------------------------
// =========================================================
void loop() {

  computeThrottle();
  handleSteeringReset();   // primero
  computeSteering();       // después
  computeCamera();
  handleLightAndRange();

  bool ok = radio.write(&ctrl, sizeof(ctrl));

  if (ok) {
    lastOkTime = millis();
    failSafeActive = false;
    digitalWrite(PIN_FAIL_LED, LOW);
  }
  else if (millis() - lastOkTime > 500) {
    failSafeActive = true;
  }

  if (failSafeActive) {
    if (millis() - blinkTimer >= 300) {
      blinkTimer = millis();
      blinkState = !blinkState;
      digitalWrite(PIN_FAIL_LED, blinkState ? HIGH : LOW);
    }
  }

  Serial.print("Throttle: "); Serial.print(ctrl.throttle);
  Serial.print(" | Steer: "); Serial.print(ctrl.steer);
  Serial.print(" | CamX: "); Serial.print(ctrl.camX);
  Serial.print(" | CamY: "); Serial.print(ctrl.camY);
  Serial.print(" | Lights: "); Serial.println(ctrl.lights);

  delay(20);
}