#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "CTRL1";

struct ControlData {
  int16_t throttle;
  int16_t steer;
  int16_t camX;
  int16_t camY;
  uint8_t lights;
  uint8_t mode;
};
ControlData ctrl;

Servo motorESC;
Servo servoSteer;
Servo servoCamX;
Servo servoCamY;

#define PIN_MOTOR 5
#define PIN_STEER 6
#define PIN_CAMX 7
#define PIN_CAMY 8
#define PIN_LIGHTS 4

unsigned long lastSignalTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 300;

// ====== AJUSTES DE DIRECCIÓN ======

float steerLimitFactor = 0.90;

// ====== AUTOCALIBRACIÓN ESC ======
bool throttleCalibrated = false;
unsigned long calibStart = 0;
long throttleSum = 0;
int throttleSamples = 0;
const unsigned long CALIB_TIME = 2000;
int throttleCenterOffset = 0;

// ====== AJUSTES DE ESC ======
const int ESC_NEUTRAL  = 1500;
const int ESC_DEADBAND = 30;   // deadband +/- alrededor de neutro
const int ESC_FWD_MIN  = 1560;
const int ESC_REV_MAX  = 1440;

bool failsafeActive = false;

void setup() {
  Serial.begin(115200);

  radio.begin();
  radio.setChannel(90);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(2, 5);
  radio.openReadingPipe(0, address);
  radio.startListening();

  motorESC.attach(PIN_MOTOR);
  servoSteer.attach(PIN_STEER);
  servoCamX.attach(PIN_CAMX);
  servoCamY.attach(PIN_CAMY);
  pinMode(PIN_LIGHTS, OUTPUT);

  motorESC.writeMicroseconds(1500);
  servoSteer.writeMicroseconds(1500);
  servoCamX.writeMicroseconds(1500);
  servoCamY.writeMicroseconds(1500);
  digitalWrite(PIN_LIGHTS, LOW);

  Serial.println("Receptor con fail-safe, cámara, luces y ajuste fino de dirección.");
}

void loop() {
  if (radio.available()) {
    radio.read(&ctrl, sizeof(ctrl));
    lastSignalTime = millis();
    failsafeActive = false;

    // ===== AUTOCALIBRACIÓN ESC =====
    if (!throttleCalibrated) {
      if (calibStart == 0) {
        calibStart = millis();
        throttleSum = 0;
        throttleSamples = 0;
        Serial.println("Calibrando neutro del throttle...");
      }

      throttleSum += ctrl.throttle;
      throttleSamples++;

      if (millis() - calibStart >= CALIB_TIME) {
        throttleCenterOffset = throttleSum / throttleSamples;
        throttleCalibrated = true;
        Serial.print("Neutro calibrado: ");
        Serial.println(throttleCenterOffset);
      }

      motorESC.writeMicroseconds(ESC_NEUTRAL); // mantener neutro durante calibración
      return; // esperar a que termine calibración
    }

    // ===== ESC con deadband =====
    int t = ctrl.throttle - throttleCenterOffset;
    int motorVal = ESC_NEUTRAL;

    if (t > ESC_DEADBAND) {
      motorVal = map(t, ESC_DEADBAND, 1000, ESC_FWD_MIN, 2000);
    } 
    else if (t < -ESC_DEADBAND) {
      motorVal = map(t, -1000, -ESC_DEADBAND, 1000, ESC_REV_MAX);
    }

    // ===== Dirección =====
    int steerInput = constrain(ctrl.steer, -1000, 1000);
    steerInput = steerInput * steerLimitFactor;        // mantiene el límite de giro
    int steerVal = map(steerInput, -1000, 1000, 2000, 1000);  // mapeo directo

    // ===== Cámara =====
    int camXVal = map(ctrl.camX, -1000, 1000, 1000, 2000);
    int camYVal = map(ctrl.camY, -1000, 1000, 1000, 2000);

    // ===== Enviar a los servos =====
    motorESC.writeMicroseconds(motorVal);
    servoSteer.writeMicroseconds(steerVal);
    servoCamX.writeMicroseconds(camXVal);
    servoCamY.writeMicroseconds(camYVal);

    digitalWrite(PIN_LIGHTS, ctrl.lights ? HIGH : LOW);

    // ===== Debug =====
    Serial.print("Throttle: "); Serial.print(ctrl.throttle);
    Serial.print(" | Steer: "); Serial.print(ctrl.steer);
    Serial.print(" | µs: "); Serial.print(steerVal);
    Serial.print(" | CamX: "); Serial.print(ctrl.camX);
    Serial.print(" | CamY: "); Serial.print(ctrl.camY);
    Serial.print(" | Lights: "); Serial.println(ctrl.lights);
  } 
  else if (millis() - lastSignalTime > FAILSAFE_TIMEOUT) {
    if (!failsafeActive) {
      failsafeActive = true;
      Serial.println("⚠️ Fail-safe activado: sin señal");
    }

    motorESC.writeMicroseconds(ESC_NEUTRAL);
    servoSteer.writeMicroseconds(1500);
    servoCamX.writeMicroseconds(1500);
    servoCamY.writeMicroseconds(1500);
    digitalWrite(PIN_LIGHTS, LOW);
  }
}