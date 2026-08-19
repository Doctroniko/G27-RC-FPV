// ==================== DIRECCIÓN ====================

float steeringMinVoltage     = 0.0;
float steeringNeutralVoltage = 2.5;
float steeringMaxVoltage     = 5.0;

int steeringAngle = 900;       // 900 / 720 / 540
int encoderSlots  = 60;        // 60 / 30

int steeringTrim = 0;
bool steeringReverse = false;


// ================= THROTTLE / BRAKE =================

float throttleMinVoltage     = 0.0;
float throttleNeutralVoltage = 2.5;
float throttleMaxVoltage     = 5.0;

int throttleTrim = 0;
bool throttleReverse = false;

const int pedalDeadBand = 5;


// ============================================================
//                    CONFIGURACIÓN
// ============================================================

const int encoderPinA = 2;
const int encoderPinB = 3;
const int resetButtonPin = 4;
const int pwmEncoderPin = 5;

const int potAdelante = A0;
const int potReversa = A1;
const int potTrimSteer = A2;
const int potTrimThrot = A3;
const int pwmPotsPin = 9;


// ============================================================
//                    VARIABLES
// ============================================================

volatile int encoderPos = 0;
int lastEncoderPos = 0;
int lastHardwareSteeringTrim = 512;
int buttonState = 0;
int lastButtonState = 0;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;


// ============================================================
//                        SETUP
// ============================================================

void setup() {

  Serial.begin(9600);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(resetButtonPin, INPUT_PULLUP);
  pinMode(pwmEncoderPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  pinMode(pwmPotsPin, OUTPUT);

// Establecemos la dirección en NEUTRO al arrancar
int pwmNeutral =
  (int)(steeringNeutralVoltage / 5.0 * 255.0);

int hardwareSteeringTrim = map(
  analogRead(potTrimSteer),
  0,
  1023,
  -20,
  20
);

pwmNeutral =
  pwmNeutral +
  steeringTrim +
  hardwareSteeringTrim;

pwmNeutral =
  constrain(pwmNeutral, 0, 255);

analogWrite(
  pwmEncoderPin,
  pwmNeutral
);

Serial.println("Sistema combinado listo.");
}
// ============================================================
//                         LOOP
// ============================================================

void loop() {

  manejarEncoder();

  manejarPots();

  delay(10);
}


// ============================================================
//                    MANEJO DEL ENCODER
// ============================================================

void manejarEncoder() {

  int reading = digitalRead(resetButtonPin);

  if (reading != lastButtonState)
    lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading != buttonState) {

      buttonState = reading;

      if (buttonState == LOW) {

        encoderPos = 0;
        Serial.println("Encoder reseteado a 0");
      }
    }
  }

  lastButtonState = reading;


  // ========================================================
  // LEEMOS EL TRIM FÍSICO DE DIRECCIÓN
  // ========================================================

  int currentHardwareSteeringTrim = analogRead(potTrimSteer);


  // ========================================================
  // ACTUALIZAMOS EL PWM SI:
  // - HA CAMBIADO EL ENCODER
  // - O HA CAMBIADO EL TRIMPOT FÍSICO
  // ========================================================

  if (encoderPos != lastEncoderPos ||
      currentHardwareSteeringTrim != lastHardwareSteeringTrim) {


    // Convertimos las tensiones configuradas a PWM

    int pwmMin = (int)(steeringMinVoltage / 5.0 * 255.0);

    int pwmNeutral =
      (int)(steeringNeutralVoltage / 5.0 * 255.0);

    int pwmMax =
      (int)(steeringMaxVoltage / 5.0 * 255.0);


    // --------------------------------------------------------
    // Determinamos las cuentas correspondientes al ángulo
    // de giro seleccionado.
    // --------------------------------------------------------

    steeringAngle = constrain(steeringAngle, 270, 900);

    int encoderLimit = (int)(5.647 * steeringAngle - 180.0);


    // Si se utiliza un encoder de 30 ranuras,
    // reducimos aproximadamente a la mitad
    // el número de cuentas.

    if (encoderSlots == 30) {

      encoderLimit = encoderLimit / 2;
    }


    int pwmValue;


    int posicion = encoderPos;


    if (steeringReverse) {
      posicion = -posicion;
    }


    posicion = constrain(
      posicion,
      -encoderLimit,
      encoderLimit
    );


    if (posicion <= 0) {

      pwmValue = map(
        posicion,
        -encoderLimit,
        0,
        pwmMin,
        pwmNeutral
      );

    } else {

      pwmValue = map(
        posicion,
        0,
        encoderLimit,
        pwmNeutral,
        pwmMax
      );
    }


    // ========================================================
    // TRIM FÍSICO DE DIRECCIÓN
    // ========================================================

    int hardwareSteeringTrim = map(
      currentHardwareSteeringTrim,
      0,
      1023,
      -20,
      20
    );


    // ========================================================
    // SUMAMOS:
    //
    // PWM de dirección
    // + trim software
    // + trim físico
    // ========================================================

    pwmValue =
      pwmValue +
      steeringTrim +
      hardwareSteeringTrim;


    pwmValue = constrain(
      pwmValue,
      0,
      255
    );


    analogWrite(
      pwmEncoderPin,
      pwmValue
    );


    // ========================================================
    // MONITOR SERIE
    // ========================================================

    Serial.print("Encoder Pos: ");
    Serial.print(encoderPos);

    Serial.print(" | Trim SW: ");
    Serial.print(steeringTrim);

    Serial.print(" | Trim HW: ");
    Serial.print(hardwareSteeringTrim);

    Serial.print(" | PWM Encoder: ");
    Serial.println(pwmValue);


    // Guardamos los últimos valores

    lastEncoderPos = encoderPos;

    lastHardwareSteeringTrim =
      currentHardwareSteeringTrim;
  }
}

// ============================================================
//                MANEJO DE THROTTLE / BRAKE
// ============================================================

void manejarPots() {

  int valAdelante = analogRead(potAdelante);
  int valReversa  = analogRead(potReversa);

  if (throttleReverse) {
    int temp = valAdelante;
    valAdelante = valReversa;
    valReversa = temp;
  }

  float voltajeSalida = throttleNeutralVoltage;


  // ========================================================
  // CALCULAMOS EL MOVIMIENTO DE CADA PEDAL
  // ========================================================

  int throttleMovement = valAdelante;
  int brakeMovement    = valReversa;


  // ========================================================
  // DEAD BAND
  // ========================================================

  if (throttleMovement <= pedalDeadBand &&
      brakeMovement <= pedalDeadBand) {

    voltajeSalida = throttleNeutralVoltage;
  }


  // ========================================================
  // THROTTLE
  // ========================================================

  else if (throttleMovement > brakeMovement) {

    float fraccion =
      (throttleMovement - pedalDeadBand) /
      (1023.0 - pedalDeadBand);

    fraccion = constrain(fraccion, 0.0, 1.0);

    voltajeSalida =
      throttleNeutralVoltage +
      ((throttleMaxVoltage - throttleNeutralVoltage) * fraccion);
  }


  // ========================================================
  // BRAKE / REVERSA
  // ========================================================

  else {

    float fraccion =
      (brakeMovement - pedalDeadBand) /
      (1023.0 - pedalDeadBand);

    fraccion = constrain(fraccion, 0.0, 1.0);

    voltajeSalida =
      throttleNeutralVoltage -
      ((throttleNeutralVoltage - throttleMinVoltage) * fraccion);
  }


  // ========================================================
  // CONVERTIMOS LA TENSIÓN A PWM
  // ========================================================

  int pwmOut =
    (int)(voltajeSalida / 5.0 * 255.0);


  // ========================================================
  // TRIM SOFTWARE
  // ========================================================

  pwmOut = pwmOut + throttleTrim;


  // ========================================================
  // TRIM FÍSICO A3
  // ========================================================

  int hardwareThrottleTrim = map(
    analogRead(potTrimThrot),
    0,
    1023,
    -15,
    15
  );


  pwmOut =
    pwmOut +
    hardwareThrottleTrim;


  // ========================================================
  // LIMITAMOS PWM
  // ========================================================

  pwmOut = constrain(
    pwmOut,
    0,
    255
  );


  // ========================================================
  // SALIDA ESC
  // ========================================================

  analogWrite(
    pwmPotsPin,
    pwmOut
  );


  // ========================================================
  // MONITOR SERIE
  // ========================================================

  Serial.print("Pots -> Voltaje: ");
  Serial.print(voltajeSalida, 2);

  Serial.print(" V | PWM Pots: ");
  Serial.println(pwmOut);
}


// ============================================================
//                 INTERRUPCIÓN DEL ENCODER
// ============================================================

void updateEncoder() {

  static int lastEncoded = 0;

  int MSB = digitalRead(encoderPinA);

  int LSB = digitalRead(encoderPinB);

  int encoded = (MSB << 1) | LSB;

  int sum = (lastEncoded << 2) | encoded;


  if (
    sum == 0b1101 ||
    sum == 0b0100 ||
    sum == 0b0010 ||
    sum == 0b1011
  )
    encoderPos++;


  else if (
    sum == 0b1110 ||
    sum == 0b0111 ||
    sum == 0b0001 ||
    sum == 0b1000
  )
    encoderPos--;


  lastEncoded = encoded;
}