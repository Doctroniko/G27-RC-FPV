// ==================== DIRECCIÓN ====================

float steeringMinVoltage     = 0.0;
float steeringNeutralVoltage = 2.5;
float steeringMaxVoltage     = 5.0;

int steeringAngle = 900;       // 270-900
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
const int pwmPotsPin = 9;


// ============================================================
//                    VARIABLES
// ============================================================

volatile int encoderPos = 0;
int lastEncoderPos = 0;

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
int pwmNeutral = (int)(steeringNeutralVoltage / 5.0 * 255.0);
pwmNeutral = pwmNeutral + steeringTrim;
pwmNeutral = constrain(pwmNeutral, 0, 255);
analogWrite(pwmEncoderPin, pwmNeutral);

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


  if (encoderPos != lastEncoderPos) {

    // Convertimos la posición del encoder en PWM
    // utilizando las tensiones configuradas arriba.

    int pwmMin = (int)(steeringMinVoltage / 5.0 * 255.0);

    int pwmNeutral =
      (int)(steeringNeutralVoltage / 5.0 * 255.0);

    int pwmMax =
      (int)(steeringMaxVoltage / 5.0 * 255.0);


    // --------------------------------------------------------
    // Determinamos las cuentas correspondientes al ángulo
    // de giro seleccionado.
    // --------------------------------------------------------

   steeringAngle = constrain(steeringAngle, 270, 900); // Fuerza a que esté entre 360 y 900
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
    
posicion = constrain(posicion, -encoderLimit, encoderLimit);

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


    // Aplicamos el TRIM
    pwmValue = pwmValue + steeringTrim;

    pwmValue = constrain(pwmValue, 0, 255);

    analogWrite(pwmEncoderPin, pwmValue);


    Serial.print("Encoder Pos: ");
    Serial.print(encoderPos);

    Serial.print(" | PWM Encoder: ");
    Serial.println(pwmValue);

    lastEncoderPos = encoderPos;
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
  //
  // Si ninguno de los pedales ha salido de la zona muerta,
  // mantenemos la salida en NEUTRO.
  //
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
  // CONVERTIMOS LA TENSIÓN CALCULADA A PWM
  // ========================================================

  int pwmOut =
    (int)(voltajeSalida / 5.0 * 255.0);


  // ========================================================
  // APLICAMOS EL TRIM
  // ========================================================

  pwmOut = pwmOut + throttleTrim;

  pwmOut = constrain(pwmOut, 0, 255);

  analogWrite(pwmPotsPin, pwmOut);


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