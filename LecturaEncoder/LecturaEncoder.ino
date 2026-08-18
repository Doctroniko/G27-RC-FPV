// Pines del encoder
const byte encoderA = 2;
const byte encoderB = 3;

volatile long posicion = 0;
long ultimaPosicion = 0;

void setup()
{
    Serial.begin(9600);

    pinMode(encoderA, INPUT_PULLUP);
    pinMode(encoderB, INPUT_PULLUP);

    // Solo un flanco del canal A
    attachInterrupt(digitalPinToInterrupt(encoderA), leerEncoder, RISING);
}

void loop()
{
    if (posicion != ultimaPosicion)
    {
        noInterrupts();
        long p = posicion;
        interrupts();

        Serial.println(p);

        ultimaPosicion = p;
    }
}

void leerEncoder()
{
    if (digitalRead(encoderB))
        posicion++;
    else
        posicion--;
}