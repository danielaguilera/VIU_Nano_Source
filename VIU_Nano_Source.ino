// A la fecha del 05-02-2024, el presente codigo permite un resgisto adecuado del sensor mma8451
// presenta una decuando funcionamiento entre el pulsador y el accionamiento del led 
// la frecuencia de muestre medido mediante analizador logico es aprox 100 HZ

#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();

const int BUTTON_PIN = 2;  // Pulsador conectado a D2
const int LED_PIN = 4;     // LED conectado a D4
bool startAcquisition = false;
volatile bool readSensorFlag = false;  // Bandera para indicar cuando leer el sensor
bool lastButtonState = LOW;  // Estado anterior del pulsador

void setup() 
{
  Serial.begin(19200);  // Inicializa la comunicación serie con la Raspberry Pi
  
  pinMode(BUTTON_PIN, INPUT);  // Configura el pin del pulsador como entrada
  pinMode(LED_PIN, OUTPUT);    // Configura el pin del LED como salida
  
  digitalWrite(LED_PIN, LOW);  // Asegurar que el LED esté apagado inicialmente

  if (!mma.begin(0x1C))
  {
    Serial.println("Error de comunicación MM8451");
    while (1);  // Detiene el bucle si no inicia 
  }
  
  mma.setDataRate(MMA8451_DATARATE_100_HZ);
  mma.setRange(MMA8451_RANGE_4_G);
  Serial.println("MMA8451 configurado y listo!");

  // Configuración de Timer1 para interrupción cada 10 ms
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 155;  
  TCCR1B |= (1 << WGM12);  // Modo CTC
  TCCR1B |= (1 << CS12) | (1 << CS10);  // Preescalador de 1024
  TIMSK1 |= (1 << OCIE1A);  // Habilitar interrupción por comparación de Timer1
  interrupts();
}

ISR(TIMER1_COMPA_vect)  // Rutina de interrupción del Timer1
{
  readSensorFlag = true;  // Establecer la bandera para indicar que es hora de leer el sensor
}

void loop()
{
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (currentButtonState == HIGH && lastButtonState == LOW)
  {
    startAcquisition = !startAcquisition;  // Cambia el estado de adquisición (toggle)
    digitalWrite(LED_PIN, startAcquisition ? HIGH : LOW);  // Enciende o apaga el LED según el estado de adquisición

    delay(500);  // Debounce delay
  }
  lastButtonState = currentButtonState;  // Actualiza el estado anterior del pulsador

  if (startAcquisition && readSensorFlag)
  {
    mma.read();

    // Obtener los valores del acelerómetro en formato de dos bytes (int16_t)
    // los datos del acelerómetro (X, Y, Z) se envían como valores enteros de 16 bits (int16_t), 
    // lo que significa que cada valor del acelerómetro se representa con 2 bytes. 
    //Entonces, cuando se transmite un conjunto de datos del acelerómetro (X, Y, Z), se están enviando 6 bytes en total (2 bytes por cada eje).
    int16_t accelerometerX = mma.x;
    int16_t accelerometerY = mma.y;
    int16_t accelerometerZ = mma.z;

    // Enviar los datos al puerto serie de la Raspberry Pi
    Serial.write('!'); // Carácter que indica a la Raspberry Pi iniciar la lectura
    Serial.write((uint8_t*)&accelerometerX, sizeof(accelerometerX));
    Serial.write((uint8_t*)&accelerometerY, sizeof(accelerometerY));
    Serial.write((uint8_t*)&accelerometerZ, sizeof(accelerometerZ));
    Serial.write('\n');
    
    // Restablecer la bandera 'readSensorFlag'.
    readSensorFlag = false;
  }
}

void displayData()
{
  // Read the 'raw' data in 14-bit counts
  mma.read();
  Serial.print("X:\t"); Serial.print(mma.x); 
  Serial.print("\tY:\t"); Serial.print(mma.y); 
  Serial.print("\tZ:\t"); Serial.print(mma.z); 
  Serial.println();

  /* Get a new sensor event */ 
  sensors_event_t event; 
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
  Serial.println("m/s^2 ");
  
  /* Get the orientation of the sensor */
  uint8_t o = mma.getOrientation();
  
  switch (o) {
    case MMA8451_PL_PUF: 
      Serial.println("Portrait Up Front");
      break;
    case MMA8451_PL_PUB: 
      Serial.println("Portrait Up Back");
      break;    
    case MMA8451_PL_PDF: 
      Serial.println("Portrait Down Front");
      break;
    case MMA8451_PL_PDB: 
      Serial.println("Portrait Down Back");
      break;
    case MMA8451_PL_LRF: 
      Serial.println("Landscape Right Front");
      break;
    case MMA8451_PL_LRB: 
      Serial.println("Landscape Right Back");
      break;
    case MMA8451_PL_LLF: 
      Serial.println("Landscape Left Front");
      break;
    case MMA8451_PL_LLB: 
      Serial.println("Landscape Left Back");
      break;
    }  
}
