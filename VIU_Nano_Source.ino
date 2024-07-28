/* 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Código para leer 2 acelerómetros MMA8451.
Se incorpora un botón para iniciar y finalizar la lectura. Además, una luz del indicadora de estado (On/Off).
El codigo y hardwares asociados, fueron testeados con un analziador lógico para verificar que la frecuencia de muestreo sea de 100 Hz.

### Acelerometro 1 ####
Vi --> Vi arduino
GND --> GND arduino
SCL --> SCL arduino
SDA --> SDA arduino
A0 --> GND arduino (genera la direccion 0x1C)

### Acelerometro 2 ####
Vi --> Vi arduino
GND --> GND arduino
SCL --> SCL arduino
SDA --> SDA arduino
A0 --> 3.3V arduino (genera la direccion 0x1D)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

// Creación de instancias de los acelerómetros MMA8451, especificando las direcciones I2C
Adafruit_MMA8451 mma1 = Adafruit_MMA8451(0x1D);
Adafruit_MMA8451 mma2 = Adafruit_MMA8451(0x1C);

// Definición de los pines para el botón y el LED
const int BUTTON_PIN = 2;
const int LED_PIN = 4;

// Variables para el manejo de la interrupción y el control de la adquisición de datos
volatile bool readSensorFlag = false; // Bandera para leer los sensores, modificada en la ISR (Rutina de Servicio de Interrupción)
bool startAcquisition = false; // Control para el inicio/fin de la adquisición de datos
bool lastButtonState = LOW; // Estado anterior del botón para detectar flancos
unsigned long lastDebounceTime = 0; // Tiempo para controlar el rebote del botón

void setup() {
  Serial.begin(74880); // Inicio de la comunicación serial a 74880 baudios para transmitir datos (mantener o subir)
  pinMode(BUTTON_PIN, INPUT); // Conf.Pin del botón como entrada
  pinMode(LED_PIN, OUTPUT); // Conf.Pin del led como salida
  digitalWrite(LED_PIN, LOW); // Iniciar con led apagado

 // Inicialización de los acelerómetros
  if (!mma1.begin(0x1D) || !mma2.begin(0x1C)) { // Verificar la conexión con los acelerómetros
    Serial.println("Error en iniciar MMA8451"); // Mensaje de error si no se conectan
    while (1); // Bucle infinito si la conexión falla
  }
  mma1.setRange(MMA8451_RANGE_4_G); // Configura el mma1 para medir en un rango de hasta ±4g
  mma1.setDataRate(MMA8451_DATARATE_100_HZ); // Configura la tasa de muestreo a 100 Hz del mma1
  mma2.setRange(MMA8451_RANGE_4_G); // Configura el mma2 para medir en un rango de hasta ±4g
  mma2.setDataRate(MMA8451_DATARATE_100_HZ); // Configura la tasa de muestreo a 100 Hz del mma2

 // Configuración del Timer1 para generar interrupciones cada 10 ms (100 Hz)
  noInterrupts(); // Detención de interrupciones durante la configuración
  TCCR1A = 0; // Configuración del Timer1 en modo normal
  TCCR1B = 0;
  OCR1A = 2499;  // // Valor para el comparador A del Timer1. Ajuste correcto para 100Hz con un preescalador de 64 
  TCCR1B |= (1 << WGM12);  // Modo CTC (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11) | (1 << CS10);  // Preescalador a 64. Divide la frecuencia del reloj principal del uC para reducir la frecuencia con la que se incrementa el contador del timer
  TIMSK1 |= (1 << OCIE1A); // Habilitación de la interrupción por comparación del Timer
  interrupts(); // Reanuda las interrupciones
}

// Rutina de Servicio de Interrupción (ISR) para Timer1
ISR(TIMER1_COMPA_vect) {
  readSensorFlag = true; // Establece la bandera para leer datos en el loop principal
}
// Bucle principal
void loop() {
  // Lectura y debouncing del botón
  bool currentButtonState = digitalRead(BUTTON_PIN); // Lee el estado actual del botón
  if (currentButtonState != lastButtonState && (millis() - lastDebounceTime) > 500) {
    lastButtonState = currentButtonState; // Actualiza el último estado del botón
    if (currentButtonState == HIGH) { // Condición si el botón está presionado
      startAcquisition = !startAcquisition; // Cambia el estado de adquisición de datos
      digitalWrite(LED_PIN, startAcquisition ? HIGH : LOW); // Enciende o apaga el led
      lastDebounceTime = millis(); // Reinicia el temporizador de rebote
    }
  }
 // Adquisición y envío de datos de los acelerómetros
  if (startAcquisition && readSensorFlag) {
    mma1.read(); // Lee los datos del primer acelerómetro
    mma2.read(); // Lee los datos del segundo acelerómetro
   // Envío de datos por serial en un formato legible (configurado para verificar en serialplotter)
   /*
    Serial.print("A1X:"); Serial.print(mma1.x);
    Serial.print(" A1Y:"); Serial.print(mma1.y);
    Serial.print(" A1Z:"); Serial.print(mma1.z);
    Serial.print(" A2X:"); Serial.print(mma2.x);
    Serial.print(" A2Y:"); Serial.print(mma2.y);
    Serial.print(" A2Z:"); Serial.println(mma2.z);
  */

  // Envío de datos por serial en formato de bytes a la Raspberry Pi:

    // Obtener los valores del primer acelerómetro
    int16_t accelerometer1X = mma1.x;
    int16_t accelerometer1Y = mma1.y;
    int16_t accelerometer1Z = mma1.z;

    // Obtener los valores del segundo acelerómetro
    int16_t accelerometer2X = mma2.x;
    int16_t accelerometer2Y = mma2.y;
    int16_t accelerometer2Z = mma2.z;
  
    Serial.write('!'); // Carácter que indica a la Raspberry Pi iniciar la lectura
    Serial.write((uint8_t*)&accelerometer1X, sizeof(accelerometer1X));
    Serial.write((uint8_t*)&accelerometer1Y, sizeof(accelerometer1Y));
    Serial.write((uint8_t*)&accelerometer1Z, sizeof(accelerometer1Z));
    Serial.write((uint8_t*)&accelerometer2X, sizeof(accelerometer2X));
    Serial.write((uint8_t*)&accelerometer2Y, sizeof(accelerometer2Y));
    Serial.write((uint8_t*)&accelerometer2Z, sizeof(accelerometer2Z));
    Serial.write('\n');

  
    readSensorFlag = false; // Reinicio de bandera para la próxima lectura
  }
}
