#include <Wire.h>
#include "DHT.h"
#include "Multichannel_Gas_GMXXX.h"

#define BTN_START           0                         // 1: press button to start, 0: loop
#define BTN_PIN             WIO_5S_PRESS              // Pin that button is connected to
#define SAMPLING_FREQ_HZ    4                         // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
#define NUM_SAMPLES         8                         // 8 samples at 4 Hz is 2 seconds

#define BME680_I2C_ADDR     uint8_t(0x76)             // I2C address of BME680
#define PA_IN_KPA           1000.0                    // Convert Pa to KPa

GAS_GMXXX<TwoWire> gas;                               // Multichannel gas sensor v2
#define DHTPIN 0
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

void setup() {

  // Initialize button
  pinMode(BTN_PIN, INPUT_PULLUP);
  
  // Start serial
  Serial.begin(115200);

  // Initialise gas sensors
  gas.begin(Wire, 0x08);
  
  // Initialise temp and hum sensors 
  dht.begin();
}

void loop() {

  float no2;
  float eth;
  float voc;
  float co;
  float temp;
  float hum;

  unsigned long timestamp;
  int tem = temp;

  // Wait for button press
#if BTN_START
  while (digitalRead(BTN_PIN) == 1);
#endif

  // Header
  Serial.println("timestamp,voc,no2,eth,co,temp,hum");

  // Transmit samples over serial port
  for (int i = 0; i < NUM_SAMPLES; i++) {

    timestamp = millis();

    // Read from multichannel gas sensors
    no2 = gas.calcVol(gas.getGM102B());
    eth = gas.calcVol(gas.getGM302B());
    voc = gas.calcVol(gas.getGM502B());
    co = gas.calcVol(gas.getGM702B());

    // Read from temp and hum sensors
    temp = dht.readTemperature();
    hum = dht.readHumidity();
  
    // Print .CSV data with timestamp
    Serial.print(timestamp);
    Serial.print(",");
    Serial.print(voc);
    Serial.print(",");
    Serial.print(no2);
    Serial.print(",");
    Serial.print(eth);
    Serial.print(",");
    Serial.print(co);
    Serial.print(",");
    Serial.print(temp);
    Serial.print(",");
    Serial.print(hum);
    Serial.println();

    // Wait just long enough for our sampling period
    while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }

  // Print empty line to transmit termination of recording
  Serial.println();

  // Make sure the button has been released for a few milliseconds
#if BTN_START
  while (digitalRead(BTN_PIN) == 0);
  delay(100);
#endif
}
