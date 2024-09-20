#include <Wire.h>
#include "Multichannel_Gas_GMXXX.h"
#include "DHT.h"
#include "TFT_eSPI.h"
#include "kavell-intellismell2_inferencing.h"

// Settings
#define SAMPLING_FREQ_HZ    4                         // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
#define NUM_SAMPLES         EI_CLASSIFIER_RAW_SAMPLE_COUNT  // 4 samples at 4 Hz
#define READINGS_PER_SAMPLE EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME // 8
#define DEBUG_NN            false                     // Print out EI debugging info

// Constants
#define PA_IN_KPA           1000.0                    // Convert Pa to KPa

GAS_GMXXX<TwoWire> gas;
TFT_eSPI tft;
#define DHTPIN 0
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Preprocessing constants
float mins[] = { 1.0, 0.53, 0.32, 0.7, 22.7, 47.0 };
float ranges[] = { 2.01, 2.03, 2.58, 2.57, 3.5, 35.0 };

void setup() {
  Serial.begin(115200);

  // Initialize TFT display
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  // Initialize gas sensors and DHT sensor
  gas.begin(Wire, 0x08);
  dht.begin();

  // Display initial screen layout
  tft.setFreeFont(&FreeSansBoldOblique18pt7b);
  tft.setTextColor(TFT_PURPLE);
  tft.drawString("Loading...", 70, 10, 1);
  
  // Draw static elements (lines, rectangles for display)
  for (int8_t line_index = 0; line_index < 5; line_index++) {
    tft.drawLine(0, 50 + line_index, tft.width(), 50 + line_index, TFT_WHITE);
  }

  // Draw the layout boxes and labels for sensors
  drawSensorLayout();
}

void drawSensorLayout() {
  // Temp Rect and Text
  tft.drawRoundRect(5, 60, (tft.width() / 2) - 20 , tft.height() - 60 , 10, TFT_WHITE); // Left box
  tft.setFreeFont(&FreeSansBoldOblique12pt7b);
  tft.setTextColor(TFT_PURPLE);
  tft.drawString("Temp ", 7 , 65 , 1);

  // Hum Text and Rect
  tft.drawString("Hum", 7 , 150 , 1);

  // VOC Rect and Text
  tft.drawRoundRect((tft.width() / 2) - 10 , 60, (tft.width() / 2) / 2 , 90 , 10, TFT_WHITE); // s1
  tft.drawString("VOC ", (tft.width() / 2) - 1, 70, 1);

  // NO2 Rect and Text
  tft.drawRoundRect(((tft.width() / 2) + (tft.width() / 2) / 2) - 5 , 60, (tft.width() / 2) / 2 , 90 , 10, TFT_WHITE); // s2
  tft.drawString("NO2", ((tft.width() / 2) + (tft.width() / 2) / 2), 70, 1);

  // CO Rect 
  tft.drawRoundRect((tft.width() / 2) - 10 , 150, (tft.width() / 2) / 2 , 90 , 10, TFT_WHITE); // s3
  tft.drawString("CO", (tft.width() / 2) - 1 , 160 , 1);

  // Ethyl Rect 
  tft.drawRoundRect(((tft.width() / 2) + (tft.width() / 2) / 2) - 5 , 150, (tft.width() / 2) / 2 , 90 , 10, TFT_WHITE); // s4
  tft.drawString("Ethyl", ((tft.width() / 2) + (tft.width() / 2) / 2), 160, 1);
}

void loop() {
  // Variables to store sensor readings
  float gm_no2_v, gm_eth_v, gm_voc_v, gm_co_v, temp, hum;
  float raw_buf[NUM_SAMPLES * READINGS_PER_SAMPLE];

  // Collect samples and perform inference
  for (int i = 0; i < NUM_SAMPLES; i++) {
    unsigned long timestamp = millis();

    // Read from GM-X02b sensors
    gm_no2_v = gas.calcVol(gas.getGM102B());
    gm_eth_v = gas.calcVol(gas.getGM302B());
    gm_voc_v = gas.calcVol(gas.getGM502B());
    gm_co_v = gas.calcVol(gas.getGM702B());
    temp = dht.readTemperature();
    hum = dht.readHumidity();

    // Store raw data into the buffer
    raw_buf[(i * READINGS_PER_SAMPLE) + 0] = gm_voc_v;
    raw_buf[(i * READINGS_PER_SAMPLE) + 1] = gm_no2_v;
    raw_buf[(i * READINGS_PER_SAMPLE) + 2] = gm_eth_v;
    raw_buf[(i * READINGS_PER_SAMPLE) + 3] = gm_co_v;
    raw_buf[(i * READINGS_PER_SAMPLE) + 4] = temp;
    raw_buf[(i * READINGS_PER_SAMPLE) + 5] = hum;

    // Perform normalization on all readings in the sample
    for (int j = 0; j < READINGS_PER_SAMPLE; j++) {
      raw_buf[(i * READINGS_PER_SAMPLE) + j] = (raw_buf[(i * READINGS_PER_SAMPLE) + j] - mins[j]) / ranges[j];
    }

    // Wait for sampling period
    while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }

  // Perform inference
  signal_t signal;
  numpy::signal_from_buffer(raw_buf, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  ei_impulse_result_t result;
  run_classifier(&signal, &result, DEBUG_NN);

  // Find the highest prediction score
  int max_idx = 0;
  float max_val = 0.0;
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > max_val) {
      max_val = result.classification[i].value;
      max_idx = i;
    }
  }

  // Clear the "loading" text and display the prediction result and score
  tft.fillRect(10, 10, 2000, 40, TFT_BLACK);
  String predictionLabel = String(result.classification[max_idx].label);
  String predictionScore = String(result.classification[max_idx].value, 2); // Limit score to 2 decimal places
  tft.drawString(predictionLabel + ": " + predictionScore, 20, 10, 1);
  
  // Update the display with sensor values
  updateSensorDisplay(gm_voc_v, gm_no2_v, gm_eth_v, gm_co_v, temp, hum);
}

void updateSensorDisplay(float voc, float no2, float ethyl, float co, float temp, float hum) {
  // Clear previous values
  tft.fillRect(20, 95, 60, 40, TFT_BLACK);   // Temp
  tft.fillRect(20, 175, 60, 40, TFT_BLACK);  // Hum
  tft.fillRect(160, 95, 60, 40, TFT_BLACK);  // VOC
  tft.fillRect(240, 95, 60, 40, TFT_BLACK);  // NO2
  tft.fillRect(160, 185, 60, 40, TFT_BLACK); // CO 
  tft.fillRect(240, 185, 60, 40, TFT_BLACK); // Ethyl
  
  // Display updated sensor values
  tft.setFreeFont(&FreeSansBoldOblique12pt7b);
  tft.drawString(String(temp, 1), 25, 100, 1);  // Temperature
  tft.drawString(String(hum, 1), 25, 190, 1);   // Humidity
  tft.drawString(String(voc, 2), 145 + 15, 100, 1);  // VOC 
  tft.drawString(String(no2, 2), 230 + 15, 100, 1);  // NO2 
  tft.drawString(String(co, 2), 145 + 15, 190, 1);   // CO 
  tft.drawString(String(ethyl, 2), 230 + 15, 190, 1); // Ethyl 
}
