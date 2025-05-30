// ESP32-CAM con detección de color usando HSV y 5 cuadrantes
#include "esp_camera.h"
#include "Arduino.h"
#include "driver/uart.h"

// Configuración de pines
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define TX_PIN 14
#define RX_PIN 15

const int W = 160;
const int H = 120;

const int NUM_COLORES = 3;
const int HUE_MIN[NUM_COLORES] = {2, 60, 290};
const int HUE_MAX[NUM_COLORES] = {10, 90, 320};
const int SAT_MIN[NUM_COLORES] = {120, 120, 120};
const int SAT_MAX[NUM_COLORES] = {255, 255, 255};
const int VAL_MIN[NUM_COLORES] = {120, 120, 120};
const int VAL_MAX[NUM_COLORES] = {255, 255, 255};
//const char *NOMBRE_COLORES[NUM_COLORES] = {"Rojo", "Verde", "Magenta"};
const int NOMBRE_COLORES[NUM_COLORES]={10,20,30};

//String auxColor;
int auxColor;
int ubi = -1;
int colorDetectado = -1;
uint8_t *mask_buffer;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Error al iniciar la camara");
    return;
  }

  mask_buffer = (uint8_t *)malloc(W * H);
  if (!mask_buffer) {
    Serial.println("Error: No hay suficiente RAM.");
  }
}


//Los cuadrantes van de Izq a Der, cada color es una decena, los cuadrantes unidades, con función de residuo obtendremos el valor
void loop() {
  if (!mask_buffer) {
    ESP.restart();
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  process_image(fb->buf, mask_buffer);
  show_result();

  esp_camera_fb_return(fb);

  Serial2.println(auxColor);
  Serial2.flush();

  delay(100);
}

void process_image(uint8_t *imageIn, uint8_t *mask_buffer) {
  long suma_x = 0;
  int cuenta = 0;
  colorDetectado = -1;

  for (int i = 0; i < W * H; i++) {
    uint8_t r = (imageIn[i * 2] & 0xF8);
    uint8_t g = ((imageIn[i * 2] & 0x07) << 5) | ((imageIn[i * 2 + 1] & 0xE0) >> 3);
    uint8_t b = (imageIn[i * 2 + 1] & 0x1F) << 3;

    uint8_t maxVal = max(r, max(g, b));
    uint8_t minVal = min(r, min(g, b));
    uint8_t delta = maxVal - minVal;

    float h = 0;
    if (delta > 0) {
      if (maxVal == r) h = 60 * ((g - b) / (float)delta);
      else if (maxVal == g) h = 60 * (2 + (b - r) / (float)delta);
      else h = 60 * (4 + (r - g) / (float)delta);
      if (h < 0) h += 360;
    }
    uint8_t s = (maxVal == 0) ? 0 : (delta * 255 / maxVal);
    uint8_t v = maxVal;

    for (int c = 0; c < NUM_COLORES; c++) {
      if (h >= HUE_MIN[c] && h <= HUE_MAX[c] &&
          s >= SAT_MIN[c] && s <= SAT_MAX[c] &&
          v >= VAL_MIN[c] && v <= VAL_MAX[c]) {
        int x = i % W;
        suma_x += x;
        cuenta++;
        colorDetectado = c;
        break;
      }
    }
  }

  if (cuenta > 0) {
    int x_prom = suma_x / cuenta;
    if (x_prom < 32) ubi = 1;
    else if (x_prom < 64) ubi = 2;
    else if (x_prom < 96) ubi = 3;
    else if (x_prom < 128) ubi = 4;
    else ubi = 5;
  } else {
    colorDetectado = -1;
    ubi = 0;
  }
}

void show_result() {
  if (colorDetectado == -1) {
    auxColor = 0;
    Serial.println("Nada detectado");
  } else {
    auxColor = (NOMBRE_COLORES[colorDetectado]+ubi);
    Serial.print("Color "); Serial.print(auxColor);
    Serial.print(" detectado en cuadrante "); Serial.println(ubi);
  }
}
