// ESP32-CAM con detección de color (Rojo, Verde, Magenta) usando HSV y 5 sectores
#include "esp_camera.h"
#include "Arduino.h"

// Configuración de pines AI-Thinker
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
// Rangos HSV ajustados (puedes afinarlos experimentalmente)
const int HUE_MIN[NUM_COLORES] = {0,   37, 280};   // Rojo, Verde, Magenta
const int HUE_MAX[NUM_COLORES] = {18,  110, 320};
const int SAT_MIN[NUM_COLORES] = {100, 100, 100};
const int SAT_MAX[NUM_COLORES] = {255, 255, 255};
const int VAL_MIN[NUM_COLORES] = {100, 65, 100};
const int VAL_MAX[NUM_COLORES] = {255, 255, 255};

const int NOMBRE_COLORES[NUM_COLORES] = {10, 20, 30}; // Codificación
const int MIN_PIXELES_VALIDOS = 60;  // Número mínimo de píxeles para aceptar un color
//Antes en 70 min pixeles

int ubi = 0;
int colorDetectado = -1;
int auxColor = 0;

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
    Serial.println("Error al iniciar la cámara");
    return;
  }
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  process_image(fb->buf);
  show_result();

  esp_camera_fb_return(fb);

  Serial2.println(auxColor);
  Serial2.flush();

  delay(60); // sacrificamos velocidad por estabilidad
}

void process_image(uint8_t *imageIn) {
  long suma_x[NUM_COLORES] = {0};
  int cuenta[NUM_COLORES] = {0};

  for (int i = 0; i < W * H; i++) {
    uint8_t r = (imageIn[i * 2] & 0xF8);
    uint8_t g = ((imageIn[i * 2] & 0x07) << 5) | ((imageIn[i * 2 + 1] & 0xE0) >> 3);
    uint8_t b = (imageIn[i * 2 + 1] & 0x1F) << 3;

    uint8_t maxVal = max(r, max(g, b));
    uint8_t minVal = min(r, min(g, b));
    uint8_t delta = maxVal - minVal;

    float h = 0;
    if (delta > 0) {
      if (maxVal == r) h = 60 * fmod(((g - b) / (float)delta), 6);
      else if (maxVal == g) h = 60 * (((b - r) / (float)delta) + 2);
      else h = 60 * (((r - g) / (float)delta) + 4);
      if (h < 0) h += 360;
    }
    uint8_t s = (maxVal == 0) ? 0 : (delta * 255 / maxVal);
    uint8_t v = maxVal;

    // Chequeo de colores
    for (int c = 0; c < NUM_COLORES; c++) {
      bool dentro = false;

      if (c == 0) {
        // 🔴 Rojo → dos rangos: [340–360] o [0–10]
        if (((h >= 340 && h <= 360) || (h >= 0 && h <= 10)) &&
            s >= SAT_MIN[c] && s <= SAT_MAX[c] &&
            v >= VAL_MIN[c] && v <= VAL_MAX[c]) {
          dentro = true;
        }
      } else {
        // 🟢 Verde → rango simple
        if (h >= HUE_MIN[c] && h <= HUE_MAX[c] &&
            s >= SAT_MIN[c] && s <= SAT_MAX[c] &&
            v >= VAL_MIN[c] && v <= VAL_MAX[c]) {
          dentro = true;
        }
      }

      if (dentro) {
        int x = i % W;
        suma_x[c] += x;
        cuenta[c]++;
        break;
      }
    }
  }

  // Selección del color dominante
  int maxCuenta = 0;
  colorDetectado = -1;
  for (int c = 0; c < NUM_COLORES; c++) {
    if (cuenta[c] > maxCuenta) {
      maxCuenta = cuenta[c];
      colorDetectado = c;
    }
  }

  if (colorDetectado != -1 && maxCuenta > MIN_PIXELES_VALIDOS) {
    int x_prom = suma_x[colorDetectado] / maxCuenta;
    if (x_prom < 32) ubi = 1;
    else if (x_prom < 64) ubi = 2;
    else if (x_prom < 96) ubi = 3;
    else if (x_prom < 128) ubi = 4;
    else ubi = 5;
  } else {
    colorDetectado = -1;
  ubi=0;
}
}


void show_result() {
  if (colorDetectado == -1) {
    auxColor = 0;
    Serial.println("Nada detectado");
  } else {
    auxColor = (NOMBRE_COLORES[colorDetectado] + ubi);
    Serial.print("Color ");
    Serial.print(auxColor);
    Serial.print(" detectado en cuadrante ");
    Serial.println(ubi);
  }
}
