#include "esp_camera.h"
#include "Arduino.h"
#include "driver/uart.h"

// Configuración de pines de la ESP32-CAM
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

// Dimensiones de la imagen
const int W = 160;
const int H = 120;

const int NUM_COLORES = 4;  
const int HUE_MIN[NUM_COLORES] = {0, 40, 290};
const int HUE_MAX[NUM_COLORES] = {10, 80, 320};
const int SAT_MIN[NUM_COLORES] = {90, 90, 90};  
const int SAT_MAX[NUM_COLORES] = {250, 250, 250}; 
const int VAL_MIN[NUM_COLORES] = {65, 65, 65};  
const int VAL_MAX[NUM_COLORES] = {235, 235, 235};  
const char *NOMBRE_COLORES[NUM_COLORES] = {"Rojo", "Ver", "Mag", "Nada"};
const int positions[] = {7, 6, 5, 4, 3, 2, 1}; 
String auxColor;
int ubi;
int c;

#define TX_PIN 14
#define RX_PIN 15

uint8_t *mask_buffer;  // Se declara global para evitar malloc() en cada loop

void setup() {
    Serial.begin(115200);

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
    Serial.println("Cámara iniciada correctamente.");
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

    mask_buffer = (uint8_t *)malloc(W * H);
    if (!mask_buffer) {
        Serial.println("Error: No hay suficiente RAM.");
    }
}

void loop() {
    if (!mask_buffer) {
        Serial.println("No hay memoria, reiniciando...");
        delay(1000);
        ESP.restart();  // Reinicia el ESP32 si no hay RAM
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Error al obtener el frame");
        return;
    }

    Serial.println("Procesando imagen...");
    int color_detectado = -1;
    process_image(fb->buf, mask_buffer, color_detectado);
    show_mask_display(mask_buffer, color_detectado);

    esp_camera_fb_return(fb);

    Serial2.println(auxColor);
    Serial2.flush();  // Asegura que se envíen todos los datos antes de continuar
}

void process_image(uint8_t *imageIn, uint8_t *mask_buffer, int &color_detectado) {
    color_detectado = -1;
    int color_counts[NUM_COLORES] = {0};

    for (int i = 0; i < W * H; i++) {
        uint8_t r = (imageIn[i * 2] & 0xF8);
        uint8_t g = ((imageIn[i * 2] & 0x07) << 5) | ((imageIn[i * 2 + 1] & 0xE0) >> 3);
        uint8_t b = (imageIn[i * 2 + 1] & 0x1F) << 3;

        uint8_t maxVal = max(r, max(g, b));
        uint8_t minVal = min(r, min(g, b));
        uint8_t delta = maxVal - minVal;  
        uint8_t h = 0;
        if (delta > 0) {  
            if (maxVal == r) {
                h = 60 * ((g - b) / (float)delta);
                if (h < 0) h += 360;
            } else if (maxVal == g) {
                h = 60 * (2.0 + (b - r) / (float)delta);
            } else {
                h = 60 * (4.0 + (r - g) / (float)delta);
            }
        }

        uint8_t s = (maxVal == 0) ? 0 : (delta * 255 / maxVal);
        uint8_t v = maxVal;

        for (int c = 0; c < NUM_COLORES; c++) {
            if (h >= HUE_MIN[c] && h <= HUE_MAX[c] &&
                s >= SAT_MIN[c] && s <= SAT_MAX[c] &&
                v >= VAL_MIN[c] && v <= VAL_MAX[c]) {
                mask_buffer[i] = 0;
                color_counts[c]++;
                break;
            } else {
                mask_buffer[i] = 255;
            }
        }
    }

    int max_count = 0;
    for (c = 0; c < NUM_COLORES; c++) {
        if (color_counts[c] > max_count) {
            max_count = color_counts[c];
            color_detectado = c;
        }
    }
}

void show_mask_display(uint8_t *mask_buffer, int color_detectado) {
    if (color_detectado == -1) {
      auxColor="Nada";
        Serial.println(auxColor + " detectado");
    } else {
        Serial.print("Color ");
        auxColor=NOMBRE_COLORES[color_detectado];
        Serial.print(auxColor);
        Serial.println(" detectado");
        Serial.print("En posicion: ");
        ubi=positions[c];
        Serial.println(ubi);
    }
}
