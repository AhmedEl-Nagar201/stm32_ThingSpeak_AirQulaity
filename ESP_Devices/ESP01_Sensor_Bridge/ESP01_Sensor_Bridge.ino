/**
 * @file    ESP01_Sensor_Bridge.ino
 * @brief   ESP-01 (ESP8266) – Sensor Bridge Firmware
 *
 * Receives a compact binary frame from the STM32 over UART (9600 baud),
 * validates the checksum, then transmits the payload to the ESP32-S3
 * gateway via ESP-NOW.
 *
 * === UART Frame Format (STM32 → ESP-01) ===
 *  Byte  0    : SOF  = 0xAA
 *  Byte  1    : LEN  = payload length in bytes (= 30)
 *  Bytes 2-31 : Payload (see SensorPayload_t)
 *  Byte  32   : CRC  = XOR of bytes 0..31
 *  Byte  33   : EOF  = 0x55
 *  Total: 34 bytes
 *
 * === Payload Layout ===
 *  float  temperature  (4 bytes)
 *  float  humidity     (4 bytes)
 *  float  co2          (4 bytes)
 *  float  nh3          (4 bytes)
 *  float  co           (4 bytes)
 *  float  alcohol      (4 bytes)
 *  float  toluene      (4 bytes)
 *  uint16_t adc_raw    (2 bytes)
 *  → Total payload: 30 bytes
 *
 * Board: "Generic ESP8266 Module"
 * Flash:  1MB   Upload speed: 115200
 * CPU:    80 MHz
 *
 * Required library: ESP8266 core (ESP-NOW is included in the core)
 */

#include <ESP8266WiFi.h>
#include <espnow.h>

/* ---- Configuration ------------------------------------------------------ */

/* MAC address of the ESP32-S3 gateway (Wi-Fi Station MAC).
 * From your serial log: 3C:0F:02:D9:03:50
 * Stored as bytes: */
static uint8_t GATEWAY_MAC[] = { 0x3C, 0x0F, 0x02, 0xD9, 0x03, 0x50 };

/* UART baud rate matching STM32 USART3 */
#define UART_BAUD    115200

/* Frame delimiters */
#define FRAME_SOF    0xAA
#define FRAME_EOF    0x55
#define FRAME_LEN    34   /* total bytes per frame */
#define PAYLOAD_LEN  30   /* 7 floats + 1 uint16 = 30 bytes */

/* ---- Shared Data Structures --------------------------------------------- */
/* MUST match the struct in STM32 main.c and gateway sketch */

#pragma pack(push, 1)
typedef struct {
    float    temperature;   /* °C       */
    float    humidity;      /* %RH      */
    float    co2;           /* ppm      */
    float    nh3;           /* ppm      */
    float    co;            /* ppm      */
    float    alcohol;       /* ppm      */
    float    toluene;       /* ppm      */
    uint16_t adc_raw;       /* raw ADC  */
} SensorPayload_t;
#pragma pack(pop)

/* ---- Private State ------------------------------------------------------ */

static uint8_t  rx_buf[FRAME_LEN];
static uint8_t  rx_idx = 0;
static bool     in_frame = false;

static volatile bool espnow_sent = false;
static volatile bool espnow_ok   = false;

/* ---- ESP-NOW Callback --------------------------------------------------- */

void onSendCallback(uint8_t *mac, uint8_t status)
{
    espnow_sent = true;
    espnow_ok   = (status == 0);
}

/* ---- Frame Processing --------------------------------------------------- */

/**
 * @brief  Validate and unpack a complete 32-byte frame.
 * @return true if checksum is valid, false otherwise.
 */
bool validateFrame(const uint8_t *buf, SensorPayload_t *out)
{
    if (buf[0] != FRAME_SOF) return false;
    if (buf[FRAME_LEN - 1] != FRAME_EOF) return false;

    /* XOR checksum over bytes 0..(FRAME_LEN-3) */
    uint8_t crc = 0;
    for (int i = 0; i < FRAME_LEN - 2; i++)
        crc ^= buf[i];

    if (crc != buf[FRAME_LEN - 2]) return false;

    /* Copy payload bytes into struct */
    memcpy(out, &buf[2], sizeof(SensorPayload_t));
    return true;
}

/**
 * @brief  Send a validated payload to the gateway via ESP-NOW.
 */
void sendViaEspNow(const SensorPayload_t *payload)
{
    espnow_sent = false;
    espnow_ok   = false;

    int result = esp_now_send(GATEWAY_MAC,
                              (uint8_t *)payload,
                              sizeof(SensorPayload_t));

    if (result != 0) {
        Serial.println("[ESP-NOW] Send enqueue FAILED");
        return;
    }

    /* Wait for the send callback (max 200 ms) */
    uint32_t t = millis();
    while (!espnow_sent && (millis() - t) < 200)
        yield();

    if (espnow_ok)
        Serial.println("[ESP-NOW] Delivered OK");
    else
        Serial.println("[ESP-NOW] Delivery FAILED (no ACK)");
}

/* ---- Arduino Entry Points ----------------------------------------------- */

void setup()
{
    /* Debug output on the same UART (TX only – useful with USB-TTL adapter) */
    Serial.begin(UART_BAUD);
    delay(100);
    Serial.println("\r\n=== ESP-01 Sensor Bridge ===");

    /* ESP-NOW requires Wi-Fi in station mode, but NOT connected to an AP */
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (esp_now_init() != 0) {
        Serial.println("[ERROR] ESP-NOW init failed! Halting.");
        while (1) delay(1000);
    }

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(onSendCallback);

    /* Register the gateway as a peer */
    esp_now_add_peer(GATEWAY_MAC,
                     ESP_NOW_ROLE_SLAVE,
                     1,        /* Wi-Fi channel 1 – must match gateway */
                     NULL,
                     0);

    Serial.print("[ESP-NOW] Gateway peer: ");
    for (int i = 0; i < 6; i++) {
        if (i) Serial.print(":");
        Serial.printf("%02X", GATEWAY_MAC[i]);
    }
    Serial.println();
    Serial.println("[ESP-01] Waiting for STM32 frames...");
}

void loop()
{
    /* ---- Byte-by-byte frame parser ---- */
    while (Serial.available()) {
        uint8_t b = Serial.read();

        if (!in_frame) {
            if (b == FRAME_SOF) {
                in_frame = true;
                rx_idx   = 0;
                rx_buf[rx_idx++] = b;
            }
            /* Ignore any byte before SOF */
        } else {
            if (rx_idx < FRAME_LEN) {
                rx_buf[rx_idx++] = b;
            }

            if (rx_idx == FRAME_LEN) {
                in_frame = false;

                SensorPayload_t payload;
                if (validateFrame(rx_buf, &payload)) {
                    Serial.printf("[STM32] T=%.1f H=%.1f CO2=%.1f NH3=%.1f\r\n",
                                  payload.temperature, payload.humidity,
                                  payload.co2, payload.nh3);
                    sendViaEspNow(&payload);
                } else {
                    Serial.println("[WARN] Bad frame (checksum error)");
                }

                rx_idx = 0;
            }
        }
    }
}
