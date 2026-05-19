/**
 * @file    ESP32S3_Gateway.ino
 * @brief   ESP32-S3 Gateway – ESP-NOW Receiver + ThingSpeak upload
 * s
 * Stays in ESP-NOW listening mode (channel 1) most of the time.
 * Connects to Wi-Fi only when a new packet needs to be sent to ThingSpeak,
 * then disconnects and returns to ESP-NOW channel.
 *
 * Board: "ESP32S3 Dev Module"
 * Required: ESP32 Arduino core (HTTPClient built-in)
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HTTPClient.h>

/* ---- Wi-Fi credentials (replace with yours!) ---- */
const char *WIFI_SSID = "ahmed";
const char *WIFI_PASSWORD = "123456789";

/* ---- ThingSpeak settings ---- */
const char *THINGSPEAK_API_KEY = "R7CFHMP20DI4PXII"; // Write API Key
const long THINGSPEAK_CHANNEL = 3385172;
const unsigned long POST_INTERVAL = 20000; // 20 seconds minimum

/* ---- ESP-NOW configuration ---- */
static const uint8_t ALLOWED_SENDER_MAC[6] = {0x3C, 0x0F, 0x02, 0xD9, 0x03, 0x50};
#define FILTER_BY_MAC false
#define ESPNOW_CHANNEL 1 // Must match the sender's channel

/* ---- Shared Data Structure (identical to ESP-01 sender) ---- */
#pragma pack(push, 1)
typedef struct
{
    float temperature; // °C
    float humidity;    // %RH
    float co2;         // ppm
    float nh3;         // ppm
    float co;          // ppm
    float alcohol;     // ppm
    float toluene;     // ppm
    uint16_t adc_raw;  // raw ADC
} SensorPayload_t;
#pragma pack(pop)

/* ---- Statistics ---- */
static uint32_t pkt_received = 0;
static uint32_t pkt_bad_size = 0;
static uint32_t pkt_filtered = 0;

/* ---- Latest valid sensor data (protected by flag) ---- */
static SensorPayload_t latestPayload;
static bool newPayloadAvailable = false;

/* ---- Helpers ---- */
static bool macIsZero(const uint8_t *mac)
{
    for (int i = 0; i < 6; i++)
        if (mac[i] != 0)
            return false;
    return true;
}

static bool macMatch(const uint8_t *a, const uint8_t *b)
{
    for (int i = 0; i < 6; i++)
        if (a[i] != b[i])
            return false;
    return true;
}

static void printMac(const uint8_t *mac)
{
    for (int i = 0; i < 6; i++)
    {
        if (i)
            Serial.print(":");
        Serial.printf("%02X", mac[i]);
    }
}

/* ---- ESP-NOW receive callback ---- */
void onDataReceived(const esp_now_recv_info_t *info,
                    const uint8_t *data, int len)
{
    const uint8_t *sender_mac = info->src_addr;

    if (FILTER_BY_MAC && !macIsZero(ALLOWED_SENDER_MAC))
    {
        if (!macMatch(sender_mac, ALLOWED_SENDER_MAC))
        {
            pkt_filtered++;
            return;
        }
    }

    if (len != sizeof(SensorPayload_t))
    {
        pkt_bad_size++;
        Serial.printf("[WARN] Bad size: got %d, expected %d\r\n",
                      len, (int)sizeof(SensorPayload_t));
        return;
    }

    pkt_received++;
    SensorPayload_t payload;
    memcpy(&payload, data, sizeof(SensorPayload_t));
    int rssi = info->rx_ctrl->rssi;

    // Print JSON to Serial (for debugging)
    Serial.printf(
        "{\"src\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"rssi\":%d,"
        "\"t\":%.1f,\"h\":%.1f,"
        "\"co2\":%.1f,\"nh3\":%.1f,\"co\":%.1f,"
        "\"alc\":%.1f,\"tol\":%.1f,"
        "\"adc\":%u,\"pkt\":%lu}\r\n",
        sender_mac[0], sender_mac[1], sender_mac[2],
        sender_mac[3], sender_mac[4], sender_mac[5],
        rssi,
        payload.temperature, payload.humidity,
        payload.co2, payload.nh3, payload.co,
        payload.alcohol, payload.toluene,
        payload.adc_raw, pkt_received);

    // Store for later upload
    latestPayload = payload;
    newPayloadAvailable = true;
}

/* ---- Connect to Wi-Fi and upload one packet ---- */
void uploadToThingSpeak()
{
    if (!newPayloadAvailable)
        return;

    Serial.println("[WiFi] Connecting for upload...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int timeout = 10 * 20;
    while (WiFi.status() != WL_CONNECTED && timeout > 0)
    {
        delay(500);
        Serial.print(".");
        timeout--;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("\n[WiFi] Connection failed – will retry later.");
        WiFi.disconnect(true);
        return;
    }

    Serial.println("\n[WiFi] Connected. IP: " + WiFi.localIP().toString());

    // Build URL
    String url = "http://api.thingspeak.com/update?api_key=";
    url += THINGSPEAK_API_KEY;
    url += "&field1=" + String(latestPayload.temperature, 1);
    url += "&field2=" + String(latestPayload.humidity, 1);
    url += "&field3=" + String(latestPayload.co2, 1);
    url += "&field4=" + String(latestPayload.nh3, 1);
    url += "&field5=" + String(latestPayload.co, 1);
    url += "&field6=" + String(latestPayload.alcohol, 1);
    url += "&field7=" + String(latestPayload.toluene, 1);
    url += "&field8=" + String(latestPayload.adc_raw);

    HTTPClient http;
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode > 0)
    {
        Serial.printf("[ThingSpeak] HTTP %d – %s\r\n", httpCode, http.getString().c_str());
    }
    else
    {
        Serial.printf("[ThingSpeak] Error: %s\r\n", http.errorToString(httpCode).c_str());
    }
    http.end();

    // Disconnect Wi-Fi and FULLY restore ESP-NOW
    WiFi.disconnect(true);
    delay(500); // Give more time for radio to settle

    // Re-initialize ESP-NOW completely
    esp_now_deinit(); // Unregister previous callbacks and deinit

    WiFi.mode(WIFI_STA);
    delay(100);

    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(100);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ERROR] ESP-NOW re-init failed!");
        return;
    }

    esp_now_register_recv_cb(onDataReceived);

    Serial.printf("[Gateway] Back to ESP-NOW channel %d\r\n", ESPNOW_CHANNEL);

    newPayloadAvailable = false;
}

/* ---- Setup ---- */
void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\r\n=== ESP32-S3 Gateway (On‑demand Wi‑Fi) ===");

    // Start in STA mode without connecting to any AP
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Lock to ESP-NOW channel
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(100);

    Serial.print("[Gateway] Station MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ERROR] ESP-NOW init failed! Halting.");
        while (1)
            delay(1000);
    }
    esp_now_register_recv_cb(onDataReceived);

    Serial.printf("[Gateway] Listening on channel %d\r\n", ESPNOW_CHANNEL);
    if (FILTER_BY_MAC && !macIsZero(ALLOWED_SENDER_MAC))
    {
        Serial.print("[Gateway] Accepting only from: ");
        printMac(ALLOWED_SENDER_MAC);
        Serial.println();
    }
    else
    {
        Serial.println("[Gateway] Accepting ESP-NOW from ANY sender");
    }
    Serial.println("[Gateway] Ready – waiting for sensor data...\r\n");
}

/* ---- Loop ---- */
void loop()
{
    static unsigned long lastUpload = 0;

    // Check if it's time to upload and new data is available
    if (newPayloadAvailable && (millis() - lastUpload >= POST_INTERVAL))
    {
        lastUpload = millis();
        uploadToThingSpeak();
    }

    // Print statistics every 30 seconds (independent of upload)
    static uint32_t last_stats = 0;
    if (millis() - last_stats > 30000)
    {
        last_stats = millis();
        Serial.printf("[Stats] received=%lu  bad_size=%lu  filtered=%lu\r\n",
                      pkt_received, pkt_bad_size, pkt_filtered);
    }

    delay(10);
}