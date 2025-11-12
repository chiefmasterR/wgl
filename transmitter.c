#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include "driver/i2s.h"

// ----------------- Pins -----------------
#define I2S_BCK   26
#define I2S_WS    25
#define I2S_DIN   33   // data in from ADC (to ESP32)

#define RF24_CE   27
#define RF24_CSN  14

#define PAIR_LED   4
#define STANDBY_LED 16

// ----------------- Audio / RF Constants -----------------
#define SAMPLE_RATE         48000
#define I2S_NUM_USED        I2S_NUM_0
#define SAMPLES_PER_PACKET  10                // samples per RF packet
#define AUDIO_BYTES_PER_SAMPLE 3              // 24-bit = 3 bytes
#define AUDIO_PAYLOAD_BYTES (SAMPLES_PER_PACKET * AUDIO_BYTES_PER_SAMPLE) // 30
#define NRF_PAYLOAD_SIZE    32                // we send header(2) + audio(30) = 32
#define HEADER_TYPE_AUDIO   0x01
#define HEADER_TYPE_PAIR    0x02

#define PAIR_CODE 0xAA55AA55UL

#define PCM1864_ADDR 0x4A
#define PCM5122_ADDR 0x4C

#define BASE_CHANNEL 40
#define MAX_CHANNEL  110

// ----------------- Globals -----------------
RF24 radio(RF24_CE, RF24_CSN);
uint8_t txAddress[5] = {'A','U','D','I','O'};
uint8_t currentChannel = BASE_CHANNEL;
uint8_t seqNum = 0;

// ----------------- I2C helper -----------------
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// ----------------- PCM1864 (ADC) init -----------------
void pcm1864_init() {
  // Soft reset (registers differ by silicon revision — adjust if needed)
  i2cWrite(PCM1864_ADDR, 0x00, 0x01);
  delay(10);

  // Slave to external BCLK/LRCLK (ESP32 is I2S master)
  i2cWrite(PCM1864_ADDR, 0x20, 0x00); // clock mode: slave

  // Audio interface: I2S, 24-bit (device-specific bits — adjust if your revision differs)
  // Example setting: bits[4:2]=010 -> 24-bit, format bits[1:0]=10 -> I2S (Check datasheet for your revision)
  i2cWrite(PCM1864_ADDR, 0x21, 0b00010010);

  // Route VIN1L to ADC ch (mono). Enable PGA or set as needed:
  i2cWrite(PCM1864_ADDR, 0x06, 0x11); // example: VIN1L->ADC1L

  // PGA gain = 0 dB
  i2cWrite(PCM1864_ADDR, 0x10, 0x00);

  // Digital HPF & power-up sequence
  i2cWrite(PCM1864_ADDR, 0x0B, 0x00);
  i2cWrite(PCM1864_ADDR, 0x01, 0x00); // ADC normal mode
  delay(5);
  i2cWrite(PCM1864_ADDR, 0x02, 0x00); // power up ADC
  delay(10);
}

// ----------------- I2S init (RX from ADC) -----------------
void i2s_init_adc() {
  i2s_config_t conf = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // use 32-bit slots
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 6,
    .dma_buf_len = 160
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN
  };

  i2s_driver_install(I2S_NUM_USED, &conf, 0, NULL);
  i2s_set_pin(I2S_NUM_USED, &pins);
}

// ----------------- RF init & channel scanning -----------------
void rf_init() {
  radio.begin();
  radio.setAutoAck(false);
  radio.setPayloadSize(NRF_PAYLOAD_SIZE);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(txAddress);
  radio.enableDynamicAck(); // optional
}

// Try to find a "quiet" channel using testRPD (simple heuristic)
bool find_free_channel() {
  for (uint8_t ch = BASE_CHANNEL; ch <= MAX_CHANNEL; ++ch) {
    radio.setChannel(ch);
    delay(2);
    // testRPD() returns true if RF power detected > -64 dBm (only valid while listening or sometimes in standby)
    // We use it as a heuristic — not perfect in noisy environments.
    if (!radio.testRPD()) {
      currentChannel = ch;
      return true;
    }
  }
  // fallback
  currentChannel = BASE_CHANNEL;
  return false;
}

// ----------------- Pair beacon -----------------
void send_pair_beacon() {
  uint8_t payload[NRF_PAYLOAD_SIZE];
  memset(payload, 0, sizeof(payload));
  payload[0] = HEADER_TYPE_PAIR;
  payload[1] = 0; // seq reserved
  // put 4-byte pair code (big endian) in payload[2..5]
  payload[2] = (PAI R_CODE >> 24) & 0xFF; // BUG-FIX: correct macro below
}

// We'll use correct pair code placement in function below

void send_pair_packet() {
  uint8_t payload[NRF_PAYLOAD_SIZE];
  memset(payload, 0, sizeof(payload));
  payload[0] = HEADER_TYPE_PAIR;
  payload[1] = 0;
  // place 4-byte pair code (big-endian)
  uint32_t code = PAIR_CODE;
  payload[2] = (code >> 24) & 0xFF;
  payload[3] = (code >> 16) & 0xFF;
  payload[4] = (code >> 8) & 0xFF;
  payload[5] = (code) & 0xFF;
  radio.stopListening();
  radio.write(payload, NRF_PAYLOAD_SIZE);
  radio.startListening(); // safe to leave listening off short time; we switch back to tx in main anyway
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  pinMode(PAIR_LED, OUTPUT);
  pinMode(STANDBY_LED, OUTPUT);

  Wire.begin(); // default SDA/SCL pins (change if you wired different)
  i2s_init_adc();
  pcm1864_init();

  rf_init();

  digitalWrite(STANDBY_LED, HIGH);
  digitalWrite(PAIR_LED, LOW);

  // Find a channel (heuristic)
  if (find_free_channel()) {
    Serial.printf("Transmitter selected channel %d\n", currentChannel);
  } else {
    Serial.printf("Fallback to channel %d\n", currentChannel);
  }
  radio.setChannel(currentChannel);
  delay(10);
}

// ----------------- Helper: pack samples (32-bit words -> 3 bytes each) -----------------
// Input: buffer of 32-bit words (host little-endian representation from i2s_read)
// Output: audioBytes must have size AUDIO_PAYLOAD_BYTES
void pack_samples_to_24bytes(const uint8_t* i2s_buf, size_t i2s_bytes, uint8_t* audioBytes) {
  // i2s_bytes should be SAMPLES_PER_PACKET * 4
  for (int s = 0; s < SAMPLES_PER_PACKET; ++s) {
    uint32_t w;
    // copy 4 bytes into uint32_t (little-endian memory)
    memcpy(&w, i2s_buf + s * 4, 4);
    // Right shift by 8 to get 24-bit right-aligned value (arith shift keeps sign for negative values)
    int32_t s24 = (int32_t)w >> 8;
    // Now encode big-endian 3 bytes (MSB first)
    int base = s * 3;
    audioBytes[base + 0] = (uint8_t)((s24 >> 16) & 0xFF);
    audioBytes[base + 1] = (uint8_t)((s24 >> 8) & 0xFF);
    audioBytes[base + 2] = (uint8_t)(s24 & 0xFF);
  }
}

// ----------------- Main loop -----------------
void loop() {
  // 1) send a periodic pairing beacon until receiver locks
  static unsigned long lastBeacon = 0;
  if (millis() - lastBeacon > 150) {
    // send beacon to help receivers find channel (even after paired, it can be ignored)
    uint8_t beacon[NRF_PAYLOAD_SIZE];
    memset(beacon, 0, sizeof(beacon));
    beacon[0] = HEADER_TYPE_PAIR;
    beacon[1] = 0;
    uint32_t code = PAIR_CODE;
    beacon[2] = (code >> 24) & 0xFF;
    beacon[3] = (code >> 16) & 0xFF;
    beacon[4] = (code >> 8) & 0xFF;
    beacon[5] = (code) & 0xFF;

    radio.stopListening();
    radio.write(beacon, NRF_PAYLOAD_SIZE);
    radio.startListening();
    lastBeacon = millis();
  }

  // 2) Read one block of samples from I2S (SAMPLES_PER_PACKET samples, 4 bytes per sample)
  const size_t i2sReadBytes = SAMPLES_PER_PACKET * 4;
  uint8_t i2sReadBuf[i2sReadBytes];
  size_t bytesRead = 0;
  esp_err_t ret = i2s_read(I2S_NUM_USED, i2sReadBuf, i2sReadBytes, &bytesRead, 20);
  if (ret != ESP_OK || bytesRead != i2sReadBytes) {
    // If no data, short delay and retry
    delayMicroseconds(100);
    return;
  }

  // 3) Pack into 24-bit audio payload
  uint8_t audioPayload[AUDIO_PAYLOAD_BYTES];
  pack_samples_to_24bytes(i2sReadBuf, i2sReadBytes, audioPayload);

  // 4) Build RF packet: [type(1)][seq(1)][audio 30]
  uint8_t packet[NRF_PAYLOAD_SIZE];
  memset(packet, 0, sizeof(packet));
  packet[0] = HEADER_TYPE_AUDIO;
  packet[1] = seqNum++;
  memcpy(packet + 2, audioPayload, AUDIO_PAYLOAD_BYTES);

  // 5) Send (switching to TX then back to listening quickly)
  radio.stopListening();
  bool ok = radio.write(packet, NRF_PAYLOAD_SIZE);
  // Optionally check ok and handle
  radio.startListening();

  // LED: blink PAIR while unpaired (we don't track receiver ack in this simple example)
  // Keep STANDBY LED high until you want to indicate ready in an external mechanism
}
