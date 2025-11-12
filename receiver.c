#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include "driver/i2s.h"

// ----------------- Pins -----------------
#define I2S_BCK   26
#define I2S_WS    25
#define I2S_DOUT  32   // data out to DAC

#define RF24_CE   27
#define RF24_CSN  14

#define PAIR_LED   4
#define STANDBY_LED 16

// ----------------- Audio / RF Constants -----------------
#define SAMPLE_RATE         48000
#define I2S_NUM_USED        I2S_NUM_0
#define SAMPLES_PER_PACKET  10
#define AUDIO_BYTES_PER_SAMPLE 3
#define AUDIO_PAYLOAD_BYTES (SAMPLES_PER_PACKET * AUDIO_BYTES_PER_SAMPLE)
#define NRF_PAYLOAD_SIZE    32
#define HEADER_TYPE_AUDIO   0x01
#define HEADER_TYPE_PAIR    0x02

#define PAIR_CODE 0xAA55AA55UL

#define PCM1864_ADDR 0x4A
#define PCM5122_ADDR 0x4C

#define BASE_CHANNEL 40
#define MAX_CHANNEL  110

// ----------------- Globals -----------------
RF24 radio(RF24_CE, RF24_CSN);
uint8_t rxAddress[5] = {'A','U','D','I','O'};
uint8_t lockedChannel = 0;
bool linked = false;

// ----------------- I2C helper -----------------
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// ----------------- PCM5122 (DAC) init -----------------
void pcm5122_init() {
  // Soft reset / power-up sequence (addresses/value fields depend on chip revision)
  i2cWrite(PCM5122_ADDR, 0x01, 0x00);
  delay(10);

  // Clock mode: slave (ESP32 provides BCLK / LRCLK)
  i2cWrite(PCM5122_ADDR, 0x05, 0x00);

  // Serial audio interface: I2S, 24-bit
  i2cWrite(PCM5122_ADDR, 0x06, 0b00010010);

  // Route left input to both outputs for mono playback (if desired)
  i2cWrite(PCM5122_ADDR, 0x0E, 0x11);

  // Unmute / volume
  i2cWrite(PCM5122_ADDR, 0x0D, 0x00);
  i2cWrite(PCM5122_ADDR, 0x3B, 0x00);
  i2cWrite(PCM5122_ADDR, 0x3C, 0x00);

  // Power up DAC
  i2cWrite(PCM5122_ADDR, 0x02, 0x00);
  delay(10);
}

// ----------------- I2S init (TX to DAC) -----------------
void i2s_init_dac() {
  i2s_config_t conf = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 6,
    .dma_buf_len = 160
  };

  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_USED, &conf, 0, NULL);
  i2s_set_pin(I2S_NUM_USED, &pins);
}

// ----------------- RF init -----------------
void rf_init() {
  radio.begin();
  radio.setAutoAck(false);
  radio.setPayloadSize(NRF_PAYLOAD_SIZE);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, rxAddress);
  radio.startListening();
}

// ----------------- Scan channels for pair beacon -----------------
bool scan_and_lock_channel() {
  uint8_t buf[NRF_PAYLOAD_SIZE];
  for (uint8_t ch = BASE_CHANNEL; ch <= MAX_CHANNEL; ++ch) {
    radio.setChannel(ch);
    radio.startListening();
    unsigned long t0 = millis();
    while (millis() - t0 < 120) { // listen 120 ms per channel
      if (radio.available()) {
        radio.read(buf, NRF_PAYLOAD_SIZE);
        if (buf[0] == HEADER_TYPE_PAIR) {
          // verify pair code
          uint32_t code = (uint32_t(buf[2]) << 24) | (uint32_t(buf[3]) << 16) | (uint32_t(buf[4]) << 8) | uint32_t(buf[5]);
          if (code == PAIR_CODE) {
            lockedChannel = ch;
            radio.stopListening();
            radio.setChannel(lockedChannel);
            radio.startListening();
            return true;
          }
        }
      }
    }
    radio.stopListening();
  }
  return false;
}

// ----------------- Unpack 24-bit audio bytes into 32-bit word buffer -----------------
// audioBytes contains AUDIO_PAYLOAD_BYTES (30 bytes): 10 samples * 3 bytes (MSB first)
void unpack_24bytes_to_i2s_words(const uint8_t* audioBytes, uint8_t* i2sOutBuf) {
  // i2sOutBuf must be SAMPLES_PER_PACKET * 4 bytes (40 bytes)
  for (int s = 0; s < SAMPLES_PER_PACKET; ++s) {
    int baseA = s * 3;
    // reconstruct signed 24-bit big-endian into int32
    int32_t s24 = (int32_t)((audioBytes[baseA] << 16) | (audioBytes[baseA+1] << 8) | (audioBytes[baseA+2]));
    // sign-extend if top bit set
    if (s24 & 0x00800000) s24 |= 0xFF000000;
    // left-align into 32-bit slot (same style as transmitter produced)
    int32_t s32 = s24 << 8;
    // write to i2sOutBuf as little-endian 32-bit word
    uint32_t w = (uint32_t)s32;
    memcpy(i2sOutBuf + s * 4, &w, 4);
  }
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  pinMode(PAIR_LED, OUTPUT);
  pinMode(STANDBY_LED, OUTPUT);
  digitalWrite(PAIR_LED, LOW);
  digitalWrite(STANDBY_LED, HIGH);

  Wire.begin();
  i2s_init_dac();
  pcm5122_init();

  rf_init();

  // scan channels to find transmitter
  if (scan_and_lock_channel()) {
    Serial.printf("Locked channel %d\n", lockedChannel);
    digitalWrite(PAIR_LED, HIGH);
    digitalWrite(STANDBY_LED, LOW);
    linked = true;
    // ensure radio configured
    radio.setChannel(lockedChannel);
    radio.stopListening();
    radio.startListening();
  } else {
    Serial.println("No pair found at startup. Will keep scanning in loop.");
    linked = false;
  }
}

// ----------------- Main loop -----------------
void loop() {
  uint8_t recv[NRF_PAYLOAD_SIZE];
  if (!linked) {
    // try to find pair
    if (scan_and_lock_channel()) {
      Serial.printf("Locked channel %d\n", lockedChannel);
      digitalWrite(PAIR_LED, HIGH);
      digitalWrite(STANDBY_LED, LOW);
      linked = true;
      radio.setChannel(lockedChannel);
      radio.stopListening();
      radio.startListening();
    } else {
      delay(250);
      return;
    }
  }

  // When linked, read incoming audio packets
  if (radio.available()) {
    radio.read(recv, NRF_PAYLOAD_SIZE);
    if (recv[0] == HEADER_TYPE_AUDIO) {
      // packet contains seq in recv[1] (not used here)
      // audio payload starts at recv[2], length AUDIO_PAYLOAD_BYTES
      uint8_t i2sOutBuf[SAMPLES_PER_PACKET * 4];
      unpack_24bytes_to_i2s_words(recv + 2, i2sOutBuf);
      size_t bytes_written = 0;
      esp_err_t r = i2s_write(I2S_NUM_USED, i2sOutBuf, SAMPLES_PER_PACKET * 4, &bytes_written, 20);
      if (r != ESP_OK) {
        // handle write error (skip for now)
      }
    } else if (recv[0] == HEADER_TYPE_PAIR) {
      // Another pairing beacon — could be used to re-sync
    }
  } else {
    // No packet available — small sleep to avoid burning CPU
    delayMicroseconds(50);
  }
}
