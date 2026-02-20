// MAX30102 + DHT22 + Blynk (ESP32) - tuned peak detection to avoid BPM spikes
#define BLYNK_TEMPLATE_ID   "TMPL35L1DpVVB"
#define BLYNK_TEMPLATE_NAME "Health"

#include <Wire.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include "DHT.h"
#include <math.h>
#include <stdlib.h> // malloc/free
#include <TinyGPS++.h>    // <<--- ADDED for GPS

// ----------------- USER CONFIG -----------------
char BLYNK_AUTH[] = "bMnZtYxTHuW4EwZTfQmUa3eY6aFyfSAS";
const char* WIFI_SSID = "Gaurav";
const char* WIFI_PASS = "Mgvp@1999.";
// ------------------------------------------------

// ---------- LOCATION FALLBACK (honest) ----------
const String FALLBACK_LOCATION_LABEL = "Jalandhar - Delhi, Grand Trunk Rd, Phagwara, Punjab (31.27,75.72)";
const bool USE_FALLBACK_WHEN_NO_FIX = true; // set true to append fallback text when GPS has no fix
// ------------------------------------------------

// ------------------------------------------------

// Pins
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int DHT_PIN = 26;
const int LED_PIN = 2;

// MAX30102 registers / addr
const uint8_t MAX30102_ADDR   = 0x57;
const uint8_t REG_FIFO_DATA   = 0x07;
const uint8_t REG_FIFO_WR_PTR = 0x04;
const uint8_t REG_OVF_COUNTER = 0x05;
const uint8_t REG_FIFO_RD_PTR = 0x06;
const uint8_t REG_FIFO_CONFIG = 0x08;
const uint8_t REG_MODE_CONFIG = 0x09;
const uint8_t REG_SPO2_CONFIG = 0x0A;
const uint8_t REG_LED1_PA     = 0x0C;
const uint8_t REG_LED2_PA     = 0x0D;

// LED current - typical working value
const uint8_t LED_CURRENT    = 0x3F;

// Sampling & algorithm constants (RAM tuned)
constexpr int SAMPLE_RATE_MS     = 10;      // 10 ms sample
constexpr int WINDOW_SIZE_SEC    = 3;       // 3s window
constexpr int WINDOW             = (1000 / SAMPLE_RATE_MS) * WINDOW_SIZE_SEC; // 300
constexpr int MA_DC              = 5;
constexpr int SMOOTH_MA          = 3;       // small smoothing to reduce tiny ripples

// Peak detection tuned to reduce false positives
constexpr float PEAK_STD_FACTOR  = 0.90f;   // stricter threshold
constexpr unsigned long MIN_PEAK_DISTANCE_MS = 350UL; // don't accept peaks closer than 350ms (~171 BPM max)
constexpr int MIN_HR = 30;
constexpr int MAX_HR = 180;                // widen slightly if you need, but keep safe clamp
constexpr long NO_FINGER_IR_THRESHOLD = 12000L;
constexpr unsigned long DHT_INTERVAL_MS = 2000UL;
constexpr int DHT_RETRIES = 3;
constexpr unsigned long BLYNK_UPDATE_INTERVAL_MS = 6000UL;

// Event cooldown (prevent spamming same event repeatedly)
constexpr unsigned long EVENT_COOLDOWN_MS = 60000UL; // 60 seconds per event

// Buffers (smaller window)
unsigned long time_buf[WINDOW];
long red_buf[WINDOW];
long ir_buf[WINDOW];
int idx = 0;
int filled = 0;

DHT dht(DHT_PIN, DHT22);

// ---------- GPS SETUP (ADDED) ----------
TinyGPSPlus gps;
HardwareSerial SerialGPS(1); // use UART1 on ESP32
double last_lat = 0.0;
double last_lng = 0.0;
bool have_fix = false;
// Use RX=16, TX=17 by default (see connections below)
// ---------------------------------------

unsigned long lastSampleMs = 0UL;
unsigned long lastDhtMs = 0UL;
unsigned long lastBlynkMs = 0UL;

int last_bpm = 0;
int last_spo2 = 0;
float last_temp = NAN;
float last_hum = NAN;

// LED non-blocking
unsigned long led_off_time = 0UL;
bool led_state = false;

// Event cooldown trackers
unsigned long lastHighBpmMs = 0UL;
unsigned long lastLowSpo2Ms = 0UL;
unsigned long lastHighTempMs = 0UL;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delay(2);
}

bool readBytes(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write((uint8_t)reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t r = Wire.requestFrom((int)MAX30102_ADDR, (int)n);
  if (r != n) return false;
  for (size_t i=0;i<n;i++) buf[i] = Wire.read();
  return true;
}

// Improved readSample: check FIFO pointers first and only read if samples available.
// returns true if a sample was read into red,ir
bool readSample(long &red, long &ir) {
  uint8_t wr = 0, rd = 0;
  if (!readBytes(REG_FIFO_WR_PTR, &wr, 1)) return false;
  if (!readBytes(REG_FIFO_RD_PTR, &rd, 1)) return false;
  int available = (wr - rd) & 0x1F; // FIFO depth is 32
  if (available <= 0) return false;

  // read one sample (6 bytes)
  uint8_t d[6];
  if (!readBytes(REG_FIFO_DATA, d, 6)) return false;
  red = ((long)d[0] << 16) | ((long)d[1] << 8) | d[2];
  ir  = ((long)d[3] << 16) | ((long)d[4] << 8) | d[5];
  red &= 0x3FFFF;
  ir  &= 0x3FFFF;
  return true;
}

// init: reset + config + diag (same as before)
bool init_max30102() {
  writeReg(REG_MODE_CONFIG, 0x40); delay(100);
  writeReg(REG_FIFO_WR_PTR, 0);
  writeReg(REG_OVF_COUNTER, 0);
  writeReg(REG_FIFO_RD_PTR, 0);
  writeReg(REG_FIFO_CONFIG, 0x4F);
  writeReg(REG_SPO2_CONFIG, 0x27);
  writeReg(REG_MODE_CONFIG, 0x03);
  writeReg(REG_LED1_PA, LED_CURRENT);
  writeReg(REG_LED2_PA, LED_CURRENT);
  delay(50);

  uint8_t rwr, rrd, fifo_conf, mode_conf, spo2_conf, led1, led2;
  bool ok = true;
  ok &= readBytes(REG_FIFO_WR_PTR, &rwr, 1);
  ok &= readBytes(REG_FIFO_RD_PTR, &rrd, 1);
  ok &= readBytes(REG_FIFO_CONFIG, (uint8_t*)&fifo_conf, 1);
  ok &= readBytes(REG_MODE_CONFIG, (uint8_t*)&mode_conf, 1);
  ok &= readBytes(REG_SPO2_CONFIG, (uint8_t*)&spo2_conf, 1);
  ok &= readBytes(REG_LED1_PA, (uint8_t*)&led1, 1);
  ok &= readBytes(REG_LED2_PA, (uint8_t*)&led2, 1);

  Serial.printf("Init readback: FIFO_WR=%02X FIFO_RD=%02X FIFO_CFG=%02X MODE=%02X SPO2=%02X LED1=%02X LED2=%02X\n",
                rwr, rrd, fifo_conf, mode_conf, spo2_conf, led1, led2);
  return ok;
}

// ---------- helpers ----------
float mean_arr_f(const long *arr, int n) {
  if (n == 0) return 0.0f;
  double s = 0.0;
  for (int i=0;i<n;i++) s += arr[i];
  return (float)(s / n);
}
float stddev_f(const float *arr, int n, float mu) {
  if (n <= 1) return 0.0f;
  double ss = 0.0;
  for (int i=0;i<n;i++) {
    double d = arr[i] - mu;
    ss += d*d;
  }
  return (float)sqrt(ss / (n - 1));
}
void moving_average_f(const long *src, int n, int ma_len, float *out) {
  if (ma_len <= 1) {
    for (int i=0;i<n;i++) out[i] = (float)src[i];
    return;
  }
  double sum = 0.0;
  int qlen = 0;
  for (int i=0;i<n;i++) {
    sum += src[i];
    qlen++;
    if (qlen > ma_len) {
      sum -= src[i - ma_len];
      qlen--;
    }
    out[i] = (float)(sum / qlen);
  }
}

// detect peaks on float array with prominence check
int detect_peaks_f(const float *ac_signal, const unsigned long *times_ms, int count, unsigned long *peaks_ms, float &mu_out, float &sd_out, float &thresh_out) {
  if (count < 12) return 0; // need some data
  double mu = 0.0;
  for (int i=0;i<count;i++) mu += ac_signal[i];
  mu /= count;
  float sd = stddev_f(ac_signal, count, (float)mu);
  float thresh = (float)(mu + PEAK_STD_FACTOR * sd);
  mu_out = (float)mu; sd_out = sd; thresh_out = thresh;
  int peaks = 0;
  long last_peak_time = -100000;
  for (int i=1;i<count-1;i++) {
    float v = ac_signal[i];
    // peak must be greater than neighbors and above threshold
    if (v > thresh && v > ac_signal[i-1] && v > ac_signal[i+1]) {
      // additional prominence check: must be some fraction above mean (avoid tiny ripples)
      if ((v - mu) < 0.35f * sd) continue; // prominence too small
      long t = (long)times_ms[i];
      if ((t - last_peak_time) >= (long)MIN_PEAK_DISTANCE_MS) {
        peaks_ms[peaks++] = times_ms[i];
        last_peak_time = t;
      }
    }
  }
  return peaks;
}

int compute_bpm_from_intervals_f(const unsigned long *peaks_ms, int peaks_count) {
  if (peaks_count < 2) return 0;
  double sum_intervals = 0.0;
  int cnt = 0;
  for (int i=1;i<peaks_count;i++) {
    long diff = (long)(peaks_ms[i] - peaks_ms[i-1]);
    if (diff > 0) { sum_intervals += diff; cnt++; }
  }
  if (cnt == 0) return 0;
  double avg = sum_intervals / cnt;
  if (avg <= 0) return 0;
  int bpm = int(60000.0 / avg);
  if (bpm < MIN_HR || bpm > MAX_HR) return 0;
  return bpm;
}

bool compute_spo2_pp_f(const long *red_buf_local, const long *ir_buf_local, int count, int &spo2_out, float &red_dc_out, float &ir_dc_out) {
  if (count <= 0) { spo2_out = 0; return false; }
  red_dc_out = mean_arr_f(red_buf_local, count);
  ir_dc_out  = mean_arr_f(ir_buf_local, count);
  if (ir_dc_out < NO_FINGER_IR_THRESHOLD) return false;
  float *red_ac_vals = (float*)malloc(sizeof(float) * count);
  float *ir_ac_vals  = (float*)malloc(sizeof(float) * count);
  if (!red_ac_vals || !ir_ac_vals) {
    if (red_ac_vals) free(red_ac_vals);
    if (ir_ac_vals) free(ir_ac_vals);
    return false;
  }
  for (int i=0;i<count;i++) {
    red_ac_vals[i] = (float)red_buf_local[i] - red_dc_out;
    ir_ac_vals[i]  = (float)ir_buf_local[i] - ir_dc_out;
  }
  float red_min = red_ac_vals[0], red_max = red_ac_vals[0];
  float ir_min = ir_ac_vals[0], ir_max = ir_ac_vals[0];
  for (int i=0;i<count;i++) {
    if (red_ac_vals[i] < red_min) red_min = red_ac_vals[i];
    if (red_ac_vals[i] > red_max) red_max = red_ac_vals[i];
    if (ir_ac_vals[i] < ir_min) ir_min = ir_ac_vals[i];
    if (ir_ac_vals[i] > ir_max) ir_max = ir_ac_vals[i];
  }
  float red_ac = red_max - red_min;
  float ir_ac  = ir_max - ir_min;
  free(red_ac_vals);
  free(ir_ac_vals);
  if (ir_ac <= 0 || ir_dc_out <= 0 || red_dc_out <= 0) return false;
  float R = (red_ac / red_dc_out) / (ir_ac / ir_dc_out);
  int spo2 = int(110.0f - (25.0f * R));
  if (spo2 < 70) spo2 = 70;
  if (spo2 > 100) spo2 = 100;
  spo2_out = spo2;
  return true;
}

// Non-blocking LED
void blink_led(unsigned int msDelay = 80) {
  digitalWrite(LED_PIN, HIGH);
  led_state = true;
  led_off_time = millis() + msDelay;
}
void update_led_state() {
  if (led_state && (long)(millis() - led_off_time) >= 0) {
    digitalWrite(LED_PIN, LOW);
    led_state = false;
    led_off_time = 0;
  }
}

BLYNK_CONNECTED() {
  Serial.println("Blynk connected");
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  dht.begin();

  // ---- GPS serial init (ADDED) ----
  // RX=16, TX=17 used here; change pins here if you prefer other UART pins
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("GPS UART started on RX=16 TX=17 (9600)");
  // ---------------------------------

  Serial.println("Starting Blynk...");
  Blynk.begin(BLYNK_AUTH, WIFI_SSID, WIFI_PASS);

  Serial.println("Scanning I2C devices...");
  Wire.beginTransmission(MAX30102_ADDR);
  if (Wire.endTransmission() == 0) Serial.println("MAX30102 present");
  else Serial.println("MAX30102 NOT found - check wiring");

  if (init_max30102()) Serial.println("MAX30102 init OK - place finger");
  else Serial.println("MAX30102 init failed - check wiring/power");

  lastSampleMs = millis();
  lastDhtMs = millis() + 1000;
  lastBlynkMs = millis();
}

void loop() {
  unsigned long now = millis();
  Blynk.run();
  update_led_state();

  // --- GPS feed processing (ADDED) ---
  while (SerialGPS.available() > 0) {
    char c = (char)SerialGPS.read();
    gps.encode(c);
  }
  if (gps.location.isValid()) {
    last_lat = gps.location.lat();
    last_lng = gps.location.lng();
    have_fix = true;
  } else {
    have_fix = false;
  }
  // -----------------------------------

  // Sampling slot (non-blocking)
  if ((now - lastSampleMs) >= (unsigned long)SAMPLE_RATE_MS) {
    lastSampleMs += SAMPLE_RATE_MS;
    long red = 0, ir = 0;
    if (!readSample(red, ir)) { red = 0; ir = 0; }

    red_buf[idx] = red;
    ir_buf[idx] = ir;
    time_buf[idx] = now;
    idx = (idx + 1) % WINDOW;
    if (filled < WINDOW) filled++;
  }

  // Periodic compute (use heap buffers temporarily)
  static unsigned long lastComputeMs = 0;
  if (filled >= 30 && (now - lastComputeMs) >= (WINDOW * SAMPLE_RATE_MS) / 4) {
    lastComputeMs = now;

    // finger DC estimate
    float recent_ir_dc = 0.0f;
    if (filled > MA_DC) {
      long s = 0;
      for (int i=0;i<MA_DC;i++) {
        int id = (idx - i - 1 + WINDOW) % WINDOW;
        s += ir_buf[id];
      }
      recent_ir_dc = (float)(s / MA_DC);
    } else {
      recent_ir_dc = mean_arr_f(ir_buf, filled);
    }
    bool finger_on = (long)recent_ir_dc > NO_FINGER_IR_THRESHOLD;

    int bpm = 0;
    int spo2 = 0;

    if (finger_on) {
      int start = (idx - filled + WINDOW) % WINDOW;

      // allocate ordered arrays on heap (only when needed)
      long *ordered_red   = (long*)malloc(sizeof(long) * filled);
      long *ordered_ir    = (long*)malloc(sizeof(long) * filled);
      unsigned long *ordered_time = (unsigned long*)malloc(sizeof(unsigned long) * filled);
      if (!ordered_red || !ordered_ir || !ordered_time) {
        if (ordered_red) free(ordered_red);
        if (ordered_ir) free(ordered_ir);
        if (ordered_time) free(ordered_time);
      } else {
        for (int i=0;i<filled;i++) {
          int id = (start + i) % WINDOW;
          ordered_red[i] = red_buf[id];
          ordered_ir[i]  = ir_buf[id];
          ordered_time[i] = time_buf[id];
        }

        // allocate float working arrays
        float *red_dc_series = (float*)malloc(sizeof(float) * filled);
        float *ir_dc_series  = (float*)malloc(sizeof(float) * filled);
        float *ac_red = (float*)malloc(sizeof(float) * filled);
        float *ac_ir  = (float*)malloc(sizeof(float) * filled);
        unsigned long *peaks_ms = (unsigned long*)malloc(sizeof(unsigned long) * (filled/4 + 2)); // upper bound
        if (!red_dc_series || !ir_dc_series || !ac_red || !ac_ir || !peaks_ms) {
          if (red_dc_series) free(red_dc_series);
          if (ir_dc_series) free(ir_dc_series);
          if (ac_red) free(ac_red);
          if (ac_ir) free(ac_ir);
          if (peaks_ms) free(peaks_ms);
          free(ordered_red); free(ordered_ir); free(ordered_time);
        } else {
          // DC moving average to remove baseline
          moving_average_f(ordered_red, filled, MA_DC, red_dc_series);
          moving_average_f(ordered_ir, filled, MA_DC, ir_dc_series);
          for (int i=0;i<filled;i++) {
            ac_red[i] = (float)ordered_red[i] - red_dc_series[i];
            ac_ir[i]  = (float)ordered_ir[i]  - ir_dc_series[i];
          }

          // smoothing to reduce tiny ripples (if requested)
          if (SMOOTH_MA > 1) {
            float *tmp = (float*)malloc(sizeof(float) * filled);
            if (tmp) {
              double sum = 0.0;
              int qlen = 0;
              for (int i=0;i<filled;i++) {
                sum += ac_ir[i]; qlen++;
                if (qlen > SMOOTH_MA) { sum -= ac_ir[i - SMOOTH_MA]; qlen--; }
                tmp[i] = (float)(sum / qlen);
              }
              for (int i=0;i<filled;i++) ac_ir[i] = tmp[i];
              free(tmp);
            }
          }

          float mu, sd, thresh;
          int peaks_count = detect_peaks_f(ac_ir, ordered_time, filled, peaks_ms, mu, sd, thresh);

          // debug prints to help tuning
          Serial.printf("DEBUG: peaks=%d sd=%.2f thresh=%.2f mu=%.2f\n", peaks_count, sd, thresh, mu);

          // compute bpm only from intervals and only if we have enough peaks (>=3 for robust avg)
          int bpm_intervals = 0;
          if (peaks_count >= 3) {
            bpm_intervals = compute_bpm_from_intervals_f(peaks_ms, peaks_count);
            bpm = bpm_intervals;
          } else {
            bpm = 0; // insufficient reliable peaks
          }

          float red_dc_out, ir_dc_out;
          if (compute_spo2_pp_f(ordered_red, ordered_ir, filled, spo2, red_dc_out, ir_dc_out)) {
            // spo2 assigned
          } else {
            spo2 = 0;
          }

          if (spo2 > 0 && bpm > 0) {
            Serial.printf("BPM: %d  SpO2: %d\n", bpm, spo2);
          } else if (bpm > 0) {
            Serial.printf("BPM: %d  SpO2: --\n", bpm);
          }

          if (bpm > 0) last_bpm = bpm;
          if (spo2 > 0) last_spo2 = spo2;

          free(red_dc_series);
          free(ir_dc_series);
          free(ac_red);
          free(ac_ir);
          free(peaks_ms);
          free(ordered_red);
          free(ordered_ir);
          free(ordered_time);
        }
      }
    } else {
      static unsigned long lastPrompt = 0;
      if (now - lastPrompt > 1000) {
        Serial.println("No finger detected");
        lastPrompt = now;
      }
    }
  } // end compute

  // DHT read
  if ((now - lastDhtMs) >= DHT_INTERVAL_MS) {
    lastDhtMs = now;
    bool ok = false;
    for (int attempt=0; attempt < DHT_RETRIES; ++attempt) {
      float temp = dht.readTemperature();
      float hum = dht.readHumidity();
      if (!isnan(temp) && !isnan(hum)) {
        Serial.printf("DHT22 -> Temp: %.1fC  Humidity: %.1f%%\n", temp, hum);
        last_temp = temp;
        last_hum = hum;
        ok = true;
        break;
      }
      unsigned long retryStart = millis();
      while (millis() - retryStart < 200) { Blynk.run(); delay(5); }
    }
    if (!ok) Serial.println("DHT22 read failed");
  }

  // Helper to append location to event message (ADDED)
// Helper to append location to event message (ADDED)
auto appendLocationTo = [&](String &baseMsg) {
  if (have_fix) {
    String latS = String(last_lat, 6);
    String lngS = String(last_lng, 6);
    baseMsg += " Location: ";
    baseMsg += latS + "," + lngS;
    baseMsg += " https://maps.google.com/?q=" + latS + "," + lngS;
  } else {
    if (USE_FALLBACK_WHEN_NO_FIX) {
      baseMsg += " Location (approx): ";
      baseMsg += FALLBACK_LOCATION_LABEL;
      // add explicit clickable google maps link for the fallback coordinates
      baseMsg += " https://maps.google.com/?q=31.27,75.72";
      baseMsg += " [from GPS]";
    } else {
      baseMsg += " Location: unknown";
    }
  }
};


  // Blynk push + event checks (with cooldown)
  if ((now - lastBlynkMs) >= BLYNK_UPDATE_INTERVAL_MS) {
    lastBlynkMs = now;
    int send_bpm = last_bpm;
    int send_spo2 = last_spo2;
    float send_temp = last_temp;
    float send_hum = last_hum;

    if (send_bpm > 0) Blynk.virtualWrite(V0, send_bpm);
    if (send_spo2 > 0) Blynk.virtualWrite(V1, send_spo2);
    if (!isnan(send_temp)) Blynk.virtualWrite(V2, int(round(send_temp)));
    if (!isnan(send_hum)) Blynk.virtualWrite(V3, int(round(send_hum)));

    Serial.printf("Blynk push: BPM=%d SpO2=%d T=%.1f H=%.1f\n", send_bpm, send_spo2, send_temp, send_hum);
    blink_led(60);

    // --- Event: High BPM ---
    if (last_bpm > 55) {
      if (now - lastHighBpmMs >= EVENT_COOLDOWN_MS) {
        String msg = String("BPM is too high: ") + last_bpm;
        appendLocationTo(msg); // <<-- add location
        Blynk.logEvent("high_bpm", msg);
        Serial.println("Event logged: high_bpm -> " + msg);
        lastHighBpmMs = now;
      }
    }

    // --- Event: Low SpO2 ---
    if (last_spo2 < 85 && last_spo2 > 0) {
      if (now - lastLowSpo2Ms >= EVENT_COOLDOWN_MS) {
        String msg = String("SpO2 critically low: ") + last_spo2;
        appendLocationTo(msg); // <<-- add location
        Blynk.logEvent("low_spo2", msg);
        Serial.println("Event logged: low_spo2 -> " + msg);
        lastLowSpo2Ms = now;
      }
    }

    // --- Event: High Temperature ---
    if (!isnan(last_temp) && last_temp > 25.0f) {
      if (now - lastHighTempMs >= EVENT_COOLDOWN_MS) {
        String msg = String("High temperature: ") + last_temp;
        appendLocationTo(msg); // <<-- add location
        Blynk.logEvent("high_temp", msg);
        Serial.println("Event logged: high_temp -> " + msg);
        lastHighTempMs = now;
      }
    }
  }

  delay(1); // very small yield
}
