# 🩺 Portable IoT-Based Health Monitoring System (ESP32)

A multi-sensor portable health monitoring system built using ESP32 that measures Heart Rate (BPM), SpO₂, Temperature, Humidity, and GPS location in real time and uploads data to the Blynk IoT cloud. The system includes intelligent alert detection with automated email notifications.

---

## 📌 Overview

This project integrates biomedical and environmental sensors into a compact IoT-enabled device for continuous remote health monitoring. It supports elderly patients, athletes, and remote users by enabling real-time tracking and emergency alerting.

---

## 🛠 Hardware Components

- ESP32-WROOM Microcontroller
- MAX30102 (Pulse Oximeter & Heart Rate Sensor)
- DHT22 (Temperature & Humidity Sensor)
- NEO-6M GPS Module
- Li-ion Battery + 3.3V Regulator
- Wi-Fi Connectivity

---

## 📡 Sensor Interfaces

| Sensor | Communication Protocol |
|--------|------------------------|
| MAX30102 | I²C |
| DHT22 | Digital GPIO |
| NEO-6M | UART |
| Blynk | Wi-Fi (HTTP API) |

---

## ⚙️ Core Features

- Real-time BPM and SpO₂ calculation
- Temperature & humidity monitoring
- GPS location tracking
- Moving average filtering & peak detection
- Sliding window signal processing (3-second window)
- Intelligent threshold-based alert system
- Automatic email alerts with Google Maps link
- Cloud dashboard monitoring via Blynk

---

## 🚨 Alert Conditions

- SpO₂ < 92%
- HR < 50 or > 130 BPM
- Temperature > 38°C

On detection:
- Alert message generated
- GPS coordinates attached
- Email notification sent
- Event logged in Blynk

---

## 📊 Data Processing Highlights

- DC removal using moving averages
- AC component extraction
- Standard deviation-based peak detection
- SpO₂ ratio-of-ratios calculation
- Circular buffer implementation
- Event cooldown mechanism

---

## 📱 Cloud Dashboard

- Virtual Pins:
  - V0 → BPM
  - V1 → SpO₂
  - V2 → Temperature
  - V3 → Humidity
  - V9 → Event Logs
- Real-time graphs & monitoring
- Remote access via Blynk mobile app

---

## 🔋 Power & Portability

Battery-powered system designed for portable healthcare monitoring. Supports offline data collection and GPS fallback labeling when no fix is available.

---

## 👨‍💻 Developed By

Gaurav Singh Rana  
B.Tech – Electronics & Communication Engineering  
Lovely Professional University
