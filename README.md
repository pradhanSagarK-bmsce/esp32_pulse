
# 💓 MAX30105 Pulse Oximeter + 1.3" OLED Display (Enhanced UI)

###  Pulse & SpO₂ Monitor with Real-Time ECG-Like Display

**Platform:** Arduino / ESP32  
**Display:** 1.3" OLED (SH1106, 128×64)  
**Sensor:** MAX30105 / MAX30102  
**Library Dependencies:** U8g2, Wire, MAX30105

---

## 🩺 Overview

This project visualizes real-time heart rate (BPM) and blood oxygen saturation (SpO₂) readings using the **MAX30102 pulse oximeter sensor**, displayed on a 1.3" OLED screen (SH1106 controller).

It features an **enhanced graphical user interface**, displaying:

- A **scrolling ECG-style waveform**
    
- A **large pulsing heart icon**
    
- **Live BPM (Beats Per Minute)** and **SpO₂ (%)** readings
    
- **Animated calibration and analyzing screens**
    

 It’s designed for embedded telemetry, fitness tracking experiments, and health data visualization only .

---

## ⚠️ Disclaimer

> **Note:**  
> This project is **based on the MAX30102 pulse oximeter sensor** (hardware-compatible with ESP32 for red/IR LED measurements).  
> The readings for **heart rate (BPM)** and **blood oxygen saturation (SpO₂)** are **not medically accurate** — they provide **approximate or indicative values** suitable for:
> 
> - Learning and experimentation
>     
> - DIY health monitoring projects
>     
> - Signal visualization and embedded system prototyping
>     
> 
> For clinical or diagnostic purposes, always use certified medical-grade devices.

---

## 🖼️ Features

✅ **Right-to-Left Scrolling Waveform:**  
Smooth, ECG-like IR intensity waveform plotted in real time.

✅ **Animated Heartbeat Icon:**  
Heart pulses rhythmically with detected heartbeats.

✅ **Multi-State Workflow:**

- 🟡 **WAITING_FOR_FINGER:** Prompt user to place finger
    
- 🟢 **CALIBRATING:** Signal stabilization progress bar
    
- 🧠 **ANALYZING:** Sensor normalization phase
    
- ❤️ **MEASURING:** Live display of BPM and SpO₂
    
- 🔴 **ERROR_STATE:** Sensor disconnected or not detected
    

✅ **Dynamic Grid Overlay:**  
Soft dashed grid for professional look and better waveform tracking.

✅ **Robust Finger Detection:**  
Smart IR thresholding to detect finger presence or removal.

✅ **Noise-Resistant Signal Processing:**  
Includes smoothing, detrending, adaptive peak detection, and normalization.

---

## 🧩 Hardware Setup

|Component|Description|Connection|
|---|---|---|
|**MAX30105 / MAX30102**|Pulse oximeter & heart-rate sensor|I²C|
|**OLED (SH1106, 1.3")**|128×64 monochrome display|I²C|
|**Microcontroller**|ESP32 / Arduino / STM32|3.3V logic|

### 🔌 Pin Connections

| Device      | Pin | Connects To                    | Notes          |
| ----------- | --- | ------------------------------ | -------------- |
| MAX30102    | VIN | 3.3V                           | Power          |
|             | GND | GND                            | Common ground  |
|             | SCL | GPIO 22 (ESP32) / A5 (Arduino) | I²C Clock      |
|             | SDA | GPIO 21 (ESP32) / A4 (Arduino) | I²C Data       |
| OLED SH1106 | VCC | 3.3V                           | Power          |
|             | GND | GND                            | Common ground  |
|             | SCL | Same as MAX30105 SCL           | Shared I²C bus |
|             | SDA | Same as MAX30105 SDA           | Shared I²C bus |

---

## 🧰 Software Requirements

### 📦 PlatformIO (VS Code) Setup

Make sure you have the PlatformIO extension in VS Code and required libraries installed in your **PlatformIO project**.

####  Build & Upload:

- Open **VS Code → PlatformIO Toolbar**
    
- Click **Build (✓)** to compile
    
- Click **Upload (→)** to flash your ESP32
---

## ⚙️ Configuration Constants

|Constant|Purpose|Default|
|---|---|---|
|`SAMPLE_RATE`|Sampling rate (Hz)|100|
|`BUFFER_SIZE`|Buffer size for data|400|
|`DISPLAY_BUFFER`|OLED display circular buffer|128|
|`CALIBRATION_TIME`|Time to stabilize sensor|3000 ms|
|`FINGER_THRESHOLD`|IR intensity threshold for detection|50000|
|`NO_FINGER_TIMEOUT`|Timeout for auto return to idle|5000 ms|

---

## 🔄 System Workflow

### 1️⃣ Waiting for Finger

Displays "Place your finger" prompt and waits for IR > threshold.

### 2️⃣ Calibration Phase

Stabilizes signal for `CALIBRATION_TIME` (default 3s).  
Progress bar shows completion percentage.

### 3️⃣ Analyzing Phase

Smooths and normalizes data for ~1s before live measurement.

### 4️⃣ Measuring Phase

Displays live:

- BPM (Beats Per Minute)
    
- SpO₂ (%) oxygen saturation
    
- Animated heart and waveform
    

### 5️⃣ Finger Removed / Error

If finger removed → returns to **WAITING_FOR_FINGER**.  
If sensor fails to initialize → enters **ERROR_STATE**.

---

## 📈 Signal Processing Overview

### 🩺 Heart Rate (BPM) Calculation

- Detrends and normalizes IR buffer
    
- Adaptive threshold-based peak detection
    
- Filters unrealistic peaks (<45 or >120 BPM)
    
- Computes median RR interval → Converts to BPM
    
- Applies smoothing for stability
    

### 🩸 SpO₂ Calculation

- Computes AC/DC components of red and IR signals
    
- Calculates ratio ( R = (redAC/redDC) / (irAC/irDC) )
    
- Applies calibration formula:  
    [  
    SpO₂ = 110 - 25R  
    ]
    
- Smoothed using exponential averaging
    
- Clamped between 90–100%
    

---

## 🧠 UI and Animation Logic

### Heartbeat Animation

- Heart pulses using `heartbeatAnimPhase` every few frames
    
- Drawn via `drawHeartIcon()` with scalable pixel grid
    

### Calibration Progress Bar

- Renders a proportional filled bar with percentage text.
    

### Analyzing Animation

- Expanding and contracting circles visualize data stabilization.
    

### Waveform Rendering

- Right-to-left scrolling waveform
    
- High-pass filtered IR values for clarity
    
- Subtle grid overlay for visual structure
    

---

## 🖋️ Display Layout

```
+------------------------------------------------+
| BPM: 72          ❤️           SpO₂: 98%        |
|------------------------------------------------|
| ~ ~ ~ ECG-like waveform scrolling →            |
|     (dynamic, smooth, real-time plot)          |
+------------------------------------------------+
```

---

## 🧾 Serial Monitor Output

Sample Serial output at 115200 baud:

```
=== MAX30102 Oximeter (UI Improved) ===
Sensor Ready!
Finger detected -> CALIBRATING
Calibration done -> ANALYZING
Analyzing done -> MEASURING
Peaks: 7
BPM: 74.3 | SpO2: 98.1%
BPM: 74.1 | SpO2: 97.8%
```

---

## ⚠️ Troubleshooting

|Issue|Possible Cause|Solution|
|---|---|---|
|**“Sensor Error!”**|MAX30105 not detected|Check wiring, pull-up resistors, and I²C address|
|**No finger detected**|Low IR LED drive|Increase `setPulseAmplitudeIR()`|
|**Unstable readings**|Movement artifacts|Keep finger steady, improve contact|
|**Flat waveform**|Low signal|Adjust sensor LED current or reposition finger|

---

## 📚 Future Improvements

- Add temperature compensation
    
- Auto-gain control for variable lighting
    
- Store session data to SD card
    
- BLE transmission to mobile app
    
- Integrate HRV (Heart Rate Variability) visualization
    

---

## 🧑‍💻 Author & Credits

**Developed by:** [Pradhan Sagar K](https://github.com/pradhanSagarK-bmsce)
**Credits:** ChatGPT , Claude
**Libraries Used:**

- [U8g2 by olikraus](https://github.com/olikraus/u8g2)
    
- [SparkFun MAX3010x Library](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library)
    

---

## 🫀 Demo Preview (Concept)

_(Example layout on OLED)_

```
+--------------------------------+
|  72 ❤️        SpO₂ 98%         |
|--------------------------------|
|    __     _     __    _        |
| _ /  \_  / \_  /  \_ / \__     |
|--------------------------------|
```

---
