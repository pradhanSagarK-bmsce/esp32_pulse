// #include <Arduino.h>
// #include <Wire.h>
// #include <U8g2lib.h>
// #include "MAX30105.h"
// #include <math.h>

// // ===== OLED =====
// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// // ===== MAX30102 =====
// MAX30105 particleSensor;

// // ===== Constants =====
// #define SAMPLE_RATE 100
// #define BUFFER_SIZE 400
// #define CALIBRATION_SAMPLES 300
// #define FINGER_THRESHOLD 50000
// #define WAVEFORM_WIDTH 128
// #define WAVEFORM_HEIGHT 30

// // ===== State Machine =====
// enum State {
//     INIT,
//     WAITING_FOR_FINGER,
//     CALIBRATING,
//     MEASURING
// };

// State currentState = INIT;

// // ===== Buffers =====
// uint32_t irBuffer[BUFFER_SIZE];
// uint32_t redBuffer[BUFFER_SIZE];
// int bufferHead = 0;
// int bufferCount = 0;

// // ===== Metrics =====
// float BPM = 0;
// float SpO2 = 0;
// float irBaseline = 0;
// float irMin = 0;
// float irMax = 0;

// // ===== Waveform =====
// uint8_t waveform[WAVEFORM_WIDTH];
// int waveIndex = 0;

// // ===== Timing =====
// unsigned long lastSampleTime = 0;
// unsigned long stateStartTime = 0;
// unsigned long lastDisplayUpdate = 0;
// unsigned long lastMetricCalc = 0;

// // ===== Calibration =====
// int calibrationSamples = 0;

// // ===== Function prototypes =====
// void changeState(State newState);
// void updateDisplay();
// bool readSensor();
// bool isFingerDetected();
// void processMeasurement();
// float calculateBPM();
// float calculateSpO2();

// // ===== Setup =====
// void setup() {
//     Serial.begin(115200);
//     delay(100);
//     Serial.println("\n=== MAX30102 Pulse Oximeter ===");

//     // Init I2C
//     Wire.begin();
//     Wire.setClock(400000);

//     // Init OLED
//     u8g2.begin();
//     u8g2.clearBuffer();
//     u8g2.setFont(u8g2_font_ncenB08_tr);
//     u8g2.drawStr(20, 30, "MAX30102");
//     u8g2.drawStr(15, 45, "Starting...");
//     u8g2.sendBuffer();
//     delay(1000);

//     // Init MAX30102
//     Serial.println("Initializing MAX30102...");
//     if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
//         Serial.println("ERROR: MAX30102 not found!");
//         u8g2.clearBuffer();
//         u8g2.setFont(u8g2_font_ncenB08_tr);
//         u8g2.drawStr(10, 30, "Sensor Error!");
//         u8g2.drawStr(5, 45, "Check Connection");
//         u8g2.sendBuffer();
//         while (1) delay(1000);
//     }

//     Serial.println("MAX30102 found!");

//     // Configure sensor
//     byte ledBrightness = 60;
//     byte sampleAverage = 1;
//     byte ledMode = 2;
//     byte sampleRate = 100;
//     byte pulseWidth = 411;
//     byte adcRange = 4096;

//     particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    
//     // Set LED currents - lower to reduce motion sensitivity
//     particleSensor.setPulseAmplitudeRed(0x0F);
//     particleSensor.setPulseAmplitudeIR(0x0F);
    
//     // Clear FIFO
//     particleSensor.clearFIFO();
    
//     delay(100);

//     Serial.println("Sensor configured!");
//     changeState(WAITING_FOR_FINGER);
// }

// // ===== Main Loop =====
// void loop() {
//     unsigned long currentTime = millis();
    
//     // Read sensor at fixed rate
//     if (currentTime - lastSampleTime >= (1000 / SAMPLE_RATE)) {
//         lastSampleTime = currentTime;
//         readSensor();
//     }

//     // State machine
//     switch (currentState) {
//         case WAITING_FOR_FINGER:
//             if (isFingerDetected()) {
//                 Serial.println("Finger detected!");
//                 changeState(CALIBRATING);
//             }
//             break;

//         case CALIBRATING:
//             if (!isFingerDetected()) {
//                 Serial.println("Finger removed during calibration");
//                 changeState(WAITING_FOR_FINGER);
//             } else {
//                 calibrationSamples++;
                
//                 if (calibrationSamples >= CALIBRATION_SAMPLES) {
//                     // Calculate baseline stats
//                     float sum = 0;
//                     irMin = 999999;
//                     irMax = 0;
                    
//                     for (int i = 0; i < bufferCount; i++) {
//                         sum += irBuffer[i];
//                         if (irBuffer[i] < irMin) irMin = irBuffer[i];
//                         if (irBuffer[i] > irMax) irMax = irBuffer[i];
//                     }
//                     irBaseline = sum / bufferCount;
                    
//                     float range = irMax - irMin;
                    
//                     Serial.print("Calibration complete - Baseline: ");
//                     Serial.print(irBaseline);
//                     Serial.print(", Range: ");
//                     Serial.print(range);
//                     Serial.print(", Samples: ");
//                     Serial.println(bufferCount);
                    
//                     if (range > 100) {  // Check for valid pulse signal
//                         changeState(MEASURING);
//                     } else {
//                         Serial.println("No pulse detected, recalibrating...");
//                         calibrationSamples = 0;
//                     }
//                 }
//             }
//             break;

//         case MEASURING:
//             if (!isFingerDetected()) {
//                 Serial.println("Finger removed");
//                 changeState(WAITING_FOR_FINGER);
//             } else {
//                 processMeasurement();
                
//                 // Calculate metrics every 2 seconds
//                 if (currentTime - lastMetricCalc >= 2000 && bufferCount >= BUFFER_SIZE) {
//                     float newBPM = calculateBPM();
//                     float newSpO2 = calculateSpO2();
                    
//                     // Only update if values are reasonable
//                     if (newBPM >= 40 && newBPM <= 150) {
//                         BPM = newBPM;
//                     }
//                     if (newSpO2 >= 85 && newSpO2 <= 100) {
//                         SpO2 = newSpO2;
//                     }
                    
//                     lastMetricCalc = currentTime;
                    
//                     Serial.print("BPM: ");
//                     Serial.print(BPM, 1);
//                     Serial.print(" | SpO2: ");
//                     Serial.print(SpO2, 1);
//                     Serial.println("%");
//                 }
//             }
//             break;
//     }

//     // Update display
//     if (currentTime - lastDisplayUpdate >= 50) {
//         lastDisplayUpdate = currentTime;
//         updateDisplay();
//     }
// }

// // ===== Change State =====
// void changeState(State newState) {
//     currentState = newState;
//     stateStartTime = millis();
    
//     switch (newState) {
//         case WAITING_FOR_FINGER:
//             BPM = 0;
//             SpO2 = 0;
//             bufferHead = 0;
//             bufferCount = 0;
//             memset(waveform, 0, sizeof(waveform));
//             break;
            
//         case CALIBRATING:
//             calibrationSamples = 0;
//             bufferHead = 0;
//             bufferCount = 0;
//             irMin = 999999;
//             irMax = 0;
//             break;
            
//         case MEASURING:
//             lastMetricCalc = millis();
//             break;
//     }
// }

// // ===== Read Sensor =====
// bool readSensor() {
//     uint32_t ir = particleSensor.getIR();
//     uint32_t red = particleSensor.getRed();
    
//     // Only store if finger is detected
//     if (currentState == CALIBRATING || currentState == MEASURING) {
//         irBuffer[bufferHead] = ir;
//         redBuffer[bufferHead] = red;
        
//         bufferHead = (bufferHead + 1) % BUFFER_SIZE;
//         if (bufferCount < BUFFER_SIZE) {
//             bufferCount++;
//         }
        
//         // Update min/max for waveform
//         if (currentState == MEASURING && bufferCount > 0) {
//             float minVal = 999999;
//             float maxVal = 0;
//             for (int i = 0; i < min(bufferCount, BUFFER_SIZE); i++) {
//                 if (irBuffer[i] < minVal) minVal = irBuffer[i];
//                 if (irBuffer[i] > maxVal) maxVal = irBuffer[i];
//             }
//             irMin = minVal;
//             irMax = maxVal;
//         }
//     }
    
//     return true;
// }

// // ===== Finger Detection =====
// bool isFingerDetected() {
//     uint32_t ir = particleSensor.getIR();
//     return (ir > FINGER_THRESHOLD && ir < 200000);
// }

// // ===== Process Measurement =====
// void processMeasurement() {
//     if (bufferCount == 0) return;
    
//     // Update waveform
//     int latestIndex = (bufferHead - 1 + BUFFER_SIZE) % BUFFER_SIZE;
//     uint32_t irValue = irBuffer[latestIndex];
    
//     if (irMax > irMin) {
//         uint8_t waveValue = map(irValue, irMin, irMax, 0, WAVEFORM_HEIGHT);
//         waveValue = constrain(waveValue, 0, WAVEFORM_HEIGHT);
//         waveform[waveIndex] = waveValue;
//         waveIndex = (waveIndex + 1) % WAVEFORM_WIDTH;
//     }
// }

// // ===== Calculate BPM =====
// float calculateBPM() {
//     if (bufferCount < BUFFER_SIZE) return 0;
    
//     // Step 1: Apply bandpass filter (detrend + smooth)
//     float filtered[BUFFER_SIZE];
    
//     // Calculate mean (DC component)
//     float mean = 0;
//     for (int i = 0; i < BUFFER_SIZE; i++) {
//         mean += irBuffer[i];
//     }
//     mean /= BUFFER_SIZE;
    
//     // Remove DC and apply moving average filter
//     const int filterWindow = 5;
//     for (int i = 0; i < BUFFER_SIZE; i++) {
//         float sum = 0;
//         int count = 0;
//         for (int j = -filterWindow; j <= filterWindow; j++) {
//             int idx = i + j;
//             if (idx >= 0 && idx < BUFFER_SIZE) {
//                 sum += (irBuffer[idx] - mean);
//                 count++;
//             }
//         }
//         filtered[i] = sum / count;
//     }
    
//     // Step 2: Find peaks with adaptive threshold
//     const int maxPeaks = 15;
//     int peaks[maxPeaks];
//     int peakCount = 0;
    
//     // Find range of filtered signal
//     float minVal = 999999;
//     float maxVal = -999999;
//     for (int i = 0; i < BUFFER_SIZE; i++) {
//         if (filtered[i] < minVal) minVal = filtered[i];
//         if (filtered[i] > maxVal) maxVal = filtered[i];
//     }
    
//     // Adaptive threshold at 60% of range
//     float threshold = minVal + (maxVal - minVal) * 0.6;
    
//     // Minimum distance between peaks (for 30-180 BPM range)
//     int minDistance = SAMPLE_RATE * 60 / 180; // 33 samples for 180 BPM
//     int maxDistance = SAMPLE_RATE * 60 / 30;  // 200 samples for 30 BPM
    
//     // Find peaks with strict criteria
//     for (int i = 5; i < BUFFER_SIZE - 5 && peakCount < maxPeaks; i++) {
//         // Check if it's a local maximum
//         bool isPeak = filtered[i] > threshold &&
//                       filtered[i] > filtered[i-1] && 
//                       filtered[i] > filtered[i-2] &&
//                       filtered[i] > filtered[i-3] &&
//                       filtered[i] >= filtered[i+1] && 
//                       filtered[i] >= filtered[i+2] &&
//                       filtered[i] >= filtered[i+3];
        
//         if (isPeak) {
//             // Check distance from last peak
//             if (peakCount == 0 || (i - peaks[peakCount-1]) >= minDistance) {
//                 peaks[peakCount++] = i;
//             }
//         }
//     }
    
//     Serial.print("Peaks found: ");
//     Serial.println(peakCount);
    
//     if (peakCount < 3) return BPM; // Need at least 3 peaks for stable reading
    
//     // Step 3: Calculate intervals and use median
//     float intervals[maxPeaks];
//     int validIntervals = 0;
    
//     for (int i = 1; i < peakCount; i++) {
//         int interval = peaks[i] - peaks[i-1];
//         // Filter out unrealistic intervals
//         if (interval >= minDistance && interval <= maxDistance) {
//             intervals[validIntervals++] = interval;
//         }
//     }
    
//     if (validIntervals < 2) return BPM;
    
//     // Sort intervals to find median
//     for (int i = 0; i < validIntervals - 1; i++) {
//         for (int j = 0; j < validIntervals - i - 1; j++) {
//             if (intervals[j] > intervals[j + 1]) {
//                 float temp = intervals[j];
//                 intervals[j] = intervals[j + 1];
//                 intervals[j + 1] = temp;
//             }
//         }
//     }
    
//     // Use median interval for stability
//     float medianInterval = intervals[validIntervals / 2];
//     float bpm = (60.0 * SAMPLE_RATE) / medianInterval;
    
//     // Apply smoothing with previous value
//     if (BPM > 0) {
//         bpm = BPM * 0.7 + bpm * 0.3; // 70% old, 30% new
//     }
    
//     // Validate range
//     if (bpm < 40 || bpm > 150) return BPM; // Keep old value if out of range
    
//     return bpm;
// }

// // ===== Calculate SpO2 =====
// float calculateSpO2() {
//     if (bufferCount < BUFFER_SIZE) return 0;
    
//     // Calculate DC (mean)
//     float redDC = 0, irDC = 0;
//     for (int i = 0; i < BUFFER_SIZE; i++) {
//         redDC += redBuffer[i];
//         irDC += irBuffer[i];
//     }
//     redDC /= BUFFER_SIZE;
//     irDC /= BUFFER_SIZE;
    
//     // Calculate AC (RMS of deviations)
//     float redAC = 0, irAC = 0;
//     for (int i = 0; i < BUFFER_SIZE; i++) {
//         float redDiff = redBuffer[i] - redDC;
//         float irDiff = irBuffer[i] - irDC;
//         redAC += redDiff * redDiff;
//         irAC += irDiff * irDiff;
//     }
//     redAC = sqrt(redAC / BUFFER_SIZE);
//     irAC = sqrt(irAC / BUFFER_SIZE);
    
//     // Avoid division by zero
//     if (irAC == 0 || irDC == 0 || redDC == 0) return 0;
    
//     // Calculate R ratio
//     float R = (redAC / redDC) / (irAC / irDC);
    
//     // Empirical formula (calibrated)
//     float spo2 = 110 - 25 * R;
    
//     // Clamp to valid range
//     spo2 = constrain(spo2, 70, 100);
    
//     return spo2;
// }

// // ===== Update Display =====
// void updateDisplay() {
//     u8g2.clearBuffer();
//     u8g2.setFont(u8g2_font_ncenB08_tr);
    
//     char buf[32];
    
//     switch (currentState) {
//         case WAITING_FOR_FINGER: {
//             u8g2.drawStr(10, 20, "Place finger on");
//             u8g2.drawStr(25, 35, "the sensor");
            
//             // Debug info
//             uint32_t ir = particleSensor.getIR();
//             uint32_t red = particleSensor.getRed();
            
//             u8g2.setFont(u8g2_font_6x10_tr);
//             snprintf(buf, sizeof(buf), "IR: %lu", ir);
//             u8g2.drawStr(5, 55, buf);
//             snprintf(buf, sizeof(buf), "Red: %lu", red);
//             u8g2.drawStr(5, 64, buf);
//             break;
//         }
        
//         case CALIBRATING: {
//             u8g2.drawStr(20, 20, "Calibrating");
//             u8g2.drawStr(10, 35, "Hold still...");
            
//             // Progress bar
//             int progress = (calibrationSamples * 100) / CALIBRATION_SAMPLES;
//             u8g2.drawFrame(14, 42, 100, 10);
//             u8g2.drawBox(16, 44, progress * 96 / 100, 6);
            
//             u8g2.setFont(u8g2_font_6x10_tr);
//             snprintf(buf, sizeof(buf), "%d%%", progress);
//             u8g2.drawStr(50, 62, buf);
//             break;
//         }
        
//         case MEASURING: {
//             // BPM
//             u8g2.setFont(u8g2_font_ncenB10_tr);
//             if (BPM > 0) {
//                 snprintf(buf, sizeof(buf), "%.0f", BPM);
//                 u8g2.drawStr(5, 12, buf);
//             } else {
//                 u8g2.drawStr(5, 12, "--");
//             }
//             u8g2.setFont(u8g2_font_6x10_tr);
//             u8g2.drawStr(35, 12, "BPM");
            
//             // SpO2
//             u8g2.setFont(u8g2_font_ncenB10_tr);
//             if (SpO2 > 0) {
//                 snprintf(buf, sizeof(buf), "%.0f", SpO2);
//                 u8g2.drawStr(75, 12, buf);
//             } else {
//                 u8g2.drawStr(75, 12, "--");
//             }
//             u8g2.setFont(u8g2_font_6x10_tr);
//             u8g2.drawStr(105, 12, "%");
//             u8g2.drawStr(75, 20, "SpO2");
            
//             // Separator
//             u8g2.drawLine(0, 24, 127, 24);
            
//             // Waveform
//             for (int i = 1; i < WAVEFORM_WIDTH; i++) {
//                 int idx1 = (waveIndex + i - 1) % WAVEFORM_WIDTH;
//                 int idx2 = (waveIndex + i) % WAVEFORM_WIDTH;
//                 int y1 = 54 - waveform[idx1];
//                 int y2 = 54 - waveform[idx2];
//                 u8g2.drawLine(i-1, y1, i, y2);
//             }
            
//             // Status
//             if (bufferCount < BUFFER_SIZE) {
//                 u8g2.setFont(u8g2_font_6x10_tr);
//                 u8g2.drawStr(25, 63, "Analyzing...");
//             }
//             break;
//         }
//     }
    
//     u8g2.sendBuffer();
// }




#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "MAX30105.h"
#include <math.h>

// ===== OLED =====
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ===== MAX30102 =====
MAX30105 particleSensor;

// ===== Constants =====
#define SAMPLE_RATE 100
#define BUFFER_SIZE 400
#define DISPLAY_BUFFER 128
#define CALIBRATION_TIME 3000
#define FINGER_THRESHOLD 50000

// ===== State Machine =====
enum State {
    WAITING_FOR_FINGER,
    CALIBRATING,
    MEASURING
};

State currentState = WAITING_FOR_FINGER;

// ===== Buffers =====
int32_t irBuffer[BUFFER_SIZE];
int32_t redBuffer[BUFFER_SIZE];
int32_t irDisplay[DISPLAY_BUFFER];
int bufferIdx = 0;
int displayIdx = 0;
bool bufferFull = false;

// ===== Metrics =====
float BPM = 0;
float SpO2 = 0;

// ===== Timing =====
unsigned long lastSample = 0;
unsigned long stateStart = 0;
unsigned long lastCalc = 0;

// ===== Function prototypes =====
void updateDisplay();
void readSensor();
bool isFingerDetected();
float calculateBPM();
float calculateSpO2();

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== MAX30102 Oximeter ===");

    Wire.begin();
    Wire.setClock(400000);

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(15, 30, "Initializing...");
    u8g2.sendBuffer();

    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("Sensor Error!");
        u8g2.clearBuffer();
        u8g2.drawStr(10, 30, "Sensor Error!");
        u8g2.sendBuffer();
        while (1);
    }

    // Configure for best signal quality
    particleSensor.setup(50, 4, 2, 100, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x24);
    particleSensor.setPulseAmplitudeIR(0x24);
    
    delay(100);
    Serial.println("Ready!");
}

// ===== Main Loop =====
void loop() {
    unsigned long now = millis();
    
    // Sample at 100Hz
    if (now - lastSample >= 10) {
        lastSample = now;
        readSensor();
        
        // State machine
        switch (currentState) {
            case WAITING_FOR_FINGER:
                if (isFingerDetected()) {
                    Serial.println("Finger detected!");
                    currentState = CALIBRATING;
                    stateStart = now;
                    bufferIdx = 0;
                    bufferFull = false;
                }
                break;
                
            case CALIBRATING:
                if (!isFingerDetected()) {
                    Serial.println("Finger removed");
                    currentState = WAITING_FOR_FINGER;
                    BPM = 0;
                    SpO2 = 0;
                } else if (now - stateStart >= CALIBRATION_TIME) {
                    Serial.println("Calibration complete");
                    currentState = MEASURING;
                    lastCalc = now;
                }
                break;
                
            case MEASURING:
                if (!isFingerDetected()) {
                    Serial.println("Finger removed");
                    currentState = WAITING_FOR_FINGER;
                    BPM = 0;
                    SpO2 = 0;
                    memset(irDisplay, 0, sizeof(irDisplay));
                } else {
                    // Calculate every 1 second
                    if (bufferFull && now - lastCalc >= 1000) {
                        BPM = calculateBPM();
                        SpO2 = calculateSpO2();
                        lastCalc = now;
                        
                        Serial.print("BPM: ");
                        Serial.print(BPM, 1);
                        Serial.print(" | SpO2: ");
                        Serial.print(SpO2, 1);
                        Serial.println("%");
                    }
                }
                break;
        }
    }
    
    updateDisplay();
}

// ===== Read Sensor =====
void readSensor() {
    int32_t ir = particleSensor.getIR();
    int32_t red = particleSensor.getRed();
    
    if (currentState != WAITING_FOR_FINGER) {
        // Store in circular buffer
        irBuffer[bufferIdx] = ir;
        redBuffer[bufferIdx] = red;
        
        bufferIdx++;
        if (bufferIdx >= BUFFER_SIZE) {
            bufferIdx = 0;
            bufferFull = true;
        }
        
        // Store for display (real-time waveform)
        if (currentState == MEASURING) {
            irDisplay[displayIdx] = ir;
            displayIdx = (displayIdx + 1) % DISPLAY_BUFFER;
        }
    }
}

// ===== Finger Detection =====
bool isFingerDetected() {
    int32_t ir = particleSensor.getIR();
    return (ir > FINGER_THRESHOLD && ir < 200000);
}

// ===== Calculate BPM =====
float calculateBPM() {
    if (!bufferFull) return BPM;
    
    // Copy buffer to array for processing
    float signal[BUFFER_SIZE];
    
    // Calculate mean
    float mean = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        signal[i] = irBuffer[i];
        mean += signal[i];
    }
    mean /= BUFFER_SIZE;
    
    // Detrend and normalize
    float stdDev = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        signal[i] -= mean;
        stdDev += signal[i] * signal[i];
    }
    stdDev = sqrt(stdDev / BUFFER_SIZE);
    
    if (stdDev < 10) return BPM; // No pulse detected
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
        signal[i] /= stdDev;
    }
    
    // Find peaks with adaptive threshold
    int peaks[30];
    int peakCount = 0;
    
    float threshold = 0.5; // Normalized threshold
    int minPeakDistance = 40; // 0.4 sec = 150 BPM max
    
    for (int i = 5; i < BUFFER_SIZE - 5 && peakCount < 30; i++) {
        if (signal[i] > threshold &&
            signal[i] > signal[i-1] && signal[i] > signal[i-2] &&
            signal[i] > signal[i-3] && signal[i] > signal[i-4] &&
            signal[i] >= signal[i+1] && signal[i] >= signal[i+2] &&
            signal[i] >= signal[i+3] && signal[i] >= signal[i+4]) {
            
            // Check minimum distance from last peak
            if (peakCount == 0 || (i - peaks[peakCount-1]) >= minPeakDistance) {
                peaks[peakCount++] = i;
            }
        }
    }
    
    Serial.print("Peaks: ");
    Serial.println(peakCount);
    
    if (peakCount < 3) return BPM;
    
    // Calculate RR intervals
    float intervals[29];
    int intervalCount = 0;
    
    for (int i = 1; i < peakCount; i++) {
        float interval = peaks[i] - peaks[i-1];
        float bpmFromInterval = 6000.0 / interval; // 60 * 100 / interval
        
        // Only accept realistic intervals (45-120 BPM)
        if (bpmFromInterval >= 45 && bpmFromInterval <= 120) {
            intervals[intervalCount++] = interval;
        }
    }
    
    if (intervalCount < 2) return BPM;
    
    // Use median interval
    for (int i = 0; i < intervalCount - 1; i++) {
        for (int j = 0; j < intervalCount - i - 1; j++) {
            if (intervals[j] > intervals[j + 1]) {
                float temp = intervals[j];
                intervals[j] = intervals[j + 1];
                intervals[j + 1] = temp;
            }
        }
    }
    
    float medianInterval = intervals[intervalCount / 2];
    float newBPM = 6000.0 / medianInterval;
    
    // Smooth with previous value
    if (BPM > 0 && BPM >= 50 && BPM <= 110) {
        newBPM = BPM * 0.75 + newBPM * 0.25;
    }
    
    // Validate final range
    if (newBPM < 50 || newBPM > 110) {
        return BPM;
    }
    
    return newBPM;
}

// ===== Calculate SpO2 =====
float calculateSpO2() {
    if (!bufferFull) return SpO2;
    
    // Use last 2 seconds
    int samples = 200;
    
    float redDC = 0, irDC = 0;
    float redMin = 999999, redMax = 0;
    float irMin = 999999, irMax = 0;
    
    int startIdx = (bufferIdx - samples + BUFFER_SIZE) % BUFFER_SIZE;
    
    for (int i = 0; i < samples; i++) {
        int idx = (startIdx + i) % BUFFER_SIZE;
        
        redDC += redBuffer[idx];
        irDC += irBuffer[idx];
        
        if (redBuffer[idx] < redMin) redMin = redBuffer[idx];
        if (redBuffer[idx] > redMax) redMax = redBuffer[idx];
        if (irBuffer[idx] < irMin) irMin = irBuffer[idx];
        if (irBuffer[idx] > irMax) irMax = irBuffer[idx];
    }
    
    redDC /= samples;
    irDC /= samples;
    
    float redAC = (redMax - redMin) / 2.0;
    float irAC = (irMax - irMin) / 2.0;
    
    if (irAC == 0 || irDC == 0 || redDC == 0) return SpO2;
    
    float R = (redAC / redDC) / (irAC / irDC);
    
    // Calibrated formula
    float newSpO2 = 110 - 25 * R;
    
    // Smooth
    if (SpO2 > 0 && SpO2 >= 90 && SpO2 <= 100) {
        newSpO2 = SpO2 * 0.8 + newSpO2 * 0.2;
    }
    
    newSpO2 = constrain(newSpO2, 90, 100);
    
    return newSpO2;
}

// ===== Update Display =====
void updateDisplay() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    
    char buf[32];
    
    switch (currentState) {
        case WAITING_FOR_FINGER: {
            u8g2.drawStr(10, 25, "Place your finger");
            u8g2.drawStr(25, 40, "on sensor");
            
            int32_t ir = particleSensor.getIR();
            u8g2.setFont(u8g2_font_6x10_tr);
            snprintf(buf, sizeof(buf), "IR: %ld", ir);
            u8g2.drawStr(35, 58, buf);
            break;
        }
        
        case CALIBRATING: {
            u8g2.drawStr(25, 25, "Calibrating");
            u8g2.drawStr(15, 40, "Keep finger still");
            
            int progress = ((millis() - stateStart) * 100) / CALIBRATION_TIME;
            progress = constrain(progress, 0, 100);
            
            u8g2.drawFrame(14, 45, 100, 8);
            u8g2.drawBox(15, 46, progress * 98 / 100, 6);
            
            u8g2.setFont(u8g2_font_6x10_tr);
            snprintf(buf, sizeof(buf), "%d%%", progress);
            u8g2.drawStr(54, 61, buf);
            break;
        }
        
        case MEASURING: {
            // Display metrics
            u8g2.setFont(u8g2_font_ncenB14_tr);
            
            // BPM
            if (BPM >= 50) {
                snprintf(buf, sizeof(buf), "%.0f", BPM);
            } else {
                strcpy(buf, "--");
            }
            u8g2.drawStr(5, 15, buf);
            
            u8g2.setFont(u8g2_font_6x10_tr);
            u8g2.drawStr(5, 24, "BPM");
            
            // SpO2
            u8g2.setFont(u8g2_font_ncenB14_tr);
            if (SpO2 >= 90) {
                snprintf(buf, sizeof(buf), "%.0f", SpO2);
            } else {
                strcpy(buf, "--");
            }
            u8g2.drawStr(75, 15, buf);
            
            u8g2.setFont(u8g2_font_6x10_tr);
            u8g2.drawStr(110, 15, "%");
            u8g2.drawStr(75, 24, "SpO2");
            
            // Divider
            u8g2.drawLine(0, 27, 127, 27);
            
            // Waveform - real-time pulse shape
            if (bufferFull) {
                // Find min/max for scaling
                int32_t minVal = 999999;
                int32_t maxVal = 0;
                
                for (int i = 0; i < DISPLAY_BUFFER; i++) {
                    if (irDisplay[i] > 0) {
                        if (irDisplay[i] < minVal) minVal = irDisplay[i];
                        if (irDisplay[i] > maxVal) maxVal = irDisplay[i];
                    }
                }
                
                // Draw waveform
                if (maxVal > minVal) {
                    for (int i = 1; i < DISPLAY_BUFFER; i++) {
                        int idx1 = (displayIdx + i - 1) % DISPLAY_BUFFER;
                        int idx2 = (displayIdx + i) % DISPLAY_BUFFER;
                        
                        if (irDisplay[idx1] > 0 && irDisplay[idx2] > 0) {
                            int y1 = map(irDisplay[idx1], minVal, maxVal, 62, 32);
                            int y2 = map(irDisplay[idx2], minVal, maxVal, 62, 32);
                            
                            y1 = constrain(y1, 32, 62);
                            y2 = constrain(y2, 32, 62);
                            
                            u8g2.drawLine(i - 1, y1, i, y2);
                        }
                    }
                }
                
                // Baseline
                u8g2.drawLine(0, 62, 127, 62);
            } else {
                u8g2.setFont(u8g2_font_6x10_tr);
                u8g2.drawStr(30, 50, "Reading...");
            }
            
            break;
        }
    }
    
    u8g2.sendBuffer();
}

