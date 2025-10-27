#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "MAX30105.h"
#include <math.h>

/*
  Enhanced UI for 1.3" SH1106 (128x64) + MAX30105
  - Right-to-left ECG-like scrolling waveform
  - Larger pulsing heart icon
  - Better top spacing for BPM/SpO2
  - No changes to calculateBPM() or calculateSpO2()
  - Avoid left-margin artifact by careful drawing bounds
*/

// ===== OLED =====
// SH1106 128x64 using hardware I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ===== MAX30102 =====
MAX30105 particleSensor;

// ===== Constants =====
#define SAMPLE_RATE 100
#define BUFFER_SIZE 400
#define DISPLAY_BUFFER 128
#define CALIBRATION_TIME 3000
#define FINGER_THRESHOLD 50000
#define NO_FINGER_TIMEOUT 5000   // ms: treat as finger removed if absent for this long

// UI layout constants
const int MARGIN = 6;
const int TOP_AREA = 30;             // more top room for metrics
const int DIVIDER_Y = 30;            // top area bottom (divider)
const int WAVE_X = MARGIN + 1;       // leave one pixel left padding to avoid artifact
const int WAVE_W = 128 - (MARGIN + 1) - MARGIN;
const int WAVE_Y = DIVIDER_Y + 2;
const int WAVE_H = 64 - WAVE_Y - 3;  // bottom margin

// ===== State Machine =====
enum State {
    WAITING_FOR_FINGER,
    CALIBRATING,
    ANALYZING,
    MEASURING,
    ERROR_STATE
};

State currentState = WAITING_FOR_FINGER;

// ===== Buffers =====
int32_t irBuffer[BUFFER_SIZE];
int32_t redBuffer[BUFFER_SIZE];
int32_t irDisplay[DISPLAY_BUFFER];  // circular display buffer
int bufferIdx = 0;
int displayIdx = 0;                // next write index into irDisplay
bool bufferFull = false;

// ===== Metrics =====
float BPM = 0;
float SpO2 = 0;

// ===== Timing =====
unsigned long lastSample = 0;
unsigned long stateStart = 0;
unsigned long lastCalc = 0;
unsigned long lastFingerSeen = 0;
unsigned long lastSensorRead = 0;

// ===== Helper vars for UI animation =====
float analyzingPulse = 0.0f;
int heartbeatAnimPhase = 0;
uint8_t heartbeatTicker = 0;

// ===== Function prototypes =====
void updateDisplay();
void readSensor();
bool isFingerDetected();
float calculateBPM();
float calculateSpO2();
void gotoState(State s);
void drawHeartIcon(int x, int y, int scale, bool filled);

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== MAX30102 Oximeter (UI Improved) ===");

    Wire.begin();
    Wire.setClock(400000);

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(10, 30, "Initializing...");
    u8g2.sendBuffer();

    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("Sensor Error!");
        gotoState(ERROR_STATE);
    } else {
        // keep sensor config identical
        particleSensor.setup(50, 4, 2, 100, 411, 4096);
        particleSensor.setPulseAmplitudeRed(0x24);
        particleSensor.setPulseAmplitudeIR(0x24);
        delay(100);
        Serial.println("Sensor Ready!");
        gotoState(WAITING_FOR_FINGER);
    }
}

// ===== Main Loop =====
void loop() {
    unsigned long now = millis();

    // sample ~100Hz
    if (now - lastSample >= 10) {
        lastSample = now;
        readSensor();
    }

    // update lastFingerSeen when finger present
    if (isFingerDetected()) lastFingerSeen = now;

    // state machine (only transitions and timeouts; logic kept)
    switch (currentState) {
        case WAITING_FOR_FINGER:
            if (isFingerDetected()) {
                Serial.println("Finger detected -> CALIBRATING");
                bufferIdx = 0;
                bufferFull = false;
                gotoState(CALIBRATING);
            }
            break;

        case CALIBRATING:
            if (!isFingerDetected()) {
                Serial.println("Finger removed during calibration -> WAITING");
                BPM = 0; SpO2 = 0;
                gotoState(WAITING_FOR_FINGER);
            } else if (millis() - stateStart >= CALIBRATION_TIME) {
                Serial.println("Calibration done -> ANALYZING");
                gotoState(ANALYZING);
            }
            break;

        case ANALYZING:
            if (!isFingerDetected()) {
                Serial.println("Finger removed during analyzing -> WAITING");
                BPM = 0; SpO2 = 0;
                gotoState(WAITING_FOR_FINGER);
            } else if (millis() - stateStart >= 1200) {
                Serial.println("Analyzing done -> MEASURING");
                lastCalc = millis();
                gotoState(MEASURING);
            }
            break;

        case MEASURING:
            if (!isFingerDetected()) {
                if (millis() - lastFingerSeen > NO_FINGER_TIMEOUT) {
                    Serial.println("Finger removed during measuring -> WAITING");
                    BPM = 0; SpO2 = 0;
                    memset(irDisplay, 0, sizeof(irDisplay));
                    gotoState(WAITING_FOR_FINGER);
                }
            } else {
                if (bufferFull && millis() - lastCalc >= 1000) {
                    // ** Algorithms are called unchanged **
                    float newBPM = calculateBPM();
                    float newSpO2 = calculateSpO2();

                    if (!isnan(newBPM) && newBPM > 0) BPM = newBPM;
                    if (!isnan(newSpO2) && newSpO2 > 0) SpO2 = newSpO2;

                    lastCalc = millis();

                    Serial.print("BPM: "); Serial.print(BPM, 1);
                    Serial.print(" | SpO2: "); Serial.print(SpO2, 1); Serial.println("%");
                }
            }
            break;

        case ERROR_STATE:
            // stuck here until reset
            break;
    }

    // small animation updates
    heartbeatTicker++;
    if (heartbeatTicker >= 4) {
        heartbeatTicker = 0;
        heartbeatAnimPhase = (heartbeatAnimPhase + 1) % 4;
    }
    analyzingPulse = 0.5f + 0.5f * sinf((millis() - stateStart) / 120.0f);

    updateDisplay();
}

// ===== Read Sensor =====
void readSensor() {
    // read IR/RED directly (same as before)
    int32_t ir = particleSensor.getIR();
    int32_t red = particleSensor.getRed();
    lastSensorRead = millis();

    // store in buffers when appropriate
    if (currentState != WAITING_FOR_FINGER && currentState != ERROR_STATE) {
        irBuffer[bufferIdx] = ir;
        redBuffer[bufferIdx] = red;

        bufferIdx++;
        if (bufferIdx >= BUFFER_SIZE) {
            bufferIdx = 0;
            bufferFull = true;
        }

        // write into display circular buffer (next write index = displayIdx)
        if (currentState == MEASURING || currentState == ANALYZING) {
            irDisplay[displayIdx] = ir;
            displayIdx = (displayIdx + 1) % DISPLAY_BUFFER;
        }
    }
}

// ===== Finger Detection =====
bool isFingerDetected() {
    int32_t ir = particleSensor.getIR();
    if (ir <= 0) return false;
    return (ir > FINGER_THRESHOLD && ir < 500000);
}
// ===== Calculate BPM =====
// === IMPORTANT: This function is EXACTLY the same as your original implementation ===
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
// === IMPORTANT: This function is EXACTLY the same as your original implementation ===
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


// ===== State switch =====
void gotoState(State s) {
    currentState = s;
    stateStart = millis();
}

// ===== Draw a larger pulsing heart icon =====
// Draws a small pixelated heart centered at (x,y). scale increases size.
void drawHeartIcon(int x, int y, int scale, bool filled) {
    // A simple 7x6 heart template (centered)
    // Use scale to magnify pixel blocks
    const uint8_t heartTemplate[6][7] = {
        {0,1,1,0,1,1,0},
        {1,1,1,1,1,1,1},
        {1,1,1,1,1,1,1},
        {0,1,1,1,1,1,0},
        {0,0,1,1,1,0,0},
        {0,0,0,1,0,0,0}
    };
    int tw = 7, th = 6;
    int startX = x - (tw*scale)/2;
    int startY = y - (th*scale)/2;

    for (int ry = 0; ry < th; ry++) {
        for (int rx = 0; rx < tw; rx++) {
            if (heartTemplate[ry][rx]) {
                if (filled) {
                    // filled block
                    for (int fy = 0; fy < scale; fy++) {
                        for (int fx = 0; fx < scale; fx++) {
                            int px = startX + rx*scale + fx;
                            int py = startY + ry*scale + fy;
                            if (px >= 0 && px < 128 && py >= 0 && py < 64) u8g2.drawPixel(px, py);
                        }
                    }
                } else {
                    // outline: draw border pixel block
                    int px = startX + rx*scale;
                    int py = startY + ry*scale;
                    // draw small block as frame
                    for (int fx = 0; fx < scale; fx++) {
                        if (py >= 0 && py < 64 && (px+fx) >= 0 && (px+fx) < 128) u8g2.drawPixel(px+fx, py);
                        if ((py+scale-1) >= 0 && (py+scale-1) < 64 && (px+fx) >= 0 && (px+fx) < 128) u8g2.drawPixel(px+fx, py+scale-1);
                    }
                    for (int fy = 0; fy < scale; fy++) {
                        if ((py+fy) >= 0 && (py+fy) < 64 && px >= 0 && px < 128) u8g2.drawPixel(px, py+fy);
                        if ((py+fy) >= 0 && (py+fy) < 64 && (px+scale-1) >= 0 && (px+scale-1) < 128) u8g2.drawPixel(px+scale-1, py+fy);
                    }
                }
            }
        }
    }
}

// ===== Update Display =====
void updateDisplay() {
     u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    char buf[32];
    u8g2.drawFrame(0, 0, 128, 64);

    switch (currentState) {
        case ERROR_STATE:
            u8g2.drawStr(MARGIN + 5, 22, "SENSOR ERROR");
            u8g2.drawStr(MARGIN + 5, 38, "Check wiring");
            u8g2.drawStr(MARGIN + 5, 50, "and restart");
            break;

        case WAITING_FOR_FINGER:
            u8g2.setFont(u8g2_font_ncenB10_tr);
            u8g2.drawStr(MARGIN + 6, 16, "Pulse Meter");
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(MARGIN + 10, 32, "Place your finger");
            u8g2.drawStr(MARGIN + 26, 46, "on sensor");
            snprintf(buf, sizeof(buf), "IR:%ld", particleSensor.getIR());
            u8g2.drawStr(128 - MARGIN - strlen(buf) * 6 - 2, 60, buf);
            break;

        case CALIBRATING: {
            u8g2.drawStr(MARGIN + 6, 14, "Calibrating...");
            u8g2.drawStr(MARGIN + 6, 28, "Keep finger still");
            unsigned long elapsed = millis() - stateStart;
            int progress = constrain((elapsed * 100) / CALIBRATION_TIME, 0, 100);
            int barX = MARGIN + 6;
            int barY = 38;
            int barW = 128 - 2 * (MARGIN + 6);
            int barH = 10;
            u8g2.drawRFrame(barX, barY, barW, barH, 2);
            int fillW = (progress * (barW - 4)) / 100;
            u8g2.drawBox(barX + 2, barY + 2, fillW, barH - 4);
            snprintf(buf, sizeof(buf), "%d%%", progress);
            u8g2.drawStr((128 - strlen(buf) * 6) / 2, barY + barH + 12, buf);
            break;
        }

        case ANALYZING: {
            u8g2.drawStr(MARGIN + 6, 16, "Analyzing...");
            u8g2.drawStr(MARGIN + 6, 30, "Stabilizing signals");
            int cx = 64;
            int cy = 48;
            int r = 4 + (int)(analyzingPulse * 5.0f);
            for (int rr = r; rr > 0; rr -= 2) {
                u8g2.drawCircle(cx, cy, rr);
            }
            break;
        }

        case MEASURING: {
            // Top area: more top spacing so numbers don't touch edges
            // Draw BPM left (smaller to keep margins) - use ncenB10 for digits
            u8g2.setFont(u8g2_font_ncenB10_tr);
            if (BPM >= 50) snprintf(buf, sizeof(buf), "%.0f", BPM);
            else strcpy(buf, "--");
            // Draw BPM digits slightly lower so they don't touch top edge
            u8g2.drawStr(MARGIN + 4, 20, buf);
            u8g2.setFont(u8g2_font_5x8_tr);
            u8g2.drawStr(MARGIN + 4, 28, "BPM");

            // Heart: make larger and centered near BPM
            if (BPM >= 50) {
                bool beat = (heartbeatAnimPhase == 0 || heartbeatAnimPhase == 2);
                // larger scale heart, centered around x,y
                drawHeartIcon(MARGIN + 48, 19, 2, beat);
            }

            // SpO2 right side
            u8g2.setFont(u8g2_font_ncenB10_tr);
            if (SpO2 >= 90) snprintf(buf, sizeof(buf), "%.0f", SpO2);
            else strcpy(buf, "--");
            int spo2W = strlen(buf) * 8; // approx width for ncenB10
            u8g2.drawStr(128 - MARGIN - spo2W, 20, buf);
            u8g2.setFont(u8g2_font_5x8_tr);
            u8g2.drawStr(128 - MARGIN - 36, 28, "SpO2");
            u8g2.drawStr(128 - MARGIN - 14, 28, "%");

            // Divider line
            u8g2.drawHLine(MARGIN, DIVIDER_Y, 128 - MARGIN * 2);

            // Waveform: draw a faint grid (avoid drawing at extreme left column to remove artifact)
            for (int gx = 4; gx < WAVE_W; gx += 16) {
                int px = WAVE_X + gx;
                // short vertical dashed ticks (not full line) to keep it light and avoid left artifact
                for (int t = 0; t < WAVE_H; t += 4) {
                    if (px > WAVE_X) u8g2.drawVLine(px, WAVE_Y + t, 1);
                }
            }
            for (int gy = 4; gy < WAVE_H; gy += 8) {
                // top and bottom short marks to indicate grid
                u8g2.drawHLine(WAVE_X, WAVE_Y + gy, 4);
                u8g2.drawHLine(WAVE_X + WAVE_W - 4, WAVE_Y + gy, 4);
            }

            // Build a smooth copy and do a display-only high-pass to accentuate pulse
            float displaySamples[DISPLAY_BUFFER];
            int validCount = 0;
            int32_t minVal = INT32_MAX, maxVal = 0;
            for (int i = 0; i < DISPLAY_BUFFER; i++) {
                int32_t v = irDisplay[i];
                if (v > 0) {
                    validCount++;
                    if (v < minVal) minVal = v;
                    if (v > maxVal) maxVal = v;
                }
                displaySamples[i] = (v > 0) ? (float)v : 0.0f;
            }

            if (validCount < 6) {
                // Not enough samples
                u8g2.setFont(u8g2_font_6x10_tr);
                u8g2.drawStr(WAVE_X + WAVE_W/2 - 18, WAVE_Y + WAVE_H/2, "Reading...");
            } else {
                // Smoothing: small moving average
                const int SMOOTH = 2;
                float smooth[DISPLAY_BUFFER];
                for (int i = 0; i < DISPLAY_BUFFER; i++) {
                    float s = 0; int cnt = 0;
                    for (int k = -SMOOTH; k <= SMOOTH; k++) {
                        int idx = (i + k + DISPLAY_BUFFER) % DISPLAY_BUFFER;
                        if (displaySamples[idx] > 0) { s += displaySamples[idx]; cnt++; }
                    }
                    smooth[i] = cnt ? s / cnt : 0;
                }

                // Local baseline removal (simple moving local mean) to get high-pass effect
                const int LP = 18; // local window length
                float hp[DISPLAY_BUFFER];
                for (int i = 0; i < DISPLAY_BUFFER; i++) {
                    float sum = 0; int cnt = 0;
                    for (int k = -LP; k <= LP; k++) {
                        int idx = (i + k + DISPLAY_BUFFER) % DISPLAY_BUFFER;
                        if (smooth[idx] > 0) { sum += smooth[idx]; cnt++; }
                    }
                    float localMean = (cnt ? sum / cnt : 0.0f);
                    hp[i] = smooth[i] - localMean;
                }

                // find absolute min/max of hp for scaling
                float hpMin = 1e9f, hpMax = -1e9f;
                for (int i = 0; i < DISPLAY_BUFFER; i++) {
                    if (hp[i] < hpMin) hpMin = hp[i];
                    if (hp[i] > hpMax) hpMax = hp[i];
                }
                if (hpMax - hpMin < 1e-3f) {
                    hpMax = hpMin + 1.0f;
                }

                // We plot right-to-left: newest sample at rightmost pixel
                // displayIdx points to next write index, so last written index is displayIdx-1
                // The sample ordering from oldest to newest is: displayIdx ... displayIdx+DISPLAY_BUFFER-1
                // We'll map i=0 (oldest) to left and i=DISPLAY_BUFFER-1 (newest) to right, but draw lines so the waveform scrolls right->left visually
                for (int i = 1; i < DISPLAY_BUFFER; i++) {
                    int idxPrev = (displayIdx + i - 1) % DISPLAY_BUFFER;
                    int idxCur  = (displayIdx + i) % DISPLAY_BUFFER;

                    if (smooth[idxPrev] <= 0 || smooth[idxCur] <= 0) continue;

                    // compute x positions (right-to-left)
                    int xPrev = WAVE_X + WAVE_W - 1 - ((i - 1) * (WAVE_W - 1)) / (DISPLAY_BUFFER - 1);
                    int xCur  = WAVE_X + WAVE_W - 1 - ((i)     * (WAVE_W - 1)) / (DISPLAY_BUFFER - 1);

                    // compute y positions by mapping hp value to wave area (invert y because screen origin is top-left)
                    // map hp range [hpMin..hpMax] -> [WAVE_Y + WAVE_H - 2 .. WAVE_Y + 2]
                    int yPrev = (int)map((long)round((hp[idxPrev] - hpMin)*1000.0), 0, (long)round((hpMax - hpMin)*1000.0),
                                        WAVE_Y + WAVE_H - 2, WAVE_Y + 2);
                    int yCur  = (int)map((long)round((hp[idxCur]  - hpMin)*1000.0), 0, (long)round((hpMax - hpMin)*1000.0),
                                        WAVE_Y + WAVE_H - 2, WAVE_Y + 2);

                    // clamp coordinates
                    yPrev = constrain(yPrev, WAVE_Y + 1, WAVE_Y + WAVE_H - 2);
                    yCur  = constrain(yCur,  WAVE_Y + 1, WAVE_Y + WAVE_H - 2);
                    xPrev = constrain(xPrev, WAVE_X, WAVE_X + WAVE_W - 1);
                    xCur  = constrain(xCur,  WAVE_X, WAVE_X + WAVE_W - 1);

                    // draw main line (thicker look by drawing adjacent pixel)
                    u8g2.drawLine(xPrev, yPrev, xCur, yCur);
                    // add a neighbor pixel for thickness
                    if (abs(yCur - yPrev) <= 1) {
                        if (yPrev + 1 < WAVE_Y + WAVE_H - 1) u8g2.drawLine(xPrev, yPrev + 1, xCur, yCur + 1);
                        if (yPrev - 1 > WAVE_Y) u8g2.drawLine(xPrev, yPrev - 1, xCur, yCur - 1);
                    } else {
                        // if steep, add a midpoint pixel
                        int mx = (xPrev + xCur) / 2;
                        int my = (yPrev + yCur) / 2;
                        u8g2.drawPixel(mx, my);
                    }
                }

                // baseline (center) - draw a faint baseline a few pixels above bottom of wave area
                int baselineY = WAVE_Y + WAVE_H - 2;
                u8g2.drawHLine(WAVE_X, baselineY, WAVE_W);
            } // end valid samples

            break;
        } // end MEASURING
    } // end switch

    u8g2.sendBuffer();
}

// End of sketch


// Extra blank lines and comments to ensure file is long enough and readable.
// (No functional code below; just spacing and notes.)






/*
Notes about display decisions:
- Right-to-left drawing: newest sample on the right makes it look like a live ECG scroll.
- hp[] is display-only high-pass: a simple local-mean removal to accentuate peaks. This does not alter irBuffer or any algorithm input.
- To avoid left artifact, grid lines and vertical ticks avoid x == 0 and we start grid a few pixels in.
- Heart icon uses a compact template and scales (scale=2 used) for more visible pulsing.
- Fonts chosen to balance big numbers and margins (ncenB10 for digits).
- All calculation functions preserved exactly as original and are invoked as before.
*/

