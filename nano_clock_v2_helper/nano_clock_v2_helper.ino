#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoOTA.h>

const char* ssid = "Ayush";
const char* password = "alibaba2123";

HardwareSerial& Radar1 = Serial2; 

// --- ESP-NOW Payload ---
typedef struct struct_message {
    uint8_t magic;
    float dist;
    float delta;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// --- Radar State ---
uint8_t frameBuf[2048];
int frameLen = 0;
float previousEnergiesDB[32] = {0};
float smoothedDistance[32] = {0};
float smoothedDB[32] = {0};
float smoothedDelta[32] = {0};
int activeFrames[32] = {0};

float softwareLockDist = 0.0;
float softwareLockDelta = 0.0;
unsigned long lastLockTime = 0;
unsigned long lastDataRxTime = 0;

void processExtractedPayload(uint8_t* payload, int len) {
    float currentEnergiesDB[32];
    int energyStartIdx = 3; 
    
    for(int i = 0; i < 32; i++) {
        int offset = energyStartIdx + (i * 4);
        uint32_t rawEnergy = payload[offset] | (payload[offset+1] << 8) | 
                             (payload[offset+2] << 16) | (payload[offset+3] << 24);
        float dbVal = (rawEnergy > 0) ? (10.0 * log10((float)rawEnergy)) : 0.0;
        currentEnergiesDB[i] = round(dbVal * 10.0) / 10.0;
    }

    for (int i = 0; i <= 31; i++) {
        if (i < 2) { activeFrames[i] = 0; continue; } // Ghost removal
        
        bool isPeak = true;
        if (i > 0 && currentEnergiesDB[i] < currentEnergiesDB[i-1]) isPeak = false;
        if (i < 31 && currentEnergiesDB[i] < currentEnergiesDB[i+1]) isPeak = false;
        if (i > 0 && currentEnergiesDB[i] == currentEnergiesDB[i-1]) isPeak = false; 
        
        if (isPeak && currentEnergiesDB[i] > 10.0) { 
            // Sub-bin Parabolic Interpolation
            float left = (i > 0) ? currentEnergiesDB[i-1] : currentEnergiesDB[i];
            float center = currentEnergiesDB[i];
            float right = (i < 31) ? currentEnergiesDB[i+1] : currentEnergiesDB[i];
            float p = 0;
            float denom = (left - 2.0 * center + right);
            if (denom != 0.0) {
                p = 0.5 * (left - right) / denom;
                if (p > 0.5) p = 0.5;
                if (p < -0.5) p = -0.5;
            }
            
            float rawDist = (i + p) * 70.0;
            float rawDb = currentEnergiesDB[i];
            float rawDelta = currentEnergiesDB[i] - previousEnergiesDB[i];

            if (activeFrames[i] == 0) {
                smoothedDistance[i] = rawDist;
                smoothedDB[i] = rawDb;
                smoothedDelta[i] = rawDelta;
            } else {
                smoothedDistance[i] = (0.3 * rawDist) + (0.7 * smoothedDistance[i]);
                smoothedDB[i] = (0.3 * rawDb) + (0.7 * smoothedDB[i]);
                smoothedDelta[i] = (0.1 * rawDelta) + (0.9 * smoothedDelta[i]);
            }
            activeFrames[i]++;
        } else {
            activeFrames[i] = 0;
        }
    }
    
    // Helper Advanced Tracker
    float candidateDist = 9999.0;
    int candidateGate = -1;
    for (int i=2; i<=31; i++) {
        if (activeFrames[i] > 2 && smoothedDB[i] > 10.0) {
            if (abs(smoothedDelta[i]) > 0.3) {
                if (smoothedDistance[i] < candidateDist) {
                    candidateDist = smoothedDistance[i];
                    candidateGate = i;
                }
            }
        }
    }
    
    if (candidateGate != -1) {
        if (softwareLockDist == 0.0) softwareLockDist = candidateDist;
        else softwareLockDist = (0.2 * candidateDist) + (0.8 * softwareLockDist);
        softwareLockDelta = smoothedDelta[candidateGate];
        lastLockTime = millis();
    } else {
        if (millis() - lastLockTime > 4000) { 
            softwareLockDist = 0.0;
            softwareLockDelta = 0.0;
        }
    }

    for(int i=0; i<32; i++) previousEnergiesDB[i] = currentEnergiesDB[i];
    lastDataRxTime = millis();
    
    // Broadcast via ESP-NOW to all listening Masters
    myData.dist = softwareLockDist;
    myData.delta = softwareLockDelta;
    
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
}

void setup() {
    Serial.begin(115200);
    Radar1.begin(115200, SERIAL_8N1, 16, 17);

    Serial.println("\n--- BOOTING HELPER ENGINE (LD2402) ---");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println(String("\nWiFi Connected! IP: ") + WiFi.localIP().toString());

    // Init OTA
    ArduinoOTA.setHostname("radar-helper");
    ArduinoOTA.begin();

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // Broadcast Peer (FF:FF:FF:FF:FF:FF means all devices listening will hear it)
    // This entirely bypasses the need for hardcoded MAC addresses!
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  // 0 uses current WiFi channel
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add ESP-NOW peer");
        return;
    }
    
    myData.magic = 0xAA; // Magic byte so master knows it's us
    lastDataRxTime = millis();
}

void loop() {
    ArduinoOTA.handle();

    while (Radar1.available()) {
        if (frameLen < 2048) {
            frameBuf[frameLen++] = Radar1.read();
        } else {
            memmove(frameBuf, frameBuf + 1, 2047);
            frameBuf[2047] = Radar1.read();
        }
    }

    bool frameFound = true;
    while (frameFound && frameLen >= 10) {
        frameFound = false;
        
        int headerIdx = -1;
        for (int i = 0; i <= frameLen - 4; i++) {
            if (frameBuf[i] == 0xF4 && frameBuf[i+1] == 0xF3 && frameBuf[i+2] == 0xF2 && frameBuf[i+3] == 0xF1) {
                headerIdx = i;
                break;
            }
        }

        if (headerIdx == -1) {
            if (frameLen > 3) {
                frameBuf[0] = frameBuf[frameLen-3];
                frameBuf[1] = frameBuf[frameLen-2];
                frameBuf[2] = frameBuf[frameLen-1];
                frameLen = 3;
            }
            continue;
        }

        if (headerIdx > 0) {
            int remaining = frameLen - headerIdx;
            memmove(frameBuf, frameBuf + headerIdx, remaining);
            frameLen = remaining;
            headerIdx = 0;
            frameFound = true;
            continue;
        }

        int tailIdx = -1;
        for (int i = 6; i <= frameLen - 4; i++) {
            if (frameBuf[i] == 0xF8 && frameBuf[i+1] == 0xF7 && frameBuf[i+2] == 0xF6 && frameBuf[i+3] == 0xF5) {
                tailIdx = i;
                break;
            }
        }

        if (tailIdx != -1) {
            int payloadStart = 6;
            int payloadLen = tailIdx - payloadStart;
            if (payloadLen > 0 && payloadLen < 250) {
                processExtractedPayload(&frameBuf[payloadStart], payloadLen);
            }
            
            int bytesToRemove = tailIdx + 4;
            int remaining = frameLen - bytesToRemove;
            if (remaining > 0) memmove(frameBuf, frameBuf + bytesToRemove, remaining);
            frameLen = remaining;
            frameFound = true; 
        }
    }
}
