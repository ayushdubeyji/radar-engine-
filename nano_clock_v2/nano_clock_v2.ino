#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <math.h>
#include <FastLED.h>
#include <esp_now.h>
#include <ArduinoOTA.h>

// === ESP-NOW Helper Variables ===
typedef struct struct_message {
    uint8_t magic;
    float dist;
    float delta;
} struct_message;

float helperDist = 0.0;
float helperDelta = 0.0;
unsigned long lastHelperRxTime = 0;

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
    if (len == sizeof(struct_message)) {
        struct_message myData;
        memcpy(&myData, incomingData, sizeof(myData));
        if (myData.magic == 0xAA) {
            helperDist = myData.dist;
            helperDelta = myData.delta;
            lastHelperRxTime = millis();
        }
    }
}

const char* ssid = "Ayush";
const char* password = "alibaba2123";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
HardwareSerial& Radar1 = Serial2; 

// FastLED Configuration
#define LED_PIN     23
#define NUM_LEDS    1
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

enum LedMode { LED_MODE_DISTANCE, LED_MODE_BREATHING };
LedMode currentLedMode = LED_MODE_DISTANCE;
int currentLedBrightness = 0;
int targetLedBrightness = 0;
uint8_t danceHue = 0;
unsigned long lastLedUpdate = 0;

// Radar Hex Protocols
const byte enableCmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
const byte endCmd[]    = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
const byte engModeCmd[]= {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
const byte autoGainCmd[]={0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xEE, 0x00, 0x04, 0x03, 0x02, 0x01};
const byte autoThreshCmd[]={0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x09, 0x00, 0x1E, 0x00, 0x14, 0x00, 0x1E, 0x00, 0x04, 0x03, 0x02, 0x01};
const byte saveCmd[]   = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFD, 0x00, 0x04, 0x03, 0x02, 0x01};

// Efficient Memory Buffer
uint8_t frameBuf[2048];
int frameLen = 0;
unsigned long framesParsed = 0;
unsigned long calibrationUnlockTime = 0;
unsigned long lastDataRxTime = 0;

// Engine State (EMA smoothed)
float previousEnergiesDB[32] = {0};
float smoothedDistance[32] = {0};
float smoothedDB[32] = {0};
float smoothedDelta[32] = {0};
int activeFrames[32] = {0};
int maxActiveGate = 31; 

// --- Advanced Persistence Tracker ---
struct TargetTracker {
    float distance = 0.0;
    float delta = 0.0;
    float confidence = 0.0;
    bool isAlive = false;
    unsigned long lastSeen = 0;
};
TargetTracker primaryTarget;

// --- Diagnostic Helper ---
void sendDiag(String msg) {
    DynamicJsonDocument doc(512);
    doc["type"] = "diag";
    doc["msg"] = msg;
    String out; serializeJson(doc, out);
    webSocket.broadcastTXT(out);
    Serial.println("[DIAG] " + msg);
}

// Command to set parameters
void sendConfigCmd(uint16_t paramId, uint32_t value) {
    byte cmd[18] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x07, 0x00, 
                    (byte)(paramId & 0xFF), (byte)(paramId >> 8),
                    (byte)(value & 0xFF), (byte)((value >> 8) & 0xFF), (byte)((value >> 16) & 0xFF), (byte)((value >> 24) & 0xFF),
                    0x04, 0x03, 0x02, 0x01};
    Radar1.write(enableCmd, sizeof(enableCmd)); delay(50);
    Radar1.write(cmd, sizeof(cmd)); delay(50);
    Radar1.write(endCmd, sizeof(endCmd)); delay(50);
}

// --- Web UI ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Bio-Resonance Engine V14</title>
    <style>
        body { background-color: #050510; color: #00ffea; font-family: 'Courier New', Courier, monospace; margin: 0; overflow: hidden; }
        #app-container { display: flex; height: 100vh; overflow: hidden; }
        
        #sidebar { width: 300px; background: rgba(0, 255, 234, 0.05); border-right: 1px solid #00ffea; padding: 20px; display: flex; flex-direction: column; gap: 15px; box-sizing: border-box; overflow-y: auto;}
        #main-content { flex: 1; display: flex; flex-direction: column; padding: 20px; gap: 15px; overflow-y: auto; box-sizing: border-box;}
        
        h1 { font-size: 1.2rem; text-transform: uppercase; letter-spacing: 2px; margin: 0 0 10px 0; border-bottom: 1px solid #00ffea; width: 100%; padding-bottom: 5px;}
        h3 { font-size: 0.9rem; text-transform: uppercase; letter-spacing: 1px; color: #ff007f; margin: 0 0 5px 0;}
        
        .nav-btn { background: rgba(0,255,234,0.1); border: 1px solid #00ffea; color: #00ffea; padding: 10px; cursor: pointer; text-align: left; text-transform: uppercase; letter-spacing: 1px; font-weight: bold; transition: all 0.2s;}
        .nav-btn:hover { background: rgba(0,255,234,0.3); }
        .nav-btn.active { background: #00ffea; color: #000; box-shadow: 0 0 10px #00ffea;}
        
        .panel { background: rgba(0, 255, 234, 0.05); border: 1px solid #00ffea; border-radius: 8px; padding: 15px; box-shadow: 0 0 15px rgba(0,255,234,0.1); }
        .data-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 10px; width: 100%; height: 100%;}
        .data-box { border: 1px solid rgba(0,255,234,0.5); padding: 10px; text-align: center; border-radius: 5px; background: rgba(0,0,0,0.5); display: flex; flex-direction: column; justify-content: center; align-items: center; transition: border-color 0.3s;}
        .val { font-size: 1.4rem; display: block; margin-top: 5px; font-weight: bold; color: #fff; transition: color 0.3s;}
        
        button { background: rgba(0,255,234,0.1); border: 1px solid #00ffea; color: #00ffea; padding: 6px 12px; cursor: pointer; border-radius: 4px; font-weight: bold; transition: all 0.2s;}
        button:hover { background: #00ffea; color: #000; box-shadow: 0 0 10px #00ffea;}
        button.btn-danger { border-color: #ff007f; color: #ff007f; }
        button.btn-danger:hover { background: #ff007f; color: #fff; box-shadow: 0 0 10px #ff007f;}
        select, input[type=range] { background: #000; color: #00ffea; border: 1px solid #00ffea; padding: 5px; width: 100%; box-sizing: border-box;}
        label { font-size: 0.8rem; margin-bottom: 3px; display: block;}

        .heatmap-container { display: flex; width: 100%; height: 100%; background: #000; border: 1px solid #00ffea; position: relative;}
        canvas { width: 100%; height: 100%; display: block; }
    </style>
</head>
<body>
    <div id="app-container">
        <!-- SIDEBAR -->
        <div id="sidebar">
            <h1>RADAR ENGINE</h1>
            <div id="conn-status" style="font-weight:bold; text-align:center; padding:5px; background:#ff007f; color:#fff;">CONNECTING...</div>
            
            <div style="margin-top: 10px;">
                <h3>Operation Mode</h3>
                <div style="display:flex; flex-direction:column; gap:5px;">
                    <button class="nav-btn active" data-mode="telemetry">Raw Telemetry</button>
                    <button class="nav-btn" data-mode="sleep">Bio-Monitor</button>
                    <button class="nav-btn" data-mode="security">Zone Security</button>
                </div>
            </div>
            
            <div id="security-controls" style="display:none; margin-top:10px;">
                <h3>Security Settings</h3>
                <label>Tripwire Perimeter: <span id="tripwire-lbl">300</span>cm</label>
                <input type="range" id="tripwire-slider" min="70" max="1000" step="70" value="300">
            </div>

            <div style="margin-top: 10px;">
                <h3>Hardware Triggers</h3>
                <label style="margin-top:10px; display:flex; justify-content:space-between;">
                    <span>LED Brightness</span>
                    <span id="led-br-lbl">0%</span>
                </label>
                <div style="width:100%; background:#222; height:10px; border-radius:5px; overflow:hidden;">
                    <div id="led-br-bar" style="width:0%; height:100%; background:#00ffea; transition:width 0.1s;"></div>
                </div>
                
                <div style="display:flex; flex-wrap:wrap; gap:5px; margin-top: 10px;">
                    <button id="btn-led-mode" style="flex:1 1 100%;">LED Mode: DISTANCE</button>
                </div>
            </div>

            <div style="margin-top: 10px;">
                <h3>Radar Configuration</h3>
                <label>Max View Gate: <span id="range-label">31</span></label>
                <input type="range" id="range-slider" min="1" max="31" value="31">
                
                <label style="margin-top:10px;">Sensitivity (Noise Floor): <span id="noise-lbl">35</span>dB</label>
                <input type="range" id="noise-slider" min="10" max="80" value="35">

                <label style="margin-top:10px;">Hardware Max Dist: <span id="dist-lbl">10.0m</span></label>
                <input type="range" id="dist-slider" min="7" max="100" value="100">
                
                <label style="margin-top:10px;">Hardware Target Timeout: <span id="delay-lbl">30</span>s</label>
                <input type="range" id="delay-slider" min="0" max="300" value="30">
                
                <div style="display:flex; flex-wrap:wrap; gap:5px; margin-top: 15px;">
                    <button id="btn-save" style="flex:1;">Save Cfg</button>
                    <button id="btn-gain" style="flex:1;">Auto Gain</button>
                    <button class="btn-danger" id="btn-thresh" style="flex:1 1 100%;">Run AI Thresh</button>
                </div>
            </div>
            
            <div style="margin-top:auto; display:flex; flex-direction:column; height: 150px;">
                <h3>Diagnostic Console</h3>
                <div id="diag-console" style="flex:1; background: #000; padding: 5px; font-size: 0.7rem; border: 1px solid rgba(0,255,234,0.3); overflow-y:auto; color: #fff;">
                </div>
            </div>
        </div>
        
        <!-- MAIN CONTENT -->
        <div id="main-content">
            <div class="panel" id="mode-header-panel">
                <h1 id="mode-title" style="border:none; margin:0; padding:0; color:#fff; font-size:1.5rem;">RAW TELEMETRY MODE</h1>
                <div id="mode-status" style="font-size: 2rem; font-weight: bold; margin: 10px 0; color:#00ffea; transition: all 0.3s;">AWAITING DATA</div>
                <div id="mode-desc" style="color: rgba(255,255,255,0.7); font-size:0.9rem;">Extracting multi-target signatures from RF array.</div>
            </div>
            
            <div class="panel" id="metrics-panel" style="min-height: 120px;">
                <div class="data-grid" id="metrics-grid">
                    <div class="data-box" id="box-1">Slot 1<span class="val">--</span></div>
                    <div class="data-box" id="box-2">Slot 2<span class="val">--</span></div>
                    <div class="data-box" id="box-3">Slot 3<span class="val">--</span></div>
                    <div class="data-box" id="box-4">Slot 4<span class="val">--</span></div>
                    <div class="data-box" id="box-5">Slot 5<span class="val">--</span></div>
                </div>
            </div>

            <div class="panel" style="flex: 1; display:flex; flex-direction:column; padding:0; overflow:hidden;">
                <div style="padding: 10px; display:flex; justify-content:space-between; align-items:center; border-bottom: 1px solid rgba(0,255,234,0.3);">
                    <span style="font-weight:bold;">Sensing Visualizer</span>
                    <select id="view-mode" style="width: auto;">
                        <option value="1d">1D Position Tracker</option>
                        <option value="2d">2D Map</option>
                        <option value="waterfall">Waterfall Spectrogram</option>
                        <option value="bars">Live Energy (dB)</option>
                    </select>
                </div>
                <div style="flex:1; position:relative; background:#000;">
                     <canvas id="renderCanvas"></canvas>
                </div>
            </div>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('renderCanvas');
        const ctx = canvas.getContext('2d', { willReadFrequently: true });
        
        let appMode = 'telemetry';
        let noiseFloor = 35;
        let securityTripwire = 300;
        let currentViewMode = "1d";
        let activeGates = 31;
        
        // 2D Map Config
        let sensorA = { x: 50, y: 10 };
        let sensorB = { x: 350, y: 10 };
        let roomScaleCm = 500;
        let draggingSensor = null;
        
        let lastDataTime = Date.now();
        let currentData = { primaryDistance: 0, helperDistance: 0, targets: [] };
        let lerpedTargets = []; 
        
        let breathFrames = 0;
        let currentPhase = "STILL / HOLDING";

        function resizeCanvas() {
            canvas.width = canvas.parentElement.clientWidth;
            canvas.height = canvas.parentElement.clientHeight;
        }
        window.addEventListener('resize', resizeCanvas);
        resizeCanvas();

        function logDiag(msg) {
            const cons = document.getElementById('diag-console');
            const p = document.createElement('div');
            p.innerText = new Date().toLocaleTimeString() + " - " + msg;
            cons.appendChild(p);
            cons.scrollTop = cons.scrollHeight;
        }

        // Interactivity
        function getMousePos(evt) {
            let rect = canvas.getBoundingClientRect();
            return { x: (evt.clientX || (evt.touches && evt.touches[0].clientX)) - rect.left,
                     y: (evt.clientY || (evt.touches && evt.touches[0].clientY)) - rect.top };
        }
        
        function handleDown(e) {
            if (currentViewMode !== '2d') return;
            let pos = getMousePos(e);
            if (Math.hypot(pos.x - sensorA.x, pos.y - sensorA.y) < 30) draggingSensor = sensorA;
            else if (Math.hypot(pos.x - sensorB.x, pos.y - sensorB.y) < 30) draggingSensor = sensorB;
        }
        function handleMove(e) {
            if (draggingSensor) {
                let pos = getMousePos(e);
                draggingSensor.x = pos.x; draggingSensor.y = pos.y;
            }
        }
        function handleUp() { draggingSensor = null; }
        
        canvas.addEventListener('mousedown', handleDown);
        canvas.addEventListener('mousemove', handleMove);
        canvas.addEventListener('mouseup', handleUp);
        canvas.addEventListener('touchstart', handleDown);
        canvas.addEventListener('touchmove', handleMove);
        canvas.addEventListener('touchend', handleUp);

        // --- Event Listeners ---
        document.querySelectorAll('.nav-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                document.querySelectorAll('.nav-btn').forEach(b => b.classList.remove('active'));
                e.target.classList.add('active');
                appMode = e.target.getAttribute('data-mode');
                document.getElementById('mode-title').innerText = e.target.innerText + " MODE";
                document.getElementById('security-controls').style.display = appMode === 'security' ? 'block' : 'none';
            });
        });

        document.getElementById('btn-led-mode').addEventListener('click', (e) => {
            let newMode = e.target.innerText.includes("DISTANCE") ? "breathing" : "distance";
            if(websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send(JSON.stringify({ cmd: "set_led_mode", val: newMode }));
            }
        });

        document.getElementById('view-mode').addEventListener('change', (e) => { currentViewMode = e.target.value; });
        
        document.getElementById('range-slider').addEventListener('input', (e) => {
            activeGates = parseInt(e.target.value);
            document.getElementById('range-label').innerText = activeGates;
            if(websocket && websocket.readyState === WebSocket.OPEN) websocket.send(JSON.stringify({ cmd: "set_gate", val: activeGates }));
        });

        document.getElementById('noise-slider').addEventListener('input', (e) => {
            noiseFloor = parseInt(e.target.value);
            document.getElementById('noise-lbl').innerText = noiseFloor;
        });

        document.getElementById('tripwire-slider').addEventListener('input', (e) => {
            securityTripwire = parseInt(e.target.value);
            document.getElementById('tripwire-lbl').innerText = securityTripwire;
        });

        document.getElementById('dist-slider').addEventListener('change', (e) => {
            let val = parseInt(e.target.value);
            document.getElementById('dist-lbl').innerText = (val/10).toFixed(1) + "m";
            if(websocket && websocket.readyState === WebSocket.OPEN) websocket.send(JSON.stringify({ cmd: "set_max_dist", val: val }));
        });

        document.getElementById('delay-slider').addEventListener('change', (e) => {
            let val = parseInt(e.target.value);
            document.getElementById('delay-lbl').innerText = val;
            if(websocket && websocket.readyState === WebSocket.OPEN) websocket.send(JSON.stringify({ cmd: "set_delay", val: val }));
        });

        document.getElementById('btn-save').addEventListener('click', () => {
            if(websocket && websocket.readyState === WebSocket.OPEN) websocket.send(JSON.stringify({ cmd: "save_cfg" }));
        });
        document.getElementById('btn-gain').addEventListener('click', () => {
            if(websocket && websocket.readyState === WebSocket.OPEN) websocket.send(JSON.stringify({ cmd: "auto_gain" }));
        });
        document.getElementById('btn-thresh').addEventListener('click', () => {
            if(websocket && websocket.readyState === WebSocket.OPEN) websocket.send(JSON.stringify({ cmd: "auto_thresh" }));
        });

        function getHeatMapColor(dbValue) {
            let normalized = Math.max(0, Math.min(1, dbValue / 96.0));
            const hue = (1.0 - normalized) * 240; 
            const lightness = normalized > 0.8 ? 70 : (normalized > 0.2 ? 50 : 15);
            return `hsl(${hue}, 100%, ${lightness}%)`;
        }
        
        function setBox(id, title, valHTML, borderColor="#222", color="#aaa", subtitle="") {
            const el = document.getElementById(id);
            if(!el) return;
            el.style.borderColor = borderColor;
            el.innerHTML = `${title}<span class="val" style="color:${color}">${valHTML}</span>` + (subtitle ? `<span style="font-size:0.9rem; color:${borderColor}; margin-top:2px;">${subtitle}</span>` : "");
        }

        function updateDashboard(data, targets) {
            const statusEl = document.getElementById('mode-status');
            const descEl = document.getElementById('mode-desc');
            
            if (data.ledBrightness !== undefined) {
                let brPct = Math.round((data.ledBrightness / 255.0) * 100);
                document.getElementById('led-br-lbl').innerText = brPct + '%';
                document.getElementById('led-br-bar').style.width = brPct + '%';
                
                let btn = document.getElementById('btn-led-mode');
                if (data.ledMode === 'breathing') {
                    btn.innerText = "LED Mode: BREATHING";
                    btn.style.borderColor = "#ff007f";
                    btn.style.color = "#ff007f";
                    document.getElementById('led-br-bar').style.background = "#ff007f";
                } else {
                    btn.innerText = "LED Mode: DISTANCE";
                    btn.style.borderColor = "#00ffea";
                    btn.style.color = "#00ffea";
                    document.getElementById('led-br-bar').style.background = "#00ffea";
                }
            }

            if (appMode === 'telemetry') {
                statusEl.innerText = targets.length > 0 ? `${targets.length} TARGET(S) TRACKED` : "SECTOR CLEAR";
                statusEl.style.color = targets.length > 0 ? "#00ffea" : "#fff";
                
                let lockColor = data.primaryDistance > 0 ? '#ffeb3b' : '#555';
                setBox('box-1', 'MASTER LOCK', data.primaryDistance > 0 ? `${data.primaryDistance.toFixed(0)} cm` : '--', lockColor, lockColor, `Helper: ${data.helperDistance > 0 ? data.helperDistance.toFixed(0) : '--'} cm`);
                
                targets.sort((a,b) => a.distance - b.distance);
                for(let i=0; i<4; i++) {
                    if (i < targets.length) {
                        let t = targets[i];
                        let c = t.isMoving ? '#ff007f' : '#00ffea';
                        setBox(`box-${i+2}`, `Target ${i+1}`, `${t.distance.toFixed(0)} cm`, c, c, `${t.isMoving?'MOVING':'STATIC'} (&Delta;${t.delta.toFixed(1)})`);
                    } else {
                        setBox(`box-${i+2}`, `Slot ${i+2} Empty`, `--`, '#222', '#555');
                    }
                }
            } else if (appMode === 'sleep') {
                let closestStatic = targets.filter(t => !t.isMoving).sort((a,b) => a.distance - b.distance)[0];
                if (closestStatic) {
                    let d = closestStatic.delta;
                    if (Math.abs(d) < 0.2) { breathFrames++; if (breathFrames > 5) currentPhase = "STILL / HOLDING"; }
                    else if (d > 0.4) { currentPhase = "INHALING"; breathFrames = 0; }
                    else if (d < -0.4) { currentPhase = "EXHALING"; breathFrames = 0; }
                    setBox('box-1', 'Status', 'SLEEPING', '#00ffea', '#00ffea');
                    setBox('box-2', 'Subject Dist', `${closestStatic.distance.toFixed(0)} cm`, '#00ffea', '#fff');
                    setBox('box-3', 'Bio-Phase', currentPhase, '#00ffea', '#fff');
                    setBox('box-4', 'Micro-Delta', `${closestStatic.delta.toFixed(2)} dB`, '#00ffea', '#fff');
                    setBox('box-5', 'Slot Empty', '--', '#222', '#555');
                } else {
                    setBox('box-1', 'Status', 'EMPTY', '#555', '#555');
                }
            } else if (appMode === 'security') {
                let intruders = targets.filter(t => t.distance <= securityTripwire);
                if (intruders.length > 0) {
                    statusEl.innerText = "ALARM: ZONE BREACHED!";
                    statusEl.style.color = "#ff0000";
                    setBox('box-1', 'TRIPWIRE', `${securityTripwire} cm`, '#ff0000', '#ff0000');
                    for(let i=0; i<4; i++) {
                        if (i < intruders.length) setBox(`box-${i+2}`, `Intruder ${i+1}`, `${intruders[i].distance.toFixed(0)} cm`, '#ff0000', '#fff');
                        else setBox(`box-${i+2}`, `Slot Empty`, `--`, '#222', '#555');
                    }
                } else {
                    statusEl.innerText = "ZONE SECURE";
                    setBox('box-1', 'Tripwire Set', `${securityTripwire} cm`, '#555', '#fff');
                }
            }
        }

        function drawCanvas() {
            requestAnimationFrame(drawCanvas);
            let cw = canvas.width;
            let ch = canvas.height;
            ctx.fillStyle = '#000000';
            ctx.fillRect(0, 0, cw, ch);
            
        function drawCanvas() {
            if (!ctx) return;
            let cw = canvas.width;
            let ch = canvas.height;
            
            ctx.fillStyle = '#000000';
            ctx.fillRect(0, 0, cw, ch);
            
            let isFrozen = Date.now() - lastDataTime > 2000;
            
            if (isFrozen || !currentData) {
                ctx.fillStyle = 'rgba(0,0,0,0.2)';
                ctx.fillRect(0,0,cw,ch);
                ctx.fillStyle = '#ff007f';
                ctx.font = "bold 16px Courier";
                ctx.fillText("RADAR CALIBRATING...", 10, 20);
                requestAnimationFrame(drawCanvas);
                return;
            }

            let energiesDB = currentData.energiesDB;
            let targets = (currentData.targets || []).filter(t => t.db >= noiseFloor);
            
            if (currentViewMode === "1d") {
                // Draw center line
                ctx.strokeStyle = 'rgba(0, 255, 234, 0.2)';
                ctx.lineWidth = 1;
                ctx.beginPath(); ctx.moveTo(0, ch / 2); ctx.lineTo(cw, ch / 2); ctx.stroke();
                
                let maxDistCm = activeGates * 70;
                
                // Draw security tripwire
                if (appMode === 'security') {
                    let tripX = (securityTripwire / maxDistCm) * cw;
                    if(tripX <= cw) {
                        ctx.fillStyle = 'rgba(255,0,0,0.1)';
                        ctx.fillRect(0, 0, tripX, ch);
                        ctx.strokeStyle = '#ff0000';
                        ctx.lineWidth = 2;
                        ctx.beginPath(); ctx.moveTo(tripX, 0); ctx.lineTo(tripX, ch); ctx.stroke();
                    }
                }
                
                // Highlight Hardware Lock
                if (currentData.primaryDistance > 0 && currentData.primaryDistance <= maxDistCm) {
                    if (window.lockPhys === undefined) window.lockPhys = { dist: currentData.primaryDistance, vel: 0 };
                    
                    let force = (currentData.primaryDistance - window.lockPhys.dist) * 0.05;
                    window.lockPhys.vel = (window.lockPhys.vel + force) * 0.8;
                    window.lockPhys.dist += window.lockPhys.vel;

                    let primX = (window.lockPhys.dist / maxDistCm) * cw;
                    ctx.beginPath(); ctx.arc(primX, ch / 2, 25, 0, 2 * Math.PI, false);
                    ctx.fillStyle = 'rgba(255, 235, 59, 0.1)'; ctx.fill();
                    ctx.lineWidth = 2; ctx.strokeStyle = '#ffeb3b';
                    ctx.setLineDash([5, 5]); ctx.stroke(); ctx.setLineDash([]);
                    ctx.fillStyle = '#ffeb3b'; ctx.font = "bold 14px Courier";
                    ctx.fillText("LOCKED", primX - 25, ch/2 - 30);
                }

                // Plot lerped targets with Spring Physics
                let newLerped = [];
                targets.forEach(t => {
                    let existing = lerpedTargets.find(lt => lt.gate === t.gate);
                    if (existing) {
                        if (existing.velocity === undefined) existing.velocity = 0;
                        let force = (t.distance - existing.distance) * 0.05;
                        existing.velocity = (existing.velocity + force) * 0.8;
                        existing.distance += existing.velocity;
                        existing.db = t.db;
                        existing.isMoving = t.isMoving;
                        newLerped.push(existing);
                    } else {
                        newLerped.push({...t, velocity: 0});
                    }
                });
                lerpedTargets = newLerped;
                
                lerpedTargets.forEach(t => {
                    let targetX = (t.distance / maxDistCm) * cw;
                    if (targetX > cw) targetX = cw;
                    let isIntruder = appMode === 'security' && t.distance <= securityTripwire;
                    let color = isIntruder ? '#ff0000' : (t.isMoving ? '#ff007f' : '#00ffea');
                    
                    ctx.beginPath(); ctx.arc(targetX, ch / 2, 6, 0, 2 * Math.PI, false);
                    ctx.fillStyle = color; ctx.fill();
                    ctx.lineWidth = 2; ctx.strokeStyle = '#fff'; ctx.stroke();

                    ctx.beginPath(); ctx.arc(targetX, ch / 2, 16, 0, 2 * Math.PI, false);
                    ctx.fillStyle = isIntruder ? 'rgba(255,0,0,0.3)' : (t.isMoving ? 'rgba(255,0,127,0.3)' : 'rgba(0,255,234,0.3)');
                    ctx.fill();
                    
                    ctx.fillStyle = '#fff'; ctx.font = "12px Courier";
                    ctx.fillText(`${t.distance.toFixed(0)}cm`, targetX - 15, ch/2 + 35);
                });
                
            } else if (currentViewMode === "2d") {
                // Trilateration visualization
                ctx.strokeStyle = 'rgba(255,255,255,0.05)';
                ctx.lineWidth = 1;
                for(let i=0; i<cw; i+=50) { ctx.beginPath(); ctx.moveTo(i,0); ctx.lineTo(i,ch); ctx.stroke(); }
                for(let i=0; i<ch; i+=50) { ctx.beginPath(); ctx.moveTo(0,i); ctx.lineTo(cw,i); ctx.stroke(); }
                
                ctx.strokeStyle = '#333';
                ctx.beginPath(); ctx.arc(sensorA.x, sensorA.y, (currentData.primaryDistance/roomScaleCm)*cw, 0, 2*Math.PI); ctx.stroke();
                ctx.beginPath(); ctx.arc(sensorB.x, sensorB.y, (currentData.helperDistance/roomScaleCm)*cw, 0, 2*Math.PI); ctx.stroke();
                
                ctx.fillStyle = '#00ffea'; ctx.beginPath(); ctx.arc(sensorA.x, sensorA.y, 10, 0, 2*Math.PI); ctx.fill();
                ctx.fillStyle = '#fff'; ctx.fillText("A", sensorA.x-4, sensorA.y+4);
                
                ctx.fillStyle = '#ff007f'; ctx.beginPath(); ctx.arc(sensorB.x, sensorB.y, 10, 0, 2*Math.PI); ctx.fill();
                ctx.fillStyle = '#fff'; ctx.fillText("B", sensorB.x-4, sensorB.y+4);

                let rA = (currentData.primaryDistance/roomScaleCm)*cw;
                let rB = (currentData.helperDistance/roomScaleCm)*cw;
                
                if (rA > 0 && rB > 0) {
                    let d = Math.hypot(sensorB.x - sensorA.x, sensorB.y - sensorA.y);
                    let target = null;
                    if (d < rA + rB && d > Math.abs(rA - rB) && d !== 0) {
                        let a = (rA*rA - rB*rB + d*d) / (2*d);
                        let h = Math.sqrt(rA*rA - a*a);
                        let px = sensorA.x + a * (sensorB.x - sensorA.x) / d;
                        let py = sensorA.y + a * (sensorB.y - sensorA.y) / d;
                        let rx = -h * (sensorB.y - sensorA.y) / d;
                        let ry = h * (sensorB.x - sensorA.x) / d;
                        
                        let pt1 = { x: px + rx, y: py + ry };
                        let pt2 = { x: px - rx, y: py - ry };
                        target = Math.hypot(pt1.x - cw/2, pt1.y - ch/2) < Math.hypot(pt2.x - cw/2, pt2.y - ch/2) ? pt1 : pt2;
                    } else if (d !== 0) {
                        // Best Effort Intersection
                        let vX = (sensorB.x - sensorA.x) / d;
                        let vY = (sensorB.y - sensorA.y) / d;
                        let pA = { x: sensorA.x + vX * rA, y: sensorA.y + vY * rA };
                        let pB = { x: sensorB.x - vX * rB, y: sensorB.y - vY * rB };
                        target = { x: (pA.x + pB.x) / 2, y: (pA.y + pB.y) / 2 };
                    }
                    
                    if (target !== null) {
                        if (window.target2D === undefined) window.target2D = {x: target.x, y: target.y, vx: 0, vy: 0};
                        window.target2D.vx = (window.target2D.vx + (target.x - window.target2D.x) * 0.02) * 0.9;
                        window.target2D.vy = (window.target2D.vy + (target.y - window.target2D.y) * 0.02) * 0.9;
                        window.target2D.x += window.target2D.vx;
                        window.target2D.y += window.target2D.vy;
                        
                        ctx.beginPath(); ctx.arc(window.target2D.x, window.target2D.y, 25, 0, 2*Math.PI);
                        ctx.fillStyle = 'rgba(255,235,59,0.3)'; ctx.fill();
                        ctx.lineWidth = 2; ctx.strokeStyle = '#ffeb3b'; ctx.stroke();
                        ctx.fillStyle = '#fff'; ctx.fillText("2D LOCK", window.target2D.x - 28, window.target2D.y + 40);
                    }
                }
                ctx.fillStyle = 'rgba(255,255,255,0.5)';
                ctx.fillText("Drag Sensor A (Master) and B (Helper) to match your physical room layout.", 10, ch - 20);
                
            } else if (currentViewMode === "waterfall") {
                let sliceHeight = ch / (activeGates + 1);
                for (let i = 0; i <= activeGates; i++) {
                    ctx.fillStyle = getHeatMapColor(currentData.energiesDB ? currentData.energiesDB[i] : 0);
                    ctx.fillRect(cw - 5, ch - ((i+1)*sliceHeight), 5, Math.ceil(sliceHeight));
                }
                const imgData = ctx.getImageData(5, 0, cw - 5, ch);
                ctx.putImageData(imgData, 0, 0);
            } else if (currentViewMode === "bars") {
                let barWidth = (cw / (activeGates + 1)) - 2;
                for (let i = 0; i <= activeGates; i++) {
                    let normalized = Math.max(0, Math.min(1, (currentData.energiesDB ? currentData.energiesDB[i] : 0) / 96.0));
                    let h = normalized * ch;
                    let grad = ctx.createLinearGradient(0, ch - h, 0, ch);
                    grad.addColorStop(0, '#ff007f');
                    grad.addColorStop(1, '#00ffea');
                    ctx.fillStyle = grad;
                    ctx.fillRect(i * (barWidth + 2), ch - h, barWidth, h);
                }
            }
            
            requestAnimationFrame(drawCanvas);
        }

        var gateway = `ws://${window.location.hostname}:81/`;
        var websocket;
        
        function initWebSocket() {
            websocket = new WebSocket(gateway);
            const statusEl = document.getElementById('conn-status');
            websocket.onopen = function() {
                statusEl.innerText = "CONNECTED";
                statusEl.style.background = "#00ffea";
                statusEl.style.color = "#000";
                logDiag("WebSocket connected.");
            };
            websocket.onmessage = function(event) {
                try {
                    var data = JSON.parse(event.data);
                    if(data.type === "diag") { logDiag(data.msg); return; }
                    currentData = data;
                    let targets = (data.targets || []).filter(t => t.db >= noiseFloor);
                    updateDashboard(data, targets);
                } catch (err) {}
            };
            websocket.onclose = function(event){ 
                statusEl.innerText = "DISCONNECTED";
                statusEl.style.background = "#ff007f";
                statusEl.style.color = "#fff";
                logDiag("WebSocket disconnected. Retrying...");
                setTimeout(initWebSocket, 2000); 
            };
        }

        window.onload = function() {
            initWebSocket();
            requestAnimationFrame(drawCanvas);
        };
    </script>
</body>
</html>
)rawliteral";

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if(type == WStype_TEXT) {
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, payload);
        if (!error) {
            String cmd = doc["cmd"];
            if (cmd == "set_gate") {
                maxActiveGate = doc["val"];
            } 
            else if (cmd == "set_max_dist") {
                sendConfigCmd(0x0001, doc["val"].as<uint32_t>());
                sendDiag("Set Max Distance to " + String(doc["val"].as<float>()/10.0) + "m");
            }
            else if (cmd == "set_delay") {
                sendConfigCmd(0x0004, doc["val"].as<uint32_t>());
                sendDiag("Set Target Disappearance Delay to " + String(doc["val"].as<int>()) + "s");
            }
            else if (cmd == "set_led_mode") {
                if (doc["val"] == "breathing") currentLedMode = LED_MODE_BREATHING;
                else currentLedMode = LED_MODE_DISTANCE;
            }
            else if (cmd == "save_cfg") {
                Radar1.write(enableCmd, sizeof(enableCmd)); delay(50);
                Radar1.write(saveCmd, sizeof(saveCmd)); delay(50);
                Radar1.write(endCmd, sizeof(endCmd)); delay(50);
                sendDiag("Requested Save Configuration.");
            }
            else if (cmd == "auto_gain") {
                Radar1.write(enableCmd, sizeof(enableCmd)); delay(50);
                Radar1.write(autoGainCmd, sizeof(autoGainCmd)); delay(50);
                calibrationUnlockTime = millis() + 5000;
                sendDiag("Requested Auto Gain. Hardware calibrating...");
            } 
            else if (cmd == "auto_thresh") {
                Radar1.write(enableCmd, sizeof(enableCmd)); delay(50);
                Radar1.write(autoThreshCmd, sizeof(autoThreshCmd)); delay(50);
                calibrationUnlockTime = millis() + 16000;
                sendDiag("Requested Auto Threshold. Step away for 15s...");
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Radar1.begin(115200, SERIAL_8N1, 16, 17); 
    
    Serial.println("\n--- BOOTING MASTER ENGINE V14 ---");

    // Initialize FastLED
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    leds[0] = CRGB::Black;
    FastLED.show();

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println(String("\nConnected! IP: ") + WiFi.localIP().toString());
    
    // Init OTA
    ArduinoOTA.setHostname("radar-master");
    ArduinoOTA.begin();

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
    } else {
        esp_now_register_recv_cb(OnDataRecv);
    }

    server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", index_html); });
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    server.begin();

    Serial.println("\n[SYSTEM] Waiting 3 seconds for radar hardware boot...");
    delay(3000); 

    bool radarReady = false;
    int attempt = 1;

    while(!radarReady) {
        if (attempt > 5) {
            Serial.println("\n[WARNING] Handshake failed 5 times. Bypassing to prevent boot-loop. OTA active.");
            break; 
        }
        Serial.printf("\n[HANDSHAKE] Attempt %d to force Engineering Mode...\n", attempt);
        
        while(Radar1.available()) Radar1.read(); 
        
        Radar1.write(enableCmd, sizeof(enableCmd)); delay(100);
        Radar1.write(engModeCmd, sizeof(engModeCmd)); delay(100);
        Radar1.write(endCmd, sizeof(endCmd)); delay(100);
        
        unsigned long t = millis();
        bool receivingText = false;
        
        Serial.println("[HANDSHAKE] Listening for response...");
        while(millis() - t < 1500) {
            if(Radar1.available()) {
                uint8_t b = Radar1.read();
                if(b == 0xF4) {
                    radarReady = true;
                    Serial.println("[SUCCESS] RADAR IS NOW LOCKED IN BINARY MODE!");
                    break;
                }
                if (b == 0x64) receivingText = true;
            }
        }
        
        if (!radarReady) {
            Serial.println("[FAILURE] Radar ignored the command.");
            if (receivingText) {
                Serial.println("[ERROR] Radar is still sending TEXT.");
            }
            delay(1500);
            attempt++;
        }
    }
    
    Serial.println("[SYSTEM] Pushing hardware stabilization commands...");
    Radar1.write(enableCmd, sizeof(enableCmd)); delay(50);
    Radar1.write(autoGainCmd, sizeof(autoGainCmd)); delay(50);
    Radar1.write(endCmd, sizeof(endCmd)); delay(50);
    sendConfigCmd(0x0001, 35);
    sendConfigCmd(0x0004, 5);
    Radar1.write(enableCmd, sizeof(enableCmd)); delay(50);
    Radar1.write(saveCmd, sizeof(saveCmd)); delay(50);
    Radar1.write(endCmd, sizeof(endCmd)); delay(50);
    Serial.println("[SYSTEM] Hardware optimization complete!");

    lastDataRxTime = millis();
}

float currentPrimaryDistance = 0.0;

void processExtractedPayload(uint8_t* payload, int len) {
    uint8_t state = payload[0]; 
    uint16_t distance = payload[1] | (payload[2] << 8); 
    currentPrimaryDistance = distance;
    
    float currentEnergiesDB[32];
    
    int energyStartIdx = 3; 
    for(int i = 0; i < 32; i++) {
        int offset = energyStartIdx + (i * 4);
        uint32_t rawEnergy = payload[offset] | (payload[offset+1] << 8) | 
                             (payload[offset+2] << 16) | (payload[offset+3] << 24);
        
        float dbVal = (rawEnergy > 0) ? (10.0 * log10((float)rawEnergy)) : 0.0;
        currentEnergiesDB[i] = round(dbVal * 10.0) / 10.0;
    }

    DynamicJsonDocument doc(4096);
    doc["type"] = "data";
    doc["frames"] = framesParsed++;
    doc["primaryState"] = state;
    doc["primaryDistance"] = distance;
    
    JsonArray targetsArr = doc.createNestedArray("targets");
    float closestDist = 9999.0;
    float closestDelta = 0.0;
    
    for (int i = 0; i <= maxActiveGate; i++) {
        // HARDWARE CLUTTER FILTER: Ignore Gate 0 (0cm) and Gate 1 (70cm) ghosts completely
        if (i < 2) {
            activeFrames[i] = 0;
            continue;
        }

        bool isPeak = true;
        if (i > 0 && currentEnergiesDB[i] < currentEnergiesDB[i-1]) isPeak = false;
        if (i < 31 && currentEnergiesDB[i] < currentEnergiesDB[i+1]) isPeak = false;
        if (i > 0 && currentEnergiesDB[i] == currentEnergiesDB[i-1]) isPeak = false; 
        
        if (isPeak && currentEnergiesDB[i] > 10.0) { 
            // Sub-bin Parabolic Interpolation to unlock from "exact 70 multiples"
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
                float alpha = 0.3; // Distance & DB smoothing
                smoothedDistance[i] = (alpha * rawDist) + ((1.0 - alpha) * smoothedDistance[i]);
                smoothedDB[i] = (alpha * rawDb) + ((1.0 - alpha) * smoothedDB[i]);
                float deltaAlpha = 0.1; // Delta smoothing (more stable for zero-crossing)
                smoothedDelta[i] = (deltaAlpha * rawDelta) + ((1.0 - deltaAlpha) * smoothedDelta[i]);
            }
            activeFrames[i]++;

            JsonObject target = targetsArr.createNestedObject();
            target["gate"] = i;
            target["distance"] = smoothedDistance[i];
            target["db"] = smoothedDB[i];
            target["delta"] = smoothedDelta[i];
            target["isMoving"] = abs(smoothedDelta[i]) > 1.0;
            
            if (smoothedDistance[i] <= 350.0 && smoothedDistance[i] < closestDist) {
                closestDist = smoothedDistance[i];
                closestDelta = smoothedDelta[i];
            }
        } else {
            activeFrames[i] = 0;
        }
    }

    if (closestDist <= 350.0) {
        primaryTarget.confidence += 20.0;
        if (primaryTarget.confidence > 100.0) primaryTarget.confidence = 100.0;
        if (!primaryTarget.isAlive) {
            primaryTarget.distance = closestDist;
            primaryTarget.isAlive = true;
        } else {
            primaryTarget.distance = (0.3 * closestDist) + (0.7 * primaryTarget.distance);
        }
        primaryTarget.delta = (0.2 * closestDelta) + (0.8 * primaryTarget.delta);
        primaryTarget.lastSeen = millis();
    } else {
        primaryTarget.confidence -= 2.0; 
        if (primaryTarget.confidence <= 0.0 || (millis() - primaryTarget.lastSeen > 4000)) {
            primaryTarget.confidence = 0.0;
            primaryTarget.isAlive = false;
        }
    }

    doc["primaryDistance"] = primaryTarget.isAlive ? primaryTarget.distance : 0.0;
    doc["primaryDelta"] = primaryTarget.isAlive ? primaryTarget.delta : 0.0;
    
    if (millis() - lastHelperRxTime > 4000) {
        helperDist = 0.0;
        helperDelta = 0.0;
    }
    doc["helperDistance"] = helperDist;
    doc["helperDelta"] = helperDelta;

    doc["ledBrightness"] = currentLedBrightness;
    doc["ledMode"] = currentLedMode == LED_MODE_DISTANCE ? "distance" : "breathing";

    JsonArray energiesArr = doc.createNestedArray("energiesDB");
    for(int i = 0; i <= maxActiveGate; i++) {
        energiesArr.add(currentEnergiesDB[i]);
    }
    
    for(int i=0; i<32; i++) previousEnergiesDB[i] = currentEnergiesDB[i];

    lastDataRxTime = millis();

    String output;
    serializeJson(doc, output);
    webSocket.broadcastTXT(output);
}

void loop() {
    ArduinoOTA.handle();
    server.handleClient();
    webSocket.loop();
    
    // Watchdog to wake radar if it gets stuck in configuration mode
    if (millis() - lastDataRxTime > 3000 && millis() > calibrationUnlockTime) {
        lastDataRxTime = millis(); // Reset watchdog
        Radar1.write(endCmd, sizeof(endCmd));
        sendDiag("[WATCHDOG] No data received. Forcing exit config mode...");
    }

    // LED State Machine
    if (millis() - lastLedUpdate > 20) {
        lastLedUpdate = millis();
        
        static float currentLedBrightnessF = 0.0; // Float for ultra-smooth organic easing
        
        float closestDist = primaryTarget.distance;
        float closestDelta = primaryTarget.delta;
        
        // Merge Helper distance if it is tracking a closer target
        if (helperDist > 0.0 && (!primaryTarget.isAlive || helperDist < closestDist)) {
            closestDist = helperDist;
            closestDelta = helperDelta;
        }
        
        bool targetFound = (closestDist > 0.0 && closestDist <= 300.0);
        
        if (currentLedMode == LED_MODE_DISTANCE) {
            if (targetFound) {
                if (closestDist <= 60.0) {
                    danceHue += 10;
                    targetLedBrightness = 255;
                } else {
                    float normalized = (300.0 - closestDist) / 240.0;
                    if (normalized < 0.0) normalized = 0.0;
                    if (normalized > 1.0) normalized = 1.0;
                    targetLedBrightness = (int)(255.0 * (normalized * normalized));
                }
            } else {
                targetLedBrightness = 0;
            }
            
            float diff = targetLedBrightness - currentLedBrightnessF;
            if (abs(diff) > 0.5) {
                currentLedBrightnessF += (diff * 0.02); 
            } else {
                currentLedBrightnessF = targetLedBrightness;
            }
            currentLedBrightness = (int)currentLedBrightnessF;
            
            if (currentLedBrightness == 0) {
                leds[0] = CRGB::Black;
            } else {
                if (targetFound && closestDist <= 60.0) {
                    leds[0] = CHSV(danceHue, 255, currentLedBrightness);
                } else {
                    leds[0] = CHSV(128, 255, currentLedBrightness); 
                }
            }
            
        } else if (currentLedMode == LED_MODE_BREATHING) {
            if (targetFound) {
                int mappedB = 127 + (closestDelta * 500.0);
                if (mappedB > 255) mappedB = 255;
                if (mappedB < 10) mappedB = 10; 
                
                targetLedBrightness = mappedB;
                currentLedBrightnessF += (targetLedBrightness - currentLedBrightnessF) * 0.1;
                currentLedBrightness = (int)currentLedBrightnessF;
                leds[0] = CHSV(192, 255, currentLedBrightness); 
            } else {
                targetLedBrightness = 0;
                currentLedBrightnessF += (0.0 - currentLedBrightnessF) * 0.05; 
                currentLedBrightness = (int)currentLedBrightnessF;
                if (currentLedBrightness == 0) {
                    leds[0] = CRGB::Black;
                } else {
                    leds[0] = CHSV(192, 255, currentLedBrightness);
                }
            }
        }
        FastLED.show();
    }

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

        int cmdHeaderIdx = -1;
        for (int i = 0; i <= frameLen - 4; i++) {
            if (frameBuf[i] == 0xFD && frameBuf[i+1] == 0xFC && frameBuf[i+2] == 0xFB && frameBuf[i+3] == 0xFA) {
                cmdHeaderIdx = i;
                break;
            }
        }

        if (headerIdx == -1 && cmdHeaderIdx == -1) {
            if (frameLen > 3) {
                frameBuf[0] = frameBuf[frameLen-3];
                frameBuf[1] = frameBuf[frameLen-2];
                frameBuf[2] = frameBuf[frameLen-1];
                frameLen = 3;
            }
            continue;
        }

        bool isCmdFrame = false;
        int activeHeaderIdx = headerIdx;
        if (cmdHeaderIdx != -1) {
            if (headerIdx == -1 || cmdHeaderIdx < headerIdx) {
                activeHeaderIdx = cmdHeaderIdx;
                isCmdFrame = true;
            }
        }

        if (activeHeaderIdx > 0) {
            int remaining = frameLen - activeHeaderIdx;
            memmove(frameBuf, frameBuf + activeHeaderIdx, remaining);
            frameLen = remaining;
            activeHeaderIdx = 0;
            frameFound = true;
            continue;
        }

        int tailIdx = -1;
        
        if (!isCmdFrame) {
            for (int i = 6; i <= frameLen - 4; i++) {
                if (frameBuf[i] == 0xF8 && frameBuf[i+1] == 0xF7 && frameBuf[i+2] == 0xF6 && frameBuf[i+3] == 0xF5) {
                    tailIdx = i;
                    break;
                }
            }
        } else {
            if (frameLen >= 6) {
                int cmdLen = frameBuf[4] | (frameBuf[5] << 8);
                int expectedTailIdx = 6 + cmdLen;
                if (expectedTailIdx + 4 <= frameLen) {
                    if (frameBuf[expectedTailIdx] == 0x04 && frameBuf[expectedTailIdx+1] == 0x03 && 
                        frameBuf[expectedTailIdx+2] == 0x02 && frameBuf[expectedTailIdx+3] == 0x01) {
                        tailIdx = expectedTailIdx;
                    }
                }
            }
        }

        if (tailIdx != -1) {
            if (!isCmdFrame) {
                int payloadStart = 6;
                int payloadLen = tailIdx - payloadStart;
                if (payloadLen > 0 && payloadLen < 250) {
                    processExtractedPayload(&frameBuf[payloadStart], payloadLen);
                }
            } else {
                if (frameLen >= 10) {
                    uint16_t cmdWord = frameBuf[6] | (frameBuf[7] << 8);
                    if (cmdWord == 0x0114) { 
                        uint8_t status = frameBuf[8]; 
                        if (status == 0x01) sendDiag("WARNING: Power Supply Interference Detected!");
                    } else if (cmdWord == 0x01FF) {
                        // sendDiag("Config Mode Enable ACK.");
                    } else if (cmdWord == 0x0107) {
                        sendDiag("Parameter Set ACK Received.");
                    } else if (cmdWord == 0x01FD) {
                        sendDiag("Configuration Save ACK Received.");
                    } else if (cmdWord == 0x01EE) {
                        sendDiag("Auto Gain Settings Updated.");
                    }
                }
            }
            
            int bytesToRemove = tailIdx + 4;
            int remaining = frameLen - bytesToRemove;
            if (remaining > 0) memmove(frameBuf, frameBuf + bytesToRemove, remaining);
            frameLen = remaining;
            frameFound = true; 
        }
    }
}