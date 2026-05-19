/*
 * 【科學實驗室專業溫度監測儀 - V13.2 旗艦體驗版】
 * 更新：
 * 1. [UI升級] USB 權限警告視窗改為自訂的精美 Modal 介面。
 * 2. [UX優化] 警告視窗內的設定網址與 IP 支援「點擊一鍵反白全選」，方便複製貼上。
 * 3. [底層穩定] 保留 OLED 實體螢幕顯示、1Hz 穩定取樣與雙模獨立架構。
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================================================================
// 【硬體與系統設定】
// ================================================================
const char* HOST_NAME = "temp";         
const char* AP_NAME   = "TEMP_MONITOR"; 
const int   RED_PIN   = 3;   
const int   BLUE_PIN  = 4;   
const int   RGB_LED_PIN = 21; 
const int   WDT_TIMEOUT = 8000; 

// --- OLED 設定 ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define I2C_SDA       8
#define I2C_SCL       7

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel pixels(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
WebServer server(80);
WiFiManager wm;
OneWire oneWireRed(RED_PIN);
OneWire oneWireBlue(BLUE_PIN);
DallasTemperature sensorRed(&oneWireRed);
DallasTemperature sensorBlue(&oneWireBlue);

String latestDataJson = "{\"ts\":0.0,\"t1\":null,\"e1\":1,\"t2\":null,\"e2\":1,\"off\":0.0}";
SemaphoreHandle_t jsonMutex; 
bool isDrawing = false;      
unsigned long lastWiFiCheck = 0;
unsigned long lastOledUpdate = 0;
unsigned long runStartTime = 0; 

float filteredT1 = -999.0, filteredT2 = -999.0;
float calibrationOffset = 0.0; 
const float alpha = 0.3;       

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b)); pixels.show();
}

// ================================================================
// 【OLED 繪圖函數：WiFi 設定提示模式】
// ================================================================
void configModeCallback(WiFiManager *myWiFiManager) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(16, 5); display.print("== WiFi Setup ==");
  display.setCursor(0, 25); display.print("1. Connect Phone to:");
  display.setCursor(0, 37); display.print("   "); 
  display.print(myWiFiManager->getConfigPortalSSID());
  display.setCursor(0, 52); display.print("2. Go to 192.168.4.1");
  display.display();
  setRGB(255, 128, 0); 
}

// ================================================================
// 【OLED 繪圖函數：正常測量模式】
// ================================================================
void updateOLED() {
  float d_t1, d_t2;
  xSemaphoreTake(jsonMutex, portMAX_DELAY);
  d_t1 = filteredT1; d_t2 = filteredT2;
  xSemaphoreGive(jsonMutex);

  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1; uint16_t w, h;

  display.setTextSize(1);
  String title = String(HOST_NAME) + ".local";
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 2); display.print(title);
  display.drawLine(0, 13, SCREEN_WIDTH, 13, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(18, 20); display.print("RED");
  display.setCursor(82, 20); display.print("BLUE");

  display.setTextSize(2);
  String s1 = (d_t1 < -50) ? "NC" : String(d_t1, 1);
  String s2 = (d_t2 < -50) ? "NC" : String(d_t2, 1);
  display.getTextBounds(s1, 0, 0, &x1, &y1, &w, &h); display.setCursor(32 - (w / 2), 32); display.print(s1);
  display.getTextBounds(s2, 0, 0, &x1, &y1, &w, &h); display.setCursor(96 - (w / 2), 32); display.print(s2);

  display.drawLine(0, 52, SCREEN_WIDTH, 52, SSD1306_WHITE);
  display.setTextSize(1);
  String ipStr = "IP: " + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "192.168.4.1");
  display.getTextBounds(ipStr, 0, 0, &x1, &y1, &w, &h); display.setCursor((SCREEN_WIDTH - w) / 2, 56); display.print(ipStr);
  display.display();
}

// ================================================================
// 【核心取樣任務：1Hz】
// ================================================================
void TaskSampling(void * pvParameters) {
  sensorRed.begin(); sensorBlue.begin();
  sensorRed.setResolution(12); sensorBlue.setResolution(12);
  sensorRed.setWaitForConversion(false); sensorBlue.setWaitForConversion(false);
  
  for(;;) {
    sensorRed.requestTemperatures(); sensorBlue.requestTemperatures();
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    float rawT1 = sensorRed.getTempCByIndex(0);
    float rawT2 = sensorBlue.getTempCByIndex(0);

    auto processTemp = [](float raw, float &filtered, float offset) -> int {
      if (raw <= -126.0) { filtered = -999.0; return 1; } 
      if (raw < -55.0 || raw > 125.0) return 2; 
      float calVal = raw + offset;
      if (filtered == -999.0) filtered = calVal; 
      else filtered = (alpha * calVal) + (1.0 - alpha) * filtered; 
      return 0;
    };

    if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      int e1 = processTemp(rawT1, filteredT1, 0); 
      int e2 = processTemp(rawT2, filteredT2, calibrationOffset); 
      float ts = isDrawing ? (millis() - runStartTime) / 1000.0 : 0.0;
      latestDataJson = "{\"ts\":" + String(ts,1) + ",\"t1\":" + (e1==0?String(filteredT1,2):"null") + ",\"e1\":" + String(e1) + ",\"t2\":" + (e2==0?String(filteredT2,2):"null") + ",\"e2\":" + String(e2) + ",\"off\":" + String(calibrationOffset,2) + "}";
      String serialOut = String(ts, 1) + "," + (e1==0?String(filteredT1,2):"null") + "," + (e2==0?String(filteredT2,2):"null");
      Serial.println(serialOut);
      xSemaphoreGive(jsonMutex);
    }
  }
}

// ================================================================
// 【頁面一：WiFi 無線版】
// ================================================================
const char wifi_html[] PROGMEM = R"=====(
<!DOCTYPE HTML><html lang="zh-TW">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>專業實驗儀器 - WiFi版</title>
  <style>
    * { box-sizing: border-box; }
    body, html { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; font-family: sans-serif; background: #eef2f5; }
    .main-wrapper { display: flex; flex-direction: column; height: 100vh; padding: 10px; }
    .header { display: flex; justify-content: space-between; align-items: center; height: 16px; font-size: 13px; color: #555; font-weight: bold;}
    .value-container { display: flex; gap: 10px; margin: 8px 0; flex: 0 0 auto; }
    .v-card { flex: 1; background: #fff; border-radius: 10px; padding: 10px; text-align: center; box-shadow: 0 4px 6px rgba(0,0,0,0.05); }
    .v-red { border: 4px solid #d32f2f; color: #c62828; }
    .v-blue { border: 4px solid #1976d2; color: #1565c0; }
    .v-val { font-size: 36px; font-weight: bold; font-family: monospace; display: block; margin: 2px 0; }
    .v-label { font-size: 13px; color: #333; font-weight: bold; }
    .canvas-wrapper { flex: 1; position: relative; margin: 5px 0; background: #fff; border: 1px solid #d1d5db; border-radius: 10px; min-height: 0; box-shadow: 0 4px 6px rgba(0,0,0,0.05); overflow: hidden;}
    canvas { width: 100% !important; height: 100% !important; display: block; }
    .footer { flex: 0 0 auto; display: flex; flex-direction: row; gap: 6px; padding-top: 5px; }
    .btn { flex: 1; padding: 12px 2px; border-radius: 8px; cursor: pointer; border: none; font-weight: bold; transition: 0.2s; white-space: nowrap; color: #fff; display:flex; justify-content:center; align-items:center; font-size: 14px;}
    .btn:active { transform: scale(0.96); }
    .btn-start { flex: 2; background: #2ecc71; font-size: 15px;}
    .btn-stop { flex: 2; background: #e63946; font-size: 15px;}
    .btn-cal { flex: 1.2; background: #f39c12; }
    .btn-icon { background: #6c757d; text-decoration: none; }
  </style>
</head>
<body>
  <div class="main-wrapper">
    <div class="header">
      <div id="host-label">📡 WiFi 無線模式</div>
      <div id="ip-label" style="font-weight:bold;color:#1565c0;">IP...</div>
      <div id="st">○ Ready</div>
    </div>
    <div class="value-container">
      <div id="card1" class="v-card v-red"><span class="v-val" id="t1">--.-</span><div class="v-label">紅色探針 (°C)</div></div>
      <div id="card2" class="v-card v-blue"><span class="v-val" id="t2">--.-</span><div class="v-label">藍色探針 (°C)</div></div>
    </div>
    <div class="canvas-wrapper"><canvas id="cvs"></canvas></div>
    <div class="footer">
      <button id="btn" class="btn btn-start" onclick="toggle()">▶ 開始</button>
      <button class="btn btn-cal" onclick="calibrate()">⚖️ 校準</button>
      <button class="btn btn-icon" onclick="save()">💾 匯出</button>
      <button class="btn btn-icon" onclick="location.href='/wifi_config'">⚙️ 設定</button>
      <button class="btn btn-icon" style="background:#495057;" onclick="location.href='/usb'">🔌 USB</button>
    </div>
  </div>

  <script>
    let d1 = [], d2 = [], times = [], run = false;
    const cvs = document.getElementById('cvs'), ctx = cvs.getContext('2d');
    
    const calibrate = () => { if(confirm("將以紅針為基準對齊，是否繼續？")) fetch('/calibrate'); };
    
    const toggle = () => {
      run = !run; fetch('/toggle?run=' + (run?1:0));
      document.getElementById('btn').innerText = run ? "⏹ 停止" : "▶ 開始";
      document.getElementById('btn').className = run ? "btn btn-stop" : "btn btn-start";
      document.getElementById('st').innerText = run ? "● 紀錄中" : "○ Ready";
      document.getElementById('st').style.color = run ? "#d32f2f" : "#555";
      if(run) { d1 = []; d2 = []; times = []; }
      draw();
    };

    const draw = () => {
      cvs.width = cvs.offsetWidth; cvs.height = cvs.offsetHeight;
      const w = cvs.width, h = cvs.height, pL = 55, pR = 25, pT = 15, pB = 55;
      if (w === 0 || h === 0) return;
      ctx.clearRect(0,0,w,h);

      ctx.save(); ctx.translate(18, h/2); ctx.rotate(-Math.PI/2);
      ctx.textAlign = "center"; ctx.fillStyle = "#444"; ctx.font = "bold 14px sans-serif";
      ctx.fillText("溫度 (°C)", 0, 0); ctx.restore();

      ctx.textAlign = "right"; ctx.fillStyle = "#444"; ctx.font = "bold 14px sans-serif";
      ctx.fillText("時間 (秒)", w - 5, h - 12);

      let allP = d1.concat(d2).filter(v => v !== null);
      let vMin = 0, vMax = 40;
      if (allP.length > 0) {
        vMin = Math.min(0, Math.floor(Math.min(...allP) - 3));
        vMax = Math.max(40, Math.ceil(Math.max(...allP) + 3));
        if (vMax - vMin < 10) vMax = vMin + 10;
      }
      const rangeY = vMax - vMin;
      let gridStepY = (rangeY > 60) ? 20 : (rangeY > 20 ? 10 : 5);
      ctx.textAlign = "right"; ctx.font = "bold 13px sans-serif";
      for(let val = Math.floor(vMin/gridStepY)*gridStepY; val <= vMax; val += gridStepY) {
        if (val < vMin) continue;
        let y = pT + (vMax - val) / rangeY * (h - pT - pB);
        ctx.strokeStyle = (val === 0) ? "#333" : "#eee"; ctx.lineWidth = (val === 0) ? 2 : 1;
        ctx.beginPath(); ctx.moveTo(pL, y); ctx.lineTo(w - pR, y); ctx.stroke();
        ctx.fillStyle = (val === 0) ? "#000" : "#777"; ctx.fillText(Math.round(val), pL - 10, y + 4);
      }
      const maxL = Math.max(60, d1.length);
      const totalS = maxL; 
      let gridStepX = (totalS > 120) ? 30 : 10;
      ctx.textAlign = "center"; ctx.fillStyle = "#555"; ctx.font = "bold 12px sans-serif";
      for(let s = 0; s <= totalS; s += gridStepX) {
        let x = pL + (s / totalS) * (w - pL - pR);
        if (x > w - pR) break;
        ctx.fillText(s, x, h - pB + 25);
      }
      const plot = (data, col) => {
        if(data.length < 2) return;
        ctx.strokeStyle = col; ctx.lineWidth = 4.5; ctx.lineJoin = "round"; ctx.beginPath();
        let stepX = (w - pL - pR) / (maxL - 1);
        data.forEach((v, i) => {
          if(v === null) return;
          let x = pL + i * stepX, y = pT + (vMax - v) / rangeY * (h - pT - pB);
          if(i === 0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
        });
        ctx.stroke();
      };
      plot(d1, "#c62828"); plot(d2, "#1565c0");
    };

    setInterval(async () => {
      try {
        const r = await fetch('/data'); const d = await r.json();
        document.getElementById('t1').innerText = d.e1===0 ? d.t1.toFixed(1) : "NC";
        document.getElementById('t2').innerText = d.e2===0 ? d.t2.toFixed(1) : "NC";
        if(run) {
          d1.push(d.t1); d2.push(d.t2); times.push(d.ts);
          if(d1.length > 3000) { d1.shift(); d2.shift(); times.shift(); }
        }
        requestAnimationFrame(draw);
      } catch(e) {}
    }, 1000);

    window.onload = () => {
      fetch('/get_info').then(r => r.json()).then(i => { document.getElementById('ip-label').innerText = i.ip; });
      setTimeout(draw, 100); 
    };
    window.onresize = () => requestAnimationFrame(draw);
    
    const save = () => {
      let c = "Time(s),Red(C),Blue(C)\n";
      d1.forEach((v,i) => c += `${times[i].toFixed(1)},${v??'NaN'},${d2[i]??'NaN'}\n`);
      const b = new Blob([c], {type:'text/csv'});
      const a = document.createElement('a');
      const now = new Date(); const pad = n => (n < 10 ? '0' + n : n);
      const dtStr = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
      a.href = URL.createObjectURL(b); a.download = `temp_wifi_${dtStr}.csv`; a.click();
    };
  </script>
</body>
</html>
)=====";

// ================================================================
// 【頁面二：USB WebSerial 版 (含自訂 Modal 權限警告)】
// ================================================================
const char usb_html[] PROGMEM = R"=====(
<!DOCTYPE HTML><html lang="zh-TW">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>專業實驗儀器 - USB版</title>
  <style>
    * { box-sizing: border-box; }
    body, html { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; font-family: sans-serif; background: #eef2f5; }
    .main-wrapper { display: flex; flex-direction: column; height: 100vh; padding: 10px; }
    .header { display: flex; justify-content: space-between; align-items: center; height: 16px; font-size: 13px; color: #555; font-weight:bold; }
    .value-container { display: flex; gap: 10px; margin: 8px 0; flex: 0 0 auto; }
    .v-card { flex: 1; background: #fff; border-radius: 10px; padding: 10px; text-align: center; box-shadow: 0 4px 6px rgba(0,0,0,0.05); }
    .v-red { border: 4px solid #d32f2f; color: #c62828; }
    .v-blue { border: 4px solid #1976d2; color: #1565c0; }
    .v-val { font-size: 36px; font-weight: bold; font-family: monospace; display: block; margin: 2px 0; }
    .v-label { font-size: 13px; color: #333; font-weight: bold; }
    .canvas-wrapper { flex: 1; position: relative; margin: 5px 0; background: #fff; border: 1px solid #d1d5db; border-radius: 10px; min-height: 0; box-shadow: 0 4px 6px rgba(0,0,0,0.05); overflow: hidden;}
    canvas { width: 100% !important; height: 100% !important; display: block; }
    .footer { flex: 0 0 auto; display: flex; flex-direction: row; gap: 6px; padding-top: 5px; }
    .btn { flex: 1; padding: 12px 2px; border-radius: 8px; cursor: pointer; border: none; font-weight: bold; transition: 0.2s; white-space: nowrap; color: #fff; display:flex; justify-content:center; align-items:center; font-size: 14px;}
    .btn:active { transform: scale(0.96); }
    .btn-usb { flex: 2; background: #4361ee; font-size: 15px;}
    .btn-start { flex: 2; background: #2ecc71; font-size: 15px;}
    .btn-stop { flex: 2; background: #e63946; font-size: 15px;}
    .btn-cal { flex: 1.2; background: #f39c12; }
    .btn-icon { background: #6c757d; text-decoration: none; }

    /* 自訂 Modal 警告視窗樣式 */
    .modal-overlay { display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.55); z-index: 1000; justify-content: center; align-items: center; padding: 15px; backdrop-filter: blur(3px); }
    .modal-card { background: #fff; padding: 30px 25px; border-radius: 15px; width: 100%; max-width: 450px; box-shadow: 0 10px 30px rgba(0,0,0,0.2); }
    .modal-title { font-size: 22px; font-weight: bold; margin-bottom: 15px; color: #333; }
    .modal-title span { color: #e63946; }
    .modal-text { font-size: 15px; color: #555; line-height: 1.6; margin-bottom: 20px; }
    .modal-step { font-size: 15px; color: #333; margin-bottom: 8px; font-weight: bold; }
    .code-box { background: #f8f9fa; border: 1px solid #e1e4e8; border-radius: 8px; padding: 14px; text-align: center; font-family: monospace; font-size: 16px; color: #c62828; font-weight: bold; margin-bottom: 20px; cursor: pointer; user-select: all; word-break: break-all; transition: background 0.2s; }
    .code-box:hover { background: #f1f3f5; }
    .modal-btn-wrap { text-align: right; margin-top: 25px; }
    .modal-btn { background: #6c757d; color: #fff; border: none; padding: 12px 24px; border-radius: 8px; font-size: 15px; font-weight: bold; cursor: pointer; transition: 0.2s; }
    .modal-btn:hover { background: #5a6268; }
    .modal-btn:active { transform: scale(0.96); }
  </style>
</head>
<body>
  <div class="main-wrapper">
    <div class="header">
      <div id="mode">🔌 USB 模式 (等待連線)</div>
      <div id="st">○ Ready</div>
    </div>
    <div class="value-container">
      <div id="card1" class="v-card v-red"><span class="v-val" id="t1">--.-</span><div class="v-label">紅色探針 (°C)</div></div>
      <div id="card2" class="v-card v-blue"><span class="v-val" id="t2">--.-</span><div class="v-label">藍色探針 (°C)</div></div>
    </div>
    <div class="canvas-wrapper"><canvas id="cvs"></canvas></div>
    
    <div class="footer">
      <button id="btnConn" class="btn btn-usb" onclick="initUSB()">🔌 連線</button>
      <button id="btn" class="btn btn-start" style="display:none;" onclick="toggle()">▶ 開始</button>
      <button class="btn btn-cal" onclick="calibrate()">⚖️ 校準</button>
      <button class="btn btn-icon" onclick="save()">💾 匯出</button>
      <button class="btn btn-icon" onclick="location.href='/wifi_config'">⚙️ 設定</button>
      <button class="btn btn-icon" style="background:#495057;" onclick="location.href='/'">📡 WiFi</button>
    </div>
  </div>

  <div id="usbModal" class="modal-overlay">
    <div class="modal-card">
      <div class="modal-title">⚠️ <span>啟用 USB 傳輸權限</span></div>
      <div class="modal-text">
        Chrome / Edge 瀏覽器基於安全機制，預設封鎖本地網頁的 USB 權限。請照著以下步驟解鎖：<br>
        <span style="color:#888; font-size:13px;">(註：Safari / iOS 不支援 USB 功能，請改用 WiFi 模式)</span>
      </div>
      <div class="modal-step">1. 請點擊下方網址將其反白並複製，貼到瀏覽器的新分頁前往：</div>
      <div class="code-box">chrome://flags/#unsafely-treat-insecure-origin-as-secure</div>
      <div class="modal-step">2. 請點擊下方目前的網址將其反白並複製，貼入該設定頁面的文字框中：</div>
      <div class="code-box" id="modal-ip">http://192.168.x.x</div>
      <div class="modal-step">3. 將右側選單改為 <b style="color:#4361ee">Enabled</b>，點擊右下角 <b style="color:#4361ee">Relaunch</b> 重啟瀏覽器即可。</div>
      <div class="modal-btn-wrap">
        <button class="modal-btn" onclick="document.getElementById('usbModal').style.display='none'">我知道了，先關閉</button>
      </div>
    </div>
  </div>

  <script>
    let d1 = [], d2 = [], times = [], run = false, port, reader;
    const cvs = document.getElementById('cvs'), ctx = cvs.getContext('2d');
    
    const processData = (line) => {
      const p = line.trim().split(','); if(p.length < 3) return;
      const t = parseFloat(p[0]), v1 = parseFloat(p[1]), v2 = parseFloat(p[2]);
      document.getElementById('t1').innerText = isNaN(v1) ? "NC" : v1.toFixed(1);
      document.getElementById('t2').innerText = isNaN(v2) ? "NC" : v2.toFixed(1);
      if(run) {
        d1.push(isNaN(v1) ? null : v1); d2.push(isNaN(v2) ? null : v2); times.push(t);
        if(d1.length > 3000) { d1.shift(); d2.shift(); times.shift(); }
        requestAnimationFrame(draw);
      }
    };

    const initUSB = async () => {
      if (!navigator.serial) {
        // 抓取目前瀏覽器的完整 URL (Origin)，動態填入提示框
        document.getElementById('modal-ip').innerText = window.location.origin;
        // 顯示自訂的 Modal 警告窗
        document.getElementById('usbModal').style.display = 'flex';
        return;
      }
      try {
        port = await navigator.serial.requestPort(); await port.open({baudRate:115200});
        document.getElementById('mode').innerText = "🔌 USB 🟢 已連線";
        document.getElementById('btnConn').style.display = "none";
        document.getElementById('btn').style.display = "flex";
        const decoder = new TextDecoderStream(); port.readable.pipeTo(decoder.writable);
        reader = decoder.readable.getReader(); let buf="";
        while(true) { const {value, done} = await reader.read(); if(done) break; buf+=value; let lines=buf.split('\n'); buf=lines.pop(); for(let l of lines) processData(l); }
      } catch(e) { alert("⚠️ USB連線失敗！\n請確認 Arduino 序列埠監控視窗已關閉。"); }
    };

    const calibrate = () => { if(confirm("將以紅針為基準對齊，是否繼續？")) fetch('/calibrate'); };
    
    const toggle = () => {
      run = !run; fetch('/toggle?run=' + (run?1:0));
      document.getElementById('btn').innerText = run ? "⏹ 停止" : "▶ 開始";
      document.getElementById('btn').className = run ? "btn btn-stop" : "btn btn-start";
      document.getElementById('st').innerText = run ? "● 紀錄中" : "○ Ready";
      document.getElementById('st').style.color = run ? "#d32f2f" : "#555";
      if(run) { d1 = []; d2 = []; times = []; }
      draw();
    };

    const draw = () => {
      cvs.width = cvs.offsetWidth; cvs.height = cvs.offsetHeight;
      const w = cvs.width, h = cvs.height, pL = 55, pR = 25, pT = 15, pB = 55;
      if (w === 0 || h === 0) return;
      ctx.clearRect(0,0,w,h);

      ctx.save(); ctx.translate(18, h/2); ctx.rotate(-Math.PI/2);
      ctx.textAlign = "center"; ctx.fillStyle = "#444"; ctx.font = "bold 14px sans-serif";
      ctx.fillText("溫度 (°C)", 0, 0); ctx.restore();

      ctx.textAlign = "right"; ctx.fillStyle = "#444"; ctx.font = "bold 14px sans-serif";
      ctx.fillText("時間 (秒)", w - 5, h - 12);

      let allP = d1.concat(d2).filter(v => v !== null);
      let vMin = 0, vMax = 40;
      if (allP.length > 0) {
        vMin = Math.min(0, Math.floor(Math.min(...allP) - 3));
        vMax = Math.max(40, Math.ceil(Math.max(...allP) + 3));
        if (vMax - vMin < 10) vMax = vMin + 10;
      }
      const rangeY = vMax - vMin;
      let gridStepY = (rangeY > 60) ? 20 : (rangeY > 20 ? 10 : 5);
      ctx.textAlign = "right"; ctx.font = "bold 13px sans-serif";
      for(let val = Math.floor(vMin/gridStepY)*gridStepY; val <= vMax; val += gridStepY) {
        if (val < vMin) continue;
        let y = pT + (vMax - val) / rangeY * (h - pT - pB);
        ctx.strokeStyle = (val === 0) ? "#333" : "#eee"; ctx.lineWidth = (val === 0) ? 2 : 1;
        ctx.beginPath(); ctx.moveTo(pL, y); ctx.lineTo(w - pR, y); ctx.stroke();
        ctx.fillStyle = (val === 0) ? "#000" : "#777"; ctx.fillText(Math.round(val), pL - 10, y + 4);
      }
      const maxL = Math.max(60, d1.length);
      const totalS = maxL; 
      let gridStepX = (totalS > 120) ? 30 : 10;
      ctx.textAlign = "center"; ctx.fillStyle = "#555"; ctx.font = "bold 12px sans-serif";
      for(let s = 0; s <= totalS; s += gridStepX) {
        let x = pL + (s / totalS) * (w - pL - pR);
        if (x > w - pR) break;
        ctx.fillText(s, x, h - pB + 25);
      }
      const plot = (data, col) => {
        if(data.length < 2) return;
        ctx.strokeStyle = col; ctx.lineWidth = 4.5; ctx.lineJoin = "round"; ctx.beginPath();
        let stepX = (w - pL - pR) / (maxL - 1);
        data.forEach((v, i) => {
          if(v === null) return;
          let x = pL + i * stepX, y = pT + (vMax - v) / rangeY * (h - pT - pB);
          if(i === 0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
        });
        ctx.stroke();
      };
      plot(d1, "#c62828"); plot(d2, "#1565c0");
    };

    window.onload = () => setTimeout(draw, 100); 
    window.onresize = () => requestAnimationFrame(draw);
    
    const save = () => {
      let c = "Time(s),Red(C),Blue(C)\n";
      d1.forEach((v,i) => c += `${times[i].toFixed(1)},${v??'NaN'},${d2[i]??'NaN'}\n`);
      const b = new Blob([c], {type:'text/csv'});
      const a = document.createElement('a');
      const now = new Date(); const pad = n => (n < 10 ? '0' + n : n);
      const dtStr = `${now.getFullYear()}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
      a.href = URL.createObjectURL(b); a.download = `temp_usb_${dtStr}.csv`; a.click();
    };
  </script>
</body>
</html>
)=====";

// ================================================================
// 【WiFi 設定頁面】
// ================================================================
const char config_html[] PROGMEM = R"=====(
<!DOCTYPE HTML><html lang="zh-TW"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: sans-serif; background: #eef2f5; display: flex; justify-content: center; padding: 20px; margin: 0; }
  .card { background: white; padding: 25px; border-radius: 12px; width: 100%; max-width: 400px; box-shadow: 0 4px 15px rgba(0,0,0,0.08); }
  h2 { color: #333; margin-top: 0; text-align: center; }
  .ap-list { max-height: 200px; overflow-y: auto; text-align: left; border: 1px solid #eee; border-radius: 6px; padding: 5px; margin-bottom: 15px; }
  .ap-item { cursor: pointer; padding: 10px; border-bottom: 1px solid #eee; color: #4361ee; font-weight: bold; transition: background 0.2s;}
  .ap-item:hover { background: #f8f9fa; }
  label { font-weight: bold; color: #444; font-size: 14px; display: block; margin-bottom: 5px; }
  input[type="text"], input[type="password"] { width: 100%; padding: 12px; margin-bottom: 15px; border: 1px solid #ccc; border-radius: 6px; box-sizing: border-box; font-size: 16px; }
  .checkbox-group { display: flex; align-items: center; margin-bottom: 20px; gap: 8px; }
  .checkbox-group input { width: 18px; height: 18px; cursor: pointer; margin: 0;}
  .checkbox-group label { margin: 0; cursor: pointer; font-weight: normal; color: #555; }
  .btn-save { background: #2ecc71; color: white; border: none; padding: 14px; width: 100%; border-radius: 6px; font-weight: bold; font-size: 16px; cursor: pointer; margin-bottom: 10px; transition: 0.2s;}
  .btn-save:active { transform: scale(0.98); }
  .btn-back { background: #6c757d; color: white; border: none; padding: 14px; width: 100%; border-radius: 6px; font-size: 15px; cursor: pointer; font-weight: bold; transition: 0.2s;}
  .btn-back:active { transform: scale(0.98); }
</style></head><body>
<div class="card">
  <h2>WiFi 網路設定</h2>
  <div id="list" class="ap-list">掃描中...</div>
  <form action="/save_wifi" method="POST">
    <label>選擇或輸入 SSID:</label>
    <input type="text" name="ssid" id="s" placeholder="WiFi 名稱">
    <label>輸入密碼:</label>
    <input type="password" name="password" id="p" placeholder="WiFi 密碼">
    <div class="checkbox-group">
      <input type="checkbox" id="showPwd" onclick="togglePwd()">
      <label for="showPwd">顯示密碼</label>
    </div>
    <input type="submit" class="btn-save" value="儲存設定並重啟">
  </form>
  <button class="btn-back" onclick="location.href='/'">返回主畫面</button>
</div>
<script>
  const fetchList = () => {
    fetch('/scan_list').then(r => r.json()).then(d => {
      let h = "";
      if(d.length === 0) h = "<div style='padding:10px;color:#888;text-align:center;'>找不到網路</div>";
      d.forEach(ap => { h += `<div class="ap-item" onclick="document.getElementById('s').value='${ap.ssid}'">${ap.ssid} <span style="color:#999;font-size:12px;font-weight:normal;">(${ap.rssi}dBm)</span></div>`; });
      document.getElementById('list').innerHTML = h;
    }).catch(e => console.log(e));
  };
  const togglePwd = () => {
    const p = document.getElementById('p');
    p.type = p.type === "password" ? "text" : "password";
  };
  window.onload = () => {
    fetchList();
    setInterval(fetchList, 15000); 
  };
</script>
</body></html>
)=====";

void setup() {
  setCpuFrequencyMhz(240); 
  Serial.begin(115200);
  pixels.begin(); pixels.setBrightness(50);
  jsonMutex = xSemaphoreCreateMutex();
  
  // OLED 初始化 (I2C SDA=8, SCL=7)
  Wire.begin(I2C_SDA, I2C_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED 初始失敗"));
  } else {
    display.clearDisplay();
    display.display();
  }

  wm.setAPCallback(configModeCallback);

  WiFi.mode(WIFI_AP_STA);
  setRGB(255, 0, 0); 
  if(!wm.autoConnect(AP_NAME)) { ESP.restart(); }
  setRGB(0, 0, 255); 

  esp_task_wdt_config_t twdt_config = { .timeout_ms = (uint32_t)WDT_TIMEOUT, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_init(&twdt_config); esp_task_wdt_add(NULL);
  
  if (MDNS.begin(HOST_NAME)) { MDNS.addService("http", "tcp", 80); }
  
  xTaskCreatePinnedToCore(TaskSampling, "Temp", 4096, NULL, 1, NULL, 1);

  server.on("/", [](){ server.send_P(200, "text/html", wifi_html); });
  server.on("/usb", [](){ server.send_P(200, "text/html", usb_html); });
  server.on("/wifi_config", [](){ server.send_P(200, "text/html", config_html); });

  server.on("/get_info", [](){
    String json = "{\"ip\":\"" + WiFi.localIP().toString() + "\", \"host\":\"" + String(HOST_NAME) + "\"}";
    server.send(200, "application/json", json);
  });
  
  server.on("/data", [](){
    if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      server.send(200, "application/json", latestDataJson);
      xSemaphoreGive(jsonMutex);
    }
  });
  
  server.on("/calibrate", [](){
    if (filteredT1 > -50 && filteredT2 > -50) {
      calibrationOffset = filteredT1 - (filteredT2 - calibrationOffset);
    }
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/toggle", [](){
    isDrawing = (server.arg("run") == "1");
    if(isDrawing) runStartTime = millis(); 
    setRGB(0, isDrawing ? 255 : 0, isDrawing ? 0 : 255);
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/scan_list", [](){
    int n = WiFi.scanNetworks();
    JsonDocument doc; JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < n; i++) {
      JsonObject obj = arr.add<JsonObject>();
      obj["ssid"] = WiFi.SSID(i); obj["rssi"] = WiFi.RSSI(i);
    }
    String json; serializeJson(doc, json);
    server.send(200, "application/json", json);
    WiFi.scanDelete();
  });
  
  server.on("/save_wifi", HTTP_POST, [](){
    String s = server.arg("ssid"), p = server.arg("password");
    String res = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{font-family:sans-serif;background:#eef2f5;text-align:center;padding-top:50px;color:#333;}</style></head><body><h2>💾 儲存中...</h2><p>儀器正在重啟並連線至 <b style='color:#1565c0'>" + s + "</b></p><p style='color:#666;font-size:14px;'>請等待約 10 秒後，點擊下方按鈕返回。</p><br><button onclick=\"location.href='/'\" style=\"background:#2ecc71;color:#fff;border:none;padding:12px 24px;border-radius:6px;font-size:16px;font-weight:bold;cursor:pointer;\">返回儀表板</button></body></html>";
    server.send(200, "text/html", res);
    delay(2000); WiFi.begin(s.c_str(), p.c_str());
  });
  
  server.begin();
}

void loop() {
  server.handleClient();
  esp_task_wdt_reset();
  
  if (millis() - lastOledUpdate > 500) {
    lastOledUpdate = millis();
    updateOLED();
  }

  if (millis() - lastWiFiCheck > 15000) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
  delay(1);
}

