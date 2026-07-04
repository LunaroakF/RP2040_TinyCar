#include <WiFi.h>
#include <WebServer.h>
#include "driver/spi_slave.h"
#include <vector>
#include <cmath>

// ---- Wi-Fi 路由器配置 ----
const char* WIFI_SSID = "fox";
const char* WIFI_PASS = "12345678";

// ---- 静态 IP 配置 ----
IPAddress local_IP(192, 168, 199, 25);   // 你指定的 IP
IPAddress gateway(192, 168, 199, 1);     // 199 网段网关
IPAddress subnet(255, 255, 255, 0);      // 子网掩码
IPAddress primaryDNS(192, 168, 199, 1);  // 可选：主 DNS

// ---- SPI Slave 引脚(接RP2040, 对应RP2040端 ECHIP_MOSI/MISO/SCLK/CS) ----
#define ECHIP_MOSI 13  
#define ECHIP_MISO 12  
#define ECHIP_SCLK 14  
#define ECHIP_CS   27  
#define SPI_HOST_USED HSPI_HOST

#define FRAME_SIZE 64 

WebServer server(80);

// ---------------- 原始日志(仍保留,用于调试) ----------------
String logBuffer;
const size_t MAX_LOG_SIZE = 6000;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

void appendLog(const String &s) {
    portENTER_CRITICAL(&logMux);
    logBuffer += s;
    if (logBuffer.length() > MAX_LOG_SIZE) {
        logBuffer = logBuffer.substring(logBuffer.length() - MAX_LOG_SIZE);
    }
    portEXIT_CRITICAL(&logMux);
}

String getLogSnapshot() {
    portENTER_CRITICAL(&logMux);
    String s = logBuffer;
    portEXIT_CRITICAL(&logMux);
    return s;
}

// ---------------- 小车位置 & 障碍物数据 ----------------
struct Point { float x; float y; };

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool robotValid = false;
volatile float robotX = 0, robotY = 0;

std::vector<Point> obstacles;
const size_t MAX_OBSTACLES = 300;
const float OBSTACLE_MERGE_DIST = 2.0f; 

// ---------------- 小车轨迹(移动路径) ----------------
std::vector<Point> trail;
const size_t MAX_TRAIL = 500;      
const float TRAIL_MIN_DIST = 3.0f; 

void addObstacle(float x, float y) {
    portENTER_CRITICAL(&dataMux);
    bool found = false;
    for (auto &p : obstacles) {
        if (fabsf(p.x - x) < OBSTACLE_MERGE_DIST && fabsf(p.y - y) < OBSTACLE_MERGE_DIST) {
            p.x = x; p.y = y;
            found = true;
            break;
        }
    }
    if (!found) {
        obstacles.push_back({x, y});
        if (obstacles.size() > MAX_OBSTACLES) {
            obstacles.erase(obstacles.begin());
        }
    }
    portEXIT_CRITICAL(&dataMux);
}

void setRobotPos(float x, float y) {
    portENTER_CRITICAL(&dataMux);
    robotX = x;
    robotY = y;
    robotValid = true;

    if (trail.empty()) {
        trail.push_back({x, y});
    } else {
        Point &last = trail.back();
        float dx = x - last.x, dy = y - last.y;
        if (dx * dx + dy * dy >= TRAIL_MIN_DIST * TRAIL_MIN_DIST) {
            trail.push_back({x, y});
            if (trail.size() > MAX_TRAIL) {
                trail.erase(trail.begin());
            }
        }
    }
    portEXIT_CRITICAL(&dataMux);
}

void clearObstacles() {
    portENTER_CRITICAL(&dataMux);
    obstacles.clear();
    portEXIT_CRITICAL(&dataMux);
}

void clearTrail() {
    portENTER_CRITICAL(&dataMux);
    trail.clear();
    portEXIT_CRITICAL(&dataMux);
}

void parseLine(const String &lineIn) {
    String line = lineIn;
    line.trim();
    if (line.length() < 2) return;

    char tag = line.charAt(0);
    if (tag != '#' && tag != '@') return;

    String rest = line.substring(1);
    int comma = rest.indexOf(',');
    if (comma < 0) return;

    float x = rest.substring(0, comma).toFloat();
    float y = rest.substring(comma + 1).toFloat();

    if (tag == '#') {
        addObstacle(x, y);
    } else {
        setRobotPos(x, y);
    }
}

String buildDataJson() {
    portENTER_CRITICAL(&dataMux);
    String json = "{\"robot\":";
    if (robotValid) {
        json += "{\"x\":" + String(robotX, 2) + ",\"y\":" + String(robotY, 2) + "}";
    } else {
        json += "null";
    }
    json += ",\"obstacles\":[";
    for (size_t i = 0; i < obstacles.size(); i++) {
        if (i) json += ",";
        json += "{\"x\":" + String(obstacles[i].x, 2) + ",\"y\":" + String(obstacles[i].y, 2) + "}";
    }
    json += "],\"trail\":[";
    for (size_t i = 0; i < trail.size(); i++) {
        if (i) json += ",";
        json += "{\"x\":" + String(trail[i].x, 2) + ",\"y\":" + String(trail[i].y, 2) + "}";
    }
    json += "]}";
    portEXIT_CRITICAL(&dataMux);
    return json;
}

// ---- SPI Slave 接收任务 ----
void spiSlaveTask(void *pv) {
    static WORD_ALIGNED_ATTR uint8_t rxbuf[FRAME_SIZE];
    static WORD_ALIGNED_ATTR uint8_t txbuf[FRAME_SIZE];
    memset(txbuf, 0, FRAME_SIZE);

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = FRAME_SIZE * 8; 
    t.tx_buffer = txbuf;
    t.rx_buffer = rxbuf;

    static String lineBuf; 

    while (true) {
        memset(rxbuf, 0, FRAME_SIZE);
        esp_err_t ret = spi_slave_transmit(SPI_HOST_USED, &t, portMAX_DELAY);
        if (ret == ESP_OK) {
            uint8_t len = rxbuf[0];
            if (len > 0 && len < FRAME_SIZE) {
                String s;
                s.reserve(len);
                for (int i = 0; i < len; i++) s += (char)rxbuf[1 + i];

                appendLog(s); 

                for (size_t i = 0; i < s.length(); i++) {
                    char c = s[i];
                    if (c == '\n') {
                        parseLine(lineBuf);
                        lineBuf = "";
                    } else if (c != '\r') {
                        lineBuf += c;
                        if (lineBuf.length() > 200) lineBuf = ""; 
                    }
                }
            }
        }
    }
}

// HTML 保持不变...
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<title>RP2040 小车位置监视</title>
<style>
  * { box-sizing: border-box; }
  body { font-family: sans-serif; background:#1e1e1e; color:#eee; margin:0; padding:16px; }
  h2 { color:#4fc3f7; margin: 0 0 12px; }
  .tabs { display:flex; gap:8px; margin-bottom:10px; }
  .tab-btn {
    background:#333; color:#ccc; border:1px solid #555; border-radius:4px;
    padding:6px 14px; cursor:pointer; font-size:14px;
  }
  .tab-btn.active { background:#4fc3f7; color:#111; font-weight:bold; }
  .panel { display:none; }
  .panel.active { display:block; }
  #mapWrap {
    position:relative; width:100%; height:70vh; background:#0a0a0a;
    border:1px solid #444; border-radius:4px; overflow:hidden;
  }
  canvas { display:block; }
  #info {
    margin-top:10px; font-size:13px; color:#aaa; display:flex; gap:20px; flex-wrap:wrap;
  }
  #info span.dot { display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:5px; vertical-align:middle; }
  .btnRow { margin-top:10px; }
  button.action {
    background:#333; color:#eee; border:1px solid #666; border-radius:4px;
    padding:6px 12px; cursor:pointer; margin-right:8px;
  }
  button.action:hover { background:#444; }
  textarea {
    width: 100%; height: 65vh; background:#111; color:#0f0;
    font-family: monospace; font-size: 14px; border:1px solid #444;
    padding:8px; box-sizing:border-box; resize:none;
  }
</style>
</head>
<body>
  <h2>RP2040 小车 &amp; 障碍物 位置监视</h2>
  <div class="tabs">
    <button class="tab-btn active" data-tab="map">坐标地图</button>
    <button class="tab-btn" data-tab="log">原始日志</button>
  </div>

  <div id="mapPanel" class="panel active">
    <div id="mapWrap"><canvas id="mapCanvas"></canvas></div>
    <div id="info">
      <span><span class="dot" style="background:#4fc3f7;"></span>小车位置: <span id="robotInfo">--</span></span>
      <span><span class="dot" style="background:#ff5252;"></span>障碍物数量: <span id="obsCount">0</span></span>
      <span><span class="dot" style="background:#4fc3f7;opacity:0.5;"></span>轨迹点数: <span id="trailCount">0</span></span>
    </div>
    <div class="btnRow">
      <button class="action" id="clearBtn">清空障碍物</button>
      <button class="action" id="clearTrailBtn">清空轨迹</button>
    </div>
  </div>

  <div id="logPanel" class="panel">
    <textarea id="log" readonly></textarea>
  </div>

<script>
  document.querySelectorAll('.tab-btn').forEach(btn=>{
    btn.addEventListener('click', ()=>{
      document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
      document.querySelectorAll('.panel').forEach(p=>p.classList.remove('active'));
      btn.classList.add('active');
      document.getElementById(btn.dataset.tab+'Panel').classList.add('active');
    });
  });

  document.getElementById('clearBtn').addEventListener('click', ()=>{
    fetch('/clear', {method:'POST'});
  });
  document.getElementById('clearTrailBtn').addEventListener('click', ()=>{
    fetch('/clearTrail', {method:'POST'});
  });

  const logTa = document.getElementById('log');
  async function pollLog() {
    try {
      const res = await fetch('/log');
      const text = await res.text();
      const atBottom = (logTa.scrollTop + logTa.clientHeight >= logTa.scrollHeight - 5);
      logTa.value = text;
      if (atBottom) logTa.scrollTop = logTa.scrollHeight;
    } catch (e) {}
    setTimeout(pollLog, 500);
  }
  pollLog();

  const canvas = document.getElementById('mapCanvas');
  const ctx = canvas.getContext('2d');
  const wrap = document.getElementById('mapWrap');
  const OBSTACLE_RADIUS = 5; 

  function resizeCanvas() {
    canvas.width = wrap.clientWidth;
    canvas.height = wrap.clientHeight;
  }
  window.addEventListener('resize', resizeCanvas);
  resizeCanvas();

  function niceStep(raw) {
    const pow10 = Math.pow(10, Math.floor(Math.log10(raw)));
    const n = raw / pow10;
    let step;
    if (n < 1.5) step = 1;
    else if (n < 3.5) step = 2;
    else if (n < 7.5) step = 5;
    else step = 10;
    return step * pow10;
  }

  function drawMap(data) {
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0,0,W,H);

    const robot = data.robot;
    const obstacles = data.obstacles || [];
    const trail = data.trail || [];

    let minX=-50, maxX=50, minY=-50, maxY=50;
    const pts = [];
    if (robot) pts.push(robot);
    obstacles.forEach(o=>pts.push(o));
    trail.forEach(p=>pts.push(p));

    if (pts.length > 0) {
      minX = Math.min(...pts.map(p=>p.x));
      maxX = Math.max(...pts.map(p=>p.x));
      minY = Math.min(...pts.map(p=>p.y));
      maxY = Math.max(...pts.map(p=>p.y));
    }
    const pad = Math.max(20, OBSTACLE_RADIUS * 3);
    minX -= pad; maxX += pad; minY -= pad; maxY += pad;
    if (maxX - minX < 1) { maxX += 10; minX -= 10; }
    if (maxY - minY < 1) { maxY += 10; minY -= 10; }

    const rangeX = maxX - minX, rangeY = maxY - minY;
    const scale = Math.min(W / rangeX, H / rangeY);

    const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
    function toScreen(x, y) {
      return [
        W/2 + (x - cx) * scale,
        H/2 - (y - cy) * scale
      ];
    }

    const targetPx = 60;
    const step = niceStep(targetPx / scale);
    ctx.strokeStyle = '#2a2a2a';
    ctx.fillStyle = '#666';
    ctx.font = '11px monospace';
    ctx.lineWidth = 1;

    let gx = Math.ceil(minX / step) * step;
    for (; gx <= maxX; gx += step) {
      const [sx] = toScreen(gx, 0);
      ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, H); ctx.stroke();
      ctx.fillText(gx.toFixed(0), sx + 2, H - 4);
    }
    let gy = Math.ceil(minY / step) * step;
    for (; gy <= maxY; gy += step) {
      const [, sy] = toScreen(0, gy);
      ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(W, sy); ctx.stroke();
      ctx.fillText(gy.toFixed(0), 2, sy - 2);
    }

    ctx.strokeStyle = '#555';
    ctx.lineWidth = 1.5;
    if (minX <= 0 && maxX >= 0) {
      const [sx] = toScreen(0,0);
      ctx.beginPath(); ctx.moveTo(sx,0); ctx.lineTo(sx,H); ctx.stroke();
    }
    if (minY <= 0 && maxY >= 0) {
      const [, sy] = toScreen(0,0);
      ctx.beginPath(); ctx.moveTo(0,sy); ctx.lineTo(W,sy); ctx.stroke();
    }

    if (trail.length >= 2) {
      ctx.strokeStyle = 'rgba(79,195,247,0.55)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      trail.forEach((p, i)=>{
        const [sx, sy] = toScreen(p.x, p.y);
        if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
    }

    ctx.fillStyle = 'rgba(255,82,82,0.35)';
    ctx.strokeStyle = '#ff5252';
    ctx.lineWidth = 1.5;
    obstacles.forEach(o=>{
      const [sx, sy] = toScreen(o.x, o.y);
      const r = OBSTACLE_RADIUS * scale;
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, Math.PI*2);
      ctx.fill();
      ctx.stroke();
    });
    if (obstacles.length <= 25) {
      ctx.fillStyle = '#ff8a8a';
      ctx.font = '10px monospace';
      obstacles.forEach(o=>{
        const [sx, sy] = toScreen(o.x, o.y);
        ctx.fillText(`(${o.x.toFixed(0)},${o.y.toFixed(0)})`, sx + 6, sy - 6);
      });
    }

    if (robot) {
      const [sx, sy] = toScreen(robot.x, robot.y);
      ctx.fillStyle = '#4fc3f7';
      ctx.beginPath();
      ctx.arc(sx, sy, 7, 0, Math.PI*2);
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.fillStyle = '#4fc3f7';
      ctx.font = 'bold 11px monospace';
      ctx.fillText(`小车 (${robot.x.toFixed(1)},${robot.y.toFixed(1)})`, sx + 10, sy - 10);
    }

    document.getElementById('robotInfo').textContent = robot ?
      `x=${robot.x.toFixed(2)}, y=${robot.y.toFixed(2)}` : '暂无数据';
    document.getElementById('obsCount').textContent = obstacles.length;
    document.getElementById('trailCount').textContent = trail.length;
  }

  async function pollData() {
    try {
      const res = await fetch('/data');
      const data = await res.json();
      drawMap(data);
    } catch (e) {}
    setTimeout(pollData, 300);
  }
  pollData();
</script>
</body>
</html>
)rawliteral";

void handleRoot()   { server.send_P(200, "text/html", INDEX_HTML); }
void handleLog()     { server.send(200, "text/plain; charset=utf-8", getLogSnapshot()); }
void handleData()    { server.send(200, "application/json", buildDataJson()); }
void handleClear()   { clearObstacles(); server.send(200, "text/plain", "OK"); }
void handleClearTrail() { clearTrail(); server.send(200, "text/plain", "OK"); }

void setup() {
    Serial.begin(115200);

    // ---- 初始化SPI Slave ----
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = ECHIP_MOSI;
    buscfg.miso_io_num = ECHIP_MISO;
    buscfg.sclk_io_num = ECHIP_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.mode = 0;
    slvcfg.spics_io_num = ECHIP_CS;
    slvcfg.queue_size = 3;
    slvcfg.flags = 0;

    esp_err_t ret = spi_slave_initialize(SPI_HOST_USED, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("SPI slave init failed: %d\n", ret);
    }

    xTaskCreatePinnedToCore(spiSlaveTask, "spiSlave", 4096, NULL, 1, NULL, 1);

    // ---- 配置静态 IP 并在 STA 模式下连接路由器 ----
    WiFi.mode(WIFI_STA);
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)) {
        Serial.println("STA Configuration Failed!");
    }
    
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // 等待连接成功
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP()); // 这里应该雷打不动输出 192.168.199.25

    server.on("/", handleRoot);
    server.on("/log", handleLog);
    server.on("/data", handleData);
    server.on("/clear", HTTP_POST, handleClear);
    server.on("/clearTrail", HTTP_POST, handleClearTrail);
    server.begin();
    Serial.println("Web server started");
}

void loop() {
    server.handleClient();
}