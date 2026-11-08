// ---------------- text helpers ----------------
static void useFont(const uint8_t *f) {
  gfx->setFont(f);
  gfx->setTextSize(1);
}
static int textW(const char *s) {
  int16_t x1, y1;
  uint16_t w, h;
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  return (int)w;
}
static void txtAt(const char *s, int x, int y, uint16_t c) {
  gfx->setTextColor(c);
  gfx->setCursor(x, y);
  gfx->print(s);
}
static void txtC(const char *s, int y, uint16_t c) {
  int w = textW(s);
  gfx->setTextColor(c);
  gfx->setCursor(R_CX - w / 2, y);
  gfx->print(s);
}
static void txtC(const String &s, int y, uint16_t c) { txtC(s.c_str(), y, c); }

// ---------------- geometry ----------------
static double deg2rad(double d) { return d * PI / 180.0; }
static double rad2deg(double r) { return r * 180.0 / PI; }

static float haversineKm(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371.0;
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return (float)(R * c);
}

static float bearingTo(double lat1, double lon1, double lat2, double lon2) {
  double dLon = deg2rad(lon2 - lon1);
  double y = sin(dLon) * cos(deg2rad(lat2));
  double x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) -
             sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(dLon);
  float b = (float)rad2deg(atan2(y, x));
  if (b < 0) b += 360;
  return b;
}

// Spherical-mercator tile math
static void lonLatToTile(double lat, double lon, int z, int &tx, int &ty,
                         double &px, double &py) {
  double n = pow(2.0, z);
  double sf = n * 256.0;
  double mx = (lon + 180.0) / 360.0 * sf;
  double siny = sin(deg2rad(lat));
  if (siny > 0.9999) siny = 0.9999;
  if (siny < -0.9999) siny = -0.9999;
  double my = (0.5 - log((1 + siny) / (1 - siny)) / (4 * PI)) * sf;
  tx = (int)floor(mx / 256.0);
  ty = (int)floor(my / 256.0);
  px = mx - tx * 256.0;
  py = my - ty * 256.0;
}

// ---------------- PNG buffer lifecycle ----------------
// The PNG buffer is only held while downloading/decoding rain tiles, and it is
// sized to the exact tile Content-Length (~16KB base map, ~5KB rain layer)
// instead of a fixed 72KB block that no longer fits after WiFi/TLS.
// If a smaller allocation fails the size is halved down to PNG_BUF_MIN.
static bool ensurePngBuf(size_t need) {
  if (need < PNG_BUF_MIN) need = PNG_BUF_MIN;
  if (need > PNG_BUF_CAP) need = PNG_BUF_CAP;
  if (s_pngData && s_pngCap >= need) return true;
  if (s_pngData) {
    free(s_pngData);
    s_pngData = nullptr;
    s_pngCap = 0;
  }
  size_t cap = need;
  while (cap >= PNG_BUF_MIN) {
    s_pngData = (uint8_t *)malloc(cap);
    if (s_pngData) {
      s_pngCap = cap;
      Serial.printf("[mem] png buf %u free=%u maxblk=%u\n", (unsigned)cap,
                    ESP.getFreeHeap(), ESP.getMaxAllocHeap());
      return true;
    }
    cap /= 2;
  }
  Serial.printf("[mem] png alloc fail need=%u free=%u maxblk=%u\n",
                (unsigned)need, ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  return false;
}
static void releasePngBuf() {
  if (s_pngData) {
    free(s_pngData);
    s_pngData = nullptr;
  }
  s_pngCap = 0;
  s_pngLen = 0;
}

// ---------------- flash cache (SPIFFS) ----------------
static bool fsLoad(const char *path) {
  if (!s_fsOk) return false;
  File f = SPIFFS.open(path, "r");
  if (!f) return false;
  size_t sz = (size_t)f.size();
  if (!ensurePngBuf(sz)) { f.close(); return false; }
  size_t rd = f.read(s_pngData, sz);
  f.close();
  s_pngLen = rd;
  return rd == sz && sz > 64;
}
static bool fsSave(const char *path, size_t len) {
  if (!s_fsOk || !s_pngData || len < 64) return false;
  File f = SPIFFS.open(path, "w");
  if (!f) return false;
  size_t wr = f.write(s_pngData, len);
  f.close();
  return wr == len;
}

// ---------------- config portal ----------------
// Every page is served on port 80 straight from the device:
//   /       home: four entries, same layout as a phone settings portal
//   /wifi   settings (WiFi, flight scan, weather, map)
//   /info   device info
//   /ota    firmware upload
//   /exit   reboot
//   /geo    city search; the lookup is done by the device over its station link,
//           because a phone joined to the setup AP has no internet of its own.
static void saveCfg() {
  s_prefs.putInt("ver", CFG_VER);
  s_prefs.putString("ssid", s_ssid);
  s_prefs.putString("pass", s_pass);
  s_prefs.putBool("manual", s_manual);
  s_prefs.putFloat("lat", s_lat);
  s_prefs.putFloat("lon", s_lon);
  s_prefs.putString("city", s_city);
  s_prefs.putString("wcode", s_wcode);
  s_prefs.putInt("rsrc", s_rainSrc);
  s_prefs.putBool("unit_mi", s_unitMi);
  s_prefs.putBool("apt", s_showApt);
  s_prefs.putBool("anim", s_animSweep);
  s_prefs.putString("api", s_api);
}

static String escH(const String &v) {
  String o = v;
  o.replace("&", "&amp;");
  o.replace("<", "&lt;");
  o.replace(">", "&gt;");
  o.replace("\"", "&quot;");
  o.replace("'", "&#39;");
  return o;
}
static String escJ(const String &v) {
  String o = v;
  o.replace("\\", "\\\\");
  o.replace("\"", "\\\"");
  return o;
}

static const char *PORTAL_CSS =
    "body{font-family:-apple-system,'PingFang SC','Microsoft YaHei',sans-serif;"
    "margin:0;padding:16px 16px 28px;background:#f4f6fa;color:#222}"
    "h1{font-size:23px;margin:4px 0 4px;text-align:center;letter-spacing:2px}"
    ".sub{text-align:center;color:#7a8290;font-size:13px;margin:0 0 18px}"
    ".btn{display:block;width:100%;box-sizing:border-box;padding:14px 0;"
    "margin:0 0 12px;background:#1f6fd0;color:#fff;border:0;border-radius:9px;"
    "font-size:17px;font-family:inherit}"
    ".btn:active{background:#175cab}"
    "hr{border:0;border-top:1px solid #dee3ea;margin:8px 0 18px}"
    "fieldset{border:1px solid #cfd6e0;border-radius:10px;background:#fff;"
    "margin:0 0 16px;padding:8px 12px 6px}"
    "legend{font-size:15px;color:#1f6fd0;padding:0 6px}"
    "label.f{display:block;font-size:14px;color:#48505c;margin:10px 0 5px}"
    "input[type=text],input[type=password],select{width:100%;box-sizing:border-box;"
    "padding:10px;font-size:15px;color:#222;background:#fff;"
    "border:1px solid #c3cbd8;border-radius:7px;font-family:inherit}"
    "input[type=file]{width:100%;box-sizing:border-box;padding:8px 0;font-size:14px}"
    ".row{display:flex;align-items:center;margin:10px 0;font-size:14px;color:#333}"
    ".row input{width:18px;height:18px;margin:0 8px 0 0}"
    "#hits{list-style:none;margin:8px 0 0;padding:0}"
    "#hits li{padding:10px 12px;margin:0 0 6px;background:#eef3fb;"
    "border-radius:7px;font-size:14px;color:#1a4a86}"
    ".hint{font-size:12px;color:#8d94a1;margin:6px 0 10px}"
    ".foot{margin-top:16px;font-size:13px;color:#8d94a1;text-align:center}"
    ".kv{font-size:14px;line-height:2;color:#3c434d}"
    ".kv b{color:#1f6fd0;font-weight:600}";

static String pageHead(const char *title, const char *sub) {
  String h = "<!doctype html><html><head><meta charset=utf-8>";
  h += "<meta name=viewport content='width=device-width,initial-scale=1'>";
  h += "<title>";
  h += title;
  h += "</title><style>";
  h += PORTAL_CSS;
  h += "</style></head><body><h1>";
  h += title;
  h += "</h1>";
  if (sub && sub[0]) {
    h += "<div class=sub>";
    h += sub;
    h += "</div>";
  }
  return h;
}

static String linkFoot() {
  if (WiFi.status() == WL_CONNECTED)
    return String("已连接 ") + escH(s_ssid);
  return String("未连接");
}

static void handleHome() {
  String h = pageHead("雷达雨图设置", "PlaneRadar-Setup");
  h += "<button class=btn onclick=\"location.href='/wifi'\">配置无线WiFi连接</button>";
  h += "<button class=btn onclick=\"location.href='/info'\">设备信息</button>";
  h += "<button class=btn onclick=\"location.href='/exit'\">退出</button>";
  h += "<hr>";
  h += "<button class=btn onclick=\"location.href='/ota'\">固件升级</button>";
  h += "<div class=foot>";
  h += linkFoot();
  h += "</div></body></html>";
  s_srv->send(200, "text/html; charset=utf-8", h);
}

static const char *CITY_JS =
    "var qt=0;function qry(){clearTimeout(qt);qt=setTimeout(doq,420);}"
    "function doq(){var q=document.getElementById('q').value.trim();"
    "var ul=document.getElementById('hits');"
    "if(q.length<1){ul.innerHTML='';return;}"
    "fetch('/geo?q='+encodeURIComponent(q)).then(function(r){return r.json();})"
    ".then(function(a){ul.innerHTML='';(a||[]).forEach(function(o){"
    "var li=document.createElement('li');li.textContent=o.n+'  '+(o.a||'');"
    "li.onclick=function(){document.getElementById('city').value=o.n;"
    "document.getElementsByName('lat')[0].value=o.la.toFixed(5);"
    "document.getElementsByName('lon')[0].value=o.lo.toFixed(5);"
    "document.getElementsByName('wcode')[0].value=(o.id||'');"
    "ul.innerHTML='';document.getElementById('q').value='';};"
    "ul.appendChild(li);});}).catch(function(){});}";

static void handleWifi() {
  char lat[16], lon[16];
  snprintf(lat, sizeof(lat), "%.5f", s_lat);
  snprintf(lon, sizeof(lon), "%.5f", s_lon);
  String h = pageHead("雷达雨图设置", "PlaneRadar-Setup");
  h += "<form method=POST action=/save><input type=hidden name=v value=2>";

  h += "<fieldset><legend>WiFi 连接</legend>";
  h += "<label class=f>WiFi 名称</label><input type=text name=ssid value='" +
       escH(s_ssid) + "'>";
  h += "<label class=f>WiFi 密码</label><input type=password name=pass id=pw value='" +
       escH(s_pass) + "'>";
  h += "<div class=row><input type=checkbox id=shw "
       "onclick=\"var p=document.getElementById('pw');"
       "p.type=this.checked?'text':'password'\">"
       "<label for=shw>显示密码</label></div>";
  h += "</fieldset>";

  h += "<fieldset><legend>✈ 航班扫描</legend>";
  h += "<label class=f>雷达纬度 (deg)</label><input type=text name=lat value='" +
       String(lat) + "'>";
  h += "<label class=f>雷达经度 (deg)</label><input type=text name=lon value='" +
       String(lon) + "'>";
  h += "<div class=row><input type=checkbox name=unit";
  h += (s_unitMi ? " checked" : "");
  h += "><label>距离用英里显示</label></div>";
  h += "<div class=row><input type=checkbox name=apt";
  h += (s_showApt ? " checked" : "");
  h += "><label>显示机场跑道</label></div>";
  h += "<div class=row><input type=checkbox name=anim";
  h += (s_animSweep ? " checked" : "");
  h += "><label>启用雷达扫描动画</label></div>";
  h += "<label class=f>API 地址</label><input type=text name=api value='" +
       escH(s_api.length() ? s_api : String(DEF_PLANE_API)) + "'>";
  h += "<div class=hint>可用占位符 {lat} {lon} {nm}，留空则用默认接口</div>";
  h += "</fieldset>";

  h += "<fieldset><legend>☀ 天气预报</legend>";
  h += "<label class=f>天气城市搜索</label>";
  h += "<input type=text id=q placeholder='输入城市名搜索，如 北京 / 北海' oninput='qry()'>";
  h += "<ul id=hits></ul>";
  h += "<div class=hint>选中后自动填入城市名与经纬度</div>";
  h += "<label class=f>屏幕显示城市名</label><input type=text id=city name=city value='" +
       escH(s_city) + "'>";
  h += "<label class=f>Weather City Code</label><input type=text name=wcode value='" +
       escH(s_wcode) + "'>";
  h += "</fieldset>";

  h += "<fieldset><legend>🗺 雨图与定位</legend>";
  h += "<label class=f>开机位置</label><select name=manual>";
  h += "<option value=0";
  h += (s_manual ? "" : " selected");
  h += ">用 IP 估位</option><option value=1";
  h += (s_manual ? " selected" : "");
  h += ">固定用上面的经纬度</option></select>";
  h += "<label class=f>云图源</label><select name=rsrc>";
  h += "<option value=0";
  h += (s_rainSrc == 0 ? " selected" : "");
  h += ">降雨雷达回波</option><option value=1";
  h += (s_rainSrc == 1 ? " selected" : "");
  h += ">红外卫星云图</option></select>";
  h += "</fieldset>";

  h += "<button class=btn type=submit>保存</button></form>";
  h += "<button class=btn type=button onclick='location.reload()'>刷新页面</button>";
  h += "<div class=foot>";
  h += linkFoot();
  h += "</div><script>";
  h += CITY_JS;
  h += "</script></body></html>";
  s_srv->send(200, "text/html; charset=utf-8", h);
}

static void handleInfo() {
  unsigned long up = millis() / 1000;
  String h = pageHead("设备信息", "PlaneRadar-Setup");
  h += "<fieldset><legend>运行状态</legend><div class=kv>";
  h += "<b>固件版本</b> " FW_VER "<br>";
  h += "<b>MAC</b> " + WiFi.macAddress() + "<br>";
  h += "<b>局域网地址</b> " + WiFi.localIP().toString() + "<br>";
  h += "<b>热点地址</b> " + WiFi.softAPIP().toString() + "<br>";
  h += "<b>WiFi 信号</b> " + String(WiFi.RSSI()) + " dBm<br>";
  h += "<b>空闲内存</b> " + String(ESP.getFreeHeap() / 1024) + " KB<br>";
  h += "<b>最大连续块</b> " + String(ESP.getMaxAllocHeap() / 1024) + " KB<br>";
  h += "<b>运行时长</b> " + String(up / 3600) + " 小时 " +
       String((up / 60) % 60) + " 分<br>";
  h += "<b>飞机数量</b> " + String(s_planeCount) + "<br>";
  h += "<b>云图帧数</b> " + String(s_frameN) + "（" +
       String(s_rainSrc ? "红外卫星" : "雷达回波") + "）<br>";
  h += "<b>天气耗时</b> " + String((unsigned long)s_metCostMs) + " ms　";
  h += "<b>雨图耗时</b> " + String((unsigned long)s_rainCostMs) + " ms<br>";
  h += "<b>距离单位</b> " + String(s_unitMi ? "英里" : "公里") + "　";
  h += "<b>扫描动画</b> " + String(s_animSweep ? "开" : "关") + "　";
  h += "<b>机场跑道</b> " + String(s_showApt ? "开" : "关") + "<br>";
  h += "</div></fieldset>";
  h += "<button class=btn onclick=\"location.href='/wifi'\">返回设置</button>";
  h += "<button class=btn onclick=\"location.href='/'\">返回首页</button>";
  h += "<div class=foot>";
  h += linkFoot();
  h += "</div></body></html>";
  s_srv->send(200, "text/html; charset=utf-8", h);
}

static void handleOta() {
  String h = pageHead("固件升级", "PlaneRadar-Setup");
  h += "<fieldset><legend>当前版本</legend><div class=kv>" FW_VER "</div></fieldset>";
  h += "<form method=POST action=/update enctype=multipart/form-data>";
  h += "<fieldset><legend>选择固件文件</legend>";
  h += "<input type=file name=update accept='.bin'>";
  h += "<div class=hint>选 app.bin（应用镜像），不要用整片合并的 dump</div>";
  h += "</fieldset><button class=btn type=submit>上传并升级</button></form>";
  h += "<div class=hint style='text-align:center'>升级完成后设备自动重启，请等 10 秒再访问</div>";
  h += "<button class=btn onclick=\"location.href='/'\">返回首页</button>";
  h += "</body></html>";
  s_srv->send(200, "text/html; charset=utf-8", h);
}

static void handleExit() {
  String h = pageHead("已退出", "PlaneRadar-Setup");
  h += "<div class=foot style='margin-top:40px;font-size:15px'>设备重启中，热点稍后自动恢复</div>";
  h += "</body></html>";
  s_srv->send(200, "text/html; charset=utf-8", h);
  delay(600);
  ESP.restart();
}

static void handleGeocode() {
  String q = s_srv->hasArg("q") ? s_srv->arg("q") : String("");
  q.trim();
  if (q.length() == 0) {
    s_srv->send(200, "application/json", "[]");
    return;
  }
  // Free, keyless geocoder (Open-Meteo). Done on the device: the phone that
  // joined the setup AP has no internet of its own.
  String url =
      "https://geocoding-api.open-meteo.com/v1/search?count=6&language=zh&format=json&name=";
  url += q;
  url.replace(" ", "%20");
  WiFiClientSecure tls;
  tls.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(6000);
  http.setTimeout(8000);
  http.begin(tls, url);
  String out = "[]";
  int code = http.GET();
  if (code == 200) {
    String body = http.getString();
    JsonDocument doc;
    if (!deserializeJson(doc, body)) {
      JsonArray rs = doc["results"];
      String js = "[";
      int n = 0;
      for (JsonObject r : rs) {
        if (n >= 6) break;
        String nm = String((const char *)(r["name"] | ""));
        String ad = "";
        const char *a1 = r["admin1"];
        const char *c1 = r["country"];
        if (a1 && a1[0]) ad = String(a1);
        if (c1 && c1[0]) {
          if (ad.length()) ad += " ";
          ad += String(c1);
        }
        float la = r["latitude"] | 0.0f;
        float lo = r["longitude"] | 0.0f;
        long id = r["id"] | 0;
        if (nm.length() == 0) continue;
        if (n++) js += ",";
        js += "{\"n\":\"" + escJ(nm) + "\",\"a\":\"" + escJ(ad) + "\",\"la\":" +
              String(la, 5) + ",\"lo\":" + String(lo, 5) + ",\"id\":\"" +
              String(id) + "\"}";
      }
      js += "]";
      out = js;
      Serial.printf("[geo] %s -> %d found\n", q.c_str(), n);
    }
  } else {
    Serial.printf("[geo] http %d\n", code);
  }
  http.end();
  s_srv->send(200, "application/json; charset=utf-8", out);
}

static void handleSave() {
  bool full = s_srv->hasArg("v"); // the new form sends everything, old posts do not
  if (s_srv->hasArg("ssid")) s_ssid = s_srv->arg("ssid");
  if (s_srv->hasArg("pass")) s_pass = s_srv->arg("pass");
  if (s_srv->hasArg("city")) {
    String c = s_srv->arg("city");
    c.trim();
    if (c.length()) s_city = c;
  }
  if (s_srv->hasArg("wcode")) {
    String c = s_srv->arg("wcode");
    c.trim();
    s_wcode = c;
  }
  if (s_srv->hasArg("lat")) {
    float v = (float)atof(s_srv->arg("lat").c_str());
    if (v >= -90.0f && v <= 90.0f) s_lat = v;
  }
  if (s_srv->hasArg("lon")) {
    float v = (float)atof(s_srv->arg("lon").c_str());
    if (v >= -180.0f && v <= 180.0f) s_lon = v;
  }
  if (s_srv->hasArg("api")) {
    String a = s_srv->arg("api");
    a.trim();
    s_api = a;
  }
  if (s_srv->hasArg("manual")) s_manual = s_srv->arg("manual").toInt() != 0;
  if (s_srv->hasArg("rsrc")) s_rainSrc = s_srv->arg("rsrc").toInt();
  if (full) {
    s_unitMi = s_srv->hasArg("unit");
    s_showApt = s_srv->hasArg("apt");
    s_animSweep = s_srv->hasArg("anim");
  }
  saveCfg();
  Serial.printf("[cfg] saved ssid=%s city=%s lat=%.4f lon=%.4f manual=%d rsrc=%d"
                " unit=%d apt=%d anim=%d api=%s\n",
                s_ssid.c_str(), s_city.c_str(), s_lat, s_lon, (int)s_manual,
                s_rainSrc, (int)s_unitMi, (int)s_showApt, (int)s_animSweep,
                s_api.length() ? s_api.c_str() : DEF_PLANE_API);
  String h = pageHead("已保存", "PlaneRadar-Setup");
  h += "<div class=foot style='margin-top:40px;font-size:15px'>设置已保存，设备重启中，请等 10 秒</div>";
  h += "</body></html>";
  s_srv->send(200, "text/html; charset=utf-8", h);
  delay(800);
  ESP.restart();
}

// ---------------- OTA over HTTP ----------------
// Update writes straight into the inactive app slot, so the 1.3MB image never
// has to sit in RAM. Verified by the bootloader (elf-sha256) on next boot.
static void handleUpdateUpload() {
  HTTPUpload &up = s_srv->upload();
  if (up.status == UPLOAD_FILE_START) {
    Serial.printf("[ota] start %s\n", up.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
      Serial.printf("[ota] begin fail err=%u\n", Update.getError());
  } else if (up.status == UPLOAD_FILE_WRITE) {
    size_t wr = Update.write(up.buf, up.currentSize);
    if (wr != up.currentSize)
      Serial.printf("[ota] write short %u of %u\n", (unsigned)wr,
                    (unsigned)up.currentSize);
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true))
      Serial.printf("[ota] end ok %u bytes\n", (unsigned)up.totalSize);
    else
      Serial.printf("[ota] end fail err=%u\n", Update.getError());
  }
}
static void handleUpdateDone() {
  bool ok = !Update.hasError();
  String h = pageHead(ok ? "升级完成" : "升级失败", "PlaneRadar-Setup");
  h += "<div class=foot style='margin-top:40px;font-size:15px'>";
  h += (ok ? "写入成功，设备重启中，请等 10 秒" : "写入失败，固件未改动，可重试");
  h += "</div></body></html>";
  s_srv->send(200, "text/html; charset=utf-8", h);
  if (ok) {
    delay(600);
    ESP.restart();
  }
}

static void startWeb() {
  if (s_srv) return;
  // WebServer(int port): 8 would listen on port 8, unreachable from a browser.
  s_srv = new WebServer(80);
  s_srv->on("/", HTTP_GET, handleHome);
  s_srv->on("/wifi", HTTP_GET, handleWifi);
  s_srv->on("/info", HTTP_GET, handleInfo);
  s_srv->on("/ota", HTTP_GET, handleOta);
  s_srv->on("/exit", HTTP_GET, handleExit);
  s_srv->on("/geo", HTTP_GET, handleGeocode);
  s_srv->on("/save", HTTP_POST, handleSave);
  s_srv->on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  s_srv->begin();
  Serial.println("[web] ready");
}
static void webLoop() {
  if (s_srv) s_srv->handleClient();
}

static void startPortal() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("PlaneRadar-Setup");
  startWeb();
  Serial.println("AP PlaneRadar-Setup up, http://192.168.4.1");
  useFont(u8g2_font_helvB10_tr);
  gfx->fillScreen(C_BLACK);
  txtC("SETUP WIFI", 88, C_YELLOW);
  txtC("PlaneRadar-Setup", 110, C_WHITE);
  txtC("192.168.4.1", 132, C_CYAN);
  useFont(u8g2_font_unifont_t_chinese4);
  txtC("手机连它再打开网页", 154, C_GRAY);
  while (true) {
    s_srv->handleClient();
    delay(10);
  }
}

// Reverse-geocode a coordinate to a proper Chinese county/city name.
// BigDataCloud free client endpoint (no key): locality = county/district,
// city = prefecture-level city. Avoids street-level names such as "金融街".
static String reverseName(double la, double lo) {
  char url[170];
  snprintf(url, sizeof(url),
           "https://api.bigdatacloud.net/data/reverse-geocode-client?"
           "latitude=%.5f&longitude=%.5f&localityLanguage=zh",
           la, lo);
  WiFiClientSecure tls;
  tls.setInsecure();
  HTTPClient http;
  http.begin(tls, url);
  String name = "";
  if (http.GET() == 200) {
    String body = http.getString();
    JsonDocument doc;
    if (!deserializeJson(doc, body)) {
      const char *loc = doc["locality"];
      const char *city = doc["city"];
      if (loc && loc[0]) name = String(loc);
      else if (city && city[0]) name = String(city);
    }
  }
  http.end();
  return name;
}

// ---------------- IP locate ----------------
static void locateByIp() {
  HTTPClient http;
  // lang=zh-CN -> Chinese city/region names
  http.begin("http://ip-api.com/json/?fields=status,message,city,district,regionName,lat,lon&lang=zh-CN");
  int code = http.GET();
  if (code == 200) {
    String body = http.getString();
    JsonDocument doc;
    if (!deserializeJson(doc, body)) {
      const char *st = doc["status"];
      if (st && strcmp(st, "success") == 0) {
        float la = doc["lat"] | s_lat;
        float lo = doc["lon"] | s_lon;
        const char *ci = doc["city"];
        const char *rg = doc["regionName"];
        s_lat = la;
        s_lon = lo;
        // Prefer the proper county/city name from reverse geocoding;
        // fall back to ip-api city, then region, then keep current.
        String rn = reverseName(s_lat, s_lon);
        if (rn.length()) s_city = rn;
        else if (ci && ci[0]) s_city = String(ci);
        else if (rg && rg[0]) s_city = String(rg);
        saveCfg();
        Serial.printf("[loc] %s %.4f %.4f\n", s_city.c_str(), s_lat, s_lon);
      } else {
        Serial.println("[loc] status fail, keep center");
      }
    }
  } else {
    Serial.printf("[loc] http %d, keep center\n", code);
  }
  http.end();
}

// ---------------- NTP ----------------
static void syncTime() {
  configTzTime("CST-8", "ntp.aliyun.com", "cn.pool.ntp.org");
  int tries = 0;
  while (!s_timeOk && tries < 40) {
    if (getLocalTime(&s_tm, 500)) s_timeOk = true;
    tries++;
  }
  if (s_timeOk) Serial.printf("[ntp] ok %04d-%02d-%02d\n",
                              s_tm.tm_year + 1900, s_tm.tm_mon + 1, s_tm.tm_mday);
}
