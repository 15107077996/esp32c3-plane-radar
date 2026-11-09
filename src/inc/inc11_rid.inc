// ============================================================================
// Drone Remote ID detection (ASTM F3411-22a / GB 46750 broadcast RID)
//
// Passive only: nothing is transmitted. Two transports are listened to:
//   * Bluetooth LE advertisements carrying service data UUID 0xFFFA
//     (legacy advertisements and BT5 extended advertisements alike)
//   * 802.11 beacon / probe-response frames carrying the ASTM vendor specific
//     information element (OUI FA:0B:BC), which is what most Chinese drones
//     use now that GB 46750 is in force
//
// Every decoded message updates a small table keyed by the sender MAC. The
// table feeds both the round screen page and the /rid web page.
// ============================================================================
#include <NimBLEDevice.h>
#include "esp_wifi.h"

#define RID_CHANNEL 6          // 2.4 GHz channel most RID beacons sit on
#define RID_STALE_MS 45000UL   // drop an entry that stopped broadcasting
#define RID_QUEUE_N 8          // messages copied out of the radio callbacks

static void ridPage();            // web handler, defined at the bottom
static bool s_ridWebHooked = false;

// One detected drone. Kept small: 12 of them plus the shared buffers have to
// fit beside the web server and the TLS client on a 400 KB part.
struct RidDrone {
  char mac[13];
  char uasId[21];
  char opId[21];
  float lat, lon;
  float opLat, opLon;
  float altM, heightM;
  float spdMs, dirDeg;
  int8_t rssi;
  uint8_t uaType;    // 0 none, 1 aeroplane, 2 multirotor ...
  uint8_t idType;    // 1 serial number, 2 CAA reg, 3 UUID ...
  uint8_t status;    // 0 undeclared, 1 ground, 2 airborne, 3 emergency
  uint8_t transport; // 1 BLE, 2 WiFi beacon
  uint16_t seen;     // bitmask of message types received
  unsigned long lastMs;
};

static RidDrone s_rid[RID_MAX];

// ---- cross-context queue -------------------------------------------------
// Both the BLE host callback and the WiFi promiscuous callback run outside the
// Arduino loop, so they only copy a finished message into this ring and let
// ridTick() do the parsing in normal context.
struct RidMsg {
  char mac[13];
  uint8_t raw[25];
  int8_t rssi;
  uint8_t tr;
};
static RidMsg s_ridQ[RID_QUEUE_N];
static volatile int s_ridQHead = 0;
static volatile int s_ridQTail = 0;

static void ridPush(const uint8_t *m, const char *mac, int8_t rssi, uint8_t tr) {
  int h = s_ridQHead;
  int n = (h + 1) % RID_QUEUE_N;
  if (n == s_ridQTail) return; // queue full: drop, the next 1 Hz burst replaces it
  memcpy(s_ridQ[h].raw, m, 25);
  memcpy(s_ridQ[h].mac, mac, 13);
  s_ridQ[h].rssi = rssi;
  s_ridQ[h].tr = tr;
  s_ridQHead = n;
}

// Trim a fixed-width ASCII field: drop trailing spaces/NULs and anything that
// is not printable, so the web table never shows garbage.
static void ridCopyId(char *dst, const uint8_t *src, int n) {
  int k = 0;
  for (int i = 0; i < n && k < n; i++) {
    uint8_t c = src[i];
    if (c == 0 || c == ' ') { if (k == 0) continue; else break; }
    if (c < 0x20 || c > 0x7E) continue;
    dst[k++] = (char)c;
  }
  dst[k] = 0;
}

// ---- parsing -------------------------------------------------------------
static RidDrone *ridSlot(const char *mac) {
  for (int i = 0; i < s_ridCount; i++)
    if (strcmp(s_rid[i].mac, mac) == 0) return &s_rid[i];
  if (s_ridCount < RID_MAX) return &s_rid[s_ridCount++];
  // table full: recycle the least recently heard entry
  int old = 0;
  for (int i = 1; i < RID_MAX; i++)
    if ((long)(s_rid[i].lastMs - s_rid[old].lastMs) < 0) old = i;
  memset(&s_rid[old], 0, sizeof(RidDrone));
  return &s_rid[old];
}

static void ridApply(const uint8_t *m, const char *mac, int8_t rssi, uint8_t tr) {
  RidDrone *d = ridSlot(mac);
  if (!d) return;
  if (d->mac[0] == 0) memcpy(d->mac, mac, 13);
  d->lastMs = millis();
  d->rssi = rssi;
  if (tr) d->transport = tr;
  s_ridFrames++;

  uint8_t t = m[0] >> 4;
  d->seen |= (uint16_t)(1u << (t & 0x0F));

  switch (t) {
    case 0: { // Basic ID
      d->uaType = m[1] & 0x0F;
      d->idType = m[1] >> 4;
      ridCopyId(d->uasId, m + 2, 20);
      break;
    }
    case 1: { // Location / Vector
      uint8_t flags = m[1];
      d->status = (flags >> 4) & 0x0F;
      int32_t v;
      memcpy(&v, m + 5, 4); d->lat = (float)(v * 1e-7);
      memcpy(&v, m + 9, 4); d->lon = (float)(v * 1e-7);
      d->altM = (float)((uint16_t)m[13] | ((uint16_t)m[14] << 8)) - 1000.0f;
      d->heightM = (float)((uint16_t)m[17] | ((uint16_t)m[18] << 8)) - 1000.0f;
      if (m[2] <= 179) d->dirDeg = (float)m[2] * 2.0f;
      if (m[3] <= 254) d->spdMs = (flags & 1) ? (float)m[3] * 0.25f : (float)m[3];
      break;
    }
    case 4: { // System (operator location)
      int32_t v;
      memcpy(&v, m + 2, 4); d->opLat = (float)(v * 1e-7);
      memcpy(&v, m + 6, 4); d->opLon = (float)(v * 1e-7);
      break;
    }
    case 5: { // Operator ID
      ridCopyId(d->opId, m + 2, 20);
      break;
    }
    default:
      break;
  }
}

// A 25 byte ODID message can also arrive as a pack of up to 9 messages.
static void ridFeed(const uint8_t *d, int n, const char *mac, int8_t rssi, uint8_t tr) {
  if (n < 2) return;
  uint8_t t = d[0] >> 4;
  if (t == 0x0F) {
    if (n < 3) return;
    uint8_t sz = d[1], cnt = d[2];
    if (sz != 25 || cnt == 0 || cnt > 9) return;
    for (int k = 0; k < cnt; k++) {
      int off = 3 + k * 25;
      if (off + 25 > n) break;
      ridPush(&d[off], mac, rssi, tr);
    }
    return;
  }
  if (t <= 5 && n >= 25) ridPush(d, mac, rssi, tr);
}

// ---- Bluetooth LE --------------------------------------------------------
static void ridBlePayload(const std::string &pl, const char *mac, int rssi) {
  size_t i = 0;
  while (i + 1 < pl.size()) {
    uint8_t len = pl[i];
    if (len == 0) break;
    if (i + 1 + len > pl.size()) break;
    uint8_t type = pl[i + 1];
    if (type == 0x16 && len >= 4) { // service data, 16-bit UUID
      if ((uint8_t)pl[i + 2] == 0xFA && (uint8_t)pl[i + 3] == 0xFF) {
        s_ridBleN++;
        ridFeed((const uint8_t *)&pl[i + 4], len - 3, mac, (int8_t)rssi, 1);
      }
    }
    i += 1 + len;
  }
}

class RidScanCb : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice *dev) override {
    std::string pl = dev->getPayload();
    ridBlePayload(pl, dev->getAddress().toString().c_str(), dev->getRSSI());
  }
};
static RidScanCb *s_ridCb = nullptr;

// ---- 802.11 beacon / probe response --------------------------------------
// Beacon body: 24 byte header + 12 byte fixed fields, then tagged IEs.
// The ASTM payload sits in a vendor specific IE: DD len FA 0B BC 0D <msg...>.
static void ridWifiFrame(const uint8_t *f, int sigLen) {
  uint8_t fc = f[0];
  if ((fc & 0xF0) != 0x80 && (fc & 0xF0) != 0x50) return;
  int i = 24 + 12;
  int end = sigLen - 4; // drop the FCS
  if (end > 1500) end = 1500;
  while (i + 2 <= end) {
    uint8_t id = f[i];
    uint8_t elen = f[i + 1];
    if (i + 2 + elen > end) break;
    if (id == 0xDD && elen >= 4) {
      const uint8_t *ie = &f[i + 2];
      if (ie[0] == 0xFA && ie[1] == 0x0B && ie[2] == 0xBC) {
        s_ridWifiN++;
        const uint8_t *d = ie + 3;
        int dn = elen - 3;
        if (dn >= 4 && d[0] == 0x0D) { d++; dn--; }
        char mac[13];
        snprintf(mac, sizeof(mac), "%02x:%02x:%02x:%02x:%02x:%02x",
                 f[10], f[11], f[12], f[13], f[14], f[15]);
        ridFeed(d, dn, mac, 0, 2);
      } else if (s_ridOuiLog < 6) {
        s_ridOuiLog++;
        Serial.printf("[rid] vendor ie oui=%02X%02X%02X len=%d\n",
                      ie[0], ie[1], ie[2], elen);
      }
    }
    i += 2 + elen;
  }
}

static void ridWifiCb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
  ridWifiFrame(p->payload, p->rx_ctrl.sig_len);
}

// ---- start / stop --------------------------------------------------------
static void ridStart() {
  if (s_ridOn) return;
  releasePngBuf();            // give the 64 KB PNG scratch back while scanning
  WiFi.mode(WIFI_AP);         // station off: the radio belongs to the sniffer
  delay(80);
  WiFi.softAP("PlaneRadar-Setup", NULL, RID_CHANNEL);
  esp_wifi_set_channel(RID_CHANNEL, WIFI_SECOND_CHAN_NONE);

  s_ridQHead = 0;
  s_ridQTail = 0;
  s_ridCount = 0;
  s_ridFrames = 0;
  s_ridBleN = 0;
  s_ridWifiN = 0;
  s_ridOuiLog = 0;

  wifi_promiscuous_filter_t pf;
  pf.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&pf);
  esp_wifi_set_promiscuous_rx_cb(&ridWifiCb);
  esp_wifi_set_promiscuous(true);

  if (!s_ridCb) s_ridCb = new RidScanCb();
  NimBLEDevice::init("PlaneRadar-RID");
  NimBLEScan *sc = NimBLEDevice::getScan();
  sc->setAdvertisedDeviceCallbacks(s_ridCb, true);
  sc->setActiveScan(false);
  sc->setInterval(100);
  sc->setWindow(99);
  sc->start(0, false);
  s_ridBle = true;

  s_ridOn = true;
  s_ridStartMs = millis();
  Serial.printf("[rid] scan ch%d heap=%u max=%u\n", RID_CHANNEL,
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
}

static void ridStop() {
  if (!s_ridOn) return;
  esp_wifi_set_promiscuous(false);
  if (s_ridBle) {
    NimBLEScan *sc = NimBLEDevice::getScan();
    if (sc) sc->stop();
    NimBLEDevice::deinit(true);
    s_ridBle = false;
  }
  s_ridOn = false;
  s_ridCount = 0;
  // Back to AP+STA; the AP is restarted on channel 1 first so the station can
  // pick the router channel again instead of being pinned to channel 6.
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("PlaneRadar-Setup", NULL, 1);
  WiFi.setSleep(false);
  if (s_ssid.length()) WiFi.begin(s_ssid.c_str(), s_pass.c_str());
  Serial.printf("[rid] stop: %d ble, %d wifi, %d msgs, heap=%u\n",
                s_ridBleN, s_ridWifiN, s_ridFrames, (unsigned)ESP.getFreeHeap());
}

// ---- housekeeping --------------------------------------------------------
static float ridDistKm(const RidDrone &d) {
  if (d.lat == 0.0f && d.lon == 0.0f) return -1.0f;
  return haversineKm(s_lat, s_lon, d.lat, d.lon);
}

static void ridTick(unsigned long now) {
  // Attach the web routes once the server exists. startWeb() is compiled earlier
  // (inc02_util.inc), but WebServer accepts new handlers at any time, so the
  // whole RID web UI can live here and inc02_util.inc stays untouched.
  if (!s_ridWebHooked && s_srv) {
    s_srv->on("/rid", HTTP_GET, ridPage);
    s_srv->on("/rid.json", HTTP_GET, ridPage);
    s_ridWebHooked = true;
    Serial.println("[rid] web routes /rid /rid.json");
  }
  if (!s_ridOn) return;
  while (s_ridQTail != s_ridQHead) {
    RidMsg *q = &s_ridQ[s_ridQTail];
    ridApply(q->raw, q->mac, q->rssi, q->tr);
    s_ridQTail = (s_ridQTail + 1) % RID_QUEUE_N;
  }
  // expire entries that went quiet
  for (int i = 0; i < s_ridCount;) {
    if (now - s_rid[i].lastMs > RID_STALE_MS) {
      for (int k = i; k + 1 < s_ridCount; k++) s_rid[k] = s_rid[k + 1];
      s_ridCount--;
    } else {
      i++;
    }
  }
}

// ---- screen page ---------------------------------------------------------
static const char *ridUaName(uint8_t t) {
  switch (t) {
    case 1: return "固定翼";
    case 2: return "多旋翼";
    case 4: return "复合翼";
    case 6: return "滑翔机";
    case 15: return "其他";
    default: return "未知";
  }
}

static void drawRid() {
  gfx->fillScreen(C_BLACK);
  gfx->drawCircle(R_CX, R_CY, R_R, C_DKGREEN);
  useFont(u8g2_font_unifont_t_chinese4);
  txtC("无人机侦测", 28, C_YELLOW);

  char l[44];
  useFont(u8g2_font_helvB08_tr);
  if (!s_ridOn) {
    txtC("未开启", 48, C_GRAY);
  } else {
    snprintf(l, sizeof(l), "BLE %d  BEACON %d", s_ridBleN, s_ridWifiN);
    txtC(l, 48, C_CYAN);
  }

  useFont(u8g2_font_logisoso24_tn);
  snprintf(l, sizeof(l), "%d", s_ridCount);
  txtC(l, 96, s_ridCount ? C_GREEN : C_GRAY);
  useFont(u8g2_font_helvB08_tr);
  txtC("架 RID 广播", 112, C_GRAY);

  // three strongest entries (RSSI order, ties keep table order)
  int chosen[3] = {-1, -1, -1};
  int nshow = 0;
  for (int s = 0; s < 3; s++) {
    int best = -1;
    for (int i = 0; i < s_ridCount; i++) {
      bool taken = false;
      for (int k = 0; k < s; k++)
        if (chosen[k] == i) { taken = true; break; }
      if (taken) continue;
      if (best < 0 || s_rid[i].rssi > s_rid[best].rssi) best = i;
    }
    if (best < 0) break;
    chosen[s] = best;
    nshow++;
  }

  useFont(u8g2_font_helvB08_tr);
  for (int s = 0; s < nshow; s++) {
    RidDrone &d = s_rid[chosen[s]];
    float km = ridDistKm(d);
    char id[14];
    snprintf(id, sizeof(id), "%s", d.uasId[0] ? d.uasId : d.mac);
    if (strlen(id) > 12) id[12] = 0;
    if (km >= 0.0f) snprintf(l, sizeof(l), "%s %.1fkm %ddBm", id, km, d.rssi);
    else snprintf(l, sizeof(l), "%s %ddBm", id, d.rssi);
    txtC(l, 130 + s * 18, C_WHITE);
  }

  useFont(u8g2_font_unifont_t_chinese4);
  txtC("手机 192.168.4.1/rid", 196, C_CYAN);
}

// ---- JSON for the web page ----------------------------------------------
static void ridJson(String &out) {
  out = "{\"on\":";
  out += s_ridOn ? 1 : 0;
  out += ",\"ble\":";
  out += String(s_ridBleN);
  out += ",\"wifi\":";
  out += String(s_ridWifiN);
  out += ",\"frames\":";
  out += String(s_ridFrames);
  out += ",\"up\":";
  out += String((millis() - s_ridStartMs) / 1000);
  out += ",\"n\":";
  out += String(s_ridCount);
  out += ",\"list\":[";
  for (int i = 0; i < s_ridCount; i++) {
    RidDrone &d = s_rid[i];
    if (i) out += ",";
    char buf[64];
    out += "{\"mac\":\""; out += d.mac;
    out += "\",\"id\":\""; out += d.uasId;
    out += "\",\"op\":\""; out += d.opId;
    out += "\",\"ua\":\""; out += ridUaName(d.uaType);
    out += "\",\"st\":"; out += String(d.status);
    out += ",\"la\":"; out += String(d.lat, 6);
    out += ",\"lo\":"; out += String(d.lon, 6);
    out += ",\"alt\":"; out += String(d.altM, 0);
    out += ",\"h\":"; out += String(d.heightM, 0);
    out += ",\"spd\":"; out += String(d.spdMs, 1);
    out += ",\"dir\":"; out += String(d.dirDeg, 0);
    out += ",\"ola\":"; out += String(d.opLat, 6);
    out += ",\"olo\":"; out += String(d.opLon, 6);
    float km = ridDistKm(d);
    out += ",\"km\":"; out += String(km, 2);
    out += ",\"rssi\":"; out += String(d.rssi);
    out += ",\"tr\":"; out += String(d.transport);
    out += ",\"age\":"; out += String((millis() - d.lastMs) / 1000);
    snprintf(buf, sizeof(buf), ",\"seen\":%u}", (unsigned)d.seen);
    out += buf;
  }
  out += "]}";
}

// ---- web page -------------------------------------------------------------
// /rid        the phone page (mode buttons + live table)
// /rid.json   the same data as JSON, polled every 2 s by the page itself
static const char *RID_JS =
    "var SMS=['未声明','地面','空中','紧急'];";
    "function pv(v){return (v===undefined||v===null||v==='')?'-':v;}";
    "function st(){fetch('/rid.json',{cache:'no-store'}).then(function(r){return r.json();})";
    ".then(function(d){";
    "var s=document.getElementById('st');";
    "s.textContent=(d.on?'侦测中':'未开启')+' | BLE '+d.ble+' 帧 | Beacon '+d.wifi";
    "+' 帧 | 解码 '+d.frames+' 条 | '+d.n+' 架';";
    "var tb=document.getElementById('tb');tb.innerHTML='';";
    "(d.list||[]).forEach(function(o){";
    "var c=document.createElement('div');c.className='rcard';";
    "var t=document.createElement('div');t.className='rt';";
    "t.textContent=pv(o.id||o.mac);c.appendChild(t);";
    "function row(k,v){var p=document.createElement('div');p.className='rr';";
    "p.textContent=k+'：'+pv(v);c.appendChild(p);}";
    "row('状态 / 机型',(SMS[o.st]||'未知')+' · '+pv(o.ua));";
    "row('距离',o.km>0?o.km+' km':'-');";
    "row('高度 (相对地面)',o.alt>-1000?o.alt+' m ('+(o.h>-1000?o.h+' m':'-')+')':'-');";
    "row('速度 / 航向',o.spd>=0?o.spd+' m/s · '+o.dir+'°':'-');";
    "row('无人机位置',o.la?o.la+', '+o.lo:'未广播');";
    "row('操作者ID',o.op||'未广播');";
    "row('操作者位置',o.ola?o.ola+', '+o.olo:'未广播');";
    "row('信号 / 方式',pv(o.rssi)+' dBm · '+(o.tr==2?'WiFi Beacon':'BLE'));";
    "row('更新 / 来源',o.age+' 秒前 · '+pv(o.mac));";
    "tb.appendChild(c);});";
    "if(!(d.list||[]).length){var e=document.createElement('div');e.className='rcard';";
    "e.textContent=d.on?'暂未收到 RID 广播：把设备放在开阔处，等无人机通电/起飞后再看':'侦测未开启，点上面的按钮';";
    "tb.appendChild(e);}";
    "}).catch(function(){});}";
    "setInterval(st,2000);st();";

static void ridPage() {
  if (s_srv->uri() == "/rid.json") {
    String js;
    ridJson(js);
    s_srv->send(200, "application/json; charset=utf-8", js);
    return;
  }
  if (s_srv->hasArg("on")) {
    s_ridWant = true;
    if (!s_ridOn) ridStart();
  }
  if (s_srv->hasArg("off")) {
    s_ridWant = false;
    ridStop();
  }
  String h = pageHead("无人机侦测", "PlaneRadar-Setup");
  h += "<style>.rcard{background:#fff;border:1px solid #dde3ec;border-radius:10px;";
  h += "padding:10px 12px;margin:0 0 10px;font-size:14px;color:#28303b}";
  h += ".rt{font-size:16px;color:#1f6fd0;font-weight:600;margin-bottom:4px}";
  h += ".rr{line-height:1.9}</style>";
  h += "<div class=kv id=st>读取中…</div>";
  h += "<button class=btn onclick=\"location.href='/rid?on=1'\">开启侦测模式</button>";
  h += "<button class=btn style='background:#6b7280' ";
  h += "onclick=\"location.href='/rid?off=1'\">停止侦测，恢复联网</button>";
  h += "<div class=hint>开启后设备暂停联网（航班与云图停更），只保留热点 ";
  h += "PlaneRadar-Setup 和本页；手机连该热点后访问 192.168.4.1/rid。停止后自动恢复联网。</div>";
  h += "<div id=tb></div>";
  h += RID_JS;
  h += "<div class=foot>被动接收 ASTM F3411 / GB 46750 广播 RID，不发射任何信号</div>";
  h += "</body></html>";
  s_srv->send(200, "text/html; charset=utf-8", h);
}
