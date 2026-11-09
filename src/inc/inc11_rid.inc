// ============================================================================
// Drone Remote ID detection (ASTM F3411-22a / GB 46750 broadcast RID)
//
// Passive only: nothing is transmitted. Two transports are listened to:
//   * Bluetooth LE advertisements carrying service data UUID 0xFFFA
//   * 802.11 beacon / probe-response frames carrying the ASTM vendor specific
//     information element (OUI FA:0B:BC)
// ============================================================================
#ifdef local
#undef local
#endif
#include <NimBLEDevice.h>
#include <esp_bt.h>
#include "esp_wifi.h"

#define RID_CHANNEL 6
#define RID_STALE_MS 45000UL
#define RID_QUEUE_N 8
#define RID_AUTO_STOP_MS 600000UL

static void ridPage();
static void ridJson(String &out);
static bool s_ridWebHooked = false;
static unsigned long s_ridLastPoll = 0;
static bool s_ridBleWant = true;
static bool s_ridWifiWant = true;
static bool s_ridBleTaskRun = false;
static bool s_ridOffReq = false;
static unsigned long s_ridWantMs = 0;
static unsigned long s_ridHbdMs = 0;
static bool s_ridInjectReq = false;

struct RidDrone {
  char mac[13];
  char uasId[21];
  char opId[21];
  float lat, lon;
  float opLat, opLon;
  float altM, heightM;
  float spdMs, dirDeg;
  int8_t rssi;
  uint8_t uaType;
  uint8_t idType;
  uint8_t status;
  uint8_t transport;
  uint16_t seen;
  unsigned long lastMs;
};

static RidDrone s_rid[RID_MAX];

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
  if (n == s_ridQTail) return;
  memcpy(s_ridQ[h].raw, m, 25);
  memcpy(s_ridQ[h].mac, mac, 13);
  s_ridQ[h].rssi = rssi;
  s_ridQ[h].tr = tr;
  s_ridQHead = n;
}

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

static RidDrone *ridSlot(const char *mac) {
  for (int i = 0; i < s_ridCount; i++)
    if (strcmp(s_rid[i].mac, mac) == 0) return &s_rid[i];
  if (s_ridCount < RID_MAX) return &s_rid[s_ridCount++];
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
    case 0: {
      d->uaType = m[1] & 0x0F;
      d->idType = m[1] >> 4;
      ridCopyId(d->uasId, m + 2, 20);
      break;
    }
    case 1: {
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
    case 4: {
      int32_t v;
      memcpy(&v, m + 2, 4); d->opLat = (float)(v * 1e-7);
      memcpy(&v, m + 6, 4); d->opLon = (float)(v * 1e-7);
      break;
    }
    case 5: {
      ridCopyId(d->opId, m + 2, 20);
      break;
    }
    default:
      break;
  }
}

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

static void ridBlePayload(const uint8_t *pl, size_t plLen, const char *mac, int rssi) {
  size_t i = 0;
  while (i + 1 < plLen) {
    uint8_t len = pl[i];
    if (len == 0) break;
    if (i + 1 + len > plLen) break;
    uint8_t type = pl[i + 1];
    if (type == 0x16 && len >= 4) {
      if ((uint8_t)pl[i + 2] == 0xFA && (uint8_t)pl[i + 3] == 0xFF) {
        s_ridBleN++;
        ridFeed(&pl[i + 4], (int)(len - 3), mac, (int8_t)rssi, 1);
      }
    }
    i += 1 + len;
  }
}

class RidScanCb : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice *dev) override {
    char addr[13];
    const uint8_t *v = dev->getAddress().getNative();
    snprintf(addr, sizeof(addr), "%02x:%02x:%02x:%02x:%02x:%02x",
             v[0], v[1], v[2], v[3], v[4], v[5]);
    ridBlePayload(dev->getPayload(), dev->getPayloadLength(), addr, dev->getRSSI());
  }
};
static RidScanCb *s_ridCb = nullptr;

static void ridWifiFrame(const uint8_t *f, int sigLen) {
  uint8_t fc = f[0];
  if ((fc & 0xF0) != 0x80 && (fc & 0xF0) != 0x50) return;
  int i = 24 + 12;
  int end = sigLen - 4;
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

static void ridBleTask(void *arg) {
  Serial.printf("[rid] s4a task up heap=%u\n", (unsigned)ESP.getFreeHeap());
  delay(120);
  NimBLEDevice::init("PlaneRadar-RID");
  Serial.printf("[rid] s5 ble stack up heap=%u max=%u\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
  delay(100);
  NimBLEScan *sc = NimBLEDevice::getScan();
  Serial.printf("[rid] s5a getScan heap=%u\n", (unsigned)ESP.getFreeHeap());
  delay(60);
  sc->setAdvertisedDeviceCallbacks(s_ridCb, true);
  Serial.println("[rid] s5b cb set");
  delay(60);
  sc->setActiveScan(false);
  sc->setInterval(320);
  sc->setWindow(80);
  Serial.println("[rid] s5c params set itvl=320 win=80");
  delay(60);
  // 用限时窗口（6 秒）+ 循环重启，而不是 duration=0 永久扫描：
  // ble_gap_disc 在 IDF5 的 NimBLE 1.4.3 上首轮可能长时间等 HCI 完成事件，
  // 永久扫描会把 BLE 任务卡死在这里（c7 实测 s5c 之后无 s5d）。限时窗口
  // 至少让 start() 有返回机会，返回后立即重启，扫描仍然是连续的。
  while (s_ridBleTaskRun) {
    Serial.printf("[rid] s5c2 gap_disc 6s heap=%u\n", (unsigned)ESP.getFreeHeap());
    sc->start(6, false);
    Serial.printf("[rid] s5d start ret heap=%u ble=%d\n",
                  (unsigned)ESP.getFreeHeap(), s_ridBle ? 1 : 0);
    s_ridBle = true;
    delay(60);
  }
  Serial.println("[rid] s7 ble task exit");
}

static void ridStart() {
  if (s_ridOn || !s_ridWant) return;
  if (s_planeTask && millis() - s_ridWantMs < 30000UL) return;

  Serial.println("[rid] s1 release png");
  releasePngBuf();

  // ESP32-C3 是单核芯片，WiFi/BT 共用一套射频与共存状态机：
  //  a) 纯 AP 模式下共存拿不到射频时间片，BT 侧 HCI 会一直等；
  //  b) AP 一旦开始周期广播 beacon，BT 控制器初始化/同步更难抢到窗口。
  // 因此顺序是：先让 WiFi 起来但只 init 不出流量（mode + disconnect，不 softAP），
  // 打开 modem sleep 满足共存断言，再把 BLE 拉起来，最后才 broadcast 的 AP。
  Serial.println("[rid] s2 wifi init (no ap yet)");
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect();
  delay(150);
  Serial.printf("[rid] s3 wifi inited mode=%d\n", (int)WiFi.getMode());

  s_ridQHead = 0;
  s_ridQTail = 0;
  s_ridCount = 0;
  s_ridFrames = 0;
  s_ridBleN = 0;
  s_ridWifiN = 0;
  s_ridOuiLog = 0;

  // 崩溃根因（PC 0x420aa039 经符号表落点是 coex_core_condition_check）：
  // 该函数里的断言文案是 Should enable WiFi modem sleep when both WiFi and
  // Bluetooth are enabled。平时为消掉网页约 1 秒连接延迟调了 WiFi.setSleep(false)，
  // 共存条件因此不满足，使能 BT 控制器时直接 abort。侦测期间把 modem sleep
  // 打开，WiFi 与 BLE 才能共享射频。
  Serial.println("[rid] s3b modem sleep on");
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  delay(200);
  {
    wifi_ps_type_t ps = WIFI_PS_NONE;
    esp_wifi_get_ps(&ps);
    Serial.printf("[rid] s3c ps=%d mode=%d\n", (int)ps, (int)WiFi.getMode());
  }

  if (s_ridBleWant) {
    Serial.printf("[rid] s4 ble task start heap=%u max=%u\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
    if (!s_ridCb) s_ridCb = new RidScanCb();
    if (!s_ridBleTaskRun) {
      s_ridBleTaskRun = true;
      xTaskCreatePinnedToCore(ridBleTask, "ridble", 8192, NULL, 1, NULL, 0);
    }
  }

  // 给 BLE 任务最多 1.5 秒去完成控制器初始化与首次同步；超时也继续，
  // 绝不让 BLE 拖死整机（Beacon 侧仍可用，且不会触发关 modem sleep 的断言）。
  {
    unsigned long t0 = millis();
    while (s_ridBleTaskRun && !s_ridBle && millis() - t0 < 1500UL) delay(50);
    Serial.printf("[rid] s6b ble-ready=%d waited=%lu heap=%u\n", s_ridBle ? 1 : 0,
                  millis() - t0, (unsigned)ESP.getFreeHeap());
  }

  WiFi.softAP("PlaneRadar-Setup", NULL, RID_CHANNEL);
  esp_wifi_set_channel(RID_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf("[rid] s7 ap up ch%d heap=%u\n", RID_CHANNEL, (unsigned)ESP.getFreeHeap());

  if (s_ridWifiWant) {
    wifi_promiscuous_filter_t pf;
    pf.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
    esp_err_t e1 = esp_wifi_set_promiscuous_filter(&pf);
    esp_wifi_set_promiscuous_rx_cb(&ridWifiCb);
    esp_err_t e2 = esp_wifi_set_promiscuous(true);
    Serial.printf("[rid] s8 promiscuous on e=%d/%d heap=%u\n", (int)e1, (int)e2,
                  (unsigned)ESP.getFreeHeap());
  } else {
    Serial.println("[rid] s8 promiscuous skipped (wifi=0)");
  }

  s_ridOn = true;
  s_ridStartMs = millis();
  s_ridLastPoll = millis();
  s_ridHbdMs = 0;
  Serial.printf("[rid] scan ch%d ble=%d heap=%u max=%u lastPoll=%lu\n", RID_CHANNEL,
                s_ridBle ? 1 : 0, (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getMaxAllocHeap(), s_ridLastPoll);
}

static void ridStop() {
  if (!s_ridOn) return;
  Serial.printf("[rid] t1 promiscuous off heap=%u\n", (unsigned)ESP.getFreeHeap());
  esp_wifi_set_promiscuous(false);
  s_ridBleTaskRun = false;
  if (s_ridBle) {
    NimBLEScan *sc = NimBLEDevice::getScan();
    if (sc) sc->stop();
    NimBLEDevice::deinit(true);
    s_ridBle = false;
    Serial.println("[rid] t2 ble down");
  }
  s_ridOn = false;
  s_ridCount = 0;
  delay(150);
  // 共存断言要求两者同时使能时 WiFi 必须开 modem sleep。若 BT 控制器还
  // 处于 ENABLED（BLE 起来后又卡住、没能 deinit），关 modem sleep 会 abort，
  // 因此这里先查状态：没真正关掉就保持 modem sleep，宁可网页慢 1 秒。
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    Serial.println("[rid] bt still enabled: keep wifi modem sleep");
  } else {
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.println("[rid] wifi sleep restored");
  }
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("PlaneRadar-Setup", NULL, 1);
  WiFi.setAutoReconnect(true);
  if (s_ssid.length()) WiFi.begin(s_ssid.c_str(), s_pass.c_str());
  Serial.printf("[rid] stop: %d ble, %d wifi, %d msgs, heap=%u\n",
                s_ridBleN, s_ridWifiN, s_ridFrames, (unsigned)ESP.getFreeHeap());
}

static float ridDistKm(const RidDrone &d) {
  if (d.lat == 0.0f && d.lon == 0.0f) return -1.0f;
  return haversineKm(s_lat, s_lon, d.lat, d.lon);
}

// 注入自测：构造 BasicID / Location / System / OperatorID 四帧真实 ODID 报文，
// 走完整 ridFeed -> 队列 -> ridApply -> ridJson 路径，不发射任何射频信号。
// 用途：周围没有 RID 无人机时，验证解码、列表与网页渲染是否正确。
static void ridInjectTest() {
  char mac[13] = "aa:bb:cc:00:11:22";
  uint8_t m[25];
  memset(m, 0, 25); m[0] = 0x02; m[1] = 0x12;
  memcpy(m + 2, "TEST-RID-0001", 13);
  ridFeed(m, 25, mac, 0, 2);
  memset(m, 0, 25); m[0] = 0x12; m[1] = 0x20; m[2] = 45; m[3] = 8;
  int32_t la = 256614000, lo = 1147650000;
  memcpy(m + 5, &la, 4); memcpy(m + 9, &lo, 4);
  uint16_t ab = 1100, hh = 1080;
  memcpy(m + 13, &ab, 2); memcpy(m + 17, &hh, 2);
  ridFeed(m, 25, mac, 0, 2);
  memset(m, 0, 25); m[0] = 0x42;
  memcpy(m + 1, "OP-TEST", 7);
  int32_t ola = 256610000, olo = 1147640000;
  memcpy(m + 2, &ola, 4); memcpy(m + 6, &olo, 4);
  ridFeed(m, 25, mac, 0, 2);
  memset(m, 0, 25); m[0] = 0x52;
  memcpy(m + 2, "OP-TEST-001", 11);
  ridFeed(m, 25, mac, 0, 2);
  while (s_ridQTail != s_ridQHead) {
    RidMsg *q = &s_ridQ[s_ridQTail];
    ridApply(q->raw, q->mac, q->rssi, q->tr);
    s_ridQTail = (s_ridQTail + 1) % RID_QUEUE_N;
  }
  Serial.printf("[rid] inject done frames=%d n=%d\n", s_ridFrames, s_ridCount);
  String js; ridJson(js);
  Serial.printf("[rid] injjson %s\n", js.c_str());
}

static void ridTick(unsigned long now) {
  if (!s_ridWebHooked && s_srv) {
    s_srv->on("/rid", HTTP_GET, ridPage);
    s_srv->on("/rid.json", HTTP_GET, ridPage);
    s_ridWebHooked = true;
    Serial.println("[rid] web routes /rid /rid.json");
  }

  if (s_ridInjectReq) {
    s_ridInjectReq = false;
    ridInjectTest();
  }
  if (s_ridOffReq) {
    s_ridOffReq = false;
    if (s_ridOn) ridStop();
  }
  if (s_ridWant && !s_ridOn && s_ridWantMs) {
    if (now - s_ridHbdMs >= 2000UL) {
      s_ridHbdMs = now;
      Serial.printf("[rid] want pending heap=%u max=%u\n", (unsigned)ESP.getFreeHeap(),
                    (unsigned)ESP.getMaxAllocHeap());
    }
    if (millis() - s_ridWantMs >= 400UL) { ridStart(); return; }
  }

  if (!s_ridOn) return;
  if ((now - s_ridLastPoll) > RID_AUTO_STOP_MS &&
      (now - s_ridStartMs) > RID_AUTO_STOP_MS) {
    Serial.printf("[rid] auto stop: now=%lu lastPoll=%lu start=%lu\n",
                  now, s_ridLastPoll, s_ridStartMs);
    s_ridWant = false;
    ridStop();
    return;
  }
  if (now - s_ridHbdMs >= 10000UL) {
    s_ridHbdMs = now;
    Serial.printf("[rid] hb %lu s: ble=%d wifi=%d frames=%d drones=%d heap=%u\n",
                  (now - s_ridStartMs) / 1000UL, s_ridBleN, s_ridWifiN,
                  s_ridFrames, s_ridCount, (unsigned)ESP.getFreeHeap());
  }
  while (s_ridQTail != s_ridQHead) {
    RidMsg *q = &s_ridQ[s_ridQTail];
    ridApply(q->raw, q->mac, q->rssi, q->tr);
    s_ridQTail = (s_ridQTail + 1) % RID_QUEUE_N;
  }
  for (int i = 0; i < s_ridCount;) {
    if (now - s_rid[i].lastMs > RID_STALE_MS) {
      for (int k = i; k + 1 < s_ridCount; k++) s_rid[k] = s_rid[k + 1];
      s_ridCount--;
    } else {
      i++;
    }
  }
}

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

static const char *RID_JS =
    "var SMS=['未声明','地面','空中','紧急'];"
    "function pv(v){return (v===undefined||v===null||v==='')?'-':v;}"
    "function st(){fetch('/rid.json',{cache:'no-store'}).then(function(r){return r.json();})"
    ".then(function(d){"
    "var s=document.getElementById('st');"
    "s.textContent=(d.on?'侦测中':'未开启')+' | BLE '+d.ble+' 帧 | Beacon '+d.wifi"
    "+' 帧 | 解码 '+d.frames+' 条 | '+d.n+' 架 | 已跑 '+d.up+' 秒';"
    "var tb=document.getElementById('tb');tb.innerHTML='';"
    "(d.list||[]).forEach(function(o){"
    "var c=document.createElement('div');c.className='rcard';"
    "var t=document.createElement('div');t.className='rt';"
    "t.textContent=pv(o.id||o.mac);c.appendChild(t);"
    "function row(k,v){var p=document.createElement('div');p.className='rr';"
    "p.textContent=k+'：'+pv(v);c.appendChild(p);}"
    "row('状态 / 机型',(SMS[o.st]||'未知')+' · '+pv(o.ua));"
    "row('距离',o.km>0?o.km+' km':'-');"
    "row('高度 (相对地面)',o.alt>-1000?o.alt+' m ('+(o.h>-1000?o.h+' m':'-')+')':'-');"
    "row('速度 / 航向',o.spd>=0?o.spd+' m/s · '+o.dir+'°':'-');"
    "row('无人机位置',o.la?o.la+', '+o.lo:'未广播');"
    "row('操作者ID',o.op||'未广播');"
    "row('操作者位置',o.ola?o.ola+', '+o.olo:'未广播');"
    "row('信号 / 方式',pv(o.rssi)+' dBm · '+(o.tr==2?'WiFi Beacon':'BLE'));"
    "row('更新 / 来源',o.age+' 秒前 · '+pv(o.mac));"
    "tb.appendChild(c);});"
    "if(!(d.list||[]).length){var e=document.createElement('div');e.className='rcard';"
    "e.textContent=d.on?'暂未收到 RID 广播：把设备放在开阔处，等无人机通电/起飞后再看':'侦测未开启，点上面的按钮';"
    "tb.appendChild(e);}"
    "}).catch(function(){});}"
    "setInterval(st,2000);st();";

static void ridPage() {
  s_ridLastPoll = millis();
  if (s_srv->uri() == "/rid.json") {
    String js;
    ridJson(js);
    s_srv->send(200, "application/json; charset=utf-8", js);
    return;
  }
  if (s_srv->hasArg("on")) {
    s_ridWant = true;
    s_ridWantMs = millis();
    s_ridBleWant = s_srv->hasArg("ble") ? (s_srv->arg("ble") != "0") : true;
    s_ridWifiWant = s_srv->hasArg("wifi") ? (s_srv->arg("wifi") != "0") : true;
  }
  if (s_srv->hasArg("inject")) s_ridInjectReq = true;
  if (s_srv->hasArg("off")) {
    s_ridWant = false;
    if (s_ridOn) s_ridOffReq = true;
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
  h += "<button class=btn style='background:#8a8f98;font-size:13px' ";
  h += "onclick=\"location.href='/rid?on=1&amp;ble=0'\">只收 Beacon（不开蓝牙）</button>";
  h += "<button class=btn style='background:#8a8f98;font-size:13px' ";
  h += "onclick=\"location.href='/rid?on=1&amp;wifi=0'\">只收蓝牙（不开 Beacon）</button>";
  h += "<button class=btn style='background:#2f855a;font-size:13px' ";
  h += "onclick=\"location.href='/rid?inject=1'\">注入测试数据（验证解码与页面，不发射信号）</button>";
  h += "<div class=hint>点「开启侦测模式」后约 1 秒，设备会关闭联网（航班与云图停更），";
  h += "只保留热点 PlaneRadar-Setup（信道 6）。手机请连这个热点，再访问 192.168.4.1/rid 看结果。";
  h += "点「停止」后约 2 秒自动连回原来的 WiFi。</div>";
  h += "<div id=tb></div>";
  h += RID_JS;
  h += "<div class=foot>被动接收 ASTM F3411 / GB 46750 广播 RID，不发射任何信号</div>";
  h += "</body></html>";
  s_srv->send(200, "text/html; charset=utf-8", h);
}
