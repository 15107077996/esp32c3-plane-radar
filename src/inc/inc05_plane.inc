// ---------------- planes ----------------
static float planeDistKm(int i) {
  return haversineKm(s_lat, s_lon, s_planes[i].lat, s_planes[i].lon);
}

// Destination point from start, bearing (deg) and distance (km).
static void destPoint(double lat, double lon, float bear, double km,
                      double &outLat, double &outLon) {
  double R = 6371.0;
  double br = deg2rad(bear);
  double lr = km / R;
  double phi1 = deg2rad(lat), lam1 = deg2rad(lon);
  double phi2 = asin(sin(phi1) * cos(lr) +
                     cos(phi1) * sin(lr) * cos(br));
  double lam2 = lam1 + atan2(sin(br) * sin(lr) * cos(phi1),
                             cos(lr) - sin(phi1) * sin(phi2));
  outLat = rad2deg(phi2);
  outLon = rad2deg(lam2);
}

// Live position via dead reckoning: extrapolate from the last network fix
// along the track at ground speed. A new fetch resets the base.
static void planeLive(int i, unsigned long now, double &la, double &lo) {
  Plane &p = s_planes[i];
  float dt = (now - p.baseMs) / 1000.0f; // seconds since fix
  if (dt < 0) dt = 0;
  if (dt > 120) dt = 120;                // survive one missed 60 s fetch
  if (p.gs > 1.0f && dt > 0) {
    double km = (p.gs * 1.852) * (dt / 3600.0); // knots -> km/h * hours
    destPoint(p.lat, p.lon, (float)p.head, km, la, lo);
  } else {
    la = p.lat;
    lo = p.lon;
  }
}

// Append one position to the aircraft's trail ring buffer.
static void trailPush(int i, double la, double lo) {
  int h = s_trailHead[i];
  if (h < 0 || h >= TRAIL_N) h = 0;
  s_trail[i][h].lat = (float)la;
  s_trail[i][h].lon = (float)lo;
  s_trailHead[i] = (uint8_t)((h + 1) % TRAIL_N);
  if (s_trailN[i] < TRAIL_N) s_trailN[i]++;
}

// Sample every trail once per TRAIL_SAMPLE_MS while the radar page is open.
static void trailSample(unsigned long now) {
  if (s_trailMs != 0 && now - s_trailMs < TRAIL_SAMPLE_MS) return;
  s_trailMs = now;
  int tails = 0;
  int maxPts = 0;
  for (int i = 0; i < s_planeCount; i++) {
    double la, lo;
    planeLive(i, now, la, lo);
    trailPush(i, la, lo);
    if (s_trailN[i] > 1) tails++;
    if (s_trailN[i] > maxPts) maxPts = s_trailN[i];
  }
  Serial.printf("[trail] planes=%d tails=%d max=%d\n", s_planeCount, tails, maxPts);
}

// Two entries describe the same aircraft when the callsign matches. Anonymous
// targets carry "????", which says nothing, so they never match.
static bool sameCall(const Plane &a, const Plane &b) {
  if (a.call[0] == 0 || b.call[0] == 0) return false;
  if (!strcmp(a.call, "????")) return false;
  return strcmp(a.call, b.call) == 0;
}

// Live distance from the center to the extrapolated position.
static float planeLiveDistKm(int i) {
  double la, lo;
  planeLive(i, millis(), la, lo);
  return haversineKm(s_lat, s_lon, la, lo);
}

static int nearest() {
  int best = -1;
  float bd = 1e9;
  for (int i = 0; i < s_planeCount; i++) {
    float d = planeLiveDistKm(i);
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

static uint16_t altColor(int alt) {
  if (alt <= 0) return C_GRAY;
  if (alt < 3000) return C_CYAN;
  if (alt < 8000) return C_GREEN;
  if (alt < 11000) return C_YELLOW;
  return C_RED;
}

// Rotate a body-coordinate point (lx=forward, ly=right) by heading.
static void planeRot(int x, int y, float sn, float cs,
                     float lx, float ly, int &ox, int &oy) {
  ox = x + (int)(lx * sn + ly * cs + 0.5f);
  oy = y + (int)(-lx * cs + ly * sn + 0.5f);
}
static void fillTriBody(int x, int y, float sn, float cs,
                        float x0, float y0, float x1, float y1,
                        float x2, float y2, uint16_t col) {
  int ax, ay, bx, by, cx, cy;
  planeRot(x, y, sn, cs, x0, y0, ax, ay);
  planeRot(x, y, sn, cs, x1, y1, bx, by);
  planeRot(x, y, sn, cs, x2, y2, cx, cy);
  gfx->fillTriangle(ax, ay, bx, by, cx, cy, col);
}

// Clear top-view airplane icon (fuselage + swept wings + tail), rotated by heading.
static void drawPlaneMarker(int x, int y, int head, uint16_t col) {
  float a = head * PI / 180.0f;
  float sn = sin(a), cs = cos(a);
  // fuselage (nose -> tail)
  fillTriBody(x, y, sn, cs, 8, 0, -6, 1, -6, -1, col);
  // right main wing
  fillTriBody(x, y, sn, cs, 0, 0, -3, 9, -3, 0, col);
  fillTriBody(x, y, sn, cs, -3, 9, -4, 9, -3, 0, col);
  // left main wing
  fillTriBody(x, y, sn, cs, 0, 0, -3, 0, -3, -9, col);
  fillTriBody(x, y, sn, cs, -3, 0, -4, -9, -3, -9, col);
  // tail planes
  fillTriBody(x, y, sn, cs, -5, 0, -7, 3, -6, 0, col);
  fillTriBody(x, y, sn, cs, -5, 0, -6, 0, -7, -3, col);
}

// URL from the configurable template ({lat} {lon} {nm}).
static String planeUrl() {
  String u = s_api.length() ? s_api : String(DEF_PLANE_API);
  u.replace("{lat}", String(s_lat, 4));
  u.replace("{lon}", String(s_lon, 4));
  u.replace("{nm}", String(FETCH_NM, 1));
  return u;
}

// Pick the smallest range that actually holds traffic (boot / first fix only).
static void autoFitRange() {
  if (!s_rangeAuto || s_planeCount <= 0) return;
  int want = RANGE_N - 1;
  for (int ri = 0; ri < RANGE_N; ri++) {
    int fit = 0;
    for (int i = 0; i < s_planeCount; i++)
      if (s_planes[i].distM <= RANGES_KM[ri] * 1000.0f) fit++;
    if (fit >= 3) { want = ri; break; }
  }
  s_rangeIdx = want;
}

// Network fetch into dst. No drawing and no shared globals are touched, so it
// can run from a task while the screen keeps painting the sweep.
static bool fetchPlanesCore(Plane *dst, int cap, int *outN) {
  *outN = 0;
  String url = planeUrl();
  WiFiClientSecure tls;
  tls.setInsecure();
  HTTPClient http;
  http.setConnectTimeout(8000);
  http.setTimeout(12000);
  http.begin(tls, url.c_str());
  int code = http.GET();
  if (code != 200) {
    Serial.printf("[planes] http %d\n", code);
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();
  JsonDocument doc;
  if (deserializeJson(doc, body)) {
    Serial.println("[planes] json fail");
    return false;
  }
  JsonArray ac = doc["ac"];
  // Keep the nearest aircraft in distance order: the JSON order is arbitrary,
  // so taking the first entries used to drop the nearby ones.
  int n = 0, seen = 0;
  for (JsonObject p : ac) {
    seen++;
    if (p["lat"].isNull() || p["lon"].isNull()) continue;
    Plane tmp;
    const char *fl = p["flight"];
    const char *tp = p["t"];
    String cs = fl ? String(fl) : String("????");
    cs.trim();
    cs.toCharArray(tmp.call, sizeof(tmp.call));
    String tt = tp ? String(tp) : String("");
    tt.toCharArray(tmp.typ, sizeof(tmp.typ));
    tmp.lat = p["lat"].as<double>();
    tmp.lon = p["lon"].as<double>();
    tmp.alt = p["alt_baro"] | 0;
    tmp.head = p["track"] | 0;
    tmp.gs = p["gs"] | 0;
    tmp.baseMs = millis();
    float dkm = haversineKm(s_lat, s_lon, tmp.lat, tmp.lon);
    tmp.distNm = dkm / 1.852f;
    tmp.distM = dkm * 1000.0f;
    tmp.bear = bearingTo(s_lat, s_lon, tmp.lat, tmp.lon);
    if (n >= cap && dkm >= dst[cap - 1].distM / 1000.0f) continue;
    int pos = (n < cap) ? n : (cap - 1);
    while (pos > 0 && dst[pos - 1].distM > tmp.distM) {
      dst[pos] = dst[pos - 1];
      pos--;
    }
    dst[pos] = tmp;
    if (n < cap) n++;
  }
  *outN = n;
  int nt = 0;
  float nd = 1e9f;
  for (int i = 0; i < n; i++)
    if (dst[i].distM < nd) { nd = dst[i].distM; nt = i; }
  Serial.printf("[planes] total=%d seen=%d nearest=%s %.0fkm\n", n, seen,
                n > 0 ? dst[nt].call : "-",
                n > 0 ? dst[nt].distM / 1000.0f : 0.0f);
  return true;
}

// Blocking fetch, used once during setup.
static bool fetchPlanes() {
  // Never start TLS while the big PNG buffer is held.
  releasePngBuf();
  int n = 0;
  if (!fetchPlanesCore(s_planes, MAX_PLANES, &n)) return false;
  s_planeCount = n;
  s_planeMs = millis();
  autoFitRange();
  return true;
}

// Background fetch: TLS + JSON parsing take a second or two, which used to
// freeze the sweep on the radar page every minute. The task fills a shadow
// buffer and the main loop adopts it on the next frame.
static void planeFetchTask(void *) {
  int n = 0;
  bool ok = fetchPlanesCore(s_planesNew, MAX_PLANES, &n);
  s_planesNewN = ok ? n : -2;
  s_planeTask = false;
  vTaskDelete(NULL);
}
static bool fetchPlanesAsync() {
  if (s_planeTask) return false;
  // The clock starts when the fetch starts, not when its result lands: a TLS
  // round trip can take tens of seconds, and measuring from the last adopt let
  // the next fetch start immediately and run back to back.
  unsigned long now = millis();
  if (s_planeMs != 0 && now - s_planeMs < 60000) return false;
  s_planeMs = now;
  // TLS needs a large contiguous block; the rain page's PNG scratch buffer is
  // the only thing that big, and it is idle while we are on the radar page.
  releasePngBuf();
  s_planeTask = true;
  // Core 0 keeps the network stack company; the UI loop owns core 1, so TLS
  // and JSON parsing can no longer steal a frame from the sweep.
  if (xTaskCreatePinnedToCore(planeFetchTask, "planefetch", 12288, NULL, 1, NULL, 0) != pdPASS) {
    Serial.println("[planes] task spawn fail");
    s_planeTask = false;
    return false;
  }
  return true;
}
static bool adoptPlanes() {
  int n = s_planesNewN;
  if (n == -1) return false;
  s_planesNewN = -1;
  if (n <= 0) return false; // failed or empty: keep what is already on screen
  if (n > MAX_PLANES) n = MAX_PLANES;
  // The list is re-sorted by distance on every fetch, so a plane changes index.
  // Match by callsign first and hand its trail over, otherwise the tail would
  // restart at every fetch and aircraft would look pinned in place.
  static uint8_t srcOf[MAX_PLANES];
  for (int j = 0; j < n; j++) {
    srcOf[j] = 255;
    for (int i = 0; i < s_planeCount; i++) {
      if (sameCall(s_planes[i], s_planesNew[j])) { srcOf[j] = (uint8_t)i; break; }
    }
  }
  for (int j = 0; j < n; j++) {
    int src = srcOf[j];
    if (src == 255 || src >= MAX_PLANES || src >= s_planeCount) {
      s_trailN[j] = 0;
      s_trailHead[j] = 0;
      continue;
    }
    TrailPt keep[TRAIL_N];
    memcpy(keep, s_trail[src], sizeof(keep));
    memcpy(s_trail[j], keep, sizeof(keep));
    s_trailN[j] = s_trailN[src];
    s_trailHead[j] = s_trailHead[src] % TRAIL_N;
    if (s_trailN[j] > TRAIL_N) s_trailN[j] = TRAIL_N;
  }
  memcpy(s_planes, s_planesNew, sizeof(Plane) * n);
  s_planeCount = n;
  // Anchor the carried-over tail at the freshly received fix.
  unsigned long now = millis();
  for (int i = 0; i < n; i++) {
    double la, lo;
    planeLive(i, now, la, lo);
    trailPush(i, la, lo);
  }
  // The 60 s clock was already started when this fetch began; restarting it
  // here would let the next fetch fire as soon as this one lands, which is how
  // TLS ended up occupying the radio almost continuously.
  autoFitRange();
  return true;
}
