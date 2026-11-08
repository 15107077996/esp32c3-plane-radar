// ---------------- radar (static base + animated sweep wedge) ----------------
#define WEDGE_N 10
#define WEDGE_STEP 4
static const uint16_t WEDGE_COL[WEDGE_N] = {
    0x07E0, 0x0640, 0x0500, 0x0400, 0x0320,
    0x0280, 0x0200, 0x0180, 0x0120, 0x00C0};

struct Mapped { int i; int x; int y; float d; };
static Mapped s_mapped[MAX_PLANES];
static int s_mappedCnt = 0;

static void computeMapped() {
  float R = rangeKm();
  unsigned long now = millis();
  s_mappedCnt = 0;
  for (int i = 0; i < s_planeCount; i++) {
    double la, lo;
    planeLive(i, now, la, lo);
    float d = haversineKm(s_lat, s_lon, la, lo);
    if (d > R) continue;
    float b = bearingTo(s_lat, s_lon, la, lo) * PI / 180;
    int rr = (int)(R_R * d / R);
    int x = R_CX + (int)(sin(b) * rr);
    int y = R_CY - (int)(cos(b) * rr);
    if (s_mappedCnt < MAX_PLANES) {
      s_mapped[s_mappedCnt].i = i;
      s_mapped[s_mappedCnt].x = x;
      s_mapped[s_mappedCnt].y = y;
      s_mapped[s_mappedCnt].d = d;
      s_mappedCnt++;
    }
  }
  // nearest first
  for (int a = 0; a < s_mappedCnt - 1; a++)
    for (int b = a + 1; b < s_mappedCnt; b++) {
      if (s_mapped[b].d < s_mapped[a].d) {
        Mapped t = s_mapped[a]; s_mapped[a] = s_mapped[b]; s_mapped[b] = t;
      }
    }
}

static void drawRings() {
  float R = rangeKm();
  for (int i = 0; i < 4; i++) gfx->drawCircle(R_CX, R_CY, R_R - i, C_RED);
  for (int f = 1; f <= 3; f++) {
    int rr = (int)(R_R * f / 3.0f);
    gfx->drawCircle(R_CX, R_CY, rr, C_DKGREEN);
  }
  gfx->drawLine(R_CX, R_CY - R_R, R_CX, R_CY + R_R, C_DKGREEN);
  gfx->drawLine(R_CX - R_R, R_CY, R_CX + R_R, R_CY, C_DKGREEN);
  (void)R;
}

// Compass letters inside the sweep circle: 北 above, 南 below, 东 right, 西 left.
// Placed 66px off centre so they clear both the header text and the 75px ring.
// Each letter gets its own black pad: a white glyph drawn straight onto the
// bright fan was washed out, which read as the letter being cut in half while
// the fan swept over it. Pad + glyph are repainted every frame, after the fan.
static void cardinalAt(const char *c, int x, int y) {
  gfx->fillRect(x - 2, y - 2, 20, 20, C_BLACK);
  txtAt(c, x, y, C_WHITE);
}
static void drawCardinals() {
  useFont(u8g2_font_unifont_t_chinese4);
  cardinalAt("北", R_CX - 8, 60);
  cardinalAt("南", R_CX - 8, 192);
  cardinalAt("东", 186, 126);
  cardinalAt("西", 54, 126);
}

// Airports with a paved main runway (>=1800 m) around the centre: heading in
// degrees true, length in metres. Taken from the public OurAirports runway
// dataset, plus ZSGZ from its published aerodrome record (07/25, 2600 m).
struct AptRw { float hdg; uint16_t len; };
struct Apt { float lat; float lon; uint8_t n; AptRw rw[2]; };
static const Apt APTS[] = {
  {25.8533f, 114.7789f, 1, {{69, 2600}}},   // ZSGZ 赣州黄金 21km
  {23.5423f, 116.4950f, 1, {{40, 3200}}},   // ZGOW 揭阳潮汕 293km
  {23.3726f, 113.3065f, 2, {{14, 3800}, {14, 3800}}},   // ZGGG 广州白云 294km
  {28.2050f, 113.2207f, 2, {{177, 3800}, {177, 3200}}}, // ZGHA 长沙黄花 322km
  {22.6568f, 113.7806f, 2, {{155, 3800}, {153, 3600}}}, // ZGSZ 深圳宝安 349km
  {24.5346f, 118.1150f, 1, {{53, 3400}}},   // ZSAM 厦门高崎 360km
  {28.8534f, 115.8940f, 1, {{23, 3400}}},   // ZSCN 南昌昌北 372km
  {22.3157f, 113.8883f, 2, {{74, 3800}, {71, 3800}}},   // VHHH 香港赤鱲角 383km
  {24.4218f, 118.3460f, 1, {{64, 3000}}},   // RCBS 金门 386km
  {24.7871f, 118.5827f, 1, {{34, 2600}}},   // ZSQZ 泉州晋江 396km
  {22.1623f, 113.5860f, 1, {{160, 3214}}},  // VMMC 澳门 407km
};
#define APT_N ((int)(sizeof(APTS) / sizeof(APTS[0])))

// Runways are drawn at a fixed pixel length: at 400 km a 2.6 km runway is less
// than one pixel, so a true-scale strip would simply never appear.
static void drawAirports(float R) {
  if (!s_showApt) return;
  for (int i = 0; i < APT_N; i++) {
    float d = haversineKm(s_lat, s_lon, APTS[i].lat, APTS[i].lon);
    if (d > R) continue;
    float b = bearingTo(s_lat, s_lon, APTS[i].lat, APTS[i].lon) * PI / 180;
    int x = R_CX + (int)(sin(b) * R_R * d / R);
    int y = R_CY - (int)(cos(b) * R_R * d / R);
    for (int k = 0; k < APTS[i].n; k++) {
      float a = APTS[i].rw[k].hdg * PI / 180.0f;
      int dx = (int)(sin(a) * 5), dy = (int)(-cos(a) * 5);
      gfx->drawLine(x - dx, y - dy, x + dx, y + dy, C_WHITE);
      gfx->drawLine(x - dx, y - dy + 1, x + dx, y + dy + 1, C_GRAY);
    }
  }
}

static void drawMarkers() {
  for (int i = 0; i < s_mappedCnt; i++) {
    int pi = s_mapped[i].i;
    drawPlaneMarker(s_mapped[i].x, s_mapped[i].y, s_planes[pi].head,
                    altColor(s_planes[pi].alt));
  }
}

// range numbers + header + per-plane text labels (all static text)
static void drawLabels() {
  float R = rangeKm();
  float Rv = s_unitMi ? R * 0.621371f : R;
  useFont(u8g2_font_helvB08_tr);
  char lb[16];
  for (int f = 1; f <= 3; f++) {
    int rr = (int)(R_R * f / 3.0f);
    float val = Rv * f / 3.0f;
    if (val >= 10) snprintf(lb, sizeof(lb), "%.0f", val);
    else snprintf(lb, sizeof(lb), "%.1f", val);
    txtAt(lb, R_CX + 2, R_CY - rr + 3, C_CYAN);
  }

  int lc = s_mappedCnt < MAX_LABELS ? s_mappedCnt : MAX_LABELS;
  useFont(u8g2_font_helvB10_tr);
  for (int k = 0; k < lc; k++) {
    int pi = s_mapped[k].i;
    int mx = s_mapped[k].x, my = s_mapped[k].y;
    bool onRight = mx >= R_CX;
    int lx = onRight ? mx + 8 : mx - 62;
    int ly = my - 6;
    if (ly < 30) ly = 30;
    if (ly > 196) ly = 196;
    char dm[16];
    float dkm = planeLiveDistKm(pi);
    if (s_unitMi) snprintf(dm, sizeof(dm), "%.0fft", dkm * 3280.84f);
    else snprintf(dm, sizeof(dm), "%.0fm", dkm * 1000.0f);
    txtAt(s_planes[pi].call, lx, ly, C_WHITE);
    if (s_planes[pi].typ[0]) txtAt(s_planes[pi].typ, lx, ly + 12, C_YELLOW);
    txtAt(dm, lx, ly + 24, C_CYAN);
  }

  // header: Chinese location name + range/count
  useFont(u8g2_font_unifont_t_chinese4);
  txtC(s_city, 12, C_YELLOW);
  useFont(u8g2_font_helvB08_tr);
  char hdr[40];
  if (s_unitMi) snprintf(hdr, sizeof(hdr), "%.0fmi N%d", Rv, s_planeCount);
  else snprintf(hdr, sizeof(hdr), "%.0fkm N%d", Rv, s_planeCount);
  txtC(hdr, 27, C_GREEN);
}

static void drawWedge(float sweep, bool erase) {
  // Solid gradient fan: one filled pie slice per gradient step.
  for (int k = 0; k < WEDGE_N; k++) {
    float a1 = (sweep - k * WEDGE_STEP) * PI / 180;
    float a2 = (sweep - (k + 1) * WEDGE_STEP) * PI / 180;
    int x1 = R_CX + (int)(sin(a1) * (R_R - 6));
    int y1 = R_CY - (int)(cos(a1) * (R_R - 6));
    int x2 = R_CX + (int)(sin(a2) * (R_R - 6));
    int y2 = R_CY - (int)(cos(a2) * (R_R - 6));
    gfx->fillTriangle(R_CX, R_CY, x1, y1, x2, y2,
                      erase ? C_BLACK : WEDGE_COL[k]);
  }
  gfx->fillCircle(R_CX, R_CY, 3, erase ? C_BLACK : C_WHITE);
}

static void drawRadarBase() {
  gfx->fillScreen(C_BLACK);
  drawRings();
  drawAirports(rangeKm());
  computeMapped();
  drawMarkers();
  drawLabels();
  if (s_animSweep) drawWedge(s_sweep, false);
  // Compass letters go last: the fan sweeps over them, so painting them
  // before the wedge left "东/西/南/北" half cut until the fan moved on.
  drawCardinals();
}

// One animation frame: only the fan and what it covers change.
static void drawSweepStep(float oldSweep, float newSweep) {
  drawWedge(oldSweep, true);   // erase the fan where it was
  drawRings();
  drawAirports(rangeKm());
  computeMapped(); // dead-reckon live positions so aircraft glide smoothly
  drawMarkers();
  drawLabels();
  drawWedge(newSweep, false);  // draw the fan where it is now
  drawCardinals();             // letters last, on their own black pads
}

// Full redraw (page entry / after plane fetch).
static void drawPlanes(float sweep) {
  (void)sweep;
  drawRadarBase();
}
