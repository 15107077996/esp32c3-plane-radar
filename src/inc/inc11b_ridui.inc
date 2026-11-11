// ---------------- RID radar view ----------------
// 侦察页的雷达画法：扫描扇面 + 量程圈 + 落在圈上的目标点。
// 数据全部来自 inc11 的 RidDrone 表（广播 RID 里的经纬度）；没有位置帧的
// 目标只计入架数，不上图。量程按最远目标自适应：不够用时立即升档，多余时
// 每 8 秒退一档，所以屏幕上看到的就是“最大侦测范围逐级收小”。
#define RID_RANGE_N 5
static const float RID_RANGE_M[RID_RANGE_N] = {200.0f, 500.0f, 1000.0f, 2000.0f,
                                               5000.0f};
static int s_ridRangeIdx = RID_RANGE_N - 1;  // 开屏先在最大档，再逐级收
static unsigned long s_ridStepMs = 0;
static float s_ridSweepDeg = 0.0f;

static float ridBearingDeg(float lat1, float lon1, float lat2, float lon2) {
  const double d2r = 0.017453292519943295;
  double p1 = (double)lat1 * d2r;
  double p2 = (double)lat2 * d2r;
  double dl = ((double)lon2 - (double)lon1) * d2r;
  double yy = sin(dl) * cos(p2);
  double xx = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
  double b = atan2(yy, xx) * 57.29577951308232;
  if (b < 0.0) b += 360.0;
  return (float)b;
}

static void ridPickRange(unsigned long now) {
  float dmax = 0.0f;
  for (int i = 0; i < s_ridCount; i++) {
    float km = ridDistKm(s_rid[i]);
    if (km > 0.0f && km * 1000.0f > dmax) dmax = km * 1000.0f;
  }
  int need = 0;
  if (dmax > 0.0f) {
    need = RID_RANGE_N - 1;
    for (int k = 0; k < RID_RANGE_N; k++)
      if (dmax * 1.15f <= RID_RANGE_M[k]) {
        need = k;
        break;
      }
  }
  if (need > s_ridRangeIdx) {
    s_ridRangeIdx = need;
    s_ridStepMs = now;
  } else if (need < s_ridRangeIdx && now - s_ridStepMs >= 8000UL) {
    s_ridRangeIdx--;
    s_ridStepMs = now;
  }
}

static void ridRangeLabel(char *buf, int n) {
  float m = RID_RANGE_M[s_ridRangeIdx];
  if (m < 1000.0f)
    snprintf(buf, n, "%.0fm", m);
  else
    snprintf(buf, n, "%.1fkm", m / 1000.0f);
}

static void drawRid() {
  const int cx = R_CX, cy = R_CY;
  const int R = 80;
  // 只擦本页用到的三块（标题、雷达区、底部三行），不用整屏 fillScreen，避免闪
  gfx->fillRect(0, 0, 240, 44, C_BLACK);
  gfx->fillRect(cx - 92, cy - 92, 184, 184, C_BLACK);
  gfx->fillRect(0, 198, 240, 42, C_BLACK);

  s_ridSweepDeg += 8.0f;
  if (s_ridSweepDeg >= 360.0f) s_ridSweepDeg -= 360.0f;
  ridPickRange(millis());

  // 量程圈：外圈是当前量程，里面两圈是 2/3 与 1/3
  gfx->drawCircle(cx, cy, R, C_DKGREEN);
  gfx->drawCircle(cx, cy, (2 * R) / 3, C_DKGREEN);
  gfx->drawCircle(cx, cy, R / 3, C_DKGREEN);
  gfx->drawLine(cx - R, cy, cx + R, cy, C_DKGREEN);
  gfx->drawLine(cx, cy - R, cx, cy + R, C_DKGREEN);

  // 扫描扇：8 条尾迹线，靠外的两条亮
  for (int k = 0; k < 8; k++) {
    float a = (s_ridSweepDeg - (float)k * 6.0f) * 0.0174532925f;
    int x = cx + (int)(R * sinf(a));
    int y = cy - (int)(R * cosf(a));
    gfx->drawLine(cx, cy, x, y, (k < 2) ? C_GREEN : C_DKGREEN);
  }

  // 目标点：距离 + 方位落图
  float rangeM = RID_RANGE_M[s_ridRangeIdx];
  int plotted = 0;
  for (int i = 0; i < s_ridCount && plotted < 5; i++) {
    float km = ridDistKm(s_rid[i]);
    if (km <= 0.0f) continue;
    float rr = km * 1000.0f / rangeM;
    if (rr > 1.0f) rr = 1.0f;
    float br = ridBearingDeg(s_lat, s_lon, s_rid[i].lat, s_rid[i].lon) * 0.0174532925f;
    int x = cx + (int)(R * rr * sinf(br));
    int y = cy - (int)(R * rr * cosf(br));
    gfx->fillCircle(x, y, 2, C_RED);
    const char *id = s_rid[i].uasId[0] ? s_rid[i].uasId : s_rid[i].mac;
    int L = (int)strlen(id);
    const char *tail = (L > 4) ? (id + L - 4) : id;
    useFont(u8g2_font_helvB08_tr);
    int tw = textW(tail);
    int tx = (x + 5 + tw > 236) ? (x - 5 - tw) : (x + 5);
    txtAt(tail, tx, y - 5, C_WHITE);
    plotted++;
  }
  gfx->fillCircle(cx, cy, 2, C_GREEN);

  // 标题
  useFont(u8g2_font_unifont_t_chinese4);
  txtC("无人机侦测", 18, C_YELLOW);

  // 底部三行：量程/架数、开启状态、链路计数
  char l[48];
  char rl[12];
  ridRangeLabel(rl, sizeof(rl));
  useFont(u8g2_font_unifont_t_chinese4);
  snprintf(l, sizeof(l), "%s · %d架", rl, s_ridCount);
  txtC(l, 192, s_ridCount ? C_GREEN : C_GRAY);

  // 开启状态放在最醒目的位置，网页点“开启侦测”后 250ms 内就会变成这行
  txtC(s_ridOn ? "侦测已开启" : "侦测未开启", 212, s_ridOn ? C_GREEN : C_GRAY);

  if (s_ridOn) {
    useFont(u8g2_font_helvB08_tr);
    snprintf(l, sizeof(l), "BLE %d  BC %d", s_ridBleN, s_ridWifiN);
    txtC(l, 230, C_CYAN);
  } else {
    useFont(u8g2_font_unifont_t_chinese4);
    txtC("按键启动", 230, C_GRAY);
  }
}
