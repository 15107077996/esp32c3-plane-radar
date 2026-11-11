// ---------------- weather: China Weather (station obs) + Open-Meteo fallback ----------------
// 手机天气 App 的数据来自中央气象台/中国天气网：实况是气象站观测、预报是城市
// 预报。原来只用 Open-Meteo（数值模式输出），清晨谷地会比观测高 3~4 度，天气
// 现象也常对不上（模式给“阴”、实况是“晴”）。现在以天气网为主源，Open-Meteo
// 只在天气网连续失败时兜底。
static int s_windLvl = 0;   // Beaufort 0..12
static int s_windFrom = -1; // direction in degrees, -1 = unknown
static float s_gust = 0.0f; // gust km/h
static String s_alert = ""; // empty when nothing to flag

// 中国天气网（与手机 App 同源）
static String s_cwCityId = "";   // "101240704" = 南康
static String s_cwUpd = "";      // 实况观测时间 "08:35"
static String s_cwWeather = "";  // 实况文字 "晴"
static String s_cwWs = "";       // "东北风 1级"
static float s_tmaxToday = -99;  // 今天最高（城市预报）
static float s_tminToday = -99;  // 今天最低
static bool s_cwOk = false;
static int s_cwFails = 0;

static const char *CW_UA =
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36";

static int beaufort(float kmh) {
  static const float lim[12] = {1, 5, 11, 19, 28, 38, 49, 61, 74, 88, 102, 117};
  int l = 0;
  while (l < 12 && kmh > lim[l]) l++;
  return l;
}

static const char *windDirZh(int deg) {
  static const char *D[8] = {"北", "东北", "东", "东南",
                             "南", "西南", "西", "西北"};
  if (deg < 0) return "";
  return D[((deg + 22) / 45) % 8];
}

static void buildAlert(float gmax, float psum, int pop) {
  s_alert = "";
  if (s_code >= 95) {
    s_alert = "雷电预警";
  } else if (psum >= 25.0f || (s_code >= 63 && psum >= 10.0f)) {
    s_alert = "强降雨预警";
  } else if (gmax >= 39.0f || s_windLvl >= 6) {
    s_alert = "大风预警";
  } else if (s_code >= 45 && s_code <= 48) {
    s_alert = "大雾预警";
  } else if (s_temp >= 35.0f) {
    s_alert = "高温预警";
  } else if (s_temp <= 0.0f) {
    s_alert = "低温预警";
  } else if (pop >= 80 && s_code >= 51) {
    s_alert = "降雨提醒";
  }
  Serial.printf("[alert] lvl=%d gust=%.1f psum=%.1f pop=%d\n", s_windLvl, gmax,
                psum, pop);
}

// 天气网图标码（"00"~"39"）→ WMO code，喂给现有的 drawWxIcon / wmoZh
static int cw2wmo(int fa) {
  if (fa <= 0) return 0;              // 00 晴
  if (fa == 1) return 2;              // 01 多云
  if (fa == 2) return 3;              // 02 阴
  if (fa == 3) return 80;             // 03 阵雨
  if (fa == 4) return 95;             // 04 雷阵雨
  if (fa == 5) return 96;             // 05 雷阵雨伴冰雹
  if (fa == 6) return 68;             // 06 雨夹雪
  if (fa == 7) return 61;             // 07 小雨
  if (fa == 8) return 63;             // 08 中雨
  if (fa == 9) return 65;             // 09 大雨
  if (fa >= 10 && fa <= 12) return 65;  // 10~12 暴雨
  if (fa == 13) return 85;            // 13 阵雪
  if (fa == 14) return 71;            // 14 小雪
  if (fa == 15) return 73;            // 15 中雪
  if (fa == 16 || fa == 17) return 75;  // 16/17 大雪暴雪
  if (fa == 19) return 66;            // 19 冻雨
  if (fa >= 18 && fa <= 35) return 45;  // 18/32~35 雾，20~31 沙尘
  if (fa >= 36) return 45;            // 36~39 霾
  return 2;
}

// 从 "var xxx ={...}" 里取出那一段 JSON
static String cwJsonPart(const String &body, const char *key) {
  int k = body.indexOf(key);
  if (k < 0) return "";
  int s = body.indexOf('{', k);
  if (s < 0) return "";
  int e = body.indexOf("};var", s);
  if (e < 0) e = body.indexOf("};", s);
  if (e < 0) e = body.length() - 1;
  return body.substring(s, e + 1);
}

// 城市名 → 天气网城市 ID（toy1 检索，取第一条 ref 的数字部分）
static bool fetchCwCityId() {
  String base = s_city;
  base.replace("区", "");
  base.replace("县", "");
  base.replace("市", "");
  for (int pass = 0; pass < 2; pass++) {
    String q = (pass == 0) ? base : s_city;
    q.trim();
    if (q.length() == 0) continue;
    String url = "http://toy1.weather.com.cn/search?cityname=" + urlEnc(q) +
                 "&_=" + String(millis());
    HTTPClient http;
    http.setConnectTimeout(5000);
    http.setTimeout(6000);
    http.begin(url);
    http.setUserAgent(CW_UA);
    http.addHeader("Referer", "http://www.weather.com.cn/");
    int code = http.GET();
    bool ok = false;
    if (code == 200) {
      String body = http.getString();
      int i = body.indexOf("\"ref\":\"");
      if (i >= 0) {
        int j = body.indexOf('~', i);
        if (j > i + 7) {
          String id = body.substring(i + 7, j);
          id.trim();
          if (id.length() >= 9) {
            s_cwCityId = id;
            s_prefs.putString("cwid", id);
            ok = true;
          }
        }
      }
    }
    http.end();
    if (ok) {
      Serial.printf("[cw] cityid=%s\n", s_cwCityId.c_str());
      return true;
    } else {
      Serial.printf("[cw] cityid http %d\n", code);
    }
  }
  return false;
}

// 实况 + 今日高低温 + 3 天预报 + 官方预警，一次请求全拿到
static bool fetchWeatherCn() {
  unsigned long t0 = millis();
  if (s_cwCityId.length() == 0) {
    s_cwCityId = s_prefs.getString("cwid", "");
    if (s_cwCityId.length() == 0 && !fetchCwCityId()) return false;
  }
  String url = "http://d1.weather.com.cn/weather_index/" + s_cwCityId +
               ".html?_=" + String(millis());
  HTTPClient http;
  http.setConnectTimeout(6000);
  http.setTimeout(9000);
  http.begin(url);
  http.setUserAgent(CW_UA);
  http.addHeader("Referer", "http://www.weather.com.cn/weather1d/" + s_cwCityId +
                               ".shtml");
  int code = http.GET();
  bool ok = false;
  if (code == 200) {
    String body = http.getString();

    // 实况 dataSK
    String sk = cwJsonPart(body, "dataSK =");
    if (sk.length() > 8) {
      JsonDocument d;
      if (!deserializeJson(d, sk)) {
        if (!d["temp"].isNull()) s_temp = d["temp"].as<float>();
        String sd = d["SD"].as<String>();
        sd.replace("%", "");
        sd.trim();
        if (sd.length()) s_hum = sd.toFloat();
        String wc = d["weathercode"].as<String>(); // "d00"
        if (wc.length() >= 3) s_code = cw2wmo(wc.substring(1).toInt());
        s_cwWeather = d["weather"].as<String>();
        s_cwUpd = d["time"].as<String>();
        s_cwWs = d["WD"].as<String>();
        String ws = d["WS"].as<String>();
        ws.trim();
        if (s_cwWs.length() && ws.length()) s_cwWs += " " + ws;
        s_prec = d["rain"].as<String>().toFloat();
        ok = true;
      }
    }

    // 7 天预报 fc.f[]：f[0] 今天，f[1..3] 明天/后天/大后天
    String fc = cwJsonPart(body, "var fc =");
    if (fc.length() > 20) {
      JsonDocument d2;
      if (!deserializeJson(d2, fc)) {
        JsonArray arr = d2["f"].as<JsonArray>();
        if (!arr.isNull() && arr.size() >= 4) {
          JsonObject t = arr[0].as<JsonObject>();
          s_tmaxToday = t["fc"].as<float>();
          s_tminToday = t["fd"].as<float>();
          for (int i = 0; i < 3; i++) {
            JsonObject o = arr[i + 1].as<JsonObject>();
            s_days[i].tmax = o["fc"].as<float>();
            s_days[i].tmin = o["fd"].as<float>();
            s_days[i].code = cw2wmo(o["fa"].as<String>().toInt());
            s_days[i].pop = 0;
          }
          ok = true;
        }
      }
    }

    // 官方预警：有就照抄标题，没有就留空（不再用本地阈值自己编）
    s_alert = "";
    String al = cwJsonPart(body, "alarmDZ =");
    if (al.length() > 6) {
      JsonDocument d3;
      if (!deserializeJson(d3, al)) {
        JsonArray w = d3["w"].as<JsonArray>();
        if (!w.isNull() && w.size() > 0) {
          JsonObject w0 = w[0].as<JsonObject>();
          String title = w0["title"].as<String>();
          title.trim();
          if (title.length()) s_alert = title;
        }
      }
    }
  } else {
    Serial.printf("[cw] http %d\n", code);
  }
  http.end();
  s_metCostMs = millis() - t0;
  if (ok) {
    s_metMs = millis();
    s_metOk = true;
    s_cwOk = true;
    s_cwFails = 0;
    Serial.printf("[cw] %.1fC code %d hum %.0f prec %.1f upd %s today %.0f/%.0f\n",
                  s_temp, s_code, s_hum, s_prec, s_cwUpd.c_str(), s_tminToday,
                  s_tmaxToday);
    Serial.printf("[cw] %s / %s / alert %d\n", s_cwWeather.c_str(),
                  s_cwWs.c_str(), (int)s_alert.length());
    Serial.printf("[cw] days %.0f/%.0f %.0f/%.0f %.0f/%.0f\n", s_days[0].tmin,
                  s_days[0].tmax, s_days[1].tmin, s_days[1].tmax,
                  s_days[2].tmin, s_days[2].tmax);
  } else {
    s_cwFails++;
  }
  return ok;
}

// 主入口：先天气网，失败再退回 Open-Meteo（数值模式，仅兜底）
static bool fetchMeteo() {
  if (fetchWeatherCn()) return true;
  Serial.println("[cw] fallback open-meteo");
  return fetchOpenMeteo();
}

static bool fetchOpenMeteo() {
  unsigned long t0 = millis();
  char url[512];
  snprintf(url, sizeof(url),
           "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
           "&current=temperature_2m,relative_humidity_2m,weather_code,"
           "wind_speed_10m,wind_direction_10m,wind_gusts_10m,precipitation"
           "&daily=weather_code,temperature_2m_max,temperature_2m_min,"
           "precipitation_probability_max,precipitation_sum,"
           "wind_gusts_10m_max&timezone=auto&forecast_days=4",
           s_lat, s_lon);
  HTTPClient http;
  http.setConnectTimeout(6000);
  http.setTimeout(8000);
  http.begin(url);
  int code = http.GET();
  bool ok = false;
  if (code == 200) {
    String body = http.getString();
    JsonDocument doc;
    if (!deserializeJson(doc, body)) {
      JsonObject cur = doc["current"];
      if (!cur.isNull()) {
        s_temp = cur["temperature_2m"] | s_temp;
        s_hum = cur["relative_humidity_2m"] | s_hum;
        s_wind = cur["wind_speed_10m"] | s_wind;
        s_code = cur["weather_code"] | s_code;
        s_prec = cur["precipitation"] | 0.0f;
        s_windFrom = cur["wind_direction_10m"] | -1;
        s_gust = cur["wind_gusts_10m"] | 0.0f;
        s_windLvl = beaufort(s_wind);
        s_cwWeather = "";
        s_cwWs = "";
        s_cwUpd = "";
      }
      JsonObject daily = doc["daily"];
      JsonArray dcode = daily["weather_code"];
      JsonArray dmax = daily["temperature_2m_max"];
      JsonArray dmin = daily["temperature_2m_min"];
      JsonArray dpop = daily["precipitation_probability_max"];
      JsonArray dsum = daily["precipitation_sum"];
      JsonArray dgust = daily["wind_gusts_10m_max"];
      int dn = (int)dmax.size();
      // NOTE: temperatures are floats in the payload, so the default value must
      // be a float. Writing "| 0" makes ArduinoJson return the int default 0
      // for every fractional value, which is why the columns showed 0/0.
      if (dn > 0) {
        s_tmaxToday = dmax[0] | -99.0f;
        s_tminToday = dmin[0] | -99.0f;
      }
      for (int i = 0; i < 3; i++) {
        int di = i + 1;
        if (di < dn) {
          s_days[i].code = dcode[di] | 0;
          s_days[i].tmax = dmax[di] | -99.0f;
          s_days[i].tmin = dmin[di] | -99.0f;
          s_days[i].pop = dpop[di] | 0;
        }
      }
      float gmax = dgust[1] | 0.0f;
      float psum = dsum[1] | 0.0f;
      int pop0 = dpop[1] | 0;
      buildAlert(gmax, psum, pop0);
      s_metMs = millis();
      s_metOk = true;
      ok = true;
      Serial.printf("[om] %.1fC hum %.0f wind %.1f dir %d gust %.1f lvl %d code %d prec %.1f\n",
                    s_temp, s_hum, s_wind, s_windFrom, s_gust, s_windLvl, s_code,
                    s_prec);
      Serial.printf("[om] days n=%d %.0f/%.0f %.0f/%.0f %.0f/%.0f\n", dn,
                    s_days[0].tmin, s_days[0].tmax, s_days[1].tmin,
                    s_days[1].tmax, s_days[2].tmin, s_days[2].tmax);
    }
  } else {
    Serial.printf("[om] http %d\n", code);
  }
  http.end();
  s_metCostMs = millis() - t0;
  return ok;
}
