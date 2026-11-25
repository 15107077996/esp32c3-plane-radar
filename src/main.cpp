// PlaneRadar - single translation unit assembled from small include chunks.
// Kept in one TU so all static globals are shared without extern declarations.
// lwIP 底层 DNS：Arduino 2.0.17 的 WiFi.config() 对已连上的 STA 不生效，
// 只有 dns_setserver 能把解析服务器钉死。
#include <lwip/dns.h>
#include "inc/inc01_defs.inc"
#include "inc/inc01b_ver.inc"
#include "inc/inc02_util.inc"
#include "inc/inc03_meteo.inc"
#include "inc/inc04_icon.inc"
#include "inc/inc05_plane.inc"
#include "inc/inc06_radar.inc"
#include "inc/inc07_clock.inc"
#include "inc/inc08_forecast.inc"
#include "inc/inc09_rain.inc"
#include "inc/inc11_rid.inc"
#include "inc/inc11b_ridui.inc"
#include "inc/inc10_core.inc"
