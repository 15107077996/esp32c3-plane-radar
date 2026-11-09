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
#include "esp_wifi.h"

#define RID_CHANNEL 6
#define RID_STALE_MS 45000UL
#define RID_QUEUE_N 8
#define RID_AUTO_STOP_MS 600000UL

static void ridPage();
static bool s_ridWebHooked = false;
static unsigned long s_ridLastPoll = 0;
static bool s_ridBleWant = true;
static bool s_ridOffReq = false;
static unsigned long s_ridWantMs = 0;
static unsigned long s_ridHbdMs = 0;
