// ESP32 Wi-Fi remote for the MM32SPIN27 hoverboard firmware.
//
// Phone (browser) --Wi-Fi, WebSocket--> ESP32 --UART, RemoteUartBus--> master board --> slave board
//
// The ESP32 runs a Wi-Fi hotspot and serves a control page (web_page.h) with a joystick or two tank
// sliders. The page sends its input over a WebSocket every 100 ms; the ESP32 turns it into a speed
// command per wheel, ramps it, and sends it to both boards (the master relays the slave's frames).
// Board answers (speed, battery voltage, torque current) go back to the page.
//
// Safety: no message from the phone for PHONE_TIMEOUT_MS, or a disconnect, ramps both wheels to 0.
// The STOP button sets them to 0 at once. The boards stop by themselves after 1 s without frames.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "config.h"
#include "hover_protocol.h"
#include "web_page.h"

static WebServer        http(80);
static WebSocketsServer ws(81);
static HoverAnswerParser parser;

// Phone input, -1000..1000 per wheel before direction and scaling (forward = positive).
static int32_t  inputLeft = 0;
static int32_t  inputRight = 0;
static int32_t  maxRpm = DEFAULT_MAX_RPM;
static uint32_t lastInputMs = 0;
static uint8_t  clients = 0;

// Commands actually sent (rpm, board direction applied), ramped towards the targets.
static float cmdLeft = 0;
static float cmdRight = 0;

static HoverAnswer answerLeft = {};
static HoverAnswer answerRight = {};

static int32_t clampI(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void stopNow()
{
    inputLeft = inputRight = 0;
    cmdLeft = cmdRight = 0;
}

// Messages from the page:
//   "j <x> <y>"   joystick, x right+, y forward+, each -1000..1000
//   "t <l> <r>"   tank sliders, left and right wheel, -1000..1000
//   "m <rpm>"     maximum speed for full stick
//   "s"           stop now
static void handleMessage(const char *msg)
{
    long a = 0, b = 0;
    switch (msg[0]) {
        case 'j':
            if (sscanf(msg + 1, "%ld %ld", &a, &b) == 2) {
                a = clampI(a, -1000, 1000);
                b = clampI(b, -1000, 1000);
                int32_t l = b + a;       // steer right: left wheel faster
                int32_t r = b - a;
                int32_t m = max(abs(l), abs(r));
                if (m > 1000) {          // keep the ratio when both add up past full scale
                    l = l * 1000 / m;
                    r = r * 1000 / m;
                }
                inputLeft = l;
                inputRight = r;
                lastInputMs = millis();
            }
            break;
        case 't':
            if (sscanf(msg + 1, "%ld %ld", &a, &b) == 2) {
                inputLeft = clampI(a, -1000, 1000);
                inputRight = clampI(b, -1000, 1000);
                lastInputMs = millis();
            }
            break;
        case 'm':
            if (sscanf(msg + 1, "%ld", &a) == 1) {
                maxRpm = clampI(a, 0, SPEED_LIMIT_RPM);
            }
            break;
        case 's':
            stopNow();
            lastInputMs = millis();
            break;
    }
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type) {
        case WStype_CONNECTED:
            clients++;
            break;
        case WStype_DISCONNECTED:
            if (clients) {
                clients--;
            }
            inputLeft = inputRight = 0;    // ramps down; the page must reconnect to drive again
            break;
        case WStype_TEXT: {
            char msg[32];
            size_t n = min(length, sizeof(msg) - 1);
            memcpy(msg, payload, n);
            msg[n] = 0;
            handleMessage(msg);
            break;
        }
        default:
            break;
    }
}

static float ramp(float current, float target, float maxStep)
{
    if (target > current + maxStep) {
        return current + maxStep;
    }
    if (target < current - maxStep) {
        return current - maxStep;
    }
    return target;
}

static void sendToBoards()
{
    static uint32_t lastSendMs = 0;
    static bool     sendLeft = true;
    const uint32_t  interval = 1000 / (SEND_RATE_HZ * 2);    // frames alternate between the boards
    uint32_t now = millis();
    if (now - lastSendMs < interval) {
        return;
    }
    float dt = (now - lastSendMs) / 1000.0f;
    lastSendMs = now;

    bool phoneAlive = clients > 0 && now - lastInputMs < PHONE_TIMEOUT_MS;
    float targetLeft  = phoneAlive ? (float)inputLeft  * maxRpm / 1000.0f : 0;
    float targetRight = phoneAlive ? (float)inputRight * maxRpm / 1000.0f : 0;
    float step = RAMP_RPM_PER_S * dt;
    cmdLeft  = ramp(cmdLeft,  targetLeft,  step);
    cmdRight = ramp(cmdRight, targetRight, step);

    uint8_t frame[8];
    size_t len;
    if (sendLeft) {
        len = hoverSpeedFrame(frame, LEFT_ID,
                              (int16_t)clampI(lroundf(cmdLeft) * LEFT_DIR, -SPEED_LIMIT_RPM, SPEED_LIMIT_RPM));
    } else {
        len = hoverSpeedFrame(frame, RIGHT_ID,
                              (int16_t)clampI(lroundf(cmdRight) * RIGHT_DIR, -SPEED_LIMIT_RPM, SPEED_LIMIT_RPM));
    }
    Serial2.write(frame, len);
    sendLeft = !sendLeft;
}

static void readBoards()
{
    HoverAnswer a;
    while (Serial2.available()) {
        if (parser.feed((uint8_t)Serial2.read(), &a)) {
            if (a.slave == LEFT_ID) {
                answerLeft = a;
            } else if (a.slave == RIGHT_ID) {
                answerRight = a;
            }
        }
    }
}

static void appendWheel(String &s, const char *name, const HoverAnswer &a, int dir, float cmd)
{
    bool ok = a.receivedMs && millis() - a.receivedMs < 500;
    s += "\"";
    s += name;
    s += "\":{\"ok\":";
    s += ok ? "true" : "false";
    s += ",\"rpm\":";
    s += String(a.rpm * dir, 1);    // forward = positive, like the page
    s += ",\"cmd\":";
    s += String(cmd, 0);
    s += ",\"iq\":";
    s += String(a.iq * dir, 2);
    s += ",\"v\":";
    s += String(a.volt, 2);
    s += "}";
}

static void sendTelemetry()
{
    static uint32_t lastMs = 0;
    if (millis() - lastMs < 200 || clients == 0) {
        return;
    }
    lastMs = millis();
    String s = "{";
    appendWheel(s, "L", answerLeft, LEFT_DIR, cmdLeft);
    s += ",";
    appendWheel(s, "R", answerRight, RIGHT_DIR, cmdRight);
    s += ",\"max\":";
    s += maxRpm;
    s += ",\"crc\":";
    s += parser.badCrc;
    s += "}";
    ws.broadcastTXT(s);
}

// USB serial log (115200): the commands while they change or are non-zero, so the page can be
// tested without the boards connected.
static void logCommands()
{
    static uint32_t lastMs = 0;
    static long lastL = 0, lastR = 0;
    long l = lroundf(cmdLeft), r = lroundf(cmdRight);
    if (millis() - lastMs < 250 || (l == lastL && r == lastR && l == 0 && r == 0)) {
        return;
    }
    lastMs = millis();
    lastL = l;
    lastR = r;
    bool phoneAlive = clients > 0 && millis() - lastInputMs < PHONE_TIMEOUT_MS;
    Serial.printf("cmd left %4ld right %4ld rpm | input %5ld %5ld | max %ld | phone %s | boards %s %s\n",
                  l, r, (long)inputLeft, (long)inputRight, (long)maxRpm, phoneAlive ? "ok" : "none",
                  answerLeft.receivedMs && millis() - answerLeft.receivedMs < 500 ? "L" : "-",
                  answerRight.receivedMs && millis() - answerRight.receivedMs < 500 ? "R" : "-");
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX_PIN, HOVER_TX_PIN);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("Wi-Fi \"%s\", open http://%s\n", WIFI_SSID, WiFi.softAPIP().toString().c_str());

    http.on("/", []() { http.send_P(200, "text/html", WEB_PAGE); });
    http.onNotFound([]() { http.send(404, "text/plain", "not found"); });
    http.begin();

    ws.begin();
    ws.onEvent(onWsEvent);
}

void loop()
{
    http.handleClient();
    ws.loop();
    readBoards();
    sendToBoards();
    sendTelemetry();
    logCommands();
}
