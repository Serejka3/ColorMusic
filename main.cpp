#include <EEPROM.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_timer.h"

#include <FS.h>
#ifdef USE_LittleFS
#define SPIFFS LITTLEFS
#include <LITTLEFS.h>
#else
#include <SPIFFS.h>
#endif

#include "ESP8266FtpServer.h"
FtpServer ftpSrv;
//#include <ESPmDNS.h>
#include <Update.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include "FastLED.h"
//#include <FHT.h>         // преобразование Хартли

// ----- пины подключения
#define SOUND_R 36      // аналоговый пин вход аудио, правый канал
#define SOUND_L 39      // аналоговый пин вход аудио, левый канал
#define SOUND_R_FREQ 34 // аналоговый пин вход аудио для режима с частотами (через кондер)
#define LED_PIN 4       // пин DI светодиодной ленты
//
#define NUM_LEDS 26
#define MODE 6                 // режим при запуске
#define MONO 1                 // 1 - только один канал (ПРАВЫЙ!!!!! SOUND_R!!!!!), 0 - два канала
#define MAIN_LOOP 5            // период основного цикла отрисовки (по умолчанию 5)
#define EXP 1.4                // степень усиления сигнала (для более "резкой" работы) (по умолчанию 1.4)
#define EMPTY_COLOR HUE_PURPLE // цвет "не горящих" светодиодов. Будет чёрный, если яркость 0
#define MAX_COEF 1.8           // коэффициент громкости (максимальное равно срднему * этот коэф) (по умолчанию 1.8)
#define CURRENT_LIMIT 500      // лимит по току в МИЛЛИАМПЕРАХ, автоматически управляет яркостью (пожалей свой блок питания!) 0 - выключить лимит
#define FHT_N 64               // ширина спектра х2
#define LOG_OUT 1
#define STRIPE NUM_LEDS / 5
#define LOW_COLOR HUE_RED     // цвет низких частот
#define MID_COLOR HUE_GREEN   // цвет средних
#define HIGH_COLOR HUE_YELLOW // цвет высоких
#define LIGHT_SMOOTH 2
// ----- режим стробоскопа
uint16_t STROBE_PERIOD = 140;        // период вспышек, миллисекунды
#define STROBE_DUTY 20          // скважность вспышек (1 - 99) - отношение времени вспышки ко времени темноты
#define STROBE_COLOR HUE_YELLOW // цвет стробоскопа
#define STROBE_SAT 0            // насыщенность. Если 0 - цвет будет БЕЛЫЙ при любом цвете (0 - 255)
byte STROBE_SMOOTH = 200;        // скорость нарастания/угасания вспышки (0 - 255)
int freq_strobe_mode, light_mode;

uint16_t LOW_PASS = 100;       // нижний порог шумов режим VU, ручная настройка
uint16_t SPEKTR_LOW_PASS = 40; // нижний порог шумов режим спектра, ручная настройка
byte BRIGHTNESS = 50;                            // яркость по умолчанию (0 - 255)
float SMOOTH = 0.3;                             // коэффициент плавности анимации VU (по умолчанию 0.5)
byte EMPTY_BRIGHT = 30;                          // яркость "не горящих" светодиодов (0 - 255)
bool settings_mode, ONstate = true;
unsigned long main_timer, hue_timer, strobe_timer, running_timer, color_timer, rainbow_timer, eeprom_timer;

int Rlenght, Llenght;
float RsoundLevel, RsoundLevel_f;
float LsoundLevel, LsoundLevel_f;

byte this_mode = MODE;
int RcurrentLevel, LcurrentLevel;
float averageLevel = 50;
int maxLevel = 100;
int MAX_CH = NUM_LEDS / 2;
int hue;
float averK = 0.006;
int colorMusic[3];
byte count;
float index1 = (float)255 / MAX_CH; // коэффициент перевода для палитры
float RAINBOW_STEP = 5.00;          // шаг изменения цвета радуги
int thisBright[3], strobe_bright = 0;
int idx1 = MAX_CH;
unsigned long on_timer, btn_timer;
bool on_flag = true;
int COLOR_SPEED = 100;
int RAINBOW_PERIOD = 1;
float RAINBOW_STEP_2 = 0.5;
byte LIGHT_COLOR = 0; // начальный цвет подсветки
byte LIGHT_SAT = 255; // начальная насыщенность подсветки
int this_color;
// ----- режим бегущих частот
byte RUNNING_SPEED = 11;

// ----- режим анализатора спектра
byte HUE_START = 0;
byte HUE_STEP = 5;
bool running_flag[3];
float freq_max_f, rainbow_steps;
float freq_to_stripe = NUM_LEDS / 40; // /2 так как симметрия, и /20 так как 20 частот
int freq_f[32];
bool colorMusicFlash[3], strobeUp_flag, strobeDwn_flag;
unsigned int light_time = STROBE_PERIOD * STROBE_DUTY / 100;

// градиент-палитра от зелёного к красному
DEFINE_GRADIENT_PALETTE(soundlevel_gp){
    0, 0, 255, 0,     // green
    100, 255, 255, 0, // yellow
    150, 255, 100, 0, // orange
    200, 255, 50, 0,  // red
    255, 255, 0, 0    // red
};
CRGBPalette32 myPal = soundlevel_gp;

CRGB leds[NUM_LEDS];
CRGB light_color2;

// Версия программы
const char *prog_name = "ColorStrip";
const String prog_ver = "1.0 " + String(__DATE__) /*+ String(__TIME__)*/;
// Параметры по умолчанию
const String default_ap_ssid = "esp32_web";
const String default_ap_pwd = "Alco12345";
const String default_ssid = "MTS";
const String default_ssid_pwd = "123456";
//const IPAddress default_ip = IPAddress(192, 168, 1, 100);

bool espShouldReboot = false;
bool l_swfl = false;
uint32_t l_tic1000 = 0;
String g_mode;
const int32_t c_end_structure = 31415927; // Признак конца структуры

//Структура настроек
typedef struct pref_tag
{
  char ap_name[34];  // Имя точки доступа
  char ap_pwd[66];   // Пароль ТД
  char ssid[34];     // SSID сети
  char ssid_pwd[66]; // Пароль SSID
  char option;       // 1/2: AP/WIFI
  //IPAddress ip;      // Статический IP

  uint32_t status; // Признак того, что последняя версия данной структуры была хоть раз сохранена. Должен быть последним в структуре
} pref_tag;
pref_tag pref;

DynamicJsonDocument doc(2048);
String json;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Логирование в поле на клиенте
void weblog(String p_str)
{
  doc["log"] = p_str;
  json = "";
  serializeJsonPretty(doc, json);
  doc.clear();
  ws.textAll(json);
}

void weblogln(String p_str)
{
  weblog(p_str + "\n");
}

void send_wifi_list()
{
  bool l_ret_to_ap = false;
  doc.clear();
  if (WiFi.getMode() == WIFI_AP)
  {
    Serial.println("WiFi mode = WIFI_AP. Change to WIFI_STA...");
    WiFi.mode(WIFI_AP_STA);
    l_ret_to_ap = true;
  }
  int n = WiFi.scanNetworks();
  Serial.println("scanNetworks");
  Serial.println(n);
  doc["WiFiList"] = "";
  for (int i = 0; i < n; ++i)
  {
    JsonObject networks = doc.createNestedObject(String(i));
    networks["key"] = String(i);
    JsonObject network = networks.createNestedObject(String(i));
    network["ListElement"] = WiFi.SSID(i);
    network["encryption"] = WiFi.encryptionType(i);
    network["rssi"] = WiFi.RSSI(i);
    network["bssid"] = WiFi.BSSIDstr(i);
    network["channel"] = WiFi.channel(i);
  }
  json = "";
  serializeJsonPretty(doc, json);
  if (l_ret_to_ap)
  {
    Serial.println("Return to WIFI_AP mode...");
    WiFi.mode(WIFI_AP);
  }
  doc.clear();
  ws.textAll(json);
}

void onEvent(AsyncWebSocket *server,
             AsyncWebSocketClient *client,
             AwsEventType type,
             void *arg,
             uint8_t *data,
             size_t len)

{
  String str1;
  switch (type)
  {
  case WS_EVT_CONNECT:
  {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    weblogln(F("Start"));
    doc["ProgVer"] = prog_ver;
    doc["ProgName"] = prog_name;
    doc["ap_name"] = String(pref.ap_name);
    doc["ap_pwd"] = String(pref.ap_pwd);
    doc["wifi_name"] = String(pref.ssid);
    doc["wifi_pwd"] = String(pref.ssid_pwd);
    doc["option"] = String(pref.option);
    doc["MODE"] = "BACKLIGHT";

    json = "";
    serializeJsonPretty(doc, json);
    doc.clear();
    Serial.print("Sending ");
    Serial.println(json);
    client->text(json);
  }
  break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && (info->index == 0) && (info->len == len))
    {
      if (info->opcode == WS_TEXT)
      {
        data[len] = 0;
        Serial.print("data is ");
        Serial.println((char *)data);
        deserializeJson(doc, (char *)data);
        if (doc.containsKey("mode"))
        {
          g_mode = (const char *)doc["mode"];
        }
        else if (doc.containsKey("get_wifi_list"))
        {
          l_swfl = true;
        }
        else if (doc.containsKey("restart"))
        {
          espShouldReboot = true;
        }
        else if (doc.containsKey("wifi_option"))
        {
          str1 = (const char *)doc["wifi_option"];
          pref.option = str1[0];
        }
        else if (doc.containsKey("BRIGHTNESS"))
        {
          str1 = (const char *)doc["BRIGHTNESS"];
          BRIGHTNESS = str1.toInt();
          FastLED.setBrightness(BRIGHTNESS);
          Serial.println(BRIGHTNESS);
        }
        else if (doc.containsKey("MODE"))
        {
          str1 = (const char *)doc["MODE"];
          if (str1 == "RAINBOW1")
          {
            this_mode = 6;
            light_mode = 1;
            weblogln("RAINBOW1");
          }
          if (str1 == "RAINBOW2")
          {
            this_mode = 6;
            light_mode = 2;
            weblogln("RAINBOW2");
          }
          if (str1 == "COLOR_MUSIC")
          {
            this_mode = 1;
            weblogln("COLOR_MUSIC");
          }
          else if (str1 == "BACKLIGHT")
          {
            this_mode = 6;
            light_mode = 0;
            for (int i = 0; i < NUM_LEDS; i++)
              leds[i] = light_color2;
            weblogln("BACKLIGHT");
          }
          else if (str1 == "STROBE")
          {
            this_mode = 5;
            weblogln("STROBE");
          }
        }
        else if (doc.containsKey("BL_R"))
        {
          str1 = (const char *)doc["BL_R"];
          light_color2.r = str1.toInt();
          for (int i = 0; i < NUM_LEDS; i++)
            leds[i] = light_color2;
        }
        else if (doc.containsKey("BL_G"))
        {
          str1 = (const char *)doc["BL_G"];
          light_color2.g = str1.toInt();
          for (int i = 0; i < NUM_LEDS; i++)
            leds[i] = light_color2;
        }
        else if (doc.containsKey("BL_B"))
        {
          str1 = (const char *)doc["BL_B"];
          light_color2.b = str1.toInt();
          for (int i = 0; i < NUM_LEDS; i++)
            leds[i] = light_color2;
        }
        else if (doc.containsKey("save"))
        {
          str1 = (const char *)doc["ap_name"];
          str1.toCharArray(pref.ap_name, str1.length() + 1);
          Serial.println(pref.ap_name);

          str1 = (const char *)doc["ap_pwd"];
          str1.toCharArray(pref.ap_pwd, str1.length() + 1);
          Serial.println(pref.ap_pwd);

          str1 = (const char *)doc["ssid"];
          str1.toCharArray(pref.ssid, str1.length() + 1);
          Serial.println(pref.ssid);

          str1 = (const char *)doc["ssid_pwd"];
          str1.toCharArray(pref.ssid_pwd, str1.length() + 1);
          Serial.println(pref.ssid_pwd);

          pref.status = c_end_structure;
          EEPROM.put(0, pref);
          EEPROM.commit();
          weblogln("SAVED");
        }
        else
        {
          Serial.println("Other message");
        }
      }
      else
      {
        Serial.println("Received a ws message, but it isn't text");
      }
    }
    else
    {
      Serial.println("Received a ws message, but it didn't fit into one frame");
    }
  }
  break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void setup()
{
  int l_trying;
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  //Read preferences
  Serial.println("Size of pref: " + String(sizeof(pref)));
  EEPROM.begin(sizeof(pref));
  EEPROM.get(0, pref);

  // Если настройки не сохранялись, заполним дефолтными значениями
  if (pref.status != c_end_structure)
  {
    default_ap_ssid.toCharArray(pref.ap_name, default_ap_ssid.length() + 1);
    default_ap_pwd.toCharArray(pref.ap_pwd, default_ap_pwd.length() + 1);
    default_ssid.toCharArray(pref.ssid, default_ssid.length() + 1);
    default_ssid_pwd.toCharArray(pref.ssid_pwd, default_ssid_pwd.length() + 1);
    pref.option = '2';
  }
  Serial.println("option: " + char(pref.option));
  Serial.println("ap: " + String(pref.ap_name) + "/" + String(pref.ap_pwd));
  Serial.println("wifi: " + String(pref.ssid) + "/" + String(pref.ssid_pwd));

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  else
  {
    Serial.println("SPIFFS opened!");
  }

  if (pref.option == '2')
  { // Если задан режим WiFi
    // Try to Connecting Wi-Fi...
    l_trying = 3;
    WiFi.begin(pref.ssid, pref.ssid_pwd);

    while (WiFi.status() != WL_CONNECTED && l_trying > 0)
    {
      delay(2000);
      Serial.println("Try to connecting WiFi...");
      l_trying--;
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      // Print ESP Local IP Address
      Serial.println(WiFi.localIP());
    }
    else
      pref.option = '1';
  }

  if (pref.option == '1')
  {
    //WIFI AP
    WiFi.mode(WIFI_AP);
    int timeout = 0;
    while (!WiFi.softAP(pref.ap_name, pref.ap_pwd))
    {
      Serial.println(F("."));
      delay(250);
      if (timeout > 40) //delay 250ms so 10s = 40*250ms
      {
        Serial.println(F("[Wifi AP]... not start"));
      }
      timeout++;
    };
    Serial.println(F("[Wifi AP]... started"));
    Serial.println("[Wifi AP]... ip : " + WiFi.softAPIP().toString());
  }

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              //request->send(SPIFFS, "/index.html", String(), false, processor);
              request->send(SPIFFS, "/index.html", "text/html");
            });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/style.css", "text/css"); });
  /*
  // Route to load Smooothie library to show graphics
  server.on("/smoothie.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS , "/smoothie.js", "text/javascript"); });
*/
  // Иконка страницы
  server.on("/favicon.png", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/favicon.png", "image/png"); });

  server.on("^\\/gif(\\/.*)$", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              //String fileName = request->pathArg(0);
              Serial.println(request->pathArg(0));
              request->send(SPIFFS, request->pathArg(0), "image/gif");
            });

  server.on(
      "/updt", HTTP_POST, [](AsyncWebServerRequest *request)
      {
        espShouldReboot = !Update.hasError();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/html", espShouldReboot ? "<h1><strong>Update DONE</strong></h1><br><a href='/'>Return Home</a>" : "<h1><strong>Update FAILED</strong></h1><br><a href='/updt'>Retry?</a>");
        response->addHeader("Connection", "close");
        request->send(response);
      },
      [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
      {
        if (!index)
        {
          Serial.printf("Update Start: %s\n", filename.c_str());
          if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000))
          {
            Update.printError(Serial);
          }
        }
        if (!Update.hasError())
        {
          if (Update.write(data, len) != len)
          {
            Update.printError(Serial);
          }
        }
        if (final)
        {
          if (Update.end(true))
          {
            Serial.printf("Update Success: %uB\n", index + len);
          }
          else
          {
            Update.printError(Serial);
          }
        }
      });

/* ХЗ - не работает!
    // Set up mDNS responder:
    // - first argument is the domain name, in this example
    //   the fully-qualified domain name is "esp32.local"
    // - second argument is the IP address to advertise
    //   we send our IP address on the WiFi network
    if (!MDNS.begin("esp32")) {
        Serial.println("Error setting up MDNS responder!");
        while(1) {
            delay(1000);
        }
    }
    Serial.println("mDNS responder started");
*/
    // Start TCP (HTTP) server
    server.begin();
    Serial.println("TCP server started");

    // Add service to MDNS-SD
    //MDNS.addService("http", "tcp", 80);

  ftpSrv.begin("ftp", "1234"); //username, password for ftp


  FastLED.addLeds<WS2811, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  if (CURRENT_LIMIT > 0)
    FastLED.setMaxPowerInVoltsAndMilliamps(5, CURRENT_LIMIT);
  FastLED.setBrightness(BRIGHTNESS);
  light_color2.r = 255;
  light_color2.g = 255;
  light_color2.b = 255;
}

void HIGHS()
{
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CHSV(HIGH_COLOR, 255, thisBright[2]);
}
void MIDS()
{
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CHSV(MID_COLOR, 255, thisBright[1]);
}
void LOWS()
{
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CHSV(LOW_COLOR, 255, thisBright[0]);
}
void SILENCE()
{
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CHSV(EMPTY_COLOR, 255, EMPTY_BRIGHT);
}

void animation()
{
  // согласно режиму
  switch (this_mode)
  {
  case 0:
    count = 0;
    for (int i = (MAX_CH - 1); i > ((MAX_CH - 1) - Rlenght); i--)
    {
      leds[i] = ColorFromPalette(myPal, (count * index1)); // заливка по палитре " от зелёного к красному"
      count++;
    }
    count = 0;
    for (int i = (MAX_CH); i < (MAX_CH + Llenght); i++)
    {
      leds[i] = ColorFromPalette(myPal, (count * index1)); // заливка по палитре " от зелёного к красному"
      count++;
    }
    if (EMPTY_BRIGHT > 0)
    {
      CHSV this_dark = CHSV(EMPTY_COLOR, 255, EMPTY_BRIGHT);
      for (int i = ((MAX_CH - 1) - Rlenght); i > 0; i--)
        leds[i] = this_dark;
      for (int i = MAX_CH + Llenght; i < NUM_LEDS; i++)
        leds[i] = this_dark;
    }
    break;
  case 1:
    if (millis() - rainbow_timer > 30)
    {
      rainbow_timer = millis();
      hue = floor((float)hue + RAINBOW_STEP);
    }
    count = 0;
    for (int i = (MAX_CH - 1); i > ((MAX_CH - 1) - Rlenght); i--)
    {
      leds[i] = ColorFromPalette(RainbowColors_p, (count * index1) / 2 - hue); // заливка по палитре радуга
      count++;
    }
    count = 0;
    for (int i = (MAX_CH); i < (MAX_CH + Llenght); i++)
    {
      leds[i] = ColorFromPalette(RainbowColors_p, (count * index1) / 2 - hue); // заливка по палитре радуга
      count++;
    }
    if (EMPTY_BRIGHT > 0)
    {
      CHSV this_dark = CHSV(EMPTY_COLOR, 255, EMPTY_BRIGHT);
      for (int i = ((MAX_CH - 1) - Rlenght); i > 0; i--)
        leds[i] = this_dark;
      for (int i = MAX_CH + Llenght; i < NUM_LEDS; i++)
        leds[i] = this_dark;
    }
    break;
  case 2:
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (i < STRIPE)
        leds[i] = CHSV(HIGH_COLOR, 255, thisBright[2]);
      else if (i < STRIPE * 2)
        leds[i] = CHSV(MID_COLOR, 255, thisBright[1]);
      else if (i < STRIPE * 3)
        leds[i] = CHSV(LOW_COLOR, 255, thisBright[0]);
      else if (i < STRIPE * 4)
        leds[i] = CHSV(MID_COLOR, 255, thisBright[1]);
      else if (i < STRIPE * 5)
        leds[i] = CHSV(HIGH_COLOR, 255, thisBright[2]);
    }
    break;
  case 3:
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (i < NUM_LEDS / 3)
        leds[i] = CHSV(HIGH_COLOR, 255, thisBright[2]);
      else if (i < NUM_LEDS * 2 / 3)
        leds[i] = CHSV(MID_COLOR, 255, thisBright[1]);
      else if (i < NUM_LEDS)
        leds[i] = CHSV(LOW_COLOR, 255, thisBright[0]);
    }
    break;
  case 4:
    switch (freq_strobe_mode)
    {
    case 0:
      if (colorMusicFlash[2])
        HIGHS();
      else if (colorMusicFlash[1])
        MIDS();
      else if (colorMusicFlash[0])
        LOWS();
      else
        SILENCE();
      break;
    case 1:
      if (colorMusicFlash[2])
        HIGHS();
      else
        SILENCE();
      break;
    case 2:
      if (colorMusicFlash[1])
        MIDS();
      else
        SILENCE();
      break;
    case 3:
      if (colorMusicFlash[0])
        LOWS();
      else
        SILENCE();
      break;
    }
    break;
  case 5:
    if (strobe_bright > 0)
      for (int i = 0; i < NUM_LEDS; i++)
        leds[i] = CHSV(STROBE_COLOR, STROBE_SAT, strobe_bright);
    else
      for (int i = 0; i < NUM_LEDS; i++)
        leds[i] = CHSV(EMPTY_COLOR, 255, EMPTY_BRIGHT);
    break;
  case 6:
    switch (light_mode)
    {
    //-> klimov
    //case 0: for (int i = 0; i < NUM_LEDS; i++) leds[i] = CHSV(LIGHT_COLOR, LIGHT_SAT, 255);
    case 0:
      if (on_flag && (millis() - on_timer > 150))
      {
        //Serial.print(F("BRIGHTNESS ЫЫ = ")); Serial.println(BRIGHTNESS);
        if (idx1 >= 0)
        {
          leds[idx1].r = 127;
          leds[NUM_LEDS - idx1].r = 127;
        }
        if (idx1 > 7 && idx1 <= MAX_CH)
        {
          leds[idx1 + 1].r = 80;
          leds[NUM_LEDS - idx1 - 1].r = 80;
        }
        if (idx1 > 7 && idx1 <= (MAX_CH - 1))
        {
          leds[idx1 + 2].r = 25;
          leds[NUM_LEDS - idx1 - 2].r = 25;
        }
        if (idx1 > 7 && idx1 <= (MAX_CH - 2))
        {
          leds[idx1 + 3].r = 20;
          leds[NUM_LEDS - idx1 - 3].r = 20;
        }
        if (idx1 > 7 && idx1 <= (MAX_CH - 3))
        {
          leds[idx1 + 4].r = 15;
          leds[NUM_LEDS - idx1 - 4].r = 15;
        }
        if (idx1 > 7 && idx1 <= (MAX_CH - 4))
        {
          leds[idx1 + 5].r = 10;
          leds[NUM_LEDS - idx1 - 5].r = 10;
        }
        if (idx1 > 7 && idx1 <= (MAX_CH - 5))
        {
          leds[idx1 + 6].r = 5;
          leds[NUM_LEDS - idx1 - 6].r = 5;
        }
        if (idx1 > 7 && idx1 <= (MAX_CH - 6))
        {
          leds[idx1 + 7].r = 2;
          leds[NUM_LEDS - idx1 - 7].r = 2;
        }
        if (idx1 > 7 && idx1 <= (MAX_CH - 7))
        {
          leds[idx1 + 8].r = 0;
          leds[NUM_LEDS - idx1 - 8].r = 0;
        }

        if (idx1 <= (MAX_CH / 2) && idx1 >= 0)
        {
          leds[MAX_CH - ((MAX_CH / 2) - idx1) * 2] = light_color2;
          leds[NUM_LEDS - idx1 * 2] = light_color2;
          leds[MAX_CH - ((MAX_CH / 2) - idx1) * 2 - 1] = light_color2;
          leds[NUM_LEDS - idx1 * 2 - 1] = light_color2;
        }

        if (idx1-- == 0)
          on_flag = false;
        on_timer = millis();
      }
      //<- klimov
      break;
    case 1:
      if (millis() - color_timer > COLOR_SPEED)
      {
        color_timer = millis();
        if (++this_color > 255)
          this_color = 0;
      }
      for (int i = 0; i < NUM_LEDS; i++)
        leds[i] = CHSV(this_color, LIGHT_SAT, 255);
      break;
    case 2:
      if (millis() - rainbow_timer > COLOR_SPEED)
      {
        rainbow_timer = millis();
        this_color += RAINBOW_PERIOD;
        if (this_color > 255)
          this_color = 0;
        if (this_color < 0)
          this_color = 255;
      }
      rainbow_steps = this_color;
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CHSV((int)floor(rainbow_steps), 255, 255);
        rainbow_steps += RAINBOW_STEP_2;
        if (rainbow_steps > 255)
          rainbow_steps = 0;
        if (rainbow_steps < 0)
          rainbow_steps = 255;
      }
      break;
    }
    break;
  case 7:
    switch (freq_strobe_mode)
    {
    case 0:
      if (running_flag[2])
        leds[NUM_LEDS / 2] = CHSV(HIGH_COLOR, 255, thisBright[2]);
      else if (running_flag[1])
        leds[NUM_LEDS / 2] = CHSV(MID_COLOR, 255, thisBright[1]);
      else if (running_flag[0])
        leds[NUM_LEDS / 2] = CHSV(LOW_COLOR, 255, thisBright[0]);
      else
        leds[NUM_LEDS / 2] = CHSV(EMPTY_COLOR, 255, EMPTY_BRIGHT);
      break;
    case 1:
      if (running_flag[2])
        leds[NUM_LEDS / 2] = CHSV(HIGH_COLOR, 255, thisBright[2]);
      else
        leds[NUM_LEDS / 2] = CHSV(EMPTY_COLOR, 255, EMPTY_BRIGHT);
      break;
    case 2:
      if (running_flag[1])
        leds[NUM_LEDS / 2] = CHSV(MID_COLOR, 255, thisBright[1]);
      else
        leds[NUM_LEDS / 2] = CHSV(EMPTY_COLOR, 255, EMPTY_BRIGHT);
      break;
    case 3:
      if (running_flag[0])
        leds[NUM_LEDS / 2] = CHSV(LOW_COLOR, 255, thisBright[0]);
      else
        leds[NUM_LEDS / 2] = CHSV(EMPTY_COLOR, 255, EMPTY_BRIGHT);
      break;
    }
    leds[(NUM_LEDS / 2) - 1] = leds[NUM_LEDS / 2];
    if (millis() - running_timer > RUNNING_SPEED)
    {
      running_timer = millis();
      for (int i = 0; i < NUM_LEDS / 2 - 1; i++)
      {
        leds[i] = leds[i + 1];
        leds[NUM_LEDS - i - 1] = leds[i];
      }
    }
    break;
  case 8:
    byte HUEindex = HUE_START;
    for (int i = 0; i < NUM_LEDS / 2; i++)
    {
      byte this_bright = map(freq_f[(int)floor((NUM_LEDS / 2 - i) / freq_to_stripe)], 0, freq_max_f, 0, 255);
      this_bright = constrain(this_bright, 0, 255);
      leds[i] = CHSV(HUEindex, 255, this_bright);
      leds[NUM_LEDS - i - 1] = leds[i];
      HUEindex += HUE_STEP;
      if (HUEindex > 255)
        HUEindex = 0;
    }
    break;
  }
}

void mainLoop()
{
  // главный цикл отрисовки
  if (ONstate)
  {
    if (millis() - main_timer > MAIN_LOOP)
    {
      // сбрасываем значения
      RsoundLevel = 0;
      LsoundLevel = 0;

      // перваые два режима - громкость (VU meter)
      if (this_mode == 0 || this_mode == 1)
      {
        for (byte i = 0; i < 100; i++)
        {                                      // делаем 100 измерений
          RcurrentLevel = analogRead(SOUND_R); // с правого
          if (!MONO)
            LcurrentLevel = analogRead(SOUND_L); // и левого каналов

          if (RsoundLevel < RcurrentLevel)
            RsoundLevel = RcurrentLevel; // ищем максимальное
          if (!MONO)
            if (LsoundLevel < LcurrentLevel)
              LsoundLevel = LcurrentLevel; // ищем максимальное
        }

        // фильтруем по нижнему порогу шумов
        RsoundLevel = map(RsoundLevel, LOW_PASS, 2047, 0, 500);
        if (!MONO)
          LsoundLevel = map(LsoundLevel, LOW_PASS, 2047, 0, 500);

        // ограничиваем диапазон
        RsoundLevel = constrain(RsoundLevel, 0, 500);
        if (!MONO)
          LsoundLevel = constrain(LsoundLevel, 0, 500);

        // возводим в степень (для большей чёткости работы)
        RsoundLevel = pow(RsoundLevel, EXP);
        if (!MONO)
          LsoundLevel = pow(LsoundLevel, EXP);

        // фильтр
        RsoundLevel_f = RsoundLevel * SMOOTH + RsoundLevel_f * (1 - SMOOTH);
        if (!MONO)
          LsoundLevel_f = LsoundLevel * SMOOTH + LsoundLevel_f * (1 - SMOOTH);

        if (MONO)
          LsoundLevel_f = RsoundLevel_f; // если моно, то левый = правому

        // заливаем "подложку", если яркость достаточная
        if (EMPTY_BRIGHT > 5)
        {
          for (int i = 0; i < NUM_LEDS; i++)
            leds[i] = CHSV(EMPTY_COLOR, 255, EMPTY_BRIGHT);
        }

        // если значение выше порога - начинаем самое интересное
        if (RsoundLevel_f > 15 && LsoundLevel_f > 15)
        {

          // расчёт общей средней громкости с обоих каналов, фильтрация.
          // Фильтр очень медленный, сделано специально для автогромкости
          averageLevel = (float)(RsoundLevel_f + LsoundLevel_f) / 2 * averK + averageLevel * (1 - averK);

          // принимаем максимальную громкость шкалы как среднюю, умноженную на некоторый коэффициент MAX_COEF
          maxLevel = (float)averageLevel * MAX_COEF;

          // преобразуем сигнал в длину ленты (где MAX_CH это половина количества светодиодов)
          Rlenght = map(RsoundLevel_f, 0, maxLevel, 0, MAX_CH);
          Llenght = map(LsoundLevel_f, 0, maxLevel, 0, MAX_CH);

          // ограничиваем до макс. числа светодиодов
          Rlenght = constrain(Rlenght, 0, MAX_CH);
          Llenght = constrain(Llenght, 0, MAX_CH);

          animation(); // отрисовать
        }
      }
      /*
      // 3-5 режим - цветомузыка
      if (this_mode == 2 || this_mode == 3 || this_mode == 4 || this_mode == 7 || this_mode == 8) {
        analyzeAudio();
        colorMusic[0] = 0;
        colorMusic[1] = 0;
        colorMusic[2] = 0;
        for (int i = 0 ; i < 32 ; i++) {
          if (fht_log_out[i] < SPEKTR_LOW_PASS) fht_log_out[i] = 0;
        }
        // низкие частоты, выборка со 2 по 5 тон (0 и 1 зашумленные!)
        for (byte i = 2; i < 6; i++) {
          if (fht_log_out[i] > colorMusic[0]) colorMusic[0] = fht_log_out[i];
        }
        // средние частоты, выборка с 6 по 10 тон
        for (byte i = 6; i < 11; i++) {
          if (fht_log_out[i] > colorMusic[1]) colorMusic[1] = fht_log_out[i];
        }
        // высокие частоты, выборка с 11 по 31 тон
        for (byte i = 11; i < 32; i++) {
          if (fht_log_out[i] > colorMusic[2]) colorMusic[2] = fht_log_out[i];
        }
        freq_max = 0;
        for (byte i = 0; i < 30; i++) {
          if (fht_log_out[i + 2] > freq_max) freq_max = fht_log_out[i + 2];
          if (freq_max < 5) freq_max = 5;

          if (freq_f[i] < fht_log_out[i + 2]) freq_f[i] = fht_log_out[i + 2];
          if (freq_f[i] > 0) freq_f[i] -= LIGHT_SMOOTH;
          else freq_f[i] = 0;
        }
        freq_max_f = freq_max * averK + freq_max_f * (1 - averK);
        for (byte i = 0; i < 3; i++) {
          colorMusic_aver[i] = colorMusic[i] * averK + colorMusic_aver[i] * (1 - averK);  // общая фильтрация
          colorMusic_f[i] = colorMusic[i] * SMOOTH_FREQ + colorMusic_f[i] * (1 - SMOOTH_FREQ);      // локальная
          if (colorMusic_f[i] > ((float)colorMusic_aver[i] * MAX_COEF_FREQ)) {
            thisBright[i] = 255;
            colorMusicFlash[i] = true;
            running_flag[i] = true;
          } else colorMusicFlash[i] = false;
          if (thisBright[i] >= 0) thisBright[i] -= SMOOTH_STEP;
          if (thisBright[i] < EMPTY_BRIGHT) {
            thisBright[i] = EMPTY_BRIGHT;
            running_flag[i] = false;
          }
        }
        animation();
      }
*/

      
      if (this_mode == 5) {
        if ((long)millis() - strobe_timer > STROBE_PERIOD) {
          strobe_timer = millis();
          strobeUp_flag = true;
          strobeDwn_flag = false;
        }
        if ((long)millis() - strobe_timer > light_time) {
          strobeDwn_flag = true;
        }
        if (strobeUp_flag) {                    // если настало время пыхнуть
          if (strobe_bright < 255)              // если яркость не максимальная
            strobe_bright += STROBE_SMOOTH;     // увелчить
          if (strobe_bright > 255) {            // если пробили макс. яркость
            strobe_bright = 255;                // оставить максимум
            strobeUp_flag = false;              // флаг опустить
          }
        }

        if (strobeDwn_flag) {                   // гаснем
          if (strobe_bright > 0)                // если яркость не минимальная
            strobe_bright -= STROBE_SMOOTH;     // уменьшить
          if (strobe_bright < 0) {              // если пробили мин. яркость
            strobeDwn_flag = false;
            strobe_bright = 0;                  // оставить 0
          }
        }
        animation();
      }
      
      if (this_mode == 6)
        animation();

      //if (!IRLremote.receiving())    // если на ИК приёмник не приходит сигнал (без этого НЕ РАБОТАЕТ!)
      FastLED.show(); // отправить значения на ленту

      if (this_mode == 1 || this_mode == 2)
        FastLED.clear();     // очистить массив пикселей
      main_timer = millis(); // сбросить таймер
    }
  }
}

void loop()
{
  ftpSrv.handleFTP();
  ws.cleanupClients();
  if (espShouldReboot)
  {
    Serial.println(F("Esp reboot ..."));
    delay(150);
    ESP.restart();
  }
  if (l_swfl)
  {
    l_swfl = false;
    send_wifi_list();
  }

  if (millis() - l_tic1000 >= 1000)
  {
    doc.clear();
    doc["millis"] = millis();
    doc["BRIGHTNESS"] = BRIGHTNESS;
    doc["BL_R"] = light_color2.r;
    doc["BL_G"] = light_color2.g;
    doc["BL_B"] = light_color2.b;
    if (pref.option == '2')
      doc["rssi"] = WiFi.RSSI();
    json = "";
    serializeJsonPretty(doc, json);
    ws.textAll(json);
    l_tic1000 = millis();
  }

  mainLoop();
}

/*
void analyzeAudio() {
  for (int i = 0 ; i < FHT_N ; i++) {
    int sample = analogRead(SOUND_R_FREQ);
    fht_input[i] = sample; // put real data into bins
  }
  fht_window();  // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run();     // process the data in the fht
  fht_mag_log(); // take the output of the fht
}
*/
