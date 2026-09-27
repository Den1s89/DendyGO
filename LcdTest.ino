/**
 * =============================================================================
 *  DendyGO / LcdTest.ino  —  диагностика вывода на LCD (без эмулятора)
 * =============================================================================
 *  Включается в DendyConfig.h:  DENDY_LCD_TEST 1.
 *  Лежит ТОЛЬКО в папке скетча DendyGO (рядом с DendyConfig.h): пины, размер
 *  панели и частоты заданы в общем конфиге, в отдельный скетч его копировать
 *  нельзя (получится сборка с кучей «X was not declared in this scope»).
 *
 *  Тест не запускает эмулятор, а перебирает готовые конфигурации дисплея
 *  (частота SPI, драйвер панели ST7789/ILI9341/ST7796, invert, rgb_order,
 *  размеры панели) и на каждой рисует тест-картинку. Номер и параметры
 *  активной конфигурации написаны на самом экране и в Serial.
 *
 *  ЭКРАН 1 (≈3.5 с):
 *    1) восемь полос чистых цветов (красная, зелёная, синяя, жёлтая, голубая,
 *       сиреневая, белая, серая) — проверка цветов, инверсии и геометрии;
 *    2) четыре блока ОДНОЙ И ТОЙ ЖЕ картинки (слева «клетка» 1 пиксель —
 *       проверка целостности сигнала SPI, справа 8 цветных ячеек — цвета),
 *       выведенной четырьмя способами:
 *         M1 pushImage    из внутр. RAM   (эталон: панель без DMA)
 *         M2 pushImageDMA из внутр. RAM   (DMA, но из «правильной» памяти)
 *         M3 pushImage    из PSRAM        (кадр игры без DMA)
 *         M4 pushImageDMA из PSRAM        (кадр игры, как было в DendyGO)
 *
 *  ЭКРАН 2 (≈5 с): движущаяся картинка 256x240 из буфера в PSRAM — ровно тот
 *    путь, которым выводится кадр NES. Три фазы по 1.6 с, в подписи указано,
 *    какой цвет ДОЛЖЕН быть на картинке:
 *         M1 pushImage      (PSRAM, через кэш — эталон)
 *         M2 pushImageDMA   (PSRAM, без сброса кэша — как было)
 *         M3 pushImageDMA   (PSRAM + esp_cache_msync — сброс кэша перед DMA)
 *    Если на фазе M2 цвет на экране не тот, что в подписи, а на M1/M3 тот —
 *    виноват путь «DMA прямо из PSRAM». Если мусор на всех трёх, а на экране 1
 *    «клетка» рваная — виноват SPI (частота/провода/драйвер панели).
 *
 *  РЕЗУЛЬТАТ: конфигурация, где полосы правильных цветов, «клетка» ровная, а
 *  картинка на экране 2 не отстаёт, — рабочая. Её номер и параметры видно на
 *  экране и в логе; их и надо перенести в DendyConfig.h (DISPLAY_* / freq).
 *
 *  Управление: короткое нажатие BOOT (GPIO0 -> GND) замораживает текущую
 *  конфигурацию (сохраняется в NVS, переживает перезагрузку), чтобы её было
 *  удобно рассматривать; ещё одно нажатие — снова перебор. Либо сразу задать
 *  DENDY_LCD_TEST_CFG в DendyConfig.h (0..8) — тест стартует с неё.
 * =============================================================================
 */

// Файл — часть скетча DendyGO: пины берутся из общего DendyConfig.h.
// Если файл скопировали в другую папку (без DendyConfig.h) — говорим об этом
// одной понятной ошибкой вместо десятка «X was not declared in this scope».
#if !__has_include("DendyConfig.h")
#error "LcdTest.ino rabotaet tolko v papke sketcha DendyGO (rjadom s DendyConfig.h): postav DENDY_LCD_TEST 1 v DendyConfig.h i zalej obychyj sketch DendyGO."
#endif

#include "DendyConfig.h"          // пины, размеры панели, частота SPI, флаги
#include <Preferences.h>
#include <LovyanGFX.hpp>

#if __has_include(<esp_cache.h>)
  #include <esp_cache.h>
  #define LCDT_HAVE_MSYNC 1
#else
  #define LCDT_HAVE_MSYNC 0
#endif

#ifndef DENDY_LOG
  #define DENDY_LOG(...)  Serial.printf(__VA_ARGS__)
#endif

// ------------------------------- параметры теста -----------------------------
#define LCDT_BTN_PIN        0        // BOOT на большинстве плат ESP32-S3
#define LCDT_SCREEN1_MS     3500     // показ экрана 1, мс
#define LCDT_SCREEN2_MS     5000     // показ экрана 2, мс
#define LCDT_SUBPHASE_MS    1600     // длительность одной фазы на экране 2
#define LCDT_FB_W           256      // ширина картинки-кадра (как у NES)
#define LCDT_FB_H           240      // высота картинки-кадра
#define LCDT_PAT_W          64       // ширина маленькой картинки (внутр. RAM)
#define LCDT_PAT_H          40       // высота маленькой картинки

// --------------------------- конфигурации дисплея ---------------------------
enum LcdTPanel : uint8_t { LCDT_P7789, LCDT_P9341, LCDT_P7796 };

struct LcdTCfg {
  const char* name;      // подпись на экране и в логе
  uint8_t  panel;        // тип панели (LcdTPanel)
  uint16_t w, h;         // panel_width / panel_height (до поворота)
  uint16_t ox, oy;       // offset_x / offset_y
  bool     inv;          // cfg.invert     (негатив/норма)
  bool     bgr;          // cfg.rgb_order  (перестановка красный<->синий)
  uint32_t freq;         // частота SPI на запись
};

static const LcdTCfg lcdtCfgs[] = {
  { "7789 80M",       LCDT_P7789, 240, 320, 0,  0, true,  false, 80000000UL },  // как было
  { "7789 40M",       LCDT_P7789, 240, 320, 0,  0, true,  false, 40000000UL },
  { "7789 20M",       LCDT_P7789, 240, 320, 0,  0, true,  false, 20000000UL },
  { "7789 40M BGR",   LCDT_P7789, 240, 320, 0,  0, true,  true,  40000000UL },
  { "7789 40M inv0",  LCDT_P7789, 240, 320, 0,  0, false, false, 40000000UL },
  { "9341 40M",       LCDT_P9341, 240, 320, 0,  0, false, false, 40000000UL },
  { "9341 40M BGR",   LCDT_P9341, 240, 320, 0,  0, false, true,  40000000UL },
  { "7796 320x480",   LCDT_P7796, 320, 480, 0,  0, false, false, 40000000UL },
  { "7789 240x240",   LCDT_P7789, 240, 240, 0, 80, true,  false, 40000000UL },
};
#define LCDT_N ((int)(sizeof(lcdtCfgs) / sizeof(lcdtCfgs[0])))

// ------------------------- устройство: шина + панели ------------------------
// Одна шина SPI2 и один объект-устройство; панели переключаются через
// setPanel(). Перед сменой панели шина освобождается (releaseBus), иначе
// повторная spi_bus_initialize() внутри LovyanGFX завершится ошибкой.
class LcdTDev : public lgfx::LGFX_Device {
public:
  lgfx::Bus_SPI       bus;
  lgfx::Light_PWM     light;
  lgfx::Panel_ST7789  pan7789;
  lgfx::Panel_ILI9341 pan9341;
  lgfx::Panel_ST7796  pan7796;

  LcdTDev() {
    auto b        = bus.config();
    b.spi_host    = SPI2_HOST;
    b.spi_mode    = 0;
    b.freq_write  = 40000000UL;
    b.freq_read   = 8000000UL;
    b.spi_3wire   = false;
    b.use_lock    = true;
    b.dma_channel = SPI_DMA_CH_AUTO;
    b.pin_sclk    = PIN_TFT_SCLK;
    b.pin_mosi    = PIN_TFT_MOSI;
    b.pin_miso    = -1;
    b.pin_dc      = PIN_TFT_DC;
    bus.config(b);

    auto l        = light.config();
    l.pin_bl      = PIN_TFT_BL;
    l.invert      = false;
    l.freq        = 12000;
    l.pwm_channel = 7;
    light.config(l);
  }

  // Настроить конкретную панель (шаблон — чтобы работать с её типом config_t).
  template <typename PANEL>
  void cfgPanel(PANEL& p, const LcdTCfg& c) {
    p.setBus(&bus);
    p.setLight(&light);
    auto pc          = p.config();
    pc.pin_cs        = PIN_TFT_CS;
    pc.pin_rst       = PIN_TFT_RST;
    pc.pin_busy      = -1;
    pc.panel_width   = c.w;
    pc.panel_height  = c.h;
    pc.memory_width  = c.w;
    pc.memory_height = c.h;
    pc.offset_x      = c.ox;
    pc.offset_y      = c.oy;
    pc.readable      = false;
    pc.invert        = c.inv;
    pc.rgb_order     = c.bgr;
    pc.dlen_16bit    = false;
    pc.bus_shared    = false;
    p.config(pc);
  }

  // Включить конфигурацию: освободить шину, задать частоту и панель, init().
  void begin(const LcdTCfg& c, uint8_t rotation) {
    if (_active) { releaseBus(); _active = false; }

    auto b       = bus.config();
    b.freq_write = c.freq;
    b.freq_read  = (c.freq < 16000000UL) ? 8000000UL : 16000000UL;
    bus.config(b);

    switch (c.panel) {
      case LCDT_P9341: cfgPanel(pan9341, c); setPanel(&pan9341); break;
      case LCDT_P7796: cfgPanel(pan7796, c); setPanel(&pan7796); break;
      default:         cfgPanel(pan7789, c); setPanel(&pan7789); break;
    }

    init();                      // Panel::init() + Bus_SPI::init() (spi_bus_initialize)
    setRotation(rotation);
    setBrightness(DISPLAY_BRIGHTNESS);
    _active = true;
  }

private:
  bool _active = false;
};

static LcdTDev lcdtDev;

// ---------------------------- состояние теста -------------------------------
static Preferences lcdtPref;
static int      lcdtIndex  = 0;
static bool     lcdtFrozen = false;
static uint32_t lcdtBtnAt  = 0;

// Пауза с опросом кнопки BOOT: true — кнопку нажали и отпустили.
static bool lcdtWait(uint32_t ms)
{
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (digitalRead(LCDT_BTN_PIN) == LOW) {
      if (lcdtBtnAt == 0) lcdtBtnAt = millis();
      else if (millis() - lcdtBtnAt > 40) {          // антидребезг
        while (digitalRead(LCDT_BTN_PIN) == LOW) delay(5);
        lcdtBtnAt = 0;
        return true;
      }
    } else {
      lcdtBtnAt = 0;
    }
    delay(5);
  }
  return false;
}

// ------------------------------- тест-картинки ------------------------------
// Маленькая картинка 64x40: верхние 20 строк — «клетка» 1 пиксель (любая
// потеря/сдвиг байта превращают её в серую муть), нижние 20 строк — восемь
// цветных ячеек (проверка цвета и порядка каналов).
static uint16_t lcdtPat[LCDT_PAT_W * LCDT_PAT_H];        // живёт во внутренней RAM
static uint16_t* lcdtPs  = nullptr;                      // она же в PSRAM
static uint16_t* lcdtFb  = nullptr;                      // «кадр» 256x240 в PSRAM

static void lcdtBuildPattern(uint16_t* dst)
{
  static const uint16_t col[8] = { TFT_RED, TFT_GREEN, TFT_BLUE, TFT_YELLOW,
                                   TFT_CYAN, TFT_MAGENTA, TFT_WHITE, 0x8410 };
  for (int y = 0; y < LCDT_PAT_H; ++y) {
    for (int x = 0; x < LCDT_PAT_W; ++x) {
      dst[y * LCDT_PAT_W + x] = (y < 20) ? (((x + y) & 1) ? 0xFFFF : 0x0000)
                                         : col[(x / 8) & 7];
    }
  }
}

// Сброс кэша процессора для буфера в PSRAM перед передачей через DMA.
// Без этого SPI-DMA на ESP32-S3 может прочитать в PSRAM «протухшие» строки
// кэша: картинка на экране оказывается смесью старого и нового кадра.
static void lcdtCacheWriteBack(const void* ptr, size_t bytes)
{
#if LCDT_HAVE_MSYNC
  const uintptr_t a0 = ((uintptr_t)ptr) & ~(uintptr_t)63;                  // кэш-линия 64 Б
  const uintptr_t a1 = (((uintptr_t)ptr) + bytes + 63) & ~(uintptr_t)63;
  esp_cache_msync((void*)a0, a1 - a0,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
#else
  (void)ptr; (void)bytes;
#endif
}

// ------------------------------- ЭКРАН 1 ------------------------------------
// ВНИМАНИЕ: в прототипах функций (.ino -> автопрототипы Arduino IDE) не должно
// быть типов из этого файла, иначе сгенерированный прототип оказывается выше
// определения типа и сборка падает с «does not name a type». Поэтому здесь
// только int, а конфигурация берётся из таблицы внутри.
static bool lcdtScreen1(int idx)
{
  const LcdTCfg& c = lcdtCfgs[idx];
  auto& d = lcdtDev;
  const int W = d.width(), H = d.height();

  static const uint16_t bar[8] = { TFT_RED, TFT_GREEN, TFT_BLUE, TFT_YELLOW,
                                   TFT_CYAN, TFT_MAGENTA, TFT_WHITE, 0x8410 };
  const bool oneRow  = (W >= 300);
  const int  rows    = oneRow ? 1 : 2;
  const int  blockH  = 9 + LCDT_PAT_H;                 // подпись + картинка
  const int  colW    = LCDT_PAT_W;

  d.fillScreen(TFT_BLACK);
  d.setTextDatum(lgfx::textdatum_t::top_left);
  d.setTextWrap(false);

  const bool big = (W >= 300);
  d.setTextSize(big ? 2 : 1);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setCursor(4, 2);
  d.printf("%d/%d %s", idx + 1, LCDT_N, c.name);

  d.setTextSize(1);
  d.setTextColor(0xFFE0, TFT_BLACK);                   // жёлтый
  d.setCursor(4, big ? 22 : 14);
  d.printf("panel %ux%u off %u,%u inv=%d bgr=%d %luMHz", c.w, c.h, c.ox, c.oy,
           (int)c.inv, (int)c.bgr, (unsigned long)(c.freq / 1000000UL));

  // Полосы: сначала подбираем высоту полосы так, чтобы всё влезло в экран.
  int bh = 14;
  while (bh > 6 && (34 + 8 * bh + 8 + rows * (blockH + 6)) > H) --bh;
  if (bh < 6) bh = 6;
  const int by = big ? 34 : 26;
  for (int i = 0; i < 8; ++i) d.fillRect(0, by + i * bh, W, bh, bar[i]);

  // Четыре блока одной картинки, выведенные четырьмя разными способами.
  const char* names[4] = { "M1 push", "M2 DMA", "M3 PS/push", "M4 PS/DMA" };
  const int gap  = oneRow ? ((W - 4 * colW) / 5) : 24;
  const int x0   = oneRow ? gap : (W - (2 * colW + gap)) / 2;
  int y = by + 8 * bh + 8;

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < 2; ++col) {
      const int m  = row * 2 + col;
      const int x  = x0 + col * (colW + gap);
      d.setTextSize(1);
      d.setTextColor(TFT_CYAN, TFT_BLACK);
      d.setCursor(x, y);
      d.print(names[m]);
      const int iy = y + 9;
      switch (m) {
        case 0: d.pushImage(x, iy, LCDT_PAT_W, LCDT_PAT_H, lcdtPat); break;
        case 1: d.pushImageDMA(x, iy, LCDT_PAT_W, LCDT_PAT_H, lcdtPat); d.waitDMA(); break;
        case 2: if (lcdtPs) d.pushImage(x, iy, LCDT_PAT_W, LCDT_PAT_H, lcdtPs); break;
        default:
          if (lcdtPs) {
            lcdtCacheWriteBack(lcdtPs, sizeof(lcdtPat));       // «правильный» вариант
            d.pushImageDMA(x, iy, LCDT_PAT_W, LCDT_PAT_H, lcdtPs);
            d.waitDMA();
          }
          break;
      }
    }
    y += blockH + 6;
  }

  return lcdtWait(LCDT_SCREEN1_MS);
}

// ------------------------------- ЭКРАН 2 ------------------------------------
// Путь вывода кадра игры: картинка 256x240 из буфера в PSRAM. Три фазы, чтобы
// отделить «DMA не видит свежие данные в PSRAM» от «врёт SPI/панель».
static bool lcdtScreen2(int idx)
{
  const LcdTCfg& c = lcdtCfgs[idx];
  auto& d = lcdtDev;
  const int W = d.width(), H = d.height();
  const int dstX = (W > LCDT_FB_W) ? ((W - LCDT_FB_W) / 2) : 0;
  const int dstY = (H > LCDT_FB_H) ? ((H - LCDT_FB_H) / 2) : 0;

  if (!lcdtFb) {
    d.fillScreen(TFT_BLACK);
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.setTextSize(1);
    d.setCursor(4, 4);
    d.print("net PSRAM dlja kadra 256x240");
    return lcdtWait(LCDT_SCREEN2_MS);
  }

  static const char*     lbl[3] = { "M1 pushImage", "M2 pushImageDMA", "M3 DMA+msync" };
  static const uint16_t  bgc[3] = { TFT_RED,  TFT_GREEN, TFT_BLUE };
  static const char*     bgn[3] = { "fon=KRASNYJ", "fon=ZELENYJ", "fon=SINIJ" };

  uint32_t step = 0;
  for (int phase = 0; phase < 3; ++phase) {
    const uint32_t t0 = millis();
    while (millis() - t0 < LCDT_SUBPHASE_MS) {
      // 1) перерисовать картинку процессором: «свежие» данные заведомо в кэше
      uint16_t* f    = lcdtFb;
      const int barX = (int)((step * 8) & 0x00FF);        // белая полоса ползёт вправо
      for (int y = 0; y < LCDT_FB_H; ++y) {
        const bool line = ((y & 31) < 2);                 // метки каждые 32 строки
        for (int x = 0; x < LCDT_FB_W; ++x) {
          const bool vline = (x >= barX && x < barX + 8);
          f[y * LCDT_FB_W + x] = (vline || line) ? 0xFFFF : bgc[phase];
        }
      }
      ++step;

      // 2) вывести кадр одним из способов.
      //    Фаза 2 намеренно БЕЗ сброса кэша — именно так работал pushFrame()
      //    в DendyGO; если на ней картинка «отстаёт»/мешается, виноват DMA.
      switch (phase) {
        case 0:                                            // без DMA (через кэш)
          d.pushImage(dstX, dstY, LCDT_FB_W, LCDT_FB_H, f);
          break;
        case 1:                                            // DMA как было: без sync
          d.pushImageDMA(dstX, dstY, LCDT_FB_W, LCDT_FB_H, f);
          d.waitDMA();
          break;
        default:                                           // DMA + sync (сброс кэша)
          lcdtCacheWriteBack(f, (size_t)LCDT_FB_W * LCDT_FB_H * sizeof(uint16_t));
          d.pushImageDMA(dstX, dstY, LCDT_FB_W, LCDT_FB_H, f);
          d.waitDMA();
          break;
      }

      // 3) подпись — рисуется обычным путём панели, поэтому всегда верна
      d.fillRect(0, 0, W, 24, TFT_BLACK);
      d.setTextSize(1);
      d.setTextColor(TFT_WHITE, TFT_BLACK);
      d.setCursor(2, 2);
      d.printf("%d/%d %s", idx + 1, LCDT_N, c.name);
      d.setTextColor(TFT_YELLOW, TFT_BLACK);
      d.setCursor(2, 12);
      d.printf("%d/3 %s  (%s)", phase + 1, lbl[phase], bgn[phase]);

      if (lcdtWait(30)) return true;                       // кнопка — выходим
    }
    DENDY_LOG("[LCDT] cfg %d, faza %d/3 %s (%s) proshla\n",
              idx, phase + 1, lbl[phase], bgn[phase]);
  }
  return false;
}

// =============================================================================
//  Запуск теста (вызывается из setup() при DENDY_LCD_TEST = 1)
// =============================================================================
void runLcdTest()
{
  Serial.begin(DENDY_SERIAL_BAUD);
  delay(200);
  pinMode(LCDT_BTN_PIN, INPUT_PULLUP);

  lcdtPref.begin("lcdtest", false);
  lcdtIndex  = (int)lcdtPref.getInt("cfg", 0);
  lcdtFrozen = lcdtPref.getBool("frozen", false);
  if ((int)DENDY_LCD_TEST_CFG >= 0) {                    // зафиксировано в конфиге
    lcdtIndex  = (int)DENDY_LCD_TEST_CFG;
    lcdtFrozen = true;
  }
  if (lcdtIndex < 0 || lcdtIndex >= LCDT_N) lcdtIndex = 0;

  lcdtBuildPattern(lcdtPat);
  lcdtPs = (uint16_t*)ps_malloc(sizeof(lcdtPat));
  if (lcdtPs) lcdtBuildPattern(lcdtPs);
  lcdtFb = (uint16_t*)ps_malloc((size_t)LCDT_FB_W * LCDT_FB_H * sizeof(uint16_t));

  DENDY_LOG("\n=== DendyLCDTest (bez emuljatora) ===\n");
  DENDY_LOG("[LCDT] konfiguracij %d, start %d, zamorozheno=%d\n",
            LCDT_N, lcdtIndex, (int)lcdtFrozen);
  DENDY_LOG("[LCDT] kartinka 64x40 vnutr. RAM %u bayt; PSRAM: 64x40 %s, 256x240 %s\n",
            (unsigned)sizeof(lcdtPat), lcdtPs ? "OK" : "NET", lcdtFb ? "OK" : "NET");
  DENDY_LOG("[LCDT] knopka BOOT (GPIO%d, na GND) - zamorozit/otmenit vybor\n", LCDT_BTN_PIN);

  for (;;) {
    const LcdTCfg& c = lcdtCfgs[lcdtIndex];
    DENDY_LOG("[LCDT] cfg %d/%d: %s | %ux%u off %u,%u inv=%d bgr=%d %lu MHz\n",
              lcdtIndex, LCDT_N, c.name, c.w, c.h, c.ox, c.oy,
              (int)c.inv, (int)c.bgr, (unsigned long)(c.freq / 1000000UL));

    lcdtDev.begin(c, DISPLAY_ROTATION);
    DENDY_LOG("[LCDT] display %dx%d rot %d\n",
              lcdtDev.width(), lcdtDev.height(), lcdtDev.getRotation());

    bool pressed = lcdtScreen1(lcdtIndex);
    if (!pressed) pressed = lcdtScreen2(lcdtIndex);

    if (pressed) {                                       // BOOT: заморозить/отменить
      lcdtFrozen = !lcdtFrozen;
      lcdtPref.putBool("frozen", lcdtFrozen);
      lcdtPref.putInt("cfg", lcdtIndex);
      DENDY_LOG("[LCDT] %s: cfg %d (%s)\n",
                lcdtFrozen ? "ZAMOROZENO" : "perebor snova", lcdtIndex, c.name);
    }
    if (!lcdtFrozen) {
      lcdtIndex = (lcdtIndex + 1) % LCDT_N;
      lcdtPref.putInt("cfg", lcdtIndex);
    }
  }
}

