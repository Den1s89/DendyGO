/**
 * =============================================================================
 *  DENDYGO 1.0 — портативная NES/Dendy на ESP32-S3 (N16R8) + ST7789 + MAX98357
 * =============================================================================
 *
 *  ЧТО ЭТО
 *  Полноценный эмулятор NES (собственное ядро 6502 + PPU 2C02 + APU 2A03),
 *  который берёт образы .nes из LittleFS (внутренней флеш-памяти) и выводит
 *  картинку на SPI-дисплей ST7789, а звук — на I2S-кодек MAX98357.
 *
 *  ЖЕЛЕЗО (см. DendyConfig.h — там же меняются пины)
 *    Дисплей ST7789 : SCK=11  MOSI=12  CS=10  DC=13  RST=14  BLK=21
 *    Звук MAX98357  : BCLK=16 LRCK=15  DIN=17   (SD — ВХОД режима: на GND = SHUTDOWN!)
 *    Кнопки SX1509  : SDA=8   SCL=9    INT=3    (I2C №2, адрес 0x3E)
 *    !! GPIO33..37 заняты Octal PSRAM (N16R8) — использовать нельзя.
 *
 *  КНОПКИ (раскладка SX1509 -> функция: DendyConfig.h, блок 4; активный уровень LOW)
 *    вывод 0,1,2,3 : Вправо, Вниз, Вверх, Влево
 *    вывод 4  : REGION — TV AUTO/NTSC/PAL (только в меню)
 *    вывод 5  : громкость «−» (авто-повтор при удержании)
 *    вывод 6  : громкость «+» (авто-повтор при удержании)
 *    вывод 7  : MENU (короткое нажатие — Reset игры, удержание 400 мс — меню)
 *    вывод 8  : ЯРКОСТЬ экрана (по кругу 5, 15, 25, 50, 75, 100 %; при включении 70 %)
 *    вывод 9,10 : Turbo A, Turbo B (авто-повтор ~15 Гц)
 *    вывод 11,12,13,14 : A, B, Select, Start
 *    вывод 15 : свободен
 *    Громкость: отдельные кнопки (выводы 6 и 5) ИЛИ Select + Вверх/Вниз
 *    Один вывод = одна кнопка: за этим следит static_assert (sxPinsUnique, блок 8)
 *
 *  КАК ЗАЛИТЬ ИГРЫ (карты памяти нет, всё во внутренней флеш-памяти)
 *    ПОДРОБНАЯ ИНСТРУКЦИЯ — README.md рядом с этим файлом (настройки платы,
 *    таблица разделов, точные команды mklittlefs/esptool, диагностика).
 *    1) Соберите скетч и загрузите его в плату (Arduino IDE 2.x / PlatformIO).
 *       Файловая система LittleFS создастся автоматически.
 *       ВАЖНО: в проекте лежит partitions.csv (3 МБ приложение + 9.9 МБ LittleFS),
 *       он подхватывается IDE автоматически; в меню выберите схему разделов
 *       "16M Flash (3MB APP/9.9MB FATFS)" и Flash Size = 16MB, PSRAM = OPI PSRAM.
 *    2) Создайте рядом с DendyGO.ino папку data и внутри неё папку roms,
 *       положите туда файлы *.nes.
 *    3) Загрузите папку data в LittleFS:
 *         • Arduino IDE: Инструменты -> "ESP32 LittleFS Data Upload"
 *           (плагин arduino-esp32littlefs-plugin);
 *         • вручную, если плагина нет (адрес раздела spiffs из partitions.csv):
 *             mklittlefs -c data -p 256 -b 4096 -s 10354688 littlefs.bin
 *             esptool --chip esp32s3 --port COMx write-flash 0x610000 littlefs.bin
 *         • PlatformIO: "PlatformIO -> Upload Filesystem Image"
 *           (в platformio.ini: board_build.filesystem = littlefs).
 *       ВАЖНО: mklittlefs (его же использует плагин IDE) НЕ умеет упаковывать
 *       имена файлов длиннее 32 символов: на таком файле он падает с
 *       "unable to open ... / error adding file!" и в LittleFS не попадает
 *       НИЧЕГО. Держите имена ромов короче 32 символов вместе с ".nes".
 *    4) Перезагрузите плату — появится меню со списком ромов.
 *       Подробный лог (инициализация SX1509, каждое нажатие кнопки, листинг
 *       LittleFS и раздел под файловую систему) включается в DendyConfig.h:
 *       DENDY_BTN_TRACE и DENDY_ROM_TRACE.
 *
 *  ОГРАНИЧЕНИЯ
 *    • Поддержаны мапперы 0 (NROM), 1 (MMC1), 2 (UxROM), 3 (CNROM),
 *      4 (MMC3), 7 (AxROM), 11 (Color Dreams), 66 (GxROM).
 *    • Кадры кадрируются по 16.6 мс; если CPU не успевает, звук/картинка
 *      замедляются (это нормально для программного эмулятора).
 *    • Кириллица в названиях ромов заменяется на «?» (шрифт только ASCII).
 * =============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LittleFS.h>
#include <SparkFunSX1509.h>
#include "esp_partition.h"          // инфо о разделе LittleFS (адрес/размер)
#include "esp_heap_caps.h"          // свободная внутренняя SRAM/PSRAM (см. logMemory)

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "DendyConfig.h"
#include "NesCore.h"
#include "NesPpu.h"                 // палитра/перекодировка цвета для пробы видеотракта

// Штатный стек задачи loop() — 8 КБ, эмулятору с ним тесно. Макрос ядра ESP32
// подменяет getArduinoLoopTaskStackSize(), которым ядро создаёт loopTask.
SET_LOOP_TASK_STACK_SIZE(DENDY_LOOP_STACK);

#ifndef SPI_DMA_CH_AUTO
  #define SPI_DMA_CH_AUTO 3           // на случай очень старых ядер ESP32
#endif

// Отладочный вывод (включается в DendyConfig.h -> DENDY_DEBUG_SERIAL)
#if DENDY_DEBUG_SERIAL
  #define DENDY_LOG(...)  Serial.printf(__VA_ARGS__)
#else
  #define DENDY_LOG(...)  do {} while (0)
#endif

// =============================================================================
//  1. ДИСПЛЕЙ (LovyanGFX): SPI2 + ST7789 + подсветка через PWM
// =============================================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_SPI      _bus;
  lgfx::Panel_ST7789 _panel;
  lgfx::Light_PWM    _light;

public:
  LGFX() {
    {   // --- Шина SPI ---
      auto cfg        = _bus.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = DISPLAY_FREQ_WRITE;
      cfg.freq_read   = DISPLAY_FREQ_READ;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = PIN_TFT_SCLK;
      cfg.pin_mosi    = PIN_TFT_MOSI;
      cfg.pin_miso    = -1;
      cfg.pin_dc      = PIN_TFT_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {   // --- Панель ST7789 ---
      auto cfg          = _panel.config();
      cfg.pin_cs        = PIN_TFT_CS;
      cfg.pin_rst       = PIN_TFT_RST;
      cfg.pin_busy      = -1;
      cfg.panel_width   = DISPLAY_PANEL_W;
      cfg.panel_height  = DISPLAY_PANEL_H;
      cfg.memory_width  = DISPLAY_PANEL_W;
      cfg.memory_height = DISPLAY_PANEL_H;
      cfg.offset_x      = DISPLAY_OFFSET_X;
      cfg.offset_y      = DISPLAY_OFFSET_Y;
      cfg.readable      = false;
      cfg.invert        = DISPLAY_INVERT;
      cfg.rgb_order     = DISPLAY_BGR;
      cfg.dlen_16bit    = false;
      cfg.bus_shared    = false;
      _panel.config(cfg);
    }
    {   // --- Подсветка (PWM) ---
      auto cfg        = _light.config();
      cfg.pin_bl      = PIN_TFT_BL;
      cfg.invert      = false;
      cfg.freq        = 12000;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

static LGFX    display;
static SX1509  io;                        // расширитель кнопок
static TwoWire I2C2(1);                   // вторая шина I2C (контроллер №1)

// =============================================================================
//  2. ГЛОБАЛЬНОЕ СОСТОЯНИЕ
// =============================================================================
// --- Аудио ---
static int16_t  g_audioStereo[AUDIO_DMA_FRAMES * 2];  // L,R,L,R,... для I2S
static int16_t  g_audioMono[AUDIO_DMA_FRAMES];        // сэмплы, забранные из APU
static volatile bool g_i2sReady = false;              // драйвер I2S установлен
static TaskHandle_t  g_audioTask = nullptr;
// --- «Выдавить застрявший звук» (переход игра -> меню) -----------------------
// В буферах DMA и самом I2S остаётся до AUDIO_DMA_BUF_COUNT блоков уже отданного
// звука (~17 мс). Если его не вытолкнуть, он играет ЕЩЁ РАЗ уже поверх меню — на
// слух это и есть «зациклился звук» после выхода из игры. Скетч ставит ЗАПРОС
// (g_audioFlushReq в блоках тишины), задача I2S его исполняет и отмечает
// выполнение (g_audioFlushDone), скетч ждёт максимум ~100 мс.
static volatile uint32_t g_audioFlushReq  = 0;
static volatile uint32_t g_audioFlushDone = 0;
// Счётчики задачи вывода звука (диагностика: пишутся в лог раз в секунду)
static volatile uint32_t g_i2sBlocks   = 0;   // сколько блоков отправлено в I2S
static volatile uint32_t g_i2sSilent   = 0;   // из них блоков тишины (кольцо APU пусто)
static volatile uint32_t g_i2sTimeouts = 0;   // сколько записей не прошло (таймаут/ошибка)
// Минимальный и максимальный РАЗМЕР блока (сэмплов), ушедшего в I2S: считается от
// старта и только по блокам с реальными сэмплами (блоки тишины AUDIO_FEED_IDLE не
// учитываются). После «подушки» звука (AUDIO_PRIME_SAMPLES) минимум в игре держится
// около 220..256 — это и есть проверка, что нулей в середине музыки больше нет.
// min == AUDIO_FEED_IDLE (64) означает, что задача дошла до ПУСТОГО кольца.
static volatile uint32_t g_i2sBlkMin   = 0;
static volatile uint32_t g_i2sBlkMax   = 0;
// Сэмплы, реально ушедшие в I2S (в кольцо DMA): всего, ненулевых и пиковая
// амплитуда. Это единственный надёжный ответ на вопрос «звук выходит из ESP32?»:
// если nenulevyh растёт, а pik ~9000 (из 32767) — программный тракт исправен,
// и тишину надо искать в железе: SD (shutdown) у MAX98357, GAIN, питание,
// перепутанные BCLK/LRCK/DIN и сам динамик.
static volatile uint32_t g_i2sSamples  = 0;
static volatile uint32_t g_i2sNonZero  = 0;
static volatile int32_t  g_i2sPeak     = 0;
// Счётчики звука МЕНЮ (джингл): сколько сэмплов реально отдано в кольцо APU и
// какова их пиковая амплитуда. В строке [SND] это и есть ответ на вопрос
// «звук дошёл до I2S?»: otdano > 0 и pik около 9000 (из 32767) — в тракт уходят
// ненулевые сэмплы; otdano = 0 — меню не смогло положить звук в кольцо.
static volatile uint32_t g_sndPushed = 0;
static volatile int32_t  g_sndPeak   = 0;

// --- Измеритель «песка» (диагностика) ---------------------------------------
// Считается ЗАДАЧЕЙ I2S по тем сэмплам, которые реально уходят в кодек, окнами
// между строками [SND]. Нужен, чтобы отличить «песок» от музыки ЧИСЛОМ, а не на
// слух (и видеть результат правок APU без прослушивания):
//   hf   — доля энергии ВЫШЕ ~2 кГц в общей энергии, проценты («песок-метр»).
//          Чистая мелодия на пульсах/треугольнике даёт единицы процентов; шум,
//          DMC-барабаны и алиасинг (когда APU сэмплируется не на 44100 Гц, а
//          пачками) — десятки процентов.
//   rms  — средний уровень, проценты от полной шкалы (видно, что звук не тишина).
//   dyra — самая длинная серия РОВНО нулевых сэмплов за окно (1 мс = 44 сэмпла).
//          Ненулевая дыра в музыке = вставленная тишина (кольцо APU пусто).
//   clip — сколько сэмплов упёрлось в предел (|s| >= 32000): перегруз/клиппинг.
// Суммы копятся в единицах (s >> 8)^2: 65536 на сэмпл, за 2 с — 1,4 млрд, в
// uint32_t влезает. big — читает игровой цикл, поэтому поля volatile.
static volatile uint32_t g_audSq     = 0;    // сумма квадратов (s>>8)^2
static volatile uint32_t g_audHfSq   = 0;    // сумма квадратов ВЧ-части
static volatile uint32_t g_audCount  = 0;    // сколько сэмплов учтено
static volatile uint32_t g_audZeroRun = 0;   // текущая серия нулей
static volatile uint32_t g_audZeroMax = 0;   // самая длинная серия нулей за окно
static volatile uint32_t g_audClip    = 0;   // сэмплов у предела
static int32_t g_audLp1 = 0, g_audLp2 = 0;   // состояние двух ФНЧ (ФВЧ ~2 кГц)
// Последние посчитанные значения (их печатает строка [GAME]: «песок-метр» снимает
// ОКНО целиком в logSoundDiag, и второй раз за ту же секунду окно было бы пустым).
static int      g_audLastHf   = 0, g_audLastRms = 0;
static uint32_t g_audLastHole = 0, g_audLastClip = 0;

// --- Кнопки ---
static volatile bool g_sxIrq  = false;    // сработало прерывание SX1509
static volatile bool g_buttonsReady = false;  // SX1509 найден на шине I2C
static bool     g_btnIntSeen  = false;    // прерывание SX1509 хоть раз сработало
static const char* g_btnReason = "?";     // кто инициировал чтение (для лога)
static uint8_t  g_pad         = 0;        // удерживаемые кнопки (маска PAD_*)
static bool     g_selHeld     = false;    // Select удерживается (режим громкости)
static bool     g_turboA      = false;
static bool     g_turboB      = false;
static bool     g_turboPhase  = false;    // фаза "моргания" турбо
static uint32_t g_turboTimer  = 0;
static bool     g_menuHeld    = false;    // MENU удерживается
static uint32_t g_menuPressMs = 0;
static bool     g_menuHoldFired = false;  // удержание уже отработало (жди отпускания)
static bool     g_menuRequest = false;    // выход в меню (удержание MENU)
static bool     g_resetRequest = false;   // Reset игры (короткое нажатие MENU)
static uint32_t g_frameCounter = 0;
// --- РЕГИОН (PAL/NTSC): настройка меню + кнопка ------------------------------
// g_regionMode — ЧТО ВЫБРАНО в меню (AUTO/NTSC/PAL), g_regionPal — что получилось
// для текущего рома (в режиме AUTO берётся из заголовка образа). Кнопка на выводе 4
// SX1509 (SX_PIN_REGION) переключает режим по кругу прямо в меню, настройка
// сохраняется в REGION_FILE — пересборка прошивки для PAL больше не нужна.
static bool     g_regionKey    = false;   // кнопка «РЕГИОН» удерживается
// Кнопка «ЯРКОСТЬ» (SX_PIN_BRIGHT = 8): g_brKey — удерживается сейчас, g_brKeyPrev —
// фронт (одно нажатие = одна ступень). Работает и в игре, и в меню; состояние читает
// readButtons(), в обработчике обращений к I2C нет.
static bool     g_brKey        = false;
static bool     g_brKeyPrev    = false;
static uint8_t  g_regionMode   = DENDY_RMODE_AUTO;
static bool     g_regionPal    = (DENDY_REGION_PAL != 0);
// Период кадра в микросекундах ДРОБЬЮ (num/den) — выводится из региона в
// applyRegionMode(). Скетч считает по ним границы кадров (см. nextFramePeriodUs):
// 60 Гц = 50 000/3 (16 666,67 мкс), 50 Гц = 20 000/1.
static uint32_t g_frameUsNum   = DENDY_FRAME_US_NUM;
static uint32_t g_frameUsDen   = DENDY_FRAME_US_DEN;

// --- Громкость / индикатор ---
static uint8_t  g_volume      = AUDIO_VOLUME_DEFAULT;  // 0..100
// Яркость экрана в ПРОЦЕНТАХ (1..100): именно это значение показывают OSD («YARK 75 %»)
// и шапка меню. В ШИМ проценты переводит brPctToPwm(); верх шкалы — DISPLAY_BRIGHTNESS
// (блок 2 «ДИСПЛЕЙ»), то есть «100 %» = DISPLAY_BRIGHTNESS.
static uint8_t  g_brightPct   = DISPLAY_BR_DEFAULT_PCT;
// OSD: пишет игровой цикл, читает задача вывода кадра (см. osdDrawIfActive),
// поэтому оба поля volatile.
static volatile uint32_t g_osdUntilMs  = 0;    // до какого millis() показывать OSD
static volatile bool     g_osdColorMode = false;  // OSD показывает режим цвета, а не громкость
static volatile bool     g_osdBrightMode = false; // OSD показывает яркость экрана
static uint32_t g_lastPushUs  = 0;        // время последней передачи кадра (мкс)
static uint32_t g_pushSumUs   = 0;        // сумма времени передач за окно [GAME]
static uint32_t g_pushCount   = 0;        // сколько кадров выведено за окно [GAME]
static uint32_t g_emuSumUs    = 0;        // сумма времени эмуляции кадров за окно [GAME]
static uint32_t g_emuCount    = 0;        // сколько кадров эмулировано за окно [GAME]
// ХУДШИЙ кадр окна (мкс на кадр). Вместе со средним (`emu ms/kadr`) показывает,
// ПОЧЕМУ fps emu меньше 60: если среднее и худшее ровные и больше 16 667 мкс —
// эмулятор просто не успевает (нужна скорость, см. [GAME] profil); если среднее
// ~16 000, а худшее 25 000..40 000 — кадру мешают ВСПЛЕСКИ (опрос кнопок по I2C,
// печать в Serial, задачи второго ядра, прерывания), и тут помогает
// DENDY_MAX_FRAME_SKIP = 2 (долг больше не «прощается»).
static uint32_t g_emuMaxUs    = 0;
// --- Бюджет времени игрового цикла (строка [GAME] vremya) --------------------
// Зачем именно эти поля: [GAME] profil показывает время ВНУТРИ runFrame (cpu/ppu/
// apu), а «игра идёт медленнее приставки» складывается ещё и из всего остального
// (опрос кнопок, ожидание кадра, печать в Serial, задачи другого ядра). Если
// `emu` ~8 мс, а `fps emu` ~38, значит теряются НЕ в эмуляторе — и вот здесь
// видно, где именно (см. подсказку после строки vremya).
static uint32_t g_tEmuUs    = 0;          // время в runFrame («пачка» кадров)
static uint32_t g_tBtnUs    = 0;          // время в кнопках: опрос I2C + логика
static uint32_t g_tWaitUs   = 0;          // ожидание границы следующего кадра
static uint32_t g_tLogUs    = 0;          // печать в Serial из игрового цикла
static uint32_t g_tPassUs   = 0;          // ВСЁ время игрового цикла за окно
static uint32_t g_tPasses   = 0;          // проходов игрового цикла за окно
static uint32_t g_btnReadCount = 0;       // чтений шины SX1509 (видно «шторм» I2C)
static bool     g_volUpPrev   = false;    // фронты Select+Up/Down
static bool     g_volDownPrev = false;
// Отдельные кнопки громкости (SX_PIN_VOL_UP = 6, SX_PIN_VOL_DOWN = 5). Работают и
// в игре, и в меню, с авто-повтором при удержании (AUDIO_VOLUME_REPEAT_MS).
static bool     g_volKeyUp     = false;   // сейчас удержана «громкость +»
static bool     g_volKeyDown   = false;   // сейчас удержана «громкость −»
static bool     g_volKeyUpPrev = false;   // фронты отдельных кнопок
static bool     g_volKeyDownPrev = false;
static bool     g_colLeftPrev = false;    // фронты Select+Left/Right (порядок цветов)
static bool     g_colRightPrev = false;

// --- Перекрытие вывода кадра с эмуляцией (DENDY_FRAME_DOUBLE_BUFFER) ----------
// Кадр 256x240 = 122 880 байт, на 40 МГц SPI передаётся ~26 мс. Раньше это время
// эмулятор стоял (pushImage синхронный) — терялась половина времени CPU, и игра
// шла ровно вдвое медленнее приставки («1 секунда игры = 2 секунды реальности»).
// Теперь передачу ведёт отдельная задача на втором ядре, а PPU рисует следующий
// кадр в ДРУГОЙ буфер. Буферов три, поэтому главная задача НИКОГДА не ждёт:
//   [рисуется] [ждёт передачи] [передаётся] — как минимум один всегда свободен.
// Если задача не успевает, лишний кадр просто пропускается (как при пропуске
// кадров), а не тормозит эмуляцию.
#if DENDY_FRAME_DOUBLE_BUFFER
static const int DENDY_FB_COUNT = 3;
static SemaphoreHandle_t g_lcdMutex = nullptr;    // доступ к SPI-дисплею (две задачи)
static TaskHandle_t      g_pushTask = nullptr;    // задача вывода кадра
static uint16_t*         g_fb[DENDY_FB_COUNT] = { nullptr, nullptr, nullptr };
static portMUX_TYPE      g_fbMux = portMUX_INITIALIZER_UNLOCKED;
static int               g_fbRender  = -1;        // в какой буфер рисует PPU
static int               g_fbPending = -1;        // отдан задаче, но ещё не взята
static int               g_fbReading = -1;        // задача сейчас передаёт его
#endif

// --- Список ромов ---
static char     g_romPath[ROM_MAX_FILES][72];
static char     g_romName[ROM_MAX_FILES][ROM_NAME_LEN];
static uint32_t g_romSize[ROM_MAX_FILES];
static int      g_romCount    = 0;
static int      g_romSelected = 0;
static int      g_romScroll   = 0;

// --- Раскладка меню (экран 320x240 в альбомной ориентации) ---
#define MENU_TITLE_Y    4
#define MENU_SUB_Y      28
#define MENU_LIST_Y     44
#define MENU_ROW_H      18
#define MENU_ROWS       9
#define MENU_FOOTER_Y   (MENU_LIST_Y + MENU_ROW_H * MENU_ROWS)   // 206
#define MENU_NAME_CHARS ((DISPLAY_LANDSCAPE_W - 56) / 12)        // ~22 символа при size=2
#define VOLUME_FILE     "/volume.bin"                            // 1 байт с громкостью
// Режим ТВ (регион) хранится в файле: 1 байт — DENDY_RMODE_AUTO/NTSC/PAL
// (см. блок 3.1 DendyConfig.h и applyRegionMode ниже).
#define REGION_FILE     "/region.bin"

// =============================================================================
//  3. ПРОТОТИПЫ
// =============================================================================
static void     setupDisplay();
static void     setupAudio();
static void     setupButtons();
static void     audioTaskEntry(void* arg);
static void     audioFlushDma();
static void     applyVolume(uint8_t volume);
static void     loadVolume();
static void     saveVolume();
// Регион ТВ (PAL/NTSC): настройка меню на кнопке SX_PIN_REGION (см. раздел 4.1)
static void     loadRegionMode();
static void     saveRegionMode();
static void     applyRegionMode(const char* where);
static void     regionModeText(char* out, size_t n);
static const char* regionModeName(uint8_t mode);
static int      primeSamplesNow();
static void     showMessage(const char* line1, const char* line2);
static void     readButtons();
static void     updateButtonLogic();
static void     pollButtons();
static void     handleVolumeKeys();
static void     handleBrightKey();
static uint8_t  turboPad();

// Звук меню: 8-битный «бипер» (см. раздел 6.1). Работает, когда эмулятор не
// запущен, и служит проверкой звукового тракта — при входе в меню играет джингл.
#if DENDY_MENU_SOUND
static void     serviceMenuSound();
static void     menuSoundReset();     // сброс очереди/фазы генератора (смена режима)
static void     menuSoundJingle();
static void     menuSoundBlip(bool up);
static void     menuSoundLaunch();
static void     menuSoundTest();      // длинный громкий «тест звука» (в меню)
// «Прогрев» тракта тишиной перед ПЕРВЫМ звуком: состояние и функции — в разделе 6
// (sndStartGuardArm/Pending/Wait), здесь — только логика джингла меню (раздел 6.1).
static bool     menuSoundJingleOnMenuEnter(); // джингл сразу или отложен (вход в меню)
static bool     menuSoundDeferredJingle();    // цикл меню: сыграть отложенный джингл
#else
#define serviceMenuSound()  do {} while (0)
#define menuSoundReset()    do {} while (0)
#define menuSoundJingle()   do {} while (0)
#define menuSoundBlip(up)   do {} while (0)
#define menuSoundLaunch()   do {} while (0)
#define menuSoundTest()     do {} while (0)
#define menuSoundJingleOnMenuEnter()  false
#define menuSoundDeferredJingle()     false
#endif

// Диагностика кнопок (подробный лог включается в DendyConfig.h)
#if DENDY_DEBUG_SERIAL && DENDY_BTN_TRACE
static void     logButtonState(uint8_t pad, bool menu, bool turboA, bool turboB,
                               uint16_t data);
static void     logSxRegisters(const char* tag);
static void     i2cScan();
#else
#define logButtonState(pad, menu, tA, tB, raw)  do {} while (0)
#define logSxRegisters(tag)                do {} while (0)
#define i2cScan()                          do {} while (0)
#endif

static void     scanRoms();
static bool     loadRomFromFile(const char* path, uint32_t size);

// Диагностика файловой системы
#if DENDY_DEBUG_SERIAL && DENDY_ROM_TRACE
static void     listFs(const char* dirPath);
static void     logFsPartition();
#else
#define listFs(path)        do {} while (0)
#define logFsPartition()    do {} while (0)
#endif
// Диагностика видеотракта (включается DENDY_VIDEO_TRACE в DendyConfig.h)
#if DENDY_DEBUG_SERIAL && DENDY_VIDEO_TRACE
static void     drawVideoSelfTest();
static void     logVideoDiag(uint32_t emuUs, uint32_t pushUs, uint32_t frames);
#else
#define drawVideoSelfTest()                   do {} while (0)
#define logVideoDiag(emuUs, pushUs, frames)   do {} while (0)
#endif

static void     drawMenu();
static void     drawMenuRow(int row, int index, bool selected);
static int      menuNavStep();
static int      menuPageStep();
static void     runMenu();
static void     runGame();
static void     logMemory(const char* tag);    // свободная SRAM/PSRAM (см. logSystemInfo)
static void     pushFrame();
#if DENDY_FRAME_DOUBLE_BUFFER
static void     pushFrameOverlap();
static void     pushTaskEntry(void* arg);
#endif
// OSD поверх игры (полоса громкости / режим цвета кадра). Рисуется НЕ из
// игрового цикла, а из задачи вывода кадра — см. osdDrawIfActive().
static void     osdDrawIfActive();
static void     sanitizeName(char* dst, const char* src, size_t maxLen);
static void     frameLimiter(uint32_t startUs);

// Порядок каналов цвета кадра: подбор «на лету» (SELECT + LEFT/RIGHT) и
// контрольная полоса «кадровый путь vs эталон LovyanGFX» (см. раздел 7.1).
static bool     handleColorOrderKeys();
static void     applyColorOrder(uint8_t mode, const char* where);
static void     drawColorOrderBar(int y);
static void     clearFrameBorders();

// =============================================================================
//  4. УТИЛИТЫ
// =============================================================================
// Приведение строки к ASCII (шрифт меню не умеет кириллицу).
static void sanitizeName(char* dst, const char* src, size_t maxLen) {
  size_t n = 0;
  for (; src[n] && n + 1 < maxLen; ++n) {
    const uint8_t c = (uint8_t)src[n];
    dst[n] = (c >= 0x20 && c <= 0x7E) ? (char)c : '?';
  }
  dst[n] = '\0';
  // убираем хвостовые пробелы
  while (n > 0 && dst[n - 1] == ' ') dst[--n] = '\0';
}

// ---------------------------------------------------------------------------
//  Часы кадров NES: РОВНО 60,000 кадра/с (в PAL — ровно 50,000).
// ---------------------------------------------------------------------------
// Период кадра задан ДРОБЬЮ микросекунд (DENDY_FRAME_US_NUM/DEN в DendyConfig.h):
// при 60 Гц это 50 000/3 = 16 666,67 мкс, то есть границы кадров идут
// 16 667, 16 667, 16 666 — за ТРИ кадра ровно 50 000 мкс. Одна целая константа
// (16 666 или 16 667) «уползает» на 0,3..0,7 мкс за кадр: на глаз это ничто, но
// в логе [GAME] было бы 59,99..60,01 вместо 60,00, а APU/I2S копили бы
// расхождение. Поэтому остаток от деления копим здесь.
static uint32_t g_frameUsAcc = 0;      // остаток дробной части периода (0..DEN-1)

// Период СЛЕДУЮЩЕГО кадра в мкс (в среднем ровно g_frameUsNum/g_frameUsDen).
// Числа берутся из таблицы региона (см. applyRegionMode): 60 Гц = 50 000/3,
// 50 Гц = 20 000/1, поэтому при смене режима ТВ в меню меняется только эта пара.
static inline uint32_t nextFramePeriodUs() {
  g_frameUsAcc += g_frameUsNum % g_frameUsDen;
  uint32_t us = g_frameUsNum / g_frameUsDen;
  if (g_frameUsAcc >= g_frameUsDen) { g_frameUsAcc -= g_frameUsDen; ++us; }
  return us;
}

// Дождаться границы следующего кадра. boundaryUs — момент границы ТОЛЬКО ЧТО
// прошедшего кадра (его уже эмулировали), periodUs — его длительность. Считаем
// по стен-часам micros(), поэтому время, потраченное на эмуляцию, само входит в
// паузу: кадры идут по 60 в секунду независимо от того, сколько заняла работа.
// delay() спит целыми тиками RTOS (1 мс) и всегда округляет вверх, поэтому им
// отдаём только основную часть паузы, а остаток (< 1 мс) добираем
// delayMicroseconds() — иначе каждый кадр «убегал» бы на целый тик (до 6 %).
static void frameLimiter(uint32_t boundaryUs, uint32_t periodUs) {
  const int32_t p    = (int32_t)periodUs;
  int32_t       rest = p - (int32_t)(uint32_t)(micros() - boundaryUs);
  if (rest <= 0) return;                          // кадр уже «наступил»
  if (rest > 1000) delay((uint32_t)(rest - 500) / 1000);
  rest = p - (int32_t)(uint32_t)(micros() - boundaryUs);
  if (rest > 0 && rest <= 1500) delayMicroseconds((uint32_t)rest);
}

// ---------------------------------------------------------------------------
//  Что за сборка: с каким -O собран скетч и с каким — горячий код ядра.
// ---------------------------------------------------------------------------
// Печатается и в строке [SYS] при старте, и в строке [GAME] sbor при медленной
// эмуляции. Зачем в игре: «игра идёт как в замедленной съёмке» в 9 случаях из
// 10 — это Tools -> Debug Level (= -Og), а до правки про это надо было помнить
// по строке, напечатанной один раз при старте.
static const char* buildOptName() {
#if !defined(__OPTIMIZE__)
  return "O0 (Tools -> Debug Level = None!)";
#elif defined(__OPTIMIZE_SIZE__)
  return "Os";
#elif defined(__NO_INLINE__)
  return "Og (Tools -> Debug Level = Debug: sdelayte None!)";
#else
  return "O2/O3";
#endif
}
static const char* coreOptName() {
#if DENDY_CORE_O3
  return "O3 (DendyFast.h)";
#else
  return "kak skech";
#endif
}

// Громкость 0..100 -> 0..255 для ядра + сохранение в LittleFS.
static void applyVolume(uint8_t volume) {
  if (volume > 100) volume = 100;
  g_volume = volume;
  NesCore::setVolume((uint8_t)((uint16_t)volume * 255 / 100));
  saveVolume();
}

static void loadVolume() {
  File f = LittleFS.open(VOLUME_FILE, "r");
  if (!f) return;
  const int v = f.read();
  f.close();
  if (v >= 0 && v <= 100) applyVolume((uint8_t)v);
}

static void saveVolume() {
  File f = LittleFS.open(VOLUME_FILE, "w");
  if (!f) return;
  f.write(g_volume);
  f.close();
}

// =============================================================================
//  4.1. РЕГИОН ТВ (PAL 50 Гц / NTSC 60 Гц) — настройка «на кнопку 4»
// =============================================================================
// ЗАЧЕМ ЭТО ВООБЩЕ. У приставки регион задан «железом»: NTSC — 60 кадров/с
// (262 строки в кадре, тактовая 2A03 1,78684 МГц), PAL — 50 кадров/с (312 строк,
// 1,662375 МГц). Игра написана под СВОЙ регион, и в чужом идёт с неправильной
// скоростью: PAL-ром в NTSC-режиме бежит примерно на 20 % быстрее, а музыка
// звучит выше на ~3 полутона.
// ЧЕМ РЕГИОН ЗАДАН В ОБРАЗЕ: в самом коде игры его нет. Единственный признак —
// ЗАГОЛОВОК файла .nes: байт 9 бит 0 («PAL-версия»), для NES 2.0 — байт 12
// биты 0..1, для VS-образов — байт 10. Заполняет это тот, кто делал дамп, и
// вранья там хватает, поэтому выбор сделан ТРЁХПОЗИЦИОННЫМ, а не «умным»:
//   AUTO      — верить заголовку (если признака нет — NTSC: так помечена почти
//               вся библиотека, а PAL-версии чаще всего всё же помечены);
//   NTSC 60Гц — принудительно;
//   PAL 50Гц  — принудительно (нужно для «русифицированных» PAL-сборок, у
//               которых в заголовке остался NTSC).
// КНОПКА: вывод 4 SX1509 (SX_PIN_REGION) в меню переключает режим по кругу
// AUTO -> NTSC -> PAL -> AUTO. Настройка сохраняется в LittleFS (REGION_FILE) и
// живёт до следующего нажатия. В ИГРЕ КНОПКА НИЧЕГО НЕ ДЕЛАЕТ: от региона
// зависят частота кадров, число строк, тактовая 2A03 и соотношение CPU:PPU —
// то есть весь тайминг эмуляции, поэтому он применяется ПРИ ЗАПУСКЕ РОМА
// (см. applyRegionMode в runMenu/runGame/setup).
static const char* regionModeName(uint8_t mode) {
  switch (mode) {
    case DENDY_RMODE_NTSC: return "NTSC";
    case DENDY_RMODE_PAL:  return "PAL";
    default:               return "AUTO";
  }
}

// Регион из заголовка ФАЙЛА рома. Нужен, чтобы в режиме AUTO меню показывало не
// «последний применённый», а то, что реально даст ЗАГОЛОВОК ВЫБРАННОГО рома.
// Читаем только 16 байт заголовка и только при смене выбора в меню (не в игре).
// Правила те же, что в NesMapper::load() — см. блок «РЕГИОН» в NesMapper.h:
// байт 9 бит 0 = «PAL-версия», для NES 2.0 (байт 7 биты 2..3 = 10) — байт 12
// биты 0..1, для VS-образов — байт 10 биты 0..1. Возврат: 0 = NTSC, 1 = PAL,
// 2 = «в заголовке не сказано».
static uint8_t romFileRegionHint(const char* path) {
  if (!path || !path[0]) return 2;
  File f = LittleFS.open(path, "r");
  if (!f) return 2;
  uint8_t h[16];
  const int n = f.read(h, sizeof(h));
  f.close();
  if (n < 16 || memcmp(h, "NES\x1A", 4) != 0) return 2;
  const uint8_t timing = (uint8_t)((h[7] & 0x0C) >> 2);
  if (timing == 2) return (uint8_t)((h[12] & 0x03) == 0 ? 0 : 1);
  if ((h[10] & 0x03) == 0x02) return 1;
  return (uint8_t)((h[9] & 0x01) ? 1 : 0);
}

// Короткий текст для меню: «auto(NTSC)», «NTSC» или «PAL».
static void regionModeText(char* out, size_t n) {
  if (g_regionMode == DENDY_RMODE_AUTO) {
    // AUTO: показываем то, что даст заголовок ВЫБРАННОГО рома; если ромов нет
    // (или заголовок пуст) — регион, который сейчас в ядре.
    uint8_t hint = 2;
    if (g_romCount > 0 && g_romSelected >= 0 && g_romSelected < g_romCount) {
      hint = romFileRegionHint(g_romPath[g_romSelected]);
    }
    const bool pal = (hint == 2) ? g_regionPal : (hint == 1);
    snprintf(out, n, "auto(%s)", pal ? "PAL" : "NTSC");
  } else {
    snprintf(out, n, "%s", regionModeName(g_regionMode));
  }
}

// Регион для загруженного рома: принудительный режим — как выбрано в меню,
// AUTO — по заголовку образа (regionHint: 1 = PAL, 0 = NTSC, 2 = не сказано).
static bool regionPalForCurrentRom() {
  if (g_regionMode == DENDY_RMODE_PAL)  return true;
  if (g_regionMode == DENDY_RMODE_NTSC) return false;
  return NesCore::romRegionHint() == 1;
}

// Применить регион к ядру и перевести период кадра. Звать ДО NesCore::reset()
// (см. runGame): тогда и PPU, и APU, и счётчик кадров стартуют уже в нужном
// режиме, и первый же кадр идёт с правильной длительностью.
static void applyRegionMode(const char* where) {
  if (g_regionMode == DENDY_RMODE_PAL)       g_regionPal = true;
  else if (g_regionMode == DENDY_RMODE_NTSC) g_regionPal = false;
  else                                       g_regionPal = regionPalForCurrentRom();

  NesCore::setRegion(g_regionPal);              // PPU (строки), APU (тактовая), CPU:PPU
  if (g_regionPal) {
    g_frameUsNum = DENDY_PAL_FRAME_US_NUM;      // 20 000/1 = 50,000 кадра/с
    g_frameUsDen = DENDY_PAL_FRAME_US_DEN;
  } else {
    g_frameUsNum = DENDY_NTSC_FRAME_US_NUM;     // 50 000/3 = 60,000 кадра/с
    g_frameUsDen = DENDY_NTSC_FRAME_US_DEN;
  }
  g_frameUsAcc = 0;                             // остаток дробной части периода

  DENDY_LOG("[TV] %s: rezhim %s -> %s | kadr %u/%u us (%.3f kadr/s), strok %u, "
            "CPU 2A03 %.4f MHz, semplov APU na kadr %.2f | podskazka zagolovka: %s\n",
            where, regionModeName(g_regionMode), NesCore::regionName(),
            (unsigned)g_frameUsNum, (unsigned)g_frameUsDen,
            (double)(1000000.0f * (float)g_frameUsDen / (float)g_frameUsNum),
            (unsigned)(g_regionPal ? DENDY_PAL_LINES : DENDY_NTSC_LINES),
            (double)(g_regionPal ? DENDY_PAL_CPU_HZ : DENDY_NTSC_CPU_HZ) / 1000000.0,
            (double)((float)AUDIO_SAMPLE_RATE * (float)g_frameUsDen / (float)g_frameUsNum),
            NesCore::romRegionHint() == 2 ? "ne ukazano"
              : (NesCore::romRegionHint() == 1 ? "PAL" : "NTSC"));
}

static void saveRegionMode() {
  File f = LittleFS.open(REGION_FILE, "w");
  if (!f) return;
  f.write(g_regionMode);
  f.close();
}

static void loadRegionMode() {
  File f = LittleFS.open(REGION_FILE, "r");
  if (!f) return;
  const int v = f.read();
  f.close();
  if (v >= DENDY_RMODE_AUTO && v <= DENDY_RMODE_PAL) g_regionMode = (uint8_t)v;
}

// «Подушка» звука для ТЕКУЩЕГО региона: AUDIO_PRIME_FRAMES кадров в сэмплах.
// AUDIO_PRIME_SAMPLES из конфига посчитан для региона по умолчанию (735 при 60 Гц),
// а в PAL кадр длиннее (50 Гц -> 882), поэтому размер считаем на месте: кадр задан
// дробью микросекунд (см. applyRegionMode), сэмплов = rate x период.
static int primeSamplesNow() {
  int n = (int)(((uint64_t)AUDIO_PRIME_FRAMES * AUDIO_SAMPLE_RATE * g_frameUsNum)
                / ((uint64_t)g_frameUsDen * 1000000ull));
  // Та же самопроверка, что static_assert в DendyConfig.h, но на ходу: «подушка»
  // вместе с блоком DMA обязана помещаться в кольцо, иначе предзаполнение съест
  // место для свежих сэмплов и станет хуже, чем без него.
  const int maxPrime = (int)NesApu::RING_SIZE - AUDIO_DMA_FRAMES - 1;
  if (n > maxPrime) n = maxPrime;
  if (n < 0) n = 0;
  return n;
}

// =============================================================================
//  5. ИНИЦИАЛИЗАЦИЯ ДИСПЛЕЯ
// =============================================================================
// --- ЯРКОСТЬ ЭКРАНА КНОПКОЙ (SX_PIN_BRIGHT, вывод 8 SX1509) -------------------
// Ступени заданы в ПРОЦЕНТАХ (DendyConfig.h, блок 2 «ДИСПЛЕЙ»): так значение видно
// в OSD («YARK 75 %»), в шапке меню и в логе — не пересчитывая в 0..255 на глаз.
// В ШИМ проценты переводит brPctToPwm(): 100 % = DISPLAY_BRIGHTNESS (верх шкалы).
static constexpr uint8_t kBrStepsPct[] = DISPLAY_BR_STEPS_PCT;
#define BR_STEP_COUNT  ((int)(sizeof(kBrStepsPct) / sizeof(kBrStepsPct[0])))

// Самопроверка списка ступеней (тем же способом, что sxPinsUnique для кнопок):
// 2..16 значений, все в 1..100 и строго по возрастанию. Ноль здесь запрещён
// намеренно: ШИМ погасил бы подсветку совсем, и ступень пришлось бы искать вслепую.
static constexpr bool brStepsOk(const uint8_t* v, int n) {
  if (n < 2 || n > 16) return false;
  for (int i = 0; i < n; ++i) {
    if (v[i] < 1 || v[i] > 100) return false;
    if (i > 0 && v[i] <= v[i - 1]) return false;
  }
  return true;
}
static_assert(brStepsOk(kBrStepsPct, BR_STEP_COUNT),
              "DISPLAY_BR_STEPS_PCT в DendyConfig.h: ступени яркости должны идти "
              "по возрастанию, лежать в 1..100 % и их должно быть от 2 до 16");

// Проценты -> значение ШИМ. Верх шкалы берём из DISPLAY_BRIGHTNESS, поэтому «100 %»
// всегда означает «столько, сколько разрешено этой платой».
static inline uint8_t brPctToPwm(uint8_t pct) {
  return (uint8_t)((uint16_t)DISPLAY_BRIGHTNESS * pct / 100u);
}

// Что СЕЙЧАС выдано в ШИМ (0..255) и куда идём. Обычно они равны: смена яркости идёт
// короткими шагами (backlightGlideStep), чтобы не давать ступеньку тока по общему с
// кодеком питанию — та же причина, по которой подсветка включается плавно.
static uint8_t  g_blPwm       = DISPLAY_BRIGHTNESS;
static uint8_t  g_blPwmTarget = DISPLAY_BRIGHTNESS;
static uint32_t g_blGlideMs   = 0;

// Один шаг «плавной» яркости: зовётся из обоих циклов (меню и игра) — сколько бы раз
// за кадр ни позвали, шаги идут не чаще, чем DISPLAY_BR_GLIDE_STEP_MS. Шаг — восьмая
// часть остатка, минимум 1: крупный скачок «разгоняется», хвост доводится мягко, весь
// переход — ~8 шагов (по умолчанию ~50 мс). setBrightness() трогает только ШИМ
// (LEDC): SPI-мьютекс дисплея здесь не нужен и кадр не ждёт.
static void backlightGlideStep() {
  if (g_blPwm == g_blPwmTarget) return;                 // уже на месте — выходим сразу
  const uint32_t now = millis();
  if (g_blGlideMs != 0 && (int32_t)(now - g_blGlideMs) < 0) return;
  const int delta = (int)g_blPwmTarget - (int)g_blPwm;
  int step = delta / 8;
  if (step == 0) step = (delta > 0) ? 1 : -1;
  g_blPwm = (uint8_t)((int)g_blPwm + step);
  display.setBrightness(g_blPwm);
  g_blGlideMs = now + DISPLAY_BR_GLIDE_STEP_MS;
}

// Одно нажатие кнопки яркости = следующая ступень по кругу. Если текущее значение в
// списке НЕТ (при включении это DISPLAY_BR_DEFAULT_PCT, 70 %), берём БЛИЖАЙШУЮ БОЛЬШУЮ
// ступень (70 -> 75), а с самой большой круг замыкается на первую (100 -> 5).
// В меню OSD не рисуется (туда не уходит кадр) — там новое значение показывает шапка,
// поэтому список перерисовываем, как это делает громкость (volumeApplyStep).
static void brightnessStep() {
  uint8_t next = kBrStepsPct[0];
  for (int i = 0; i < BR_STEP_COUNT; ++i) {
    if (kBrStepsPct[i] > g_brightPct) { next = kBrStepsPct[i]; break; }
  }
  if (next == g_brightPct) return;                      // крутить некуда (список из 1)
  g_brightPct   = next;
  g_blPwmTarget = brPctToPwm(next);
  DENDY_LOG("[VID] knopka 8 (yarkost): %u%% = %u/255, plavno %u ms (DISPLAY_BR_GLIDE_STEP_MS)\n",
            (unsigned)next, (unsigned)g_blPwmTarget, (unsigned)DISPLAY_BR_GLIDE_STEP_MS);
  // OSD — как у громкости: в игре его нарисует задача вывода кадра после кадра.
  g_osdColorMode  = false;
  g_osdBrightMode = true;
  g_osdUntilMs    = millis() + OSD_TIMEOUT_MS;
  if (!NesCore::isLoaded()) drawMenu();                 // в меню обновляем шапку целиком
}

// --- МЯГКОЕ ВКЛЮЧЕНИЕ ПОДСВЕТКИ (DISPLAY_BL_RAMP_MS) --------------------------
// Подсветка на GPIO21 идёт через lgfx::Light_PWM (см. LGFX выше), поэтому яркость
// можно поднимать плавно, а не ступенькой 0 -> рабочую яркость (DISPLAY_BR_DEFAULT_PCT).
// ЗАЧЕМ: подсветка — самый большой потребитель на плате, и её ступенька (вместе с
// первой полной отрисовкой экрана по SPI) даёт бросок тока по общему с кодеком
// питанию; по времени он попадал в первое включение звука и слышался щелчком (см.
// DendyConfig.h, «ПЕРВЫЙ ЗВУК ПОСЛЕ ВКЛЮЧЕНИЯ»). Здесь этот бросок растянут на
// DISPLAY_BL_RAMP_MS и уходит из окна первого звука: подъём идёт ДО setupAudio() и
// задолго до джингла.
// Функция БЛОКИРУЮЩАЯ: это старт, подождать можно — ровно столько она и добавляет к
// времени загрузки (шаг 15 мс). При DISPLAY_BL_RAMP_MS = 0 яркость ставится сразу.
static void backlightSoftStart() {
  const uint8_t target = brPctToPwm((uint8_t)DISPLAY_BR_DEFAULT_PCT);   // яркость включения
#if DISPLAY_BL_RAMP_MS > 0
  const uint16_t ms    = (uint16_t)DISPLAY_BL_RAMP_MS;
  const uint16_t step  = 15;                        // шаг нарастания, мс
  const uint16_t steps = (uint16_t)(ms / step + 1u);
  for (uint16_t i = 1; i <= steps; ++i) {
    display.setBrightness((uint8_t)((uint16_t)target * i / steps));
    delay(step);
  }
#endif
  // Конечное значение — точное, и оно же становится состоянием для кнопки яркости
  // (g_blPwm = g_blPwmTarget): дальше «плавный переход» считает уже от него.
  g_blPwm = g_blPwmTarget = target;
  display.setBrightness(target);
}

static void setupDisplay() {
  display.init();
  display.setRotation(DISPLAY_ROTATION);
#if DISPLAY_BL_RAMP_MS > 0
  display.setBrightness(0);      // начинаем с погашенной подсветки — см. ниже
#endif
  display.setTextWrap(false);
  // Первая ПОЛНАЯ отрисовка — ДО подъёма подсветки: всплеск тока по SPI уходит в
  // темноту, а не накладывается на подъём яркости (см. backlightSoftStart).
  display.fillScreen(UI_BG_COLOR);
  backlightSoftStart();
  DENDY_LOG("[VID] podsvetka: %u/255 (%u%%, DISPLAY_BR_DEFAULT_PCT), plavnyj podjem %u ms (DISPLAY_BL_RAMP_MS)\n",
            (unsigned)brPctToPwm((uint8_t)DISPLAY_BR_DEFAULT_PCT),
            (unsigned)DISPLAY_BR_DEFAULT_PCT, (unsigned)DISPLAY_BL_RAMP_MS);
  DENDY_LOG("[VID] display %dx%d (rot %d) | nastrojka: panel %dx%d, offset %d,%d\n",
            display.width(), display.height(), display.getRotation(),
            DISPLAY_PANEL_W, DISPLAY_PANEL_H, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y);
}

// Короткое сообщение на экране (заставка / ошибка).
static void showMessage(const char* line1, const char* line2) {
  display.fillScreen(UI_BG_COLOR);
  display.setTextDatum(lgfx::textdatum_t::middle_center);
  display.setTextSize(2);
  display.setTextColor(UI_TITLE_COLOR);
  display.drawString(line1, display.width() / 2, display.height() / 2 - 16);
  if (line2 && line2[0]) {
    display.setTextSize(1);
    display.setTextColor(UI_TEXT_COLOR);
    display.drawString(line2, display.width() / 2, display.height() / 2 + 16);
  }
  display.setTextDatum(lgfx::textdatum_t::top_left);
}

// =============================================================================
//  6. ЗВУК: I2S (MAX98357) + задача вывода
// =============================================================================
#define AUDIO_I2S_TIMEOUT_MS 200   // таймаут записи в DMA (мс)

// --- «ПРОГРЕВ» ТРАКТА ТИШИНОЙ ПЕРЕД ПЕРВЫМ ЗВУКОМ ПОСЛЕ ВКЛЮЧЕНИЯ -------------
// Разбор ТРЁХ причин щелчка и все настройки — в DendyConfig.h, блок
// «ПЕРВЫЙ ЗВУК ПОСЛЕ ВКЛЮЧЕНИЯ» (AUDIO_START_GUARD_MS — «прогрев» тракта тишиной,
// AUDIO_FIRST_SOUND_QUIET_MS — «тихое окно» после тяжёлых шагов загрузки,
// AUDIO_START_FADE_MS — «мягкое включение» самого сигнала).
// Здесь состояние и операции: задать прогрев, отметить тяжёлый шаг загрузки,
// спросить, можно ли уже играть первый звук, и дождаться этого в «медленном» месте
// (старт игры). Логика меню — в 6.1, «проявление» сигнала — в audioTaskEntry.
//
// ПОЧЕМУ ДВА ОКНА, А НЕ ОДНО. «Прогрев» отсчитывается от setupAudio() (момент,
// когда I2S начал тактировать кодек), а «тихое окно» — от КОНЦА последнего тяжёлого
// шага загрузки (первая полная отрисовка, чтение рома, кнопки). Монтирование
// LittleFS + скан ромов + первая отрисовка занимают на разных ромах разное время,
// поэтому одно окно «иногда» кончалось раньше всплеска тока — отсюда и щелчок
// «иногда». Второе окно привязано к настоящему концу всплесков.
//
// До какого момента играть первый звук НЕЛЬЗЯ (0 — окно не задано/уже пройдено).
static uint32_t g_sndGuardUntilMs = 0;        // «прогрев» тракта тишиной
static uint32_t g_sndQuietUntilMs = 0;        // «тихое окно» после тяжёлого шага
static bool     g_sndFirstSoundDone = false;  // первый звук сыгран — окна не нужны

// Задать «прогрев». Звать ОДИН раз при включении — в setup() сразу после
// setupAudio(): именно с этого момента I2S тактирует кодек, значит с него и
// отсчитывается тишина. При AUDIO_START_GUARD_MS = 0 прогрев выключен.
static void sndStartGuardArm() {
#if AUDIO_START_GUARD_MS > 0
  g_sndGuardUntilMs = millis() + (uint32_t)AUDIO_START_GUARD_MS;
#endif
}

// «Тяжёлый» шаг загрузки закончился (монтирование флеша, NesCore::init, кнопки,
// скан ромов, чтение рома, первая полная отрисовка): первый звук нельзя играть ещё
// AUDIO_FIRST_SOUND_QUIET_MS — за это время «хвост» просадки питания от всплеска
// тока успевает пройти. Вызовы ПОСЛЕ первого звука игнорируются: тогда окно уже не
// нужно (а сюда попадают и перерисовки меню, и повторный скан ромов — они ничего
// откладывать не должны).
static void sndBootBusyMark(const char* tag) {
#if AUDIO_FIRST_SOUND_QUIET_MS > 0
  if (g_sndFirstSoundDone) return;
  g_sndQuietUntilMs = millis() + (uint32_t)AUDIO_FIRST_SOUND_QUIET_MS;
  DENDY_LOG("[SND] тихое окно: после «%s» первый звук не раньше +%u мс "
            "(AUDIO_FIRST_SOUND_QUIET_MS)\n",
            tag ? tag : "?", (unsigned)AUDIO_FIRST_SOUND_QUIET_MS);
#else
  (void)tag;
#endif
}

// Первый звук разрешён: после этого оба окна не действуют НИКОГДА (проверять их в
// цикле меню/игры больше не нужно). Печатаем строку — по ней в мониторе видно, что
// механизм сработал и какое «мягкое включение» настроено.
// ВАЖНО: функция идемпотентна. Её зовут и на каждом входе в меню, и на каждом старте
// игры, а первый звук в сеансе один — иначе лог печатался бы при каждом запуске рома.
static void sndFirstSoundPlayed(const char* where) {
  if (g_sndFirstSoundDone) return;        // уже было: ни окна, ни лог не нужны
  g_sndFirstSoundDone = true;
  g_sndGuardUntilMs   = 0;
  g_sndQuietUntilMs   = 0;
#if AUDIO_START_FADE_MS > 0
  DENDY_LOG("[SND] первый звук разрешён (%s) — «мягкое включение» %u мс "
            "(AUDIO_START_FADE_MS)\n",
            where ? where : "?", (unsigned)AUDIO_START_FADE_MS);
#else
  (void)where;
#endif
}

// Можно ли уже играть первый звук? true — ждём конца «прогрева» и/или «тихого
// окна». Сравнение через разность с приведением к знаковому — как везде в скетче:
// переживает переполнение millis(). После первого звука отвечает сразу false.
static bool sndStartGuardPending() {
  if (g_sndFirstSoundDone) return false;
  const uint32_t now = millis();
  if (g_sndGuardUntilMs != 0 && (int32_t)(now - g_sndGuardUntilMs) < 0) return true;
  if (g_sndQuietUntilMs != 0 && (int32_t)(now - g_sndQuietUntilMs) < 0) return true;
  return false;
}

// Дождаться разрешения первого звука — для «медленных» мест (старт игры: при
// ROM_AUTORUN_SINGLE единственный ром запускается сразу из setup(), и первым
// звуком после включения становится музыка игры; её точно так же нельзя пускать в
// неустоявшийся тракт). Ждёт максимум AUDIO_START_GUARD_WAIT_MAX_MS: если в окнах
// случайно оказались огромные числа, загрузка игры не встанет колом.
// true — ждать пришлось (для лога). Ожидание снимается в любом случае.
static bool sndStartGuardWait() {
  if (!sndStartGuardPending()) {
    sndFirstSoundPlayed("старт игры, ожидания не было");
    return false;
  }
  const uint32_t t0 = millis();
  while (sndStartGuardPending() &&
         (uint32_t)(millis() - t0) < (uint32_t)AUDIO_START_GUARD_WAIT_MAX_MS) {
    delay(1);
  }
  sndFirstSoundPlayed("старт игры, дождался");
  return true;
}

// --- «МЯГКОЕ ВКЛЮЧЕНИЕ ЗВУКА» (AUDIO_START_FADE_MS) ---------------------------
// Первый звук после включения не появляется сразу, а ПРОЯВЛЯЕТСЯ: уровень сэмплов
// поднимается от нуля до полного за AUDIO_START_FADE_MS. Считается в audioTaskEntry
// (ниже в этом же разделе) — там ВСЁ уходит в кодек, поэтому одна правка закрывает
// и джингл меню, и музыку рома. Ручки плавного включения у MAX98357 нет: его можно
// только включить или заглушить выводом SD (в этой сборке SD никуда не подключён),
// так что нарастание делается в цифре.
// g_sndFadeArmed — «проявление» ещё не начиналось (снимается навсегда после первого
// звука), g_sndFadeOn — нарастание идёт. Порог AUDIO_FADE_TRIGGER отсекает «пустой
// звук» (дизеринг ±1 LSB): иначе нарастание началось бы ещё в «прогреве».
#if AUDIO_START_FADE_MS > 0
static bool     g_sndFadeArmed = true;
static bool     g_sndFadeOn    = false;
static uint32_t g_sndFadePos   = 0;
static uint32_t g_sndFadeTotal = 1;
#endif
#define AUDIO_FADE_TRIGGER  2

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #include <driver/i2s_std.h>      // новый драйвер I2S (ESP-IDF 5.x, ядро 3.x)
  #define DENDY_I2S_NEW_API 1
#else
  #include <driver/i2s.h>          // старый драйвер I2S (ESP-IDF 4.x, ядро 2.x)
  #define DENDY_I2S_NEW_API 0
#endif

#if DENDY_I2S_NEW_API
static i2s_chan_handle_t g_i2sTx = nullptr;
#endif

static void setupAudio() {
#if DENDY_I2S_NEW_API
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num  = AUDIO_DMA_BUF_COUNT;   // количество DMA-буферов
  chanCfg.dma_frame_num = AUDIO_DMA_FRAMES;      // кадров (сэмплов) в буфере
  chanCfg.auto_clear    = true;                  // при опустошении выдавать тишину

  if (i2s_new_channel(&chanCfg, &g_i2sTx, nullptr) != ESP_OK) {
    DENDY_LOG("[I2S] i2s_new_channel failed\n");
    return;
  }

  i2s_std_config_t stdCfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                    I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)PIN_I2S_BCLK,
      .ws   = (gpio_num_t)PIN_I2S_LRCK,
      .dout = (gpio_num_t)PIN_I2S_DOUT,
      .din  = I2S_GPIO_UNUSED,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };

  if (i2s_channel_init_std_mode(g_i2sTx, &stdCfg) != ESP_OK) {
    DENDY_LOG("[I2S] init_std_mode failed\n");
    return;
  }
  if (i2s_channel_enable(g_i2sTx) != ESP_OK) {
    DENDY_LOG("[I2S] enable failed\n");
    return;
  }
#else
  const i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate          = AUDIO_SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = AUDIO_DMA_BUF_COUNT,
    .dma_buf_len          = AUDIO_DMA_FRAMES,
    .use_apll             = false,
    .tx_desc_auto_clear   = true,
    .fixed_mclk           = 0,
  };
  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr) != ESP_OK) {
    DENDY_LOG("[I2S] driver_install failed\n");
    return;
  }
  const i2s_pin_config_t pins = {
    .bck_io_num   = PIN_I2S_BCLK,
    .ws_io_num    = PIN_I2S_LRCK,
    .data_out_num = PIN_I2S_DOUT,
    .data_in_num  = I2S_PIN_NO_CHANGE,
  };
  if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) {
    DENDY_LOG("[I2S] set_pin failed\n");
    return;
  }
  i2s_zero_dma_buffer(I2S_NUM_0);
#endif
  g_i2sReady = true;
  DENDY_LOG("[I2S] MAX98357 ready @ %d Hz\n", AUDIO_SAMPLE_RATE);
}

// Задача вывода звука: забирает сэмплы APU, дублирует в стерео и пишет в DMA.
// Темп вывода задаёт сам I2S (AUDIO_SAMPLE_RATE), поэтому задача сама себя пейсит.
//
// ГЛАВНОЕ ЗДЕСЬ — блок в DMA пишется РОВНО того размера, сколько сэмплов реально
// забрано из кольца. Раньше писалось всегда AUDIO_DMA_FRAMES, а недобор добивался
// нулями, и это и был ИСТОЧНИК ХРИПА В ИГРЕ: APU отдаёт сэмплы на кадр НЕ
// ровным потоком — они набираются за время эмуляции кадра, а затем идёт пауза
// ~1,7 мс до границы следующего кадра. В этот момент в кольце оказывалось меньше
// сэмплов, чем просит блок DMA, и «добивка» вставляла в СЕРЕДИНУ музыки дырку
// примерно раз в кадр (~57 раз в секунду) — на слух это хрип/«песок». В меню
// джингл звучал чисто: там serviceMenuSound() держит кольцо полным и недобора не
// бывает. Нули уходят в кодек только если кольцо пусто ЦЕЛИКОМ (меню, пауза), и то
// коротким блоком AUDIO_FEED_IDLE — «дырка» от него в разы короче прежней.
// Подробный разбор и «подушка» звука (AUDIO_PRIME_SAMPLES) — в DendyConfig.h.
//
// ЧТО ИЗМЕНИЛОСЬ ПОТОМ: «добивка тишиной» не лечит ГЛАВНЫЙ случай — когда
// эмулятор идёт медленнее приставки (`fps emu` 49,4 вместо 60,0). Тогда тишины не
// хватает не «изредка», а ВСЕГДА: ~127 блоков в секунду по 1,45 мс, и это слышно
// как ровный «песок» поверх музыки (в логе `tishina`/`dyra` растут). Поэтому
// теперь в DMA пишется ровно AUDIO_DMA_FRAMES сэмплов, а недобор берётся
// РАСТЯЖЕНИЕМ звука: NesApu::drainResampled читает кольцо с дробным шагом
// (AUDIO_ADAPTIVE_RATE в DendyConfig.h, блок 3). Пока эмулятор выдаёт 60,0
// кадров/с, шаг равен ровно 1:1 — сэмплы уходят в кодек бит в бит, как раньше.
// (Ниже — вспомогательные функции вывода; сама задача audioTaskEntry идёт после них.)

// ---------------------------------------------------------------------------
//  Записать в I2S указанное число кадров (моно-сэмплы дублируются в L/R) из
// g_audioStereo. Один и тот же путь для обычного вывода (сколько сэмплов забрали
// из кольца, столько и отдаём) и для «выдавливания» тишины при переходе
// игра -> меню (см. audioFlushDma) — чтобы не дублировать вызовы API I2S.
static inline void audioWriteI2S(int frames) {
  size_t written = 0;
  const size_t bytes = (size_t)frames * 2 * sizeof(int16_t);
#if DENDY_I2S_NEW_API
  if (i2s_channel_write(g_i2sTx, g_audioStereo, bytes,
                        &written, AUDIO_I2S_TIMEOUT_MS) != ESP_OK) ++g_i2sTimeouts;
#else
  if (i2s_write(I2S_NUM_0, g_audioStereo, bytes, &written,
                portMAX_DELAY) != ESP_OK) ++g_i2sTimeouts;
#endif
  ++g_i2sBlocks;
}

// Выдавить из тракта «застрявший» звук: попросить задачу I2S записать
// AUDIO_DMA_BUF_COUNT+1 блоков ТИШИНЫ и подождать (до ~100 мс). Нужно на переходе
// игра <-> меню: в буферах DMA и в самом I2S остаётся ещё ~17 мс уже отданного
// звука, и без этого он звучит ПОВЕРХ меню — на слух «звук зациклился после
// выхода из игры». Заодно это гарантирует, что меню начинает играть свой джингл
// с ЧИСТОГО тракта, а не поверх хвоста игровой музыки.
// ВАЖНО: звать только из «медленного» контекста (меню/старт игры) — функция ждёт
// около 25 мс, в игровом кадре столько ждать нельзя.
static void audioFlushDma() {
  if (!g_i2sReady) return;
  g_audioFlushDone = 0;
  g_audioFlushReq  = AUDIO_DMA_BUF_COUNT + 1;
  const uint32_t t0 = millis();
  while (g_audioFlushDone < g_audioFlushReq && (uint32_t)(millis() - t0) < 100) {
    delay(1);                                 // задача I2S на другом ядре — ждём её
  }
  g_audioFlushReq  = 0;
  g_audioFlushDone = 0;
}

static void audioTaskEntry(void* arg) {
  (void)arg;
  for (;;) {
    if (!g_i2sReady) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

    // --- «Выдавить застрявший звук» (запрос от скетча, см. g_audioFlushReq) ----
    // Пишем блоки ТИШИНЫ поверх того, что уже лежит в DMA/I2S: через
    // AUDIO_DMA_BUF_COUNT+1 блоков (~23 мс) в тракте гарантированно не остаётся
    // ни одного сэмпла старого звука. Делается один раз на переход игра <-> меню.
    if (g_audioFlushReq > g_audioFlushDone) {
      for (int i = 0; i < AUDIO_DMA_FRAMES; ++i) {
        g_audioStereo[i * 2] = 0;
        g_audioStereo[i * 2 + 1] = 0;
      }
      audioWriteI2S(AUDIO_DMA_FRAMES);
      ++g_audioFlushDone;
      continue;
    }

    // Темп вывода — АДАПТИВНЫЙ (AUDIO_ADAPTIVE_RATE в DendyConfig.h, блок 3):
    // просим ровно AUDIO_DMA_FRAMES сэмплов, а NesApu::drainResampled растягивает
    // то, что есть в кольце, если эмулятор не выдаёт 60 кадров/с. Раньше
    // недостающее добивалось тишиной — это и был «песок» в музыке (~127 дырок в
    // секунду по 1,45 мс). 0 (кольцо пусто целиком и растягивать нечего) бывает
    // только в меню/на паузе: там, как и раньше, уходит короткий блок тишины.
    int got = NesCore::drainAudioAdaptive(g_audioMono, AUDIO_DMA_FRAMES);
    if (got == 0) {                                     // кольцо пусто целиком:
      ++g_i2sSilent;                                    // короткий блок «пустого звука»
      got = AUDIO_FEED_IDLE;
#if AUDIO_IDLE_DITHER
      // «ПУСТОЙ ЗВУК» (AUDIO_IDLE_DITHER в DendyConfig.h): вместо ровных нулей —
      // знак, который меняется раз в 256 сэмплов (≈86 Гц) на ±1 LSB. Для уха это
      // тишина (−90 дБ), но выход кодекa/усилителя всё время чуть «шевелится»:
      // так MAX98357 не остаётся на постоянном уровне между звуками (на переходе
      // «долгая тишина -> первый звук» у него бывает слышен щелчок), и видно, что
      // цифровой тракт жив. Само тактирование BCLK/LRCK идёт непрерывно в любом
      // случае — его даёт драйвер I2S, пока мы пишем блоки (а мы пишем всегда).
      static uint32_t idleDither = 0;
      for (int i = 0; i < got; ++i) {
        g_audioMono[i] = (int16_t)((((idleDither++ >> 8) & 1u) != 0) ? 1 : -1);
      }
#else
      memset(g_audioMono, 0, (size_t)got * sizeof(g_audioMono[0]));
#endif
    } else {                                            // есть сэмплы — учтём размер
      if (g_i2sBlkMin == 0 || got < (int)g_i2sBlkMin) g_i2sBlkMin = (uint32_t)got;
      if ((uint32_t)got > g_i2sBlkMax)                g_i2sBlkMax = (uint32_t)got;
    }
    uint32_t nonZero = 0;
    int32_t  peak    = 0;
    for (int i = 0; i < got; ++i) {                     // моно -> стерео
      int32_t sm = g_audioMono[i];
#if AUDIO_START_FADE_MS > 0
      // «МЯГКОЕ ВКЛЮЧЕНИЕ ЗВУКА» (AUDIO_START_FADE_MS, раздел 6): первый РЕАЛЬНЫЙ
      // звук после включения проявляется от нуля. «Пустой звук» (дизеринг ±1 LSB)
      // нарастание не запускает — иначе оно бы кончилось ещё в «прогреве». После
      // первого нарастания флаг снимается НАВСЕГДА: в горячем цикле остаётся одна
      // проверка bool на сэмпл.
      if (g_sndFadeArmed) {
        if (!g_sndFadeOn) {
          if (sm > AUDIO_FADE_TRIGGER || sm < -AUDIO_FADE_TRIGGER) {
            g_sndFadeOn    = true;
            g_sndFadePos   = 0;
            g_sndFadeTotal = (uint32_t)((uint64_t)AUDIO_SAMPLE_RATE *
                                        (uint64_t)AUDIO_START_FADE_MS / 1000u);
            if (g_sndFadeTotal == 0) g_sndFadeTotal = 1;
          }
        }
        if (g_sndFadeOn) {
          sm = (int32_t)(((int64_t)sm * (int64_t)g_sndFadePos) /
                         (int64_t)g_sndFadeTotal);
          if (++g_sndFadePos >= g_sndFadeTotal) {   // нарастание закончилось
            g_sndFadeOn    = false;
            g_sndFadeArmed = false;
          }
        }
      }
#endif
      const int16_t s = (int16_t)sm;
      g_audioStereo[i * 2]     = s;
      g_audioStereo[i * 2 + 1] = s;
      if (s) {                                         // статистика: что реально уходит
        ++nonZero;
        const int32_t a = (s < 0) ? -(int32_t)s : (int32_t)s;
        if (a > peak) peak = a;
        g_audZeroRun = 0;                              // дыра (серия нулей) прервалась
      } else {
        if (++g_audZeroRun > g_audZeroMax) g_audZeroMax = g_audZeroRun;
      }
      // --- «Песок-метр»: энергия всего и энергия выше ~2 кГц -----------------
      // Двухзвенный ФВЧ: два однополюсных ФНЧ (x += (s-x)>>3), их разность с
      // входом и есть ВЧ-часть. Считается по сэмплам, которые РЕАЛЬНО уходят в
      // кодек, поэтому показывает и алиасинг из APU, и вставленную тишину.
      const int32_t x = s;
      g_audLp1 += (x     - g_audLp1) >> 3;
      const int32_t h1 = x - g_audLp1;
      g_audLp2 += (h1    - g_audLp2) >> 3;
      const int32_t hp = h1 - g_audLp2;
      const uint32_t xq = (uint32_t)((x  >> 8) * (x  >> 8));   // (s>>8)^2
      const uint32_t hq = (uint32_t)((hp >> 8) * (hp >> 8));
      g_audSq   += xq;
      g_audHfSq += hq;
      ++g_audCount;
      const int32_t as = (x < 0) ? -x : x;
      if (as >= 32000) ++g_audClip;
    }
    g_i2sSamples += (uint32_t)got;
    g_i2sNonZero += nonZero;
    if (peak > g_i2sPeak) g_i2sPeak = peak;
    // В DMA уходит ровно столько, сколько забрали: стерео — по 2 слова на сэмпл.
    audioWriteI2S(got);
  }
}

// =============================================================================
//  6.1. ЗВУК МЕНЮ: 8-битный «бипер» (проверка, есть ли звук вообще)
// =============================================================================
// В меню эмулятор не работает и APU молчит — непонятно, звука нет или сломан
// тракт (I2S / MAX98357 / динамик). Поэтому меню генерирует свой квадратный
// сигнал и кладёт его в ТО ЖЕ кольцо APU через NesCore::pushAudio(): путь
// сэмплов полностью совпадает с игровым (audioTaskEntry -> I2S -> кодек).
// Услышали джингл при включении — тракт и громкость в порядке, и причину
// тишины в игре надо искать в APU/роме. Не услышали — дело в железе/I2S.
#if DENDY_MENU_SOUND

#define MENU_SND_CHUNK   256     // сэмплов генерируем за один заход
#define MENU_SND_AMP     9000    // амплитуда джингла (из 32767)
// Амплитуда «теста звука» (TURBO B в меню): 30% от полной шкалы, чтобы тест не
// бил по ушам сразу. Громкость меню на тест не влияет (см. g_sndTestMode) — он
// проверяет весь тракт на заведомо слышимом уровне; если тихо — поднимайте
// MENU_SND_TEST_AMP (например 20000), если нужно проверить на максимуме.
#define MENU_SND_TEST_AMP 9830   // 30% от 32767 (-10 dB)
#define MENU_SND_QUEUE   8       // длина очереди нот

struct MenuTone { uint16_t freq; uint16_t ms; };         // freq = 0 -> пауза

static MenuTone  g_sndQueue[MENU_SND_QUEUE];
static uint8_t   g_sndHead = 0, g_sndTail = 0;           // очередь нот (кольцевая)
static uint16_t  g_sndFreq = 0;                          // частота текущей ноты
static int16_t   g_sndStage[MENU_SND_CHUNK];             // сгенерировано, но не отправлено
static int       g_sndStageLen = 0, g_sndStagePos = 0;
static uint32_t  g_sndPhase = 0;                         // фаза квадрата (2^32 = период)
static uint32_t  g_sndStep  = 0;                         // приращение фазы на сэмпл
static uint32_t  g_sndLeft  = 0, g_sndTotal = 1;         // сэмплов до конца ноты / всего
static uint32_t  g_sndPos   = 0;                         // позиция внутри ноты (огибающая)
static bool      g_sndTestMode = false;                  // идёт «тест звука»: громкость не применять

// Полный сброс генератора звука меню: очередь нот, недоданный буфер и фаза.
// ЗАЧЕМ: джингл запуска рома (menuSoundLaunch) длиннее кольца APU (2048 сэмплов =
// 46 мс), поэтому при выходе из игры в очереди мог остаться его «хвост» и меню
// начинало играть не свой джингл, а продолжение старого — на слух это и есть
// «зациклился звук после выхода из игры». Звать ПЕРЕД тем, как класть джингл
// меню (и на старте игры, чтобы очередь не «протекала» между запусками).
static void menuSoundReset() {
  g_sndHead = g_sndTail = 0;      // очередь нот пуста
  g_sndStageLen = g_sndStagePos = 0;
  g_sndFreq  = 0;
  g_sndPhase = 0;
  g_sndStep  = 0;
  g_sndLeft  = 0;
  g_sndTotal = 1;
  g_sndPos   = 0;
  g_sndTestMode = false;          // «тест звука» (TURBO B) тоже снимаем
}

// --- «ПРОГРЕВ» ТРАКТА ТИШИНОЙ ПЕРЕД ПЕРВЫМ ЗВУКОМ (AUDIO_START_GUARD_MS) ------
// Само состояние («прогрев» + «тихое окно») и его функции (sndStartGuardArm,
// sndBootBusyMark, sndStartGuardPending/Wait) живут в разделе 6: они нужны не только
// меню — при ROM_AUTORUN_SINGLE единственный ром стартует сразу из setup(), и первым
// звуком после включения становится музыка игры. Разбор причин щелчка —
// DendyConfig.h, блок «ПЕРВЫЙ ЗВУК ПОСЛЕ ВКЛЮЧЕНИЯ». Само «мягкое включение»
// (плавное нарастание) считает audioTaskEntry — тоже в разделе 6.
//
// Вход в меню: играть джингл сразу, если первый звук уже разрешён (обычный вход, в
// том числе после игры). false — джингл ОТЛОЖЕН: его сыграет
// menuSoundDeferredJingle() в цикле меню, когда кончатся «прогрев» и «тихое окно».
static bool menuSoundJingleOnMenuEnter() {
  if (sndStartGuardPending()) return false;
  sndFirstSoundPlayed("меню, ожидания не было");
  menuSoundJingle();
  return true;
}

// Цикл меню: как только первый звук разрешён — сыграть отложенный джингл РОВНО ОДИН
// раз (дальше окна сняты навсегда, и функция молчит). true — джингл отправлен именно
// сейчас, тогда же печатается лог (см. logMenuJingleSent в runMenu).
static bool menuSoundDeferredJingle() {
  if (!sndStartGuardPending()) return false;
  sndFirstSoundPlayed("меню, дождался");
  menuSoundJingle();
  return true;
}

// Положить ноту в очередь (freq в герцах; freq = 0 — пауза).
static void menuSoundPush(uint16_t freq, uint16_t ms) {
  const uint8_t next = (uint8_t)((g_sndHead + 1) % MENU_SND_QUEUE);
  if (next == g_sndTail) return;                         // очередь полна — пропускаем
  g_sndQueue[g_sndHead].freq = freq;
  g_sndQueue[g_sndHead].ms   = ms;
  g_sndHead = next;
}

// Взять следующую ноту. false — очередь пуста (значит, тишина).
static bool menuSoundNextTone() {
  if (g_sndTail == g_sndHead) return false;
  const MenuTone t = g_sndQueue[g_sndTail];
  g_sndTail  = (uint8_t)((g_sndTail + 1) % MENU_SND_QUEUE);
  g_sndFreq  = t.freq;
  g_sndPhase = 0;
  g_sndPos   = 0;
  g_sndTotal = (uint32_t)t.ms * AUDIO_SAMPLE_RATE / 1000u;
  if (g_sndTotal == 0) g_sndTotal = 1;
  g_sndLeft  = g_sndTotal;
  g_sndStep  = g_sndFreq ? (uint32_t)(((uint64_t)g_sndFreq << 32) / AUDIO_SAMPLE_RATE) : 1;
  return true;
}

// Один сэмпл «джингла». false — играть больше нечего.
static bool menuSoundSample(int16_t* out) {
  if (g_sndLeft == 0 && !menuSoundNextTone()) return false;

  // Максимальная амплитуда: в «тесте звука» громкая всегда (тракт проверяем).
  const int32_t ampMax = g_sndTestMode ? MENU_SND_TEST_AMP : MENU_SND_AMP;

  int32_t amp = 0;
  if (g_sndFreq != 0) {
    const uint32_t attack = (uint32_t)(AUDIO_SAMPLE_RATE / 500u);     // ~2 мс вход
    if (g_sndPos < attack) {
      amp = (int32_t)((int64_t)ampMax * g_sndPos / attack);           // без щелчка в начале
    } else {                                                          // плавный спад к концу
      amp = (int32_t)((uint32_t)(ampMax / 2) +
                      (uint32_t)(ampMax / 2) * g_sndLeft / g_sndTotal);
    }
  }

  g_sndPhase += g_sndStep;
  int32_t v = (g_sndPhase & 0x80000000u) ? amp : -amp;
  if (!g_sndTestMode) v = v * (int32_t)g_volume / 100;   // тест звука громкость игнорирует
  { const int32_t a = (v < 0) ? -v : v; if (a > g_sndPeak) g_sndPeak = a; }
  *out = (int16_t)v;
  ++g_sndPos;
  --g_sndLeft;
  return true;
}

// Долить в кольцо APU столько сэмплов, сколько туда влезает. Звать в цикле меню:
// кольцо всего 2048 сэмплов (~46 мс), поэтому подкачивать надо регулярно.
static void serviceMenuSound() {
  if (!g_i2sReady) return;
  // Пока первый звук не разрешён (раздел 6: «прогрев» + «тихое окно»), в кодек идёт
  // ТОЛЬКО «пустой звук»: иначе первым звуком после включения мог бы стать «клик»
  // перемещения по списку, нажатый в первые миллисекунды. Ноты при этом не теряются —
  // они уже стоят в очереди и заиграют, как только окна кончатся.
  if (sndStartGuardPending()) return;
  for (int pass = 0; pass < 16; ++pass) {
    if (g_sndStagePos < g_sndStageLen) {                 // сначала отдаём недоданное
      const int n = NesCore::pushAudio(g_sndStage + g_sndStagePos,
                                       g_sndStageLen - g_sndStagePos);
      g_sndStagePos += n;
      g_sndPushed   += (uint32_t)n;                      // счётчик для строки [SND]
      if (g_sndStagePos >= g_sndStageLen) { g_sndStageLen = 0; g_sndStagePos = 0; }
      if (n <= 0) return;                                // кольцо полно — выходим
      continue;
    }
    g_sndStageLen = 0;
    g_sndStagePos = 0;
    while (g_sndStageLen < MENU_SND_CHUNK && menuSoundSample(&g_sndStage[g_sndStageLen])) {
      ++g_sndStageLen;
    }
    if (g_sndStageLen == 0) {                            // очередь нот пуста — тишина
      g_sndTestMode = false;                             // «тест звука» отзвучал
      return;
    }
  }
}

// Джингл меню: восходящее арпеджио — слышно сразу при включении платы.
static void menuSoundJingle() {
  menuSoundPush(523, 90);   menuSoundPush(659, 90);
  menuSoundPush(784, 90);   menuSoundPush(1047, 200);
  menuSoundPush(784, 80);   menuSoundPush(1047, 260);
}

// Короткий «клик» при перемещении по списку ромов.
static void menuSoundBlip(bool up) { menuSoundPush((uint16_t)(up ? 988 : 740), 40); }

// Подтверждение: ром выбран, игра запускается.
static void menuSoundLaunch() {
  menuSoundPush(784, 70);  menuSoundPush(1047, 70);  menuSoundPush(1319, 180);
}

// «ТЕСТ ЗВУКА»: длинный, самый громкий сигнал — на нём легко проверить тракт
// на слух (и осциллографом/мультиметром) даже при малой громкости меню:
// громкость для теста игнорируется (см. g_sndTestMode), а частота идёт «лестницей»
// 220 -> 440 -> 880 Гц: так слышно, что работает не только громкость, но и тональность.
static void menuSoundTest() {
  menuSoundPush(220, 500);
  menuSoundPush(0,   150);
  menuSoundPush(440, 500);
  menuSoundPush(0,   150);
  menuSoundPush(880, 500);
  menuSoundPush(0,   150);
  menuSoundPush(440, 900);
}

#endif  // DENDY_MENU_SOUND

// =============================================================================
//  7. ВЫВОД КАДРА NES И ИНДИКАТОР ГРОМКОСТИ
// =============================================================================
#define NES_FRAME_W 256
#define NES_FRAME_H 240

// Отправка одного кадра на панель — ОДНИМ И ТЕМ ЖЕ путём и для синхронного
// вывода (pushFrame), и для задачи вывода (pushTaskEntry). Раньше задача звала
// display.pushImage() напрямую, и DENDY_GFX_EXPLICIT_DMA на игровой кадр вообще
// не действовал: переключатель был «мёртвым» в главном режиме вывода. Теперь
// параметр работает везде (см. DendyConfig.h: кадры лежат в PSRAM, и LovyanGFX
// 1.2.19 не сбрасывает её кэш перед DMA, поэтому штатное значение — 0).
static inline void pushFrameToPanel(const uint16_t* fb) {
#if DENDY_GFX_EXPLICIT_DMA
  display.pushImageDMA(NES_VIEW_X, NES_VIEW_Y, NES_FRAME_W, NES_FRAME_H, fb);
  display.waitDMA();
#else
  display.pushImage(NES_VIEW_X, NES_VIEW_Y, NES_FRAME_W, NES_FRAME_H, fb);
#endif
}

static void pushFrame() {
  const uint16_t* fb = NesCore::frameBuffer();
  if (!fb) return;
  const uint32_t t0 = micros();
  pushFrameToPanel(fb);
  // Время передачи кадра: по нему планировщик (DENDY_FRAME_BURST_BY_PUSH) решает,
  // сколько кадров NES эмулировать за один проход цикла — см. DendyConfig.h.
  g_lastPushUs = (uint32_t)(micros() - t0);
  g_pushSumUs += g_lastPushUs;
  ++g_pushCount;
  osdDrawIfActive();                      // OSD поверх кадра: на SPI мы одни
                                          // (в статистику передачи не входит)
}

// ---------------------------------------------------------------------------
//  7.0. ПЕРЕКРЫТИЕ ВЫВОДА КАДРА С ЭМУЛЯЦИЕЙ (DENDY_FRAME_DOUBLE_BUFFER)
// ---------------------------------------------------------------------------
// Кадр 256x240 = 122 880 байт, на 40 МГц SPI это ~26 мс. Синхронный pushImage()
// держит CPU всё это время, поэтому эмулятор шёл ровно вдвое медленнее приставки
// («1 секунда игры = 2 секунды реальности»). Здесь передачу кадра ведёт отдельная
// задача на ВТОРОМ ядре, а PPU в это время рисует следующий кадр в ДРУГОЙ буфер:
// эмуляция и SPI работают параллельно, и время передачи больше не теряется.
// Мьютекс нужен потому, что кадровый путь и меню/OSD используют один SPI.
#if DENDY_FRAME_DOUBLE_BUFFER

static inline bool gfxLock() {
  return g_lcdMutex && xSemaphoreTake(g_lcdMutex, portMAX_DELAY) == pdTRUE;
}
static inline void gfxUnlock() {
  if (g_lcdMutex) xSemaphoreGive(g_lcdMutex);
}

// Задача вывода кадра (ядро DENDY_PUSH_TASK_CORE). Забирает кадр, помеченный
// главной задачей, отдаёт его в SPI и снова ждёт. Пока она этим занята, PPU
// рисует следующий кадр в другом буфере — время передачи не теряется.
static void pushTaskEntry(void* arg) {
  (void)arg;
  for (;;) {
    uint16_t* fb = nullptr;
    portENTER_CRITICAL(&g_fbMux);
    if (g_fbPending >= 0) {                // есть готовый кадр — забираем его
      g_fbReading = g_fbPending;
      g_fbPending = -1;
      fb = g_fb[g_fbReading];
    }
    portEXIT_CRITICAL(&g_fbMux);

    if (!fb) { vTaskDelay(1); continue; }   // кадра нет — 1 мс сна

    const uint32_t t0 = micros();           // начало работы с кадром
    uint32_t dt   = 0;
    bool     sent = false;                  // кадр реально ушёл в SPI (для `na ekran`)
    if (gfxLock()) {
      // Часы пускаем ПОСЛЕ мьютекса: в `peredacha` должно попадать ТОЛЬКО время
      // передачи кадра. Раньше t0 стоял до gfxLock(), и ожидание SPI (если его
      // держало меню/OSD) приписывалось передаче — в логе появлялись «страшные»
      // 35 мс вместо 12..18 мс, хотя передача просто ждала шину.
      const uint32_t t1 = micros();
      pushFrameToPanel(fb);                 // тот же путь, что и pushFrame()
      dt   = (uint32_t)(micros() - t1);     // время ТОЛЬКО передачи кадра
      sent = true;
      // OSD (громкость/цвета) рисуем здесь же, пока мьютекс наш: игровой цикл
      // за SPI не встаёт, а индикатор уходит поверх кадра (см. 7.1.1). В
      // статистику передачи он не входит — по ней планировщик считает «пачку».
      osdDrawIfActive();
      gfxUnlock();
    }
    if (!dt) dt = (uint32_t)(micros() - t0);   // мьютекс не дали — так и запишем
    g_lastPushUs  = dt;                     // статистика для строки [GAME]
    g_pushSumUs  += dt;
    if (sent) ++g_pushCount;                // счётчик РЕАЛЬНО переданных кадров —
                                            // именно он печатается как `na ekran`

    portENTER_CRITICAL(&g_fbMux);
    g_fbReading = -1;                       // буфер снова свободен
    portEXIT_CRITICAL(&g_fbMux);
  }
}

// Разовая подготовка: мьютекс, три кадра в PSRAM, задача вывода.
static void setupFrameOverlap() {
  logMemory("pered buferami kadra");           // видно, влезет ли кадр во внутреннюю SRAM
  if (!g_lcdMutex) g_lcdMutex = xSemaphoreCreateMutex();
  if (!g_fb[0])   g_fb[0] = (uint16_t*)NesCore::frameBuffer();   // кадр от NesCore
  for (int i = 1; i < DENDY_FB_COUNT; ++i) {
    if (!g_fb[i]) g_fb[i] = NesCore::allocFrameBuffer();
  }
  bool ok = true;
  for (int i = 0; i < DENDY_FB_COUNT; ++i) if (!g_fb[i]) ok = false;
  if (!ok) {
    DENDY_LOG("[VID] odin bufer kadra (net PSRAM pod ostalnye) — pushImage sinhronnyj\n");
    return;
  }
  g_fbRender  = 0;
  g_fbPending = -1;
  g_fbReading = -1;
  NesCore::setFrameBuffer(g_fb[0]);
  if (!g_pushTask) {
    xTaskCreatePinnedToCore(pushTaskEntry, "dendy_push", DENDY_PUSH_TASK_STACK,
                            nullptr, DENDY_PUSH_TASK_PRIO, &g_pushTask,
                            DENDY_PUSH_TASK_CORE);
  }
  DENDY_LOG("[VID] %d bufera kadra: emulyaciya i SPI idut parallelno (yadro %d, "
            "push %.1f ms; v SRAM: %u iz %d — ostatnye v PSRAM)\n",
            DENDY_FB_COUNT, DENDY_PUSH_TASK_CORE,
            g_lastPushUs ? g_lastPushUs / 1000.0f : 0.0f,
            (unsigned)NesCore::frameBuffersInternal(), (int)DENDY_FB_COUNT);
}

// Отдать готовый кадр задаче вывода и сразу переключить PPU на свободный буфер.
// Ожидания здесь нет by design: из трёх буферов один рисуется, один ждёт
// передачи, один передаётся — свободный есть всегда. Не успел прошлый кадр
// уйти — он просто пропускается (главное: эмуляция не тормозит).
static void pushFrameOverlap() {
  if (!g_pushTask || g_fbRender < 0) { pushFrame(); return; }   // резервный путь
#if DENDY_PUSH_EVERY_N_FRAMES > 1
  // ЭКСПЕРИМЕНТ (DENDY_PUSH_EVERY_N_FRAMES в DendyConfig.h): выводим только
  // каждый N-й эмулированный кадр. Буфер НЕ переключаем — PPU продолжит рисовать
  // в тот же кадр, эмуляция и звук идут как шли, просто на экране 60/N обновлений
  // картинки в секунду. По строкам [GAME] (`emu ms/kadr`, `hudshij`, `fps emu`)
  // сразу видно, отбирает ли передача кадра время у эмулятора.
  static uint32_t pushTick = 0;
  if (++pushTick % (uint32_t)DENDY_PUSH_EVERY_N_FRAMES) return;
#endif

  portENTER_CRITICAL(&g_fbMux);
  g_fbPending = g_fbRender;                 // «вот этот кадр надо отдать»
  int next = -1;
  for (int i = 0; i < DENDY_FB_COUNT; ++i) {
    if (i != g_fbPending && i != g_fbReading) { next = i; break; }
  }
  const int done = g_fbRender;
  portEXIT_CRITICAL(&g_fbMux);

  if (next >= 0) {
    g_fbRender = next;
    NesCore::setFrameBuffer(g_fb[next]);
  } else {                                  // не бывает при 3 буферах (страховка)
    NesCore::setFrameBuffer(g_fb[done]);
  }
}

// Перед выходом в меню: пусть задача допередаёт кадр (меню рисует по тому же SPI).
static void waitPushIdle() {
  const uint32_t t0 = micros();
  for (;;) {
    portENTER_CRITICAL(&g_fbMux);
    const bool busy = (g_fbPending >= 0) || (g_fbReading >= 0);
    portEXIT_CRITICAL(&g_fbMux);
    if (!busy) break;
    if ((uint32_t)(micros() - t0) >= DENDY_PUSH_WAIT_US) break;
    vTaskDelay(1);
  }
  g_fbPending = -1;
}
#else
#define gfxLock()    true
#define gfxUnlock()  do {} while (0)
#define setupFrameOverlap()  do {} while (0)
#define pushFrameOverlap()   pushFrame()
#define waitPushIdle()       do {} while (0)
#endif

// Кадр NES занимает ровно 256x240, а панель в альбомной ориентации шире (320).
// Полосы слева и справа (по NES_VIEW_X = 32 точки) надо один раз залить фоном:
// иначе там остаются «хвосты» меню и диагностических картинок — те самые
// «цветные квадраты и цифры по бокам игры». Заливка бесплатна (пару КБ SPI один
// раз при запуске рома), а не каждый кадр.
static void clearFrameBorders() {
  if (NES_VIEW_X <= 0) return;
  display.fillRect(0, NES_VIEW_Y, NES_VIEW_X, NES_FRAME_H, 0x0000);
  const int rightX = NES_VIEW_X + NES_FRAME_W;
  if (rightX < display.width()) {
    display.fillRect(rightX, NES_VIEW_Y, display.width() - rightX, NES_FRAME_H, 0x0000);
  }
}

// Кадровые буферы — В ЧИСТОЕ СОСТОЯНИЕ (переходы меню <-> игра).
// ЗАЧЕМ. Буферов три (DENDY_FRAME_DOUBLE_BUFFER), но NesPpu::reset() чистит только
// тот, в который PPU пишет СЕЙЧАС: в двух других остаётся ПОСЛЕДНИЙ КАДР прошлой
// игры. Пока новый ром держит рендер выключенным (загрузка уровня, распаковка CHR) —
// а так делают почти все — строки в кадр не пишутся, и на экран уходит чужой кадр:
// пользователь видит, что при запуске новой игры «мелькает предыдущая». Плюс к этому
// PPU теперь сам заливает такие строки цветом фона (см. NesPpu::blankLine), так что
// после этой чистки показывать чужое просто нечего.
// ЧТО ДЕЛАЕМ: ждём, пока задача вывода отдаст кадр (в тракте DMA/SPI не должно
// остаться ни одного сэмпла чужого кадра), берём мьютекс SPI, заливаем ВСЕ буферы
// нулями и возвращаем PPU к первому. Цена — 3 x 122 880 байт memset (единицы
// миллисекунд) и только на переходах, а не в игровом цикле.
static void invalidateFrameBuffers() {
#if DENDY_FRAME_DOUBLE_BUFFER
  waitPushIdle();                        // кадр из прошлой игры отдан в SPI целиком
  if (gfxLock()) {                       // и SPI не занят меню/OSD/диагностикой
    for (int i = 0; i < DENDY_FB_COUNT; ++i) {
      if (g_fb[i]) {
        memset(g_fb[i], 0, (size_t)NES_FRAME_W * NES_FRAME_H * sizeof(uint16_t));
      }
    }
    g_fbRender  = 0;
    g_fbPending = -1;
    g_fbReading = -1;
    if (g_fb[0]) NesCore::setFrameBuffer(g_fb[0]);
    gfxUnlock();
  }
#else
  // Один кадр, задача вывода не заведена (см. DENDY_FRAME_DOUBLE_BUFFER = 0).
  uint16_t* fb = (uint16_t*)NesCore::frameBuffer();
  if (fb) memset(fb, 0, (size_t)NES_FRAME_W * NES_FRAME_H * sizeof(uint16_t));
#endif
}

// ---------------------------------------------------------------------------
//  7.1. ПОРЯДОК КАНАЛОВ ЦВЕТА КАДРА (подбор на лету, без пересборки)
// ---------------------------------------------------------------------------
// Цвета кадра собираются таблицей палитры PPU и уходят на экран тем же
// pushImage(), что и всё остальное, поэтому в норме там должен лежать обычный
// RGB565 — тогда цвета игры совпадают с цветами меню (его рисует LovyanGFX).
// Если панель переставляет каналы, небо $22 (сине-голубое) становится розовым, а
// кирпич $17 — фиолетовым. Режим переключается на лету: SELECT + LEFT/RIGHT
// (и в игре, и в меню), всего 4 варианта (см. DENDY_COLOR_ORDER). Правильный
// подбирается по контрольной полосе в меню: верхняя строка — КАДРОВЫЙ путь
// (pushImage), нижняя — эталон LovyanGFX (fillRect). Совпали — режим верный.
// Имя режима порядка каналов — чтобы в логе и на экране было видно не только
// номер, но и ЧТО именно выбрано (см. DENDY_COLOR_ORDER в DendyConfig.h).
static const char* colorOrderName(uint8_t mode) {
  switch (mode & 0x03) {
    case 1:  return "bajty (1)";
    case 2:  return "G<->B (2)";
    case 3:  return "R<->B/BGR (3)";
    default: return "kak est (0)";
  }
}

static void applyColorOrder(uint8_t mode, const char* where) {
  NesCore::setColorOrder((uint8_t)(mode & 0x03));
  const uint8_t  m   = NesCore::colorOrder();
  const uint16_t ref = NesCore::paletteRaw(0x22);          // логический RGB565 «неба»
  // ВАЖНО (чтобы не гоняться за призраком): сравнение «кадр == эталон» ничего не
  // доказывает. В режиме 0 перекодировка — тождество (dendyColorOrder(x,0) == x),
  // поэтому «SOVPALO!» горит даже тогда, когда цвета на экране неверные.
  // Верный режим выбирается ГЛАЗАМИ: на этой плате это 1 («bajty») — с 0 небо $22
  // зелёное, с 2 — розовое (проверено на живой плате).
  DENDY_LOG("[VID] %s: poryadok cvetov (DENDY_COLOR_ORDER) = %u %s | nebo $22: kadr "
            "%04X, etalon (LovyanGFX) %04X. Sravnenie nichego ne dokazyvaet (v rezhime 0 "
            "perekodirovka - tozhdestvo): smotrite nebo NA EKRANE, a ne v logu\n",
            where, (unsigned)m, colorOrderName(m),
            (unsigned)dendyColorOrder(ref, m), (unsigned)ref);
}

// SELECT + LEFT/RIGHT — следующий/предыдущий режим цвета. true, если сменили.
static bool handleColorOrderKeys() {
  const bool left  = (g_pad & PAD_LEFT)  != 0;
  const bool right = (g_pad & PAD_RIGHT) != 0;
  if (!g_selHeld) {
    g_colLeftPrev = g_colRightPrev = false;
    return false;
  }
  bool changed = false;
  if ((left && !g_colLeftPrev) || (right && !g_colRightPrev)) {
    const int m = (int)NesCore::colorOrder() + (left ? 3 : 1);   // по кругу 0..3
    applyColorOrder((uint8_t)(m & 3), "SELECT+LEFT/RIGHT");
    changed = true;
  }
  g_colLeftPrev  = left;
  g_colRightPrev = right;
  return changed;
}

// Контрольная полоса из 64 цветов NES (каждый по 5 точек, вся ширина 320):
//   верхняя половина полосы — КАДРОВЫЙ путь (буфер + pushImage, как у кадра игры);
//   нижняя — эталон LovyanGFX (fillRect логическим RGB565).
// Совпали по всей длине — порядок каналов подобран верно.
static void drawColorOrderBar(int y) {
  enum { BW = 5, BH = 20, W = 64 * BW };                   // 320 x 20
  static uint16_t bar[W * BH];                             // буфер кадрового пути
  if (!gfxLock()) return;                                  // SPI общий с задачей вывода
  const uint8_t mode = NesCore::colorOrder();
  for (int i = 0; i < 64; ++i) {
    const uint16_t ref   = NesCore::paletteRaw((uint8_t)i);
    const uint16_t inKad = dendyColorOrder(ref, mode);
    for (int yy = 0; yy < BH; ++yy) {
      uint16_t* row = bar + (size_t)yy * W + (size_t)i * BW;
      for (int xx = 0; xx < BW; ++xx) row[xx] = inKad;
    }
    display.fillRect(i * BW, y + BH + 2, BW, BH, ref);     // эталон
  }
  display.pushImage(0, y, W, BH, bar);                     // путь самой игры
  display.fillRect(0x22 * BW, y - 3, BW, 2, UI_SEL_TEXT_COLOR);   // метка столбца «неба»

  char txt[64];
  snprintf(txt, sizeof(txt), "CVETA: %u %s  (verh = kadr, nizh = etalon)",
           (unsigned)mode, colorOrderName(mode));
  display.setTextSize(1);
  display.setTextColor(UI_TEXT_COLOR, UI_BG_COLOR);
  display.setTextDatum(lgfx::textdatum_t::top_left);
  display.drawString(txt, 8, y - 12);
  gfxUnlock();
}

// ---------------------------------------------------------------------------
//  Диагностика видеотракта (DENDY_VIDEO_TRACE = 1).
//
//  Игра занимает x = NES_VIEW_X..NES_VIEW_X+255 (то есть 32..287 при 320x240),
//  поэтому полосы 0..31 и 288..319 видны во время игры всегда. Одну и ту же
//  цветную картинку рисуем четырьмя РАЗНЫМИ способами — по тому, какие полосы
//  появились, сразу видно, где ломается вывод:
//      x   0..15  pushImageDMA() из буфера во внутренней RAM   (эталон DMA)  [верх]
//      x  16..31  pushImage()    из буфера во внутренней RAM   (путь без DMA) [верх]
//      x   0..15  восемь полос через fillRect()                (панель и окно) [низ]
//      x  16..31  pushImageDMA() из буфера в PSRAM             (путь самой игры) [низ]
//      x 288..319 проба порядка каналов цвета: 4 столбца (режимы
//                 DENDY_COLOR_ORDER 0..3, подписаны «1»..«4») × 4 цвета NES
//  Полосы сверху вниз: красная, зелёная, синяя, жёлтая, голубая, сиреневая,
//  белая, серая (по 30 строк каждая в своей половине).
//  В пробе строки сверху вниз: небо $22 (голубое), кирпич $17 (коричневый),
//  белый $30, чёрный $0F. Столбец, где небо голубое, а кирпич коричневый, —
//  это и есть нужное значение DENDY_COLOR_ORDER в DendyConfig.h.
// ---------------------------------------------------------------------------
#if DENDY_DEBUG_SERIAL && DENDY_VIDEO_TRACE

#define VT_STRIP_W 16
#define VT_STRIP_H NES_FRAME_H                 // 240
#define VT_HALF_H  (VT_STRIP_H / 2)            // 120: четыре способа вывода в двух половинах

// --- Проба порядка каналов цвета (DENDY_COLOR_ORDER) -------------------------
// 4 столбца по 8 точек (режимы перекодировки 0..3) и 4 строки-цвета NES друг под
// другом. Рисуется ТЕМ ЖЕ путём, что и кадр игры (буфер в PSRAM + pushImage),
// поэтому столбец, где небо голубое, а кирпич коричневый, и есть нужный режим —
// его значение и надо прописать в DENDY_COLOR_ORDER (DendyConfig.h).
#define VT_PROBE_W   8
#define VT_PROBE_H   (VT_STRIP_H / 4)          // 60
#define VT_PROBE_X   (DISPLAY_LANDSCAPE_W - VT_PROBE_W * 4)   // 288

static uint16_t* g_vtProbePs = nullptr;        // проба цветов (PSRAM)
static const uint32_t VT_PROBE_NES[4] = {
  0x6888FC,   // $22 — небо (должно быть голубым)
  0xE45C10,   // $17 — кирпич/земля (должен быть коричневым)
  0xFCFCFC,   // $30 — белый
  0x000000,   // $0F — чёрный
};
// RGB888 -> RGB565 (та же формула, что в палитре PPU; нужна только пробе)
static uint16_t vt565(uint32_t c)
{
  return (uint16_t)((((c >> 19) & 0x1F) << 11) | (((c >> 10) & 0x3F) << 5) | ((c >> 3) & 0x1F));
}


static const uint16_t VT_COLORS[8] = {
  0xF800, 0x07E0, 0x001F, 0xFFE0, 0x07FF, 0xF81F, 0xFFFF, 0x8410,
};
static uint16_t  g_vtPattern[VT_STRIP_W * VT_STRIP_H];   // 7,5 КБ во внутренней RAM
static uint16_t* g_vtPatternPs = nullptr;                // та же картинка в PSRAM

// Один раз готовим картинку: 8 горизонтальных полос по 30 строк.
static void vtBuildPatterns() {
  if (g_vtPatternPs) return;
  for (int y = 0; y < VT_STRIP_H; ++y) {
    const uint16_t c = VT_COLORS[y / (VT_STRIP_H / 8)];
    for (int x = 0; x < VT_STRIP_W; ++x) g_vtPattern[y * VT_STRIP_W + x] = c;
  }
  g_vtPatternPs = (uint16_t*)ps_malloc(sizeof(g_vtPattern));
  if (g_vtPatternPs) memcpy(g_vtPatternPs, g_vtPattern, sizeof(g_vtPattern));

  // Проба порядка каналов: строка = цвет NES, столбец = режим перекодировки.
  g_vtProbePs = (uint16_t*)ps_malloc((size_t)(VT_PROBE_W * 4) * VT_STRIP_H * sizeof(uint16_t));
  if (g_vtProbePs) {
    for (int r = 0; r < 4; ++r) {
      const uint16_t base = vt565(VT_PROBE_NES[r]);
      for (int y = 0; y < VT_PROBE_H; ++y) {
        for (int m = 0; m < 4; ++m) {
          const uint16_t c = dendyColorOrder(base, (uint8_t)m);
          for (int x = 0; x < VT_PROBE_W; ++x)
            g_vtProbePs[(r * VT_PROBE_H + y) * (VT_PROBE_W * 4) + m * VT_PROBE_W + x] = c;
        }
      }
    }
  }
  DENDY_LOG("[VID] proba cvetov: stolbcy 1..4 = DENDY_COLOR_ORDER 0,1,2,3 | ryady: "
            "nebo $22, kirpich $17, belyj $30, chernyj $0F | sejchas rezhim %d\n",
            (int)DENDY_COLOR_ORDER);
  DENDY_LOG("[VID] test-polosa: vnutr. RAM %u bayt, PSRAM %s\n",
            (unsigned)sizeof(g_vtPattern), g_vtPatternPs ? "OK" : "NET");
}

static void drawVideoSelfTest() {
#if DENDY_VIDEO_TEST_STRIPS
  if (!gfxLock()) return;                                  // SPI общий с задачей вывода
  vtBuildPatterns();
  // Левая полоса (x=0..31), четыре РАЗНЫХ способа вывода одной картинки:
  //   верхняя половина: 0..15 pushImageDMA из RAM, 16..31 pushImage из RAM
  //   нижняя половина : 0..15 fillRect (панель), 16..31 pushImageDMA из PSRAM
  display.pushImageDMA(0, 0, VT_STRIP_W, VT_HALF_H, g_vtPattern);          // 1) RAM + DMA
  display.waitDMA();
  display.pushImage(16, 0, VT_STRIP_W, VT_HALF_H, g_vtPattern);            // 2) RAM без DMA
  for (int i = 0; i < 8; ++i)                                              // 3) только заливка
    display.fillRect(0, VT_HALF_H + i * (VT_HALF_H / 8), VT_STRIP_W, VT_HALF_H / 8, VT_COLORS[i]);
  if (g_vtPatternPs) {                                                     // 4) PSRAM + DMA
    // Старый путь вывода кадра: DMA прямо из PSRAM. LovyanGFX не сбрасывает
    // кэш PSRAM, поэтому тут вполне может быть мусор — это и было причиной
    // артефактов. Сравните: если 1..3 чистые, а тут мусор — виноват DMA/PSRAM.
    display.pushImageDMA(16, VT_HALF_H, VT_STRIP_W, VT_HALF_H, g_vtPatternPs);
    display.waitDMA();
  }
  // Правая полоса (x=288..319) — проба порядка каналов цвета: 4 столбца
  // (режимы 0..3, подписаны «1»..«4») по 4 цвета NES. Смотрите, в каком столбце
  // небо голубое, а кирпич коричневый — этот режим и есть DENDY_COLOR_ORDER.
  if (g_vtProbePs) {
    display.pushImage(VT_PROBE_X, 0, VT_PROBE_W * 4, VT_STRIP_H, g_vtProbePs);
    display.setTextSize(1);
    display.setTextColor(0xFFFF, 0x0000);           // белым по чёрному: эти цвета
    for (int m = 0; m < 4; ++m) {                   // одинаковы во всех режимах
      const char label[2] = { (char)('1' + m), '\0' };
      display.drawString(label, VT_PROBE_X + m * VT_PROBE_W + 1, 2);
    }
  }
  gfxUnlock();                                             // SPI снова свободен
#else
  // Полосы выключены (DENDY_VIDEO_TEST_STRIPS = 0 в DendyConfig.h). Во время игры
  // игра занимает только 256 из 320 точек, поэтому полосы по бокам видно всегда —
  // выглядят они как «цветные квадраты и цифры 1..4 по краям». Порядок каналов
  // цвета теперь подбирается без них: SELECT+LEFT/RIGHT в меню (см. 7.1).
  (void)vtBuildPatterns;
#endif
}

// ASCII-«фотография» кадра: 64 знака в ширину (каждый 4-й пиксель) и 30 строк
// (каждая 8-я строка). По логу сразу видно, ЧТО нарисовано: сдвиг картинки,
// мусорные тайлы, пропавшие строки, где полоса статуса. ' ' — почти чёрный,
// '@' — самый яркий. Это основной инструмент поиска «артефактов»: по трём
// пикселям и сумме байтов причину не угадать.
static void logAsciiFrame() {
  static const char RAMP[] = " .:-=+*#%@";
  char row[65];
  for (int r = 0; r < 30; ++r) {
    for (int c = 0; c < 64; ++c) {
      const uint16_t p = NesCore::framePixel(4 * c + 2, 8 * r + 4);
      const int lum = (int)((((p >> 11) & 0x1F) * 3 + ((p >> 5) & 0x3F) * 6 + (p & 0x1F)) / 16);
      row[c] = RAMP[(lum * 9) / 31];                   // 0..9 -> RAMP
    }
    row[64] = '\0';
    DENDY_LOG("[VID] |%s|\n", row);
  }
}

// Печать состояния видеотракта: время эмуляции/передачи кадра, содержимое
// кадрового буфера и регистры PPU. Если «нечёрных» пикселей почти нет или
// mask = 00, значит картинку не рисует сам эмулятор (ром/маппер/ядро), а не
// вывод на экран.
static void logVideoDiag(uint32_t emuUs, uint32_t pushUs, uint32_t frames) {
  const uint16_t* fb = NesCore::frameBuffer();
  if (!fb) return;
  const uint32_t total = (uint32_t)NES_FRAME_W * NES_FRAME_H;
  uint32_t nonBlack = 0;
  uint32_t sum      = 0;
  for (uint32_t i = 0; i < total; ++i) {
    const uint16_t p = fb[i];
    if (p) ++nonBlack;
    sum += p;
  }
  const float n = frames ? (float)frames : 1.0f;
  DENDY_LOG("[VID] emu %.1f ms, peredacha %.1f ms, kadrov %u | ppu ctrl=%02X mask=%02X "
            "status=%02X s0=%u serial=%d pc=%04X\n",
            emuUs / n / 1000.0f, pushUs / n / 1000.0f, (unsigned)frames,
            NesCore::ppuCtrl(), NesCore::ppuMask(), NesCore::ppuStatus(),
            (unsigned)NesCore::ppuSprite0Count(), NesCore::ppuFrameSerial(), NesCore::cpuPc());
  DENDY_LOG("[VID] kadr: nechernyh %u iz %u, sum=%08X | y120 x0=%04X x128=%04X x255=%04X | "
            "buffer v %s\n",
            (unsigned)nonBlack, (unsigned)total, (unsigned)sum,
            fb[120 * NES_FRAME_W + 0], fb[120 * NES_FRAME_W + 128], fb[120 * NES_FRAME_W + 255],
            (((uintptr_t)fb >> 24) >= 0x3C) ? "PSRAM" : "vnutr. RAM");

  // --- Диагностика ядра CPU: регистры, счётчики прерываний и трасса адресов ---
  // Если irq растёт на тысячи в секунду, а в трассе PC одни и те же адреса —
  // это «шторм IRQ»: обработчик не успевает вернуть управление в игру.
  DENDY_LOG("[VID] cpu A=%02X X=%02X Y=%02X SP=%02X P=%02X | IRQ vsego %u, NMI vsego %u, "
            "instrukcij vsego %u | istochnik IRQ=%u (1=maper 2=frame 4=dmc), flagi sejchas=%u, APU=%u\n",
            (unsigned)NesCore::cpuA(), (unsigned)NesCore::cpuX(), (unsigned)NesCore::cpuY(),
            (unsigned)NesCore::cpuSp(), (unsigned)NesCore::cpuP(),
            (unsigned)NesCore::cpuIrqCount(), (unsigned)NesCore::cpuNmiCount(),
            (unsigned)NesCore::cpuInstrCount(),
            (unsigned)NesCore::lastIrqSource(), (unsigned)NesCore::irqFlagsNow(),
            (unsigned)NesCore::apuFlags());

  char tr[16 * 5 + 1];                       // 16 адресов по 4 знака + разделители
  int  pos = 0;
  for (int i = 0; i < 16; ++i)
    pos += snprintf(tr + pos, sizeof(tr) - (size_t)pos, "%04X ",
                    (unsigned)NesCore::cpuTraceAt(i));
  DENDY_LOG("[VID] trassa PC (staraja -> novaja): %s\n", tr);
  DENDY_LOG("[VID] MMC3: latch=%02X schetchik=%02X IRQ razreshen=%u | maper=%d | "
            "$8000=%02X (PRG rezhim %u, CHR rezhim %u) | R0..R7: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            (unsigned)NesCore::mapperLatch(), (unsigned)NesCore::mapperCounter(),
            (unsigned)(NesCore::mapperIrqEnabled() ? 1 : 0), NesCore::romMapperNumber(),
            (unsigned)NesCore::mapperBankSelect(),
            (unsigned)((NesCore::mapperBankSelect() & 0x40) ? 1 : 0),
            (unsigned)((NesCore::mapperBankSelect() & 0x80) ? 1 : 0),
            (unsigned)NesCore::mapperReg(0), (unsigned)NesCore::mapperReg(1),
            (unsigned)NesCore::mapperReg(2), (unsigned)NesCore::mapperReg(3),
            (unsigned)NesCore::mapperReg(4), (unsigned)NesCore::mapperReg(5),
            (unsigned)NesCore::mapperReg(6), (unsigned)NesCore::mapperReg(7));

  // --- Скролл и sprite-0 -----------------------------------------------------
  // sprite0: «strok» — в скольких строках кадра спрайт 0 вообще есть на экране,
  // «bez fona» — из них те, где фон под ним прозрачный (флаг поднять нечем).
  // Если игра стоит в цикле «LDA $2002 / AND #$40» и strok=0 — спрайт 0 уехал
  // за экран (OAM/DMA); если strok>0 и bez fona=strok — врёт рендер фона.
  uint16_t vv = 0, tt = 0; uint8_t fx = 0;
  NesCore::ppuScroll(&vv, &tt, &fx);
  const uint8_t* oam = NesCore::ppuOam();
  DENDY_LOG("[VID] skroll v=%04X t=%04X fineX=%u | sprite0: vsego %u, strok %d, bez fona %d, "
            "poslednij x=%d | OAM0: y=%u tile=%02X attr=%02X x=%u\n",
            (unsigned)vv, (unsigned)tt, (unsigned)fx,
            (unsigned)NesCore::ppuSprite0Count(), NesCore::ppuSprite0Lines(),
            NesCore::ppuSprite0Misses(), NesCore::ppuSprite0LastDot(),
            (unsigned)(oam ? oam[0] : 0xFF), (unsigned)(oam ? oam[1] : 0xFF),
            (unsigned)(oam ? oam[2] : 0xFF), (unsigned)(oam ? oam[3] : 0xFF));
  if (oam) {                                        // первые 12 спрайтов: Y TT AA XX
    char line[48];
    int  p = 0;
    line[0] = '\0';
    for (int i = 0; i < 12; ++i) {
      p += snprintf(line + p, sizeof(line) - (size_t)p, "%02X %02X %02X %02X | ",
                    (unsigned)oam[i * 4], (unsigned)oam[i * 4 + 1],
                    (unsigned)oam[i * 4 + 2], (unsigned)oam[i * 4 + 3]);
      if ((i % 3) == 2) {
        DENDY_LOG("[VID] OAM%02d..%02d: %s\n", i - 2, i, line);
        p = 0; line[0] = '\0';
      }
    }
  }

  // --- Ввод, шина и ОЗУ игры: «игра не реагирует» или «игра встала раньше» ----
  // vvod: сколько раз игра прочитала $4016 и ЧТО приняла (бит0 = A, бит3 = Start).
  // Байт должен меняться вслед за нажатиями — тогда кнопки доходят до игры.
  // Если чтений нет вообще, игра до опроса не доходит (дело не в SX1509).
  // shina: счётчики за всё время игры. Растущие $2007 (данные VRAM), $4014 (DMA
  // спрайтов) и APU означают, что игра жива и настраивает PPU/звук; `maper`
  // растёт у MMC3-игр (переключение банков); `kanaly APU` <> 0 — музыка включена.
  DENDY_LOG("[VID] vvod: $4016 oprosov %u, igra prinjalo $%02X (A,B,SL,ST,U,D,L,R)\n",
            (unsigned)NesCore::padSeenReads(), (unsigned)NesCore::padSeenByte());
  DENDY_LOG("[VID] shina: zapisej $2000=%u $2001=%u $2005=%u $2006=%u $2007=%u "
            "$4014=%u $4015=%u | APU vsego=%u, maper=%u | chtenij $2002=%u $4015=%u | "
            "kanaly APU=$%02X\n",
            (unsigned)NesCore::ioWrites(0x2000), (unsigned)NesCore::ioWrites(0x2001),
            (unsigned)NesCore::ioWrites(0x2005), (unsigned)NesCore::ioWrites(0x2006),
            (unsigned)NesCore::ioWrites(0x2007), (unsigned)NesCore::ioWrites(0x4014),
            (unsigned)NesCore::ioWrites(0x4015),
            (unsigned)NesCore::apuWrites(), (unsigned)NesCore::mapperWrites(),
            (unsigned)NesCore::ioReads(0x2002), (unsigned)NesCore::ioReads(0x4015),
            (unsigned)NesCore::apuChannels());
  DENDY_LOG("[VID] RAM: $%04X=$%02X (SMB: rezhim igry), $%04X=$%02X (SMB: knopki igry)\n",
            (unsigned)DENDY_TRACE_RAM_A, (unsigned)NesCore::ramPeek(DENDY_TRACE_RAM_A),
            (unsigned)DENDY_TRACE_RAM_B, (unsigned)NesCore::ramPeek(DENDY_TRACE_RAM_B));

  // --- Звук ------------------------------------------------------------------
  DENDY_LOG("[VID] zvuk: koltso APU %d/%d, I2S blokov %u (tishina %u, oshek %u), gromkost %u%%\n",
            NesCore::audioFill(), (int)NesApu::RING_SIZE,
            (unsigned)g_i2sBlocks, (unsigned)g_i2sSilent, (unsigned)g_i2sTimeouts,
            (unsigned)g_volume);

  // --- Что именно нарисовано (ASCII-кадр) ------------------------------------
  logAsciiFrame();
}
#endif  // DENDY_VIDEO_TRACE

// ------------------------------ Монитор звука --------------------------------
// Печатается в меню (раз в 0,5 с) и в игре (раз в секунду) — по этой строке
// видно, ДОХОДИТ ли звук до кодека, причём не запуская игру:
//   I2S gotov        — драйвер I2S поднят (setupAudio() прошёл);
//   kolco APU 0/2048 — в кольце нет сэмплов (эмулятор молчит / джингл не отдан);
//   v I2S: semplov / nenulevyh / pik — СКОЛЬКО сэмплов реально ушло в кольцо DMA
//                    драйвера I2S, сколько из них ненулевых и какова их пиковая
//                    амплитуда (из 32767). Если nenulevyh растёт, а pik около
//                    9000 (джингл меню) или хотя бы 1000 (музыка игры), то звук
//                    ИЗ ESP32 ВЫХОДИТ, и причину тишины надо искать в железе:
//                    SD (MAX98357 при напряжении на SD < 0.16 В уходит в
//                    SHUTDOWN — к GND его подключать НЕЛЬЗЯ, оставьте в воздухе
//                    или подтяните к VIN), GAIN (в воздухе = 9 дБ, 100 кОм на
//                    GND = 15 дБ), перепутанные BCLK/LRCK/DIN (16/15/17),
//                    питание кодека 2.5..5.5 В и сам динамик (BTL: только между
//                    выводами + и -, НЕ на GND);
//   blokov растёт — задача вывода крутится;
//   tishina растёт вместе с blokov — блоки уходят пустыми (кольцо APU пусто),
//                    значит эмулятор не успевает наполнять звук: смотрите
//                    [GAME] fps и [VID] emu/peredacha (см. README);
//   oshek > 0        — i2s_channel_write() не успевает за 200 мс;
//   blok min/max     — минимальный и максимальный РАЗМЕР блока (сэмплов), ушедшего
//                    в DMA, без блоков тишины. Блок меньше 256 — это нормально:
//                    задача пишет ровно столько, сколько забрала (см.
//                    audioTaskEntry). А вот min == 64 (AUDIO_FEED_IDLE) означает,
//                    что кольцо доходило до нуля и в музыку вставлялась тишина:
//                    в игре такого быть не должно (лечится «подушкой» —
//                    AUDIO_PRIME_SAMPLES в DendyConfig.h).
// ---------------------------------------------------------------------------
//  «Песок-метр»: забрать накопленное задачей I2S и обнулить окно.
//    hfPct  — доля ВЧ-энергии (выше ~2 кГц) в общей, %;
//    rmsPct — средний уровень, % от полной шкалы;
//    hole   — самая длинная серия ровно нулевых сэмплов за окно (1 мс = 44);
//    clip   — сэмплов у предела (|s| >= 32000) за окно.
//  Как читать: hf в игре — единицы процентов (чистая мелодия); десятки процентов
//  означают «песок» (шум/DMC/алиасинг при неверной частоте сэмплирования APU).
//  hole > 48 (1 мс) — в музыку вставлялась тишина (кольцо APU пусто).
// ---------------------------------------------------------------------------
static void audioMeterTake(int* hfPct, int* rmsPct, uint32_t* hole, uint32_t* clip) {
  const uint32_t cnt = g_audCount;
  const uint32_t sq  = g_audSq;
  const uint32_t hf  = g_audHfSq;
  g_audCount = 0; g_audSq = 0; g_audHfSq = 0;
  *hole = g_audZeroMax; g_audZeroMax = 0;      // текущую серию нулей НЕ сбрасываем:
  *clip = g_audClip;    g_audClip    = 0;      // дыра может идти через границу окна
  *hfPct = 0; *rmsPct = 0;
  if (!cnt || !sq) return;
  *hfPct = (int)((uint64_t)hf * 100u / (uint64_t)sq);
  // Целый корень из среднего квадрата (без sqrt): rms в единицах (сэмпл>>8), 0..128
  const uint32_t msq = sq / cnt;
  uint32_t r = 0;
  while (r < 128u && (r + 1u) * (r + 1u) <= msq) ++r;
  *rmsPct = (int)(r * 100u / 128u);
}

// ---------------------------------------------------------------------------
//  «Дефицит-метр» звука: сколько сэмплов РЕАЛЬНО ушло в I2S за секунду.
// ---------------------------------------------------------------------------
// Норма — ровно AUDIO_SAMPLE_RATE (сейчас 44 100). Меньше — эмулятор не успевает
// наполнять звук: APU отдаёт в среднем AUDIO_SAMPLE_RATE/FPS сэмплов на каждый
// ЭМУЛИРОВАННЫЙ кадр (735 при 44 100 и 367,5 при 22 050), поэтому при `fps emu ~ 52` выходит
// 52 × 735 = 38 220 сэмплов/с вместо 44 100 (при 22 050 — 19 110 вместо 22 050:
// тот же процент дефицита). Разницу задача I2S добивает
// тишиной (AUDIO_FEED_IDLE) — в музыке это и слышно как «песок»/рваность, хотя
// сам APU считает каналы правильно. Поэтому ЭТО ЧИСЛО РАЗДЕЛЯЕТ ДВЕ ПРИЧИНЫ
// «песка»: норма — звук обеспечен полностью, ищите в APU (алиасинг, см. раздел
// «Почему в игре был „песок“»); меньше — виноват ТЕМП ЭМУЛЯЦИИ (высота тона и
// темп при этом «плывут» ровно на ту же величину, см. раздел «Игра идёт
// 10–15 % медленнее — и откуда в звуке „песок“»).
// Считается по РАЗНИЦЕ счётчика сэмплов между вызовами: slot 0 — строка [GAME]
// (окно 5 с), slot 1 — строка [SND] (окно 0,5…1 с). У каждого свои счётчики,
// иначе вызовы «съедали» бы окно друг у друга.
static uint32_t audioRatePerSec(int slot) {
  static uint32_t lastMs[2]  = { 0, 0 };
  static uint32_t lastCnt[2] = { 0, 0 };
  const int      idx   = (slot == 1) ? 1 : 0;
  const uint32_t nowMs = millis();
  const uint32_t cnt   = (uint32_t)g_i2sSamples;
  uint32_t rate = 0;
  if (lastMs[idx] && nowMs != lastMs[idx])
    rate = (uint32_t)((uint64_t)(cnt - lastCnt[idx]) * 1000u / (nowMs - lastMs[idx]));
  lastMs[idx]  = nowMs;
  lastCnt[idx] = cnt;
  return rate;
}

static void logSoundDiag(const char* tag) {
  int      hfPct = 0, rmsPct = 0;
  uint32_t hole = 0, clip = 0;
  audioMeterTake(&hfPct, &rmsPct, &hole, &clip);
  // Шаг чтения кольца за окно (промилле): `za okno` показывает, СТОИТ ли тон ровно
  // (min == max) или «дышит». Это главный признак «расплывания» звука: база шага
  // берётся из периода кадра (см. DendyConfig.h, AUDIO_PRODUCER_WIN_FRAMES).
  int      rateMin = 0, rateMax = 0;
  const int rateNow = NesCore::audioRateRange(&rateMin, &rateMax);
  g_audLastHf = hfPct; g_audLastRms = rmsPct;   // их печатает строка [GAME]
  g_audLastHole = hole; g_audLastClip = clip;
  DENDY_LOG("[SND] %s: I2S %s | kolco APU %d/%d, blokov %u (tishina %u, oshek %u, "
            "blok min/max %u/%u) | gromkost %u%% | v I2S: semplov %u, "
            "nenulevyh %u, pik %d | zvuk: hf %d%%, rms %d%%, dyra %u, clip %u, drob %u, "
            "temp %d (1000 = 1:1) za okno %d..%d, golod %u, sempl/s %u iz %d",
            tag, g_i2sReady ? "gotov" : "NET",
            NesCore::audioFill(), (int)NesApu::RING_SIZE,
            (unsigned)g_i2sBlocks, (unsigned)g_i2sSilent, (unsigned)g_i2sTimeouts,
            (unsigned)g_i2sBlkMin, (unsigned)g_i2sBlkMax,
            (unsigned)g_volume,
            (unsigned)g_i2sSamples, (unsigned)g_i2sNonZero, (int)g_i2sPeak,
            hfPct, rmsPct, (unsigned)hole, (unsigned)clip,
            (unsigned)NesCore::audioDrops(),
            rateNow, rateMin, rateMax, (unsigned)NesCore::audioStarved(),
            (unsigned)audioRatePerSec(1), (int)AUDIO_SAMPLE_RATE);
#if DENDY_MENU_SOUND
  DENDY_LOG(", djingl: otdano %u, pik %d, not v ocheredi %u, v bufere %d",
            (unsigned)g_sndPushed, (int)g_sndPeak,
            (unsigned)((g_sndHead + MENU_SND_QUEUE - g_sndTail) % MENU_SND_QUEUE),
            g_sndStageLen - g_sndStagePos);
#endif
  DENDY_LOG("\n");
}

// ---------------------------------------------------------------------------
//  OSD поверх игры (полоса громкости / режим цвета кадра / яркость) — раздел 7.1.1.
//
//  ПОЧЕМУ НЕ ИЗ ИГРОВОГО ЦИКЛА: кадр в этот момент отдаёт в SPI задача вывода
//  (раздел 7.0). Если рисовать OSD из игрового цикла, gfxLock() ждал бы конца
//  передачи кадра — до 26 мс на 40 МГц и ~12 мс на 80 МГц. Игра на это время
//  замирала, пока крутишь громкость («фризы»). Теперь тела отрисовки
//  (drawVolumeOsdBody/drawColorOsdBody) зовёт задача вывода СРАЗУ после кадра,
//  под уже захваченным мьютексом — см. osdDrawIfActive() ниже.
// ---------------------------------------------------------------------------

// Индикатор громкости поверх игры (полоса + проценты).
// Вызывать ТОЛЬКО когда мьютекс SPI уже захвачен (см. osdDrawIfActive).
static void drawVolumeOsdBody() {
  const int w = 152, h = 36;
  const int x = (display.width() - w) / 2;
  const int y = display.height() - h - 8;

  display.fillRect(x, y, w, h, UI_BG_COLOR);
  display.drawRect(x, y, w, h, UI_TEXT_COLOR);

  const int inner = w - 10;
  display.fillRect(x + 5, y + 5, inner, 16, UI_FOOTER_COLOR);
  const int bars = inner * g_volume / 100;
  if (bars > 0) display.fillRect(x + 5, y + 5, bars, 16, UI_SEL_TEXT_COLOR);

  char txt[16];
  snprintf(txt, sizeof(txt), "VOL %3u%%", (unsigned)g_volume);
  display.setTextSize(1);
  display.setTextColor(UI_TEXT_COLOR);
  display.setTextDatum(lgfx::textdatum_t::middle_center);
  display.drawString(txt, x + w / 2, y + h - 8);
  display.setTextDatum(lgfx::textdatum_t::top_left);
}

// Индикатор «режим цвета кадра» поверх игры (появляется на SELECT+LEFT/RIGHT).
// Вызывать ТОЛЬКО когда мьютекс SPI уже захвачен (см. osdDrawIfActive).
static void drawColorOsdBody() {
  const int w = 200, h = 44;
  const int x = (display.width() - w) / 2;
  const int y = display.height() - h - 8;

  display.fillRect(x, y, w, h, UI_BG_COLOR);
  display.drawRect(x, y, w, h, UI_TEXT_COLOR);
  display.fillRect(x + 2, y + 2, w - 4, 4, UI_SEL_TEXT_COLOR);   // цветовая метка

  char txt[32];
  const uint8_t mode = NesCore::colorOrder();
  snprintf(txt, sizeof(txt), "CVETA %u/3", (unsigned)mode);
  display.setTextSize(2);
  display.setTextColor(UI_SEL_TEXT_COLOR);
  display.setTextDatum(lgfx::textdatum_t::middle_center);
  display.drawString(txt, x + w / 2, y + h / 2 - 2);

  // Имя режима — сразу видно, что выбрано (kak est / bajty / G<->B / BGR).
  display.setTextSize(1);
  display.setTextColor(UI_TEXT_COLOR);
  display.drawString(colorOrderName(mode), x + w / 2, y + h - 9);
  display.setTextDatum(lgfx::textdatum_t::top_left);
  display.setTextSize(1);
  display.setTextColor(UI_TEXT_COLOR);
}

// Индикатор яркости экрана поверх игры (появляется на кнопке «ЯРКОСТЬ» — вывод 8).
// Полоса — доля от верхней ступени (100 % = DISPLAY_BRIGHTNESS), подпись — процент
// из настройки, поэтому видно и «куда можно», и «сколько стоит сейчас».
// Вызывать ТОЛЬКО когда мьютекс SPI уже захвачен (см. osdDrawIfActive).
static void drawBrightOsdBody() {
  const int w = 152, h = 36;
  const int x = (display.width() - w) / 2;
  const int y = display.height() - h - 8;

  display.fillRect(x, y, w, h, UI_BG_COLOR);
  display.drawRect(x, y, w, h, UI_TEXT_COLOR);

  const int inner = w - 10;
  display.fillRect(x + 5, y + 5, inner, 16, UI_FOOTER_COLOR);
  const int bars = inner * g_brightPct / 100;
  if (bars > 0) display.fillRect(x + 5, y + 5, bars, 16, UI_SEL_TEXT_COLOR);

  char txt[16];
  snprintf(txt, sizeof(txt), "YARK %3u%%", (unsigned)g_brightPct);
  display.setTextSize(1);
  display.setTextColor(UI_TEXT_COLOR);
  display.setTextDatum(lgfx::textdatum_t::middle_center);
  display.drawString(txt, x + w / 2, y + h - 8);
  display.setTextDatum(lgfx::textdatum_t::top_left);
}

// Нарисовать OSD, если он сейчас «живой» (флаг ставит игровой цикл на 1.2 с).
//
// Зовётся ТОЛЬКО тем, кто владеет SPI: задача вывода кадра (мьютекс уже
// захвачен) и pushFrame() в резервном синхронном пути (там мы на SPI одни).
// Игровой цикл SPI не трогает вообще — поэтому подкрутка громкости/цвета больше
// не отбирает время у эмуляции.
static void osdDrawIfActive() {
  if (g_osdUntilMs == 0) return;
  if ((int32_t)(millis() - g_osdUntilMs) >= 0) return;   // время показа вышло
  if (g_osdColorMode)       drawColorOsdBody();
  else if (g_osdBrightMode) drawBrightOsdBody();
  else                      drawVolumeOsdBody();
}

// =============================================================================
//  8. КНОПКИ (SX1509) — чтение, антидребезг, логика MENU/Turbo/Громкость/Яркость
// =============================================================================
#define SX_BTN_COUNT 15

// Вывод SX1509 -> бит маски PAD_* (0 = спец-кнопка, обрабатывается отдельно).
// Массив constexpr (а не const): его проверяет static_assert ниже, а в константном
// выражении может участвовать только constexpr-массив.
static constexpr uint8_t SX_BTN_PIN[SX_BTN_COUNT] = {
  SX_PIN_A, SX_PIN_B, SX_PIN_SELECT, SX_PIN_START,
  SX_PIN_UP, SX_PIN_DOWN, SX_PIN_LEFT, SX_PIN_RIGHT,
  SX_PIN_TURBO_A, SX_PIN_TURBO_B, SX_PIN_MENU,
  SX_PIN_VOL_UP, SX_PIN_VOL_DOWN,
  SX_PIN_REGION,
  SX_PIN_BRIGHT,
};
static const uint8_t SX_BTN_PAD[SX_BTN_COUNT] = {
  PAD_A, PAD_B, PAD_SELECT, PAD_START,
  PAD_UP, PAD_DOWN, PAD_LEFT, PAD_RIGHT,
  0, 0, 0, 0, 0,
  0,                            // REGION — в игре не участвует, только меню
  0,                            // ЯРКОСТЬ — ступень по кругу (brightnessStep)
};

// Самопроверка раскладки (DendyConfig.h, блок «КНОПКИ SX1509»): один и тот же
// вывод SX1509 не должен обслуживать ДВЕ кнопки. При опечатке в SX_PIN_*
// (например Влево и «громкость −» на выводе 4) одна кнопка «зажмёт» сразу два
// действия, а по логу это почти не заметно. Проверяем на этапе компиляции.
static constexpr bool sxPinsUnique(const uint8_t* pins, int n) {
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j)
      if (pins[i] == pins[j]) return false;
  return true;
}
static_assert(sxPinsUnique(SX_BTN_PIN, SX_BTN_COUNT),
              "Раскладка SX_PIN_* в DendyConfig.h: один вывод SX1509 назначен "
              "двум кнопкам — сверьтесь с таблицей «КНОПКИ SX1509»");

// Прерывание SX1509 (GPIO3, активный уровень LOW)
static void IRAM_ATTR sxIntIsr() { g_sxIrq = true; }

// ---------------------------------------------------------------------------
//  Низкоуровневый доступ к SX1509 по второй шине I2C (в обход библиотеки).
//  Библиотека SparkFun держит readByte()/readWord() в private, а её begin()
//  делает софт-ресет и СРАЗУ читает тестовый регистр, ожидая 0xFF00. SX1509
//  после ресета ~2 мс грузит настройки и на шине молчит — из-за этого begin()
//  ошибочно сообщает «не найден», хотя чип на шине есть (виден при скане).
//  Поэтому связь проверяем и инициализацию доводим своими руками.
//  Регистры идут парами: reg — банк B (выводы 8..15), reg+1 — банк A (0..7),
//  значит бит N прочитанного 16-битного слова = вывод N.
// ---------------------------------------------------------------------------
static bool sxAddrAck() {                    // отвечает ли кто-нибудь по адресу
  I2C2.beginTransmission(SX1509_ADDRESS);
  return I2C2.endTransmission() == 0;        // 0 = адрес принят (ACK)
}

static bool sxReadWord(uint8_t reg, uint16_t* word) {
  I2C2.beginTransmission(SX1509_ADDRESS);
  I2C2.write(reg);
  if (I2C2.endTransmission() != 0) return false;
  if (I2C2.requestFrom((uint8_t)SX1509_ADDRESS, (uint8_t)2) != 2) return false;
  const int msb = I2C2.read();
  const int lsb = I2C2.read();
  if (msb < 0 || lsb < 0) return false;
  *word = (uint16_t)(((uint16_t)msb << 8) | (uint8_t)lsb);
  return true;
}

// ---------------------------------------------------------------------------
//  Диагностика SX1509/кнопок (включается DENDY_BTN_TRACE в DendyConfig.h).
//  Печатает: скан шины I2C, побитовый дамп регистров SX1509 и каждое
//  изменение состояния кнопок. Если выключено — макросы-пустышки.
// ---------------------------------------------------------------------------
#if DENDY_DEBUG_SERIAL && DENDY_BTN_TRACE

// Имена кнопок в порядке SX_BTN_PIN[] (для таблиц и лога нажатий).
static const char* SX_BTN_NAME[SX_BTN_COUNT] = {
  "A", "B", "SEL", "START", "UP", "DOWN", "LEFT", "RIGHT",
  "TURBO_A", "TURBO_B", "MENU", "VOL+", "VOL-", "REGION",
  // Имена идут ОДИН В ОДИН с SX_BTN_PIN/SX_BTN_PAD выше: при новой кнопке её имя
  // обязательно добавляется и сюда (для порядка их ровно SX_BTN_COUNT).
  "BRIGHT",
};

// Прямое чтение регистров SX1509 — общие функции sxReadWord()/sxWriteByte()
// определены выше (банк B содержит выводы 8..15, банк A — выводы 0..7).

// 16 бит в виде строки '0'/'1' (слева вывод 15, справа вывод 0).
static void bits16(uint16_t v, char* out) {
  for (int i = 0; i < 16; ++i) out[i] = (v & (1u << (15 - i))) ? '1' : '0';
  out[16] = '\0';
}

// Скан второй шины I2C: печатает все адреса, которые отвечают.
static void i2cScan() {
  int found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
    I2C2.beginTransmission(addr);
    if (I2C2.endTransmission() == 0) {
      DENDY_LOG("[I2C2] устройство по адресу 0x%02X%s\n", addr,
                (addr == SX1509_ADDRESS) ? "   <== это SX1509" : "");
      ++found;
    }
  }
  if (found == 0) {
    DENDY_LOG("[I2C2] НИКОГО не найдено (SDA=GPIO%d, SCL=GPIO%d, %u Гц)\n",
              PIN_I2C2_SDA, PIN_I2C2_SCL, (unsigned)SX1509_I2C_FREQ);
  }
}

// Побитовая таблица по всем кнопкам (SX_BTN_COUNT, блок 4 DendyConfig.h): сразу
// видно, настроен ли вывод как вход, включена ли подтяжка, разрешено ли прерывание
// и что читается сейчас.
static void logSxRegisters(const char* tag) {
  uint16_t dir = 0, data = 0, pull = 0, imask = 0, deb = 0, isrc = 0;
  uint16_t evt = 0, clkmisc = 0, debcfg = 0;
  const bool ok = sxReadWord(0x0E, &dir)     &&   // REG_DIR_B (1 = вход)
                  sxReadWord(0x10, &data)    &&   // REG_DATA_B (1 = HIGH)
                  sxReadWord(0x06, &pull)    &&   // REG_PULL_UP_B
                  sxReadWord(0x12, &imask)   &&   // REG_INTERRUPT_MASK_B
                  sxReadWord(0x23, &deb)     &&   // REG_DEBOUNCE_ENABLE_B
                  sxReadWord(0x18, &isrc)    &&   // REG_INTERRUPT_SOURCE_B
                  sxReadWord(0x1A, &evt)     &&   // REG_EVENT_STATUS_B
                  sxReadWord(0x22, &debcfg)  &&   // REG_DEBOUNCE_CONFIG (старший байт)
                  sxReadWord(0x1E, &clkmisc);     // REG_CLOCK (ст.) + REG_MISC (мл.)
  if (!ok) {
    DENDY_LOG("[SX1509] %s: регистры не читаются (нет ответа на I2C)\n", tag);
    return;
  }

  char b1[17], b2[17];
  bits16(dir, b1);  bits16(data, b2);
  DENDY_LOG("[SX1509] %s: Dir=%s  Data=%s   (выводы 15..0)\n", tag, b1, b2);
  bits16(imask, b1); bits16(deb, b2);
  DENDY_LOG("[SX1509] %s: IntMask=%s  Debounce=%s\n", tag, b1, b2);
  bits16(pull, b1);  bits16(isrc, b2);
  DENDY_LOG("[SX1509] %s: PullUp=%s  IntSource=%s\n", tag, b1, b2);
  DENDY_LOG("[SX1509] %s: Clock=0x%02X (0x40 = вкл. внутр. 2 МГц)  Misc=0x%02X  "
            "DebConf=0x%02X  EventStatus=0x%04X\n",
            tag, (unsigned)(clkmisc >> 8), (unsigned)(clkmisc & 0xFF),
            (unsigned)(debcfg >> 8), (unsigned)evt);

  DENDY_LOG("[SX1509] кнопка |выв|вход|HIGH|подт|прерв|читается\n");
  for (int i = 0; i < SX_BTN_COUNT; ++i) {
    const uint16_t bit = (uint16_t)(1u << SX_BTN_PIN[i]);
    const int high = (data & bit) ? 1 : 0;          // 1 = HIGH = отпущена
    DENDY_LOG("[SX1509] %-7s| %2u|  %d |  %d |  %d |  %d  | %s\n",
              SX_BTN_NAME[i] ? SX_BTN_NAME[i] : "?", (unsigned)SX_BTN_PIN[i],
              (dir & bit) ? 1 : 0, high, (pull & bit) ? 1 : 0,
              (imask & bit) ? 0 : 1,
              high ? "otpuschena" : "NAZHATA ili net kontakta");
  }
}

// Маска PAD_* в строку вида "UP+DOWN+A".
static void padToStr(uint8_t pad, char* out, size_t n) {
  if (!out || n == 0) return;
  struct { uint8_t bit; const char* nm; } items[] = {
    { PAD_UP, "UP" }, { PAD_DOWN, "DOWN" }, { PAD_LEFT, "LEFT" },
    { PAD_RIGHT, "RIGHT" }, { PAD_A, "A" }, { PAD_B, "B" },
    { PAD_SELECT, "SELECT" }, { PAD_START, "START" },
  };
  size_t w = 0;
  out[0] = '\0';
  for (size_t i = 0; i < sizeof(items) / sizeof(items[0]); ++i) {
    if (!(pad & items[i].bit)) continue;
    const size_t len = strlen(items[i].nm);
    if (w + len + 2 >= n) break;
    if (w) out[w++] = '+';
    memcpy(out + w, items[i].nm, len);
    w += len;
    out[w] = '\0';
  }
  if (w == 0) snprintf(out, n, "net (vse otpuscheny)");
}

// Печатается при каждом изменении состояния кнопок (и один раз при старте).
// Слово регистра данных приходит из readButtons(): шину I2C второй раз не
// дёргаем (раньше здесь был свой sxReadWord на КАЖДОЕ нажатие/отпускание).
static void logButtonState(uint8_t pad, bool menu, bool turboA, bool turboB,
                           uint16_t data) {
  static bool    bootLogged = false;
  static uint8_t prevPad    = 0;
  static bool    prevMenu   = false, prevTurboA = false, prevTurboB = false;

  if (bootLogged && pad == prevPad && menu == prevMenu &&
      turboA == prevTurboA && turboB == prevTurboB) {
    return;                                    // ничего не изменилось — молчим
  }
  const bool boot = !bootLogged;
  bootLogged = true;
  prevPad = pad; prevMenu = menu; prevTurboA = turboA; prevTurboB = turboB;

  char keys[64];
  padToStr(pad, keys, sizeof(keys));

  char hw[48];
  {
    char bits[17];
    bits16(data, bits);
    snprintf(hw, sizeof(hw), "SX1509 Data=0x%04X (b%s)", data, bits);
  }

  DENDY_LOG("[BTN] %s: pad=0x%02X [%s] MENU=%d TURBO_A=%d TURBO_B=%d | %s\n",
            boot ? "СТАРТ" : g_btnReason, pad, keys, menu, turboA, turboB, hw);

  if (!boot && !g_btnIntSeen) {
    DENDY_LOG("[BTN] подсказка: изменение поймано страховочным опросом, "
              "прерывание INT (GPIO%d) ни разу не сработало\n", PIN_SX1509_INT);
  }
}

#endif  // DENDY_DEBUG_SERIAL && DENDY_BTN_TRACE

// ---------------------------------------------------------------------------
//  Запуск второй шины I2C и поиск SX1509.
//  Перебираем варианты: штатные пины -> пины с перепутанными SDA/SCL (частый
//  монтажный ляп), и для каждого 400 кГц -> 100 кГц (на длинных проводках
//  шина не держит фронты). Пины и частоту, на которых чип ответил, печатаем.
// ---------------------------------------------------------------------------
static bool sx1509BusInit() {
  const int      pairs[][2] = { { PIN_I2C2_SDA, PIN_I2C2_SCL },     // как в конфиге
                                { PIN_I2C2_SCL, PIN_I2C2_SDA } };   // наоборот
  const uint32_t freqs[]    = { SX1509_I2C_FREQ, SX1509_I2C_FREQ_SLOW };
  const int      nPairs     = SX1509_AUTO_SWAP_PINS ? 2 : 1;

  for (int f = 0; f < 2; ++f) {
    for (int p = 0; p < nPairs; ++p) {
      I2C2.end();                     // перезапуск шины на других выводах
      delay(2);
      I2C2.begin(pairs[p][0], pairs[p][1], freqs[f]);
      delay(2);                       // чипу нужно выйти в рабочий режим
      if (!sxAddrAck()) {
        DENDY_LOG("[SX1509] нет отклика: SDA=GPIO%d, SCL=GPIO%d, %u Гц\n",
                  pairs[p][0], pairs[p][1], (unsigned)freqs[f]);
        continue;
      }
      DENDY_LOG("[SX1509] отклик 0x%02X есть: SDA=GPIO%d, SCL=GPIO%d, %u Гц\n",
                SX1509_ADDRESS, pairs[p][0], pairs[p][1], (unsigned)freqs[f]);
      if (pairs[p][0] != PIN_I2C2_SDA) {
        DENDY_LOG("[SX1509] ВНИМАНИЕ: SDA/SCL подключены НАОБОРОТ (в DendyConfig.h "
                  "задано SDA=GPIO%d, SCL=GPIO%d) — работать будет и так\n",
                  PIN_I2C2_SDA, PIN_I2C2_SCL);
      }
      if (freqs[f] != SX1509_I2C_FREQ) {
        DENDY_LOG("[SX1509] ВНИМАНИЕ: шина работает только на %u Гц — проверьте "
                  "подтяжки 4.7 кОм и длину проводов\n", (unsigned)freqs[f]);
      }
      return true;                    // нашли рабочие пины/частоту
    }
  }
  return false;
}

static void setupButtons() {
  if (!sx1509BusInit()) {                          // чип не отвечает вообще
    DENDY_LOG("[SX1509] не найден по адресу 0x%02X — кнопки не работают\n",
              SX1509_ADDRESS);
    DENDY_LOG("[SX1509] проверьте: питание 3.3 В, SDA=GPIO%d, SCL=GPIO%d, "
              "A0/A1 на GND, INT=GPIO%d, подтяжки шины 4.7 кОм\n",
              PIN_I2C2_SDA, PIN_I2C2_SCL, PIN_SX1509_INT);
#if DENDY_DEBUG_SERIAL && DENDY_BTN_TRACE
    i2cScan();                                     // что вообще есть на шине
#endif
    g_buttonsReady = false;
    return;
  }

#if DENDY_DEBUG_SERIAL && DENDY_BTN_TRACE
  i2cScan();                                       // полный список устройств
#endif

  // begin() библиотеки = софт-ресет + чтение тестового регистра (ждёт 0xFF00).
  // Паузы после ресета библиотека не делает, поэтому первый вызов может вернуть
  // 0 даже у живого чипа: ждём и проверяем связь сами.
  bool alive = (io.begin(SX1509_ADDRESS, I2C2) == I2C_ERROR_OK);
  // begin() успевает сохранить в библиотеке адрес и шину ДО проверки связи,
  // поэтому дальнейшие io.* работают с нашим чипом даже при неудачном тесте.
  DENDY_LOG("[SX1509] begin(): %s\n", alive ? "ок" : "тест 0xFF00 не прошёл");
  for (int i = 0; !alive && i < SX1509_BEGIN_TRIES; ++i) {
    delay(SX1509_RESET_SETTLE_MS);                 // SX1509 грузит настройки ~2 мс
    uint16_t imask = 0;
    alive = sxReadWord(0x12, &imask);              // REG_INTERRUPT_MASK_B
    if (alive) {
      DENDY_LOG("[SX1509] после ресета чип читается (IntMask=0x%04X) — "
                "инициализация продолжается\n", imask);
    }
  }
  if (!alive) {
    DENDY_LOG("[SX1509] ошибка: адрес 0x%02X отвечает, но регистры не читаются — "
              "проверьте подтяжки SDA/SCL 4.7 кОм и качество контактов\n",
              SX1509_ADDRESS);
    g_buttonsReady = false;
    return;
  }
  // init() внутри begin() настраивает часы SX1509 (нужны для аппаратного
  // антидребезга), но если его тест не прошёл — включаем часы сами.
  io.clock(INTERNAL_CLOCK_2MHZ);

  for (int i = 0; i < SX_BTN_COUNT; ++i) {
    // ВАЖНО: подтяжку включает только INPUT_PULLUP (внутри pinMode -> pinDir:
    // `if (inOut == INPUT_PULLUP) writePin(pin, HIGH)`). Третий параметр
    // initialLevel работает лишь для OUTPUT/ANALOG_OUTPUT — при INPUT он
    // игнорируется, входы остаются «плавающими» и читаются случайно.
    io.pinMode(SX_BTN_PIN[i], INPUT_PULLUP);
    io.debouncePin(SX_BTN_PIN[i]);
    io.enableInterrupt(SX_BTN_PIN[i], CHANGE);   // прерывание по любому фронту
  }
  io.debounceTime(SX1509_DEBOUNCE_MS);
  io.interruptSource(true);                      // сброс защёлки прерываний

  pinMode(PIN_SX1509_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SX1509_INT), sxIntIsr, FALLING);

  g_buttonsReady = true;
  DENDY_LOG("[SX1509] готов, кнопок: %d\n", SX_BTN_COUNT);
#if DENDY_DEBUG_SERIAL && DENDY_BTN_TRACE
  logSxRegisters("после инициализации");
  readButtons();                 // печатает "СТАРТ: pad=..." — база для сравнения
  DENDY_LOG("[SX1509] проверка: нажимайте кнопки по одной — на каждое\n"
            "[SX1509] изменение в мониторе появится строка [BTN]\n");
#endif
}

// Чтение состояния всех кнопок ОДНИМ сеансом I2C (только по событию/страховке).
//
// ПОЧЕМУ НЕ io.digitalRead() НА КАЖДУЮ КНОПКУ (как было раньше): в библиотеке
// SparkFun digitalRead() -> readPin() читает ДВА 16-битных регистра (Dir и Data)
// на КАЖДЫЙ вывод, то есть 15 кнопок = 30 слов = 60 обращений к шине (~3,4 мс
// на 400 кГц). Эти миллисекунды вычитались прямо из кадра эмуляции — игра
// «дёргалась» на каждое нажатие и на каждый страховочный опрос, в том числе
// когда кнопки вообще не трогали.
//
// ЗДЕСЬ: одно чтение регистра данных. Регистры идут парой (см. sxReadWord):
// 0x10 — REG_DATA_B, это выводы 8..15 (старший байт слова), 0x11 — REG_DATA_A,
// выводы 0..7 (младший байт). Значит бит N прочитанного слова = вывод N,
// и вся раскладка берётся из SX_BTN_PIN[]. Итого 2 обращения к шине (~0,25 мс)
// вместо 44 — в 20 раз дешевле, нажатие почти не отбирает время у кадра.
static void readButtons() {
  if (!g_buttonsReady) return;
  ++g_btnReadCount;              // для строки [GAME] vremya (сколько читали шину)
  io.interruptSource(true);      // читаем и сбрасываем защёлку SX1509
  g_sxIrq = false;

  uint16_t data = 0;
  if (!sxReadWord(0x10, &data)) {          // нет ответа — состояние не меняем
#if DENDY_DEBUG_SERIAL && DENDY_BTN_TRACE
    static uint32_t lastErrMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastErrMs >= 1000) {       // чтобы обрыв шины не залил лог
      lastErrMs = nowMs;
      DENDY_LOG("[SX1509] нет ответа на чтение Data (0x10) — смотрите шину I2C\n");
    }
#endif
    return;
  }

  uint8_t pad = 0;
  bool    menu = false;
  for (int i = 0; i < SX_BTN_COUNT; ++i) {
    const uint8_t pin = SX_BTN_PIN[i];
    const bool pressed = ((data >> pin) & 1u) == 0;   // активный уровень — LOW
    if (SX_BTN_PAD[i] != 0) {
      if (pressed) pad |= SX_BTN_PAD[i];
    } else {
      switch (pin) {
        case SX_PIN_TURBO_A:  g_turboA    = pressed; break;
        case SX_PIN_TURBO_B:  g_turboB    = pressed; break;
        case SX_PIN_MENU:     menu        = pressed; break;
        case SX_PIN_VOL_UP:   g_volKeyUp  = pressed; break;   // отдельная «+»
        case SX_PIN_VOL_DOWN: g_volKeyDown = pressed; break;  // отдельная «−»
        case SX_PIN_REGION:   g_regionKey  = pressed; break;  // РЕГИОН (только меню)
        case SX_PIN_BRIGHT:   g_brKey      = pressed; break;  // ЯРКОСТЬ (шаг по кругу)
        default: break;
      }
    }
  }
  logButtonState(pad, menu, g_turboA, g_turboB, data);  // пусто, если лог выключен
  g_pad      = pad;
  g_selHeld  = (pad & PAD_SELECT) != 0;
  g_menuHeld = menu;
}

// Опрос по прерыванию + страховочный опрос.
//
// ЗАЧЕМ СТРАХОВОЧНЫЙ ОПРОС ВООБЩЕ: на части сборок провод INT от SX1509 к GPIO
// не подключён (в логе это видно по подсказке «изменение поймано страховочным
// опросом»). Раньше страховка шла раз в 200 мс, из-за чего нажатие доходило до
// игры с задержкой ДО 200 МС — это и воспринималось как «кнопки тормозят».
// Теперь, пока INT ни разу не сработал, опрашиваем каждый кадр (~16 мс — как у
// настоящей приставки: чтение стоит ~0,25 мс, см. readButtons). Как только INT
// заработал, возвращаемся к редкому опросу 200 мс: он нужен лишь как страховка.
static void pollButtons() {
  // Ограничение частоты обращений к шине (BTN_POLL_MIN_US в DendyConfig.h).
  // Нужно потому, что pollButtons() зовётся по разу на КАЖДЫЙ эмулируемый кадр,
  // а внутри «пачки» кадров — до DENDY_MAX_FRAME_SKIP раз за проход. Если провод
  // INT не подключён (висит в воздухе) или залип в LOW, без ограничения мы бы
  // читали I2C на каждом вызове и отбирали у кадра миллисекунды — это видно в
  // строке [GAME] vremya (`knopki`) и по полю `oprosov`. 2 мс = 500 Гц: палец
  // быстрее не нажимает, а кадр не страдает.
  static uint32_t lastReadUs = 0;
  static bool     started    = false;
  const uint32_t  nowUs      = micros();
  if (started && (uint32_t)(nowUs - lastReadUs) < BTN_POLL_MIN_US) return;
  started    = true;
  lastReadUs = nowUs;

  if (g_sxIrq || digitalRead(PIN_SX1509_INT) == LOW) {
    g_btnIntSeen = true;
    g_btnReason  = "прерывание INT";
    readButtons();
    return;
  }
  static uint32_t lastPollMs = 0;
  const uint32_t now  = millis();
  const uint32_t step = g_btnIntSeen ? 200u : 16u;
  if (now - lastPollMs >= step) {
    lastPollMs  = now;
    g_btnReason = g_btnIntSeen ? "страховочный опрос 200 мс"
                               : "опрос по кадру (INT не приходит)";
    readButtons();
  }
}

// Логика MENU: короткое нажатие = Reset игры, удержание = выход в меню.
// Вызывается КАЖДЫЙ кадр (без обращения к I2C). Одно нажатие = одно действие.
static void updateButtonLogic() {
  if (g_menuHeld) {
    if (g_menuPressMs == 0) {
      g_menuPressMs   = millis();
      g_menuHoldFired = false;
    } else if (!g_menuHoldFired &&
               (millis() - g_menuPressMs) >= BTN_MENU_HOLD_MS) {
      g_menuRequest   = true;
      g_menuHoldFired = true;
      DENDY_LOG("[BTN] MENU: удержание %u мс -> выход в меню\n",
                (unsigned)BTN_MENU_HOLD_MS);
    }
  } else if (g_menuPressMs != 0) {
    if (!g_menuHoldFired && (millis() - g_menuPressMs) < BTN_MENU_HOLD_MS) {
      g_resetRequest = true;
      DENDY_LOG("[BTN] MENU: короткое нажатие (%u мс) -> Reset игры\n",
                (unsigned)(millis() - g_menuPressMs));
    }
    g_menuPressMs   = 0;
    g_menuHoldFired = false;
  }
}

// Громкость. Два способа, оба работают и в игре, и в меню:
//   * ОТДЕЛЬНЫЕ кнопки SX_PIN_VOL_UP (вывод 6) / SX_PIN_VOL_DOWN (вывод 5) —
//     с авто-повтором при удержании: первый шаг сразу по нажатию, дальше каждые
//     AUDIO_VOLUME_REPEAT_MS (150 мс), поэтому громкость удобно «крутить»;
//   * как было: SELECT + Вверх/Вниз — ровно один шаг на нажатие.
// Вызывается каждый кадр (обращения к I2C здесь нет: состояния уже прочитаны).
static void volumeApplyStep(bool up, const char* src) {
  int v = (int)g_volume + (up ? AUDIO_VOLUME_STEP : -AUDIO_VOLUME_STEP);
  if (v < 0)   v = 0;
  if (v > 100) v = 100;
  if (v != (int)g_volume) applyVolume((uint8_t)v);
  DENDY_LOG("[BTN] %s: громкость %u%%\n", src, (unsigned)v);
  g_osdColorMode  = false;                // OSD показывает громкость, а не цвет
  g_osdBrightMode = false;                // ...и не яркость
  g_osdUntilMs    = millis() + OSD_TIMEOUT_MS;
  if (!NesCore::isLoaded()) drawMenu();   // в меню обновляем список целиком
}

static void handleVolumeKeys() {
  // --- Отдельные кнопки громкости (с авто-повтором при удержании) ---
  static uint32_t volRepeatMs = 0;        // когда делать следующий шаг повтора
  const bool vUp   = g_volKeyUp;
  const bool vDown = g_volKeyDown;
  if (vUp || vDown) {
    if (vUp && !g_volKeyUpPrev) {                       // нажатие «+»
      volumeApplyStep(true, "VOL+");
      volRepeatMs = millis() + AUDIO_VOLUME_REPEAT_MS;
    } else if (vDown && !g_volKeyDownPrev) {            // нажатие «−»
      volumeApplyStep(false, "VOL-");
      volRepeatMs = millis() + AUDIO_VOLUME_REPEAT_MS;
    } else if ((int32_t)(millis() - volRepeatMs) >= 0) { // кнопка удержана
      volumeApplyStep(vUp, vUp ? "VOL+ (удержание)" : "VOL- (удержание)");
      volRepeatMs = millis() + AUDIO_VOLUME_REPEAT_MS;
    }
  }
  g_volKeyUpPrev   = vUp;
  g_volKeyDownPrev = vDown;

  // --- Как было: SELECT + Вверх/Вниз (один шаг на нажатие) ---
  if (!g_selHeld) {
    g_volUpPrev = g_volDownPrev = false;
    return;
  }
  const bool up   = (g_pad & PAD_UP)   != 0;
  const bool down = (g_pad & PAD_DOWN) != 0;
  if ((up && !g_volUpPrev) || (down && !g_volDownPrev)) {
    volumeApplyStep(up, up ? "SELECT+UP" : "SELECT+DOWN");
  }
  g_volUpPrev   = up;
  g_volDownPrev = down;
}

// Яркость экрана (SX_PIN_BRIGHT, вывод 8): одно нажатие — одна ступень по кругу
// (список и порядок — DISPLAY_BR_STEPS_PCT в блоке 2 «ДИСПЛЕЙ»). Работает и в игре,
// и в меню СРАЗУ: состояния уже прочитаны readButtons(), обращений к I2C тут нет.
static void handleBrightKey() {
  if (g_brKey && !g_brKeyPrev) brightnessStep();
  g_brKeyPrev = g_brKey;
}

// Turbo A/B — «моргание» ~15 Гц, добавляется к обычным кнопкам A/B.
static uint8_t turboPad() {
  if (!g_turboA && !g_turboB) {
    g_turboPhase = false;
    g_turboTimer = millis();
    return 0;
  }
  const uint32_t now = millis();
  if (now - g_turboTimer >= TURBO_HALF_PERIOD_MS) {
    g_turboTimer = now;
    g_turboPhase = !g_turboPhase;
  }
  uint8_t extra = 0;
  if (g_turboPhase) {
    if (g_turboA) extra |= PAD_A;
    if (g_turboB) extra |= PAD_B;
  }
  return extra;
}

// =============================================================================
//  9. РОМЫ: поиск в LittleFS, загрузка в PSRAM
// =============================================================================
static const char* baseName(const char* path) {
  const char* slash = strrchr(path, '/');
  return slash ? (slash + 1) : path;
}

static bool hasNesExt(const char* name) {
  const size_t n = strlen(name);
  const size_t e = strlen(ROM_EXT);
  if (n <= e) return false;
  for (size_t i = 0; i < e; ++i) {
    char a = name[n - e + i];
    const char b = ROM_EXT[i];
    if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
    if (a != b) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
//  Диагностика файловой системы (включается DENDY_ROM_TRACE в DendyConfig.h)
// ---------------------------------------------------------------------------
#if DENDY_DEBUG_SERIAL && DENDY_ROM_TRACE
// Печать содержимого каталога LittleFS: сразу видно, куда реально попали файлы
// после заливки образа и почему ром не попал в список.
static void listFs(const char* dirPath) {
  File dir = LittleFS.open(dirPath);
  if (!dir || !dir.isDirectory()) {
    DENDY_LOG("[FS] %s — каталог не найден\n", dirPath);
    if (dir) dir.close();
    return;
  }
  uint32_t files = 0, dirs = 0, bytes = 0;
  File e = dir.openNextFile();
  while (e) {
    const char* nm = e.name();
    if (e.isDirectory()) {
      ++dirs;
      DENDY_LOG("[FS]   <каталог> %s\n", nm);
    } else {
      ++files;
      bytes += (uint32_t)e.size();
      DENDY_LOG("[FS]   %-46s %8u Б%s\n", nm, (unsigned)e.size(),
                hasNesExt(nm) ? "  <- ром" : "");
    }
    e = dir.openNextFile();
  }
  dir.close();
  DENDY_LOG("[FS] %s: файлов %u, каталогов %u, объём %u КБ\n", dirPath,
            (unsigned)files, (unsigned)dirs, (unsigned)(bytes / 1024));
}

// Раздел LittleFS: адрес и размер — те же, что нужны mklittlefs/esptool.
static void logFsPartition() {
  const esp_partition_t* p = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, nullptr);
  if (!p) {
    DENDY_LOG("[FS] раздел data/spiffs НЕ найден — проверьте partitions.csv\n");
    return;
  }
  DENDY_LOG("[FS] раздел '%s': адрес 0x%06X, размер %u байт (%u КБ)\n",
            p->label, (unsigned)p->address, (unsigned)p->size,
            (unsigned)(p->size / 1024));
  DENDY_LOG("[FS] сборка образа: mklittlefs -c data -p 256 -b 4096 -s %u "
            "littlefs.bin\n", (unsigned)p->size);
  DENDY_LOG("[FS] заливка образа: esptool --chip esp32s3 --port COMx "
            "write-flash 0x%06X littlefs.bin\n", (unsigned)p->address);
}
#endif  // DENDY_DEBUG_SERIAL && DENDY_ROM_TRACE

// Сравнение имён без учёта регистра (шрифт/сортировка меню).
static int compareNames(const char* a, const char* b) {
  while (*a && *b) {
    char ca = *a++;
    char cb = *b++;
    if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
    if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
    if (ca != cb) return (ca < cb) ? -1 : 1;
  }
  if (*a == *b) return 0;
  return (*a) ? 1 : -1;
}

// Сортировка списка по имени (простая вставка — файлов не больше 64).
static void sortRoms() {
  for (int i = 1; i < g_romCount; ++i) {
    char     p[sizeof(g_romPath[0])];
    char     n[ROM_NAME_LEN];
    uint32_t s;
    memcpy(p, g_romPath[i], sizeof(p));
    memcpy(n, g_romName[i], sizeof(n));
    s = g_romSize[i];

    int j = i - 1;
    while (j >= 0 && compareNames(g_romName[j], n) > 0) {
      memcpy(g_romPath[j + 1], g_romPath[j], sizeof(p));
      memcpy(g_romName[j + 1], g_romName[j], sizeof(n));
      g_romSize[j + 1] = g_romSize[j];
      --j;
    }
    memcpy(g_romPath[j + 1], p, sizeof(p));
    memcpy(g_romName[j + 1], n, sizeof(n));
    g_romSize[j + 1] = s;
  }
}

// Обход папки /roms — заполняет g_romPath / g_romName / g_romSize.
// Если папки нет, ромы ищутся в корне LittleFS (на случай заливки не в roms).
static void scanRoms() {
  g_romCount = 0;

  const char* dirPath = ROM_DIR;
  File dir = LittleFS.open(ROM_DIR);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    DENDY_LOG("[ROM] папка %s не найдена — смотрю корень \"/\"\n", ROM_DIR);
    dirPath = "/";
    dir = LittleFS.open(dirPath);
  }
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    DENDY_LOG("[ROM] в LittleFS нет ни %s, ни файлов в корне\n", ROM_DIR);
    listFs("/");                 // печатает, что вообще есть (или что пусто)
    return;
  }
  listFs(dirPath);               // листинг того каталога, который обходим

  const bool root = (dirPath[1] == '\0');
  uint32_t   seen = 0;           // сколько файлов вообще лежит в каталоге

  File f = dir.openNextFile();
  while (f && g_romCount < ROM_MAX_FILES) {
    if (!f.isDirectory()) {
      ++seen;
      const char* nm = f.name();
      if (nm && hasNesExt(nm)) {
        const char* base = baseName(nm);
        if (root) {
          snprintf(g_romPath[g_romCount], sizeof(g_romPath[0]), "/%s", base);
        } else {
          snprintf(g_romPath[g_romCount], sizeof(g_romPath[0]), "%s/%s",
                   dirPath, base);
        }
        // имя для меню — без расширения
        char tmp[ROM_NAME_LEN];
        sanitizeName(tmp, base, sizeof(tmp));
        char* dot = strrchr(tmp, '.');
        if (dot) *dot = '\0';
        memcpy(g_romName[g_romCount], tmp, sizeof(g_romName[0]));
        g_romSize[g_romCount] = (uint32_t)f.size();
        DENDY_LOG("[ROM] %-32s %u КБ\n", base,
                  (unsigned)(g_romSize[g_romCount] / 1024));
        ++g_romCount;
      } else {
        DENDY_LOG("[ROM] пропущен (нужен %s): %s\n", ROM_EXT,
                  nm ? nm : "?");
      }
    }
    f = dir.openNextFile();
  }
  dir.close();
  DENDY_LOG("[ROM] в %s файлов %u, ромов принято %d\n", dirPath,
            (unsigned)seen, g_romCount);

  sortRoms();
  if (g_romSelected >= g_romCount) g_romSelected = g_romCount - 1;
  if (g_romSelected < 0) g_romSelected = 0;
  g_romScroll = 0;
}

// Чтение файла рома в PSRAM и передача его ядру (ядро забирает владение).
static bool loadRomFromFile(const char* path, uint32_t size) {
  if (size < 16 || size > ROM_MAX_SIZE) {
    DENDY_LOG("[ROM] размер %u не подходит\n", (unsigned)size);
    return false;
  }
  File f = LittleFS.open(path, "r");
  if (!f) {
    DENDY_LOG("[ROM] не открыть %s\n", path);
    return false;
  }
  uint8_t* buf = (uint8_t*)ps_malloc(size);
  if (!buf) {
    f.close();
    DENDY_LOG("[ROM] нет памяти в PSRAM (%u байт)\n", (unsigned)size);
    return false;
  }

  size_t done = 0;
  while (done < size) {
    const int n = f.read(buf + done, size - done);
    if (n <= 0) break;
    done += (size_t)n;
  }
  f.close();

  if (done != size) {
    DENDY_LOG("[ROM] прочитано %u из %u байт\n", (unsigned)done, (unsigned)size);
    free(buf);
    return false;
  }

  NesCore::unloadRom();                     // на всякий случай освобождаем старое
  if (!NesCore::loadRom(buf, size)) {       // при ошибке владение остаётся у нас
    free(buf);
    DENDY_LOG("[ROM] ядро отклонило образ (%s)\n",
              NesCore::romSupported() ? "нет заголовка NES" : "маппер не поддержан");
    return false;
  }
  DENDY_LOG("[ROM] загружен: %s, mapper %d (%s)\n", path,
            NesCore::romMapperNumber(), NesCore::romMapperName());
  // Образ рома прочитан из флеша в PSRAM — самый длинный «тяжёлый» шаг перед первой
  // музыкой игры (ROM_AUTORUN_SINGLE). Отмечаем его для «тихого окна»
  // (AUDIO_FIRST_SOUND_QUIET_MS): музыка не заиграет в «хвост» просадки питания.
  sndBootBusyMark("rom");
  return true;
}

// =============================================================================
//  10. МЕНЮ: отрисовка и навигация
// =============================================================================
// Название рома с обрезкой по ширине строки (шрифт size=2 -> ~12 px на символ).
static void drawRomName(const char* src, int x, int y, size_t maxChars) {
  char tmp[ROM_NAME_LEN + 2];
  size_t n = strlen(src);
  if (n > maxChars) {
    memcpy(tmp, src, maxChars - 1);
    tmp[maxChars - 1] = '~';
    tmp[maxChars] = '\0';
  } else {
    memcpy(tmp, src, n + 1);
  }
  display.drawString(tmp, x, y);
}

// Одна строка списка ромов.
static void drawMenuRow(int row, int index, bool selected) {
  const int y = MENU_LIST_Y + row * MENU_ROW_H;
  const int w = DISPLAY_LANDSCAPE_W - 8;

  if (index < 0 || index >= g_romCount) {          // пустая строка
    display.fillRect(4, y, w, MENU_ROW_H - 2, UI_BG_COLOR);
    return;
  }

  const uint16_t bg = selected ? UI_SEL_BG_COLOR : UI_BG_COLOR;
  display.fillRect(4, y, w, MENU_ROW_H - 2, bg);
  display.setTextColor(selected ? UI_SEL_TEXT_COLOR : UI_TEXT_COLOR, bg);

  display.setTextSize(2);
  display.setTextDatum(lgfx::textdatum_t::top_left);
  drawRomName(g_romName[index], 8, y, MENU_NAME_CHARS - 4);

  char size[16];
  snprintf(size, sizeof(size), "%uK",
           (unsigned)((g_romSize[index] + 1023) / 1024));
  display.setTextSize(1);
  display.setTextDatum(lgfx::textdatum_t::top_right);
  display.drawString(size, DISPLAY_LANDSCAPE_W - 8, y + 5);
  display.setTextDatum(lgfx::textdatum_t::top_left);

  if (selected) display.fillRect(4, y, 3, MENU_ROW_H - 2, UI_SEL_TEXT_COLOR);
}

// Полная перерисовка меню (заголовок, список, полоса прокрутки, подсказки).
static void drawMenu() {
  display.fillScreen(UI_BG_COLOR);

  display.setTextDatum(lgfx::textdatum_t::top_left);
  display.setTextColor(UI_TITLE_COLOR, UI_BG_COLOR);
  display.setTextSize(2);
  display.drawString("DendyGO " DENDY_VERSION, 8, MENU_TITLE_Y);

  char info[64];
  char reg[16];
  regionModeText(reg, sizeof(reg));
  snprintf(info, sizeof(info), "VOL %u%%  YARK %u%%  TV %s",
           (unsigned)g_volume, (unsigned)g_brightPct, reg);
  display.setTextSize(1);
  display.setTextColor(UI_TEXT_COLOR, UI_BG_COLOR);
  display.setTextDatum(lgfx::textdatum_t::top_right);
  display.drawString(info, DISPLAY_LANDSCAPE_W - 8, MENU_TITLE_Y + 6);

  if (g_romCount > 0) {
    snprintf(info, sizeof(info), "igr: %d   vybor: %d/%d", g_romCount,
             g_romSelected + 1, g_romCount);
  } else {
    snprintf(info, sizeof(info), "net fajlov %s v %s", ROM_EXT, ROM_DIR);
  }
  display.drawString(info, DISPLAY_LANDSCAPE_W - 8, MENU_SUB_Y);
  display.setTextDatum(lgfx::textdatum_t::top_left);

  for (int r = 0; r < MENU_ROWS; ++r) {
    const int idx = g_romScroll + r;
    drawMenuRow(r, (idx < g_romCount) ? idx : -1, idx == g_romSelected);
  }

  // Полоса прокрутки справа (если ромов больше, чем строк).
  if (g_romCount > MENU_ROWS) {
    const int x = DISPLAY_LANDSCAPE_W - 3;
    const int h = MENU_ROW_H * MENU_ROWS;
    display.fillRect(x, MENU_LIST_Y, 3, h, UI_FOOTER_COLOR);
    const int bh = h * MENU_ROWS / g_romCount;
    const int by = MENU_LIST_Y + (h - bh) * g_romScroll / (g_romCount - MENU_ROWS);
    display.fillRect(x, by, 3, bh, UI_SEL_TEXT_COLOR);
  }

  display.setTextSize(1);
  display.setTextColor(UI_FOOTER_COLOR, UI_BG_COLOR);
  display.drawString("A / START - start      UP/DOWN - vybor      L / R - stranica",
                     8, MENU_FOOTER_Y);
  display.drawString("MENU - spisok / reset      SELECT+UP/DOWN - gromkost",
                     8, MENU_FOOTER_Y + 10);
  // Регион ТВ переключается кнопкой на выводе 4 SX1509 (см. SX_PIN_REGION), текущий
  // режим виден справа вверху («TV AUTO(NTSC)» / «TV NTSC» / «TV PAL»); яркость —
  // кнопкой на выводе 8 (SX_PIN_BRIGHT), её значение там же («YARK 70%»). Строку
  // держим короткой: подсказки идут тремя строками от MENU_FOOTER_Y (206, 216, 226),
  // а экран — 240 точек по высоте; «K4»/«K8» — это те самые выводы SX1509.
  display.drawString("TURBO B - TEST  SEL+L/R - CVETA  K4 - TV  K8 - YARK",
                     8, MENU_FOOTER_Y + 20);
  display.setTextColor(UI_TEXT_COLOR, UI_BG_COLOR);
  // Полная перерисовка меню — последний «тяжёлый» шаг перед ПЕРВЫМ звуком в меню
  // (список, шрифты, полоса прокрутки): отмечаем его для «тихого окна»
  // (AUDIO_FIRST_SOUND_QUIET_MS), чтобы джингл не совпал с всплеском тока по SPI.
  // После первого звука вызов ничего не делает (см. sndBootBusyMark).
  sndBootBusyMark("kadr");
}

// UP/DOWN в меню: первый шаг сразу, затем автоповтор.
static int menuNavStep() {
  static uint8_t  dir = 0;
  static uint32_t nextMs = 0;
  const bool up   = (g_pad & PAD_UP)   != 0;
  const bool down = (g_pad & PAD_DOWN) != 0;
  const uint8_t now = up ? 1 : (down ? 2 : 0);
  if (now == 0) { dir = 0; return 0; }
  const uint32_t t = millis();
  if (now != dir) { dir = now; nextMs = t + 400; return (now == 1) ? -1 : 1; }
  if ((int32_t)(t - nextMs) >= 0) { nextMs = t + 110; return (now == 1) ? -1 : 1; }
  return 0;
}

// LEFT/RIGHT в меню: листание страницами (с автоповтором).
static int menuPageStep() {
  static uint8_t  dir = 0;
  static uint32_t nextMs = 0;
  const bool left  = (g_pad & PAD_LEFT)  != 0;
  const bool right = (g_pad & PAD_RIGHT) != 0;
  const uint8_t now = left ? 1 : (right ? 2 : 0);
  if (now == 0) { dir = 0; return 0; }
  const uint32_t t = millis();
  if (now != dir) { dir = now; nextMs = t + 500; return (now == 1) ? -MENU_ROWS : MENU_ROWS; }
  if ((int32_t)(t - nextMs) >= 0) { nextMs = t + 250; return (now == 1) ? -MENU_ROWS : MENU_ROWS; }
  return 0;
}

// =============================================================================
//  11. МЕНЮ РОМОВ (главный экран) И ИГРОВОЙ ЦИКЛ
// =============================================================================
// Лог «джингл меню отправлен»: печатается и при входе в меню, и тогда, когда
// отложенный джингл наконец заиграл после «прогрева» тракта тишиной (см. 6.1,
// menuSoundDeferredJingle). Раньше он печатался сразу на входе в меню — теперь
// это было бы неверно: до конца прогрева джингл ещё не отправлен, и в полях
// `djingl`/`pik` стояли бы нули, хотя тракт исправен.
static void logMenuJingleSent() {
  logSoundDiag("меню: джингл отправлен");
  DENDY_LOG("[SND] меню: джингл отправлен. Если в динамике тишина — смотрите [SND] "
            "выше: 'nenulevyh' должны расти, 'pik' ~9000. Программный тракт, значит, "
            "исправен, и виновато железо. Быстрый тест на слух: TURBO B в меню "
            "(громкая лестница 220/440/880 Гц). Пины: BCLK=%d LRCK=%d DIN=%d | SD "
            "(shutdown) у MAX98357 НЕ должен быть на GND (при SD<0.16 В усилитель "
            "ВЫКЛЮЧЕН!) — оставьте SD в воздухе или подтяните к VIN; GAIN в воздухе = "
            "9 дБ; динамик подключается только между выводами + и - (не на GND)\n",
            PIN_I2S_BCLK, PIN_I2S_LRCK, PIN_I2S_DOUT);
}

static void runMenu() {
  g_menuRequest  = false;
  g_resetRequest = false;

  // Кадровые буферы — в чистое состояние: в них лежат последние кадры вышедшей игры
  // (см. invalidateFrameBuffers). Сюда заходим сразу после выхода из игры (и вообще
  // при каждом входе в меню), поэтому чужому кадру больше неоткуда взяться.
  invalidateFrameBuffers();

  // Звук: полностью приводим тракт к состоянию меню и играем джингл — это заодно
  // проверка тракта «кольцо APU -> задача I2S -> MAX98357 -> динамик».
  // Тишина здесь означает проблему железа/I2S, а не эмулятора.
  // ЧТО ИМЕННО СБРАСЫВАЕТСЯ (иначе после игры слышен «зацикленный» хвост):
  //   * setAudioAdaptive(false) — в меню растягивать нечего (кольцо держит
  //     serviceMenuSound), и регулятор темпа больше не считает окно по игре;
  //   * flushAudio() + resetAudioStream() — кольцо сэмплов и состояние регулятора;
  //   * audioFlushDma() — УЖЕ ОТДАННЫЙ в DMA/I2S звук (~17 мс): без этого он
  //     играет ещё раз поверх меню (на слух — «звук зациклился»);
  //   * menuSoundReset() — очередь и фаза генератора звука меню: при выходе из
  //     игры там мог остаться недоданный джингл запуска рома (он длиннее кольца
  //     APU, 46 мс), и меню начинало играть ЕГО продолжение.
  NesCore::setAudioAdaptive(false);
  NesCore::flushAudio();
  NesCore::resetAudioStream();
  audioFlushDma();
  menuSoundReset();
  // Джингл меню. Играем сразу, если «прогрев» тракта тишиной уже пройден (обычный
  // вход в меню, в том числе после игры). В первый вход после включения он
  // ОТЛОЖЕН на AUDIO_START_GUARD_MS: тогда в кодек идёт только «пустой звук», а
  // джингл заиграет в цикле меню (menuSoundDeferredJingle ниже) — так первый звук
  // не совпадает с инициализацией кодека/кнопок/флеша и первой перерисовкой
  // экрана, из-за чего в его начале проскакивал щелчок. Разбор — DendyConfig.h,
  // блок «ПЕРВЫЙ ЗВУК ПОСЛЕ ВКЛЮЧЕНИЯ».
  const bool jingleNow = menuSoundJingleOnMenuEnter();
  serviceMenuSound();
  if (jingleNow) {
    logMenuJingleSent();
  } else {
    DENDY_LOG("[SND] меню: джингл отложен — идут окна «прогрева тракта» (%u мс, "
              "AUDIO_START_GUARD_MS) и «тихого окна» после загрузочных всплесков "
              "(%u мс, AUDIO_FIRST_SOUND_QUIET_MS): в кодек пока идёт пустой звук, "
              "джингл заиграет в цикле меню\n",
              (unsigned)AUDIO_START_GUARD_MS,
              (unsigned)AUDIO_FIRST_SOUND_QUIET_MS);
  }

  if (g_romSelected >= g_romCount) g_romSelected = g_romCount - 1;
  if (g_romSelected < 0) g_romSelected = 0;
  g_romScroll = 0;
  if (g_romSelected >= MENU_ROWS) g_romScroll = g_romSelected - MENU_ROWS + 1;
  if (g_romCount > MENU_ROWS && g_romScroll > g_romCount - MENU_ROWS) {
    g_romScroll = g_romCount - MENU_ROWS;
  }
  if (g_romScroll < 0) g_romScroll = 0;

  drawMenu();

  for (;;) {
    pollButtons();
    updateButtonLogic();
    handleVolumeKeys();
    handleBrightKey();                  // кнопка «ЯРКОСТЬ» (вывод 8): шаг по кругу
    backlightGlideStep();               // доводим яркость плавно (только ШИМ, без SPI)
    // «Прогрев» кончился (первый вход в меню после включения) — играем отложенный
    // джингл. Дальше вызовы молчат: ожидание уже снято (см. 6.1).
    if (menuSoundDeferredJingle()) logMenuJingleSent();
    serviceMenuSound();                 // подкачиваем звук меню (кольцо APU ~46 мс)

    // --- ТЕСТ ЗВУКА: TURBO B в меню -------------------------------------------
    // Громкая «лестница» 220 -> 440 -> 880 Гц, громкость для теста игнорируется.
    // Это проверка всего тракта (кольцо APU -> задача I2S -> MAX98357 -> динамик)
    // без запуска рома: услышали три тона — со звуком в железе всё в порядке.
    {
      static bool turboBPrev = false;
      if (g_turboB && !turboBPrev) {
#if DENDY_MENU_SOUND
        g_sndTestMode = true;           // тест звучит на полной амплитуде
#endif
        menuSoundTest();
        serviceMenuSound();
        logSoundDiag("меню: TEST ZVUKA 220/440/880 Gc");
        DENDY_LOG("[SND] тест звука отправлен в I2S. Если в динамике тишина, а выше "
                  "'nenulevyh' растёт и 'pik' ~30000 — до кодека звук доходит, и "
                  "виновато железо: у MAX98357 пин SD НА GND означает SHUTDOWN "
                  "(подтяжка к VIN или в воздухе!), GAIN в воздухе = 9 дБ, динамик — "
                  "только между выводами + и - (не на GND), пины BCLK=%d LRCK=%d DIN=%d\n",
                  PIN_I2S_BCLK, PIN_I2S_LRCK, PIN_I2S_DOUT);
      }
      turboBPrev = g_turboB;
    }

    // --- РЕГИОН ТВ (PAL/NTSC): кнопка SX_PIN_REGION (вывод 4 SX1509) -----------
    // Переключает режим по кругу AUTO -> NTSC -> PAL и СРАЗУ его применяет, чтобы
    // в меню (справа вверху) было видно, что выбрано, и чтобы следующий запуск
    // рома пошёл уже в новом режиме. Настройка сохраняется в LittleFS, поэтому
    // живёт до следующего нажатия. В игре кнопка не обрабатывается: регион —
    // это весь тайминг эмуляции, его меняют только до старта рома.
    {
      static bool regionPrev = false;
      if (g_regionKey && !regionPrev) {
        g_regionMode = (uint8_t)((g_regionMode + 1) % 3);
        saveRegionMode();
        applyRegionMode("knopka TV");
        menuSoundBlip(true);
        drawMenu();
      }
      regionPrev = g_regionKey;
    }

    // --- SELECT удерживается: калибровочная полоса цветов ----------------------
    // SELECT+LEFT/RIGHT перебирает режимы порядка каналов (0..3, см. DENDY_COLOR_ORDER).
    // Сверху рисуется КАДРОВЫЙ путь (pushImage), снизу — эталон LovyanGFX
    // (fillRect). Совпали по цвету — это и есть нужный режим.
    {
      static bool barShown = false;
      const bool changed = handleColorOrderKeys();
      const bool volumeNow = (g_pad & (PAD_UP | PAD_DOWN)) != 0;   // SELECT+UP/DN — звук
      if (g_selHeld && !volumeNow) {
        if (changed || !barShown) {
          drawColorOrderBar(126);
          barShown = true;
        }
      } else if (barShown) {
        barShown = false;
        drawMenu();                     // убираем полосу — возвращаем список ромов
      }
    }

    // Монитор звука в меню: раз в 2 с видно, уходят ли сэмплы джингла в I2S,
    // или тракт молчит (тогда причина в железе: BCLK/LRCK/DIN, SD->GND, динамик).
    // Почему 2000, а не 500 мс: строка ~470 символов = ~40 мс занятого UART на
    // 115200. Раз в 0,5 с это ~8 % всего канала UART, который делит с эмулятором
    // то же ядро; для контроля тракта (и `sempl/s` — счётчик берётся заново от
    // вызова до вызова) двух секунд достаточно.
    {
      static uint32_t sndLogMs = 0;
      if ((uint32_t)(millis() - sndLogMs) >= 2000) {
        sndLogMs = millis();
        logSoundDiag("меню");
      }
    }

    // MENU в меню — заново прочитать список ромов из LittleFS
    if (g_menuRequest) {
      g_menuRequest = false;
      menuSoundBlip(false);             // «перечитать список»
      showMessage("Scanning...", ROM_DIR);
      scanRoms();
      g_romSelected = 0;
      g_romScroll   = 0;
      drawMenu();
    }
    g_resetRequest = false;             // в меню Reset не нужен

    // С зажатым SELECT меню не листаем: SELECT+UP/DOWN — громкость,
    // SELECT+LEFT/RIGHT — порядок цветов кадра (см. блок ниже).
    int step = 0;
    if (!g_selHeld) {
      step = menuNavStep();
      if (step == 0) step = menuPageStep();
    }
    if (step != 0 && g_romCount > 0) {
      int sel = g_romSelected + step;
      if (sel < 0) sel = 0;
      if (sel > g_romCount - 1) sel = g_romCount - 1;
      if (sel != g_romSelected) {
        g_romSelected = sel;
        menuSoundBlip(step < 0);        // щелчок на каждое перемещение по списку
        if (g_romSelected < g_romScroll) g_romScroll = g_romSelected;
        if (g_romSelected >= g_romScroll + MENU_ROWS) {
          g_romScroll = g_romSelected - MENU_ROWS + 1;
        }
        drawMenu();
      }
    }

    // A или START — запуск выбранного рома
    if (g_romCount > 0 && (g_pad & (PAD_A | PAD_START))) {
      char title[ROM_NAME_LEN];
      memcpy(title, g_romName[g_romSelected], sizeof(title));
      DENDY_LOG("[MENU] запуск: %s\n", title);
      menuSoundLaunch();                // «пик-пик-дзынь»: слышно, что запуск принят
      serviceMenuSound();
      showMessage(title, "loading...");

      NesCore::unloadRom();
      if (loadRomFromFile(g_romPath[g_romSelected], g_romSize[g_romSelected])) {
        // ждём отпускания кнопки, иначе она «провалится» в игру
        const uint32_t t0 = millis();
        while ((g_pad & (PAD_A | PAD_START)) && (millis() - t0) < 500) {
          pollButtons();
          serviceMenuSound();           // джингл не должен обрываться
          delay(5);
        }
        NesCore::setPad(0, 0);
        runGame();
      } else {
        showMessage("Load error", g_romName[g_romSelected]);
        delay(1500);
      }
      drawMenu();
    }

    delay(10);
  }
}

static void runGame() {
  g_menuRequest  = false;
  g_resetRequest = false;
  g_osdUntilMs   = 0;
  g_osdBrightMode = false;      // OSD яркости из меню в игру не «переезжает»
  g_frameCounter = 0;

  // Двойная буферизация: три кадра в PSRAM + задача вывода на втором ядре
  // (см. DENDY_FRAME_DOUBLE_BUFFER). Передачу кадра эмулятор больше не ждёт.
  setupFrameOverlap();
  // Кадровые буферы — в чистое состояние ДО первого кадра новой игры: в двух из трёх
  // лежит последний кадр ПРОШЛОЙ игры (NesPpu::reset чистит только тот, куда PPU пишет
  // сейчас), а новый ром первые кадры обычно держит рендер выключенным — без этой
  // чистки на экране мелькала предыдущая игра (см. invalidateFrameBuffers).
  invalidateFrameBuffers();

  NesCore::flushAudio();                // убираем остатки джингла меню из кольца
  NesCore::resetAudioStream();          // и состояние регулятора темпа (см. runMenu)
  audioFlushDma();                      // выталкиваем уже отданный в DMA звук меню
  menuSoundReset();                     // очередь «бипера» меню — чистая
  // Разрешение ПЕРВОГО звука после включения: «прогрев» тракта
  // (AUDIO_START_GUARD_MS) плюс «тихое окно» после загрузочных всплесков
  // (AUDIO_FIRST_SOUND_QUIET_MS). Сюда попадает и запуск единственного рома прямо
  // из setup() (ROM_AUTORUN_SINGLE): если окна ещё не кончились, дожидаемся их,
  // иначе первая же нота музыки придёт в неустоявшийся кодек или в «хвост» просадки
  // питания (щелчок). При обычном запуске из меню первый звук давно сыгран, и ждать
  // нечего. Сам лог печатает sndFirstSoundPlayed(); разбор причин — DendyConfig.h,
  // блок «ПЕРВЫЙ ЗВУК ПОСЛЕ ВКЛЮЧЕНИЯ».
  sndStartGuardWait();
  // РЕГИОН: применяем ДО reset() — от него зависят частота кадров, число строк
  // в кадре, тактовая 2A03 и соотношение CPU:PPU, то есть весь тайминг. В режиме
  // AUTO регион берётся из заголовка загруженного рома (см. applyRegionMode).
  applyRegionMode("igra");
  // В ИГРЕ — адаптивный темп вывода: если эмулятор не выдаёт 60 кадров/с, звук
  // растягивается (синхронно с тем, что видно на экране), а не добивается тишиной
  // («песок»). Ставим ДО reset(): он готовит регулятор, но флаг adaptive не трогает.
  NesCore::setAudioAdaptive(true);
  // База адаптивного темпа — ровно 1:1 до ПЕРВОГО измерения: в другом регионе
  // номинальный период кадра иной (16 667 против 20 000 мкс), и «унаследованное»
  // отношение от прошлой игры было бы неверным (до 20 %). Первое измерение придёт
  // через AUDIO_PRODUCER_WIN_FRAMES кадров (~0,27 с), и «подушка» звука
  // (AUDIO_PRIME_SAMPLES) это покрывает.
  NesCore::setProducerRatioQ16(65536);
  NesCore::reset();
  // «Подушка» звука игры: кольцо APU предзаполняется тишиной на AUDIO_PRIME_SAMPLES
  // (~1 кадр, ~17 мс при 60 Гц и ~20 мс при 50). ОБЯЗАТЕЛЬНО после reset() — он
  // чистит кольцо. Без «подушки» задача I2S подходит к пустому кольцу в паузах
  // между кадрами, добивает блок DMA нулями, и это слышно как хрип (см.
  // audioTaskEntry и DendyConfig.h, блок 3). В PAL кадр ДЛИННЕЕ, поэтому размер
  // «подушки» считается по текущему региону (primeSamplesNow).
  NesCore::primeAudio(primeSamplesNow());
  clearFrameBorders();                  // бока экрана — фоном (там были «хвосты» меню)
  // --- Планировщик кадров NES -------------------------------------------------
  // frameAccUs — «долг» эмуляции по стен-часам (мкс): сколько времени кадров NES
  // уже «наступило». lastTickUs — момент последней проверки. nextFrameUs — период
  // ТЕКУЩЕГО (следующего) кадра: при 60 Гц он не целый (16 666,67 мкс), поэтому
  // очередную границу берём из nextFramePeriodUs(). fpsPushes считает кадры,
  // которые реально дошли до экрана (их не больше, чем эмулированных: SPI-дисплей
  // может не успевать).
  uint32_t frameAccUs  = 0;
  uint32_t lastTickUs  = micros();
  uint32_t nextFrameUs = nextFramePeriodUs();
  // Предел «долга» — DENDY_MAX_FRAME_SKIP кадров, но НЕ МЕНЬШЕ одного периода кадра.
  // Округление ВВЕРХ тут обязательно: при NTSC период равен 50 000/3 = 16 666,67 мкс,
  // и деление вниз дало бы 16 666 — на 1 мкс МЕНЬШЕ границы кадра. Тогда при
  // DENDY_MAX_FRAME_SKIP = 1 условие цикла `frameAccUs >= nextFrameUs` не выполнялось
  // бы НИКОГДА (16 666 < 16 667) и эмулятор встал бы, не эмулируя ни одного кадра
  // (защита от такой правки — static_assert в DendyConfig.h). Период берём из
  // ТЕКУЩЕГО региона (см. applyRegionMode): в PAL это 20 000/1.
  const uint32_t accMaxUs = (uint32_t)(((uint64_t)g_frameUsNum
                                       * DENDY_MAX_FRAME_SKIP + g_frameUsDen - 1)
                                      / g_frameUsDen);
  uint32_t fpsStart   = millis();
  uint32_t fpsFrames  = 0;                 // кадров эмулировано за окно
  uint32_t fpsPushes  = 0;                 // из них выведено на экран
  // --- ИЗМЕРЕНИЕ ТЕМПА ЭМУЛЯТОРА (база адаптивного темпа звука) ----------------
  // prodWinUs/prodWinFrames копят стен-часы и кадры между публикациями, prodLastUs
  // — момент ПРОШЛОГО прохода, в котором эмулировались кадры. Смысл: отдать в APU
  // ФАКТИЧЕСКИЙ период кадра (мкс) — непрерывную величину. Прежняя оценка темпа
  // «по кольцу» квантована кадровыми пачками APU (735 сэмплов раз в кадр) и гуляла
  // на ±9 % за окно, отчего тон «расплывался» (разбор — DendyConfig.h,
  // AUDIO_PRODUCER_WIN_FRAMES); здесь та же ошибка делится на число кадров в окне.
  uint32_t prodWinUs     = 0;
  uint32_t prodWinFrames = 0;
  uint32_t prodLastUs    = micros();
  // Первое окно — вдвое короче: его результат нужен как можно раньше (до него
  // база шага = 1:1, и кольцо медленно «худеет»), а точности 8 кадров хватает с
  // запасом: оценка про ВРЕМЯ, а не про сэмплы (см. AUDIO_PRODUCER_WIN_FRAMES).
  uint32_t prodWinNeed   = AUDIO_PRODUCER_WIN_FRAMES / 2;
  // Счётчик 5-секундных окон статистики. Нужен, чтобы ПОДРОБНЫЙ блок [GAME]
  // (sbor / подсказка / profil / kesh bankov / vremya) печатался не каждое окно:
  // вместе это ~2,5 КБ в UART, то есть ~220 мс печати на 115200 бод, и всё это
  // время задача эмулятора ЖДЁТ освобождения буфера Serial — то есть печать прямо
  // отбирается у кадра. Печатаем его во ВТОРОМ окне (в первом идёт загрузка рома
  // и прогрев кэшей, числа там «нечестные») и дальше раз в 30 с. Главные числа
  // видны каждое окно и без него: строка `[GAME] fps emu / na ekran / emu ms` и
  // строка `[SND]` (в ней же `temp`/`golod` — состояние адаптивного темпа).
  uint32_t verboseWin = 0;
#if DENDY_VIDEO_TRACE
  uint32_t vtStart    = millis();          // окно диагностики видеотракта (1 с)
  uint32_t vtFrames   = 0;
  uint32_t vtEmuUs    = 0;                 // суммарное время эмуляции кадра
  uint32_t vtPushUs   = 0;                 // суммарное время передачи кадра
  bool     vtSelfTest = false;             // тест-полосы уже нарисованы
#endif

  for (;;) {
    pollButtons();
    updateButtonLogic();

    if (g_resetRequest) {                 // короткое MENU — Reset игры
      g_resetRequest = false;
      NesCore::reset();
      NesCore::primeAudio(primeSamplesNow());   // вернуть «подушку» звука (reset её снял)
      DENDY_LOG("[GAME] reset\n");
    }
    if (g_menuRequest) {                  // удержание MENU — выход в меню
      g_menuRequest   = false;
      g_menuPressMs   = 0;
      g_menuHoldFired = true;             // отпускание не даст лишний Reset
      break;
    }

    handleVolumeKeys();
    handleBrightKey();                  // кнопка «ЯРКОСТЬ» (вывод 8): шаг по кругу
    backlightGlideStep();               // доводим яркость плавно (только ШИМ, без SPI)
    // SELECT+LEFT/RIGHT — подбор порядка каналов цвета кадра (0..3, см. 7.1).
    // Кадр начнёт приходить в новых цветах сразу; выбранный режим показываем OSD.
    if (handleColorOrderKeys()) {
      g_osdColorMode  = true;
      g_osdBrightMode = false;
      g_osdUntilMs    = millis() + OSD_TIMEOUT_MS;
    }

    // --- Сколько кадров NES «уже наступило» по стен-часам ---------------------
    const uint32_t nowUs = micros();
    frameAccUs += (uint32_t)(nowUs - lastTickUs);
    lastTickUs  = nowUs;
    if (frameAccUs > accMaxUs) frameAccUs = accMaxUs;   // отстали — долг не копим

    // --- Эмуляция РОВНО 60,000 кадра в секунду --------------------------------
    // Догоняем все кадры, время которых уже наступило (не больше
    // DENDY_MAX_FRAME_SKIP, чтобы не копить долг). Звук зависит именно от этого:
    // APU наполняет кольцо I2S по эмулированному времени, а не по числу
    // переданных на экран кадров. В норме (эмулятор успевает) на один проход
    // цикла приходится РОВНО ОДИН кадр: 60 проходов и 60 обновлений картинки в
    // секунду. Пачка из нескольких кадров получается только если эмулятор реально
    // отстал, то есть когда его надо догонять.
    //
    // Ограничивать «пачку» временем передачи кадра (DENDY_FRAME_BURST_BY_PUSH = 1)
    // при перекрытом выводе НЕ нужно: передача идёт на втором ядре, цикл её не
    // ждёт, а длинная пачка только увеличивает задержку ввода. По умолчанию 0.
    const uint32_t burstUs = DENDY_FRAME_BURST_BY_PUSH
                           ? (g_lastPushUs ? g_lastPushUs
                                           : (uint32_t)((g_frameUsNum + g_frameUsDen / 2)
                                                        / g_frameUsDen))
                           : 0xFFFFFFFFu;
    uint32_t emulated   = 0;
    const uint32_t passStartUs = nowUs;
    const uint32_t burstStart  = micros();
    while (frameAccUs >= nextFrameUs && emulated < DENDY_MAX_FRAME_SKIP) {
      // Кнопки читаем перед КАЖДЫМ кадром: в одном проходе цикла кадров может
      // быть несколько (пропуск кадров), а нажатие должно попасть в свой кадр,
      // а не ждать следующего прохода (иначе задержка ввода до 4 кадров).
      const uint32_t btnStart = micros();
      pollButtons();
      updateButtonLogic();
      uint8_t pad = (uint8_t)(g_pad | turboPad());
      if (g_selHeld) pad &= (uint8_t)~(PAD_UP | PAD_DOWN | PAD_LEFT | PAD_RIGHT);  // SELECT: громкость/цвета
      NesCore::setPad(pad, 0);
      g_tBtnUs += (uint32_t)(micros() - btnStart);
      NesCore::runFrame();                      // один кадр NES
      frameAccUs -= nextFrameUs;
      nextFrameUs = nextFramePeriodUs();        // следующая граница (дробь копится)
      ++emulated;
      if (DENDY_FRAME_BURST_BY_PUSH &&
          (uint32_t)(micros() - burstStart) >= burstUs) break;   // передача не дешевле
    }
    // Чистое время эмуляции (без передачи кадра) — по нему видно, какая скорость
    // эмулятора в мс на кадр: 16,7 мс = 60 кадров/с, 33 мс = ровно вдвое медленнее.
    const uint32_t emuThisUs = (uint32_t)(micros() - burstStart);
#if DENDY_VIDEO_TRACE
    const uint32_t vtEmuThis = emuThisUs;
#endif

    if (emulated) {
      // На экран — последний готовый кадр. При DENDY_FRAME_DOUBLE_BUFFER передачу
      // ведёт задача на втором ядре, поэтому эмулятор её НЕ ждёт (см. раздел 7.0).
      pushFrameOverlap();
      g_frameCounter += emulated;
      fpsFrames      += emulated;
      g_emuSumUs     += emuThisUs;
      g_tEmuUs       += emuThisUs;
      g_emuCount     += emulated;
      // --- Темп продюсера для звука: средний период кадра ---------------------
      // Интервал между проходами, в которых ЭМУЛИРОВАЛИСЬ кадры, делим на их
      // число — получаем фактический период кадра в микросекундах. Ошибка такой
      // оценки падает как 1/число кадров (16 кадров ≈ 0,1 %, секунда ≈ 0,01 %), а
      // кадровые пачки APU ей не мешают ВООБЩЕ: она про ВРЕМЯ, а не про сэмплы.
      // Публикуем отношение «номинал/факт» (дробью NUM/DEN, без округления
      // периода до мкс) — APU берёт его базой шага чтения кольца: см.
      // NesApu::setProducerRatioQ16 и DendyConfig.h (AUDIO_PRODUCER_WIN_FRAMES).
      prodWinUs += (uint32_t)(nowUs - prodLastUs);
      prodLastUs = nowUs;
      prodWinFrames += emulated;
      if (prodWinFrames >= prodWinNeed && prodWinUs) {
        const uint64_t ratio = (((uint64_t)g_frameUsNum << 16) * prodWinFrames)
                               / ((uint64_t)g_frameUsDen * prodWinUs);
        NesCore::setProducerRatioQ16((uint32_t)ratio);
        prodWinUs     = 0;
        prodWinFrames = 0;
        prodWinNeed   = AUDIO_PRODUCER_WIN_FRAMES;   // дальше — полные окна
      }
      // Худший кадр окна (мкс на кадр, см. g_emuMaxUs): отделяет «эмулятор
      // равномерно не успевает» от «кадру мешают всплески».
      const uint32_t emuPerFrame = emuThisUs / emulated;
      if (emuPerFrame > g_emuMaxUs) g_emuMaxUs = emuPerFrame;
      ++fpsPushes;                              // кадров реально выведено на LCD
#if DENDY_VIDEO_TRACE
      vtEmuUs  += vtEmuThis;                                    // время эмуляции
      // Время передачи кадра берём из последнего измерения (pushFrame() при
      // синхронном выводе или задача pushTaskEntry при двойной буферизации):
      // при перекрытии цикл игры передачи не ждёт, и по его часам она = 0.
      vtPushUs += g_lastPushUs;
      vtFrames += emulated;
      if (!vtSelfTest) {                    // тест-полосы: 4 разных пути вывода кадра
        vtSelfTest = true;
        drawVideoSelfTest();
      }
#endif
    } else {
      // Эмуляция успевает за реальным временем: ждём, пока «наступит» следующий
      // кадр NES — именно это держит РОВНО 60,000 Гц (иначе игра ускоряется, а
      // APU отдаёт слишком много сэмплов и кольцо I2S переполняется). Граница
      // прошедшего кадра = nowUs - frameAccUs, период следующего = nextFrameUs.
      const uint32_t waitStart = micros();
      frameLimiter((uint32_t)(nowUs - frameAccUs), nextFrameUs);
      g_tWaitUs += (uint32_t)(micros() - waitStart);
    }
#if DENDY_VIDEO_TRACE
    if (millis() - vtStart >= 1000) {     // раз в секунду — состояние видеотракта
      logVideoDiag(vtEmuUs, vtPushUs, vtFrames);
      logSoundDiag("игра");                // звук игры (при трассе 0 — блок ниже)
      vtStart  = millis();
      vtFrames = 0;
      vtEmuUs  = 0;
      vtPushUs = 0;
    }
#endif

    // OSD (полоса громкости / режим цвета) рисует задача вывода кадра сразу
    // после кадра — см. osdDrawIfActive(). Здесь только снимаем флаг, когда
    // время показа вышло: игровой цикл SPI не трогает и не ждёт передачу кадра.
#if DENDY_DEBUG_SERIAL && !DENDY_VIDEO_TRACE
    // --- Строка [SND] ИГРЫ без видеотрассы ------------------------------------
    // При DENDY_VIDEO_TRACE = 1 её печатает блок выше (вместе со строками [VID]).
    // При 0 (обычная игра — трасса отнимает до 45 % времени кадра) печатаем её
    // здесь: это одно сообщение про звук, и в нём ГЛАВНОЕ — `kolco APU` (пустеет
    // ли кольцо), `tishina`/`dyra` (сколько тишины вставлено в музыку), `temp`/
    // `golod` (насколько растянут звук) и `sempl/s` (темп эмуляции). Это тот же
    // самый тракт, что проверяет джингл меню (см. audioTaskEntry/logSoundDiag).
    // ПОЧЕМУ 5 С, А НЕ 1 С: строка ~470 символов держит UART ~40 мс, а пока буфер
    // драйвера полон, задача эмулятора ждёт — раз в секунду это стоило ~4 % канала
    // UART и столько же времени у кадра. Все счётчики в строке НАКОПИТЕЛЬНЫЕ, так
    // что на большем окне она только честнее (а `sempl/s` считается по разнице
    // счётчика между вызовами — от периода не зависит).
    static uint32_t sndLogMs   = 0;
    static uint32_t silentPrev = 0;
    if (millis() - sndLogMs >= 5000) {
      logSoundDiag("игра");
      // Растут блоки тишины — значит APU не успевает наполнять кольцо. Это НЕ
      // лечится размерами буферов (см. три уровня буферизации в DendyConfig.h,
      // блок 3 «ЗВУК»): при 44 кадрах/с APU отдаёт 44*735 = 32 340 сэмплов в
      // секунду вместо 44 100, и задача I2S обязана добить разницу тишиной.
      // Ищите причину в `fps emu` (строки [GAME] fps emu / profil / vremya).
      const uint32_t dSilent = g_i2sSilent - silentPrev;
      silentPrev = g_i2sSilent;
      if (sndLogMs && dSilent > 20) {
        DENDY_LOG("[SND] VNIMANIE: kolco APU pusto (%u blokov tishiny za sekundu) | "
                  "temp %d (1000 = 1:1), golod %u. V igre tishiny byt ne dolzhno: "
                  "esli `golod` rastet — emulyator medlennee ~30 kadrov/s (sm. [GAME] fps "
                  "emu/profil/vremya) ili AUDIO_ADAPTIVE_RATE = 0 v DendyConfig.h; "
                  "razmery buferov eto NE lechat\n",
                  (unsigned)dSilent, NesCore::audioRatePerMille(),
                  (unsigned)NesCore::audioStarved());
      }
      // --- РАСТЯНУТ ЛИ ЗВУК ПО ТЕМПУ (`temp` в строке [SND]) --------------------
      // `temp` — шаг чтения кольца в промилле: 1000 = 1:1. Меньше — эмулятор идёт
      // медленнее 60,000 (50,000) кадров/с, и звук тянется ВСЛЕД за кадрами (так и
      // задумано: AUDIO_ADAPTIVE_RATE, иначе в музыке был бы «песок»). На слух это
      // именно «звуки чуть длиннее, чем на приставке» — и никакая правка APU такого
      // не уберёт, потому что длительность самих эффектов считают КАДРЫ:
      //   * Battle City: движок в NMI уменьшает на 1 за кадр счётчик длительности
      //     эффекта из своей таблицы (см. разбор $EA7E-$EB41 в README);
      //   * Super Mario Bros.: джингл смерти играет музыкальный движок, тоже по кадрам.
      // Значит, при `temp` заметно ниже 1000 ищите причину в скорости эмуляции
      // (`fps emu`, `emu ms/kadr`, `hudshij`, печать в Serial — [GAME]).
      // ЕСЛИ `temp` ≈ 1000, а на слух всё равно «тянется» — проверьте регион: в режиме
      // PAL игра честно идёт в 1,2 раза медленнее (50 кадров/с вместо 60), и звуки
      // вместе с ней (в меню это видно как «TV auto(PAL)»).
      const int temp = NesCore::audioRatePerMille();
      if (sndLogMs && temp < 990) {
        DENDY_LOG("[SND] VNIMANIE: zvuk rastyanut na %.1f%% (temp %d vmesto 1000): "
                  "emulyator idet medlennee nominala, sm. [GAME] fps emu. Dlitelnost "
                  "effektov schitaetsya kadrami NMI, poetomu oni slyshny dlinnee\n",
                  (1000.0f - (float)temp) / 10.0f, temp);
      }
      sndLogMs = millis();
    }
#endif
    if (g_osdUntilMs != 0 && (int32_t)(millis() - g_osdUntilMs) >= 0) {
      g_osdUntilMs    = 0;
      g_osdColorMode  = false;
      g_osdBrightMode = false;
    }

#if DENDY_DEBUG_SERIAL
    // logT0 — чтобы в строке [GAME] vremya было видно, сколько времени съедает
    // САМ лог: на 115200 бод длинная строка это десятки миллисекунд, и это время
    // прямо отбирается у эмуляции (поэтому для честных 60 держите DENDY_DEBUG_SERIAL = 0).
    const uint32_t logT0 = micros();
    if (millis() - fpsStart >= 5000) {    // статистика раз в 5 с
      const uint32_t dtMs = (uint32_t)(millis() - fpsStart);
      const float    k    = dtMs ? (1000.0f / (float)dtMs) : 0.0f;
      // fps emu   — с какой скоростью идёт эмуляция: должно быть РОВНО 60,0 (в
      //             PAL-режиме — ровно 50,0). Меньше и «некруглое» — эмулятор не
      //             успевает: смотрите [GAME] sbor (сборка/CPU) и [GAME] vremya;
      // na ekran  — сколько кадров реально успевает принять SPI-дисплей;
      // peredacha — сколько миллисекунд занимает отправка кадра (122 КБ по SPI).
      // na ekran  — кадры, которые РЕАЛЬНО ушли в SPI (g_pushCount: счётчик задачи
      //             вывода, растёт только при настоящей передаче, а не при отдаче);
      // otdano    — кадры, которые отдал игровой цикл (fpsPushes). Если `otdano`
      //             больше `na ekran`, экран не успевает принимать (см. `peredacha`):
      //             игра идёт с нормальной скоростью, а картинка обновляется реже;
      // peredacha — сколько миллисекунд занимает отправка кадра (122 КБ по SPI).
      // В норме (DENDY_MAX_FRAME_SKIP = 2 и 80 МГц) все три числа совпадают;
      // `na ekran` может быть на 1–2 меньше — это догон пропущенного кадра после
      // всплеска (долг по стен-часам больше не «прощается», см. DendyConfig.h).
      DENDY_LOG("[GAME] fps emu ~ %.1f, na ekran ~ %.1f (otdano %.1f), kadrov %u, "
                "gromkost %u%% | emu %.1f ms/kadr (hudshij %.1f), peredacha %.1f ms "
                "| zvuk: koltso APU %d/%d, I2S blokov %u (tishina %u, oshek %u, "
                "blok min/max %u/%u), sempl/s %u iz %d, hf %d%%, rms %d%%, dyra %u, "
                "clip %u, drob %u\n",
                fpsFrames * k, (float)g_pushCount * k, fpsPushes * k,
                (unsigned)g_frameCounter, (unsigned)g_volume,
                g_emuCount ? (float)g_emuSumUs / g_emuCount / 1000.0f : 0.0f,
                g_emuMaxUs / 1000.0f,
                g_pushCount ? (float)g_pushSumUs / g_pushCount / 1000.0f : 0.0f,
                NesCore::audioFill(), (int)NesApu::RING_SIZE,
                (unsigned)g_i2sBlocks, (unsigned)g_i2sSilent, (unsigned)g_i2sTimeouts,
                (unsigned)g_i2sBlkMin, (unsigned)g_i2sBlkMax,
                (unsigned)audioRatePerSec(0), (int)AUDIO_SAMPLE_RATE,
                g_audLastHf, g_audLastRms, (unsigned)g_audLastHole,
                (unsigned)g_audLastClip, (unsigned)NesCore::audioDrops());
#if DENDY_EMU_PROFILE
      // --- Профиль снимаем КАЖДОЕ окно (5 с), а печатаем — в подробном блоке ---
      // ВАЖНО: profileTake() ОБНУЛЯЕТ счётчики тактов. Раньше он вызывался вместе
      // с подробной печатью (то есть раз в 30 с), а делился на число кадров ОДНОГО
      // окна (5 с) — поэтому `profil` показывал в 2..6 раз больше, чем есть на
      // самом деле (например `vsego 42.74 ms/kadr` при `emu 23.7` в строке выше).
      // Теперь снимаем вместе с обнулением g_emuCount (ниже) — числа честные.
      // Заодно считаем по разнице счётчиков ядра: сколько ИНСТРУКЦИЙ 6502
      // выполняется за кадр и сколько раз процессор вошёл в IRQ. По ним сразу
      // видно, `cpu` — это интерпретатор или что-то забирает у него время
      // (нормальный интерпретатор — 20..40 тактов на инструкцию).
      static uint32_t sProfCpu = 0, sProfPpu = 0, sProfApu = 0;
      static uint32_t sProfRender = 0, sProfBg = 0, sProfSpr = 0;
      static uint32_t sProfInstr = 0, sProfIrq = 0;
      static uint32_t sProfInstrPrev = 0, sProfIrqPrev = 0;
      {
        NesCore::profileTake(&sProfCpu, &sProfPpu, &sProfApu, &sProfRender,
                             &sProfBg, &sProfSpr);
        const uint32_t instr = NesCore::cpuInstrCount();
        const uint32_t irq   = NesCore::cpuIrqCount();
        sProfInstr = instr - sProfInstrPrev;
        sProfIrq   = irq   - sProfIrqPrev;
        sProfInstrPrev = instr;
        sProfIrqPrev   = irq;
      }
#endif
      // Игра идёт медленнее приставки? Перечисляем, что проверить (в порядке
      // влияния): сборка (-Og при Debug Level), частота CPU, IRAM и перекрытие
      // передачи кадра. Строка печатается только когда эмулятор не успевает за
      // 60 кадрами/с (это и есть «1 секунда игры = 2 секунды реальности»).
      // ВАЖНО: «медленно из-за сборки» и «PAL» — разные вещи. Регион приставки
      // виден в строке [SYS] region: PAL даёт в ЭТОМ ЖЕ поле `fps emu` РОВНО 50,0
      // (игра идёт медленнее НАМЕРЕННО — так было на европейской приставке, у неё
      // 312 строк в кадре вместо 262, а VBlank реже). Любое «некруглое» число
      // (38, 42, 55) — это не успевающий эмулятор, то есть регион ни при чём.
      const float fpsEmu = fpsFrames * k;
      // Подробный блок — во 2-м окне и дальше раз в 30 с (см. verboseWin выше).
      if (++verboseWin == 2 || (verboseWin % 6) == 0) {
      // Порог «эмулятор не успевает» зависит от РЕГИОНА: 60 Гц — это 55 кадров/с,
      // а в PAL штатные 50 кадров/с не должны выглядеть как тормоза (см. режим TV).
      const float fpsWarn = NesCore::regionIsPal() ? 45.0f : 55.0f;
      if (fpsEmu < fpsWarn) {
        // Сборку печатаем ПЕРЕД подсказкой: по ней сразу видно, надо ли вообще
        // лезть в код (skech -Og / CPU 160 MHz — причина уже в самой строке).
        DENDY_LOG("[GAME] sbor: skech -%s, yadro -%s | CPU %u MHz | v SRAM kadrov "
                  "%d iz %d | region %s | skip %u, burst %u\n",
                  buildOptName(), coreOptName(), (unsigned)getCpuFrequencyMhz(),
                  (int)NesCore::frameBuffersInternal(), (int)DENDY_FB_COUNT,
                  NesCore::regionName(), (unsigned)DENDY_MAX_FRAME_SKIP,
                  (unsigned)DENDY_FRAME_BURST_BY_PUSH);
        DENDY_LOG("[GAME] medlennee pristavki (%.1f kadr/s): 1) Tools -> Debug Level = "
                  "None (s -Og byvaet rovno vdvoe medlennee), 2) Tools -> CPU Frequency = "
                  "240 MHz, 3) DENDY_CORE_IRAM = 1 v DendyConfig.h (+30..100%% skorosti), "
                  "4) DENDY_FRAME_DOUBLE_BUFFER = 1 (perekrytie peredachi kadra), "
                  "5) DENDY_DEBUG_SERIAL = 0 i DENDY_VIDEO_TRACE = 0 (pechat na 115200 "
                  "bod sedaet do 45%% kadra). "
                  "Esli `fps emu` rovno 50.0 — eto rezhim TV = PAL (tak i dolzhno byt dlya "
                  "evropejskih romov), a ne tormozhenie; perekljuchaetsya knopkoj SX_PIN_REGION. "
                  "Stroki 'profil' i 'vremya' nizhe pokazhut, chto imenno tormozit: "
                  "'profil' — vremya VNUTRI runFrame (cpu/ppu/apu), 'vremya' — ves "
                  "prohod cikla (emu/knopki/pauza/log/prochee)\n",
                  fpsEmu);
#if !DENDY_EMU_PROFILE
        // Без этого пояснения `profil` в логе просто ОТСУТСТВУЕТ, и кажется, что
        // подсказка ссылается на несуществующую строку (так и было: искали
        // `profil`, а его печатает только сборка с DENDY_EMU_PROFILE = 1).
        DENDY_LOG("[GAME] podskazka: stroki 'profil' net, potomu chto DENDY_EMU_PROFILE = 0 "
                  "v DendyConfig.h — postavte 1, chtoby uvidet vremya cpu/ppu/render/apu\n");
#endif
      }
#if DENDY_EMU_PROFILE
      // --- Профиль: на что уходит время кадра ---------------------------------
      // Значения сняты (и счётчики обнулены) в начале ЭТОГО окна — см. выше, сразу
      // после строки [GAME]. Такты -> миллисекунды (делим на частоту ядра) и на
      // число кадров окна. Сразу видно, что оптимизировать: интерпретатор 6502
      // (cpu), рендер PPU (ppu/render: fon/sprajty) или звук (apu).
      // Поля `instrukcij`/`IRQ` — за кадр: по ним видно, честный ли большой `cpu`
      // (нормальный интерпретатор — 20..40 тактов на инструкцию 6502).
      {
        const uint32_t mhz = (uint32_t)getCpuFrequencyMhz();
        // Такты -> мс на кадр: /частоту (МГц) даёт микросекунды, /1000 — мс,
        // и всё это делится на число кадров за прошедшее окно.
        const float    f   = (mhz && g_emuCount)
                           ? (1.0f / ((float)mhz * 1000.0f * (float)g_emuCount))
                           : 0.0f;
        const float    kfr = g_emuCount ? (1.0f / (float)g_emuCount) : 0.0f;
        DENDY_LOG("[GAME] profil: cpu %.2f ms, ppu %.2f ms (iz nih render %.2f: fon "
                  "%.2f, sprajty %.2f), apu %.2f ms | vsego %.2f ms/kadr (%u kadr, "
                  "instrukcij %.0f, IRQ %.1f na kadr)\n",
                  sProfCpu * f, sProfPpu * f, sProfRender * f, sProfBg * f,
                  sProfSpr * f, sProfApu * f, (sProfCpu + sProfPpu + sProfApu) * f,
                  (unsigned)g_emuCount, (double)((float)sProfInstr * kfr),
                  (double)((float)sProfIrq * kfr));
      }
#if DENDY_CART_IRAM
      // Сколько окон банков пришлось скопировать в SRAM за окно. Если это число
      // сравнимо с числом кадров или больше (игра меняет банки на каждой строке),
      // кэш может быть невыгоден — тогда быстрее DENDY_CART_IRAM = 0.
      const uint32_t copies = NesCore::mapperCacheCopies();   // забрать и обнулить
      DENDY_LOG("[GAME] kesh bankov: kopij %u, v srednem %.1f na kadr (0 — banki "
                "ne menyalis, eto normalno dlya NROM i CNROM)\n",
                (unsigned)copies,
                g_emuCount ? (float)copies / (float)g_emuCount : 0.0f);
#endif
#endif
      // --- Куда уходит игровой цикл (стен-часы), а не только эмулятор --------
      // Строка `profil` выше показывает время ВНУТРИ runFrame (cpu/ppu/apu).
      // Эта строка показывает ВСЁ время цикла — и если `emu` маленькое, а
      // `fps emu` меньше 60, то сразу видно, где теряется остальное:
      //   emu     — эмуляция кадров (то же число, что `emu` строкой выше);
      //   knopki  — опрос кнопок по I2C + логика кнопок (при исправном INT это
      //             доли миллисекунды; большое значение и `I2C chtenij` сотнями
      //             в секунду = провод INT не работает или залип);
      //   pauza   — ожидание границы кадра. Это НОРМА: именно она держит 60 Гц;
      //   log     — печать в Serial из игрового цикла (длинная строка ~200
      //             символов = ~17 мс на 115200 бод);
      //   prochee — всё остальное. Если оно сравнимо с `emu`, время отбирают
      //             задачи (I2S, задача вывода кадра) и прерывания, а не код игры.
      {
        const uint32_t accounted = g_tEmuUs + g_tBtnUs + g_tWaitUs + g_tLogUs;
        const uint32_t other     = (g_tPassUs > accounted) ? (g_tPassUs - accounted) : 0;
        const uint32_t perPass   = g_tPasses ? (other / g_tPasses) : 0;
        // kadrUs — сколько МИКРОСЕКУНД стен-часов уходит на один кадр NES (время
        // всего прохода цикла, делённое на число эмулированных кадров). 16 667 мкс
        // = ровно 60 кадров/с; 24 000 мкс = игра идёт на 0,7 от скорости приставки.
        // Раньше это поле печаталось как «ms», хотя значение всегда было в мкс.
        const float    kadrUs    = g_emuCount ? (float)g_tPassUs / (float)g_emuCount
                                              : 0.0f;
        DENDY_LOG("[GAME] vremya %u ms: emu %u, knopki %u, pauza %u, log %u, "
                  "prochee %u ms | prohodov %u, na kadr %.0f mks, prochee %.2f ms/prohod, "
                  "I2C chtenij %u\n",
                  (unsigned)dtMs,
                  (unsigned)(g_tEmuUs / 1000), (unsigned)(g_tBtnUs / 1000),
                  (unsigned)(g_tWaitUs / 1000), (unsigned)(g_tLogUs / 1000),
                  (unsigned)(other / 1000), (unsigned)g_tPasses, kadrUs,
                  (double)(perPass / 1000.0), (unsigned)g_btnReadCount);
        if (perPass >= 2000) {           // 2 мс на проход — это уже 12 % кадра
          DENDY_LOG("[GAME] vremya: `prochee` bolshe 2 ms na prohod — eto NE "
                    "emulyator, a vremya, kotoroe zabirajut zadachi (I2S, vyvod kadra) "
                    "i prekljuchenija. Smotrite [VID] `peredacha` i [SND]; esli prochee "
                    "bolshoj dazhe bez strok [GAME] — uvelichte okno statistiki\n");
        }
      }
      }   // конец «подробный блок» (verboseWin: 2-е окно и далее раз в 30 с)
      fpsStart  = millis();
      fpsFrames = 0;
      fpsPushes = 0;
      g_pushSumUs = 0;
      g_pushCount = 0;
      g_emuSumUs  = 0;
      g_emuCount  = 0;
      g_emuMaxUs  = 0;
      g_tEmuUs    = 0;
      g_tBtnUs    = 0;
      g_tWaitUs   = 0;
      g_tLogUs    = 0;
      g_tPassUs   = 0;
      g_tPasses   = 0;
      g_btnReadCount = 0;
    }
    // Время печати ЭТОГО окна попадает в бюджет СЛЕДУЮЩЕГО (часы сняты после
    // печати, а счётчики обнулены выше). За много окон это усредняется, зато
    // `log` в строке vremya — честное время UART, а не ноль.
    g_tLogUs += (uint32_t)(micros() - logT0);
#endif

    // --- Сколько занял весь проход игрового цикла -----------------------------
    // passStartUs снят в начале прохода (там же, где считали frameAccUs), поэтому
    // сюда входит ВСЁ: кнопки, эмуляция, отдача кадра, пауза и печать. Разница
    // с `emu`+`knopki`+`pauza`+`log` и есть `prochee` в строке [GAME] vremya.
    g_tPassUs += (uint32_t)(micros() - passStartUs);
    ++g_tPasses;
  }

  waitPushIdle();                       // пусть задача вывода отдаст кадр (меню — тот же SPI)
  NesCore::setPad(0, 0);
  NesCore::unloadRom();
  DENDY_LOG("[GAME] выход в меню\n");
}

// =============================================================================
//  12. SETUP / LOOP
// =============================================================================
// Свободная память: внутренняя SRAM (в неё стараемся класть кадры, см. NesCore
// fbAlloc) и PSRAM (образ рома + всё остальное). Печатается на старте и перед
// выделением кадровых буферов — по этим числам видно, влезет ли кадр в SRAM
// и сколько буферов окажется в PSRAM.
static void logMemory(const char* tag) {
  DENDY_LOG("[MEM] %s: SRAM svobodno %u KB (krupnejshij blok %u KB), PSRAM %u KB\n", tag,
            (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024u),
            (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024u),
            (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024u));
}

// Что за сборка и железо — по этой строке сразу видно, ЕСТЬ ли вообще смысл
// искать проблему в коде. Главное:
//   sbor skech — с каким -O собран скетч: при Tools -> Debug Level = Debug это
//                -Og, и всё (кроме ядра, у него -O3 из DendyFast.h) идёт вдвое
//                медленнее — типичная причина «игра идёт как в замедленной съёмке»;
//   CPU        — 240 МГц обязательно (Tools -> CPU Frequency);
//   yadro      — с какой оптимизацией собран горячий код ядра;
//   IRAM       — DENDY_CORE_IRAM (горячий код в IRAM вместо flash-кэша).
static void logSystemInfo() {
  const uint32_t cpuMhz = (uint32_t)getCpuFrequencyMhz();
  DENDY_LOG("[SYS] sbor: skech -%s, yadro -%s | CPU %u MHz, flash %u MHz, PSRAM %u KB\n",
            buildOptName(), coreOptName(), (unsigned)cpuMhz,
            (unsigned)(ESP.getFlashChipSpeed() / 1000000u),
            (unsigned)(ESP.getPsramSize() / 1024u));
  // Регион приставки (DENDY_REGION_PAL в DendyConfig.h) и ЧЕСТНЫЕ числа кадра.
  // ВАЖНО: это регион ПО УМОЛЧАНИЮ — с него прошивка стартует, и его проверяют
  // static_assert'ы. Рабочий режим печатает строка [TV] (см. applyRegionMode): он
  // выбирается в меню кнопкой SX_PIN_REGION — AUTO (по заголовку рома), NTSC или
  // PAL — и больше НЕ требует пересборки прошивки.
  // Это и есть самопроверка «60 кадров в секунду»: частота кадров — главная
  // величина, из неё выведены период кадра и тактовая 2A03 (блок 3.1
  // DendyConfig.h), поэтому печатаем период ДРОБЬЮ, тактов в кадре и «сколько
  // сэмплов APU на кадр» — последнее равно AUDIO_SAMPLE_RATE/FPS (735 при 44 100;
  // 367,50 при 22 050 — дробное значение не «плывёт»: сэмплы считает аккумулятор).
  // PAL-режим даёт РОВНО 50,0 кадра/с в поле `fps emu` строки [GAME]
  // (европейская приставка: 312 строк в кадре, VBlank реже, каждое игровое
  // действие на 16,7 % медленнее). «Некруглое» 38 — это не PAL, а не успевающий
  // эмулятор (сборка/частота CPU) — смотрите строки [GAME] sbor и [GAME] vremya.
  DENDY_LOG("[SYS] region po umolchaniyu (DendyConfig.h): %s | kadr %u/%u us (%.3f kadr/s), "
            "strok %u, tochek %u, CPU 2A03 %.4f MHz, taktov CPU v kadre %.1f, "
            "semplov APU na kadr %.2f | rabochij rezhim sm. v stroke [TV]\n",
            DENDY_REGION_NAME,
            (unsigned)DENDY_FRAME_US_NUM, (unsigned)DENDY_FRAME_US_DEN,
            (float)DENDY_FPS, (unsigned)(DENDY_PPU_PRERENDER + 1),
            (unsigned)DENDY_PPU_DOTS_PER_FRAME, (float)DENDY_CPU_HZ / 1000000.0f,
            (float)DENDY_FRAME_CYCLES,
            (double)((float)AUDIO_SAMPLE_RATE / DENDY_FPS));
  logMemory("start");
#if DENDY_CORE_IRAM
  DENDY_LOG("[SYS] DENDY_CORE_IRAM = 1: goryachij kod yadra rabotaet iz IRAM\n");
#endif
#if DENDY_CART_IRAM
  DENDY_LOG("[SYS] DENDY_CART_IRAM = 1: aktivnye banki PRG/CHR citayutsya iz "
            "vnutrennej SRAM (kopiya 40 KB vmesto PSRAM)\n");
#endif
#if DENDY_FRAME_DOUBLE_BUFFER
  DENDY_LOG("[SYS] DENDY_FRAME_DOUBLE_BUFFER = 1: peredacha kadra idet parallelno "
            "emulyacii (3 bufera kadra)\n");
#endif
#if DENDY_EMU_PROFILE
  DENDY_LOG("[SYS] DENDY_EMU_PROFILE = 1: v stroke [GAME] pechataetsya 'profil' "
            "(cpu/ppu/render/apu v ms) — po nem vidno, chto tormozit. Otkluchit: 0\n");
#endif
  if (cpuMhz < 240) {
    DENDY_LOG("[SYS] VNIMANIE: CPU %u MHz. V Tools -> CPU Frequency vyberite 240 MHz, "
              "inache emulyaciya idet medlennee pristavki\n", (unsigned)cpuMhz);
  }
}

#if DENDY_LCD_TEST
void runLcdTest();                // определена в LcdTest.ino (тест вывода на LCD)
#endif

void setup() {
#if DENDY_DEBUG_SERIAL
  Serial.begin(DENDY_SERIAL_BAUD);
  delay(100);
#endif
  DENDY_LOG("\n=== DendyGO %s (ESP32-S3) ===\n", DENDY_VERSION);
  logSystemInfo();                // сборка (-O!), частота CPU, flash, PSRAM

#if DENDY_LCD_TEST
  runLcdTest();                   // режим диагностики дисплея: эмулятор не грузится
#endif

  setupDisplay();                 // экран первым — на нём видно ошибки
  showMessage("DendyGO " DENDY_VERSION, "init...");

  if (!LittleFS.begin(true)) {    // true = форматировать при сбое монтирования
    DENDY_LOG("[FS] LittleFS не смонтирована\n");
    showMessage("LittleFS error", "check partition scheme");
    delay(2500);
  } else {
    DENDY_LOG("[FS] LittleFS: свободно %u КБ из %u КБ\n",
              (unsigned)((LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024),
              (unsigned)(LittleFS.totalBytes() / 1024));
    logFsPartition();
  }

  sndBootBusyMark("LittleFS");    // флеш смонтирована (или попытка уже кончилась)

  if (!NesCore::init()) {
    DENDY_LOG("[CORE] NesCore::init() не удался\n");
    showMessage("Core init failed", "not enough PSRAM?");
    while (true) delay(1000);
  }

  sndBootBusyMark("jadro");       // NesCore::init: кадровые буферы в PSRAM

  loadVolume();                   // громкость из прошлого запуска
  NesCore::setVolume((uint8_t)((uint16_t)g_volume * 255 / 100));

  // Порядок каналов цвета кадра: печатаем текущий режим и «эталон» неба $22 —
  // по этой строке сразу видно, что уходит на экран. Подбор — SELECT+LEFT/RIGHT
  // (в меню рисуется контрольная полоса «кадр vs эталон», см. раздел 7.1).
  applyColorOrder((uint8_t)DENDY_COLOR_ORDER, "start");

  // Регион ТВ (PAL 50 Гц / NTSC 60 Гц): режим из прошлого запуска + применение к
  // ядру. В режиме AUTO регион уточняется по заголовку уже загруженного рома —
  // это делает applyRegionMode("igra") в runGame(). Здесь важнее другое: ядро
  // (PPU/APU/CPU:PPU) должно быть настроено ДО первого запуска, иначе самый
  // первый ром стартовал бы в регионе по умолчанию из DendyConfig.h.
  loadRegionMode();
  applyRegionMode("start");

  setupAudio();
  // «Прогрев» тракта тишиной перед ПЕРВЫМ звуком (AUDIO_START_GUARD_MS): пока он
  // идёт, в кодек уходит только «пустой звук», а первый звук ждёт (джингл меню —
  // через 6.1, музыка рома — через sndStartGuardWait в runGame). Зовём сразу после
  // setupAudio(): I2S уже тактирует кодек, значит отсчёт тишины идёт с этого места.
  // Дальше каждый тяжёлый шаг загрузки отмечается sndBootBusyMark(): от последнего
  // из них отсчитывается «тихое окно» (AUDIO_FIRST_SOUND_QUIET_MS), поэтому первый
  // звук не совпадёт с «хвостом» просадки питания от всплеска тока. Разбор причин
  // щелчка — DendyConfig.h, блок «ПЕРВЫЙ ЗВУК ПОСЛЕ ВКЛЮЧЕНИЯ».
  sndStartGuardArm();
  setupButtons();
  sndBootBusyMark("knopki");      // SX1509 по I2C: 11 выводов и прерывание

#if DENDY_FRAME_DOUBLE_BUFFER
  // Мьютекс дисплея нужен уже в меню: кадр может отдаваться задачей вывода (ядро 0)
  // одновременно с перерисовкой меню/OSD. Создаём ДО первого drawMenu().
  if (!g_lcdMutex) g_lcdMutex = xSemaphoreCreateMutex();
#endif

  if (g_audioTask == nullptr) {   // задача вывода звука (ядро 1)
    xTaskCreatePinnedToCore(audioTaskEntry, "dendy_audio", AUDIO_TASK_STACK, nullptr,
                            AUDIO_TASK_PRIO, &g_audioTask, AUDIO_TASK_CORE);
  }

  scanRoms();
  DENDY_LOG("[ROM] ромов найдено: %d\n", g_romCount);
  sndBootBusyMark("romy");        // каталог LittleFS прочитан и отсортирован
  if (g_romCount == 0) {
    DENDY_LOG("[ROM] ромов нет: залит ли образ data? (README -> data/LittleFS)\n");
  }

#if ROM_AUTORUN_SINGLE
  if (g_romCount == 1) {          // единственный ром — запускаем сразу
    showMessage(g_romName[0], "loading...");
    g_romSelected = 0;
    if (loadRomFromFile(g_romPath[0], g_romSize[0])) {
      runGame();
    } else {
      showMessage("Load error", g_romName[0]);
      delay(1500);
    }
  }
#endif

  DENDY_LOG("[BOOT] готово\n");
}

void loop() {
  runMenu();
}



