/**
 * =============================================================================
 *  DendyGO / NesCore.cpp
 *  Шина эмулятора ($0000-$FFFF) и публичный API ядра.
 *
 *  Карта памяти:
 *    $0000-$1FFF  2 КБ RAM (4 зеркала)
 *    $2000-$3FFF  регистры PPU (зеркала каждые 8 байт)
 *    $4000-$4013  регистры APU
 *    $4014        DMA спрайтов OAM (+513 тактов CPU)
 *    $4015        статус/разрешения APU
 *    $4016        строб джойстиков (+ чтение)
 *    $4017        frame counter APU / второй джойстик
 *    $6000-$7FFF  WRAM картриджа
 *    $8000-$FFFF  PRG ROM (окна банков задаёт маппер)
 * =============================================================================
 */
#include <string.h>
#include "DendyFast.h"            // -O3 для горячего кода (см. DendyConfig.h)
#include "NesCore.h"
#include "DendyConfig.h"

#if defined(ESP32)
  #include <Arduino.h>
  #include "esp32-hal-psram.h"
  #include "esp_heap_caps.h"
#endif

// ================================ Утилиты ====================================
// Выделение в PSRAM (фреймбуфер и образ рома не помещаются в обычное ОЗУ)
static void* psAlloc(size_t size)
{
#if defined(ESP32)
  void* p = ps_malloc(size);
  if (!p) p = malloc(size);                  // запасной вариант: обычная куча
  return p;
#else
  return malloc(size);
#endif
}

static void psFree(void* ptr)
{
  if (ptr) free(ptr);                        // ps_malloc освобождается обычным free()
}

// ---------------------------- Кадровые буферы --------------------------------
// Кадр 256x240 RGB565 = 122 880 байт. Его стараемся положить во ВНУТРЕННЮЮ SRAM,
// а не в PSRAM. Почему: профиль (DENDY_EMU_PROFILE) показал, что отрисовка одной
// строки стоит ~60 тактов на пиксель — столько дают промахи кэша. Запись кадра
// в PSRAM (120 КБ на кадр) вытесняет кэш, мимо которого CPU и PPU читают PRG/CHR
// картриджа (тоже в PSRAM), и почти каждое обращение к рому становится промахом.
// Во внутренней SRAM кадр вообще не трогает PSRAM. Если внутренней памяти не
// хватает (её занимают драйверы, I2S-DMA, буферы LovyanGFX) — как раньше, PSRAM:
// работает то же самое, но медленнее.
static uint8_t g_fbInternalCount = 0;        // сколько кадров легло в SRAM (для лога)

static void* fbAlloc(size_t size)
{
#if defined(ESP32)
  void* p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (p) { ++g_fbInternalCount; return p; }
#endif
  return psAlloc(size);
}

// ============================== Состояние ====================================
namespace {
  Nes  g_nes;                    // всё состояние эмулятора
  bool g_ready = false;          // буферы в PSRAM выделены
  uint8_t g_irqSource = 0;       // источник последнего принятого IRQ (диагностика)
  // --- APU пачками (DENDY_APU_GRANULARITY) и профиль (DENDY_EMU_PROFILE) -----
  uint32_t g_apuAcc   = 0;       // такты CPU, ещё не отданные APU
  // Остаток перевода тактов CPU в точки PPU: нужен там, где 1 такт CPU — НЕ
  // целое число точек (PAL: 16/5 = 3,2). Копим в единицах 1/DENDY_PPU_PER_CPU
  // точки, иначе строки «уползают». В NTSC (3:1) делитель равен 1, значит
  // аккумулятор всегда нулевой и шаг — ровно 3 точки на такт.
  uint32_t g_ppuAcc   = 0;       // точки PPU в единицах 1/PPU_PER_CPU — см. runFrame()
  // --- РЕГИОН ЭМУЛЯЦИИ (NTSC/PAL): см. NesCore::setRegion ----------------------
  // В горячем пути нужны ровно два числа: коэффициент CPU:PPU (в runFrame) и
  // потолок тактов кадра (защита от незаканчивающегося кадра). Соотношение держим
  // ВЕТВЛЕНИЕМ с константными делителями, а не делением на переменную: на Xtensa
  // деление переменных — вызов библиотеки (десятки тактов), и он стоял бы на
  // КАЖДОЙ инструкции 6502. Ветка предсказуема, поэтому стоит почти ничего.
  bool     g_regionPal   = (DENDY_REGION_PAL != 0);
  uint32_t g_frameCycles = (uint32_t)DENDY_FRAME_CYCLES;
  uint32_t g_profCpu  = 0;       // такты, ушедшие на инструкции 6502
  uint32_t g_profPpu  = 0;       // такты, ушедшие на продвижение PPU (с отрисовкой)
  uint32_t g_profApu  = 0;       // такты, ушедшие на APU
}

// ================================== Шина =====================================
// Тело nesRead()/nesWrite() теперь в NesCore.h (inline): вызывается по 2..6 раз
// на каждую инструкцию 6502, поэтому вызов через границу модуля был слишком
// дорогим. Здесь ничего не определяется — так и надо, иначе будет дубль символа.

// ============================ Публичный API ==================================
namespace NesCore {

bool init()
{
  if (g_ready) return true;

  // Фреймбуфер 256x240 RGB565 = 122 880 байт — сначала внутренняя SRAM (см. fbAlloc)
  g_nes.ppu.frame = (uint16_t*)fbAlloc(NesPpu::WIDTH * NesPpu::HEIGHT * sizeof(uint16_t));
  if (!g_nes.ppu.frame) {
#if DENDY_DEBUG_SERIAL
    Serial.println("[NesCore] Нет PSRAM под фреймбуфер!");
#endif
    return false;
  }
  memset(g_nes.ppu.frame, 0, NesPpu::WIDTH * NesPpu::HEIGHT * sizeof(uint16_t));

  g_nes.apu.reset(&g_nes);
  g_ready = true;
  return true;
}

bool loadRom(uint8_t* data, uint32_t size)
{
  if (!init()) return false;
  if (!data || size < 16 || size > ROM_MAX_SIZE) return false;

  unloadRom();                               // на всякий случай

  Nes& n = g_nes;
  if (!n.mapper.load(data, size)) return false;   // буфер остаётся у вызывающего

  n.romBuffer = data;                        // владение перешло ядру
  n.romSize   = size;
  n.romLoaded = true;                        // важно: до reset(), иначе нет вектора сброса

  reset();

#if DENDY_DEBUG_SERIAL
  Serial.printf("[NesCore] ROM: %u байт, маппер %d (%s)%s\n",
                (unsigned)size, n.mapper.number, n.mapper.mapperName(),
                n.mapper.supported() ? "" : " — НЕ ПОДДЕРЖИВАЕТСЯ");
#endif
  return true;
}

void unloadRom()
{
  Nes& n = g_nes;

  // Сбрасываем окна банков ДО освобождения памяти: они указывали внутрь буфера.
  // Только через clear()! Присваивание NesMapper() создаёт временный объект
  // ~16 КБ на стеке задачи loopTask и валит прошивку (stack canary).
  n.mapper.clear();

  if (n.romBuffer) {
    psFree(n.romBuffer);
    n.romBuffer = nullptr;
  }
  n.romSize   = 0;
  n.romLoaded = false;
}

void reset()
{
  if (!g_ready) return;
  Nes& n = g_nes;

  memset(n.ram, 0, sizeof(n.ram));
  n.padState[0] = n.padState[1] = 0;
  n.padShift[0] = n.padShift[1] = 0;
  n.strobe = 0;

  n.ppu.reset(&n);
  n.apu.reset(&n);
  n.mapper.reset();                          // пересчёт окон банков
  n.cpu.cycles = 0;

  if (n.romLoaded) n.cpu.reset(&n);          // PC <- вектор сброса из PRG ROM
  else             n.cpu.pc = 0;
}

DENDY_FAST_ATTR void runFrame()
{
  if (!g_ready || !g_nes.romLoaded) return;
  Nes& n = g_nes;

  const int      startSerial = n.ppu.frameSerial;
  uint32_t       guard       = 0;            // защита от бесконечного цикла
  // Потолок тактов на кадр берём из таблицы РЕГИОНА (33 247,5 в PAL, 29 780,7 в
  // NTSC) плюс запас на инструкцию DMA/IRQ. Это поле, а не константа: регион
  // выбирается в меню (NesCore::setRegion). Срабатывает только если кадр почему-то
  // не заканчивается, поэтому в норме не влияет ни на что.
  const uint32_t maxFrameCycles = g_frameCycles + 64u;

  while (n.ppu.frameSerial == startSerial && guard < maxFrameCycles) {
    // Линия IRQ: маппер (MMC3) + флаги APU (frame counter / DMC)
    n.cpu.irqLine = n.mapper.irqPending || n.apu.frameIrqFlag || n.apu.dmcIrqFlag;

    const uint32_t before    = n.cpu.cycles;
    const uint32_t irqBefore = n.cpu.irqCount;
#if DENDY_EMU_PROFILE
    const uint32_t c0 = DENDY_CYCLES();
#endif
    n.cpu.step(&n);
#if DENDY_EMU_PROFILE
    const uint32_t c1 = DENDY_CYCLES();
#endif
    const uint32_t delta = n.cpu.cycles - before;
    if (!delta) break;                       // страховка

    // Диагностика: запомнить, какой источник поднял линию IRQ в этот раз
    // (важно делать это сразу после шага — обработчик ещё не успел его сбросить).
    if (n.cpu.irqCount != irqBefore) {
      g_irqSource = (uint8_t)((n.mapper.irqPending ? 1 : 0) |
                              (n.apu.frameIrqFlag ? 2 : 0) |
                              (n.apu.dmcIrqFlag ? 4 : 0));
    }

    // PPU продвигается в тактах CPU, а соотношение берём ИЗ ТАБЛИЦЫ РЕГИОНА:
    //   NTSC 3:1  — 1 такт CPU = ровно 3 точки (89 342 точки за 262 строки);
    //   PAL 16:5  — 1 такт CPU = 3,2 точки (106 392 точки за 312 строк).
    // Дробную часть копим в g_ppuAcc (в единицах 1/PPU_PER_CPU точки): за кадр
    // набегает РОВНО DENDY_*_PPU_DOTS_PER_FRAME точек, иначе кадр «съезжает» на
    // строку. В NTSC делитель равен 1: аккумулятор всегда нулевой, а шаг — ровно
    // delta*3, то есть ровно тот же код, что и раньше, без арифметики.
    // ВАЖНО (скорость): ветка по региону нужна, чтобы делитель остался КОНСТАНТОЙ
    // (деление на переменную на Xtensa — вызов библиотеки). Ветка предсказуема и
    // не меняется до следующего запуска рома, так что цена — доли такта.
    if (g_regionPal) {
      g_ppuAcc += delta * DENDY_PAL_PPU_PER_CPU;                 // 16/5 точки на такт
      const uint32_t ppuDots = g_ppuAcc / DENDY_PAL_CPU_PER_PPU;  // делитель 5
      g_ppuAcc -= ppuDots * DENDY_PAL_CPU_PER_PPU;
      n.ppu.step(&n, ppuDots);
    } else {
      n.ppu.step(&n, delta * DENDY_NTSC_PPU_PER_CPU);       // ровно 3 точки на такт
    }

#if DENDY_EMU_PROFILE
    const uint32_t c2 = DENDY_CYCLES();
#endif
    // APU продвигаем пачками, а не на каждую инструкцию 6502 (~10 000 вызовов за
    // кадр): внутри APU и таймеры каналов, и frame counter, и генерация 44100 Гц
    // работают на аккумуляторах, поэтому от размера порции результат не зависит
    // (см. DENDY_APU_GRANULARITY). Остаток добираем в конце кадра — так у APU не
    // копится постоянное отставание.
    g_apuAcc += delta;
    if (g_apuAcc >= DENDY_APU_GRANULARITY) {
      n.apu.step(&n, g_apuAcc);
      g_apuAcc = 0;
    }
#if DENDY_EMU_PROFILE
    const uint32_t c3 = DENDY_CYCLES();
    g_profCpu += c1 - c0;
    g_profPpu += c2 - c1;
    g_profApu += c3 - c2;
#endif
    guard += delta;
  }

  if (g_apuAcc) {                            // добираем остаток тактов кадра
    n.apu.step(&n, g_apuAcc);
    g_apuAcc = 0;
  }
}

const uint16_t* frameBuffer()
{
  return g_nes.ppu.frame;
}

// Дополнительный кадр для двойной буферизации (см. DENDY_FRAME_DOUBLE_BUFFER).
// Выделяется тем же способом, что и основной кадр в init() (сначала внутренняя
// SRAM, потом PSRAM — см. fbAlloc).
uint16_t* allocFrameBuffer()
{
  const size_t bytes = (size_t)NesPpu::WIDTH * NesPpu::HEIGHT * sizeof(uint16_t);
  uint16_t* p = (uint16_t*)fbAlloc(bytes);
  if (!p) return nullptr;
  memset(p, 0, bytes);                       // первый кадр не должен показывать мусор
  return p;
}

// Сколько кадровых буферов удалось разместить во внутренней SRAM (диагностика).
// Печатается в строке [VID] — по ней видно, сработала ли экономия на PSRAM.
uint8_t frameBuffersInternal()
{
  return g_fbInternalCount;
}

// Переключить PPU на другой кадровый буфер (буфер приходит из allocFrameBuffer()).
void setFrameBuffer(uint16_t* fb)
{
  if (fb) g_nes.ppu.frame = fb;
}

void setPad(uint8_t pad0, uint8_t pad1)
{
  g_nes.padState[0] = pad0;
  g_nes.padState[1] = pad1;
  if (g_nes.strobe) {                        // строб удерживается — регистры «прозрачны»
    g_nes.padShift[0] = pad0;
    g_nes.padShift[1] = pad1;
  }
}

int drainAudio(int16_t* dst, int maxSamples)
{
  return g_nes.apu.drain(dst, maxSamples);
}

// Забрать ровно maxOut сэмплов с АДАПТИВНЫМ темпом (см. NesApu::drainResampled и
// разбор AUDIO_ADAPTIVE_RATE в DendyConfig.h): если эмулятор идёт медленнее
// приставки, звук растягивается вместо вставки тишины в музыку («песок»).
int drainAudioAdaptive(int16_t* dst, int maxOut)
{
  return g_nes.apu.drainResampled(dst, maxOut);
}

// Адаптивный темп включает скетч: в игре — да (эмулятор может не успевать), в
// меню — нет (там кольцо держит полным serviceMenuSound, растягивать нечего).
void setAudioAdaptive(bool on) { g_nes.apu.setAdaptive(on); }
int  audioRatePerMille()       { return g_nes.apu.ratePerMille(); }
uint32_t audioStarved()        { return g_nes.apu.starvedSamples(); }
// Темп продюсера (фактический период кадра из runGame) и диапазон шага для лога:
// см. NesApu::setProducerRatioQ16 / takeRateRange, разбор — DendyConfig.h, блок 3
// («ТЕМП ЭМУЛЯТОРА ДЛЯ ЗВУКА: ЧЕМ ИЗМЕРЯТЬ»).
void setProducerRatioQ16(uint32_t q16) { g_nes.apu.setProducerRatioQ16(q16); }
int  audioRateRange(int* mn, int* mx)  { return g_nes.apu.takeRateRange(mn, mx); }

// Сбросить состояние звукового ВЫВОДА (регулятор адаптивного темпа и «залипший»
// сэмпл). Зовётся на переходах игра <-> меню вместе с flushAudio(): кольцо — это
// одно, а окно измерения темпа/шаг — другое (см. NesApu::resetStream).
void resetAudioStream() { g_nes.apu.resetStream(); }

// ------------------------------ РЕГИОН (NTSC/PAL) ---------------------------
// Скетч зовёт это ПЕРЕД запуском рома (режим меню «TV» + подсказка из заголовка
// образа), до reset(): смена региона меняет частоту кадров, число строк в кадре,
// соотношение CPU:PPU и тактовую 2A03 — то есть весь тайминг эмуляции.
// «На горячую» посреди игры переключать нельзя: PPU уже стоит на середине кадра,
// а APU — на середине окна сэмпла, и один кадр получится «разорванным».
void setRegion(bool pal)
{
  g_regionPal   = pal;
  g_frameCycles = pal ? (uint32_t)DENDY_PAL_FRAME_CYCLES
                      : (uint32_t)DENDY_NTSC_FRAME_CYCLES;
  g_ppuAcc      = 0;                   // остаток перевода тактов CPU -> точки PPU
  g_nes.ppu.setRegion(pal);            // 312 или 262 строки в кадре
  g_nes.apu.setRegion(pal);            // тактовая 2A03 (высота тона и темп музыки)
}

bool     regionIsPal()        { return g_regionPal; }
uint32_t regionFrameCycles()  { return g_frameCycles; }
const char* regionName()      { return g_regionPal ? DENDY_PAL_NAME : DENDY_NTSC_NAME; }
uint8_t  romRegionHint()      { return g_nes.mapper.regionHintFromHeader(); }

// Протолкнуть в кольцо APU СВОИ сэмплы (звук меню, когда эмулятор не крутится).
int pushAudio(const int16_t* src, int maxSamples)
{
  return g_nes.apu.pushExternal(src, maxSamples);
}

int audioFree() { return g_nes.apu.freeSpace(); }
int audioFill() { return g_nes.apu.fill(); }
uint32_t audioDrops() { return g_nes.apu.droppedSamples(); }
void flushAudio() { g_nes.apu.flush(); }

// «Подушка» звука игры: AUDIO_PRIME_SAMPLES сэмплов тишины в кольце APU перед
// стартом рома (см. NesApu::pushSilence и разбор хрипа в DendyConfig.h, блок 3).
// Вызывать ПОСЛЕ reset(): NesApu::reset() чистит кольцо и обнуляет его индексы.
void primeAudio(int samples)
{
  if (samples > 0) g_nes.apu.pushSilence(samples);
}

void setVolume(uint8_t volume)
{
  g_nes.apu.setVolume(volume);
}

// ---- Порядок каналов цвета кадра (см. DENDY_COLOR_ORDER в DendyConfig.h) -----
// Скетч вызывает это по «горячим» клавишам: таблица палитры пересобирается на
// лету, кадр сразу идёт в новых цветах — правильный режим виден без пересборки.
void setColorOrder(uint8_t mode)
{
  g_nes.ppu.setColorOrder(mode);
}

uint8_t colorOrder()
{
  return g_nes.ppu.colorOrder();
}

uint16_t paletteRaw(uint8_t index)
{
  return g_nes.ppu.paletteRaw(index);
}

bool isLoaded()
{
  return g_nes.romLoaded;
}

bool romSupported()
{
  return g_nes.mapper.supported();
}

int romMapperNumber()
{
  return g_nes.mapper.number;
}

const char* romMapperName()
{
  return g_nes.mapper.mapperName();
}

// -------------------- Диагностика видеотракта (только чтение) -----------------
uint8_t  ppuCtrl()        { return g_nes.ppu.ctrlReg(); }
uint8_t  ppuMask()        { return g_nes.ppu.maskReg(); }
uint8_t  ppuStatus()      { return g_nes.ppu.statusReg(); }
int      ppuFrameSerial() { return g_nes.ppu.frameSerial; }
uint32_t ppuSprite0Count(){ return g_nes.ppu.sprite0Count(); }
int      ppuSprite0Lines()  { return g_nes.ppu.sprite0LineCount(); }
int      ppuSprite0Misses() { return g_nes.ppu.sprite0MissCount(); }
int      ppuSprite0LastDot(){ return g_nes.ppu.sprite0LastDot(); }
const uint8_t* ppuOam()     { return g_nes.ppu.oamPtr(); }
void     ppuScroll(uint16_t* v, uint16_t* t, uint8_t* fineX)
{
  if (v)     *v     = g_nes.ppu.vAddr();
  if (t)     *t     = g_nes.ppu.tAddr();
  if (fineX) *fineX = g_nes.ppu.fineXReg();
}
uint16_t framePixel(int x, int y)
{
  const uint16_t* fb = g_nes.ppu.frame;
  if (!fb || x < 0 || y < 0 || x >= NesPpu::WIDTH || y >= NesPpu::HEIGHT) return 0;
  return fb[(size_t)y * NesPpu::WIDTH + (size_t)x];
}

// -------------------- Профиль времени по фазам (DENDY_EMU_PROFILE) ------------
// Отдаёт накопленные за прошедшие кадры такты CPU и обнуляет счётчики, поэтому
// скетч просто печатает их в лог (такты -> мс: ticks / (частота_CPU_МГц * 1000)).
// Это ответ на вопрос «что именно тормозит»: ядро 6502, PPU/рендер или APU.
void profileTake(uint32_t* cpu, uint32_t* ppu, uint32_t* apu, uint32_t* render,
                 uint32_t* bg, uint32_t* spr)
{
  if (cpu)    { *cpu    = g_profCpu; g_profCpu = 0; }
  if (ppu)    { *ppu    = g_profPpu; g_profPpu = 0; }
  if (apu)    { *apu    = g_profApu; g_profApu = 0; }
  if (render) { *render = g_nes.ppu.profRenderCycles; g_nes.ppu.profRenderCycles = 0; }
  if (bg)     { *bg     = g_nes.ppu.profBgCycles;      g_nes.ppu.profBgCycles     = 0; }
  if (spr)    { *spr    = g_nes.ppu.profSprCycles;     g_nes.ppu.profSprCycles    = 0; }
}
uint16_t cpuPc()          { return g_nes.cpu.pc; }
uint8_t  cpuA()           { return g_nes.cpu.a; }
uint8_t  cpuX()           { return g_nes.cpu.x; }
uint8_t  cpuY()           { return g_nes.cpu.y; }
uint8_t  cpuSp()          { return g_nes.cpu.sp; }
uint8_t  cpuP()           { return g_nes.cpu.p; }
uint32_t cpuIrqCount()    { return g_nes.cpu.irqCount; }
uint32_t cpuNmiCount()    { return g_nes.cpu.nmiCount; }
uint32_t cpuInstrCount()  { return g_nes.cpu.instrCount; }
uint8_t  lastIrqSource()  { return g_irqSource; }
uint8_t  irqFlagsNow()
{
  return (uint8_t)((g_nes.mapper.irqPending ? 1 : 0) |
                   (g_nes.apu.frameIrqFlag ? 2 : 0) |
                   (g_nes.apu.dmcIrqFlag ? 4 : 0));
}
uint8_t  apuFlags()
{
  return (uint8_t)((g_nes.apu.frameIrqFlag ? 1 : 0) | (g_nes.apu.dmcIrqFlag ? 2 : 0));
}
uint8_t  mapperLatch()      { return g_nes.mapper.mmc3LatchValue(); }
uint8_t  mapperCounter()    { return g_nes.mapper.mmc3CounterValue(); }
bool     mapperIrqEnabled() { return g_nes.mapper.mmc3IrqEnabled(); }
uint8_t  mapperBankSelect() { return g_nes.mapper.mmc3BankSelect(); }
uint8_t  mapperReg(uint8_t index) { return g_nes.mapper.mmc3RegValue(index); }
// Диагностика кэша окон банков (DENDY_CART_IRAM): забрать и обнулить счётчик копий.
uint32_t mapperCacheCopies() { return g_nes.mapper.cacheCopies(true); }

// -------------------- Диагностика ввода и шины (для лога) --------------------
uint32_t padSeenReads() { return g_nes.padReads[0]; }
uint8_t  padSeenByte()  { return g_nes.padByte[0]; }
uint32_t apuWrites()    { return g_nes.cntWrApu; }
uint32_t mapperWrites() { return g_nes.cntWrMapper; }
uint8_t  apuChannels()  { return g_nes.apu.enabledChannels(); }
uint8_t  ramPeek(uint16_t addr) { return g_nes.ram[addr & 0x07FF]; }

uint32_t ioWrites(uint16_t addr)
{
  switch (addr) {
    case 0x2000: return g_nes.cntWr2000;
    case 0x2001: return g_nes.cntWr2001;
    case 0x2005: return g_nes.cntWr2005;
    case 0x2006: return g_nes.cntWr2006;
    case 0x2007: return g_nes.cntWr2007;
    case 0x4014: return g_nes.cntDma4014;
    case 0x4015: return g_nes.cntWr4015;
    default:     return 0;
  }
}

uint32_t ioReads(uint16_t addr)
{
  if (addr == 0x2002) return g_nes.cntRd2002;
  if (addr == 0x4015) return g_nes.cntRd4015;
  return 0;
}

uint16_t cpuTraceAt(int i)
{
  const int idx = (g_nes.cpu.pcTraceIdx + i) & (NesCpu::TRACE_LEN - 1);
  return g_nes.cpu.pcTrace[idx];
}

}  // namespace NesCore
