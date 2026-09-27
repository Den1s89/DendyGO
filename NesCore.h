/**
 * =============================================================================
 *  DendyGO / NesCore.h
 *  Полное состояние эмулятора NES + шина обмена между CPU/PPU/APU/картриджем.
 *
 *  Внешний мир общается с ядром только через namespace NesCore (см. низ файла),
 *  поэтому ядро можно заменить на InfoNES/NoNES без правок остальных модулей.
 * =============================================================================
 */
#pragma once
#include <stdint.h>
#include "NesCpu.h"
#include "NesPpu.h"
#include "NesApu.h"
#include "NesMapper.h"

// ------------------------- Биты стандартного джойстика NES --------------------
enum : uint8_t {
  PAD_A      = 0x01,
  PAD_B      = 0x02,
  PAD_SELECT = 0x04,
  PAD_START  = 0x08,
  PAD_UP     = 0x10,
  PAD_DOWN   = 0x20,
  PAD_LEFT   = 0x40,
  PAD_RIGHT  = 0x80,
};

// ------------------------- Полное состояние эмулятора ------------------------
struct Nes {
  NesCpu    cpu;
  NesPpu    ppu;
  NesApu    apu;
  NesMapper mapper;

  uint8_t  ram[0x800];                 // 2 КБ встроенного ОЗУ
  uint8_t  padState[2] = {0, 0};       // текущее состояние портов (маска PAD_*)
  uint8_t  padShift[2] = {0, 0};       // сдвиговые регистры $4016/$4017
  uint8_t  strobe = 0;                 // 1 -> сдвиговые регистры загружаются заново

  // --- Диагностика ввода (DENDY_VIDEO_TRACE): что игра РЕАЛЬНО прочитала ------
  // Копится от строба $4016: так видно, доходит ли нажатие до игры. Если байт
  // не меняется вслед за кнопками, проблема в ядре/проводке, а не в самой игре.
  uint32_t padReads[2] = {0, 0};       // сколько раз читали $4016 / $4017
  uint8_t  padByte[2]  = {0, 0};       // принятый байт: бит0 = A, бит3 = Start...
  uint8_t  padBits[2]  = {0, 0};       // сколько битов уже сдвинуто с последнего строба

  // --- Диагностика шины (DENDY_VIDEO_TRACE): признаки «живой» игры ------------
  // Пишет ли игра VRAM, льёт ли спрайты через DMA, включает ли каналы APU и
  // переключает ли банки маппера (MMC3). Счётчики монотонные — смотрим прирост.
  uint32_t cntWr2000 = 0, cntWr2001 = 0, cntWr2005 = 0, cntWr2006 = 0, cntWr2007 = 0;
  uint32_t cntRd2002 = 0, cntDma4014 = 0, cntWr4015 = 0, cntRd4015 = 0;
  uint32_t cntWrApu = 0, cntWrMapper = 0;

  uint8_t* romBuffer = nullptr;        // образ .nes в PSRAM (владелец — NesCore)
  uint32_t romSize   = 0;
  bool     romLoaded = false;

  // Чтение джойстика для $4016/$4017 (формат: A,B,Select,Start,Up,Down,Left,Right)
  uint8_t readPad(int port) {
    const uint8_t bit = (padShift[port] & 1);
    padShift[port] = (uint8_t)((padShift[port] >> 1) | 0x80);
    ++padReads[port];                                // диагностика: игра читает порт
    if (padBits[port] < 8) padByte[port] |= (uint8_t)(bit << padBits[port]);
    padBits[port] = (uint8_t)(padBits[port] + 1);
    return bit;
  }
};

// ------------------------- Шина (inline — см. NesCore.cpp) -------------------
// ВАЖНО: тело шины живёт ЗДЕСЬ, а не в NesCore.cpp. Одна инструкция 6502 делает
// 2..6 обращений к памяти, и вызов шины из другого модуля (без LTO) стоил
// заметно дороже самой инструкции — эмуляция не дотягивала до 60 кадров/с.
// Определение в заголовке даёт компилятору встроить разбор адреса прямо в ядро
// CPU, PPU, APU и маппер. Ничего, кроме памяти Nes и вызовов модулей, тут нет.
inline uint8_t nesRead(Nes* nes, uint16_t addr)
{
  // ВАЖНО (скорость): самое частое обращение всей эмуляции — код и данные игры из
  // $8000-$FFFF (2..4 обращения на инструкцию 6502, то есть десятки тысяч за кадр).
  // Поэтому первым идёт ИМЕННО этот случай, и без лишней работы: окно банка берётся
  // прямо из prgBank[] — указатель на 8 КБ банк ставит маппер (prgWindow()), и он
  // всегда действителен: в конце configureBanks() есть страховка, которая чинит
  // любое окно, не указывающее в ром/кэш.
  // ЗАЧЕМ ТАК: раньше здесь вызывался mapper.readPrg(), а он ПОВТОРНО проверял те
  // же самые диапазоны адресов ($6000-$7FFF и «< $8000»). На каждое чтение кода
  // уходило на ~6 инструкций больше, а при ~30 000 чтений за кадр это несколько
  // миллисекунд времени кадра (в логе это большой `cpu` в строке [GAME] profil).
  if (addr >= 0x8000) return nes->mapper.prgBank[(addr >> 13) & 3][addr & 0x1FFF];
  if (addr >= 0x6000) return nes->mapper.wram[addr & 0x1FFF];   // WRAM картриджа
  if (addr < 0x2000) return nes->ram[addr & 0x07FF];
  if (addr < 0x4000) {                       // регистры PPU (зеркала каждые 8 байт)
    const uint8_t reg = (uint8_t)(addr & 0x07);
#if DENDY_VIDEO_TRACE
    if (reg == 2) ++nes->cntRd2002;          // диагностика: игра ждёт статус PPU
#endif
    return nes->ppu.readRegister(nes, reg);
  }
#if DENDY_VIDEO_TRACE
  if (addr == 0x4015) { ++nes->cntRd4015; return nes->apu.readStatus(nes); }
#else
  if (addr == 0x4015) return nes->apu.readStatus(nes);
#endif
  if (addr == 0x4016) return nes->readPad(0);
  if (addr == 0x4017) return nes->readPad(1);
  return 0;                                  // $4018-$5FFF: не используется
}

inline void nesWrite(Nes* nes, uint16_t addr, uint8_t value)
{
  if (addr < 0x2000) { nes->ram[addr & 0x07FF] = value; return; }   // ОЗУ (чаще всего)
  if (addr >= 0x8000) {                      // PRG ROM картриджа: банки/IRQ маппера
#if DENDY_VIDEO_TRACE
    ++nes->cntWrMapper;                      // диагностика: банки маппера (MMC3)
#endif
    nes->mapper.writePrg(nes, addr, value);
    return;
  }
  if (addr < 0x4000) {                       // регистры PPU (зеркала каждые 8 байт)
    const uint8_t reg = (uint8_t)(addr & 0x07);
#if DENDY_VIDEO_TRACE
    switch (reg) {                           // диагностика: видно, что игра делает
      case 0: ++nes->cntWr2000; break;
      case 1: ++nes->cntWr2001; break;
      case 5: ++nes->cntWr2005; break;
      case 6: ++nes->cntWr2006; break;
      case 7: ++nes->cntWr2007; break;
      default: break;
    }
#endif
    nes->ppu.writeRegister(nes, reg, value);
    return;
  }

  if (addr == 0x4014) {                      // DMA спрайтов: 256 байт + 513 тактов
#if DENDY_VIDEO_TRACE
    ++nes->cntDma4014;
#endif
    nes->ppu.writeOamDma(nes, value);
    nes->cpu.cycles += 513;
    return;
  }

  if (addr == 0x4016) {                      // строб джойстиков
    nes->strobe = (uint8_t)(value & 1);
    if (nes->strobe) {
      nes->padShift[0] = nes->padState[0];
      nes->padShift[1] = nes->padState[1];
      // Диагностика: с нового строба собираем принятый игрой байт заново.
      nes->padByte[0] = nes->padByte[1] = 0;
      nes->padBits[0] = nes->padBits[1] = 0;
    }
    return;
  }

  if (addr >= 0x4000 && addr <= 0x4017) {    // APU ($4000-$4013, $4015, $4017)
#if DENDY_VIDEO_TRACE
    ++nes->cntWrApu;
    if (addr == 0x4015) ++nes->cntWr4015;
#endif
    nes->apu.writeRegister(nes, addr, value);
    return;
  }

  if (addr >= 0x6000) nes->mapper.writePrg(nes, addr, value);   // WRAM картриджа
}

// ============================== Публичный API ================================
// Единственная точка входа для внешнего мира (main-скетч, меню, аудио-задача).
namespace NesCore {
  bool   init();                            // разовая инициализация буферов (PSRAM)
  // Загрузка образа .nes. ВНИМАНИЕ: при успехе ядро ЗАБИРАЕТ ВЛАДЕНИЕ буфером
  // (он должен быть выделен в PSRAM, например ps_malloc) и само освободит его
  // в unloadRom(). При ошибке буфер НЕ освобождается — вызывающий решает сам.
  bool   loadRom(uint8_t* data, uint32_t size);
  void   unloadRom();                       // освободить буфер рома
  void   reset();                           // сброс эмулятора (Reset кнопкой)
  // ВАЖНО (скорость): DENDY_FAST_ATTR = IRAM_ATTR. В runFrame() встраивается шаг
  // PPU (NesPpu::step — DENDY_INLINE), то есть весь цикл точек (89 342 точки за
  // кадр) и разбор тактов CPU/PPU/APU. Из flash он исполняется через кэш 32 КБ,
  // а тот же кэш на каждом кадре вытесняют записи кадрового буфера и чтение кадра
  // в PSRAM — поэтому код из flash в этих местах начинает тормозить в разы.
  // IRAM не кэшируется вообще: см. README, «Скорость и звук».
  DENDY_FAST_ATTR void runFrame();          // эмулировать один кадр
  const uint16_t* frameBuffer();            // 256x240 RGB565 (куда PPU РИСУЕТ сейчас)
  // --- Двойная буферизация кадра (DENDY_FRAME_DOUBLE_BUFFER) -------------------
  // Пока один кадр уходит на экран по SPI, PPU рисует следующий в ДРУГОЙ буфер:
  // эмуляция и передача идут параллельно, игровое время больше не теряется.
  uint16_t* allocFrameBuffer();             // выделить ещё один кадр (сначала SRAM, потом PSRAM)
  uint8_t   frameBuffersInternal();         // сколько кадров легло во внутреннюю SRAM (диагностика)
  void      setFrameBuffer(uint16_t* fb);   // переключить PPU на другой кадр (fb из allocFrameBuffer)
  void   setPad(uint8_t pad0, uint8_t pad1); // передать состояние кнопок
  int    drainAudio(int16_t* dst, int maxSamples); // забрать сэмплы APU
  // --- Адаптивный темп вывода (AUDIO_ADAPTIVE_RATE в DendyConfig.h) ----------
  // Забрать РОВНО maxOut сэмплов, растягивая кольцо, если эмулятор идёт медленнее
  // приставки (иначе в музыку вставлялась тишина — слышно как «песок»). Когда
  // эмулятор успевает (60,0 кадра/с), шаг ровно 1:1 и звук уходит бит в бит.
  int    drainAudioAdaptive(int16_t* dst, int maxOut);
  void   setAudioAdaptive(bool on);         // игра — true, меню — false
  int    audioRatePerMille();               // темп вывода: 1000 = 1:1 ([SND] `temp`)
  // Темп продюсера от скетча (NesApu::setProducerRatioQ16): отношение «номинальный
  // период кадра / фактический» в Q16 — база шага чтения кольца. Плюс диапазон
  // шага за окно (в промилле) для строки [SND]: «стоит ровно» или «дышит».
  void   setProducerRatioQ16(uint32_t q16);
  int    audioRateRange(int* minPermille, int* maxPermille);
  uint32_t audioStarved();                  // «зажато» сэмплов ([SND] `golod`)
  int    pushAudio(const int16_t* src, int maxSamples); // протолкнуть свои сэмплы (звук меню)
  int    audioFree();                       // сколько сэмплов ещё влезет в кольцо APU
  int    audioFill();                       // сколько сэмплов лежит в кольце APU
  // Сэмплы APU, потерянные из-за полного кольца (задача I2S не успевала забирать).
  // Должно быть 0: ненулевое значение = «песок»/дырки в музыке (см. [SND] `drob`).
  uint32_t audioDrops();
  void   flushAudio();                      // очистить кольцо (переход меню <-> игра)
  // Предзаполнить кольцо тишиной — «подушка» звука игры (AUDIO_PRIME_SAMPLES):
  // без неё задача I2S подходит к пустому кольцу в паузах между кадрами и добивала
  // блок DMA нулями, что и звучало как хрип. Звать ПОСЛЕ reset() — он чистит кольцо.
  void   primeAudio(int samples);
  void   setVolume(uint8_t volume);         // громкость 0..255
  // --- РЕГИОН ЭМУЛЯЦИИ (NTSC / PAL) -------------------------------------------
  // Эмулятор НЕ «угадывает» регион по рому: в коде самой игры этого не написано.
  // Признак есть только в заголовке образа (байт 9/10/12 iNES — см.
  // NesMapper::regionHint), поэтому скетч делает так:
  //   * режим AUTO  — берёт регион из заголовка рома (romRegionHint());
  //   * режим NTSC/PAL — берёт то, что выбрано в меню (кнопка SX_PIN_REGION);
  // и ПЕРЕД запуском рома зовёт setRegion(pal). Менять регион «на горячую»
  // посреди игры нельзя: от него зависят частота кадров, число строк в кадре,
  // соотношение CPU:PPU и тактовая 2A03, то есть весь тайминг эмуляции.
  void   setRegion(bool pal);               // true = PAL 50 Гц (312 строк)
  bool   regionIsPal();
  uint32_t regionFrameCycles();             // тактов CPU в кадре (защита runFrame)
  const char* regionName();                 // «PAL 50 Hz (312 strok)» и т. п.
  uint8_t romRegionHint();                  // 0 = NTSC, 1 = PAL, 2 = в заголовке нет
  // Сбросить состояние ЗВУКОВОГО ВЫВОДА (регулятор темпа + «залипший» сэмпл) на
  // переходе игра <-> меню. Кольцо чистит flushAudio(), это — другое.
  void   resetAudioStream();
  // --- Порядок каналов цвета кадра ------------------------------------------
  // 0 — как есть (обычно это и нужно: цвета кадра совпадают с цветами меню,
  //     которое рисует LovyanGFX), 1 — байты слова, 2 — G<->B, 3 — R<->B («BGR»).
  void     setColorOrder(uint8_t mode);     // сменить на лету (0..3)
  uint8_t  colorOrder();                    // текущий режим
  uint16_t paletteRaw(uint8_t index);       // логический RGB565 цвета NES (эталон)
  bool   isLoaded();
  bool   romSupported();                    // поддерживается ли маппер рома
  int    romMapperNumber();
  const char* romMapperName();

  // --- Диагностика ввода и шины (печатается скетчем при DENDY_VIDEO_TRACE) ---
  uint32_t padSeenReads();                // сколько раз игра прочитала $4016
  uint8_t  padSeenByte();                 // что игра приняла: бит0=A, бит3=Start
  uint32_t ioWrites(uint16_t addr);       // счётчик записей ($2000-$2007/$4014/$4015)
  uint32_t ioReads(uint16_t addr);        // счётчик чтений ($2002/$4015)
  uint32_t apuWrites();                   // записи в APU ($4000-$4017) — есть ли музыка
  uint32_t mapperWrites();                // записи $8000-$FFFF — банки/IRQ маппера
  uint8_t  apuChannels();                 // последнее значение $4015 (включённые каналы)
  uint8_t  ramPeek(uint16_t addr);        // байт ОЗУ эмулятора (дамп адресов игры)

  // --- Диагностика видеотракта (печатается скетчем при DENDY_VIDEO_TRACE) ---
  uint8_t  ppuCtrl();                     // последнее значение регистра $2000
  uint8_t  ppuMask();                     // регистр $2001: биты 3/4 — BG/спрайты
  uint8_t  ppuStatus();                   // регистр $2002
  int      ppuFrameSerial();              // счётчик собранных кадров
  uint32_t ppuSprite0Count();             // сколько раз поднят флаг sprite-0 hit
  int      ppuSprite0Lines();             // строк кадра, где спрайт 0 есть на экране
  int      ppuSprite0Misses();            // из них — где фон под спрайтом 0 прозрачный
  int      ppuSprite0LastDot();           // x последнего попадания sprite-0 (-1 = не было)
  const uint8_t* ppuOam();                // 256 байт OAM (для дампа в скетче)
  void     ppuScroll(uint16_t* v, uint16_t* t, uint8_t* fineX);  // loopy-скролл
  uint16_t framePixel(int x, int y);      // пиксель кадра RGB565 (для ASCII-дампа)
  // --- Профиль времени по фазам (DENDY_EMU_PROFILE) ---
  // Такты CPU, накопленные за прошедшие кадры: cpu — интерпретатор 6502,
  // ppu — продвижение PPU, apu — звук, render — из ppu на отрисовку строк.
  // Любой указатель можно не передавать (nullptr). Счётчики обнуляются.
  void     profileTake(uint32_t* cpu, uint32_t* ppu, uint32_t* apu, uint32_t* render,
                       uint32_t* bg, uint32_t* spr);
  uint16_t cpuPc();                       // текущий PC процессора 6502
  uint8_t  cpuA();                        // регистр A
  uint8_t  cpuX();                        // регистр X
  uint8_t  cpuY();                        // регистр Y
  uint8_t  cpuSp();                       // указатель стека
  uint8_t  cpuP();                        // регистр флагов (NV-BDIZC)
  uint32_t cpuIrqCount();                 // принято IRQ с момента сброса
  uint32_t cpuNmiCount();                 // принято NMI с момента сброса
  uint32_t cpuInstrCount();               // выполнено инструкций с момента сброса
  uint16_t cpuTraceAt(int i);             // адрес инструкции: 0 — самая старая из трассы
  uint8_t  lastIrqSource();               // источник последнего IRQ: 1=маппер, 2=APU frame, 4=APU DMC
  uint8_t  irqFlagsNow();                 // те же биты, но в текущий момент (0 — линия чистая)
  uint8_t  apuFlags();                    // бит0 = frame IRQ флаг, бит1 = DMC IRQ флаг
  uint8_t  mapperLatch();                 // MMC3: значение перезагрузки IRQ-счётчика
  uint8_t  mapperCounter();               // MMC3: текущий счётчик строк
  bool     mapperIrqEnabled();            // MMC3: разрешён ли IRQ по строкам
  uint8_t  mapperBankSelect();            // MMC3: запись $8000 (биты 6/7 — режимы PRG/CHR)
  uint8_t  mapperReg(uint8_t index);      // MMC3: регистр R0..R7 (окна банков PRG/CHR)
  // --- Диагностика кэша окон банков (DENDY_CART_IRAM) ---
  // Сколько раз за прошедшее окно окно банка пришлось скопировать во внутреннюю
  // SRAM. Если число большое (игра меняет банки каждую строку), кэш может быть
  // невыгоден — тогда DENDY_CART_IRAM = 0 быстрее. Забирает и обнуляет счётчик.
  uint32_t mapperCacheCopies();
}
