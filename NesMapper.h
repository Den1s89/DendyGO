/**
 * =============================================================================
 *  DendyGO / NesMapper.h
 *  Картридж NES: разбор заголовка iNES, банковка PRG/CHR, зеркалирование VRAM
 *  и счётчик прерываний маппера.
 *
 *  Поддержаны мапперы:
 *    0  - NROM            (Super Mario Bros., Battle City, Donkey Kong, ...)
 *    1  - MMC1            (Metroid, The Legend of Zelda, Bomberman, ...)
 *    2  - UxROM           (Contra (U), Castlevania, Mega Man, ...)
 *    3  - CNROM           (Arkanoid, ...)
 *    4  - MMC3 (IRQ по строкам, приближённо)
 *    7  - AxROM           (Battletoads - только без IRQ-эффектов)
 *    11 - Color Dreams
 *    23 - VRC2b (Konami)  (Contra (J), Wai Wai World, Ganbare Goemon, ...)
 *    66 - GxROM
 *    71 - Camerica (Codemasters) (Super Robin Hood, Fantastic Adventures of Dizzy)
 * =============================================================================
 */
#pragma once
#include <stdint.h>
#include "DendyFast.h"                // DENDY_FAST_ATTR (IRAM для горячего кода)
struct Nes;

class NesMapper {
public:
  // --- Общие поля (используются шиной и PPU) ---
  uint8_t  number       = 0;      // номер маппера
  uint8_t  mirroring    = 0;      // 0=horizontal, 1=vertical, 2=single0, 3=single1, 4=four screen
  bool     chrIsRam     = false;  // true -> используется CHR RAM (8 КБ)
  uint8_t* prg          = nullptr;// PRG ROM (указывает внутрь буфера рома в PSRAM)
  uint32_t prgSize      = 0;      // размер PRG ROM (байт)
  uint8_t* chr          = nullptr;// CHR ROM/RAM
  uint32_t chrSize      = 0;
  bool     irqPending   = false;  // активный IRQ от маппера (MMC3)

  // --- Регион рома из заголовка iNES ------------------------------------------
  // Эмулятор сам НЕ «понимает», PAL ром или NTSC: в самом коде игры этого нет,
  // разница только в частоте кадров (и, значит, в скорости игры). Единственный
  // признак, который можно прочитать, — заголовок образа:
  //   * байт 9, бит 0:  1 = PAL-версия (так помечают почти все европейские дампы);
  //   * байт 10, биты 0..1 (только для VS/PlayChoice-образов): 0 = NTSC, 2 = PAL;
  //   * байт 7, биты 2..3 == 10 (NES 2.0): тайминг в байте 12, биты 0..1:
  //     0 = NTSC, 1 = PAL, 2 = «оба», 3 = Dendy (у нас это тоже 50 Гц).
  // regionHint: 0 = NTSC, 1 = PAL (50 Гц), 2 = «в заголовке не сказано».
  // Режим в меню «TV» решает, верить ли заголовку (AUTO) или взять своё.
  uint8_t  regionHint   = 2;
  uint8_t  regionHintFromHeader() const { return regionHint; }

  // Окна банков: 4 x 8 КБ PRG ($8000,$A000,$C000,$E000) и 8 x 1 КБ CHR
  uint8_t* prgBank[4];
  uint8_t* chrBank[8];

  // Память картриджа
  uint8_t  wram[0x2000];          // 8 КБ PRG RAM ($6000-$7FFF)
  uint8_t  chrRam[0x2000];        // 8 КБ CHR RAM (если в роме нет CHR ROM)

  // --- API ---
  bool load(uint8_t* romFile, uint32_t fileSize); // разбор .nes (без копирования данных)
  void reset();
  void clear();                   // обнулить всё состояние (ром выгружен)
  // readPrg()/readChr() определены НИЖЕ, в этом же заголовке (см. пояснение там):
  // это самые горячие обращения к памяти всего эмулятора.
  uint8_t readPrg(Nes* nes, uint16_t addr);
  DENDY_FAST_ATTR void    writePrg(Nes* nes, uint16_t addr, uint8_t value);
  uint8_t readChr(Nes* nes, uint16_t addr);
  void    writeChr(Nes* nes, uint16_t addr, uint8_t value);
  DENDY_FAST_ATTR void    scanlineTick(Nes* nes);   // вызов из PPU в начале каждой видимой строки (MMC3)
  const char* mapperName() const;
  bool    supported() const;

  // --- Диагностика MMC3 (печатается скетчем при DENDY_VIDEO_TRACE) ---
  uint8_t mmc3LatchValue()   const { return mmc3Latch; }    // значение перезагрузки счётчика
  uint8_t mmc3CounterValue() const { return mmc3Counter; }  // текущий счётчик строк
  bool    mmc3IrqEnabled()   const { return mmc3IrqEnable; }
  uint8_t mmc3BankSelect()   const { return mmc3BankSel; }  // $8000: биты 6/7 — режимы PRG/CHR
  uint8_t mmc3RegValue(int i) const { return (i >= 0 && i < 8) ? mmc3Regs[i] : 0; }

  // --- Диагностика кэша окон банков (DENDY_CART_IRAM) ---
  // Сколько раз реально выполнялось копирование окна в кэш. Если на конкретной
  // игре это число большое (игра меняет банки на каждой строке), кэш может стать
  // дороже, чем польза: тогда DENDY_CART_IRAM = 0 быстрее. С reset=true — забрать
  // и обнулить (для строки [GAME] profil).
  uint32_t cacheCopies(bool reset = false);

private:
  // --- Вспомогательные функции окон банков ---
  void configureBanks();                     // пересчёт всех окон под текущий маппер
  void setPrg32(uint32_t bank32);            // 32 КБ в $8000-$FFFF
  void setPrg8(int slot, uint32_t bank8);    // 8 КБ в окно slot: 0=$8000 .. 3=$E000
  void setPrg16(int half, uint32_t bank16);  // 16 КБ: half=0 -> $8000, half=1 -> $C000
  void setChr8(uint32_t bank8);              // 8 КБ CHR в $0000-$1FFF
  void setChr1k(int slot, uint32_t bank1k);  // 1 КБ CHR в окно slot (0..7)

  // --- Кэш активных окон банков во внутренней SRAM (DENDY_CART_IRAM) ----------
  // PRG/CHR лежат в PSRAM, а это самые горячие обращения всего эмулятора: PRG
  // читается 2..4 раза на каждую инструкцию 6502, CHR — на каждый тайл фона и
  // спрайта (десятки тысяч раз за кадр, см. строку [GAME] profil). Промах кэша
  // в PSRAM стоит десятки тактов, а внутренняя SRAM читается за 1..2 такта и её
  // не вытесняет кадровый буфер. Копия окна происходит только при СМЕНЕ банка
  // (десятки раз за кадр у MMC3), поэтому стоит копейки.
  void prgWindow(int slot, uint32_t bank);   // поставить окно PRG (8 КБ) + кэш
  void chrWindow(int slot, uint32_t bank);   // поставить окно CHR (1 КБ) + кэш
  bool prgWindowOk(int slot) const;          // окно указывает в ROM или в кэш
  bool chrWindowOk(int slot) const;          // окно указывает в CHR или в кэш
#if DENDY_CART_IRAM
  uint8_t prgCache[4][0x2000];               // 32 КБ: активные окна PRG ROM
  uint8_t chrCache[8][0x0400];               // 8 КБ: активные окна CHR ROM
  int32_t prgCacheBank[4];                   // какой банк лежит в кэше (-1 = пусто)
  int32_t chrCacheBank[8];
#endif
  uint32_t prgCopies = 0;                    // сколько копий PRG/CHR сделано (диагностика)
  uint32_t chrCopies = 0;

  // --- MMC1 ---
  uint8_t mmc1Shift = 0x10, mmc1Ctrl = 0x0C, mmc1Chr0 = 0, mmc1Chr1 = 0, mmc1Prg = 0;
  void mmc1Write(uint16_t addr, uint8_t value);

  // --- MMC3 ---
  // mmc3Select  — номер регистра (биты 0..2 записи в $8000)
  // mmc3BankSel — вся запись $8000: бит 6 = режим PRG, бит 7 = режим CHR.
  // ВАЖНО: раньше «полный» селект не хранился (mmc3Select = value & 0x07), и
  // configureBanks() читал режимы из mmc3Regs[6] — то есть из НОМЕРА БАНКА R6.
  // В итоге любая MMC3-игра всегда получала «PRG mode 0 / CHR mode A»: игровые
  // окна банков не совпадали с задуманными, и картинка рассыпалась в мусорные
  // тайлы (так выглядят Felix the Cat и Adventure Island III).
  uint8_t mmc3Select = 0, mmc3BankSel = 0, mmc3Regs[8] = {0};
  uint8_t mmc3Latch = 0, mmc3Counter = 0;
  bool    mmc3Reload = false, mmc3IrqEnable = false;
  void mmc3Write(uint16_t addr, uint8_t value);

  // --- VRC2b (маппер 23: Contra (J), Wai Wai World, ...) ---------------------
  // PRG: 8 КБ в $8000-$9FFF (vrcPrg[0]), 8 КБ в $A000-$BFFF (vrcPrg[1]), а окно
  // $C000-$FFFF всегда указывает на последние 16 КБ рома. CHR: 8 окон по 1 КБ.
  // Заодно есть «защёлка» $6000-$6FFF (Microwire): игра пишет байт в $6000 и
  // читает его назад (значим бит 0) — это уже покрывает общая эмуляция WRAM в
  // readPrg()/writePrg(); без неё Contra (J) «зависает почти сразу после старта».
  uint8_t vrcPrg[2] = {0, 0};                     // номера 8 КБ банков PRG
  uint8_t vrcChr[8] = {0};                        // номера 1 КБ банков CHR
  void vrc2Write(uint16_t addr, uint8_t value);   // регистры VRC2b

  // --- Codemasters / Camerica (маппер 71: Super Robin Hood, Dizzy) -----------
  // Клон UNROM: 16 КБ банк в $8000-$BFFF, последние 16 КБ всегда в $C000-$FFFF,
  // CHR ROM нет (8 КБ CHR RAM, без банковки).
  // cmBank — номер 16 КБ банка для $8000-$BFFF. Его пишут в ЛЮБОЙ адрес
  // $8000-$FFFF, в том числе в $9000-$9FFF: Super Robin Hood меняет банк из
  // RAM-трамплина («LDA #$18 / STA $9000», $18 = номер банка движка Dizzy; у
  // 64 КБ рома он берётся по модулю 4), рядом в том же роме те же номера пишут
  // в $C016-$C018. То есть $9000-$9FFF — ЭТО ЖЕ регистр номера банка, а не
  // переключатель зеркалирования.
  // cmMirrorReg — включать ли регистр 1-screen зеркалирования в $9000-$9FFF.
  // Он есть ТОЛЬКО у платы BF9097 (Fire Hawk); остальные платы линейки
  // (BF9093/BF9096: Super Robin Hood, Fantastic Adventures of Dizzy, Bee 52)
  // имеют ЖЁСТКОЕ зеркалирование из заголовка рома. В iNES 1.0 это различие не
  // хранится, поэтому признак — NES 2.0 submapper 1 (см. load()). По NESdev,
  // «INES Mapper 071»: «This register is present only on Fire Hawk, which uses
  // the BF9097 IC». FCEUX включает 1-screen по любой записи в $9000, но это
  // компромисс «без submapper»: у Super Robin Hood он ломает и зеркалирование,
  // и номер банка сразу.
  uint8_t cmBank      = 0;                        // 16 КБ банк $8000-$BFFF
  uint8_t cmMirror    = 0;                        // 2/3 = 1-screen младший/старший банк CIRAM
  bool    cmMirrorReg = false;                    // true -> $9000-$9FFF меняет банк CIRAM
  void camericaWrite(uint16_t addr, uint8_t value);
};

// =================== Быстрые обращения к памяти картриджа =====================
// ВАЖНО (скорость): тела этих двух функций живут в заголовке, поэтому GCC
// встраивает их прямо в интерпретатор 6502 и в рендерер PPU.
//   readPrg() — каждое чтение кода и данных из $8000-$FFFF: 2..4 раза на одну
//               инструкцию 6502 (~30 000 раз за кадр). Через границу модуля
//               (в .cpp, без LTO) это был настоящий вызов функции ~35 тактов,
//               то есть несколько миллисекунд на кадр;
//   readChr() — по 2 раза на каждый тайл фона и спрайта (~35 000 раз за кадр).
// Эти вызовы и были основной причиной «игра идёт медленнее приставки» вместе
// с сорванным инлайном на -Og (Tools -> Debug Level = Debug).
DENDY_INLINE uint8_t NesMapper::readPrg(Nes*, uint16_t addr)
{
  if (addr >= 0x6000 && addr < 0x8000) return wram[addr & 0x1FFF];   // WRAM картриджа
  if (addr < 0x8000) return 0;
  return prgBank[(addr >> 13) & 3][addr & 0x1FFF];
}

DENDY_INLINE uint8_t NesMapper::readChr(Nes*, uint16_t addr)
{
  return chrBank[(addr >> 10) & 7][addr & 0x03FF];
}
