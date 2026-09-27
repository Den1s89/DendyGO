/**
 * =============================================================================
 *  DendyGO / NesMapper.cpp
 *  Разбор образа .nes и логика банковки картриджа.
 * =============================================================================
 */
#include <string.h>
#include "DendyFast.h"            // -O3 для горячего кода (см. DendyConfig.h)
#include "NesMapper.h"
#include "NesCore.h"

#define KB8  0x2000u
#define KB16 0x4000u

// ------------------------------- Загрузка рома ------------------------------
bool NesMapper::load(uint8_t* romFile, uint32_t fileSize)
{
  if (fileSize < 16 || memcmp(romFile, "NES\x1A", 4) != 0) return false;

  const uint8_t flags6 = romFile[6];
  const uint8_t flags7 = romFile[7];

  // --- Регион из заголовка (см. regionHint в NesMapper.h) ---------------------
  // ВАЖНО: заголовок — это только ПОДСКАЗКА, а не «понимание» рома. Часть дампов
  // PAL-игр вообще не несёт признака (байт 9 = 0), а часть NTSC-игр после
  // «русификации»/хак-сборки наоборот помечена как PAL. Поэтому в меню есть режим
  // TV (AUTO / NTSC / PAL): AUTO берёт то, что написано в заголовке, а если игра
  // идёт не в своём темпе — режим ставят руками (кнопка SX_PIN_REGION в меню).
  {
    const uint8_t timing = (uint8_t)((flags7 & 0x0C) >> 2);
    if (timing == 2) {                             // NES 2.0: тайминг в байте 12
      const uint8_t tm = (uint8_t)(romFile[12] & 0x03);
      regionHint = (tm == 0) ? 0 : 1;              // 0 NTSC, 1 PAL, 2 «оба», 3 Dendy
    } else if ((romFile[10] & 0x03) == 0x02) {     // VS/PlayChoice-образ: 2 = PAL
      regionHint = 1;
    } else {                                       // классический iNES: байт 9, бит 0
      regionHint = (romFile[9] & 0x01) ? 1 : 0;
    }
  }

  number    = (uint8_t)((flags7 & 0xF0) | (flags6 >> 4));
  mirroring = (flags6 & 0x01) ? 1 : 0;          // 0=horizontal, 1=vertical
  if (flags6 & 0x08) mirroring = 4;             // four-screen
  const bool trainer = (flags6 & 0x04) != 0;

  // --- Маппер 71: есть ли регистр 1-screen зеркалирования (см. NesMapper.h) ----
  // Он есть ТОЛЬКО у платы BF9097 (Fire Hawk): в NES 2.0 это submapper 1 (байт 8,
  // старший ниббл; проверяется вместе с признаком NES 2.0 в байте 7, биты 2-3).
  // У всех остальных картриджей Camerica/Codemasters (BF9093/BF9096 — Super Robin
  // Hood, Fantastic Adventures of Dizzy, Bee 52) зеркалирование ЖЁСТКОЕ и берётся
  // из заголовка, а запись в $9000-$9FFF — это обычная смена банка PRG. Раньше мы
  // включали 1-screen по любой записи в $9000: у Super Robin Hood («LDA #$18 /
  // STA $9000» из RAM-трамплина) два nametable схлопывались в один, да ещё и
  // номер банка не менялся — картинка рассыпалась на чёрные куски.
  cmMirrorReg = (number == 71 && (flags7 & 0x0C) == 0x08 &&
                 ((romFile[8] >> 4) & 0x0F) == 1);
  cmMirror    = 0;                              // защёлка CIRAM нового рома сброшена

  const uint32_t prgBytes = (uint32_t)romFile[4] * 16384u;
  const uint32_t chrBytes = (uint32_t)romFile[5] *  8192u;
  const uint32_t offset   = 16u + (trainer ? 512u : 0u);

  if (prgBytes == 0) return false;                          // нет кода игры
  if (offset + prgBytes + chrBytes > fileSize) return false; // файл обрезан

  prg     = romFile + offset;
  prgSize = prgBytes;

  if (chrBytes < 0x2000) {       // CHR ROM нет (или он меньше 8 КБ) -> CHR RAM
    chrIsRam = true;
    memset(chrRam, 0, sizeof(chrRam));
    if (chrBytes) memcpy(chrRam, prg + prgBytes, chrBytes);
    chr     = chrRam;
    chrSize = sizeof(chrRam);
  } else {
    chrIsRam = false;
    chr      = prg + prgBytes;
    chrSize  = chrBytes;
  }

  // Новый ром: содержимое кэша окон недействительно (см. prgWindow/chrWindow).
#if DENDY_CART_IRAM
  for (int i = 0; i < 4; i++) prgCacheBank[i] = -1;
  for (int i = 0; i < 8; i++) chrCacheBank[i] = -1;
#endif

  memset(wram, 0, sizeof(wram));
  reset();                                   // reset() -> configureBanks(): окна + кэш
  return true;
}

bool NesMapper::supported() const
{
  switch (number) {
    case 0: case 1: case 2: case 3: case 4: case 7: case 11: case 23:
    case 66: case 71: return true;
    default: return false;
  }
}

const char* NesMapper::mapperName() const
{
  switch (number) {
    case 0:  return "NROM";
    case 1:  return "MMC1";
    case 2:  return "UxROM";
    case 3:  return "CNROM";
    case 4:  return "MMC3";
    case 7:  return "AxROM";
    case 11: return "ColorDreams";
    case 23: return "VRC2";                       // Konami VRC2b (Contra (J))
    case 66: return "GxROM";
    case 71: return "Camerica";                   // Codemasters (Super Robin Hood)
    default: return "unsupported";
  }
}

void NesMapper::reset()
{
  irqPending = false;
  switch (number) {
    case 1:  mmc1Shift = 0x10; mmc1Ctrl = 0x0C;
             mmc1Chr0 = mmc1Chr1 = mmc1Prg = 0; break;
    case 4:  mmc3Select = mmc3BankSel = 0; mmc3Latch = mmc3Counter = 0;
             mmc3Reload = mmc3IrqEnable = false;
             memset(mmc3Regs, 0, sizeof(mmc3Regs)); break;
    case 23: vrcPrg[0] = vrcPrg[1] = 0;
             memset(vrcChr, 0, sizeof(vrcChr)); break;
    case 71: cmBank = 0; break;                   // cmMirror переживает Reset, как на железе

    default: break;
  }
  configureBanks();
}

// Полное обнуление состояния картриджа (вызывается при выгрузке рома: окна
// банков указывали внутрь освобождаемого буфера).
// ВАЖНО: обнуляем НА МЕСТЕ через memset, а не присваиванием NesMapper() —
// объект занимает ~16 КБ (wram + chrRam по 8 КБ), и его временная копия
// переполняет стек задачи loopTask ("Stack canary watchpoint triggered").
void NesMapper::clear()
{
  memset(this, 0, sizeof(*this));
  regionHint = 2;                          // «регион рома неизвестен» (см. NesMapper.h)
#if DENDY_CART_IRAM
  // Кэш окон пуст: memset обнулил бы «номер банка в кэше» в 0, и нужный банк 0
  // не был бы скопирован (в кэше лежал бы мусор). -1 = «банка в кэше нет».
  for (int i = 0; i < 4; i++) prgCacheBank[i] = -1;
  for (int i = 0; i < 8; i++) chrCacheBank[i] = -1;
#endif
}

// ---------------------- Установка окон банков (хелперы) ---------------------
// ---------------- Окна банков: обычные и с кэшем во внутренней SRAM ----------
// Все окна ставятся ТОЛЬКО через prgWindow()/chrWindow(): так одна точка знания
// решает, читать ли ром из PSRAM напрямую или из копии во внутренней SRAM.
void NesMapper::prgWindow(int slot, uint32_t bank)
{
  if (slot < 0 || slot > 3) return;
  const uint32_t banks = prgSize / KB8;
  if (!banks) return;
  const uint32_t b = bank % banks;
#if DENDY_CART_IRAM
  if (prgCacheBank[slot] != (int32_t)b) {       // банк сменился -> обновить копию
    memcpy(prgCache[slot], prg + b * KB8, KB8);
    prgCacheBank[slot] = (int32_t)b;
    ++prgCopies;
  }
  prgBank[slot] = prgCache[slot];
#else
  prgBank[slot] = prg + b * KB8;
#endif
}

void NesMapper::chrWindow(int slot, uint32_t bank)
{
  if (slot < 0 || slot > 7) return;
  const uint32_t banks = chrSize / 0x400u;
  if (!banks) return;
  const uint32_t b = bank % banks;
#if DENDY_CART_IRAM
  if (!chrIsRam) {                              // CHR RAM игра пишет — кэш недопустим
    if (chrCacheBank[slot] != (int32_t)b) {
      memcpy(chrCache[slot], chr + b * 0x400u, 0x400u);
      chrCacheBank[slot] = (int32_t)b;
      ++chrCopies;
    }
    chrBank[slot] = chrCache[slot];
    return;
  }
#endif
  chrBank[slot] = chr + b * 0x400u;
}

uint32_t NesMapper::cacheCopies(bool reset)
{
  const uint32_t n = prgCopies + chrCopies;
  if (reset) { prgCopies = 0; chrCopies = 0; }
  return n;
}

// Окно считается рабочим, если указывает внутрь своего буфера ИЛИ в кэш
// (в кэше оно верно по построению). Используется страховкой в configureBanks().
bool NesMapper::prgWindowOk(int slot) const
{
  if (slot < 0 || slot > 3) return false;
#if DENDY_CART_IRAM
  if (prgBank[slot] == prgCache[slot]) return true;
#endif
  return prgBank[slot] != nullptr && prgBank[slot] >= prg &&
         prgBank[slot] + KB8 <= prg + prgSize;
}

bool NesMapper::chrWindowOk(int slot) const
{
  if (slot < 0 || slot > 7) return false;
#if DENDY_CART_IRAM
  if (chrBank[slot] == chrCache[slot]) return true;
#endif
  return chrBank[slot] != nullptr && chrBank[slot] >= chr &&
         chrBank[slot] + 0x400u <= chr + chrSize;
}

void NesMapper::setPrg32(uint32_t bank32)
{
  // ВАЖНО: банки считаем по 8 КБ, а не по 32 КБ.
  // У NROM-128 (Battle City, Excitebike, Lode Runner, F-1 Race...) весь PRG —
  // 16 КБ, и раньше prgSize / 0x8000 давало 0 -> функция выходила сразу: окна
  // $8000-$FFFF вообще не выставлялись, и «страховка» в конце configureBanks()
  // раскладывала их как 0,1,1,1 вместо зеркала 0,1,0,1. Процессор по вектору
  // сброса (у Battle City это $C070) уходил в данные вместо кода — чёрный экран.
  const uint32_t banks = prgSize / KB8;
  if (!banks) return;
  const uint32_t base = (bank32 * 4u) % banks;      // в банках по 8 КБ
  for (int i = 0; i < 4; i++) prgWindow(i, base + i);
}

void NesMapper::setPrg8(int slot, uint32_t bank8)
{
  prgWindow(slot, bank8);
}

void NesMapper::setPrg16(int half, uint32_t bank16)
{
  const uint32_t banks = prgSize / KB16;
  if (!banks || half < 0 || half > 1) return;
  const uint32_t base = (bank16 % banks) * 2u;      // в банках по 8 КБ
  prgWindow(half * 2,     base);
  prgWindow(half * 2 + 1, base + 1);
}

void NesMapper::setChr8(uint32_t bank8)
{
  const uint32_t banks = chrSize / KB8;
  if (!banks) return;
  const uint32_t base = (bank8 % banks) * 8u;       // в банках по 1 КБ
  for (int i = 0; i < 8; i++) chrWindow(i, base + i);
}

void NesMapper::setChr1k(int slot, uint32_t bank1k)
{
  chrWindow(slot, bank1k);
}

// ------------------------- Полный пересчёт банков ---------------------------
void NesMapper::configureBanks()
{
  switch (number) {
    // ------------------------- MMC1 (mapper 1) -------------------------
    case 1: {
      const uint8_t  prgMode = (mmc1Ctrl >> 2) & 3;
      const uint32_t banks16 = prgSize / KB16;
      const uint32_t b       = banks16 ? ((mmc1Prg & 0x0F) % banks16) : 0;
      if (prgMode <= 1) {                       // 32 КБ, банк берётся парами
        setPrg32(b & ~1u);
      } else if (prgMode == 2) {                // $8000 фиксирован, $C000 переключается
        setPrg16(0, 0);
        setPrg16(1, b);
      } else {                                  // $8000 переключается, $C000 фиксирован
        setPrg16(0, b);
        setPrg16(1, banks16 - 1);
      }
      const uint32_t chrBanks = chrSize / 0x1000u;   // банки по 4 КБ
      if (chrBanks) {
        if ((mmc1Ctrl & 0x10) == 0) {           // 8 КБ CHR
          const uint32_t base = ((mmc1Chr0 & 0x1E) % chrBanks) * 4u;    // в банках 1 КБ
          for (int i = 0; i < 8; i++) chrWindow(i, base + i);
        } else {                                // два независимых 4 КБ банка
          const uint32_t b0 = (mmc1Chr0 % chrBanks) * 4u;    // в банках 1 КБ
          const uint32_t b1 = (mmc1Chr1 % chrBanks) * 4u;
          for (int i = 0; i < 4; i++) chrWindow(i, b0 + i);
          for (int i = 4; i < 8; i++) chrWindow(i, b1 + (i - 4));
        }
      }
      const uint8_t m = mmc1Ctrl & 3;           // зеркалирование
      mirroring = (m == 0) ? 2 : (m == 1) ? 3 : (m == 2) ? 1 : 0;
      break;
    }

    // ------------------------- MMC3 (mapper 4) -------------------------
    case 4: {
      const uint32_t banks8 = prgSize / KB8;
      const uint32_t last  = banks8 ? banks8 - 1 : 0;          // последний банк 8 КБ (векторы/NMI)
      const uint32_t prev8 = banks8 >= 2 ? banks8 - 2 : 0;     // предпоследний (фиксированное окно)
      const uint32_t r6    = banks8 ? (mmc3Regs[6] % banks8) : 0;
      const uint32_t r7    = banks8 ? (mmc3Regs[7] % banks8) : 0;
      // Окна PRG ROM по режиму из бита 6 записи в $8000:
      //   mode 1 (бит 6 = 1): $8000 = предпоследний банк, $A000 = R7,
      //                       $C000 = R6,                $E000 = последний банк;
      //   mode 0 (бит 6 = 0): $8000 = R6,                $A000 = R7,
      //                       $C000 = предпоследний банк, $E000 = последний банк.
      // ВАЖНО: раньше в mode 0 окна $A000 и $C000 были ПЕРЕПУТАНЫ (предпоследний
      // банк уходил в $A000, а R7 — в $C000). В mode 0 окно $C000-$DFFF на
      // настоящей MMC3 всегда фиксировано предпоследним банком, и именно там у
      // большинства игр лежат игровые циклы и обработчики. С перепутанными
      // окнами код игры выполнялся из ЧУЖОГО банка: процессор уходил в данные,
      // стек «убегал» (SP=02), игра сыпала сотни записей в $8000-$FFFF за кадр,
      // переставала опрашивать джойстик и включать музыку, а на экране был мусор
      // (так выглядели Adventure Island III и Felix the Cat).
      if (mmc3BankSel & 0x40) {                 // PRG mode 1
        setPrg8(0, prev8);
        setPrg8(1, r7);
        setPrg8(2, r6);
        setPrg8(3, last);
      } else {                                  // PRG mode 0
        setPrg8(0, r6);
        setPrg8(1, r7);
        setPrg8(2, prev8);
        setPrg8(3, last);
      }
      if (mmc3BankSel & 0x80) {                 // CHR: режим B
        setChr1k(0, mmc3Regs[2]); setChr1k(1, mmc3Regs[3]);
        setChr1k(2, mmc3Regs[4]); setChr1k(3, mmc3Regs[5]);
        setChr1k(4, mmc3Regs[0] & 0xFE); setChr1k(5, (mmc3Regs[0] & 0xFE) + 1);
        setChr1k(6, mmc3Regs[1] & 0xFE); setChr1k(7, (mmc3Regs[1] & 0xFE) + 1);
      } else {                                  // CHR: режим A
        setChr1k(0, mmc3Regs[0] & 0xFE); setChr1k(1, (mmc3Regs[0] & 0xFE) + 1);
        setChr1k(2, mmc3Regs[1] & 0xFE); setChr1k(3, (mmc3Regs[1] & 0xFE) + 1);
        setChr1k(4, mmc3Regs[2]); setChr1k(5, mmc3Regs[3]);
        setChr1k(6, mmc3Regs[4]); setChr1k(7, mmc3Regs[5]);
      }
      // ВАЖНО: зеркалирование VRAM у MMC3 задаётся ТОЛЬКО записью в $A000,
      // а не регистрами банков. Раньше здесь стояло
      //   mirroring = (mmc3Regs[6] & 1) ? 0 : 1;
      // и любая запись $8001 (смена PRG/CHR банка) перебивала настройку из
      // $A000 значением, взятым из номера банка — картинка ломалась.
      break;
    }

    // ------------------------- UxROM (mapper 2) -------------------------
    case 2: {
      const uint32_t banks16 = prgSize / KB16;
      if (banks16 <= 1) {                       // ром 16 КБ — оба окна одинаковы
        setPrg16(0, 0);
        setPrg16(1, 0);
      } else {
        setPrg16(1, banks16 - 1);               // $C000 всегда последний банк
        // окно $8000 устанавливается в writePrg(), при reset() — банк 0
        setPrg16(0, 0);
      }
      break;
    }

    // ------------------------- CNROM / AxROM / прочие -------------------
    case 3:                                       // CNROM: PRG фиксирован
    case 7:                                       // AxROM: окна задаёт writePrg()
    case 11:
    case 66: {
      if (!prgWindowOk(0)) setPrg32(0);
      break;
    }

    // ------------------------- VRC2b (маппер 23) ------------------------
    case 23: {
      // $8000-$9FFF и $A000-$BFFF — по 8 КБ (номера берутся из регистров),
      // а $C000-$FFFF — ВСЕГДА последние 16 КБ рома: там же лежат векторы
      // сброса/NMI/IRQ и весь «движок» игры (у Contra (J) — $C000-$FFFF).
      const uint32_t banks8 = prgSize / KB8;
      setPrg8(0, vrcPrg[0]);
      setPrg8(1, vrcPrg[1]);
      setPrg8(2, banks8 > 2 ? banks8 - 2 : 0);
      setPrg8(3, banks8 > 1 ? banks8 - 1 : 0);
      for (int i = 0; i < 8; i++) setChr1k(i, vrcChr[i]);
      // Раскладку VRAM маппер не задаёт: её выставляет vrc2Write() ($9000).
      break;
    }

    // --------------------- Codemasters (маппер 71) ----------------------
    case 71: {
      const uint32_t banks16 = prgSize / KB16;
      setPrg16(0, cmBank);                          // $8000-$BFFF: переключаемый
      setPrg16(1, banks16 ? banks16 - 1 : 0);       // $C000-$FFFF: последние 16 КБ
      setChr8(0);                                   // CHR RAM (8 КБ), банков нет
      if (cmMirror) mirroring = cmMirror;           // Fire Hawk: 1-screen
      break;
    }

    // ------------------------- NROM (mapper 0) --------------------------
    default: {
      setPrg32(0);
      setChr8(0);
      break;
    }
  }

  // Страховка: окна не должны выходить за пределы ROM
  const uint32_t pBanks = prgSize / KB8;
  for (int i = 0; i < 4 && pBanks; i++) {
    // Зеркало как у NROM-128 (Battle City): окна идут 0,1,0,1, то есть i % pBanks
    if (!prgWindowOk(i)) prgWindow(i, (uint32_t)i % pBanks);
  }
  const uint32_t cBanks = chrSize / 0x400u;
  for (int i = 0; i < 8 && cBanks; i++) {
    if (!chrWindowOk(i)) chrWindow(i, (uint32_t)i % cBanks);
  }
}

// ------------------------------ Чтение PRG/CHR ------------------------------
// readPrg() и readChr() вынесены в заголовок (NesMapper.h) — они вызываются по
// несколько раз на каждую инструкцию 6502 и на каждый тайл экрана, и вызов через
// границу модуля съедал миллисекунды на кадр (см. пояснение в NesMapper.h).


void NesMapper::writeChr(Nes*, uint16_t addr, uint8_t value)
{
  if (chrIsRam) chrBank[(addr >> 10) & 7][addr & 0x03FF] = value;    // CHR ROM только для чтения
}

// ------------------------------ Запись PRG -----------------------------------
DENDY_FAST_ATTR void NesMapper::writePrg(Nes* nes, uint16_t addr, uint8_t value)
{
  if (addr >= 0x6000 && addr < 0x8000) { wram[addr & 0x1FFF] = value; return; }
  if (addr < 0x8000) return;

  switch (number) {
    case 1:  mmc1Write(addr, value); break;

    case 2: {                                        // UxROM: банк $8000-$BFFF
      const uint32_t banks16 = prgSize / KB16;
      if (banks16) setPrg16(0, value % banks16);
      break;
    }

    case 3:  setChr8(value & 0x03); break;           // CNROM: 8 КБ CHR

    case 4:  mmc3Write(addr, value); break;          // MMC3

    case 7: {                                        // AxROM: 32 КБ + зеркалирование
      setPrg32(value & 0x0F);
      mirroring = (value & 0x10) ? 3 : 2;            // single-screen
      break;
    }

    case 11: {                                       // Color Dreams
      setPrg32(value & 0x0F);
      setChr8((value >> 4) & 0x0F);
      break;
    }

    case 66: {                                       // GxROM
      setPrg32((value >> 4) & 0x03);
      setChr8(value & 0x03);
      break;
    }

    case 23: vrc2Write(addr, value); break;          // VRC2b: PRG ($8000/$A000),
                                                     // CHR ($B000-$E003), VRAM ($9000)
    case 71: camericaWrite(addr, value); break;      // Camerica: банк $8000-$BFFF,
                                                     // $9000-$9FFF — тоже номер банка
                                                     // (1-screen только у Fire Hawk)

    default: break;                                  // NROM: запись в ROM игнорируется
  }
}

// -------------------------------- MMC1 --------------------------------------
void NesMapper::mmc1Write(uint16_t addr, uint8_t value)
{
  if (value & 0x80) {                    // сброс последовательного регистра
    mmc1Shift = 0x10;
    mmc1Ctrl |= 0x0C;                    // PRG в режиме 3 (32 КБ)
    configureBanks();
    return;
  }

  const bool complete = (mmc1Shift & 1) != 0;
  mmc1Shift = (uint8_t)((mmc1Shift >> 1) | ((value & 1) << 4));

  if (complete) {
    const uint8_t data = mmc1Shift & 0x1F;
    if      (addr < 0xA000) mmc1Ctrl = data;    // $8000-$9FFF: управление
    else if (addr < 0xC000) mmc1Chr0 = data;    // $A000-$BFFF: CHR банк 0
    else if (addr < 0xE000) mmc1Chr1 = data;    // $C000-$DFFF: CHR банк 1
    else                    mmc1Prg  = data;    // $E000-$FFFF: PRG банк
    mmc1Shift = 0x10;                           // регистр готов к новой последовательности
    configureBanks();
  }
}

// -------------------------------- MMC3 --------------------------------------
void NesMapper::mmc3Write(uint16_t addr, uint8_t value)
{
  switch (addr & 0xE001) {
    case 0x8000: mmc3BankSel = value;                            // биты 6/7 — режимы PRG/CHR
                 mmc3Select  = value & 0x07;                     // номер регистра R0..R7
                 break;
    case 0x8001: mmc3Regs[mmc3Select] = value; configureBanks(); break;
    // $A000: бит 0 = 0 — вертикальная раскладка nametable (0 и 2 -> NT0, 1 и 3 -> NT1),
    //        бит 0 = 1 — горизонтальная. В нашей нотации 1 = вертикальная, 0 = горизонтальная,
    //        поэтому значение ИНВЕРТИРУЕТСЯ. Раньше стояло наоборот, и картинка
    //        MMC3-игр собиралась из «не тех» nametable (экран рвался по вертикали).
    case 0xA000: mirroring = (value & 1) ? 0 : 1; break;
    case 0xA001: break;                                          // защита WRAM — игнорируем
    case 0xC000: mmc3Latch = value; break;                       // значение перезагрузки счётчика
    case 0xC001: mmc3Reload = true; break;                       // перезагрузить при следующей строке
    case 0xE000: mmc3IrqEnable = false; irqPending = false; break;// запрет IRQ + сброс
    case 0xE001: mmc3IrqEnable = true; break;                    // разрешение IRQ
    default: break;
  }
}

// ------------------------------ VRC2b (маппер 23) ---------------------------
// Регистры Konami VRC2b. Линии A0/A1 выбирают регистр ВНУТРИ блока, поэтому
// блок определяется старшим нибблом адреса, а номер регистра — младшими 2 битами:
//   $8000-$8FFF : PRG банк 0 (8 КБ в $8000-$9FFF)
//   $9000-$9FFF : раскладка nametable (VRAM)
//   $A000-$AFFF : PRG банк 1 (8 КБ в $A000-$BFFF)
//   $B000-$BFFF : CHR банки 0 и 1 (по 1 КБ)
//   $C000-$CFFF : CHR банки 2 и 3
//   $D000-$DFFF : CHR банки 4 и 5
//   $E000-$EFFF : CHR банки 6 и 7
//   $F000-$FFFF : IRQ (у VRC2 его нет — игнорируем)
// Внутри пары CHR-банков ЧЁТНЫЙ адрес пишет младший ниббл номера банка,
// НЕЧЁТНЫЙ — старший; у VRC2 старших бит только 4 (бит 4 нечётного регистра
// не подключён). $C000-$FFFF всегда указывают на последние 16 КБ рома.
void NesMapper::vrc2Write(uint16_t addr, uint8_t value)
{
  switch (addr & 0xF000) {
    case 0x8000:                                   // PRG банк в $8000-$9FFF
      vrcPrg[0] = (uint8_t)(value & 0x1F);
      break;

    case 0xA000:                                   // PRG банк в $A000-$BFFF
      vrcPrg[1] = (uint8_t)(value & 0x1F);
      break;

    case 0x9000: {                                 // раскладка nametable
      // 0 = vertical (CIRAM A10 = PPU A10), 1 = horizontal (A11),
      // 2 = 1-screen младший банк CIRAM, 3 = 1-screen старший.
      // $FF игнорируем: так пишет Wai Wai World, и без этого игра получает
      // «не ту» раскладку (см. NESdev: VRC2_and_VRC4).
      if (value != 0xFF) {
        const uint8_t m = (uint8_t)(value & 3);
        mirroring = (m == 0) ? 1 : (m == 1) ? 0 : (m == 2) ? 2 : 3;
      }
      return;                                      // окна банков не менялись
    }

    case 0xB000: case 0xC000: case 0xD000: case 0xE000: {   // 8 окон по 1 КБ CHR
      const uint8_t r    = (uint8_t)(addr & 3);    // номер регистра в блоке
      // база = 0,2,4,6 (по два окна на блок) + 0/1 внутри пары
      const uint8_t slot = (uint8_t)((((uint32_t)(addr & 0xF000) - 0xB000u) >> 11) + (r >> 1));
      if (r & 1) vrcChr[slot] = (uint8_t)((vrcChr[slot] & 0x0F) | ((value & 0x0F) << 4));
      else       vrcChr[slot] = (uint8_t)((vrcChr[slot] & 0xF0) |  (value & 0x0F));
      break;
    }

    default: return;                               // $F000-$FFFF: у VRC2 нет IRQ
  }
  configureBanks();
}

// -------------------- Codemasters / Camerica (маппер 71) --------------------
// Клон UNROM: 16 КБ банк в $8000-$BFFF, последние 16 КБ рома в $C000-$FFFF,
// CHR ROM у этих картриджей нет (стоит 8 КБ CHR RAM, без банковки).
// ВАЖНО: номер 16 КБ банка PRG пишется в ЛЮБОЙ адрес $8000-$FFFF, включая
// $9000-$9FFF. Так делает Super Robin Hood: банк меняется из RAM-трамплина
// («LDA #$18 / STA $9000»), потому что код, который эту запись выполняет, живёт в
// самом переключаемом окне $8000-$BFFF. $18 — это номер банка движка Dizzy
// (те же номера: $C016←$0C, $C017←$0D, $C018←$0E); для 64 КБ рома номер берётся
// по модулю 4. Регистр 1-screen зеркалирования в $9000-$9FFF есть ТОЛЬКО у платы
// BF9097 (Fire Hawk) — по NESdev «INES Mapper 071»: «This register is present
// only on Fire Hawk, which uses the BF9097 IC». Поэтому переключаем CIRAM лишь
// тогда, когда об этом сказал NES 2.0 submapper 1 (cmMirrorReg, см. load()):
// иначе «одноэкранный» режим включался бы от обычной записи банка, оба nametable
// сливались бы в один, и половина экрана получалась бы чёрной.
void NesMapper::camericaWrite(uint16_t addr, uint8_t value)
{
  if (cmMirrorReg && (addr & 0xF000) == 0x9000) {   // плата BF9097 (Fire Hawk)
    cmMirror  = (uint8_t)(2 + ((value >> 4) & 1));  // 2 = младший банк CIRAM, 3 = старший
    mirroring = cmMirror;                          // 1-screen: PPU читает поле сразу
    return;                                        // так же поступает FCEUX (map71)
  }
  cmBank = value;                                  // 16 КБ банк $8000-$BFFF
  configureBanks();
}

DENDY_FAST_ATTR void NesMapper::scanlineTick(Nes*)
{
  if (number != 4) return;                       // IRQ по строкам нужен только MMC3
  if (mmc3Counter == 0 || mmc3Reload) {
    mmc3Counter = mmc3Latch;
    mmc3Reload  = false;
    if (mmc3IrqEnable && mmc3Latch == 0) irqPending = true;
  } else {
    if (--mmc3Counter == 0 && mmc3IrqEnable) irqPending = true;
  }
}
