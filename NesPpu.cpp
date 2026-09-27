/**
 * =============================================================================
 *  DendyGO / NesPpu.cpp
 *  Построчный рендерер PPU 2C02 в буфер RGB565.
 * =============================================================================
 */
#include <string.h>
#include "DendyFast.h"            // -O3 для горячего кода (см. DendyConfig.h)
#include "NesPpu.h"
#include "NesCore.h"
#include "DendyConfig.h"

// ------------------- Палитра 2C02 (64 цвета) в формате RGB565 -----------------
// Здесь хранятся ЦВЕТА NES (RGB888, как в документации), а не готовые слова:
// слова собираются в palRaw/palFrame в buildPalette(). Так порядок каналов
// можно менять на лету (setColorOrder) без пересборки прошивки.
static constexpr uint16_t rgb565(uint32_t c)
{
  return (uint16_t)((((c >> 19) & 0x1F) << 11) | (((c >> 10) & 0x3F) << 5) | ((c >> 3) & 0x1F));
}

static const uint32_t NES_COLORS[64] = {
  0x7C7C7C, 0x0000FC, 0x0000BC, 0x4428BC, 0x940084, 0xA80020, 0xA81000, 0x881400,
  0x503000, 0x007800, 0x006800, 0x005800, 0x004058, 0x000000, 0x000000, 0x000000,
  0xBCBCBC, 0x0078F8, 0x0058F8, 0x6844FC, 0xD800CC, 0xE40058, 0xF83800, 0xE45C10,
  0xAC7C00, 0x00B800, 0x00A800, 0x00A844, 0x008888, 0x000000, 0x000000, 0x000000,
  0xF8F8F8, 0x3CBCFC, 0x6888FC, 0x9878F8, 0xF878F8, 0xF85898, 0xF87858, 0xFCA044,
  0xF8B800, 0xB8F818, 0x58D854, 0x58F898, 0x00E8D8, 0x787878, 0x000000, 0x000000,
  0xFCFCFC, 0xA4E4FC, 0xB8B8F8, 0xD8B8F8, 0xF8B8F8, 0xF8A4C0, 0xF0D0B0, 0xFCE0A8,
  0xF8D878, 0xD8F878, 0xB8F8B8, 0xB8F8D8, 0x00FCFC, 0xF8D8F8, 0x000000, 0x000000,
};

// Пересборка таблицы палитры под текущий порядок каналов.
void NesPpu::buildPalette()
{
  for (int i = 0; i < 64; ++i) {
    palRaw[i]   = rgb565(NES_COLORS[i]);                              // эталон
    palFrame[i] = dendyColorOrder(palRaw[i], colorOrderMode);         // в кадр
  }
}

// Смена режима «на лету» (0..3, см. DENDY_COLOR_ORDER). Кадр начнёт выходить
// в новых цветах со следующей строки: таблица читается при каждой отрисовке.
void NesPpu::setColorOrder(uint8_t mode)
{
  if (mode > 3) mode = 0;
  colorOrderMode = mode;
  buildPalette();          // всегда: таблица нужна и до первого сброса (лог/меню)
  pairDirty = true;        // и таблицы пар пикселей для рендера фона (см. buildPairTables)
}

// colorOf() — в NesPpu.h (DENDY_INLINE): вызывается по разу на пиксель.

// ================================= Сброс =====================================
void NesPpu::reset(Nes* nes)
{
  (void)nes;
  ctrl = mask = status = 0;
  v = t = 0; fineX = 0; writeToggle = false;
  oamAddr = 0; readBuf = 0;
  scanline = 0; dot = 0; oddFrame = false; frameSerial = 0; sprite0HitDot = -1;
  memset(vram, 0, sizeof(vram));
  memset(oam, 0xFF, sizeof(oam));
  memset(bgOpaque, 0, sizeof(bgOpaque));
  memset(sprOpaque, 0, sizeof(sprOpaque));
  for (int i = 0; i < 0x20; i++) palette[i] = 0x0F;      // «чёрная» начальная палитра
  buildPalette();                                        // таблица 64 цветов (режим цвета)
  pairDirty = true;                                      // палитра сменилась -> пары заново
  if (frame) memset(frame, 0, (size_t)WIDTH * HEIGHT * sizeof(uint16_t));
}

// ============================ Чтение видеопамяти =============================
uint16_t NesPpu::mirrorAddress(Nes* nes, uint16_t addr) const
{
  const uint16_t a     = (uint16_t)((addr - 0x2000) & 0x0FFF);
  const uint16_t table = (uint16_t)(a >> 10);        // 0..3
  const uint16_t off   = (uint16_t)(a & 0x03FF);
  switch (nes->mapper.mirroring) {
    case 1:                                          // вертикальное: 0,2 -> NT0; 1,3 -> NT1
      return (uint16_t)((table & 1) * 0x400 + off);
    case 2: return off;                              // single-screen 0
    case 3: return (uint16_t)(0x400 + off);          // single-screen 1
    case 4:                                          // four-screen (в VRAM только 2 КБ)
    default:                                         // горизонтальное: 0,1 -> NT0; 2,3 -> NT1
      return (uint16_t)((table >> 1) * 0x400 + off);
  }
}

DENDY_FAST_ATTR uint8_t NesPpu::readVram(Nes* nes, uint16_t addr)
{
  addr &= 0x3FFF;
  // ВАЖНО: палитра проверяется ДО свёртки зеркала $3000-$3EFF. Раньше здесь
  // сначала вычиталось 0x1000, и адрес $3F00 превращался в $2F00 — чтение
  // палитры через $2007 возвращало содержимое VRAM (мусор), а не цвет.
  if (addr >= 0x3F00) {                              // палитра
    uint16_t p = (uint16_t)(addr & 0x1F);
    if (p == 0x10 || p == 0x14 || p == 0x18 || p == 0x1C) p -= 0x10;   // зеркала прозрачного цвета
    return palette[p];
  }
  if (addr >= 0x3000) addr -= 0x1000;                // зеркало $3000-$3EFF на $2000-$2EFF
  if (addr < 0x2000) return nes->mapper.readChr(nes, addr);
  return vram[mirrorAddress(nes, addr)];
}

// ============================== Регистры PPU =================================
void NesPpu::writeRegister(Nes* nes, uint8_t reg, uint8_t value)
{
  switch (reg & 7) {
    case 0:                                            // $2000 — управление
      ctrl = value;
      t = (uint16_t)((t & 0xF3FF) | ((value & 0x03) << 10));
      break;

    case 1: mask = value; break;                       // $2001 — маска/эффекты

    case 2: break;                                     // $2002 — только чтение

    case 3: oamAddr = value; break;                    // $2003 — адрес OAM

    case 4: oam[oamAddr++] = value; break;             // $2004 — данные OAM

    case 5:                                            // $2005 — скролл
      if (!writeToggle) {
        t = (uint16_t)((t & 0xFFE0) | (value >> 3));
        fineX = (uint8_t)(value & 0x07);
      } else {
        t = (uint16_t)((t & 0x8C1F) | ((value & 0x07) << 12) | ((value & 0xF8) << 2));
      }
      writeToggle = !writeToggle;
      break;

    case 6:                                            // $2006 — адрес VRAM
      if (!writeToggle) t = (uint16_t)((t & 0x00FF) | ((value & 0x3F) << 8));
      else { t = (uint16_t)((t & 0xFF00) | value); v = t; }
      writeToggle = !writeToggle;
      break;

    case 7:                                            // $2007 — данные VRAM
      { const uint16_t addr = (uint16_t)(v & 0x3FFF);
        if (addr < 0x2000)      nes->mapper.writeChr(nes, addr, value);
        else if (addr < 0x3F00) vram[mirrorAddress(nes, addr)] = value;
        else { uint16_t p = (uint16_t)(addr & 0x1F);
               if (p >= 0x10 && (p & 3) == 0) p -= 0x10;
               palette[p] = value;
               // Палитра изменилась: таблицы пар пикселей (buildPairTables)
               // пересобираются перед следующей отрисовкой строки.
               pairDirty = true; }
        v = (uint16_t)(v + ((ctrl & 0x04) ? 32 : 1)); }
      break;

    default: break;
  }
}

uint8_t NesPpu::readRegister(Nes* nes, uint8_t reg)
{
  (void)nes;
  uint8_t result = readBuf;                            // «открытая шина»
  switch (reg & 7) {
    case 2:                                            // $2002 — статус
      result = (uint8_t)((status & 0xE0) | (readBuf & 0x1F));
      status &= (uint8_t)~0x80;                        // сброс VBlank
      writeToggle = false;
      break;

    case 4: result = oam[oamAddr]; break;              // $2004 — OAM

    case 7: {                                          // $2007 — данные VRAM
      const uint16_t addr  = (uint16_t)(v & 0x3FFF);
      const uint8_t  value = readVram(nes, addr);
      if (addr >= 0x3F00) {                            // палитра читается напрямую
        result  = (uint8_t)((value & 0x3F) | (readBuf & 0xC0));
        readBuf = readVram(nes, (uint16_t)(addr - 0x1000));
      } else {
        result  = readBuf;
        readBuf = value;
      }
      v = (uint16_t)(v + ((ctrl & 0x04) ? 32 : 1)); } break;

    default: break;
  }
  return result;
}

// ============================ DMA спрайтов ($4014) ==========================
void NesPpu::writeOamDma(Nes* nes, uint8_t page)
{
  const uint16_t base = (uint16_t)(page << 8);
  for (int i = 0; i < 256; i++) oam[(oamAddr + i) & 0xFF] = nesRead(nes, (uint16_t)(base + i));
}

// =============== Тайминг: 341x(262 NTSC | 312 PAL) точки =====================
// Последняя строка кадра — DENDY_PPU_PRERENDER (261 в NTSC, 311 в PAL): именно
// на её конце кадр считается готовым, поэтому от неё зависит и число тактов CPU
// на кадр (29780,7 в NTSC против 33247,5 в PAL — см. DENDY_REGION_PAL).
void NesPpu::setVBlank(Nes* nes)
{
  status |= 0x80;                                      // флаг VBlank
  if (ctrl & 0x80) nes->cpu.nmiPending = true;          // NMI, если разрешён
}

DENDY_FAST_ATTR void NesPpu::stepSlow(Nes* nes, uint32_t dots)
{
  while (dots) {
    const uint32_t toEnd  = (uint32_t)(DENDY_PPU_DOTS_PER_LINE - dot);
    const uint32_t chunk  = (dots < toEnd) ? dots : toEnd;
    const uint32_t newDot = (uint32_t)dot + chunk;

    // События внутри строки
    if (scanline == 241 && dot < 2 && newDot >= 2) {
      setVBlank(nes);
    } else if (scanline == prerender && dot < 2 && newDot >= 2) {
      status &= (uint8_t)~0xE0;                        // pre-render: сброс VBlank/overflow/sprite0
    }
    const int hitAt = sprite0HitDot;                   // точка попадания для текущей строки
    if (hitAt > 0 && !(status & 0x40) &&               // флаг ставится один раз за кадр
        (uint32_t)dot < (uint32_t)hitAt && newDot >= (uint32_t)hitAt) {
      status |= 0x40;                                  // sprite-0 hit
      ++sprite0HitCount;                               // счётчик для диагностики
    }

    dot   = (int)newDot;
    dots -= chunk;
    if (dot >= DENDY_PPU_DOTS_PER_LINE) {
      dot -= DENDY_PPU_DOTS_PER_LINE;
      endLine(nes);
    }
  }
}

DENDY_FAST_ATTR void NesPpu::endLine(Nes* nes)
{
  const bool fromPreRender = (scanline == prerender);  // шли по предварительной строке

  if (fromPreRender) {                                 // конец кадра
    scanline = 0;
    oddFrame = !oddFrame;
    // Диагностика sprite-0: фиксируем итоги кадра и обнуляем рабочие счётчики.
    sprite0LinesFrame  = sprite0Lines;
    sprite0MissesFrame = sprite0Misses;
    sprite0Lines  = 0;
    sprite0Misses = 0;
    frameSerial++;                                     // сообщаем главному циклу о готовом кадре
  } else {
    scanline++;
  }

  if (scanline == prerender) {
    // Pre-render: пропуск точки в нечётном кадре и полная перезагрузка скролла из t
    if (renderingEnabled()) {
      if (oddFrame) dot = 1;
      v = t;
    }
  } else if (renderingEnabled() && !fromPreRender && scanline <= HEIGHT) {
    // ВАЖНО (иначе игры, обновляющие VRAM/CHR RAM при включённом рендере,
    // показывают «чёрные дыры» и пропавшие цвета): и вертикальный инкремент, и
    // копирование горизонтали из t делаются ТОЛЬКО на видимых строках.
    // В реальном PPU выборка nametable/атрибутов/образов идёт «на точках 321-336
    // и 1-256 строк 0-239 и 261», строка 240 (post-render) «просто простаивает»,
    // а на строках VBlank 241-260 PPU вообще не обращается к памяти (см. NESdev,
    // «PPU rendering» и «PPU scrolling»). Значит на строках 240-260 регистр v
    // НЕ меняется, и запись $2006/$2007 кладёт байты ровно туда, куда хотела игра.
    // Раньше v двигался на всех строках кадра, и блочная запись, пересекающая
    // конец строки, обрывалась в середине: остаток блока уходил по адресу из t.
    // Так выглядят Battle City (льёт 16 байт палитры — примерно 1,5 строки) и
    // Super Robin Hood (32 байта столбца nametable + потоковые тайлы в CHR RAM):
    // часть цветов не доходила до своих ячеек, часть тайлов не дописывалась.
    // ВАЖНО: на переходе «предварительная строка -> строка 0» вертикальный
    // инкремент НЕ делается — v только что целиком взят из t. Раньше он делался,
    // и весь кадр рисовался со сдвигом на одну строку: фон уезжал на 1 точку
    // вверх относительно спрайтов (и последняя строка бралась из-за экрана).
    // Инкремент вертикали (fine Y -> coarse Y -> переключение nametable) как в PPU
    if ((v & 0x7000) != 0x7000) {
      v = (uint16_t)(v + 0x1000);
    } else {
      v = (uint16_t)(v & ~0x7000);
      uint16_t cy = (uint16_t)((v >> 5) & 0x1F);
      if (cy == 29)      { cy = 0; v ^= 0x0800; }      // переход в соседний nametable
      else if (cy == 31) { cy = 0; }
      else               { cy++; }
      v = (uint16_t)((v & ~0x03E0) | (cy << 5));
    }
    v = (uint16_t)((v & ~0x041F) | (t & 0x041F));       // копирование горизонтали из t
  }

  // Рисуем строку в начале её отображения.
  // ВАЖНО: sprite0HitDot сбрасывается ДО renderLine, потому что renderSprites()
  // именно в нём запоминает точку попадания sprite-0 для этой строки, а step()
  // по ней поднимает флаг $2002.6. Если сбросить после отрисовки (как было
  // раньше), флаг sprite-0 hit не поднимется НИКОГДА, и любая игра с
  // разделением экрана по sprite-0 (Super Mario Bros. и др.) навсегда
  // зависнет в цикле «LDA $2002 / AND #$40 / BEQ».
  sprite0HitDot = -1;
  // ВАЖНО (был «замороженный» кадр ПРОШЛОЙ игры): когда на экране выключены и фон
  // ($2001.3), и спрайты ($2001.4), реальный PPU выводит ЦВЕТ ФОНА ($3F00), а вовсе
  // не то, что было в прошлом кадре. У нас строка в этом случае не писалась ВООБЩЕ
  // (renderLine не звался), и в кадровом буфере оставался старый кадр, который затем
  // честно уходил на экран. Так выглядел запуск ромов, которые первые кадры держат
  // рендер выключенным (загрузка уровня, распаковка CHR): на экране на секунду
  // «мелькала» предыдущая игра — а кадровых буферов три, и NesPpu::reset() чистит
  // только тот, в который PPU пишет сейчас, так что чужой кадр лежал в двух других и
  // ждал своей передачи (см. invalidateFrameBuffers() в скетче — он чистит все три
  // на переходах меню <-> игра).
  if (scanline < HEIGHT) {
    if (renderingEnabled()) renderLine(nes, scanline);    // обычный путь: фон и спрайты
    else                    blankLine(scanline);          // рендер выключен: цвет фона
  }
}

// Строка при ВЫКЛЮЧЕННОМ рендере: ровный цвет фона ($3F00) — как на приставке.
// Заливка нужна не «для красоты»: без неё в кадре остаётся предыдущий кадр (см. вызов
// в step() выше), и он уходит на экран вместо фона. Цена — те же 256 записей на
// строку, что и у обычного рендера, и только на кадрах с выключенным рендером.
void NesPpu::blankLine(int y)
{
  uint16_t* dst = &frame[(size_t)y * WIDTH];
  const uint16_t backdrop = colorOf(palette[0]);
  for (int x = 0; x < WIDTH; ++x) dst[x] = backdrop;
  memset(bgOpaque, 0, sizeof(bgOpaque));      // непрозрачного фона нет (спрайты/скролл)
}

DENDY_FAST_ATTR void NesPpu::renderLine(Nes* nes, int y)
{
#if DENDY_EMU_PROFILE
  const uint32_t profT0 = DENDY_CYCLES();
#endif
  lineDst = &frame[(size_t)y * WIDTH];       // строка пишется ПРЯМО в кадровый буфер
  const uint16_t backdrop = colorOf(palette[0]);
  memset(sprOpaque, 0, sizeof(sprOpaque));

  if (mask & 0x08) {
#if DENDY_EMU_PROFILE
    const uint32_t profBgT0 = DENDY_CYCLES();    // профиль: только фон
#endif
    renderBackground(nes, y);
#if DENDY_EMU_PROFILE
    profBgCycles += DENDY_CYCLES() - profBgT0;
#endif
  } else {
    for (int x = 0; x < WIDTH; x++) { lineDst[x] = backdrop; bgOpaque[x] = 0; }
  }

#if DENDY_EMU_PROFILE
  const uint32_t profSprT0 = DENDY_CYCLES();     // профиль: только спрайты
#endif
  if (mask & 0x10) renderSprites(nes, y);
#if DENDY_EMU_PROFILE
  profSprCycles += DENDY_CYCLES() - profSprT0;
#endif

  // Копирования строки в кадр здесь больше нет: renderBackground() и renderSprites()
  // пишут сразу в кадровый буфер (lineDst — начало строки), поэтому каждый пиксель
  // строки записывается РОВНО один раз. Раньше строка собиралась в SRAM (lineBuf) и
  // копировалась сюда memcpy: 240 копий по 512 байт за кадр — это лишние 61 КБ
  // чтений и 61 КБ записей в SRAM на каждом кадре (и лишний проход по кэшу).

  // MMC3: счётчик строк для IRQ-разбиения экрана
  if (nes->mapper.number == 4) nes->mapper.scanlineTick(nes);
#if DENDY_EMU_PROFILE
  profRenderCycles += DENDY_CYCLES() - profT0;      // сколько тактов ушло на строку
#endif
}

// ------------------- Таблицы «два пикселя фона за раз» -----------------------
// Ключ таблиц: бит 0 — бит плоскости 0 ПРАВОГО пикселя пары, бит 1 — ЛЕВОГО,
// биты 2/3 — то же для плоскости 1. Такой порядок потому, что в байте плоскости
// бит 7 — левый пиксель группы, а бит 6 — правый, и в цикле ключ собирается как
// (a >> 6) | ((b >> 4) & 0x0C) — два бита из каждого байта на своих местах.
// Дальше по таблице: pairCol даёт сразу оба цвета (левый в младших 16 битах,
// правый в старших), pairOpq — флаги непрозрачности фона (для sprite-0 и
// приоритета спрайтов), то есть в самом цикле рендера НЕТ ни веток «прозрачный
// или нет», ни обращений к palette[]/colorOf().
void NesPpu::buildPairTables()
{
  for (int pal = 0; pal < 4; ++pal) {
    // Индекс 0 палитры — всегда фоновый цвет (backdrop), как и было в рендере.
    const uint16_t cc[4] = {
      colorOf(palette[0]),
      colorOf(palette[pal * 4 + 1]),
      colorOf(palette[pal * 4 + 2]),
      colorOf(palette[pal * 4 + 3])
    };
    for (int key = 0; key < 16; ++key) {
      const uint8_t right = (uint8_t)((key & 1) | (((key >> 2) & 1) << 1));
      const uint8_t left  = (uint8_t)(((key >> 1) & 1) | (((key >> 3) & 1) << 1));
      pairCol[pal][key] = (uint32_t)cc[left] | ((uint32_t)cc[right] << 16);
      pairOpq[pal][key] = (uint16_t)((left ? 1u : 0u) | (right ? 0x100u : 0u));
    }
  }
  pairDirty = false;
}

// ============================== Фон (nametable) ==============================
DENDY_FAST_ATTR void NesPpu::renderBackground(Nes* nes, int y)
{
  if (pairDirty) buildPairTables();    // палитра/порядок цветов менялись (см. ниже)
  // Базовая nametable по Y — это бит v.11: он добавляет к позиции 30 тайлов
  // (240 строк). Раньше бит игнорировался, и таблица выбиралась «по факту
  // перехода через 240»: как только игра прокручивала экран на nametable,
  // картинка собиралась из чужой таблицы — экран «дублировался» и рассыпался.
  const uint32_t scrollY  = (uint32_t)(((((v & 0x0800) ? 30u : 0u) + ((v >> 5) & 0x1F)) * 8)
                                     + ((v >> 12) & 7));
  const uint32_t tableV   = (scrollY >= 240) ? 1u : 0u;
  const uint32_t rowInNt  = (scrollY >= 240) ? (scrollY - 240) : scrollY;
  const uint32_t tileRow  = rowInNt >> 3;
  const uint32_t inTileY  = rowInNt & 7;
  const uint16_t patBase  = (ctrl & 0x10) ? 0x1000 : 0x0000;   // таблица образов фона
  const uint16_t backdrop = colorOf(palette[0]);
  const bool clipLeft     = (mask & 0x02) == 0;

  // Базовая nametable по X — бит v.10: при скролле больше 256 точек игра
  // переключает именно этот бит, а coarse X при этом уже обнулён. Раньше бит
  // терялся, и вторая половина «широкого» экрана рисовалась из первой nametable.
  const uint32_t baseX = (v & 0x0400) ? 256u : 0u;

  // --- Всё, что зависит только от строки, считаем один раз -------------------
  // Раньше это пересчитывалось в цикле на каждый тайл (33 раза на строку,
  // ~8000 раз на кадр) — вместе с вызовами readVram() это и было главной
  // стоимостью рендеринга.
  const uint32_t scrollX   = (uint32_t)((v & 0x1F) * 8) + fineX;
  const uint32_t rowBase   = tileRow * 32;                    // строка в nametable
  const uint32_t attrRow   = 0x3C0 + (tileRow >> 2) * 8;      // строка в таблице атрибутов
  const uint8_t  shiftRow  = (uint8_t)((tileRow & 2) << 1);
  uint16_t ntPhysAddr = 0xFFFF;      // кэш: адрес nametable -> смещение в VRAM
  uint16_t ntPhysOff  = 0;
  int      lastAttrAddr = -1;        // кэш: байт атрибутов читаем не чаще 1/4 тайлов
  uint8_t  lastAttr     = 0;

  int x = 0;
  while (x < WIDTH) {
    const uint32_t sx      = baseX + scrollX + (uint32_t)x;
    const uint32_t tableH  = (sx >> 8) & 1u;
    const uint32_t tileCol = (sx & 0xFF) >> 3;
    const uint32_t inTileX = sx & 7;
    const uint16_t ntBase  = (uint16_t)(0x2000 + (tableV * 2 + tableH) * 0x400);

    // Зеркалирование nametable пересчитываем только при смене самой таблицы
    if (ntBase != ntPhysAddr) {
      ntPhysAddr = ntBase;
      ntPhysOff  = mirrorAddress(nes, ntBase);
    }

    // Nametable и таблица атрибутов — прямое чтение VRAM (без readVram: он ещё
    // разбирает палитру и зеркала $3000, что для рендера лишнее).
    const uint16_t tileIdx = vram[(uint16_t)(ntPhysOff + rowBase + tileCol)];
    const int attrAddr = (int)(ntPhysOff + attrRow + (tileCol >> 2));
    if (attrAddr != lastAttrAddr) {           // один байт атрибутов = 4x4 тайла
      lastAttrAddr = attrAddr;
      lastAttr     = vram[attrAddr];
    }
    const uint8_t  pal     = (uint8_t)((lastAttr >> (shiftRow | (uint8_t)(tileCol & 2))) & 3);
    const uint16_t patAddr = (uint16_t)(patBase + tileIdx * 16 + inTileY);
    // Образы тайлов лежат в CHR ($0000-$1FFF) — читаем прямо через маппер,
    // минуя разбор адреса в readVram.
    const uint8_t  plane0  = nes->mapper.readChr(nes, patAddr);
    const uint8_t  plane1  = nes->mapper.readChr(nes, (uint16_t)(patAddr + 8));

    int run = 8 - (int)inTileX;
    if (run > WIDTH - x) run = WIDTH - x;

    // --- Вывод тайла: по ДВА пикселя за проход (см. buildPairTables) ----------
    // Самая горячая точка эмулятора: ~63 000 пикселей за кадр. Цвета и флаги
    // непрозрачности берутся ИЗ ТАБЛИЦ, поэтому в цикле нет ни веток
    // «прозрачный/нет», ни чтения palette[]/colorOf() (раньше они были на каждый
    // пиксель), а обе плоскости читаются одним сдвигом.
    const uint32_t* pc = pairCol[pal];
    const uint16_t* po = pairOpq[pal];
    uint16_t* dp = lineDst + x;
    uint8_t*  op = bgOpaque + x;
    uint8_t   a  = (uint8_t)(plane0 << inTileX);   // плоскость 0: бит 7 — 1-й пиксель
    uint8_t   b  = (uint8_t)(plane1 << inTileX);   // плоскость 1 (inTileX = 0..7)
    int       i  = 0;

    // Левая граница экрана (mask.1 = 0 — первые 8 точек не показываются): в них
    // выводятся только фоновый цвет и ноль в маске непрозрачности. Раньше это
    // проверялось НА КАЖДЫЙ ПИКСЕЛЬ строки (i < clip в цикле пикселей), а граница
    // может попасть и на второй тайл строки (при сдвинутом скролле) — поэтому
    // смещение считаем по абсолютному x и на столько же сдвигаем байты плоскостей.
    if (clipLeft && x < 8) {
      int n = 8 - x;
      if (n > run) n = run;
      for (int k = 0; k < n; k++) { dp[k] = backdrop; op[k] = 0; }
      dp += n;
      op += n;
      a = (uint8_t)(a << n);
      b = (uint8_t)(b << n);
      i = n;
    }

    if (!(a | b)) {
      // Тайл из одних нулей (пустое небо, фон) — частый случай: все пиксели
      // прозрачные, поэтому пишем фон и маску без разбора пар.
      for (; i < run; i++) { *dp++ = backdrop; *op++ = 0; }
    } else {
      for (; i + 2 <= run; i += 2) {
        const uint32_t key = (uint32_t)((a >> 6) | ((b >> 4) & 0x0Cu));
        const uint32_t w   = pc[key];              // цвет левого | цвет правого << 16
        dp[0] = (uint16_t)w;
        dp[1] = (uint16_t)(w >> 16);
        const uint16_t f = po[key];                // 1 = левый непрозрачный, 0x100 = правый
        op[0] = (uint8_t)f;
        op[1] = (uint8_t)(f >> 8);
        dp += 2;
        op += 2;
        a = (uint8_t)(a << 2);                     // к следующей паре пикселей
        b = (uint8_t)(b << 2);
      }
      // Остаток (run нечётный) — один пиксель. Плоскости уже сдвинуты так, что
      // нужный пиксель стоит в бите 7, а бит 6 нулевой (сдвиг затягивает нули),
      // поэтому в ключе он попадает на место левого пикселя пары.
      if (i < run) {
        const uint32_t key = (uint32_t)((a >> 6) | ((b >> 4) & 0x0Cu));
        *dp = (uint16_t)pc[key];
        *op = (uint8_t)po[key];
      }
    }
    x += run;
  }
}

// =============================== Спрайты (OAM) ===============================
DENDY_FAST_ATTR void NesPpu::renderSprites(Nes* nes, int y)
{
  const bool     size16   = (ctrl & 0x20) != 0;
  const uint16_t sprBase  = (ctrl & 0x08) ? 0x1000 : 0x0000;
  const bool     clipLeft = (mask & 0x04) == 0;
  const int      height   = size16 ? 16 : 8;
  bool           overflow = false;
  int            found    = 0;
  int            hitDot   = -1;
  bool           anyOpaque = false;                    // у спрайта 0 есть непрозрачные точки

  for (int i = 0; i < 64; i++) {
    const uint8_t oamY = oam[i * 4 + 0];
    const uint8_t tile = oam[i * 4 + 1];
    const uint8_t attr = oam[i * 4 + 2];
    const uint8_t oamX = oam[i * 4 + 3];
    const int     top  = (int)oamY + 1;                // спрайт виден начиная со строки Y+1

    if (y < top || y >= top + height) continue;
    if (found >= 8) { overflow = true; break; }        // максимум 8 спрайтов в строке
    found++;

    uint16_t patBase = sprBase;
    uint16_t tileIdx = tile;
    if (size16) {                                      // 8x16: младший бит выбирает таблицу
      patBase = (tile & 1) ? 0x1000 : 0x0000;
      tileIdx = (uint16_t)(tile & 0xFE);
    }

    int row = y - top;
    if (attr & 0x80) row = height - 1 - row;           // вертикальный переворот
    const uint16_t baseRow = (uint16_t)(row & 7);
    const uint16_t patAddr = (uint16_t)(patBase + (size16 ? (tileIdx + (row >= 8 ? 1 : 0)) : tileIdx) * 16 + baseRow);
    const uint8_t  plane0  = nes->mapper.readChr(nes, patAddr);
    const uint8_t  plane1  = nes->mapper.readChr(nes, (uint16_t)(patAddr + 8));
    const uint8_t  pal     = (uint8_t)(attr & 3);
    const bool     behind  = (attr & 0x20) != 0;       // 1 -> спрайт за фоном
    const bool     flipH   = (attr & 0x40) != 0;

    for (int p = 0; p < 8; p++) {
      const int sx = (int)oamX + p;
      if (sx < 0 || sx >= WIDTH) continue;
      const uint8_t bit = (uint8_t)(flipH ? p : (7 - p));
      const uint8_t pix = (uint8_t)(((plane0 >> bit) & 1) | (((plane1 >> bit) & 1) << 1));
      if (pix) anyOpaque = true;                       // у спрайта 0 есть непрозрачные пиксели
      if (pix == 0) continue;                          // прозрачный пиксель спрайта
      if (sx < 8 && clipLeft) continue;
      if (i == 0 && bgOpaque[sx] && hitDot < 0) hitDot = sx;   // sprite-0 hit
      if (behind && bgOpaque[sx]) continue;            // спрайт позади фона
      if (sprOpaque[sx]) continue;                     // более ранний спрайт важнее
      sprOpaque[sx] = 1;
      lineDst[sx]   = colorOf(palette[0x10 + pal * 4 + pix]);
    }

    // Диагностика: спрайт 0 есть на строке (anyOpaque) — но перекрытия с
    // непрозрачным фоном не нашлось -> флаг $2002.6 поднять нечем, а игра
    // (Super Mario Bros. и др.) ждёт его в цикле «LDA $2002 / AND #$40».
    if (i == 0 && anyOpaque) {
      ++sprite0Lines;
      if (hitDot < 0) ++sprite0Misses;
    }
  }

  if (overflow) status |= 0x20;                        // флаг переполнения спрайтов
  // +1: ноль нельзя использовать как «нет попадания», а попадание в x=0 допустимо
  if (hitDot >= 0 && !(status & 0x40)) {
    sprite0HitDot    = hitDot + 1;
    sprite0LastHitDot = hitDot;
  }
}
