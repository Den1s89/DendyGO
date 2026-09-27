/**
 * =============================================================================
 *  DendyGO / NesPpu.h
 *  PPU 2C02: рендеринг построчно (scanline renderer) — быстрый и стабильный
 *  режим для ESP32. Кадр NES 256x240 заполняется в буфер RGB565 в PSRAM.
 *
 *  Реализовано: регистры $2000-$2007, loopy-скролл, зеркалирование по
 *  мапперу, 8x8/8x16 спрайты, приоритеты, sprite-0 hit, overflow,
 *  VBlank/NMI, чётный/нечётный кадр, DMA OAM ($4014).
 * =============================================================================
 */
#pragma once
#include <stdint.h>
#include "DendyConfig.h"          // DENDY_COLOR_ORDER (режим по умолчанию)
#include "DendyFast.h"            // DENDY_FAST_ATTR (IRAM для горячего кода)
struct Nes;

// ------------------- Перекодировка 16-битного цвета ---------------------------
// В кадровом буфере цвет всегда правильный RGB565. Если конкретная панель (или
// путь SPI-DMA) принимает слово иначе, цвета на экране «плывут»: небо $22
// (голубое) становится зелёным, а бежевый $36 — розовым. Компенсация делается
// перекодировкой палитры (DENDY_COLOR_ORDER в DendyConfig.h):
//   0 — как есть, 1 — переставить байты слова, 2 — поменять поля G<->B,
//   3 — поменять поля R<->B («BGR»). Все преобразования — самобратные.
// Режим НЕ прибит к сборке: setColorOrder() пересобирает таблицу на лету
// (в скетче это SELECT+LEFT/RIGHT как в игре, так и в меню), поэтому нужное
// значение подбирается за пару секунд и потом вписывается в DendyConfig.h.
static constexpr uint16_t dendyColorOrder(uint16_t c, uint8_t mode)
{
  return mode == 1 ? (uint16_t)((c << 8) | (c >> 8))
       : mode == 2 ? (uint16_t)(((c >> 11) << 11) | (((c & 0x1F) << 1) << 5) |
                                (((c >> 5) & 0x3F) >> 1))
       : mode == 3 ? (uint16_t)(((c & 0x001F) << 11) | (c & 0x07E0) | ((c & 0xF800) >> 11))
       : c;
}

class NesPpu {
public:
  static const int WIDTH  = 256;
  static const int HEIGHT = 240;

  uint16_t* frame = nullptr;      // 256*240 пикселей RGB565 (выделяется в PSRAM)
  int   frameSerial = 0;          // счётчик кадров (увеличивается по завершении кадра)

  // Профиль (DENDY_EMU_PROFILE): такты CPU, потраченные на отрисовку строк.
  // Скетч вычитает это из общего времени кадра — сразу видно, сколько времени
  // съедает рендер PPU, а сколько ядро 6502 и APU (см. строку [GAME]).
  uint32_t profRenderCycles = 0;
  uint32_t profBgCycles = 0;      // из них — фон (renderBackground)
  uint32_t profSprCycles = 0;     // из них — спрайты (renderSprites)

  void reset(Nes* nes);
  void writeRegister(Nes* nes, uint8_t reg, uint8_t value);
  uint8_t readRegister(Nes* nes, uint8_t reg);
  // --- Регион (NTSC/PAL): выбирается в меню скетча (NesCore::setRegion) --------
  // От региона зависит ПОСЛЕДНЯЯ (предрисуночная) строка кадра: 261 в NTSC
  // (262 строки по 341 точке) и 311 в PAL (312 строк). Конец кадра PPU отсчитывает
  // именно от неё, поэтому она задаёт и число строк, и частоту кадров.
  // DENDY_PPU_PRERENDER осталась значением по умолчанию (регион из DendyConfig.h),
  // а в горячую проверку step() идёт ЭТО поле: обращение к уже загруженному
  // объекту, зато PAL больше не требует пересборки прошивки.
  void setRegion(bool pal) { prerender = (pal ? DENDY_PAL_LINES : DENDY_NTSC_LINES) - 1; }
  int  prerenderLine() const { return prerender; }
  // Продвинуть PPU на dots точек (1 такт CPU = 3 точки). Вызывается на КАЖДУЮ
  // инструкцию 6502 — десятки тысяч раз за кадр, поэтому здесь быстрый путь:
  // обычный случай «строка не закончилась и событий в ней нет» = одна запись в
  // dot. Всё остальное (VBlank на 241-й строке, сброс флагов на предпоследней
  // строке кадра — DENDY_PPU_PRERENDER, sprite-0 hit, конец строки с отрисовкой)
  // — в stepSlow().
  DENDY_INLINE void step(Nes* nes, uint32_t dots)
  {
    const uint32_t nd = (uint32_t)dot + (uint32_t)dots;
    if (nd < (uint32_t)DENDY_PPU_DOTS_PER_LINE && sprite0HitDot <= 0 && scanline != 241 &&
        scanline != prerender) {
      dot = (int)nd;
      return;
    }
    stepSlow(nes, dots);
  }
  DENDY_FAST_ATTR void stepSlow(Nes* nes, uint32_t dots);
  DENDY_FAST_ATTR uint8_t readVram(Nes* nes, uint16_t addr);

  // $4014 — быстрый DMA 256 байт страницы памяти в OAM
  void writeOamDma(Nes* nes, uint8_t page);

  // --- Регистры только для чтения (диагностика видеотракта в скетче) ---
  uint8_t ctrlReg()   const { return ctrl; }     // $2000
  uint8_t maskReg()   const { return mask; }     // $2001 (биты 3/4 — фон/спрайты)
  uint8_t statusReg() const { return status; }   // $2002
  uint32_t sprite0Count() const { return sprite0HitCount; }  // поднятий флага sprite-0

  // Диагностика sprite-0: сразу видно, ПОЧЕМУ флаг не поднимается —
  // спрайт 0 вообще не на экране (0 строк) или на строках есть, а фон под ним
  // прозрачный (тогда строки идут в miss, а попаданий нет). Значения относятся
  // к последнему ЗАВЕРШЁННОМУ кадру.
  int      sprite0LineCount() const { return sprite0LinesFrame; }   // строк кадра со спрайтом 0
  int      sprite0MissCount() const { return sprite0MissesFrame; }  // из них без попадания в фон
  int      sprite0LastDot()   const { return sprite0LastHitDot; }   // x последнего попадания
  const uint8_t* oamPtr()     const { return oam; }             // 256 байт OAM
  uint16_t vAddr()            const { return v; }               // текущий адрес loopy-v
  uint16_t tAddr()            const { return t; }               // адрес loopy-t (скролл)
  uint8_t  fineXReg()         const { return fineX; }           // тонкий X ($2005.2..0)

  // --- Порядок каналов цвета (см. DENDY_COLOR_ORDER в DendyConfig.h) -----------
  // setColorOrder() пересобирает таблицу палитры на лету — можно подобрать режим
  // по картинке, не пересобирая прошивку (в скетче: SELECT+LEFT/RIGHT).
  void     setColorOrder(uint8_t mode);                 // 0..3
  uint8_t  colorOrder() const { return colorOrderMode; }
  // Логический RGB565 цвета (эталон: ровно то же значение, что понимает
  // LovyanGFX в fillRect/drawString). Скетч рисует им «эталонную» полосу рядом
  // с полосой из кадрового буфера — совпали, значит режим подобран верно.
  uint16_t paletteRaw(uint8_t index) const { return palRaw[index & 0x3F]; }

private:
  // --- Память PPU ---
  uint8_t vram[0x800];            // 2 КБ VRAM (2 nametable)
  uint8_t palette[0x20];          // 32 байта палитры
  uint8_t oam[0x100];             // 256 байт OAM
  uint8_t oamAddr = 0;
  uint8_t readBuf = 0;            // буфер чтения $2007

  // --- Регистры ---
  uint8_t ctrl = 0, mask = 0, status = 0;
  uint16_t v = 0, t = 0;
  uint8_t fineX = 0;
  bool writeToggle = false;

  // --- Тайминг ---
  int  prerender = DENDY_PPU_PRERENDER;   // последняя строка кадра (241 NTSC, 311 PAL)
  int  scanline = 0;              // 0..prerender (последняя — pre-render)
  int  dot = 0;                   // 0..340
  bool oddFrame = false;
  int  sprite0HitDot = -1;        // точка (x+1), где поднять флаг sprite-0 hit (-1 = нет)
  uint32_t sprite0HitCount = 0;   // сколько раз поднимали флаг sprite-0 (диагностика)
  int  sprite0Lines        = 0;   // строк в кадре, где спрайт 0 вообще присутствует
  int  sprite0Misses       = 0;   // из них — где перекрытия с непрозрачным фоном нет
  int  sprite0LinesFrame   = 0;   // то же по последнему завершённому кадру (для лога)
  int  sprite0MissesFrame  = 0;
  int  sprite0LastHitDot   = -1;  // x последнего попадания sprite-0 (диагностика)
  bool renderingEnabled() const { return (mask & 0x18) != 0; }

  // --- Отрисовка строки ---
  // Куда пишется ТЕКУЩАЯ строка: указывает внутрь кадрового буфера (frame + y*256,
  // см. renderLine). Отдельного lineBuf в SRAM больше нет: раньше строка собиралась
  // в него и копировалась memcpy в кадр (240 копий по 512 байт за кадр), то есть
  // каждый пиксель писался дважды.
  uint16_t* lineDst = nullptr;
  uint8_t  bgOpaque[WIDTH];       // маска непрозрачных пикселей фона (для sprite-0)
  uint8_t  sprOpaque[WIDTH];      // маска пикселей, уже занятых спрайтами (приоритет)
  DENDY_FAST_ATTR void renderLine(Nes* nes, int y);
  // Строка при ВЫКЛЮЧЕННОМ рендере (сняты обе маски $2001: и фон, и спрайты):
  // реальный PPU выводит на экран цвет фона, поэтому строку надо залить им, а не
  // оставлять в кадре то, что было раньше (см. blankLine в NesPpu.cpp).
  void blankLine(int y);
  DENDY_FAST_ATTR void renderBackground(Nes* nes, int y);
  DENDY_FAST_ATTR void renderSprites(Nes* nes, int y);
  DENDY_FAST_ATTR void endLine(Nes* nes);
  void setVBlank(Nes* nes);

  // --- Скорость фона: таблицы «два пикселя за раз» (см. renderBackground) ------
  // Раньше на каждый из ~63 000 пикселей кадра шли: два сдвига плоскостей, выбор
  // индекса, ветка «прозрачный/нет» и ДВА набора записей (кадр + bgOpaque).
  // Теперь пиксели берутся парами: ключ = (2 бита плоскости 0) | (2 бита
  // плоскости 1 << 2) — всего 16 вариантов на палитру, а таблица сразу даёт
  // 4 байта цвета (левый пиксель | правый << 16) и 2 байта флагов непрозрачности
  // фона (1 = левый, 0x100 = правый) — ни веток, ни чтения палитры в цикле.
  // Таблицы собираются только при смене палитры ($2007 в палитру) или порядка
  // цветов, а не на каждый тайл и не на каждый пиксель.
  uint32_t pairCol[4][16];        // цвета двух пикселей (в кадровом порядке)
  uint16_t pairOpq[4][16];        // флаги непрозрачности: 1 = левый, 0x100 = правый
  bool     pairDirty = true;      // таблицы надо пересобрать
  void buildPairTables();

  // --- Таблица палитры (64 цвета NES) -----------------------------------------
  // palRaw   — логический RGB565 (эталон, как у LovyanGFX);
  // palFrame — то же, но с перекодировкой dendyColorOrder() под конкретную панель:
  //            именно эти слова кладутся в кадровый буфер и уходят в pushImage().
  uint16_t palRaw[64];
  uint16_t palFrame[64];
  uint8_t  colorOrderMode = (uint8_t)DENDY_COLOR_ORDER;
  void buildPalette();

  // Цвет палитры кадра: самое частое обращение рендерера (по разу на пиксель),
  // поэтому тело — здесь, в заголовке (DENDY_INLINE = всегда встраивать).
  DENDY_INLINE uint16_t colorOf(uint8_t paletteIndex) const { return palFrame[paletteIndex & 0x3F]; }
  uint16_t mirrorAddress(Nes* nes, uint16_t addr) const;
};
