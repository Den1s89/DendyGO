/**
 * =============================================================================
 *  DendyGO / NesCpu.h
 *  Ядро 6502 (Ricoh 2A03) — точная реализация всех официальных инструкций
 *  с корректным подсчётом тактов. Неофициальные инструкции трактуются как NOP
 *  с правильной длиной (этого достаточно для 99% игр).
 * =============================================================================
 */
#pragma once
#include <stdint.h>
#include "DendyFast.h"                // DENDY_FAST_ATTR (IRAM для горячего кода)
struct Nes;

class NesCpu {
public:
  // Регистры
  uint8_t  a = 0, x = 0, y = 0, sp = 0xFD;
  uint8_t  p = 0x24;              // флаги: NV-BDIZC (бит 5 всегда 1)
  uint16_t pc = 0;
  uint32_t cycles = 0;            // общий счётчик тактов CPU (для синхронизации PPU/APU)

  // Линии прерываний
  bool irqLine    = false;        // запрос IRQ (APU, маппер)
  bool nmiPending = false;        // запрос NMI (PPU)

  // --- Диагностика ядра (печатается скетчем при DENDY_VIDEO_TRACE) ---
  // Счётчики и кольцевая трасса адресов инструкций: по ним сразу видно,
  // крутится ли CPU в обработчике прерываний (шторм IRQ) или просто ждёт.
  static const int TRACE_LEN = 16;
  uint32_t irqCount = 0;                     // сколько IRQ принято
  uint32_t nmiCount = 0;                     // сколько NMI принято
  uint32_t instrCount = 0;                   // сколько инструкций выполнено
  uint16_t pcTrace[TRACE_LEN] = {0};         // последние адреса инструкций
  uint8_t  pcTraceIdx = 0;                   // следующая позиция кольца

  void reset(Nes* nes);
  DENDY_FAST_ATTR void step(Nes* nes);   // выполнить одну инструкцию (cycles увеличиваются)

private:
  // --- Вспомогательные функции ---
  // DENDY_INLINE (= always_inline), а не просто inline: эти хелперы вызываются
  // 3..6 раз на инструкцию 6502, и без принудительного встраивания каждое
  // обращение к памяти становится настоящим вызовом функции (см. NesCpu.cpp).
  DENDY_INLINE uint8_t  rd(Nes* nes, uint16_t addr);
  DENDY_INLINE void     wr(Nes* nes, uint16_t addr, uint8_t value);
  DENDY_INLINE uint8_t  fetch8(Nes* nes);
  DENDY_INLINE uint16_t fetch16(Nes* nes);

  // Режимы адресации: возвращают исполнительный адрес, pageCross — переход через страницу
  uint16_t ea = 0;
  bool     pageCross = false;
  DENDY_INLINE uint16_t amZp(Nes* nes);
  DENDY_INLINE uint16_t amZpX(Nes* nes);
  DENDY_INLINE uint16_t amZpY(Nes* nes);
  DENDY_INLINE uint16_t amAbs(Nes* nes);
  DENDY_INLINE uint16_t amAbsX(Nes* nes);
  DENDY_INLINE uint16_t amAbsY(Nes* nes);
  DENDY_INLINE uint16_t amIndX(Nes* nes);
  DENDY_INLINE uint16_t amIndY(Nes* nes);

  // Стек
  DENDY_INLINE void     push(Nes* nes, uint8_t value);
  DENDY_INLINE uint8_t  pull(Nes* nes);
  DENDY_INLINE void     push16(Nes* nes, uint16_t value);
  DENDY_INLINE uint16_t pull16(Nes* nes);

  // Флаги
  DENDY_INLINE void setZN(uint8_t value);
  DENDY_INLINE void setFlag(uint8_t mask, bool value);

  void doNmi(Nes* nes);
  void doIrq(Nes* nes);
};
