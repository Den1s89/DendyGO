/**
 * =============================================================================
 *  DendyGO / NesApu.cpp
 *  Реализация APU 2A03 (Pulse x2, Triangle, Noise, DMC) + микширование на AUDIO_SAMPLE_RATE.
 * =============================================================================
 */
#include <string.h>
#include <math.h>
#include "DendyFast.h"            // -O3 для горячего кода (см. DendyConfig.h)
#include "NesApu.h"
#include "NesCore.h"

#include "DendyConfig.h"          // DENDY_CPU_HZ и регион (DENDY_REGION_PAL)

// Тактовая 2A03 берётся ИЗ КОНФИГА: DENDY_CPU_HZ выведена из частоты кадров
// (DENDY_FPS_MILLI) и числа точек в кадре, а не задана отдельным числом. От неё
// зависит перевод тактов CPU в сэмплы AUDIO_SAMPLE_RATE, поэтому:
//   * в PAL-режиме музыка автоматически звучит ниже и идёт медленнее — как на
//     живом PAL-NES (иначе высота тона и темп остались бы «NTSC» и это слышно);
//   * на кадр выходит в среднем AUDIO_SAMPLE_RATE/FPS сэмплов (735 при 44 100 и
//     60 кадрах/с, 882 при 50; при 22 050 — 367,5: аккумулятор чередует
//     367/368), то есть APU и I2S идут секунда в секунду и кольцо не «плывёт».
// ВАЖНО: частота — это ПОЛЕ cpuHz (а не константа CPU_FREQ, как было раньше):
// она переключается вместе с регионом из меню (NesApu::setRegion), иначе для
// PAL-режима пришлось бы пересобирать прошивку.

// --- Самопроверка кольца сэмплов -------------------------------------------
// Индексы кольца считаются маской & (RING_SIZE-1), поэтому размер обязан быть
// степенью двойки. Остальные проверки «подушки» звука (AUDIO_PRIME_SAMPLES +
// AUDIO_DMA_FRAMES <= RING_SIZE) стоят в DendyConfig.h, блок 3 «ЗВУК».
static_assert((NesApu::RING_SIZE & (NesApu::RING_SIZE - 1)) == 0,
              "AUDIO_RING_SIZE обязано быть степенью двойки: индексы кольца "
              "маскируются как & (RING_SIZE-1)");

// Длительность звучания (length counter), индекс — значение поля NNN.
// ВАЖНО: у 2A03 таблица периодична по 16: индексы 16..31 дают те же значения,
// что 0..15. Раньше во второй половине стояли нули, и любая нота, у которой
// поле длины было >= 16 (то есть байт $4003 >= $80), получала lengthCounter = 0
// и молчала: часть нот в музыке пропадала.
static const uint8_t LENGTH_TABLE[32] = {
  10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 120, 12, 240, 14,
  10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 120, 12, 240, 14
};
// Последовательность значений треугольника (32 шага, 4 бита)
static const uint8_t TRI_SEQ[32] = {
  15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
   0,  1,  2,  3,  4,  5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
};
// Префиксные суммы TRI_SEQ: TRI_PREF[i] — сумма первых i значений, TRI_PREF[32]
// = 240 — сумма всего периода. Нужны антиалиасингу (DENDY_APU_ANTIALIAS):
// «площадь» уровня за окно сэмпла — это сумма ПОДРЯД идущих шагов
// последовательности, и её можно взять за O(1), а не крутить цикл по шагам.
// Это важно: игры умеют ставить треугольнику период 0..1 (Mega Man 2, «тишина»
// ультразвуком), и цикл на десятки шагов в каждом окне сэмпла (десятки тысяч раз
// в секунду) съел бы заметную часть кадра.
static const uint16_t TRI_PREF[33] = {
  0, 15, 29, 42, 54, 65, 75, 84, 92, 99, 105, 110, 114, 117, 119, 120,
  120, 120, 121, 123, 126, 130, 135, 141, 148, 156, 165, 175, 186, 198, 211,
  225, 240
};
// Периоды шума (NTSC)
static const uint16_t NOISE_PERIOD[16] = {
  4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};
// Периоды DMC (NTSC)
static const uint16_t DMC_RATE[16] = {
  428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 85, 72, 54
};

// ------------- Средний уровень канала за окно сэмпла (антиалиасинг) ----------
// Сэмпл = СРЕДНЕЕ значение канала по окну (все такты CPU, попавшие в окно).
// Деления на Xtensa нет — «a / b» превращается в вызов __udivsi3 (десятки тактов),
// пришлось бы 5 раз на каждый сэмпл (десятки тысяч раз в секунду). Поэтому
// длина берётся из таблицы в Q16:
//     среднее = (площадь * recip[длина] + 0.5) >> 16
// Площадь ≤ 127 * 82 = 10 414, произведение ≤ 6.9·10^8 — влезает в uint32.
// Длина окна — 40/41 такт при 44 100 Гц и 81/82 при 22 050 (CPU_FREQ/RATE),
// таблица на 128 значений с запасом: медленное деление в avgOf не включается.
// Для длин 40/41 результат ТОЧНЫЙ (проверено: уровень 15 при окне 41 даёт 15).
static uint16_t g_areaRecip[128] = {0};

DENDY_FAST_ATTR static inline uint8_t avgOf(uint32_t area, uint32_t winLen)
{
  if (winLen == 0) return 0;
  if (winLen < 128) return (uint8_t)((area * g_areaRecip[winLen] + 32768u) >> 16);
  return (uint8_t)(area / winLen);           // страховка (медленный путь без таблицы)
}

// Сумма `count` подряд идущих значений TRI_SEQ, начиная с фазы `phase`.
// Последовательность периодична (32 шага), поэтому целые периоды считаются как
// (count/32) * 240, а остаток — по префиксным суммам (с заворотом через 0).
DENDY_FAST_ATTR static inline uint32_t triSum(uint32_t phase, uint32_t count)
{
  const uint32_t rem = count & 31;
  const uint32_t ph  = (phase + (count - rem)) & 31;
  uint32_t sum = (count >> 5) * 240u;
  if (ph + rem <= 32) sum += (uint32_t)TRI_PREF[ph + rem] - TRI_PREF[ph];
  else                sum += (240u - TRI_PREF[ph]) + TRI_PREF[ph + rem - 32];
  return sum;
}

// ================================= Сброс =====================================
void NesApu::reset(Nes* nes)
{
  (void)nes;
  pulse[0] = Pulse();
  pulse[1] = Pulse();
  tri      = Triangle();
  noise    = Noise();
  dmc      = Dmc();
  frameCounterAcc = 0;
  frameStep       = 0;
  fiveStepMode    = false;
  frameIrqInhibit = false;
  sampleAcc       = 0;
  channelEnabled  = 0;
  frameIrqFlag    = false;
  dmcIrqFlag      = false;
  ringHead = ringTail = 0;
  dropped  = 0;
  memset(ring, 0, sizeof(ring));
  dcPrevQ = dcOutQ = 0;                        // выходной фильтр — в исходное состояние
  lpf1 = lpf2 = 0;                             // «аналоговый» ФНЧ — тоже
  apuPhase = 0;                                // делитель CPU->APU — с начала
  winLen   = 0;                                // окно сэмпла пустое
  // Адаптивный темп вывода — в исходное состояние (шаг 1:1, окно измерения пусто).
  // Сам флаг adaptive тут НЕ трогаем: его ставит скетч (игра — 1, меню — 0), а
  // reset() вызывается уже после этого (см. runGame).
  outPhase   = 0;
  // База шага — темп продюсера, опубликованный скетчем (setProducerRatioQ16): он
  // выживает и Reset игры (игра от этого не ускоряется/не замедляется), а скетч
  // ставит 1:1 на СТАРТЕ игры и переизмеряет за первые AUDIO_PRODUCER_WIN_FRAMES
  // кадров. Пока публикации не было, prodRatioQ16 = 65536, то есть ровно 1:1.
  outStep    = prodRatioQ16;
  prodRatioSm = prodRatioQ16;
  rateMinQ16 = rateMaxQ16 = (int32_t)prodRatioQ16;
  holdLast   = 0;
  starved    = 0;
  prodAcc    = 0;
  consAcc    = 0;
  fillPrev   = -1;
  // Суммы «глубины» окна (lowSum/highSum) обнуляем ОБЯЗАТЕЛЬНО: они копятся по
  // блокам окна, и «унаследованный» мусор дал бы поправку темпа с первого же блока.
  // Раньше здесь были крайние значения (fillMin/fillMax), причём заполнялись они
  // нулём — и первый блок окна «видел» минимум 0 < AUDIO_RATE_FLOOR, то есть
  // регулятор выдавал поправку вниз на FLOOR/64 (6,25 % при FLOOR = 64) и держал её
  // всё окно (~93 мс): музыка в начале каждой игры/после Reset уходила ниже тона и
  // потом «подтягивалась». Теперь такой поправке взяться неоткуда (см. lowSum).
  fillAvg    = 0;
  lowSum     = 0;
  highSum    = 0;
  ctrlBlocks = 0;
  buildMixTable();                             // таблицы микшера и антиалиасинга
}

// Сбросить состояние ВЫВОДА (регулятор темпа + «залипший» сэмпл), не трогая
// каналы и кольцо. Скетч зовёт это на переходах игра <-> меню
// (NesCore::resetAudioStream): после игры в кольце/регуляторе остаётся её окно
// измерения, из-за которого меню начиналось не с 1:1, а с «подтягивающегося»
// темпа, а holdLast держал уровень последнего сэмпла музыки.
// Кольцо при этом чистит NesApu::flush() (NesCore::flushAudio) — это разные вещи.
void NesApu::resetStream()
{
  outPhase   = 0;
  // Шаг — из ПОСЛЕДНЕГО опубликованного темпа продюсера (а не жёстко 1:1): после
  // перехода игра -> меню адаптивный темп выключается, а при следующем старте игры
  // скетч публикует 1:1 и переизмеряет темп за первые 16 кадров. Сглаживание EMA
  // начинаем с того же значения, чтобы не «разгонять» его от единицы.
  outStep    = prodRatioQ16;
  prodRatioSm = prodRatioQ16;
  rateMinQ16 = rateMaxQ16 = (int32_t)prodRatioQ16;
  holdLast   = 0;
  starved    = 0;
  prodAcc    = 0;
  consAcc    = 0;
  fillPrev   = -1;
  fillAvg    = 0;
  lowSum     = 0;
  highSum    = 0;
  ctrlBlocks = 0;
}

// =========================== Импульсный канал ================================
uint8_t NesApu::Pulse::out() const
{
  if (lengthCounter == 0 || timerPeriod < 8 || sweepMute) return 0;
  static const uint8_t DUTY[4] = {0x40, 0x60, 0x78, 0x9F};    // 12.5/25/50/75%
  if (!(DUTY[duty & 3] & (0x80 >> (step & 7)))) return 0;
  return constantVolume ? (uint8_t)(volume & 0x0F) : (uint8_t)(envelope & 0x0F);
}

uint8_t NesApu::Pulse::avg(uint32_t winLen) const
{
  return avgOf(area, winLen);
}

// ВНИМАНИЕ: cycles (apuTicks) — это APU-ТАКТЫ, то есть ПОЛОВИНА тактов CPU.
// На живом 2A03 таймер импульсного канала обновляется раз в APU-такт (каждые
// два такта CPU), поэтому период волны = 8 шагов × 2 такта CPU × (t+1) =
// 16·(t+1) тактов CPU, то есть f = CPU/(16·(t+1)) и максимум ~12,4 кГц
// (NESdev «APU Pulse»).
// РАНЬШЕ сюда приходили такты CPU: таймер шёл вдвое быстрее, каждая нота
// пульсов звучала НА ОКТАВУ ВЫШЕ живой приставки, а максимум уезжал к 24,8 кГц —
// выше Найквиста (22,05 кГц). Всё, что выше Найквиста, при сэмплировании
// заворачивается обратно в слышимый диапазон: на слух это ровный «песок» поверх
// музыки. В меню его не было, потому что джингл синтезируется сразу на выходной
// частоте (AUDIO_SAMPLE_RATE) и в APU вообще не заходит (serviceMenuSound).
DENDY_INLINE void NesApu::Pulse::clockTimer(uint32_t apuTicks)
{
  if (timerPeriod < 8) { timer = 1; return; }                 // канал заглушён
  const uint32_t period = (uint32_t)timerPeriod + 1;
  uint32_t left = apuTicks;
  for (;;) {
    // Уровень держится всё время до следующей смены фазы скважности. «Площадь»
    // хранится в тактах CPU (см. Pulse::area), поэтому один APU-такт даёт вклад
    // 2 × уровень × такт.
    if (left < timer) {
      area += (left << 1) * out();
      timer = (uint16_t)(timer - left);
      return;
    }
    area += ((uint32_t)timer << 1) * out();
    left -= timer;
    timer = (uint16_t)period;
    step  = (uint8_t)((step + 1) & 7);
  }
}

void NesApu::Pulse::clockEnvelope()
{
  if (envelopeDivider == 0) {
    envelopeDivider = (uint8_t)(volume & 0x0F);
    if (envelope != 0) envelope--;
    else if (lengthHalt) envelope = 15;                       // зацикливание огибающей
  } else {
    envelopeDivider--;
  }
}

void NesApu::Pulse::clockLength()
{
  if (lengthCounter > 0 && !lengthHalt) lengthCounter--;
}

void NesApu::Pulse::clockSweep(bool isPulse1)
{
  // Шаг блока sweep выполняется раз в (sweepPeriod + 1) полукадров.
  if (sweepDivider == 0) {
    if (sweepEnable && sweepShift != 0) {
      const int32_t delta  = (int32_t)(timerPeriod >> sweepShift);
      const int32_t target = sweepNegate
                           ? (int32_t)timerPeriod - delta - (isPulse1 ? 1 : 0)
                           : (int32_t)timerPeriod + delta;
      if (target >= 8 && target <= 0x7FF) timerPeriod = (uint16_t)target;
    }
    if (!sweepReload) sweepDivider = sweepPeriod;
  } else {
    sweepDivider--;
  }
  sweepReload = false;

  // Флаг «заглушения» (period > $7FF) пересчитываем на каждом полукадре.
  sweepMute = false;
  if (sweepEnable && sweepShift != 0) {
    const int32_t delta  = (int32_t)(timerPeriod >> sweepShift);
    const int32_t target = sweepNegate
                         ? (int32_t)timerPeriod - delta - (isPulse1 ? 1 : 0)
                         : (int32_t)timerPeriod + delta;
    if (target > 0x7FF) sweepMute = true;
  }
}

// ============================== Треугольник ==================================
uint8_t NesApu::Triangle::out() const
{
  if (lengthCounter == 0 || linearCounter == 0) return 0;
  return TRI_SEQ[step & 31];
}

uint8_t NesApu::Triangle::avg(uint32_t winLen) const
{
  return avgOf(area, winLen);
}

// Треугольник (в отличие от пульсов и шума) тактируется ТАКТОМ CPU — так у живого
// 2A03 (NESdev «APU Triangle»: «this timer ticks at the rate of the CPU clock
// rather than the APU (CPU/2) clock»), поэтому 32-шаговая последовательность даёт
// f = CPU/(32·(t+1)). Здесь же набирается «площадь» для антиалиасинга, причём
// СУММА шагов берётся по префиксным суммам (triSum), а не циклом: период у
// треугольника может быть 1 такт (игры глушат канал ультразвуком), и цикл на
// 40 шагов в каждом окне сэмпла был бы дорогим. Деление здесь одно (steps), как
// и в прежнем коде (там было деление с остатком, то есть два).
DENDY_INLINE void NesApu::Triangle::clockTimer(uint32_t cycles)
{
  const bool muted = (lengthCounter == 0 || linearCounter == 0);
  const uint32_t period = (uint32_t)timerPeriod + 1;
  if (cycles < timer) {
    timer = (uint16_t)(timer - cycles);
    if (!muted) area += cycles * (uint32_t)TRI_SEQ[step & 31];
    return;
  }
  const uint32_t left  = cycles - timer;
  const uint32_t steps = left / period;              // полных смен фазы
  const uint32_t rem   = left - steps * period;      // остаток на новой фазе
  if (!muted) {
    // текущая фаза держится `timer` тактов, затем `steps` шагов по `period`
    // тактов, затем остаток `rem` тактов на последней фазе
    area += (uint32_t)timer * TRI_SEQ[step & 31]
          + triSum((uint32_t)(step + 1) & 31, steps) * period
          + rem * (uint32_t)TRI_SEQ[(step + steps) & 31];
  }
  step  = (uint8_t)((step + steps) & 31);
  timer = (uint16_t)(period - rem);
}

void NesApu::Triangle::clockLinear()
{
  if (linearReload) linearCounter = linearReloadValue;
  else if (linearCounter) linearCounter--;
  if (!control) linearReload = false;
}

void NesApu::Triangle::clockLength()
{
  if (lengthCounter > 0 && !control) lengthCounter--;
}

// ================================== Шум ======================================
uint8_t NesApu::Noise::out() const
{
  if ((lfsr & 1) || lengthCounter == 0) return 0;
  return constantVolume ? (uint8_t)(volume & 0x0F) : (uint8_t)(envelope & 0x0F);
}

uint8_t NesApu::Noise::avg(uint32_t winLen) const
{
  return avgOf(area, winLen);
}

// Шум тактируется тактом CPU (таблица NOISE_PERIOD — в тактах CPU: период 4 даёт
// 447,4 кГц, как в таблице NESdev «APU Noise»). Здесь же — «площадь» для
// антиалиасинга; цикл ограничен: период ≥ 4, а окно сэмпла ≈ 40 тактов.
DENDY_INLINE void NesApu::Noise::clockTimer(uint32_t cycles)
{
  uint32_t left   = cycles;
  uint32_t period = (uint32_t)timerPeriod;
  if (period < 4) period = 4;
  for (;;) {
    if (left < timer) {
      area += left * out();
      timer = (uint16_t)(timer - left);
      return;
    }
    area += (uint32_t)timer * out();
    left -= timer;
    timer = (uint16_t)period;
    const uint16_t fb = (uint16_t)(((lfsr & 1) ^ ((lfsr >> (mode ? 6 : 1)) & 1)) & 1);
    lfsr = (uint16_t)((lfsr >> 1) | (fb << 14));
  }
}

void NesApu::Noise::clockEnvelope()
{
  if (envelopeDivider == 0) {
    envelopeDivider = (uint8_t)(volume & 0x0F);
    if (envelope != 0) envelope--;
    else if (lengthHalt) envelope = 15;
  } else {
    envelopeDivider--;
  }
}

void NesApu::Noise::clockLength()
{
  if (lengthCounter > 0 && !lengthHalt) lengthCounter--;
}

// ================================== DMC ======================================
void NesApu::Dmc::restart(Nes* nes)
{
  (void)nes;
  curAddr        = (uint16_t)(0xC000 + (uint16_t)sampleAddr * 64);
  bytesRemaining = (uint16_t)((uint16_t)sampleLength * 16 + 1);
}

uint8_t NesApu::Dmc::avg(uint32_t winLen) const
{
  return avgOf(area, winLen);
}

// ВАЖНО (была ошибка): окно сэмпла (81/82 такта CPU при 22 050 Гц, 40/41 при
// 44 100) КОРОЧЕ почти любого периода DMC (54..428 такта). Прежний цикл
// `while (cycles >= timer)` на таком окне чаще всего не срабатывал вообще: таймер
// не уменьшался на остаток окна, а «площадь» за окно оставалась нулевой, поэтому
// канал DMC фактически стоял (барабанов не слышно), а когда период всё-таки
// попадал в окно (106/85/72/54 при 22 050 Гц) биты шли не своим темпом, а по
// границам сэмплов — вместо барабана слышался рваный «песок». Теперь, как у шума
// и треугольника, уровень на неполном периоде тоже попадает в площадь, а таймер
// уменьшается на остаток окна.
DENDY_INLINE void NesApu::Dmc::clockTimer(Nes* nes, uint32_t cycles)
{
  uint32_t left = cycles;
  for (;;) {
    if (timer == 0) timer = DMC_RATE[rateIndex & 0x0F];   // период следующего бита
    if (left < timer) {                         // окно кончилось внутри периода:
      area += left * outputLevel;               // уровень держится до конца окна
      timer = (uint16_t)(timer - left);
      return;
    }
    area += (uint32_t)timer * outputLevel;      // уровень держится весь период бита
    left -= timer;
    timer = DMC_RATE[rateIndex & 0x0F];

    if (bitsRemaining == 0) {                   // начинается новая группа из 8 бит
      bitsRemaining = 8;
      if (bytesRemaining == 0) {
        silence = true;                         // данных нет — держим уровень
      } else {
        shiftReg = nesRead(nes, curAddr);
        curAddr  = (uint16_t)(0x8000 | (uint16_t)(curAddr + 1));
        if (--bytesRemaining == 0) {
          if (loop) restart(nes);
          else if (irqEnable) nes->apu.dmcIrqFlag = true;
        }
        silence = false;
      }
    }

    if (!silence) {                             // дельта-модуляция 7-битного уровня
      if (shiftReg & 1) { if (outputLevel <= 125) outputLevel += 2; }
      else              { if (outputLevel >= 2)   outputLevel -= 2; }
    }
    shiftReg >>= 1;
    bitsRemaining--;
  }
}

// ======================= Запись регистров $4000-$4017 =========================
DENDY_FAST_ATTR void NesApu::writeRegister(Nes* nes, uint16_t addr, uint8_t value)
{
  switch (addr) {
    // ---------------------------- Pulse 1 ------------------------------
    case 0x4000:
      pulse[0].duty           = (uint8_t)(value >> 6);
      pulse[0].lengthHalt     = (value & 0x20) != 0;
      pulse[0].constantVolume = (value & 0x10) != 0;
      pulse[0].volume         = (uint8_t)(value & 0x0F);
      pulse[0].envelope       = 15;
      break;
    case 0x4001:
      pulse[0].sweepEnable = (value & 0x80) != 0;
      pulse[0].sweepPeriod = (uint8_t)((value >> 4) & 0x07);
      pulse[0].sweepNegate = (value & 0x08) != 0;
      pulse[0].sweepShift  = (uint8_t)(value & 0x07);
      pulse[0].sweepReload = true;               // счётчик sweep начнётся заново
      break;
    case 0x4002:
      pulse[0].timerPeriod = (uint16_t)((pulse[0].timerPeriod & 0x700) | value);
      break;
    case 0x4003:
      pulse[0].timerPeriod = (uint16_t)((pulse[0].timerPeriod & 0x0FF) | ((value & 0x07) << 8));
      if (channelEnabled & 0x01) pulse[0].lengthCounter = LENGTH_TABLE[value >> 3];
      pulse[0].step            = 0;              // фаза скважности сбрасывается
      pulse[0].envelope        = 15;
      pulse[0].envelopeDivider = (uint8_t)(pulse[0].volume & 0x0F);
      break;
    // ---------------------------- Pulse 2 ------------------------------
    case 0x4004:
      pulse[1].duty           = (uint8_t)(value >> 6);
      pulse[1].lengthHalt     = (value & 0x20) != 0;
      pulse[1].constantVolume = (value & 0x10) != 0;
      pulse[1].volume         = (uint8_t)(value & 0x0F);
      pulse[1].envelope       = 15;
      break;
    case 0x4005:
      pulse[1].sweepEnable = (value & 0x80) != 0;
      pulse[1].sweepPeriod = (uint8_t)((value >> 4) & 0x07);
      pulse[1].sweepNegate = (value & 0x08) != 0;
      pulse[1].sweepShift  = (uint8_t)(value & 0x07);
      pulse[1].sweepReload = true;               // счётчик sweep начнётся заново
      break;
    case 0x4006:
      pulse[1].timerPeriod = (uint16_t)((pulse[1].timerPeriod & 0x700) | value);
      break;
    case 0x4007:
      pulse[1].timerPeriod = (uint16_t)((pulse[1].timerPeriod & 0x0FF) | ((value & 0x07) << 8));
      if (channelEnabled & 0x02) pulse[1].lengthCounter = LENGTH_TABLE[value >> 3];
      pulse[1].step            = 0;
      pulse[1].envelope        = 15;
      pulse[1].envelopeDivider = (uint8_t)(pulse[1].volume & 0x0F);
      break;
    // --------------------------- Треугольник ---------------------------
    case 0x4008:
      tri.control           = (value & 0x80) != 0;
      tri.linearReloadValue = (uint8_t)(value & 0x7F);
      break;
    case 0x400A:
      tri.timerPeriod = (uint16_t)((tri.timerPeriod & 0x700) | value);
      break;
    case 0x400B:
      tri.timerPeriod = (uint16_t)((tri.timerPeriod & 0x0FF) | ((value & 0x07) << 8));
      if (channelEnabled & 0x04) tri.lengthCounter = LENGTH_TABLE[value >> 3];
      tri.linearReload = true;
      break;
    // ------------------------------ Шум --------------------------------
    case 0x400C:
      noise.lengthHalt     = (value & 0x20) != 0;
      noise.constantVolume = (value & 0x10) != 0;
      noise.volume         = (uint8_t)(value & 0x0F);
      noise.envelope       = 15;
      break;
    case 0x400E:
      noise.mode        = (value & 0x80) != 0;
      noise.timerPeriod = NOISE_PERIOD[value & 0x0F];
      break;
    case 0x400F:
      if (channelEnabled & 0x08) noise.lengthCounter = LENGTH_TABLE[value >> 3];
      noise.envelope        = 15;
      noise.envelopeDivider = (uint8_t)(noise.volume & 0x0F);
      break;
    // ------------------------------ DMC --------------------------------
    case 0x4010:
      dmc.irqEnable = (value & 0x80) != 0;
      dmc.loop      = (value & 0x40) != 0;
      dmc.rateIndex = (uint8_t)(value & 0x0F);
      if (!dmc.irqEnable) dmcIrqFlag = false;
      break;
    case 0x4011:
      dmc.outputLevel = (uint8_t)(value & 0x7F);
      break;
    case 0x4012:
      dmc.sampleAddr = value;
      break;
    case 0x4013:
      dmc.sampleLength = value;
      break;
    // --------------------------- Разрешения ----------------------------
    case 0x4015:
      channelEnabled = value;
      if (!(value & 0x01)) pulse[0].lengthCounter = 0;
      if (!(value & 0x02)) pulse[1].lengthCounter = 0;
      if (!(value & 0x04)) tri.lengthCounter      = 0;
      if (!(value & 0x08)) noise.lengthCounter    = 0;
      if (value & 0x10) {                        // запуск DMC
        if (dmc.bytesRemaining == 0) dmc.restart(nes);
      } else {
        dmc.bytesRemaining = 0;
        dmc.silence        = true;
      }
      dmcIrqFlag = false;
      break;
    // -------------------------- Frame counter --------------------------
    case 0x4017:
      fiveStepMode    = (value & 0x80) != 0;
      frameIrqInhibit = (value & 0x40) != 0;
      if (frameIrqInhibit) frameIrqFlag = false;
      frameStep       = 0;
      frameCounterAcc = 0;
      if (fiveStepMode) { quarterFrame(); halfFrame(); }   // переход сразу делает пару шагов
      break;
    default:
      break;
  }
}

// ======================= Чтение статуса $4015 =================================
DENDY_FAST_ATTR uint8_t NesApu::readStatus(Nes* nes)
{
  (void)nes;
  uint8_t result = 0;
  if (pulse[0].lengthCounter) result |= 0x01;
  if (pulse[1].lengthCounter) result |= 0x02;
  if (tri.lengthCounter)      result |= 0x04;
  if (noise.lengthCounter)    result |= 0x08;
  if (dmc.bytesRemaining)     result |= 0x10;
  if (frameIrqFlag)           result |= 0x40;
  if (dmcIrqFlag)             result |= 0x80;
  frameIrqFlag = false;                        // чтение $4015 сбрасывает frame IRQ
  return result;
}

// ============================ Кадровые шаги ==================================
// ЧТО ТАКТУЕТ КАКОЙ СИГНАЛ (NESdev «APU Envelope» и «APU Frame Counter»):
//   * четвертькадр (quarter frame) — ОГИБАЮЩИЕ (pulse1/2 и шум) и linear counter
//     треугольника;
//   * полукадр (half frame)        — length counter ВСЕХ четырёх каналов и sweep
//     пульсов.
// ВАЖНО: полукадр огибающие НЕ тактует. В схеме 2A03 вход огибающей — именно
// «Quarter frame clock» (диаграмма на странице NESdev «APU Envelope»: Quarter frame
// clock -> Divider -> Decay level). В 4-шаговом режиме полукадры (шаги 2 и 4)
// совпадают с четвертькадрами, поэтому там разницы не видно, а в 5-шаговом видно:
// четвертькадры идут на шагах 1..4, полукадры — на 2 и 5, и лишний такт огибающей на
// шаге 5 сдвигал бы затухание на 7,4 мс (раньше так и было — см. step()).
void NesApu::quarterFrame()
{
  pulse[0].clockEnvelope();
  pulse[1].clockEnvelope();
  tri.clockLinear();
  noise.clockEnvelope();
}

void NesApu::halfFrame()
{
  pulse[0].clockLength();
  pulse[1].clockLength();
  tri.clockLength();
  noise.clockLength();
  pulse[0].clockSweep(true);
  pulse[1].clockSweep(false);
}

// ============================== Продвижение ==================================
// ГЛАВНОЕ ПРО ЗВУК («песок» в игре): каналы двигаются НЕ всей порцией тактов
// сразу, а дробятся по границам выходных сэмплов 44100 Гц.
//
// Почему: скетч зовёт step() «пачками» по DENDY_APU_GRANULARITY тактов (~10 000
// инструкций за кадр превращаются в ~230 вызовов). Микшер (mixSample) читает
// состояние каналов ровно в тот момент, когда сэмпл кладётся в кольцо. Если
// продвинуть каналы сразу на все 128 тактов порции и только потом набрать из
// аккумулятора ~3 сэмпла, то ВСЕ ЭТИ СЭМПЛЫ получат ОДНО И ТО ЖЕ (конечное)
// состояние каналов: звук фактически обновляется с частотой DENDY_CPU_HZ/128 ≈
// 14 кГц вместо 44,1 кГц. Всё, что в APU меняется быстрее (шум, DMC-барабаны,
// высокие ноты, арпеджио), при сэмплировании на 44,1 кГц заворачивается обратно
// в слышимый диапазон — это и слышно как «песок»/шипение поверх музыки. Высоту
// тона и темп это не портит (таймеры каналов считаются точно), поэтому симптом
// именно такой: мелодия «та же», но с песком. В меню песка нет потому, что джингл
// синтезируется сразу на выходной частоте и в APU вообще не заходит (serviceMenuSound).
//
// Теперь такты дробятся так, что каналы продвигаются РОВНО до момента снятия
// сэмпла (точность — один такт CPU, ~0,6 мкс). Число сэмплов и, значит, высота
// тона и темп НЕ меняются: аккумулятор sampleAcc тот же, просто сэмплы снимаются в
// свои моменты, а не пачкой в конце порции.
DENDY_FAST_ATTR void NesApu::step(Nes* nes, uint32_t cpuCycles)
{
  if (cpuCycles == 0) return;

  // --- Frame counter ($4017): цикл из 4 или 5 шагов ---
  // Его можно двигать всей порцией: он сам считает свои границы, а ошибка в 128
  // тактов (~70 мкс) для огибающих и IRQ неслышна (см. DENDY_APU_GRANULARITY).
  // ГРАНИЦЫ ШАГОВ — В ТАКТАХ CPU И СВОИ У КАЖДОГО РЕГИОНА. Счётчик считает такты
  // CPU, а у 2A07 (PAL) тактовая ниже, поэтому одинаковые константы давали бы в PAL
  // не 200/100 Гц (документ NESdev «APU Frame Counter»), а 223/111 — на 11 % быстрее,
  // то есть ноты в PAL-играх обрывались бы раньше написанного. Значения таблицы
  // NESdev даны в тактах APU (1 APU-такт = 2 такта CPU), а «0 (N)» — конец цикла:
  //   NTSC 2A03: 4-шаговый конец 29 830, 5-шаговый 37 282;
  //   PAL  2A07: 4-шаговый конец 33 254, 5-шаговый 41 566.
  // Выбор таблицы — пара ветвлений на порцию тактов (~230 вызовов step() за кадр),
  // в горячем цикле шагов ничего не меняется.
  static const uint32_t BOUNDS4_NTSC[4] = { 7457, 14913, 22371, 29829 };
  static const uint32_t BOUNDS5_NTSC[5] = { 7457, 14913, 22371, 29829, 37281 };
  static const uint32_t BOUNDS4_PAL[4]  = { 8313, 16627, 24939, 33253 };
  static const uint32_t BOUNDS5_PAL[5]  = { 8313, 16627, 24939, 33253, 41565 };
  const bool      pal      = (cpuHz == DENDY_PAL_CPU_HZ);
  const uint8_t   maxSteps = fiveStepMode ? 5 : 4;
  const uint32_t* bounds   = pal ? (fiveStepMode ? BOUNDS5_PAL : BOUNDS4_PAL)
                                 : (fiveStepMode ? BOUNDS5_NTSC : BOUNDS4_NTSC);
  const uint32_t  endCycle = bounds[maxSteps - 1] + 1;   // «0 (N)» в таблице NESdev

  frameCounterAcc += cpuCycles;
  for (;;) {
    if (frameStep >= maxSteps) {                       // все шаги цикла сделаны
      if (frameCounterAcc < endCycle) break;
      frameCounterAcc -= endCycle;
      frameStep = 0;
      continue;
    }
    if (frameCounterAcc < bounds[frameStep]) break;    // следующий шаг ещё не наступил
    frameStep++;
    // ЧТО КАКОЙ ШАГ ТАКТУЕТ (NESdev «APU Frame Counter»; такты — уже в тактах CPU).
    // В ОБОИХ режимах четвертькадр идёт на шагах 1..4 (огибающие + linear counter
    // треугольника), а полукадр (length counter + sweep) — на:
    //   4-шаговый ($4017 бит7 = 0): шаги 1..4, полукадры на 2 и 4 → 240 Гц и 120 Гц;
    //   5-шаговый ($4017 бит7 = 1): шаги 1..5, полукадры на 2 и 5 → 192 Гц и 96 Гц
    //                               за 37 282 такта (в PAL — 160 и 80 Гц за 41 566).
    // Отдельно про шаг 4 в 5-шаговом режиме: четвертькадр на нём ЕСТЬ (строка «4»
    // таблицы NESdev), а полукадра нет — он уходит на шаг 5.
    // РАНЬШЕ здесь стояло `case 2: case 4: halfFrame()` для ОБОИХ режимов, то есть в
    // 5-шаговом полукадр случался ТОЛЬКО на шаге 2, а шаг 5 не делал ничего. Значит,
    // length counter и sweep тактировались РОВНО ВДВОЕ реже (48 Гц вместо 96), а
    // огибающие (в прежнем коде их тактовал ещё и полукадр — теперь нет, см.
    // halfFrame) шли 144 Гц вместо 192. Поэтому любой звук, который заканчивается
    // сам — конец length counter'а (шумовые эффекты Mario) или затухание огибающей
    // (взрыв танка в Battle City) — звучал длиннее написанного и «не хотел
    // замолкать». Высота тона при этом не менялась (таймеры каналов тикают независимо
    // от frame counter), поэтому симптом был именно «длиннее, но той же высоты».
    // А 5-шаговый режим — это как раз то, чем пользуются эти игры: Battle City в своём
    // init пишет $4017 = $C0, Super Mario Bros. — $4017 = $FF (оба — бит 7 = 1), и
    // больше $4017 в игре не трогают, то есть последовательность честно доходит до
    // 5-го шага (см. разбор в README, «5-шаговый frame counter»).
    // Четвертькадр — на шагах 1..4 (огибающие + linear counter), полукадр (length
    // counter + sweep) — на шагах 2 и 4 в 4-шаговом режиме и на 2 и 5 в 5-шаговом.
    if (frameStep <= 4) quarterFrame();
    if (frameStep == 2 || frameStep == (fiveStepMode ? 5 : 4)) halfFrame();
    if (!fiveStepMode && frameStep == 4 && !frameIrqInhibit) frameIrqFlag = true;
  }

  // --- Каналы + генерация сэмплов 44100 Гц: дробим такты по границам сэмплов ---
  // Идём от границы сэмпла к границе: до следующего сэмпла остаётся `until` тактов
  // (ceil, но не больше остатка порции), на них и продвигаем все каналы, затем при
  // наступлении границы снимаем сэмпл. sampleAcc всегда меньше CPU_FREQ (после
  // каждого вычитания), поэтому until >= 1 и цикл конечен.
  //
  // ВАЖНО про единицы измерения тактов (см. NESdev «APU Pulse/Triangle/Noise»):
  //   * пульсы тактируются APU-тактом = 2 такта CPU. Период волны = 16·(t+1)
  //     тактов CPU (f = CPU/(16·(t+1)), максимум ~12,4 кГц). Раньше им отдавали
  //     такты CPU — пульсы звучали на октаву выше и лезли выше Найквиста, что и
  //     давало «песок» в игре;
  //   * треугольник и шум тактируются тактом CPU (период шума из NOISE_PERIOD —
  //     в тактах CPU, период 4 = 447,4 кГц);
  //   * DMC — тоже такт CPU (DMC_RATE: 428 = 4,18 кГц).
  // Остаток нечётного такта CPU при переводе в APU-такты переносим в следующий
  // вызов (apuPhase): иначе на коротких порциях терялся бы каждый второй такт и
  // высота тона пульсов «плыла» бы вниз.
  uint32_t left = cpuCycles;
  while (left) {
    uint32_t until = ((uint32_t)cpuHz - sampleAcc + (AUDIO_SAMPLE_RATE - 1))
                     / AUDIO_SAMPLE_RATE;              // тактов до следующего сэмпла
    if (until == 0) until = 1;                         // страховка (на всякий случай)
    if (until > left) until = left;                    // не выходим за порцию

    const uint32_t apuTicks = (until + apuPhase) >> 1; // 1 APU-такт = 2 такта CPU
    apuPhase = (uint8_t)((until + apuPhase) & 1);

    pulse[0].clockTimer(apuTicks);                     // APU-такты
    pulse[1].clockTimer(apuTicks);                     // APU-такты
    tri.clockTimer(until);                             // такты CPU
    noise.clockTimer(until);                           // такты CPU
    dmc.clockTimer(nes, until);                        // барабаны: уровень к сэмплу

    winLen += until;                                   // тактов CPU в этом окне
    sampleAcc += until * AUDIO_SAMPLE_RATE;
    if (sampleAcc >= (uint32_t)cpuHz) {
      sampleAcc -= (uint32_t)cpuHz;
      pushSample(mixSample(winLen));                   // средние уровни за окно
      resetAreas();                                    // началось новое окно
    }
    left -= until;
  }
}

// Начать новое окно сэмпла: «площади» каналов и длина окна — с нуля.
void NesApu::resetAreas()
{
  pulse[0].area = 0;
  pulse[1].area = 0;
  tri.area      = 0;
  noise.area    = 0;
  dmc.area      = 0;
  winLen        = 0;
}

// ============================ Кольцевой буфер ================================
void NesApu::pushSample(int16_t s)
{
  const uint16_t next = (uint16_t)((ringHead + 1) & (RING_SIZE - 1));
  if (next == ringTail) {                       // буфер полон — сэмпл теряется
    ++dropped;                                  // счётчик для [SND] `drob`:
    return;                                     // ненулевое значение = в музыке дырка
  }
  ring[ringHead] = s;
  ringHead = next;
}

int NesApu::drain(int16_t* dst, int maxSamples)
{
  int n = 0;
  while (n < maxSamples && (uint16_t)ringTail != (uint16_t)ringHead) {
    dst[n++] = ring[ringTail];
    ringTail = (uint16_t)((ringTail + 1) & (RING_SIZE - 1));
  }
  return n;
}

// ------------------- Адаптивный темп вывода (рессемплер) ---------------------
// Задача I2S просит РОВНО AUDIO_DMA_FRAMES сэмплов (ровный темп для DMA), а мы
// отдаём их, растягивая то, что лежит в кольце. Зачем — в DendyConfig.h (блок 3,
// AUDIO_ADAPTIVE_RATE): при медленном эмуляторе (fps emu < 60) кольцо скудеет, и
// вместо вставки тишины в середину музыки («песок») звук растягивается во столько
// раз, во сколько эмулятор медленнее приставки (при 49,4 кадра/с — в 1,215, то
// есть шаг чтения 0,823 сэмпла кольца на один выходной сэмпл).
//
// Как читаем: outPhase — позиция чтения в Q16 (целая часть — номер сэмпла от
// ringTail, дробная — вес линейной интерполяции между соседними сэмплами). За
// каждый выходной сэмпл позиция растёт на outStep: 65536 — ровно 1:1 (сэмплы
// уходят в кодек бит в бит, как в drain()), меньше — растяжение (музыка медленнее
// и ниже, но СИНХРОННО с тем, что на экране), больше — сжатие (кольцо слишком
// полное). Шаг регулятор не «угадывает», а ИЗМЕРЯЕТ по продюсеру (см. ниже, в
// конце функции): сколько сэмплов APU реально дал на один выходной — таков и шаг.
// При живом эмуляторе (60,0 кадров/с) это ровно 65536, и сэмплы уходят бит в бит.
int NesApu::drainResampled(int16_t* dst, int maxOut)
{
#if AUDIO_ADAPTIVE_RATE
  if (maxOut <= 0) return 0;
  if (!adaptive) return drain(dst, maxOut);    // меню: кольцо держит сам джингл

  int consumed = 0;                            // сколько сэмплов кольца мы забрали
  int n = 0;
  for (; n < maxOut; ++n) {
    const uint16_t head  = ringHead, tail = ringTail;      // снимок индексов
    const int      avail = (int)((uint16_t)((head - tail) & (RING_SIZE - 1)));
    const uint32_t idx   = outPhase >> 16;                 // целая часть позиции
    const uint32_t frac  = outPhase & 0xFFFF;              // дробная — вес смеси
    if (avail > (int)idx + 1) {                            // есть между чем смешать
      const int16_t a = ring[(uint16_t)(tail + (uint16_t)idx) & (RING_SIZE - 1)];
      const int16_t b = ring[(uint16_t)(tail + (uint16_t)idx + 1) & (RING_SIZE - 1)];
      holdLast = (int16_t)((int32_t)a +
                           ((((int32_t)b - (int32_t)a) * (int32_t)frac) >> 16));
    } else if (avail > (int)idx) {                         // последний — как есть
      holdLast = ring[(uint16_t)(tail + (uint16_t)idx) & (RING_SIZE - 1)];
    } else {
      ++starved;                                           // пусто — держим уровень
    }
    dst[n] = holdLast;

    outPhase += outStep;
    uint32_t adv = outPhase >> 16;                         // сколько сэмплов съели
    // ЗАЩИТА ОТ «УБЕЖАВШЕЙ» ЧИТАЮЩЕЙ ГОЛОВКИ. Раньше ringTail сдвигался на adv
    // ВСЕГДА, даже когда в кольце не было ни одного сэмпла (adv=1 при шаге 1:1 или
    // больше). Тогда tail уходил ВПЕРЁД head, и fill() начинал показывать почти
    // ПОЛНОЕ кольцо (разность по маске) — регулятор это видел как «кольцо под
    // завязку» и начинал СЖИМАТЬ звук, то есть лекарство было хуже болезни:
    // опустевшее кольцо сжималось ещё сильнее и в музыке появлялись дырки.
    // Теперь за голову не читаем: съедаем столько, сколько есть, а «лишнюю»
    // целую часть фазы отбрасываем (дробную сохраняем — по ней идёт интерполяция).
    if (adv > (uint32_t)avail) {
      adv = (uint32_t)avail;
      outPhase &= 0xFFFF;
    }
    if (adv) {
      ringTail = (uint16_t)((ringTail + (uint16_t)adv) & (RING_SIZE - 1));
      outPhase &= 0xFFFF;
      consumed += (int)adv;
    }
  }

  // --- Регулятор темпа: измеряем, сколько сэмплов за блок реально дал APU -----
  // За блок (maxOut выходных сэмплов) APU положил в кольцо
  //     prod = (заполнение сейчас) - (заполнение на прошлом блоке) + забрано нами,
  // то есть сколько сэмплов НАДО было забрать, чтобы кольцо осталось на месте.
  // Отношение prod/maxOut и есть нужный коэффициент: скорость эмулятора узнаём
  // ПРЯМО (без «угадывания» регулятором): при 49,4 кадрах/с выходит 0,823, при
  // 60,0 — ровно 1,000. Окно AUDIO_RATE_WIN блоков (~93 мс при 128 сэмплах) —
  // чтобы дробные 367/368 сэмпла на кадр не дёргали тон.
  const int fillNow = (int)((uint16_t)((ringHead - ringTail) & (RING_SIZE - 1)));
  if (fillPrev < 0) fillPrev = fillNow;                    // первый блок: базы нет
  if (fillAvg <= 0) fillAvg = fillNow;                     // страховка (после reset)
  fillAvg += (fillNow - fillAvg) >> 3;                     // сглаживание для трима
  // «Кольцо упёрлось»: копим ГЛУБИНУ по блокам окна (см. lowSum/highSum в NesApu.h).
  if (fillNow < AUDIO_RATE_FLOOR) {
    lowSum += AUDIO_RATE_FLOOR - fillNow;
  }
  if (fillNow > RING_SIZE - AUDIO_DMA_FRAMES) {
    highSum += fillNow - (RING_SIZE - AUDIO_DMA_FRAMES);
  }
  prodAcc += (fillNow - fillPrev) + consumed;
  consAcc += n;
  fillPrev = fillNow;
  if (++ctrlBlocks >= AUDIO_RATE_WIN) {
    // --- БАЗА ШАГА: темп продюсера от скетча (setProducerRatioQ16) ------------
    // Скетч измеряет ФАКТИЧЕСКИЙ ПЕРИОД КАДРА по micros() и публикует отношение
    // «номинал/факт» — это непрерывная величина, поэтому она не «дышит» от того,
    // что APU отдаёт сэмплы неравномерно по кадру (работает по ходу кадра и молчит,
    // пока цикл ждёт границу), в отличие от прежней оценки
    // «сколько сэмплов пришло в кольцо за окно» (она гуляла на ±9 %, и тон от
    // этого расплывался — разбор в DendyConfig.h, AUDIO_PRODUCER_WIN_FRAMES).
    // Сглаживаем EMA: публикация приходит раз в AUDIO_PRODUCER_WIN_FRAMES кадров
    // (~0,27 с), всплески нагрузки сглаживаются за пару публикаций.
    // Прежняя оценка (prodAcc/consAcc) осталась ЗАПАСНЫМ вариантом — пока скетч
    // ничего не публиковал (первые кадры игры).
    int32_t step;
    if (prodRatioOk) {
      prodRatioSm += ((int32_t)prodRatioQ16 - (int32_t)prodRatioSm)
                     >> AUDIO_PRODUCER_EMA_SHIFT;
      step = (int32_t)prodRatioSm;
    } else {
      step = consAcc
           ? (int32_t)(((uint64_t)(prodAcc > 0 ? prodAcc : 0) << 16)
                       / (uint32_t)consAcc)
           : 65536;
    }
    // Слабая «подтяжка» к середине рабочей зоны кольца: сама по себе оценка выше
    // держит заполнение ГДЕ УГОДНО (коэффициент верный, но точки равновесия нет),
    // а из-за квантования шага в Q16 кольцо медленно сползает к дну или к верху
    // (~0,3 сэмпла в секунду). Трим в 1/64 на сэмпл отклонения — это доли
    // промилле по темпу (неслышно), но сползание он убирает.
    step += (fillAvg - AUDIO_RATE_MID) >> 6;
    // Страховка «кольцо упёрлось»: прямая оценка в этих случаях слепа — когда
    // кольцо пусто, задача I2S «съедает» ровно то, что успел дать APU, и оценка
    // даёт 1:1, хотя эмулятор не успевает. Кольцо лежало на дне — растянуть
    // сильнее, стояло под завязку — сжать.
    // Берём СРЕДНЮЮ глубину за окно (lowSum/highSum делим на число блоков окна):
    // одиночный провал кольца (всплеск: загрузка рома, опрос кнопок, пачка кадров)
    // даёт доли процента и на слух не заметен, а настоящее «кольцо лежит на дне»
    // (эмулятор не успевает) — полную поправку. Подробнее — в NesApu.h (lowSum).
    if (lowSum) {
      const int32_t deep = lowSum / AUDIO_RATE_WIN;
      step -= ((step >> 4) * deep) >> 6;              // FLOOR = 64 = 1 << 6
    }
    if (highSum) {
      const int32_t over = highSum / AUDIO_RATE_WIN;
      // Кольцо ПОД ЗАВЯЗКУ: продюсеру некуда класть сэмплы, значит он НЕ медленнее
      // нас — шаг меньше 1:1 здесь заведомо наша ошибка, а не медленный эмулятор.
      // Прямая оценка этого не видит (она считает только то, что ВЛЕЗЛО в кольцо),
      // поэтому раньше регулятор мог «залипнуть» на заниженном шаге: кольцо полное,
      // расход 240 сэмплов на блок, продюсер ровно столько же кладёт (остальное
      // теряется — `drob`), и оценка снова показывает 0,94 — музыка тихо «плывёт»
      // вниз, пока что-нибудь не опустошит кольцо. Теперь при устойчивом «верхе»
      // (over > 8 — то есть кольцо стояло у верха не один блок, а весь участок)
      // шаг поднимается минимум до 1:1, и дальше мягко сжимается, чтобы уровень
      // вернулся к середине рабочей зоны (AUDIO_RATE_MID).
      if (over > 8 && step < 65536) step = 65536;
      step += ((65536 >> 4) * over) >> 7;             // до +6 % у самого верха
    }
    if (step < AUDIO_RATE_MIN_Q16) step = AUDIO_RATE_MIN_Q16;
    if (step > AUDIO_RATE_MAX_Q16) step = AUDIO_RATE_MAX_Q16;
    // --- Сглаживание МЕЛКОЙ подстройки (AUDIO_RATE_SLEW_Q16) ------------------
    // Большие отклонения (эмулятор реально медленнее приставки) ставятся сразу:
    // иначе кольцо успело бы опустеть и в музыке появились бы дырки. То же и в
    // «спасательных» случаях (кольцо упиралось вниз/вверх — lowSum/highSum):
    // там важна скорость, а не гладкость, потому что после всплеска кольцо надо
    // быстро вернуть в рабочую зону. Сглаживается только ровная работа.
    const int32_t dev = step - 65536;
    if (!lowSum && !highSum && dev > -3277 && dev < 3277) {
      const int32_t lim = (int32_t)AUDIO_RATE_SLEW_Q16;
      if (step > (int32_t)outStep + lim)      step = (int32_t)outStep + lim;
      else if (step < (int32_t)outStep - lim) step = (int32_t)outStep - lim;
    }
    // Мёртвая зона: если регулятор сошёлся к 1:1 (эмулятор выдаёт ровно 60,000
    // или 50,000 кадра/с), ставим РОВНО 1:1 — тогда сэмплы уходят в кодек бит в
    // бит, без линейной интерполяции, которая при дробном шаге «мылит» фронты.
    if (step >= 65536 - (int32_t)AUDIO_RATE_DEADBAND_Q16 &&
        step <= 65536 + (int32_t)AUDIO_RATE_DEADBAND_Q16) step = 65536;
    outStep  = (uint32_t)step;
    // Диапазон шага за время между вызовами (см. takeRateRange, поле `temp` в
    // [SND]): обновляем ЗДЕСЬ, то есть раз в окно измерения, а не каждый блок.
    if ((int32_t)outStep < rateMinQ16) rateMinQ16 = (int32_t)outStep;
    if ((int32_t)outStep > rateMaxQ16) rateMaxQ16 = (int32_t)outStep;
    prodAcc  = 0; consAcc = 0; ctrlBlocks = 0;
    lowSum   = 0; highSum = 0;                    // новое окно — новая «глубина»
  }
  return n;
#else
  return drain(dst, maxOut);                   // адаптивный темп выключен в конфиге
#endif
}

// Текущий темп вывода в промилле (1000 = 1:1) — для строки [SND] поле `temp`:
// сразу видно, во сколько раз растянут звук (823 = эмулятор идёт 49,4 кадра/с).
int NesApu::ratePerMille() const
{
  return (int)((outStep * 1000u) >> 16);
}

// Темп продюсера от скетча (разбор — NesApu.h и DendyConfig.h,
// AUDIO_PRODUCER_WIN_FRAMES). Q16: 65536 = «эмулятор идёт ровно 60,000 (50,000)
// кадров/с». Число становится базой шага чтения кольца в drainResampled
// (со сглаживанием EMA), а оценка по кольцу — только страховкой.
void NesApu::setProducerRatioQ16(uint32_t q16)
{
  prodRatioQ16 = q16;
  if (!prodRatioOk) {                  // первая публикация: база сразу верная,
    prodRatioSm = q16;                 // без «разгона» EMA от 1:1
    prodRatioOk = true;
  }
}

// Диапазон шага чтения (в промилле) за время с последнего вызова. Смысл — увидеть
// в строке [SND], СТОИТ ли тон ровно: `temp 992 [za okno 992..992]` — стоит (как
// и должно быть после правки базы шага), `temp 1000 [... 995..1000]` — «дышит»
// (так было, когда база считалась по кольцу и гуляла на ±9 % за окно).
int NesApu::takeRateRange(int* minPermille, int* maxPermille)
{
  if (minPermille) *minPermille = (int)((rateMinQ16 * 1000) >> 16);
  if (maxPermille) *maxPermille = (int)((rateMaxQ16 * 1000) >> 16);
  rateMinQ16 = (int32_t)outStep;       // новое окно наблюдения
  rateMaxQ16 = (int32_t)outStep;
  return ratePerMille();
}

// ---------------- Кольцо для внешнего источника (звук меню) ------------------
// pushSample()/drain() принадлежат APU и задаче I2S. Эти функции нужны скетчу:
// в меню эмулятор не крутится, APU молчит, и чтобы проверить звуковой тракт,
// меню кладёт в то же кольцо свой «8-битный» сигнал.
int NesApu::freeSpace() const
{
  const uint16_t head = ringHead, tail = ringTail;
  return (int)((uint16_t)((tail - head - 1) & (RING_SIZE - 1)));
}

int NesApu::fill() const
{
  const uint16_t head = ringHead, tail = ringTail;
  return (int)((uint16_t)((head - tail) & (RING_SIZE - 1)));
}

void NesApu::flush()
{
  ringTail = ringHead;                      // всё, что не успели отдать, — в утиль
}

int NesApu::pushExternal(const int16_t* src, int maxSamples)
{
  int n = 0;
  while (n < maxSamples) {
    const uint16_t next = (uint16_t)((ringHead + 1) & (RING_SIZE - 1));
    if (next == ringTail) break;             // буфер полон — лишнее не влезает
    ring[ringHead] = src[n++];
    ringHead = next;
  }
  return n;
}

// ------------------- «Подушка» звука игры (предзаполнение) --------------------
// Перед стартом рома скетч кладёт в кольцо AUDIO_PRIME_SAMPLES сэмплов тишины
// (NesCore::primeAudio). Смысл: задача I2S не должна подходить к ПУСТОМУ кольцу —
// иначе в DMA уходит вставка тишины, а это дырка в середине музыки (именно так и
// звучал «хрип» в игре, см. разбор в DendyConfig.h, блок 3 «ЗВУК»). С «подушкой»
// в один кадр (~17 мс) уровень в кольце колеблется вокруг ~700 сэмплов, поэтому
// любое дрожание продюсера (кадр пришёл позже, опрос кнопок по I2C, пачка кадров
// при DENDY_MAX_FRAME_SKIP) гасится запасом, а не вставкой нулей.
int NesApu::pushSilence(int maxSamples)
{
  int n = 0;
  while (n < maxSamples) {
    const uint16_t next = (uint16_t)((ringHead + 1) & (RING_SIZE - 1));
    if (next == ringTail) break;             // кольцо полно — остальное не влезает
    ring[ringHead] = 0;
    ringHead = next;
    ++n;
  }
  return n;
}

// ============================== Микширование =================================
void NesApu::setVolume(uint8_t volume)
{
  // Громкость в Q16: раньше было float gain = volume * (1.1/255).
  gainQ = ((uint32_t)volume * 72090u + 127u) / 255u;   // 1.1 * 65536 = 72090
}

// Таблица нелинейной формулы пульсов: pulse = 95.88 / (8128/pulses + 100).
// Значений всего 31 (сумма двух 4-битных каналов), а раньше это деление
// считалось на КАЖДЫЙ сэмпл — 44 100 раз в секунду.
void NesApu::buildMixTable()
{
  pulseQ12[0] = 0;                                        // каналы молчат
  for (int p = 1; p <= 30; ++p) {
    const float v = 95.88f / (8128.0f / (float)p + 100.0f);   // 0..0.26
    pulseQ12[p] = (uint16_t)(v * 4096.0f + 0.5f);             // в Q12
  }

  // Таблица 1/длина окна (Q16) для антиалиасинга: среднее = (площадь*recip+0.5)>>16.
  // Длины окна — 40/41 такт CPU при 44 100 Гц и 81/82 при 22 050; таблица на 128
  // значений, дальше работает обычное деление (avgOf). Считается один раз здесь,
  // чтобы в горячем пути (5 каналов на каждый сэмпл) не было делений.
  // ВНИМАНИЕ: (65536 + i/2)/i при i = 1 даёт ровно 65536 и в uint16_t не влезает
  // (получился бы 0 — «среднее» обнулялось бы на окне длиной 1 такт CPU).
  // Для i = 1 берём максимум шкалы: ошибка 1,5·10^-5, а для площадей ≤ 32767
  // (это наш случай: уровень ≤ 127, окно ≤ 82) результат всё равно точный.
  for (uint32_t i = 1; i < 128; ++i) {
    const uint32_t r = (65536u + i / 2) / i;
    g_areaRecip[i] = (uint16_t)((r > 65535u) ? 65535u : r);
  }

  // Коэффициент «аналогового» ФНЧ выхода (DENDY_APU_LOWPASS_HZ) в Q12:
  // k = 1 - exp(-2*pi*fc/fs). 0 = фильтр выключен (см. DendyConfig.h, блок 3).
#if DENDY_APU_LOWPASS_HZ > 0
  const float kf = 1.0f - expf(-6.2831853f * (float)DENDY_APU_LOWPASS_HZ
                                       / (float)AUDIO_SAMPLE_RATE);
  lpfCoef = (uint16_t)(kf * 4096.0f + 0.5f);
#else
  lpfCoef = 0;
#endif

  mixTblReady = true;
}

DENDY_FAST_ATTR int16_t NesApu::mixSample(uint32_t winLen)
{
  if (!mixTblReady) buildMixTable();           // страховка: таблица нужна всегда

#if DENDY_APU_ANTIALIAS
  // Уровни каналов — СРЕДНИЕ по окну сэмпла (см. DENDY_APU_ANTIALIAS в
  // DendyConfig.h): мгновенное значение округляло бы фронты до сетки сэмплов,
  // что и слышно как «песок». Высота тона и темп от этого не меняются.
  const uint8_t p1 = pulse[0].avg(winLen);
  const uint8_t p2 = pulse[1].avg(winLen);
  const uint8_t tl = tri.avg(winLen);
  const uint8_t nz = noise.avg(winLen);
  const uint8_t dm = dmc.avg(winLen);
#else
  const uint8_t p1 = pulse[0].out();
  const uint8_t p2 = pulse[1].out();
  const uint8_t tl = tri.out();
  const uint8_t nz = noise.out();
  const uint8_t dm = dmc.outputLevel;
#endif

  // Нелинейная формула NES (пульсы + TND) целиком в целых числах, масштаб Q12:
  //   pulse = 95.88 / (8128/pulses + 100)                         -> таблица
  //   tnd   = 159.79 / (1/(tl/8227 + nz/12241 + dm/22638) + 100)  -> ниже
  // Раньше здесь было до четырёх делений float на каждый сэмпл (44 100/с).
  int32_t xQ = (int32_t)pulseQ12[(uint32_t)(p1 + p2)];

  if (tl | nz | dm) {
    // S = tl/8227 + nz/12241 + dm/22638 в масштабе 2^24 (деления на константы
    // компилятор заменяет умножениями — настоящих делений тут нет).
    const uint32_t s24 = ((uint32_t)tl << 24) / 8227u
                       + ((uint32_t)nz << 24) / 12241u
                       + ((uint32_t)dm << 24) / 22638u;
    // u = S/(1+100*S) в Q12 (одно 32-битное деление), затем tnd = 159.79*u/4096
    // — сразу получается тот же Q12, что у пульсовой таблицы.
    const uint32_t u = (s24 << 12) / (16777216u + 100u * s24);
    xQ += (int32_t)((15979u * u + 50u) / 100u);
  }

  // Убираем постоянную составляющую (однополюсный ФВЧ ~35 Гц при 44 100, ~18 при 22 050):
  // dcOut = (x - dcPrev) + 0.995*dcOut. В Q12: 0.995 * 4096 = 4076.
  dcOutQ  = (xQ - dcPrevQ) + ((dcOutQ * 4076) >> 12);
  dcPrevQ = xQ;

  // Громкость и ограничение: раньше было float dcOut * gain * 32767.0f.
  // (dcOutQ/4096) * (gainQ/65536) * 32767 = (dcOutQ*gainQ) >> 13 (32767/2^32).
  // Ограничение состояния фильтра — защита от переполнения int32: всё, что выше
  // 4 полных шкал (16384 в Q12), всё равно обрезается клиппингом ниже.
  if (dcOutQ >  16384) dcOutQ =  16384;
  if (dcOutQ < -16384) dcOutQ = -16384;

  // «Аналоговый» ФНЧ выхода (DENDY_APU_LOWPASS_HZ > 0): два каскада однополюсного
  // ФНЧ по 12 дБ/окт — модель фильтра в тракте приставки. Всё в целых (Q12),
  // как и остальной горячий код: произведение ≤ 32768*4096 = 1.3·10^8, влезает
  // в int32. lpfCoef = 0 — фильтр выключен, и работа не тратится.
  int32_t outQ = dcOutQ;
  if (lpfCoef) {
    lpf1 += ((outQ - lpf1) * lpfCoef) >> 12;
    lpf2 += ((lpf1 - lpf2) * lpfCoef) >> 12;
    outQ  = lpf2;
  }

  int32_t v = (outQ * (int32_t)gainQ) >> 13;
  if (v >  32767) v =  32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}
